"""Clean a real desktop CPU's old thermal paste off its heat spreader with a 91% IPA lint-free wipe.

A PC-repair bench: an ATX motherboard on brass standoffs with an AM4-class CPU latched in its socket, its IHS up and a
30 x 20 mm grey patch of old paste on it. The cooler has been lifted off. On a foil tray at the near left lies a folded
91% IPA lint-free wipe (50 x 50 mm, 24 x 24 mm contact face); a waste tray waits at the near right. The instruction
states only the goal.

The paste is a 2 mm-cell coverage layer on the IHS. A cell is removed only when the wipe's soft face is really in
contact with the IHS (the live MuJoCo pad-CPU normal force is at least 0.12 N, the face is flat within 20 degrees) and
is rubbing across it (the pad's planar motion since the last control step). A wipe held still on the lid, a wipe
hovering 1 mm above it, or a wipe dragged off the lid removes nothing. The wipe starts wet (1.0) and dries slowly; a
dry wipe removes nothing and leaves lint. The pad's load is tracked; a loaded pad redeposits paste on clean cells. The
socket frame, the load plate and the lever are separate solids: any pad contact with the board (or the CPU outside the
IHS) is paste on the socket/pins, and any gripper contact with the CPU is a scratch. The final 1.0 needs every cell
clean, the IPA film dry, no lint, no paste on the pins/socket, no scratch, a clean wipe (below its load limit), and 1 s
with the pad and both grippers clear of the CPU.

Run it:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/thermal-paste-removal/thermal_paste_removal.py --steps 1000 --trace

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with the
physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations.
"""

TAG = "thermal_paste"
import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself times the 1 s full-score hold
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) -----------------------------
CPU_X, CPU_Y = 0.360, -0.070
MAT_TOP = TOP + 0.0030
BOARD_TOP = MAT_TOP + 0.0065 + 0.0016
IHS_TOP = BOARD_TOP + 0.00735  # AMD AM4: 7.35 mm from the PCB top to the IHS top
PAD_W = 0.050  # the folded 50 x 50 mm wipe
PAD_T = 0.0020  # 2 mm thick at the flange
FACE_W = 0.024  # the 24 x 24 mm soft contact face
PAD_TCP_ABOVE = 0.0250  # the TCP is this far above the contact face (the fingers sit on the 20 mm grip bar, clear of the backing)
PRESS_EXTRA_MM = 1.0  # Isaac: the arm holds the wipe face ~0.9 mm high at the MuJoCo press z; aim this much lower
#                       so the face really touches the IHS (the 0.12 N contact rule is unchanged)
NUDGE_X = float(os.environ.get("TP_NUDGE_X", "0")) * MM
NUDGE_Y = float(os.environ.get("TP_NUDGE_Y", "0")) * MM
NUDGE_YAW = float(os.environ.get("TP_NUDGE_YAW", "0"))
PAD_START = (0.240 + NUDGE_X, -0.240 + NUDGE_Y, TOP + 0.0060)
PAD_ROT = (0.0, 0.0, math.sin(math.radians(NUDGE_YAW) / 2), math.cos(math.radians(NUDGE_YAW) / 2))
PAD_GRIP = -0.00925  # 18.5 mm opening: a 20 mm grip bar squeezed 0.75 mm a finger
PAD_PICK_OPEN = -0.0125  # 25 mm: clear of the 20 mm grip bar
WET_START, WET_DRY = 1.0, 0.10  # the wipe starts wet; below 0.10 it is a dry wipe
WET_LIFE = 1.5  # s: the IPA film on the IHS at 91% flashes off in seconds (declared)
PAD_WET_LIFE = 240.0  # s: a folded wipe in the open stays damp for minutes (declared)
WET_USE = 0.0006  # wetness lost per 0.05 s control step in contact
PAD_CAP = 0.120  # g: paste a 50 x 50 mm folded wipe can hold before it redeposits
REDEPOSIT = 0.30  # paste fraction a saturated wipe leaves on a clean cell
PASTE_G = 0.080  # g, the 30 x 20 x 0.05 mm patch at about 2.7 g/cm3
CELL_PASTE = PASTE_G / 122.0  # g per cell (the paste asset prints the count; 122 cells)
FORCE_MIN = 0.12  # N: the soft face is really on the IHS
FORCE_HARD = 6.0  # N: a wet wipe pressed this hard can scratch with the paste's grit
FORCE_DRY = 2.0  # N: a dry wipe pressed this hard scratches
SCRATCH_TILT = 25.0  # deg: a wipe leaning beyond this on the lid can scratch with an edge
RUB_MIN = 0.0002  # m: a wipe moved less than this across the lid is held still, not rubbed
K_REMOVE = 0.060  # fraction removed per mm of rubbing, per sqrt(N)
CLEAN = 0.02  # a cell is clean when less than 2 % of its paste remains
MOST, ALMOST = 0.60, 0.95  # score ladder: most paste removed, then 95 % of the cells clean
CLEAR = 0.060  # m: a pad or gripper must be this far from the CPU for the 1.0 hold
STILL_MOVE, STILL_TURN, STILL_S = 0.002, math.radians(2.0), 1.0

# -- the objects ------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: a free rigid body, an articulation
    (``articulation``: its root is free) or a fixture (``kinematic``)."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False
    no_gravity: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        # No _spawn_fixture: it calls spawn_from_usd.__wrapped__, which skips Arena's
        # activate_contact_sensors pass, so a kinematic CPU/board could not be a PhysX contact filter.
        # These USDs author no CCD, the flag _spawn_fixture was there to clear.
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        # No fixed-root joint: on Isaac 6.1 that joint made PhysX report "Nested articulation roots are not
        # allowed" and drop the paste's articulation root, so the prim was never found.  The paste keeps a
        # free root (gravity is authored off in the USD) and extra_states pins it to the fixed CPU each step.
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )


def _add_contact_sensors(env_cfg) -> None:
    """PhysX contact reporters for the pairs the scorer reads, through Isaac Lab's ``ContactSensor``.

    ``wipe_cpu_contact`` (on the wipe, filtered to the CPU and the board) gives the pad-CPU and pad-board
    forces and the contact point; ``cpu_arm_contact`` and ``board_arm_contact`` (on the fixed CPU and board,
    filtered to each arm's body regex) give the robot-CPU and robot-board forces.  Every one is the PhysX
    ``RigidContactView`` contact force (``normal_force_matrix_w``), not a proximity or a timer.  The
    MuJoCo harness has no ``env_cfg.scene`` and keeps its own solver reader."""
    scene_cfg = getattr(env_cfg, "scene", None)
    if scene_cfg is None:
        return
    global _ISAAC
    _ISAAC = True
    if "wipe" not in _BUILT:
        return
    wipe, cpu, board = (_BUILT.get(n) for n in ("wipe", "cpu", "board"))
    if wipe is None or cpu is None or board is None:
        return
    from isaaclab.sensors import ContactSensorCfg

    def sensor(obj, filters, points=False):
        cfg = ContactSensorCfg(prim_path=obj.get_contact_sensor_prim_path(),
                               filter_prim_paths_expr=[f.get_contact_sensor_prim_path() for f in filters])
        cfg.track_contact_points = points
        cfg.history_length = 0
        cfg.max_contact_data_count_per_prim = 16
        return cfg

    scene_cfg.wipe_cpu_contact = sensor(wipe, (cpu, board), points=True)
    # PhysX filters require one body per expression; use the distal links that can reach the socket.
    from yam_arena.robot import asset_link_paths

    links = [asset_link_paths()[n] for n in ("link4", "link5", "gripper", "tip_left", "tip_right")]
    arm_filters = [f"{{ENV_REGEX_NS}}/{arm}{rel}" for arm in ("LeftArm", "RightArm") for rel in links]
    for key, body in (("cpu_arm_contact", cpu), ("board_arm_contact", board)):
        cfg = ContactSensorCfg(prim_path=body.get_contact_sensor_prim_path(),
                               filter_prim_paths_expr=arm_filters)
        cfg.history_length = 0
        cfg.max_contact_data_count_per_prim = 32
        setattr(scene_cfg, key, cfg)


def _torch(array, device):
    """A float32 torch tensor for Isaac Lab's tensor writes (its API rejects a bare NumPy array)."""
    import torch

    return torch.as_tensor(np.asarray(array, dtype=np.float32), dtype=torch.float32, device=device)


@dataclass(frozen=True)
class GripperState:
    tcp: np.ndarray
    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float
    rot: np.ndarray


PAD_BACK = 0.025


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        rot = np.asarray(runtime.link_poses(robot)["gripper"][:3, :3], float)
        axis = rot[:, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)
        out[f"{side}_gripper"] = GripperState(
            tcp=tcp, back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()), contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()),
                                                 float(GRIPPER["effort"])), rot=rot)
    return out


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


_BUILT = {}  # the spawned Arena objects by name (Isaac only; MuJoCo uses the sim's own scene)
_ISAAC = False  # set when env_cfg_callback sees a real Isaac scene config (Isaac's arm droops; MuJoCo does not)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        obj = _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)
        _BUILT[spec.name] = obj
        return obj

    def object_states(env, names):
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            data = base.scene[name].data
            pose = _arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=_arr(data.root_lin_vel_w),
                                    ang_vel=_arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        extra_states(base, out)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg

            env_cfg = configure(env_cfg)
            _add_contact_sensors(env_cfg)
            env_cfg.events.paste_reset = EventTermCfg(func=_reset_event, mode="reset", params={})
            return env_cfg

        return with_events

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


class _AfterSceneImport(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        run = spec.loader.exec_module

        def exec_module(module):
            run(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _AfterSceneImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _AfterSceneImport())

# -- the objects in the scene -----------------------------------------------------------------------------------------
BENCH = UsdPart("bench", asset="bench", xy=(0.35, 0.0), extent=0.60, mass=1.0,
                usd_path=str(ASSETS / "bench.usda"), pos=(0.0, 0.0, TOP), kinematic=True)
BOARD = UsdPart("board", asset="board", xy=(CPU_X, CPU_Y), extent=0.31, mass=0.90,
                usd_path=str(ASSETS / "board.usda"), pos=(CPU_X, CPU_Y, BOARD_TOP), kinematic=True)
CPU = UsdPart("cpu", asset="cpu", xy=(CPU_X, CPU_Y), extent=0.04, mass=0.050,
              usd_path=str(ASSETS / "cpu.usda"), pos=(CPU_X, CPU_Y, IHS_TOP), kinematic=True)
WIPE = UsdPart("wipe", asset="wipe", xy=PAD_START[:2], extent=0.05, mass=0.0275,
               usd_path=str(ASSETS / "wipe.usda"), pos=PAD_START, rot_xyzw=PAD_ROT)
PASTE = UsdPart("paste", asset="paste", xy=(CPU_X, CPU_Y), extent=0.03, mass=0.001,
                usd_path=str(ASSETS / "paste.usda"), pos=(CPU_X, CPU_Y, IHS_TOP),
                articulation=True)
OBJECTS = [BENCH, BOARD, CPU, WIPE, PASTE]
MOVERS = ("wipe",)

# -- the paste cells (the same order as the asset's joints C0..C121) ---------------------------------------------------
PASTE_CELLS = []
for _j in range(10):
    for _i in range(15):
        _x = (_i + 0.5) * 0.002 - 0.015
        _y = (_j + 0.5) * 0.002 - 0.010
        if (_x / 0.015) ** 2 + (_y / 0.010) ** 2 <= 1.0:
            PASTE_CELLS.append((_x, _y))
NC = len(PASTE_CELLS)

# -- scoring state ----------------------------------------------------------------------------------------------------
@dataclass
class Contact:
    """One physical contact at a control step, from MuJoCo's contact solver: a pad-CPU contact (points in the CPU
    frame), a pad-board/socket contact, or a robot-CPU contact (a hard part on the lid)."""

    kind: str  # "pad", "socket", "hard", "hard_socket"
    force: float
    points: tuple = ()


def _pose(objects, name):
    s = objects[name]
    return np.asarray(s.pos, float), quat_xyzw_to_matrix(np.asarray(s.quat_xyzw, float))


def _gripper_holds(objects) -> bool:
    """A gripper holds the wipe: both fingers on its 50 mm flange (a 40-60 mm opening, a real grip force), with the
    TCP within 5 cm of the wipe's contact face. The wipe's flange is 50 mm; a 49 mm opening is a light squeeze."""
    w = objects.get("wipe")
    if w is None:
        return False
    p = np.asarray(w.pos, float)
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.contact > 0.5 and 0.016 <= g.opening <= 0.024 and float(np.linalg.norm(g.tcp - p)) < 0.050:
            return True
    return False


def _cells_under_pad(pad_pos, R, margin=0.0010):
    """The paste cells whose centres lie under the wipe's 24 x 24 mm contact face (in the wipe's own frame)."""
    out = []
    for n, (x, y) in enumerate(PASTE_CELLS):
        w = np.array([CPU_X + x, CPU_Y + y, IHS_TOP])  # the patch is fixed on the IHS
        loc = R.T @ (w - pad_pos)
        if abs(loc[0]) <= FACE_W / 2 + margin and abs(loc[1]) <= FACE_W / 2 + margin:
            out.append(n)
    return out


class _Paste:
    """The paste's coverage, the wipe's IPA/load and every fault of the run. One update per control step; the live
    contacts are the physical evidence."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.time = -1.0
        self.r = np.ones(NC)
        self.wet = np.zeros(NC)
        self.lint = np.zeros(NC, dtype=np.int32)
        self.pad_wet = WET_START
        self.load = 0.0
        self.picked = False
        self.wet_seen = False
        self.last = None
        self.last_yaw = None
        self.force = 0.0
        self.force_peak = 0.0
        self.ds = 0.0
        self.dirty_used = False
        self.pins = False
        self.scratch = False
        self.pad_touching = False
        self.flat = True
        self.last_time = 0.0

    def clean(self) -> int:
        return int(np.sum(self.r <= CLEAN))

    def fraction(self) -> float:
        return self.clean() / NC

    def step(self, objects, dt=0.05) -> None:
        t = float(objects.get("episode_time", self.time + dt))
        if t == self.time:
            return
        if t < self.time:
            self.reset()
        self.time = t
        dt = 0.05 if t - self.last_time <= 0.0 or t - self.last_time > 0.25 else t - self.last_time
        self.last_time = t
        # the wipe's IPA: it is consumed only while the face is rubbing the lid; it also evaporates slowly
        contacts = tuple(objects.get("contacts", ()))
        pad_force = sum(float(c.force) for c in contacts if c.kind == "pad" and c.force > 0.0)
        self.force = pad_force
        self.force_peak = max(self.force_peak, pad_force)
        self.picked = self.picked or _gripper_holds(objects)
        if self.picked and self.pad_wet > 0.30:
            self.wet_seen = True
        w = objects.get("wipe")
        if w is not None:
            pad_pos, R = _pose(objects, "wipe")
        else:
            pad_pos, R = None, None
        # a pad contact with the board/socket, or with the CPU outside the IHS, is paste on the pins/contacts
        if any(c.kind in ("socket", "hard_socket") and c.force > 0.05 for c in contacts):
            self.pins = True
        if any(c.kind == "hard" and c.force > 0.10 for c in contacts):
            self.scratch = True
        if pad_pos is None:
            self._decay(dt)
            return
        # is the soft face flat on the lid?  The pad's local -z is its contact face.
        tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(R[2, 2])))))
        self.flat = tilt <= 20.0
        # rubbing distance across the lid (planar translation plus the face's rotation)
        if self.last is None:
            ds = 0.0
        else:
            ds = float(math.hypot(pad_pos[0] - self.last[0], pad_pos[1] - self.last[1]))
        self.last = (pad_pos[0], pad_pos[1])
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        if self.last_yaw is None:
            self.last_yaw = yaw
        else:
            dyaw = abs((yaw - self.last_yaw + math.pi) % (2 * math.pi) - math.pi)
            ds += dyaw * (FACE_W / 2)
            self.last_yaw = yaw
        self.ds = ds
        under = _cells_under_pad(pad_pos, R)
        # the pad is touching the IHS only when the solver reports a real normal force and the face is on the lid
        touching = pad_force > FORCE_MIN and self.flat
        self.pad_touching = pad_force > 0.02
        if touching:
            self.pad_wet = max(0.0, self.pad_wet - WET_USE)
        else:
            self.pad_wet *= math.exp(-dt / PAD_WET_LIFE)
        # wet the cells the face covers and dry all films
        for n in under:
            if touching and self.pad_wet > WET_DRY:  # only a wet face leaves IPA (and lifts a dry rub's lint)
                self.wet[n] = 1.0
                self.lint[n] = 0
        self.wet *= math.exp(-dt / WET_LIFE)
        # a dry wipe leaves lint
        if self.picked and self.pad_wet < WET_DRY and pad_force > 0.20 and ds >= RUB_MIN:
            for n in under:
                self.lint[n] = 1
        # a saturated wipe redeposits on clean cells and is capped
        if self.load > PAD_CAP and pad_force > 0.05 and ds >= RUB_MIN:
            self.dirty_used = True
            for n in under:
                if self.r[n] <= CLEAN:
                    self.r[n] = max(self.r[n], REDEPOSIT)
        # a hard press or a dry press scratches; a leaning edge scratches too
        if (pad_force > FORCE_HARD or (self.pad_wet < WET_DRY and pad_force > FORCE_DRY)
                or (tilt > SCRATCH_TILT and pad_force > 0.5)):
            self.scratch = True
        # the removal: only on real contact, only while rubbing, only with IPA in the wipe
        if (touching and self.pad_wet > WET_DRY and ds >= RUB_MIN
                and self.load <= PAD_CAP and under):
            wet = self.pad_wet
            k = K_REMOVE * (ds * 1000.0) * math.sqrt(max(pad_force, 0.0)) * (0.55 + 0.45 * wet)
            for n in under:
                dr = min(self.r[n], k)
                self.r[n] -= dr
                self.load += dr * CELL_PASTE
        self._decay(dt)

    def _decay(self, dt):
        pass

    def pad_clear(self, objects) -> bool:
        w = objects.get("wipe")
        if w is None:
            return False
        p = np.asarray(w.pos, float)
        return bool(abs(p[0] - CPU_X) > CLEAR or abs(p[1] - CPU_Y) > CLEAR or p[2] > IHS_TOP + 0.040)

    def cpu_clear(self, objects) -> bool:
        for side in ("left", "right"):
            g = objects.get(f"{side}_gripper")
            if g is None:
                return False
            if float(np.linalg.norm(np.asarray(g.tcp, float)[:2] - np.array([CPU_X, CPU_Y]))) <= CLEAR:
                return False
        return True

    def full(self, objects) -> bool:
        return bool(self.picked and self.wet_seen and not self.pins and not self.scratch
                    and not self.dirty_used and self.clean() == NC
                    and float(self.wet.max()) <= 0.05 and int(self.lint.sum()) == 0
                    and not self.pad_touching and self.pad_clear(objects) and self.cpu_clear(objects)
                    and self.load <= PAD_CAP)


_PASTE = _Paste()
_PASTE.last_yaw = None


def _reset_event(env, env_ids, asset_cfg=None):
    import torch

    if env_ids is None:
        return
    art = env.scene["paste"]
    ids = art.find_joints([f"C{n}" for n in range(NC)], preserve_order=True)[0]
    zero = torch.zeros((len(env_ids), len(ids)), device=art.device, dtype=torch.float32)
    art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
    _write_paste_targets(env, np.zeros(NC))
    _PASTE.reset()
    _PASTE.last_yaw = None
    _HOLD.update(t=None, pos=None, quat=None)


def _write_paste_targets(env, targets) -> None:
    import torch

    art = env.scene["paste"]
    view = art.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    t[:, :] = torch.as_tensor(np.asarray(targets, float), dtype=t.dtype, device=t.device)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, art._ALL_INDICES)


def _poses(objects, name):
    s = objects[name]
    return np.asarray(s.pos, float), quat_xyzw_to_matrix(np.asarray(s.quat_xyzw, float))


def _mujoco_contacts(sim, base, objects) -> list:
    """The real contacts this step, from MuJoCo's solver: pad-CPU (points in the CPU frame), pad-board, robot-CPU.
    A pad-CPU contact outside the IHS is flagged 'socket' (paste on the socket/pins), not counted for removal."""
    import mujoco

    model, data = sim.model, sim.data
    pad_ids = set(int(b) for b in base.scene["wipe"].body_ids)
    cpu_ids = set(int(b) for b in base.scene["cpu"].body_ids)
    board_ids = set(int(b) for b in base.scene["board"].body_ids)
    robot_ids = set()
    for side in ("left", "right"):
        robot_ids |= set(int(b) for b in base.scene[f"{side}_robot"].body_ids)
    cpu_pos, cpu_R = _poses(objects, "cpu")
    out = []
    for i in range(data.ncon):
        c = data.contact[i]
        b1 = int(model.geom_bodyid[c.geom1])
        b2 = int(model.geom_bodyid[c.geom2])
        pair = {b1, b2}
        arr = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, arr)
        force = abs(float(arr[0]))
        if force <= 0.01:
            continue
        if b1 in pad_ids and b2 in cpu_ids or b2 in pad_ids and b1 in cpu_ids:
            loc = cpu_R.T @ (np.asarray(c.pos, float) - cpu_pos)
            on_ihs = (loc[2] > -0.0008 and abs(loc[0]) <= 0.019 and abs(loc[1]) <= 0.019)
            out.append(Contact("pad" if on_ihs else "socket", force, (tuple(loc),)))
        elif b1 in pad_ids and b2 in board_ids or b2 in pad_ids and b1 in board_ids:
            out.append(Contact("socket", force, (tuple(np.asarray(c.pos, float)),)))
        elif (b1 in cpu_ids and b2 in robot_ids) or (b2 in cpu_ids and b1 in robot_ids):
            out.append(Contact("hard", force, (tuple(np.asarray(c.pos, float)),)))
        elif (b1 in board_ids and b2 in robot_ids) or (b2 in board_ids and b1 in robot_ids):
            out.append(Contact("hard_socket", force, (tuple(np.asarray(c.pos, float)),)))
    return out


def _physx_contacts(base, objects) -> list:
    """The same contact classes from Isaac Lab's PhysX contact reporter (``ContactSensor``).

    ``normal_force_matrix_w`` is the filtered contact force of a sensor body against each filter body;
    ``contact_pos_w`` (on the wipe sensor) is the average real contact point.  A body that is not in
    contact reports zero force (and NaN points), so a wipe above the lid removes nothing."""
    cpu_pos, cpu_R = _poses(objects, "cpu")
    out = []
    data = base.scene["wipe_cpu_contact"].data
    fm, cp = data.normal_force_matrix_w, data.contact_pos_w
    if fm is None or cp is None:
        raise RuntimeError("wipe_cpu_contact: the PhysX contact sensor has no filtered force/points; "
                           "set filter_prim_paths_expr and track_contact_points")
    if fm is not None and cp is not None:
        forces = np.asarray(fm.torch[0, 0].detach().cpu(), dtype=np.float64)  # (cpu, board)
        points = np.asarray(cp.torch[0, 0].detach().cpu(), dtype=np.float64)
        for j in range(min(len(forces), 2)):
            force = float(np.linalg.norm(forces[j]))
            if force <= 0.01:
                continue
            point = points[j] if np.all(np.isfinite(points[j])) else None
            if j == 0:
                if point is None:
                    continue
                loc = cpu_R.T @ (point - cpu_pos)
                on_ihs = (loc[2] > -0.0008 and abs(loc[0]) <= 0.019 and abs(loc[1]) <= 0.019)
                out.append(Contact("pad" if on_ihs else "socket", force, (tuple(loc),)))
            else:
                out.append(Contact("socket", force, (tuple(point),) if point is not None else ()))
    for key, kind in (("cpu_arm_contact", "hard"), ("board_arm_contact", "hard_socket")):
        fm = base.scene[key].data.normal_force_matrix_w
        if fm is None:
            raise RuntimeError(f"{key}: the PhysX contact sensor has no filtered force matrix")
        forces = np.asarray(fm.torch[0, 0].detach().cpu(), dtype=np.float64)  # (left arm, right arm)
        for j in range(len(forces)):
            force = float(np.linalg.norm(forces[j]))
            if force > 0.01:
                out.append(Contact(kind, force, ()))
    return out


def _contacts(base, objects) -> list:
    """The real contacts this step: MuJoCo's solver locally, the PhysX contact reporter on Isaac."""
    sim = getattr(base, "sim_backend", None)
    if sim is not None:
        return _mujoco_contacts(sim, base, objects)
    return _physx_contacts(base, objects)


def extra_states(base, out) -> None:
    pad = out.get("wipe")
    if pad is not None:
        out["contacts"] = _contacts(base, out)
    else:
        out["contacts"] = []
    _PASTE.step(out)
    targets = [-0.0005 if _PASTE.r[n] <= CLEAN else 0.0 for n in range(NC)]
    _write_paste_targets(base, targets)
    if "cpu" in out:
        # the paste is fixed to the IHS: pin its free root to the (fixed) CPU every control step
        try:
            paste = base.scene["paste"]
        except KeyError:
            pass
        else:
            p = np.asarray(out["cpu"].pos, float)
            q = np.asarray(out["cpu"].quat_xyzw, float)
            paste.write_root_pose_to_sim_index(
                root_pose=_torch(np.array([[p[0], p[1], p[2], q[0], q[1], q[2], q[3]]]), paste.device),
                env_ids=None)
            paste.write_root_velocity_to_sim_index(
                root_velocity=_torch(np.zeros((1, 6)), paste.device), env_ids=None)
    if int(base.episode_length_buf[0]) % 20 == 0:
        print(f"[{TAG}] t={_PASTE.time:5.1f}s clean {_PASTE.clean()}/{NC} wipe wet {_PASTE.pad_wet:.2f} "
              f"lint {int(_PASTE.lint.sum())} load {_PASTE.load:.3f} g force {_PASTE.force:.2f} N "
              f"peak {_PASTE.force_peak:.2f} N wipe z {out['wipe'].pos[2]:.4f} "
              f"isaac {_ISAAC} press {PRESS_EXTRA_MM if _ISAAC else 0.0:.2f} tcp z {out['right_gripper'].tcp[2]:.4f} "
              f"rub {_PASTE.ds * 1000:.1f} mm pins {_PASTE.pins} scratch {_PASTE.scratch} "
              f"dirty {_PASTE.dirty_used}", flush=True)
        _PASTE.force_peak = 0.0


# -- scoring ----------------------------------------------------------------------------------------------------------
_HOLD = {"t": None, "pos": None, "quat": None}
STEPS = ("the wet wipe was picked up", "most of the paste removed (60 %)",
         "95 % of the cells clean", "clean, dry, no lint, no paste on the pins, no scratch")
POINTS = 0.25


def _update(objects) -> None:
    _PASTE.step(objects)


def score(objects) -> float:
    """0 until a gripper has picked up the wet wipe. 0.25 once it has. 0.5 once at least 60 % of the paste cells
    are clean. 0.75 once 95 % are clean. 1.0 when every cell is clean, the IPA film is dry (max cell wetness
    <= 0.05), there is no lint, no paste on the socket/pins (no pad contact with the board or with the CPU outside
    the IHS), no scratch (no hard part on the lid, no dry press over 2 N, no wet press over 6 N, no leaning edge),
    the wipe is below its paste capacity (0.12 g), and the wipe and both grippers are clear of the CPU for the 1 s
    hold. A saturated wipe that redeposits caps the run at 0.5; paste on the socket/pins caps it at 0.25; a scratch
    caps it at 0.5. A dry wipe removes nothing (and can leave lint). Doing nothing scores 0."""
    _update(objects)
    if not (_PASTE.picked and _PASTE.wet_seen):
        return 0.0
    f = _PASTE.fraction()
    base = 0.25
    if f >= MOST:
        base = 0.5
    if f >= ALMOST:
        base = 0.75
    if _PASTE.full(objects):
        base = 1.0
    if _PASTE.pins:
        base = min(base, 0.25)
    if _PASTE.scratch or _PASTE.dirty_used:
        base = min(base, 0.5)
    return float(base)


def steps_done(objects) -> list[bool]:
    _update(objects)
    f = _PASTE.fraction()
    return [_PASTE.picked and _PASTE.wet_seen, f >= MOST, f >= ALMOST, _PASTE.full(objects)]


def success(objects) -> bool:
    """1.0 held for 1 s, with the wipe and both grippers clear of the CPU and the wipe still (within 2 mm and 2
    degrees of where it was when the second began). The CPU is fixed in its socket."""
    _update(objects)
    if not _PASTE.full(objects):
        _HOLD.update(t=None, pos=None, quat=None)
        return False
    w = objects["wipe"]
    p = np.asarray(w.pos, float)
    q = np.asarray(w.quat_xyzw, float)
    if _HOLD["pos"] is None:
        _HOLD.update(t=0.0, pos=p.copy(), quat=q.copy())
        return False
    dot = min(1.0, abs(float(np.dot(q, _HOLD["quat"]))))
    turn = 2.0 * math.acos(dot)
    if float(np.linalg.norm(p - _HOLD["pos"])) > STILL_MOVE or turn > STILL_TURN:
        _HOLD.update(t=0.0, pos=p.copy(), quat=q.copy())
        return False
    _HOLD["t"] += 0.05
    return bool(_HOLD["t"] >= STILL_S - 1e-9)


# -- the scripted demo ------------------------------------------------------------------------------------------------
@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None
    press: float = 0.0  # 1.0: a contact waypoint; Isaac's arm holds the wipe ~1 mm high, so aim lower there


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float).copy()
        if _ISAAC and getattr(w, "press", 0.0):
            goal[2] -= PRESS_EXTRA_MM * MM * float(w.press)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
OPEN = FINGER_OPEN


def _grip(yaw_deg: float = 0.0) -> np.ndarray:
    return yam_demo.grip_rotation(math.radians(yaw_deg))


def demo():
    """Open loop, known geometry. The right arm picks the folded wipe by its flange, carries it over the CPU, lowers
    its 24 x 24 mm contact face on the IHS 20 um into the lid (about 0.8 N), wipes four overlapping passes over the
    30 x 20 mm patch, lifts, places the wipe in the waste tray and parks while the IPA film dries. The left arm stays
    at ready. The wipe is held by its flange, so the fingers never come near the lid."""
    r = _grip(0.0)
    pick_z = PAD_START[2] + PAD_TCP_ABOVE
    wipe_z = IHS_TOP + PAD_TCP_ABOVE - 0.00002
    right = [Waypoint(1.0, tuple(_START_TCP["right"]), OPEN, "home", rot=_START_ROT["right"]),
             Waypoint(1.2, (PAD_START[0], PAD_START[1], TOP + 0.120), OPEN, "over the wipe", rot=r),
             Waypoint(1.0, (PAD_START[0], PAD_START[1], pick_z), PAD_PICK_OPEN, "down to the wipe", rot=r),
             Waypoint(0.8, (PAD_START[0], PAD_START[1], pick_z), PAD_GRIP, "grip the wipe", rot=r),
             Waypoint(1.2, (PAD_START[0], PAD_START[1], TOP + 0.120), PAD_GRIP, "lift", rot=r),
             Waypoint(1.6, (CPU_X, CPU_Y, TOP + 0.120), PAD_GRIP, "over the CPU", rot=r),
             Waypoint(1.0, (CPU_X, CPU_Y, wipe_z), PAD_GRIP, "wipe on the IHS", rot=r, press=1.0)]
    # four overlapping passes across the 30 x 20 mm patch; the face is 24 mm wide, the centres +-14 mm x, +-4 mm y
    for k, y in enumerate((-0.0040, -0.0013, 0.0013, 0.0040)):
        x0, x1 = (-0.0140, 0.0140) if k % 2 == 0 else (0.0140, -0.0140)
        right += [Waypoint(0.6, (CPU_X + x0, CPU_Y + y, wipe_z), PAD_GRIP, "line up the pass", rot=r, press=1.0),
                  Waypoint(2.8, (CPU_X + x1, CPU_Y + y, wipe_z), PAD_GRIP, "wipe", rot=r, press=1.0)]
    # the two edge columns are at the ends of the raster: one slow pass down each edge cleans them
    right += [Waypoint(0.6, (CPU_X + 0.0135, CPU_Y - 0.004, wipe_z), PAD_GRIP, "right edge line-up", rot=r, press=1.0),
              Waypoint(2.4, (CPU_X + 0.0135, CPU_Y + 0.004, wipe_z), PAD_GRIP, "right edge wipe", rot=r, press=1.0),
              Waypoint(0.6, (CPU_X - 0.0135, CPU_Y + 0.004, wipe_z), PAD_GRIP, "left edge line-up", rot=r, press=1.0),
              Waypoint(2.4, (CPU_X - 0.0135, CPU_Y - 0.004, wipe_z), PAD_GRIP, "left edge wipe", rot=r, press=1.0)]
    right += [Waypoint(1.0, (CPU_X, CPU_Y, TOP + 0.120), PAD_GRIP, "off the CPU", rot=r),
              Waypoint(1.6, (0.460, -0.240, TOP + 0.120), PAD_GRIP, "over the waste tray", rot=r),
              Waypoint(0.9, (0.460, -0.240, TOP + 0.0072 + PAD_TCP_ABOVE), PAD_GRIP, "lower", rot=r),
              Waypoint(0.6, (0.460, -0.240, TOP + 0.0072 + PAD_TCP_ABOVE), OPEN, "release the wipe", rot=r),
              Waypoint(1.0, (0.460, -0.240, TOP + 0.120), OPEN, "up", rot=r),
              Waypoint(1.6, (0.120, -0.300, TOP + 0.200), OPEN, "park", rot=_START_ROT["right"]),
              Waypoint(9.0, (0.120, -0.300, TOP + 0.200), OPEN, "let the IPA dry", rot=_START_ROT["right"])]
    return {"right": right}


if __name__ == "__main__":
    print("objects:", [o.name for o in OBJECTS], "paste cells:", NC)
