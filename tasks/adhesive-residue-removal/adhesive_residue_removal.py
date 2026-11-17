"""Remove leftover price-label adhesive from a returned pan's base with the 91% IPA wipe.

On the returns bench: a real Calphalon Premier hard-anodized 12-inch pan (30.5 cm, 1.63 kg) lies upside down on a
high-friction silicone mat.  An Avery 5163 label was pulled off its base and lies on the waste tray; a real 80 x 40 mm
patch of pressure-sensitive adhesive is left.  The left hand holds the pan's stainless handle (the pan is free, not
clamped).  The right hand lays a Chemtronics Chempad (91% IPA) on the patch, lets it dwell, then wipes with a fresh
pad and finally with a third fresh white pad, and leaves the base clean and dry.

The residue is a declared 128-cell material-removal layer (16 x 8 cells of 5 mm on the 80 x 40 mm patch).  A cell is
removed only by a pad that is in real contact with the pan base: the live scorer reads the normal force and the
contact points from MuJoCo's contact solver, and the rubbing distance from the pad's motion in the pan's frame.  A
pad held above the base removes nothing; moving the pad beside the patch removes nothing.  The pad's IPA wets the
cells and the dwell makes them soft; the wipe removes them by contact force and rubbing distance, not by a timer.

Run it:  python -m mjkit.run adhesive_residue_removal.py --steps 1800 --trace
"""

TAG = "adhesive"
import importlib.abc
import importlib.util
import math
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

# -- the real products (see REFERENCE.md and task.json fidelity) ------------------------------------------------------
PAN_MASS, PAN_R, PAN_BASE_R, PAN_H = 1.63, 0.1525, 0.120, 0.064
PAN_BASE_Z = 0.064  # in the pan's frame: the wiped base face
PAD_W, PAD_L, PAD_T = 0.076, 0.102, 0.003
PAD_MASS = 0.045  # the holder and one Chempad
MAT_TOP = TOP + 0.003
PAN_START = (0.270, 0.000, MAT_TOP + 0.0008)
HANDLE_L = 0.200
PATCH_X, PATCH_Y = 0.040, 0.080
N_X, N_Y, CELL = 8, 16, 0.005
NC = N_X * N_Y

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) -----------------------------
TOOL_XY = (0.280, -0.290)  # the 260 x 220 mm tool/pad tray
PAD_XY = ((0.180, -0.300), (0.260, -0.300), (0.340, -0.300))
WASTE_XY = (0.480, -0.220)
GRID_XY = (0.470, 0.330)

# -- the declared wipe process ----------------------------------------------------------------------------------------
DWELL_S = 20.0  # sim: 20 s wet contact (the real protocol uses 30 s; the stopwatch decides)
WET_LIFE = 6.0  # s for the surface IPA film to dry; the base must be dry at the end
SOFT_LIFE = 45.0  # s the swollen adhesive stays soft after the pad lifts
K_SOFT, K_WET, K_DRY = 0.20, 0.08, 0.02  # residue fraction removed per mm of pad rubbing
PAD_FORCE_MIN = 0.5  # N
PAD_FLAT_MAX = 20.0  # deg
LOAD_PER_CELL = 0.0005  # g of adhesive in one 5 mm cell (0.02 mm film at 20 g/m2)
PAD_LOAD_MAX = 0.050  # g: a pad carrying more than this redeposits a film
REDEPOSIT = 0.12  # residue left on a clean cell by a loaded pad
OUTSIDE = 0.02  # any visible film outside the 80 x 40 mm patch
LADDER = (0.60, 0.90, 0.98)  # cells clean: 0.5 / 0.75 / 1.0; 0.25 is the pad wetted and placed

# -- assets ---------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple = (0.0, 0.0, 0.0)
    rot_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False
    no_gravity: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


BENCH = UsdPart("bench", asset="bench", xy=(0.0, 0.0), extent=0.55, mass=1.0,
                usd_path=str(ASSETS / "bench.usda"), pos=(0.0, 0.0, TOP), kinematic=True)
PAN = UsdPart("pan", asset="pan", xy=PAN_START[:2], extent=0.32, mass=PAN_MASS,
              usd_path=str(ASSETS / "pan_calphalon.usda"), pos=PAN_START)
PADS = tuple(
    UsdPart(f"pad{i + 1}", asset="pad", xy=xy, extent=0.11, mass=PAD_MASS,
            usd_path=str(ASSETS / "pad_holder.usda"), pos=(xy[0], xy[1], TOP + 0.020 + 0.0005 + 0.0006))
    for i, xy in enumerate(PAD_XY))
RESIDUE = UsdPart("residue", asset="residue", xy=PAN_START[:2], extent=0.06, mass=0.001,
                  usd_path=str(ASSETS / "residue.usda"),
                  pos=(PAN_START[0], PAN_START[1], PAN_START[2] + PAN_BASE_Z + 0.00004),
                  articulation=True, no_gravity=True)
OBJECTS = [BENCH, PAN, *PADS, RESIDUE]
MOVERS = ("pan", "pad1", "pad2", "pad3")
SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2


def _spawn(scene, spec):
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             solver_position_iteration_count=SOLVER_ITERATIONS,
                                                             solver_velocity_iteration_count=1,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False)
    return scene.Object(
        name=spec.name, usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon)


def _add_contact_sensors(env_cfg) -> None:
    """One PhysX contact reporter per pad, filtered to the pan: the supported Isaac Lab contact API.

    Isaac Lab's ``ContactSensor`` reads PhysX's ``RigidContactView`` (``normal_force_matrix_w`` and, with
    ``track_contact_points``, ``contact_pos_w``).  The pad and pan spawners activate ``PhysxContactReportAPI``
    on their rigid bodies (Arena does it for every RIGID object), so these terms report only the real
    solver contact force and the average contact point of each pad-pan pair.  Without contact there is no
    force and no point, exactly as the MuJoCo reader returns nothing.  ``env_cfg.scene`` is absent in the
    MuJoCo harness, which keeps its own contact reader."""
    scene_cfg = getattr(env_cfg, "scene", None)
    if scene_cfg is None or "pan" not in _BUILT:
        return
    pan = _BUILT["pan"]
    for name in ("pad1", "pad2", "pad3"):
        pad = _BUILT.get(name)
        if pad is None or not hasattr(pad, "get_contact_sensor_cfg"):
            continue
        cfg = pad.get_contact_sensor_cfg(pan)
        cfg.track_contact_points = True
        cfg.history_length = 0
        cfg.max_contact_data_count_per_prim = 16
        setattr(scene_cfg, f"{name}_pan_contact", cfg)


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


PAD_BACK = 0.025  # m behind the TCP


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        rot = runtime.link_poses(robot)["gripper"][:3, :3]
        axis = rot[:, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)
        out[f"{side}_gripper"] = GripperState(
            tcp=tcp, back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()), contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()),
                                                 float(GRIPPER["effort"])), rot=np.asarray(rot, float))
    return out


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


_BUILT = {}  # the spawned Arena objects by name (Isaac only; MuJoCo uses the sim's own scene)


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
        extra_states(base, out, origin)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            _add_contact_sensors(env_cfg)
            env_cfg.events.adhesive_reset = EventTermCfg(func=_reset_event, mode="reset",
                                                         params={"asset_cfg": SceneEntityCfg("residue")})
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

# -- the residue state -----------------------------------------------------------------------------------------------


def cell_xy(n: int) -> tuple[float, float]:
    i, j = n % N_X, n // N_X
    return (i + 0.5) * CELL - PATCH_X / 2, (j + 0.5) * CELL - PATCH_Y / 2


def outside_patch(x: float, y: float) -> bool:
    return abs(x) > PATCH_X / 2 + 0.003 or abs(y) > PATCH_Y / 2 + 0.003


@dataclass
class Contact:
    """One pad's contact with the pan base at a control step, as the task model needs it."""

    kind: str  # "pad1", "pad2", "pad3"
    force: float  # total normal force on the base, N
    angle: float  # deg between the pad face and the base plane
    ds: float  # m the pad center moved over the pan since the last step
    points: tuple = ()  # contact points in the pan frame (x, y, z)
    pad_box: tuple = ()  # pad footprint in the pan frame: (cx, cy, hx, hy, z)
    grit: bool = False  # a contaminated pad scratches the anodized finish
    dry: bool = False  # a pad with no IPA: it does not wet the residue


class _Residue:
    def __init__(self):
        self.time = -1.0
        self.r = np.ones(NC)
        self.wet = np.zeros(NC)
        self.last_wet = np.full(NC, -1e9)
        self.dwell = np.zeros(NC)
        self.soft = np.zeros(NC, dtype=bool)
        self.soft_until = np.full(NC, -1e9)
        self.load = {"pad1": 0.0, "pad2": 0.0, "pad3": 0.0}
        self.pad_time = {"pad1": 0.0, "pad2": 0.0, "pad3": 0.0}
        self.pad_wiped = {"pad1": 0.0, "pad2": 0.0, "pad3": 0.0}
        self.last_tool = None
        self.last_pad = None
        self.placed = False
        self.wetted = False
        self.scratch = False
        self.outside = 0.0
        self.dropped = False

    def reset(self):
        self.__init__()

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is None:
            t = self.time + 0.05
        if t == self.time:
            return
        if t < self.time:
            self.__init__()
        self.time = t
        dt = 0.05
        wet = np.clip(1.0 - (t - self.last_wet) / WET_LIFE, 0.0, 1.0)
        wet[self.last_wet < -1e8] = 0.0
        self.wet = wet
        self.soft = (self.dwell >= DWELL_S) & (t < self.soft_until)
        left = bool(objects.get("left_holding"))
        for c in objects.get("contacts", ()):
            if c.force <= 0.0:
                continue
            self._pad(c, left, dt)
        if objects.get("pan_off"):
            self.dropped = True

    def _pad(self, c: Contact, left: bool, dt: float) -> None:
        if not c.pad_box or c.force < PAD_FORCE_MIN or c.angle > PAD_FLAT_MAX or not left:
            return
        cx, cy, hx, hy, z = c.pad_box
        if abs(z - PAN_BASE_Z) > 0.002:
            return
        under = [n for n in range(NC)
                 if abs(cell_xy(n)[0] - cx) <= hx + CELL / 2 and abs(cell_xy(n)[1] - cy) <= hy + CELL / 2]
        if not under:
            # a loaded pad dragged off the patch leaves adhesive there; a fresh pad can lift a smear
            if c.ds > 0.0:
                if self.load.get(c.kind, 0.0) > PAD_LOAD_MAX:
                    self.outside = max(self.outside, 0.35)
                elif self.load.get(c.kind, 0.0) <= PAD_LOAD_MAX:
                    self.outside = max(0.0, self.outside - 0.20)
            return
        self.placed = True
        if c.grit and c.ds > 0.0:
            self.scratch = True
        if not c.dry:
            self.wetted = True
            for n in under:
                self.last_wet[n] = self.time
                self.wet[n] = 1.0
                self.dwell[n] += dt
                self.soft_until[n] = max(self.soft_until[n], self.time + SOFT_LIFE)
        if self.load[c.kind] <= PAD_LOAD_MAX and c.ds > 0.0:
            for n in under:
                k = K_DRY if c.dry else (K_SOFT if self.soft[n] else (K_WET if self.wet[n] > 0.15 else K_DRY))
                dr = min(self.r[n], k * (c.ds * 1000.0))
                self.r[n] -= dr
                self.load[c.kind] += dr * LOAD_PER_CELL
        elif self.load[c.kind] > PAD_LOAD_MAX:
            for n in under:
                if self.r[n] < REDEPOSIT:
                    self.r[n] = REDEPOSIT
        self.pad_time[c.kind] = self.pad_time.get(c.kind, 0.0) + dt
        self.pad_wiped[c.kind] += c.ds
        self.last_tool = c.kind
        self.last_pad = c.kind

    def clean(self) -> int:
        return int(np.sum(self.r <= 0.01))

    def fraction(self) -> float:
        return self.clean() / NC

    def wet_max(self) -> float:
        return float(self.wet.max())

    def fresh_final_pad(self) -> bool:
        if self.last_pad is None or self.last_tool != self.last_pad:
            return False
        if self.pad_time.get(self.last_pad, 0.0) > 3.0:
            return False
        if self.load.get(self.last_pad, 0.0) > PAD_LOAD_MAX:
            return False
        return self.pad_time.get(self.last_pad, 0.0) >= 0.5

    def full(self) -> bool:
        return (not self.scratch and not self.dropped and self.outside <= OUTSIDE
                and self.clean() >= math.ceil(LADDER[2] * NC - 1e-9)
                and self.fresh_final_pad() and self.wet_max() <= 0.05)


_RES = _Residue()
_TOOL_LAST = {"pad1": None, "pad2": None, "pad3": None}


def _reset_event(env, env_ids, asset_cfg=None):
    import torch

    if env_ids is None:
        return
    art = env.scene["residue"]
    ids = art.find_joints([f"C{n}" for n in range(NC)], preserve_order=True)[0]
    zero = torch.zeros((1, len(ids)), device=art.device, dtype=torch.float32)
    art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
    _write_cell_targets(env, np.zeros(NC))
    _RES.reset()
    _HOLD.update(t=None, pos=None, quat=None)
    for k in _TOOL_LAST:
        _TOOL_LAST[k] = None


def _write_cell_targets(env, targets) -> None:
    import torch

    art = env.scene["residue"]
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


def _pose(objects, name):
    s = objects[name]
    return np.asarray(s.pos, float), quat_xyzw_to_matrix(np.asarray(s.quat_xyzw, float))


def _to_pan(objects, pts, R=None, p=None):
    if R is None or p is None:
        p, R = _pose(objects, "pan")
    q = np.asarray(pts, float).reshape(-1, 3)
    return (q - p) @ R


def _pad_contact(k, objects, force, points) -> Contact:
    """One pad's contact record: force and points already known, the pad box, flatness and rubbing from the pose."""
    p, R = _pose(objects, "pan")
    s = objects[k]
    pose = np.asarray(s.pos, float), quat_xyzw_to_matrix(np.asarray(s.quat_xyzw, float))
    corners = np.array([[-PAD_W / 2, -PAD_L / 2, 0.0], [PAD_W / 2, -PAD_L / 2, 0.0],
                        [PAD_W / 2, PAD_L / 2, 0.0], [-PAD_W / 2, PAD_L / 2, 0.0]])
    w = pose[0] + corners @ pose[1].T
    l = _to_pan(objects, w)
    cx, cy = float(l[:, 0].mean()), float(l[:, 1].mean())
    hx = (float(l[:, 0].max()) - float(l[:, 0].min())) / 2.0
    hy = (float(l[:, 1].max()) - float(l[:, 1].min())) / 2.0
    normal = pose[1] @ np.array([0.0, 0.0, 1.0])
    angle = math.degrees(math.acos(min(1.0, float(np.clip(abs(normal @ R[:, 2]), -1.0, 1.0)))))
    ctr = np.array([cx, cy])
    if _TOOL_LAST[k] is None:
        ds = 0.0
    else:
        ds = float(np.linalg.norm(ctr - _TOOL_LAST[k]))
    _TOOL_LAST[k] = ctr
    return Contact(k, float(force), angle, ds, tuple(points),
                   pad_box=(cx, cy, hx, hy, float(l[:, 2].mean())))


def _mujoco_tool_contacts(sim, base, objects):
    """The real pad contact force, points, angle and rubbing distance, from MuJoCo's contacts."""
    import mujoco

    model, data = sim.model, sim.data
    want = {"pad1": "pad1", "pad2": "pad2", "pad3": "pad3"}
    pan_ids = set(int(b) for b in base.scene["pan"].body_ids)
    tool_ids = {name: set(int(b) for b in base.scene[name].body_ids) for name in want}
    p, R = _pose(objects, "pan")
    force = {k: 0.0 for k in want}
    points = {k: [] for k in want}
    arr = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        b1 = int(model.geom_bodyid[c.geom1])
        b2 = int(model.geom_bodyid[c.geom2])
        k = None
        if b1 in pan_ids:
            k = next((name for name, ids in tool_ids.items() if b2 in ids), None)
        elif b2 in pan_ids:
            k = next((name for name, ids in tool_ids.items() if b1 in ids), None)
        if k is None:
            continue
        mujoco.mj_contactForce(model, data, i, arr)
        local = (np.asarray(c.pos, float) - p) @ R
        if math.hypot(local[0], local[1]) > PAN_BASE_R - 0.001:
            continue
        if abs(local[2] - PAN_BASE_Z) > 0.004:
            continue
        force[k] += float(arr[0])
        points[k].append(tuple(float(v) for v in local))
    out = []
    for k in want:
        if force[k] <= 0.02 or not points[k]:
            _TOOL_LAST[k] = None
            continue
        out.append(_pad_contact(k, objects, force[k], points[k]))
    return out


def _physx_tool_contacts(base, objects):
    """The pad-pan contacts from Isaac Lab's PhysX contact reporter (``ContactSensor``).

    The sensor's ``normal_force_matrix_w`` is the filtered contact force on the pad from the pan; its
    component along the pan base normal is the normal force.  ``contact_pos_w`` is the average of the
    real contact points of that pair (NaN with no contact).  Both are world-frame PhysX contact data,
    not a timer or a proximity estimate: a pad above the base reports zero force and no point."""
    p, R = _pose(objects, "pan")
    normal = R[:, 2]  # the pan base face points up in the pan frame (+z)
    out = []
    for k in ("pad1", "pad2", "pad3"):
        data = base.scene[f"{k}_pan_contact"].data
        fm, cp = data.normal_force_matrix_w, data.contact_pos_w
        if fm is None or cp is None:
            raise RuntimeError(f"{k}_pan_contact: the PhysX contact sensor has no filtered force/points; "
                               "set filter_prim_paths_expr and track_contact_points")
        f = np.asarray(fm.torch[0, 0, 0].detach().cpu(), dtype=np.float64)
        c = np.asarray(cp.torch[0, 0, 0].detach().cpu(), dtype=np.float64)
        force = abs(float(f @ normal))
        if force <= 0.02 or not np.all(np.isfinite(c)):
            _TOOL_LAST[k] = None
            continue
        local = (c - p) @ R
        if math.hypot(local[0], local[1]) > PAN_BASE_R - 0.001:
            _TOOL_LAST[k] = None
            continue
        if abs(local[2] - PAN_BASE_Z) > 0.004:
            _TOOL_LAST[k] = None
            continue
        out.append(_pad_contact(k, objects, force, [tuple(float(v) for v in local)]))
    return out


def _tool_contacts(base, objects):
    """The real pad contact force, points, angle and rubbing distance: MuJoCo's solver locally, PhysX on Isaac."""
    sim = getattr(base, "sim_backend", None)
    if sim is not None:
        return _mujoco_tool_contacts(sim, base, objects)
    return _physx_tool_contacts(base, objects)


def _holds_handle(objects) -> bool:
    g = objects.get("left_gripper")
    if g is None or g.contact < 1.0 or not (0.020 <= g.opening <= 0.040):
        return False
    p, R = _pose(objects, "pan")
    q = (g.tcp - p) @ R
    return abs(q[0]) <= 0.020 and 0.145 <= q[1] <= 0.360 and 0.018 <= q[2] <= 0.052


def extra_states(base, out, origin) -> None:
    p, R = _pose(out, "pan")
    out["pan_off"] = bool(p[2] < TOP - 0.02 or p[2] > TOP + 0.15
                          or math.degrees(math.acos(min(1.0, abs(float(R[2, 2]))))) > 60.0)
    out["left_holding"] = _holds_handle(out)
    out["contacts"] = _tool_contacts(base, out)
    art = base.scene["residue"]
    root = p + R @ np.array([0.0, 0.0, PAN_BASE_Z + 0.00004])
    q = np.asarray(out["pan"].quat_xyzw, float)
    art.write_root_pose_to_sim_index(
        root_pose=_torch(np.array([[root[0], root[1], root[2], q[0], q[1], q[2], q[3]]]), art.device), env_ids=None)
    art.write_root_velocity_to_sim_index(
        root_velocity=_torch(np.zeros((1, 6)), art.device), env_ids=None)
    _RES.update(out)
    _write_cell_targets(base, [-0.0003 if _RES.r[n] <= 0.01 else 0.0 for n in range(NC)])
    if int(base.episode_length_buf[0]) % 20 == 0:
        f = max((c.force for c in out["contacts"]), default=0.0)
        print(f"[{TAG}] t={_RES.time:5.1f}s clean {_RES.clean()}/{NC} wet {_RES.wet_max():.2f} "
              f"soft {int(_RES.soft.sum())} load {_RES.load['pad1']:.3f}/{_RES.load['pad2']:.3f}/"
              f"{_RES.load['pad3']:.3f} g outside {_RES.outside:.2f} scratch {_RES.scratch} "
              f"placed {_RES.placed} left {out['left_holding']} force {f:.1f} N", flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("the pad wetted and placed on the patch", "60% of the cells clean", "90% of the cells clean",
         "98% clean, fresh white final pad, dry")
STILL_MOVE, STILL_TURN, STILL_S = 0.002, math.radians(1.0), 1.0
_HOLD = {"t": None, "pos": None, "quat": None}


def _reset_hold():
    _HOLD.update(t=None, pos=None, quat=None)


def score(objects) -> float:
    _RES.update(objects)
    if _RES.scratch or _RES.outside > OUTSIDE or _RES.dropped or objects.get("pan_off"):
        return 0.0
    if not _RES.placed or not _RES.wetted:
        return 0.0
    f = _RES.fraction()
    if f >= LADDER[2]:
        return 1.0 if (_RES.fresh_final_pad() and _RES.wet_max() <= 0.05) else 0.5
    if f >= LADDER[1]:
        return 0.75 if _RES.fresh_final_pad() else 0.5
    if f >= LADDER[0]:
        return 0.5
    return 0.25


def steps_done(objects) -> tuple[bool, ...]:
    _RES.update(objects)
    f = _RES.fraction()
    return (bool(_RES.placed and _RES.wetted), f >= LADDER[0], f >= LADDER[1],
            f >= LADDER[2] and _RES.fresh_final_pad() and _RES.wet_max() <= 0.05)


def success(objects) -> bool:
    """1.0 held for 1 s with the tool off the pan and the pan still; the left hand may keep holding the handle."""
    _RES.update(objects)
    if score(objects) < 1.0 or any(c.force > 0.05 for c in objects.get("contacts", ())):
        _reset_hold()
        return False
    p, R = _pose(objects, "pan")
    q = np.asarray(objects["pan"].quat_xyzw, float)
    if _HOLD["pos"] is None:
        _HOLD.update(t=0.05, pos=p.copy(), quat=q.copy())
        return False
    dot = min(1.0, abs(float(np.dot(q, _HOLD["quat"]))))
    turn = 2.0 * math.acos(dot)
    if float(np.linalg.norm(p - _HOLD["pos"])) > STILL_MOVE or turn > STILL_TURN:
        _HOLD.update(t=0.05, pos=p.copy(), quat=q.copy())
        return False
    _HOLD["t"] += 0.05
    return _HOLD["t"] >= STILL_S - 1e-9


# -- demo -----------------------------------------------------------------------------------------------------------
@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
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


def _grip(yaw_deg: float) -> np.ndarray:
    return yam_demo.grip_rotation(math.radians(yaw_deg))


def _pick(xy, z_top, handle_open, approach=0.04, lift=0.04):
    """Top-down pick of a pad holder at (x, y); z_top is the TCP's grasp height."""
    x, y = xy
    return [Waypoint(1.8, (x, y, TOP + approach + 0.04), OPEN, "above", yaw=90.0),
            Waypoint(1.2, (x, y, z_top + 0.06), OPEN, "lower", yaw=90.0),
            Waypoint(1.0, (x, y, z_top), OPEN, "at the grip", yaw=90.0),
            Waypoint(0.5, (x, y, z_top), handle_open, "close", yaw=90.0),
            Waypoint(1.0, (x, y, lift + TOP + 0.04), handle_open, "lift", yaw=90.0)]


def _home(side, duration=1.5):
    return Waypoint(duration, tuple(_START_TCP[side]), OPEN, "home", rot=_START_ROT[side])


def demo():
    """Open loop, known geometry: the left hand grips the pan handle.  The right hand wets pad 1 on the patch for
    20 s, wipes with fresh pad 2, wipes with fresh white pad 3, waits 6 s for the IPA to dry and goes home.  The pan
    is never clamped or lifted."""
    o, c = OPEN, 0.0
    left = [Waypoint(2.0, (0.270, 0.175, TOP + 0.100), OPEN, "above the handle", yaw=90.0),
            Waypoint(1.2, (0.270, 0.175, TOP + 0.0385), OPEN, "down to the handle", yaw=90.0),
            Waypoint(0.6, (0.270, 0.175, TOP + 0.0385), -0.0120, "grip the handle", yaw=90.0),
            Waypoint(6.0, (0.270, 0.175, TOP + 0.0385), -0.0120, "hold the pan", yaw=90.0)]
    pad_handle_z = TOP + 0.020 + 0.0005 + 0.0006 + 0.007 + 0.015  # pad bottom + foot + half the grip bar
    pad_press_z = MAT_TOP + PAN_BASE_Z + 0.007 + 0.015 - 0.0030  # a light press on the Chempad
    grip = -0.0100
    right = [Waypoint(1.0, tuple(_START_TCP["right"]), OPEN, "wait for the left hand", rot=_START_ROT["right"])]

    # pad 1: wet and dwell 20 s
    right += _pick(PAD_XY[0], pad_handle_z, grip)
    right += [Waypoint(1.8, (0.270, 0.000, TOP + 0.155), grip, "over the pan", rot=_grip(90)),
              Waypoint(1.2, (0.270, 0.000, pad_press_z + 0.030), grip, "lower", rot=_grip(90)),
              Waypoint(0.8, (0.270, 0.000, pad_press_z), grip, "pad on the patch", rot=_grip(90)),
              Waypoint(0.5, (0.270, 0.000, pad_press_z - 0.0005), grip, "press", rot=_grip(90)),
              Waypoint(19.0, (0.270, 0.000, pad_press_z - 0.0005), grip, "dwell 20 s", rot=_grip(90)),
              Waypoint(0.5, (0.270, 0.000, pad_press_z - 0.0005), grip, "dwell", rot=_grip(90)),
              Waypoint(1.0, (0.270, 0.000, TOP + 0.155), grip, "lift the wet pad", rot=_grip(90)),
              Waypoint(1.8, (PAD_XY[0][0], PAD_XY[0][1], TOP + 0.150), grip, "over the wet pad's foil", rot=_grip(90)),
              Waypoint(1.0, (PAD_XY[0][0], PAD_XY[0][1], pad_handle_z + 0.010), grip, "lower", rot=_grip(90)),
              Waypoint(0.5, (PAD_XY[0][0], PAD_XY[0][1], pad_handle_z + 0.010), o, "release the wet pad", rot=_grip(90)),
              Waypoint(0.8, (PAD_XY[0][0], PAD_XY[0][1], TOP + 0.150), o, "up", rot=_grip(90)),
              Waypoint(1.4, (0.300, -0.100, TOP + 0.200), OPEN, "reset the wrist", rot=_START_ROT["right"])]

    # pad 2: wipe the softened residue
    right += _pick(PAD_XY[1], pad_handle_z, grip)
    right += [Waypoint(1.8, (0.270, 0.000, TOP + 0.155), grip, "over the pan", rot=_grip(90)),
              Waypoint(1.2, (0.270, 0.000, pad_press_z + 0.030), grip, "lower", rot=_grip(90)),
              Waypoint(0.8, (0.270, 0.000, pad_press_z), grip, "wipe pad on the patch", rot=_grip(90)),
              Waypoint(0.5, (0.270, 0.000, pad_press_z - 0.0005), grip, "press", rot=_grip(90)),
              Waypoint(0.7, (0.285, 0.000, pad_press_z - 0.0005), grip, "wipe across", rot=_grip(90)),
              Waypoint(0.7, (0.255, 0.000, pad_press_z - 0.0005), grip, "wipe back", rot=_grip(90)),
              Waypoint(0.6, (0.270, 0.000, pad_press_z - 0.0005), grip, "finish the wipe", rot=_grip(90)),
              Waypoint(1.0, (0.270, 0.000, TOP + 0.155), grip, "lift the wipe pad", rot=_grip(90)),
              Waypoint(1.8, (WASTE_XY[0], WASTE_XY[1], TOP + 0.100), grip, "over the waste", rot=_grip(90)),
              Waypoint(1.0, (WASTE_XY[0], WASTE_XY[1], pad_handle_z + 0.010), grip, "lower", rot=_grip(90)),
              Waypoint(0.5, (WASTE_XY[0], WASTE_XY[1], pad_handle_z + 0.010), o, "release the wipe pad", rot=_grip(90)),
              Waypoint(0.8, (WASTE_XY[0], WASTE_XY[1], TOP + 0.100), o, "up", rot=_grip(90))]

    # pad 3: fresh white final wipe
    right += _pick(PAD_XY[2], pad_handle_z, grip)
    right += [Waypoint(1.8, (0.270, 0.000, TOP + 0.155), grip, "over the pan", rot=_grip(90)),
              Waypoint(1.2, (0.270, 0.000, pad_press_z + 0.030), grip, "lower", rot=_grip(90)),
              Waypoint(0.8, (0.270, 0.000, pad_press_z), grip, "fresh pad on the patch", rot=_grip(90)),
              Waypoint(0.5, (0.270, 0.000, pad_press_z - 0.0005), grip, "press", rot=_grip(90)),
              Waypoint(0.7, (0.285, 0.000, pad_press_z - 0.0005), grip, "final wipe across", rot=_grip(90)),
              Waypoint(0.7, (0.255, 0.000, pad_press_z - 0.0005), grip, "final wipe back", rot=_grip(90)),
              Waypoint(0.6, (0.270, 0.000, pad_press_z - 0.0005), grip, "finish the final wipe", rot=_grip(90)),
              Waypoint(1.0, (0.270, 0.000, TOP + 0.155), grip, "lift the final pad", rot=_grip(90)),
              Waypoint(1.8, (WASTE_XY[0], WASTE_XY[1], TOP + 0.100), grip, "over the waste", rot=_grip(90)),
              Waypoint(1.0, (WASTE_XY[0], WASTE_XY[1], pad_handle_z + 0.010), grip, "lower", rot=_grip(90)),
              Waypoint(0.5, (WASTE_XY[0], WASTE_XY[1], pad_handle_z + 0.010), o, "release the final pad", rot=_grip(90)),
              Waypoint(0.8, (WASTE_XY[0], WASTE_XY[1], TOP + 0.100), o, "up", rot=_grip(90))]

    right += [Waypoint(6.0, (_START_TCP["right"][0], _START_TCP["right"][1] - 0.06, TOP + 0.240), o,
                       "let the base dry", rot=_START_ROT["right"]),
              _home("right", 1.5),
              Waypoint(2.0, tuple(_START_TCP["right"]), o, "hold", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    f = _RES.fraction()
    return (f"clean {_RES.clean()}/{NC} ({f * 100:.1f}%), wet {_RES.wet_max():.2f}, soft {int(_RES.soft.sum())}, "
            f"load {_RES.load['pad1']:.3f}/{_RES.load['pad2']:.3f}/{_RES.load['pad3']:.3f} g, "
            f"outside {_RES.outside:.2f}, scratch {_RES.scratch}, placed {_RES.placed}, wetted {_RES.wetted}")
