"""On the station table: a grey anti-static (metal-in static shielding) stand-up pouch, zipped shut, its bottom glued
to the table, with two yellow 18650 cells standing inside it; on the right, the bottom half of an empty 2x2 18650
battery holder. The left arm pulls the bag's zipper slider open and goes back; the right arm reaches into the opened bag,
takes one cell out and stands it positive end up in a cup of the holder, and goes back. The other cell stays in the bag.

The bag is one articulation from ``assets/`` pinned to the table by its bottom: the pouch's floor, lower walls and side
seals (the root), eight film panels per wall above them on sprung hinges that want the mouth open, each with its piece
of the zipper profile on top, and the slider on a damped prismatic joint along the zipper. The zipper's interlock is
the panels' hinge springs: every step (``_zip_lips``) each panel's spring target follows the slider, shut until the
slider's centre reaches the panel and open once it has run over it, so the mouth opens behind the slider (and would
shut again if the slider were pushed back). The holder and each cell are their own rigid bodies.

Run it:  inspect-robots run --embodiment yam_arena -E task=antistatic_bag.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

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
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it first holds
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the bag (its own frame: origin at the bin's underside centre, x along the mouth, the front wall at -y, z up) -------
POUCH_FLOOR = 0.0005  # the pouch's gusset floor (glued to the table), where the cells stand
ZIP_TOP = 0.099  # the zipper's top: the bag's mouth
ZIP_PARK_X = -0.0385  # the slider's centre at its end stop at -x, shut
ZIP_TRAVEL = 0.077  # ... to its end stop at +x, open
SLIDER_Z0 = 0.0974  # the slider's side plates reach down to here, and it is 7.8 mm across the zipper
SLIDER_WIDTH = 0.0078
SLIDER_HALF_LENGTH = 0.0065  # its trailing end (with the plates' rounded noses) this far behind its centre
POUCH_HALF_WIDTH = 0.045  # the side seals' inner edges
POUCH_HALF_THICK = 0.017  # the pouch's widest, at its floor
N_PANELS = 8
# The panels' edges along the mouth (m) and open angles (deg: a lens-shaped mouth), as built; the
# front panels open + about x, the back ones -.
PANEL_EDGES = (-0.045, -0.0295, -0.019667, -0.009833, 0.0, 0.009833, 0.019667, 0.0295, 0.045)
PANEL_OPEN_DEG = (21.0, 24.0, 25.0, 26.0, 26.0, 25.0, 24.0, 21.0)
HINGES = [f"Hinge_F{k}" for k in range(N_PANELS)] + [f"Hinge_B{k}" for k in range(N_PANELS)]
SLIDE = "Slide"

# -- the cells and the holder ------------------------------------------------------------------------------------------
CELL_RADIUS = 0.009
CELL_HALF = 0.03325  # 18650 cell with its positive nub: 66.5 mm
CELLS_IN_BAG = (-0.016, 0.016)  # along the mouth, standing on the floor, positive ends up
# The holder: the bottom half of a 2x2 18650 holder (41.2 x 41.2 x 9.5 mm, 20 mm pitch), origin at its bottom face.
CELL_PITCH = 0.020
CELL_SEATED_Z = 0.03475  # a cell's centre above the holder's bottom face, seated on its ledge
HOLDER_TOP = 0.0095
CUPS = ((-1, -1), (1, -1), (-1, 1), (1, 1))  # in half-pitch units, in the holder's frame

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The bag stands with its mouth along y (its +x toward the robot's left), so the left arm pulls the slider toward
# itself; cell_1 (nearer the right arm) is the one taken out.
BAG_XY, BAG_YAW = (0.26, -0.02), math.pi / 2
HOLDER_XY = (0.30, -0.16)
HOLDER_FIXED = True  # taped to the table: loose, the 6.5 g holder and a cell standing in it kept rattling (~50 mm/s)
TARGET_CUP = 2  # CUPS[2]: the cup nearest the bag and the robot


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


BAG_POS = (*BAG_XY, TOP)
_BAG_R = _rz(BAG_YAW)
HOLDER_POS = (*HOLDER_XY, TOP + 0.0002)
CELL_POS = tuple(np.array(BAG_POS) + _BAG_R @ np.array([x, 0.0, POUCH_FLOOR + CELL_HALF + 0.0002])
                 for x in CELLS_IN_BAG)


def in_bag(p) -> np.ndarray:
    """A world point in the bag's frame (the bag never moves: its bin is screwed to the table)."""
    return _BAG_R.T @ (np.asarray(p, float) - np.array(BAG_POS))


def slider_world(travel: float) -> np.ndarray:
    """The slider's centre on the zipper's top (world) at ``travel`` (m from its shut end stop)."""
    return np.array(BAG_POS) + _BAG_R @ np.array([ZIP_PARK_X + travel, 0.0, ZIP_TOP])


def cup_seat(k: int = TARGET_CUP, holder_pos=HOLDER_POS, holder_quat=(0.0, 0.0, 0.0, 1.0)) -> np.ndarray:
    i, j = CUPS[k]
    return np.asarray(holder_pos, float) + quat_xyzw_to_matrix(np.asarray(holder_quat, float)) @ np.array(
        [i * CELL_PITCH / 2, j * CELL_PITCH / 2, CELL_SEATED_Z])


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation
    (``fixed_base`` pins its root link where it spawns). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed_base: bool = False
    kinematic: bool = False  # a rigid body that never moves (taped to the table)

    @property
    def free(self) -> bool:
        return not (self.fixed_base or self.kinematic)


# Arena places an articulation by its root link: the bag's is its lower part (origin at its floor's centre, on the table).
BAG = UsdPart("bag", asset="bag", xy=BAG_XY, extent=0.1, mass=0.0036, usd_path=str(ASSETS / "antistatic_bag.usda"),
              pos=BAG_POS, rot_xyzw=_yaw_xyzw(BAG_YAW), articulation=True, fixed_base=True)
HOLDER = UsdPart("holder", asset="holder", xy=HOLDER_XY, extent=0.042, mass=0.0065,
                 usd_path=str(ASSETS / "holder_18650_2x2.usda"), pos=HOLDER_POS, kinematic=HOLDER_FIXED)
CELLS = [UsdPart(f"cell_{k + 1}", asset="cell", xy=tuple(float(v) for v in p[:2]), extent=0.0665, mass=0.047,
                 usd_path=str(ASSETS / "battery_18650_yellow.usda"), pos=tuple(float(v) for v in p))
         for k, p in enumerate(CELL_POS)]
OBJECTS = [BAG, HOLDER, *CELLS]

SOLVER_ITERATIONS = 32  # position iterations per step for the bag's articulation (0.15 g panels, a 0.9 g slider)
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=spec.fixed_base or None, enabled_self_collisions=False,
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): the span of its finger pads along its pointing axis (``back``
    to the pad ``tips``), how far apart the pads are, and the smaller of the two fingers' grip forces (N): each
    finger's drive stiffness times how far something holds it back from its commanded position."""

    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float


PAD_BACK = 0.025  # m: the pads reach this far behind the TCP


def _gripper_states(env) -> dict:
    """Each gripper's ``GripperState`` and the episode's time (s), keyed ``left_gripper``, ``right_gripper`` and
    ``episode_time``."""
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        axis = runtime.link_poses(robot)["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)  # fingers open to -q each; a closing finger is held back
        out[f"{side}_gripper"] = GripperState(
            back=tcp - PAD_BACK * axis,
            tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
        )
    return out


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root (Arena's geometry queries take rigid objects only; the bag's bounds are left at its
        root), plus the grippers, the episode time, the slider's travel and the panels' hinge angles."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            data = base.scene[name].data

            def arr(t):
                return t.torch[0].detach().cpu().numpy().astype(np.float64)

            pose = arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=arr(data.root_lin_vel_w),
                                    ang_vel=arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        bag = base.scene["bag"]
        ids = bag.find_joints([SLIDE] + HINGES, preserve_order=True)[0]
        q = bag.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
        out["slider_travel"] = float(q[0])  # m from the shut end stop
        out["panel_angles"] = q[1:].copy()  # rad: front panels + open, back panels - open
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that shuts the bag (Arena never writes an
        articulation's joint angles at reset), and an event every step that makes the panels' hinge targets follow
        the slider (the zipper's interlock)."""
        configure = stock_cfg_callback(control_hz)

        def with_bag(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.bag_shut = EventTermCfg(func=_shut_bag, mode="reset",
                                                   params={"asset_cfg": SceneEntityCfg("bag")})
            env_cfg.events.bag_zipper = EventTermCfg(func=_zip_lips, mode="interval", interval_range_s=(0.0, 0.0),
                                                     params={"asset_cfg": SceneEntityCfg("bag")})
            return env_cfg

        return with_bag

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


def panel_targets(travel: float) -> np.ndarray:
    """The panels' hinge targets (rad, ``HINGES`` order) for the slider at ``travel``: each panel opens as the slider's
    trailing end runs over it, from its -x edge to its +x edge, and shuts again if the slider comes back. (At the far
    end stop the last panel, still under the slider, stays nearly shut.)"""
    x = ZIP_PARK_X + travel - SLIDER_HALF_LENGTH
    lo, hi = np.array(PANEL_EDGES[:-1]), np.array(PANEL_EDGES[1:])
    opened = np.radians(PANEL_OPEN_DEG) * np.clip((x - lo) / (hi - lo), 0.0, 1.0)
    return np.concatenate([opened, -opened])


def _write_targets(bag, names, values) -> None:
    """Writes joint drive targets (rows per env, ``names`` order) to PhysX directly: Arena gives task objects no
    actuators, so Isaac Lab's own joint targets never reach the simulator."""
    import torch

    view = bag.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    dofs = list(view.shared_metatype.dof_names)
    cols = [dofs.index(n) for n in names]
    t[:, cols] = torch.as_tensor(np.asarray(values, np.float32), device=t.device)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, bag._ALL_INDICES)


def _zip_lips(env, env_ids, asset_cfg) -> None:
    """Interval event (every step): the panels' hinge targets follow the slider's travel."""
    bag = env.scene[asset_cfg.name]
    s_id = bag.find_joints([SLIDE])[0]
    travel = bag.data.joint_pos.torch[:, s_id].detach().cpu().numpy()[:, 0]
    _write_targets(bag, HINGES, np.stack([panel_targets(float(s)) for s in travel]))


def _shut_bag(env, env_ids, asset_cfg) -> None:
    """Reset event: the slider at its shut end stop and every panel shut, at rest, their targets shut."""
    import torch

    if env_ids is None:
        return
    bag = env.scene[asset_cfg.name]
    ids = bag.find_joints([SLIDE] + HINGES, preserve_order=True)[0]
    zero = torch.zeros((len(env_ids), len(ids)), device=bag.device)
    bag.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    bag.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
    _write_targets(bag, [SLIDE] + HINGES, np.zeros((bag.num_instances, 1 + len(HINGES))))


class _AfterSceneImport(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` right after it is first imported (it needs Isaac Sim, which starts later)."""

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


# -- scoring ----------------------------------------------------------------------------------------------------------
UNZIPPED = 0.9  # the slider pulled at least this fraction of its travel: the bag is open
LEFT_UPRIGHT = math.radians(30)  # a cell left in the bag stands within this of upright
SEATED_OFF = 0.003  # m: a cell's centre within this of a cup's seat, and
UPRIGHT = math.radians(10)  # ... its positive end up within this
CLEAR_ABOVE = 0.005  # m: a held cell's lowest point this far above the bag's mouth is out of the bag
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
CELL_BOX = (np.array([-CELL_RADIUS, -CELL_RADIUS, -CELL_HALF]), np.array([CELL_RADIUS, CELL_RADIUS, CELL_HALF]))
CELL_NAMES = tuple(c.name for c in CELLS)
STEPS = ("bag unzipped", "battery taken out of the bag", "battery in the holder", "other battery left in the bag")
POINTS = 0.2  # per step; only success scores 1.0
STILL_S = 1.0  # s: success needs every full-success condition held without a break this long, among them
STILL_SPEED, STILL_SPIN = 0.01, 0.5  # ... the seated cell and the holder still: under 1 cm/s and 0.5 rad/s
TRACE_S = 1.0  # s between trace lines in a run (0: none)


@dataclass(frozen=True)
class PartPose:
    pos: np.ndarray
    quat_xyzw: np.ndarray


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose``."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ quat_xyzw_to_matrix(pose.quat_xyzw).T + pose.pos
    return w.min(0), w.max(0)


def _held(bounds, gripper) -> bool:
    """``gripper`` holds the part with world ``bounds``: its pads reach them and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = bounds[0] - GRIP_SLACK, bounds[1] + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def _held_by_any(objects, bounds) -> bool:
    return any(_held(bounds, objects.get(g)) for g in ("left_gripper", "right_gripper"))


def _cell_held(objects, name) -> bool:
    return _held_by_any(objects, _box(objects[name], CELL_BOX))


def _cell_bottom(cell) -> float:
    """The cell's lowest point (m, world z)."""
    axis_z = abs(float(quat_xyzw_to_matrix(cell.quat_xyzw)[2, 2]))
    return float(cell.pos[2] - CELL_HALF * axis_z - CELL_RADIUS * math.sqrt(max(0.0, 1.0 - axis_z ** 2)))


def _mouth_z(objects) -> float:
    return float(objects["bag"].pos[2] + ZIP_TOP)


def cell_in_bag(objects, name) -> bool:
    """The cell's centre lies inside the pouch (below its mouth, within its outline)."""
    p = in_bag(objects[name].pos - (np.asarray(objects["bag"].pos) - np.array(BAG_POS)))
    return bool(abs(p[0]) < POUCH_HALF_WIDTH and abs(p[1]) < POUCH_HALF_THICK and POUCH_FLOOR < p[2] < ZIP_TOP)


def cell_taken_out(objects) -> bool:
    """A gripper holds a cell lifted clear of the bag: its lowest point above the mouth."""
    mouth = _mouth_z(objects)
    return any(_cell_bottom(objects[n]) > mouth + CLEAR_ABOVE and _cell_held(objects, n) for n in CELL_NAMES)


def cell_in_cup(objects, name):
    """How far the cell's centre is from the nearest cup's seat (m) and the angle of its positive end from the holder's
    up (rad)."""
    h = objects["holder"]
    c = objects[name]
    off = min(float(np.linalg.norm(c.pos - cup_seat(k, h.pos, h.quat_xyzw))) for k in range(len(CUPS)))
    up = quat_xyzw_to_matrix(h.quat_xyzw)[:, 2]
    return off, math.acos(float(np.clip(quat_xyzw_to_matrix(c.quat_xyzw)[:, 2] @ up, -1.0, 1.0)))


def seated_cell(objects) -> str | None:
    """A cell standing in a cup of the holder, positive end up, not held, if any."""
    for name in CELL_NAMES:
        off, angle = cell_in_cup(objects, name)
        if off < SEATED_OFF and angle < UPRIGHT and not _cell_held(objects, name):
            return name
    return None


def cell_left_in_bag(objects) -> bool:
    """A cell still stands in the bag (its centre inside the pouch, upright within LEFT_UPRIGHT), not held."""
    return any(cell_in_bag(objects, n) and quat_xyzw_to_matrix(objects[n].quat_xyzw)[2, 2] > math.cos(LEFT_UPRIGHT)
               and not _cell_held(objects, n) for n in CELL_NAMES)


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode, updated once per control step: whether the bag has been unzipped and a
    cell taken out of it, and since when every full-success condition (``conditions_met``) has held without a break
    (``None`` while one fails: any lapse, such as the seated cell moving, restarts the count). Starts over when the
    episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.unzipped = False
        self.taken_out = False
        self.met_since = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.unzipped |= bool(objects["slider_travel"] >= UNZIPPED * ZIP_TRAVEL)
        self.taken_out |= cell_taken_out(objects)
        if t is None:  # no clock: nothing can be timed
            return
        met = conditions_met(objects, self.unzipped, self.taken_out)
        if met and self.met_since is None:
            self.met_since = t
            print(f"[antistatic_bag] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)
        elif not met and self.met_since is not None:
            self.met_since = None
            print(f"[antistatic_bag] t={t:.2f}s  success conditions lapsed: timing restarts", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: steps 1 and 2 once reached; 3 and 4 as things stand. Steps 3 and 4 count only once a cell
    has been taken out of the bag by a gripper."""
    p = _PROGRESS
    p.update(objects)
    seated = seated_cell(objects)
    done = [bool(p.unzipped), bool(p.taken_out), bool(p.taken_out and seated is not None),
            bool(p.taken_out and cell_left_in_bag(objects))]
    t = objects.get("episode_time")
    if t is not None and TRACE_S and round(t / 0.05) % round(TRACE_S / 0.05) == 0 and t != p.traced:
        p.traced = t
        c1, c2, h = objects["cell_1"].pos, objects["cell_2"].pos, objects["holder"].pos
        print(f"[antistatic_bag] t={t:.2f}s  trace: slider {objects['slider_travel'] * 1000:.1f} mm; panels front "
              f"{' '.join(f'{a:.0f}' for a in np.degrees(objects['panel_angles'][:N_PANELS]))} deg; cell_1 "
              f"({c1[0]:.4f}, {c1[1]:.4f}, {c1[2] - TOP:.4f}) cell_2 ({c2[0]:.3f}, {c2[1]:.3f}, {c2[2] - TOP:.3f}) "
              f"holder ({h[0]:.4f}, {h[1]:.4f})"
              + "".join(f"; {sd[0].upper()} tips ({g.tips[0]:.3f}, {g.tips[1]:.3f}, {g.tips[2] - TOP:.3f}) open "
                        f"{g.opening * 1000:.0f} mm {g.contact:.0f} N" for sd in ("left", "right")
                        if (g := objects.get(f"{sd}_gripper")) is not None)
              + f"; home L {_home(objects, 'left')} R {_home(objects, 'right')} v cell_1 "
              f"{np.linalg.norm(objects['cell_1'].lin_vel) * 1000:.0f} mm/s {np.linalg.norm(objects['cell_1'].ang_vel):.2f} "
              f"rad/s holder {np.linalg.norm(objects['holder'].lin_vel) * 1000:.0f} mm/s", flush=True)
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        off = min(cell_in_cup(objects, n)[0] for n in CELL_NAMES)
        ang = np.degrees(objects["panel_angles"])
        print(f"[antistatic_bag] t={objects['episode_time']:.2f}s  steps done: {got}  (slider "
              f"{objects['slider_travel'] * 1000:.1f} mm; panels front {' '.join(f'{a:.0f}' for a in ang[:N_PANELS])} "
              f"deg; nearest cell {off * 1000:.1f} mm from a cup)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: a cell or the holder off the table, or a state that is not finite."""
    for name in (*CELL_NAMES, "holder"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return not (np.isfinite(objects["slider_travel"]) and np.all(np.isfinite(objects["panel_angles"])))


def conditions_met(objects, unzipped: bool, taken_out: bool) -> bool:
    """Every full-success condition, as things stand: all four steps done, both arms back where they started with
    nothing in hand, the seated cell and the holder still (under ``STILL_SPEED`` and ``STILL_SPIN``), and no
    violation."""
    seated = seated_cell(objects)
    return bool(unzipped and taken_out and seated is not None and cell_left_in_bag(objects)
                and not _violation(objects) and _home(objects, "left") and _home(objects, "right")
                and objects[seated].resting(STILL_SPEED, STILL_SPIN)
                and objects["holder"].resting(STILL_SPEED, STILL_SPIN))


def success(objects) -> bool:
    """Every full-success condition (``conditions_met``) has held without a break for ``STILL_S``, counted from the
    first control step it held: the seated cell still for that whole second, both arms home, all four steps."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for ``STILL_S``); otherwise 0.2 per step done (``STEPS``, at
    most 0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from above, fingers across the zipper on the slider's side plates; pull it to the far end stop (toward the
# left arm), let go and go home.
# Right arm: once the bag is open, from above, fingers closing along the mouth around cell_1's top; lift it straight
# out, carry it over the holder's cup and lower it in, positive end up; let go just above its seat; go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (~20 N a finger)
SLIDER_HOLD = -(SLIDER_WIDTH - SQUEEZE / 2) / 2  # ~10 N a finger on the 0.9 g slider
SLIDER_OPEN = -0.012  # the fingers 24 mm apart over the slider
SLIDER_TIPS_UP = 0.0015  # the pad tips this far above the slider body's bottom
PULL_PAST = 0.001  # m: pulled this far past the end stop (the stop holds the slider; the pads slip a little)
CELL_HOLD = -(2 * CELL_RADIUS - SQUEEZE) / 2  # ~20 N a finger
CELL_OPEN = -0.0145  # the fingers 29 mm apart around the cell: clear of the other cell and the end panels
GRIP_BELOW_TOP = 0.008  # the pad tips this far below the cell's top (the gripper widens to 40 mm 14 mm behind its TCP)
RELEASE_ABOVE = -0.0005  # m: the cell set down on its seat, pressed 0.5 mm, and let go upright (dropped even 1.5 mm, it
# landed leaning in its 8 mm cup and kept rocking)
LIFT = 0.115  # the cell lifted this far out of the bag (its bottom 16 mm above the mouth)
CARRY = 0.06  # its bottom this far above the holder's top while carried over


@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None  # a full gripper (TCP) rotation; replaces ``yaw`` when set


def _target(self, t: float):
    """``ArmPlan.target`` that also follows a waypoint's ``rot``."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (
                prev_pos + s * (goal - prev_pos),
                ik.slerp_matrix(prev_rot, rot, s),
                prev_finger + s * (w.finger - prev_finger),
                w.label,
            )
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target

# The left gripper pinches the slider across the zipper, its pads' width along it, leaned SLIDER_LEAN about its closing
# axis toward the left arm (what its wrist reaches); the right gripper points straight down into the mouth and closes
# along it.
_X = np.array([1.0, 0.0, 0.0])
SLIDER_LEAN, CELL_LEAN = math.radians(20), 0.0
SLIDER_ROT = axis_angle_matrix(_X, -SLIDER_LEAN) @ yam_demo.grip_rotation(BAG_YAW - math.pi)
CELL_ROT = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -CELL_LEAN) @ yam_demo.grip_rotation(BAG_YAW - math.pi / 2)


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ np.array([0.0, 0.0, 1.0]))


def slider_tcp(travel: float) -> np.ndarray:
    """The left TCP pinching the slider at ``travel``: the pads' tips SLIDER_TIPS_UP above its plates' bottom, at its
    middle."""
    s = slider_world(travel)
    return _tcp_with_tips((s[0], s[1], TOP + SLIDER_Z0 + SLIDER_TIPS_UP), 0.0, SLIDER_ROT)


def cell_tcp(centre) -> np.ndarray:
    """The right TCP holding a cell (upright) whose centre is at ``centre``: the pads' tips on its axis GRIP_BELOW_TOP
    below its top."""
    c = np.asarray(centre, float)
    return _tcp_with_tips((c[0], c[1], c[2] + CELL_HALF - GRIP_BELOW_TOP), 0.0, CELL_ROT)


def left_plan(hold_s: float) -> list[Waypoint]:
    """Unzip the bag, go home and wait ``hold_s`` s."""
    rot = SLIDER_ROT
    at_shut, opened = slider_tcp(0.0), slider_tcp(ZIP_TRAVEL + PULL_PAST)
    up = np.array([0.0, 0.0, 0.05])
    return [
        Waypoint(2.5, at_shut + up, SLIDER_OPEN, "over the slider", rot=rot),
        Waypoint(1.5, at_shut, SLIDER_OPEN, "down to the slider", rot=rot),
        Waypoint(0.8, at_shut, SLIDER_HOLD, "pinch the slider", rot=rot),
        Waypoint(UNZIP_S, opened, SLIDER_HOLD, "unzip", rot=rot),
        Waypoint(0.8, opened, SLIDER_OPEN, "let go", rot=rot),
        Waypoint(1.0, opened + up, SLIDER_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(hold_s, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


UNZIP_S = 4.0
LEFT_UNZIPPED_S = 2.5 + 1.5 + 0.8 + UNZIP_S + 0.8 + 1.0 + 2.5  # the left arm has unzipped the bag and is home


def right_plan() -> list[Waypoint]:
    """Wait for the bag to open, take cell_1 out, stand it in the holder's cup, go home."""
    rot = CELL_ROT
    at = cell_tcp(CELL_POS[0])
    lifted = at + (0, 0, LIFT)
    seat = cup_seat()
    release = cell_tcp(seat + (0, 0, RELEASE_ABOVE))
    carried = cell_tcp(seat + (0, 0, HOLDER_TOP + CARRY + CELL_HALF - CELL_SEATED_Z))
    return [
        Waypoint(LEFT_UNZIPPED_S, _START_TCP["right"], FINGER_OPEN, "wait for the bag", rot=_START_ROT["right"]),
        Waypoint(2.5, at + (0, 0, 0.07), CELL_OPEN, "over the opened bag", rot=rot),
        Waypoint(2.0, at, CELL_OPEN, "reach in around the cell", rot=rot),
        Waypoint(0.8, at, CELL_HOLD, "grip the cell", rot=rot),
        Waypoint(2.5, lifted, CELL_HOLD, "lift it out", rot=rot),
        Waypoint(3.0, carried, CELL_HOLD, "over the holder", rot=rot),
        Waypoint(1.5, release + (0, 0, 0.012), CELL_HOLD, "over the cup", rot=rot),
        Waypoint(1.0, release + (0, 0, 0.012), CELL_HOLD, "settle", rot=rot),
        Waypoint(2.5, release, CELL_HOLD, "into the cup", rot=rot),
        Waypoint(0.5, release, CELL_HOLD, "still", rot=rot),
        Waypoint(1.5, release, CELL_OPEN, "let go", rot=rot),
        Waypoint(1.0, release + (0, 0, 0.06), CELL_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The left arm unzips the bag and goes home; the right takes a cell out and stands it in the holder, and goes
    home."""
    right = right_plan()
    right.append(Waypoint(2.0, _START_TCP["right"], FINGER_OPEN, "wait", rot=_START_ROT["right"]))
    left = left_plan(0.0)
    left[-1].duration = sum(w.duration for w in right) - sum(w.duration for w in left[:-1])
    return {"left": left, "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
