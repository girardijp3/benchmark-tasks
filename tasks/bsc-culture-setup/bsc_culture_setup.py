"""On the station table, a biosafety-cabinet work zone (x 0.25 to 0.55): a slotted front grille (x 0.25 to 0.35), a slotted
rear grille (x 0.50 to 0.55), a sash raised to a 380 mm opening, a back wall. Three items stand in a jumble on the front grille: a
250 mL media bottle, a pipette tip box and a waste pan. Cabinet practice: work 10 cm in from the front grille, keep the rear
grille clear, 3 cm between items, clean items on one side and waste on the other.

Left arm: takes the media bottle and then the tip box to the clean (left) side; right arm: takes the waste pan to the dirty (right)
side. The cabinet is a fixed body from ``assets/cabinet.usda``; each item a rigid body.

Run it:  inspect-robots run --embodiment yam_arena -E task=bsc_culture_setup.py --scorer task_score ...

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
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "bsc_culture_setup"
REST_GAP = 0.0003

# -- the cabinet (m, world): work surface x 0.25-0.55; front grille 0.25-0.35; rear grille 0.50-0.55; sash 380 mm above
FRONT_IN = 0.35  # an item's footprint must lie at x >= this (10 cm in from the front edge)
REAR_OUT = 0.50  # ... and at x <= this (the rear grille stays clear)
SPACING = 0.03  # items' footprints at least this far apart
Y_LIM = 0.28  # inside the cabinet's width

# -- the items: footprint half extents (x, y, local, with the offset of the footprint's middle), start and clean/waste side
ITEMS = {
    # name: (x0, x1, y0, y1) footprint in the item's frame (m), start xy, side
    "bottle": ((-0.035, 0.035, -0.035, 0.035), (0.29, 0.11), "clean"),
    "tips": ((-0.0635, 0.0635, -0.0415, 0.0415), (0.29, 0.24), "clean"),
    "pan": ((-0.040, 0.040, -0.065, 0.065), (0.29, -0.10), "waste"),
}
NAMES = tuple(ITEMS)
PLAN_XY = {"bottle": (0.425, 0.06), "tips": (0.425, 0.22), "pan": (0.425, -0.12)}  # where the demo puts them

_X, _Y, _Z = np.eye(3)


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self) -> bool:
        return not self.fixed


CABINET = UsdPart("cabinet", asset="cabinet", xy=(0.45, 0.0), extent=0.6, mass=20.0, usd_path=str(ASSETS / "cabinet.usda"),
                  pos=(0.0, 0.0, TOP), fixed=True)
_FILES = {"bottle": "media_bottle.usda", "tips": "tip_box.usda", "pan": "waste_pan.usda"}
_MASS = {"bottle": 0.30, "tips": 0.12, "pan": 0.25}
_EXT = {"bottle": 0.07, "tips": 0.127, "pan": 0.13}
PARTS = {k: UsdPart(k, asset=k, xy=ITEMS[k][1], extent=_EXT[k], mass=_MASS[k], usd_path=str(ASSETS / _FILES[k]),
                    pos=(*ITEMS[k][1], TOP + REST_GAP)) for k in NAMES}
OBJECTS = [CABINET, *PARTS.values()]

SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=None)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                  rotation_xyzw=tuple(float(v) for v in spec.rot_xyzw)),
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
        """The panel's and the jack's roots, the grippers, the episode time and the jack's two tab angles."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}

        def arr(t):
            return t.torch[0].detach().cpu().numpy().astype(np.float64)

        for name in names:
            data = base.scene[name].data
            pose = arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=arr(data.root_lin_vel_w),
                                    ang_vel=arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        return out

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states


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

# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


# -- scoring ----------------------------------------------------------------------------------------------------------
UPRIGHT = math.radians(10.0)  # an item within this of upright
FALLEN = math.radians(45.0)  # tilted this far, an item has fallen (scores 0)
CLEAR = 0.03  # m: both grippers at least this far from every item
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
STEPS = ("media bottle in the cabinet, clean side", "tip box in the cabinet, clean side", "waste pan in the cabinet, dirty side",
         "all three laid out with 3 cm between them, let go of everything")
POINTS = 0.2


def _rot(q) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(q, float))


def _pose(objects, name):
    o = objects[name]
    return np.asarray(o.pos, float), _rot(o.quat_xyzw)


def tilt(objects, name) -> float:
    _, r = _pose(objects, name)
    return math.acos(max(-1.0, min(1.0, float(r[2, 2]))))


def footprint(objects, name) -> np.ndarray:
    """The item's footprint corners (4 x 2) in the world."""
    pos, r = _pose(objects, name)
    x0, x1, y0, y1 = ITEMS[name][0]
    return np.array([(pos + r @ np.array([x, y, 0.0]))[:2] for x in (x0, x1) for y in (y0, y1)])


def placed(objects, name) -> bool:
    """Upright on the work surface with the whole footprint at x between FRONT_IN and REAR_OUT, inside the cabinet's width, and
    on its side of the middle: clean items at y > 0, waste at y < 0."""
    pos, _ = _pose(objects, name)
    c = footprint(objects, name)
    side_ok = bool(np.all(c[:, 1] > 0.0)) if ITEMS[name][2] == "clean" else bool(np.all(c[:, 1] < 0.0))
    return bool(np.all(c[:, 0] >= FRONT_IN) and np.all(c[:, 0] <= REAR_OUT) and np.all(np.abs(c[:, 1]) <= Y_LIM)
                and side_ok and tilt(objects, name) <= UPRIGHT and abs(pos[2] - (TOP + REST_GAP)) <= 0.004)


def spaced(objects) -> bool:
    """Every two items' footprints (as boxes) at least SPACING apart."""
    boxes = {n: (footprint(objects, n).min(axis=0), footprint(objects, n).max(axis=0)) for n in NAMES}
    for i, a in enumerate(NAMES):
        for b in NAMES[i + 1:]:
            gx = max(boxes[a][0][0] - boxes[b][1][0], boxes[b][0][0] - boxes[a][1][0])
            gy = max(boxes[a][0][1] - boxes[b][1][1], boxes[b][0][1] - boxes[a][1][1])
            if max(gx, gy) < SPACING:
                return False
    return True


def _pads(g) -> list[np.ndarray]:
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def let_go(objects) -> bool:
    """No pad point within CLEAR of any item's footprint box (its height included)."""
    for g in ("left_gripper", "right_gripper"):
        gr = objects.get(g)
        if gr is None:
            continue
        for p in _pads(gr):
            for n in NAMES:
                c = footprint(objects, n)
                lo, hi = c.min(axis=0), c.max(axis=0)
                if (lo[0] - CLEAR <= p[0] <= hi[0] + CLEAR and lo[1] - CLEAR <= p[1] <= hi[1] + CLEAR
                        and TOP - CLEAR <= p[2] <= TOP + 0.15 + CLEAR):
                    return False
    return True


def _poses(objects) -> tuple:
    out = []
    for n in NAMES:
        o = objects[n]
        out += [np.array(o.pos, float), np.array(o.quat_xyzw, float)]
    return tuple(out)


def _turn(q0, q1) -> float:
    return 2 * math.acos(min(1.0, abs(float(np.dot(q0, q1)))))


def still(objects, anchor) -> bool:
    now = _poses(objects)
    return all(np.linalg.norm(now[i] - anchor[i]) <= STILL_MOVE and _turn(anchor[i + 1], now[i + 1]) <= STILL_TURN
               for i in range(0, len(now), 2))


class _Progress:
    """The episode so far, updated once per control step: whether an item fell, and since when every full-success condition
    has held (``met_since``, ``anchor``)."""

    def __init__(self):
        self.time = -1.0
        self.fell = False
        self.met_since = None
        self.anchor = None
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
        self.fell |= any(tilt(objects, n) > FALLEN for n in NAMES)
        if t is None:
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): timing restarts",
                  flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    a, b, c = (placed(objects, n) for n in NAMES)
    return [a, b, c, bool(a and b and c and spaced(objects) and let_go(objects))]


def steps_done(objects) -> list[bool]:
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = "(" + "; ".join(f"{n} at x {objects[n].pos[0]:.3f} y {objects[n].pos[1]:.3f}" for n in NAMES) + f"; spaced {spaced(objects)})"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: an item fallen (now or earlier), any item off the table, a non-finite state."""
    values = [v for n in NAMES for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.fell or any(objects[n].pos[2] < TOP - 0.03 for n in NAMES))


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every condition held for STILL_S); otherwise 0.2 per step done (at most 0.8); 0 for a violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
SQUEEZE = 0.008  # the fingers close this far inside the grip width (about 20 N a finger)
HOVER = {"bottle": 0.11, "tips": 0.06, "pan": 0.06}  # the TCP hovers this far above its grip (the pads clear the bottle's cap)
CARRY = {"bottle": 0.030, "tips": 0.045, "pan": 0.045}  # the items' bases pass this far above the table
AWAY = 0.05  # after letting go the TCP rises this far
ROT_Y = yam_demo.grip_rotation(0.0)  # straight down, fingers closing along y
ROT_X = yam_demo.grip_rotation(math.pi / 2)  # fingers closing along x
# grip: (rotation, width across the fingers, TCP height above the base)
GRIPS = {"bottle": (ROT_Y, 0.070, 0.065), "tips": (ROT_Y, 0.083, 0.045), "pan": (ROT_X, 0.080, 0.050)}

_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


def move_item(name, wait_s=0.0):
    rot, width, up = GRIPS[name]
    open_, hold = -(width + 0.022) / 2, -(width - SQUEEZE) / 2
    sx, sy = ITEMS[name][1]
    tx, ty = PLAN_XY[name]
    z0 = TOP + REST_GAP + up
    zc = z0 + CARRY[name]
    w = [Waypoint(wait_s, None, FINGER_OPEN, "wait", rot=None)] if False else []
    w += [
        Waypoint(2.5, [sx, sy, z0 + HOVER[name]], open_, f"over the {name}", rot=rot),
        Waypoint(1.5, [sx, sy, z0], open_, "down on it", rot=rot),
        Waypoint(0.8, [sx, sy, z0], hold, "grip", rot=rot),
        Waypoint(1.2, [sx, sy, zc], hold, "lift it clear", rot=rot),
        Waypoint(3.0, [tx, ty, zc], hold, "carry it back into the cabinet", rot=rot),
        Waypoint(1.0, [tx, ty, z0 + 0.002], hold, "set it down", rot=rot),
        Waypoint(0.6, [tx, ty, z0 + 0.002], open_, "let go", rot=rot),
        Waypoint(1.0, [tx, ty, z0 + AWAY], open_, "lift away", rot=rot),
        Waypoint(1.5, [tx - 0.09, ty, z0 + AWAY], open_, "back out toward the front", rot=rot),
    ]
    return w


def demo():
    """Left arm: the media bottle, then the tip box, to the clean side. Right arm: the waste pan to the dirty side."""
    left = move_item("bottle") + move_item("tips") + [Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"])]
    right = move_item("pan") + [Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}
