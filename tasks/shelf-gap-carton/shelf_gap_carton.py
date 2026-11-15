"""On the station table: an open shelf bay (an 18 mm melamine board 436 x 220 mm between two 140 mm end dividers, 400 mm
clear between them, a 3 mm back panel), fixed to the table. Four retail cartons stand in a row on the board, front
faces level, with small gaps between them and at both ends: a yellow 100-bag tea carton, a blue herbal-tea carton, a
red soup carton and a white tablet carton, in that order from the robot's right to its left. A green 80-bag tea carton
stands on the table in front of the shelf. The green carton is 74 mm wide and no single gap in the row is as wide as
that (the widest is 30 mm), but the free width in the row is 120 mm, so it fits only once the neighbouring cartons are
slid together.

The left arm and the right arm make room and then shelve the green carton in the row between the blue and the red
carton, standing, with the four others still standing in their order. Only the final state is scored.

The shelf bay is one fixed rigid body and each carton is a rigid body, from ``assets/*.usda``.

Run it:  inspect-robots run --embodiment yam_arena -E task=shelf_gap_carton.py --scorer task_score ...

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
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "shelf_gap"

# -- the shelf (world frame; the bay's origin is the middle of its board, +x toward the back panel, +y the robot's left)
SHELF_XY = (0.42, 0.0)
BOARD_T = 0.018
BOARD_X, BOARD_Y = 0.22, 0.436
HALF_CLEAR = 0.200  # half the clear width between the dividers
ROW_FRONT = SHELF_XY[0] - BOARD_X / 2 + 0.045  # every carton's front face stands here (45 mm behind the board's lip)
SHELF_TOP = TOP + BOARD_T  # the board's top face

# -- the cartons (frame: origin at the middle of the base, x depth, y width, z up) --------------------------------------
# name: (width y, depth x, height z, mass, usd file)
CARTONS = {
    "yellow": (0.080, 0.060, 0.130, 0.24, "carton_tea100"),
    "blue": (0.062, 0.070, 0.105, 0.095, "carton_tea40"),
    "red": (0.078, 0.052, 0.120, 0.18, "carton_soup"),
    "white": (0.060, 0.045, 0.110, 0.06, "carton_tablets"),
    "green": (0.074, 0.062, 0.125, 0.20, "carton_tea80"),
}
ROW = ("yellow", "blue", "red", "white")  # from the robot's right (-y) to its left (+y)
ROW_Y = {"yellow": -0.140, "blue": -0.049, "red": 0.047, "white": 0.140}  # start centres (y)
GREEN_START = (0.22, -0.14)  # the green carton stands on the table here
FINAL_Y = {"blue": -0.065, "red": 0.093, "white": 0.168, "green": 0.010}  # where the demo leaves them
NAMES = tuple(CARTONS)

_X, _Y, _Z = np.eye(3)


def rz(a: float) -> np.ndarray:
    return axis_angle_matrix(_Z, a)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body or a fixture (``kinematic``).
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


def _carton(name: str) -> UsdPart:
    w, d, h, m, f = CARTONS[name]
    if name == "green":
        x, y, z = GREEN_START[0], GREEN_START[1], TOP + 0.0005
    else:
        x, y, z = ROW_FRONT + d / 2, ROW_Y[name], SHELF_TOP + 0.0005
    return UsdPart(name, asset=name, xy=(x, y), extent=h, mass=m, usd_path=str(ASSETS / f"{f}.usda"), pos=(x, y, z))


SHELF = UsdPart("shelf", asset="shelf", xy=SHELF_XY, extent=BOARD_Y, mass=6.0, usd_path=str(ASSETS / "shelf_bay.usda"),
                pos=(*SHELF_XY, TOP), kinematic=True)
OBJECTS = [SHELF] + [_carton(n) for n in NAMES]

MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): the span of its finger pads along its pointing axis (``back``
    to the pad ``tips``), how far apart the pads are, and the smaller of the two fingers' grip forces (N)."""

    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float


PAD_BACK = 0.025


def _gripper_states(env) -> dict:
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
        held_back = np.clip(target - q, 0.0, None)
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
        """Each object's root pose, the grippers and the episode time."""
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




# -- scoring ----------------------------------------------------------------------------------------------------------
UPRIGHT = math.radians(10.0)  # a carton stands within this of upright
BOARD_TOL = 0.006  # m: its base within this of the board's top face
FRONT_TOL = 0.020  # m: its front face within this of the row's front line
EDGE_TOL = 0.002  # m: its sides within this of the dividers' inner faces
CLEAR = 0.03  # m: both grippers at least this far from every carton
GAP_EXTRA = 0.004  # m: the opened gap is at least the green carton's width plus this
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
STEPS = ("gap between blue and red opened wide enough for the green carton",
         "green carton standing in the row between blue and red",
         "all five cartons standing on the board, within the dividers, in order",
         "that, with both hands clear")
POINTS = 0.2
GRIP_FORCE, GRIP_OPENING = 0.5, 0.010


def _rot(q) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(q, float))


def _tilt(objects, name) -> float:
    return math.acos(max(-1.0, min(1.0, float(_rot(objects[name].quat_xyzw)[2, 2]))))


def extent(objects, name):
    """(y0, y1, x0, x1): the carton's footprint extent along y and x in the world, whatever its yaw."""
    w, d, _, _, _ = CARTONS[name]
    r = _rot(objects[name].quat_xyzw)
    p = np.asarray(objects[name].pos, float)
    hy = abs(r[1, 0]) * d / 2 + abs(r[1, 1]) * w / 2
    hx = abs(r[0, 0]) * d / 2 + abs(r[0, 1]) * w / 2
    return p[1] - hy, p[1] + hy, p[0] - hx, p[0] + hx


def on_board(objects, name) -> bool:
    """Standing on the board (base on its top face, within 10 deg of upright), its front on the row's line and its
    sides inside the dividers."""
    p = np.asarray(objects[name].pos, float)
    y0, y1, x0, _ = extent(objects, name)
    return bool(abs(p[2] - SHELF_TOP) <= BOARD_TOL and _tilt(objects, name) <= UPRIGHT
                and abs(x0 - ROW_FRONT) <= FRONT_TOL
                and y0 >= SHELF_XY[1] - HALF_CLEAR - EDGE_TOL and y1 <= SHELF_XY[1] + HALF_CLEAR + EDGE_TOL)


def gap_width(objects) -> float:
    return extent(objects, "red")[0] - extent(objects, "blue")[1]


def gap_opened(objects) -> bool:
    return bool(on_board(objects, "blue") and on_board(objects, "red")
                and gap_width(objects) >= CARTONS["green"][0] + GAP_EXTRA)


def green_in_row(objects) -> bool:
    """The green carton stands on the board in the row, its centre between the blue and the red carton's centres (the
    cartons cannot overlap, so this is between them)."""
    if not (on_board(objects, "green") and on_board(objects, "blue") and on_board(objects, "red")):
        return False
    return bool(float(objects["blue"].pos[1]) < float(objects["green"].pos[1]) < float(objects["red"].pos[1]))


def _corners(objects, name) -> np.ndarray:
    w, d, _, _, _ = CARTONS[name]
    r = _rot(objects[name].quat_xyzw)
    p = np.asarray(objects[name].pos, float)
    return np.array([p[:2] + r[:2, 0] * sx * d / 2 + r[:2, 1] * sy * w / 2 for sx in (-1, 1) for sy in (-1, 1)])


def penetration(objects, a: str, b: str) -> float:
    """How deep two cartons' footprints overlap (m, separating-axis test; 0 when apart)."""
    ca, cb = _corners(objects, a), _corners(objects, b)
    depth = 1e9
    for c in (ca, cb):
        for edge in ((c[1] - c[0]), (c[2] - c[0])):
            axis = edge / np.linalg.norm(edge)
            pa, pb = ca @ axis, cb @ axis
            depth = min(depth, min(pa.max(), pb.max()) - max(pa.min(), pb.min()))
    return max(0.0, depth)


OVERLAP_TOL = 0.002  # m: footprints may overlap this much (contact softness), not more


def in_order(objects) -> bool:
    """All five stand on the board, none overlapping another, and their centres run yellow, blue, green, red, white
    from the robot's right to its left."""
    if not all(on_board(objects, n) for n in NAMES):
        return False
    if any(penetration(objects, a, b) > OVERLAP_TOL for i, a in enumerate(NAMES) for b in NAMES[i + 1:]):
        return False
    ys = [float(objects[n].pos[1]) for n in ("yellow", "blue", "green", "red", "white")]
    return bool(all(a < b for a, b in zip(ys[:-1], ys[1:], strict=True)))


def _pads(g) -> list[np.ndarray]:
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def let_go(objects) -> bool:
    """No gripper pad is within CLEAR of any carton."""
    for g in ("left_gripper", "right_gripper"):
        gr = objects.get(g)
        if gr is None:
            continue
        for p in _pads(gr):
            for name in NAMES:
                w, d, h, _, _ = CARTONS[name]
                c = objects[name]
                q = _rot(c.quat_xyzw).T @ (p - np.asarray(c.pos, float))
                if abs(q[0]) <= d / 2 + CLEAR and abs(q[1]) <= w / 2 + CLEAR and -CLEAR <= q[2] <= h + CLEAR:
                    return False
    return True


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in NAMES)


def still(objects, anchor) -> bool:
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened this episode, updated once per control step: whether a carton fell off the table (the only
    thing tracked over the whole run), and since when every full-success condition has held (``met_since``) with the
    poses then (``anchor``). Any lapse restarts the count; starts over when the episode time goes back."""

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
        self.fell |= any(float(objects[n].pos[2]) < TOP - 0.03 for n in NAMES)
        if t is None:
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    orderly = in_order(objects)
    return [gap_opened(objects), green_in_row(objects), orderly, bool(orderly and let_go(objects))]


def steps_done(objects) -> list[bool]:
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = f"(gap {gap_width(objects) * 1000:.0f} mm; green y {float(objects['green'].pos[1]) * 1000:.0f} mm)"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    values = [v for n in NAMES for v in objects[n].pos]
    return bool(not np.all(np.isfinite(values)) or _PROGRESS.fell)


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
SQUEEZE = 0.008
FREE = 0.020  # fingers open to the carton's depth plus this


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


def grip_frame(x_axis, z_axis) -> np.ndarray:
    x, z = np.asarray(x_axis, float), np.asarray(z_axis, float)
    return np.column_stack([x, np.cross(z, x), z])


LEAN = math.radians(35.0)


def _rot_down(side: str) -> np.ndarray:
    """Pointing down and leaned 35 deg away from the arm's own base, the pads closing along x (across a carton's
    depth): each arm reaches the whole row this way, with its wrist over the cartons on its own side."""
    if side == "right":
        return grip_frame(_X, np.array([0.0, math.sin(LEAN), -math.cos(LEAN)]))
    return grip_frame(-_X, np.array([0.0, -math.sin(LEAN), -math.cos(LEAN)]))


TOP_GRIP = 0.010  # the TCP is this far below a carton's top edge (the pads hold its top 23 mm)
UP = 0.060  # approach height of the TCP above the grip height
LIFT = 0.008  # a shifted carton is carried this high above the board


def shift_plan(name: str, y_from: float, y_to: float, wait_s: float, side: str) -> list[Waypoint]:
    """Grip a standing carton across its depth, lift it 8 mm, carry it along the row, set it down, let go and rise."""
    w, d, h, _, _ = CARTONS[name]
    cx = ROW_FRONT + d / 2
    z = SHELF_TOP + h - TOP_GRIP
    hold, opn = -(d - SQUEEZE) / 2, -(d + FREE) / 2
    r = _rot_down(side)
    return [
        Waypoint(wait_s, _START_TCP[side], FINGER_OPEN, "wait", rot=_START_ROT[side]) if wait_s > 0.05 else None,
        Waypoint(2.0, np.array([cx, y_from, z + UP]), opn, f"over the {name} carton", rot=r),
        Waypoint(0.9, np.array([cx, y_from, z]), opn, f"down around the {name} carton", rot=r),
        Waypoint(0.6, np.array([cx, y_from, z]), hold, f"grip the {name} carton", rot=r),
        Waypoint(0.4, np.array([cx, y_from, z + LIFT]), hold, "lift", rot=r),
        Waypoint(1.2, np.array([cx, y_to, z + LIFT]), hold, "slide along the row", rot=r),
        Waypoint(0.4, np.array([cx, y_to, z]), hold, "set down", rot=r),
        Waypoint(0.5, np.array([cx, y_to, z]), opn, "let go", rot=r),
        Waypoint(0.7, np.array([cx, y_to, z + UP]), opn, "rise", rot=r),
    ]


def green_plan(wait_s: float, side: str = "right") -> list[Waypoint]:
    w, d, h, _, _ = CARTONS["green"]
    gx, gy = GREEN_START
    z = TOP + h - TOP_GRIP
    hold, opn = -(d - SQUEEZE) / 2, -(d + FREE) / 2
    r = _rot_down(side)
    carry = z + 0.034  # its base ends up 16 mm above the board's top face
    nx = ROW_FRONT + d / 2
    ny = FINAL_Y["green"]
    return [
        Waypoint(wait_s, _START_TCP[side], FINGER_OPEN, "wait", rot=_START_ROT[side]) if wait_s > 0.05 else None,
        Waypoint(2.0, np.array([gx, gy, z + UP]), opn, "over the green carton", rot=r),
        Waypoint(0.9, np.array([gx, gy, z]), opn, "down around the green carton", rot=r),
        Waypoint(0.6, np.array([gx, gy, z]), hold, "grip the green carton", rot=r),
        Waypoint(0.6, np.array([gx, gy, carry]), hold, "lift", rot=r),
        Waypoint(1.4, np.array([gx, ny, carry]), hold, "carry along to the gap", rot=r),
        Waypoint(1.8, np.array([nx, ny, carry]), hold, "slide into the gap", rot=r),
        Waypoint(0.5, np.array([nx, ny, carry - 0.014]), hold, "set down", rot=r),
        Waypoint(0.5, np.array([nx, ny, carry - 0.014]), opn, "let go", rot=r),
        Waypoint(0.8, np.array([nx, ny, carry + UP]), opn, "rise", rot=r),
    ]


def _plan(parts) -> list[Waypoint]:
    return [w for part in parts for w in part if w is not None]


def demo():
    """The right arm slides the blue carton against the yellow one while the left arm slides the white carton against
    the right-hand divider, then the red carton against the white one; once the left arm is home the right arm stands
    the green carton in the opened gap."""
    shift_time = 2.0 + 0.9 + 0.6 + 0.4 + 1.2 + 0.4 + 0.5 + 0.7
    right = _plan([shift_plan("blue", ROW_Y["blue"], FINAL_Y["blue"], 0.0, "right"),
                   green_plan(shift_time + 3.5, "right")])
    right.append(Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]))
    left = _plan([shift_plan("white", ROW_Y["white"], FINAL_Y["white"], 0.0, "left"),
                  shift_plan("red", ROW_Y["red"], FINAL_Y["red"], 1.5, "left")])
    left.append(Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]))
    return {"left": left, "right": right}


_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
