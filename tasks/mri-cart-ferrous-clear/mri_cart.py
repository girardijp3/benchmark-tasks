"""On the station table: a compact rolling sedation cart (460 x 360 mm tray, 120 mm up, casters locked) that is about to
be wheeled into an MRI scanner room, and a red storage tote (320 x 240 x 100 mm) beside it. On the tray: three things that
must not go in (a Kelly hemostat of magnetic stainless steel, 140 mm; a fingertip pulse oximeter with its batteries; a
handheld two-way radio) and three that are MRI safe (a pair of disposable plastic locking forceps that look like the
hemostat, a roll of paper tape, a specimen container with a blue lid).

The task is to clear the magnetic and battery-powered things off the cart into the tote and leave the safe ones.

Run it:  inspect-robots run --embodiment yam_arena -E task=mri_cart.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts).
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
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "mri_cart"

# -- the parts (as built in assets/; each frame's origin and axes are given in its USD file's doc string) --------------
CART_X, CART_Y, TOP_Z = 0.46, 0.36, 0.12  # the tray, and its upper face above the table
RIM_H, RIM_T = 0.008, 0.003
BIN_X, BIN_Y, BIN_H, BIN_WALL, BIN_FLOOR = 0.32, 0.24, 0.10, 0.003, 0.003

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
CART_XY = (0.30, -0.12)
BIN_XY = (0.24, -0.52)
TRAY = TOP + TOP_Z  # the tray's upper face
BIN_TOP = TOP + BIN_H
BIN_FLOOR_TOP = TOP + BIN_FLOOR
# each item's start on the tray, from the tray's centre: (dx, dy, yaw degrees)
ON_TRAY = {
    "hemostat": (-0.10, -0.12, 0.0),
    "radio": (0.06, -0.13, 0.0),
    "oximeter": (-0.10, -0.03, 0.0),
    "cup": (-0.12, 0.08, 0.0),
    "tape": (0.0, 0.07, 0.0),
    "forceps": (0.15, 0.06, 0.0),
}
UNSAFE = ("hemostat", "radio", "oximeter")  # magnetic steel and electronics
SAFE = ("forceps", "tape", "cup")  # plastic and paper: stay on the cart
RIGID = tuple(ON_TRAY)
MASS = {"hemostat": 0.052, "oximeter": 0.050, "radio": 0.260, "forceps": 0.012, "tape": 0.025, "cup": 0.021}
# Each part's bounds in its own frame (low, high), m.
BOX = {
    "hemostat": ((-0.090, -0.027, 0.0), (0.055, 0.027, 0.0095)),
    "oximeter": ((-0.029, -0.017, 0.0), (0.029, 0.017, 0.031)),
    "radio": ((-0.056, -0.029, 0.0), (0.127, 0.029, 0.036)),
    "forceps": ((-0.080, -0.026, 0.0), (0.047, 0.026, 0.0080)),
    "tape": ((-0.020, -0.020, 0.0), (0.020, 0.020, 0.025)),
    "cup": ((-0.030, -0.030, 0.0), (0.030, 0.030, 0.055)),
}


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body, an articulation, or a fixture
    (``kinematic``). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False
    articulation: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


def _yaw_xyzw(deg: float):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


CART = UsdPart("cart", asset="cart", xy=CART_XY, extent=CART_X, mass=4.0, usd_path=str(ASSETS / "cart.usda"),
               pos=(*CART_XY, TOP), kinematic=True)
BIN = UsdPart("bin", asset="bin", xy=BIN_XY, extent=BIN_X, mass=0.6, usd_path=str(ASSETS / "bin.usda"),
              pos=(*BIN_XY, TOP), kinematic=True)
REST_GAP = 0.0008  # parts spawn this far above the tray and settle onto it
USD_FILE = {"hemostat": "hemostat.usda", "oximeter": "pulse_oximeter.usda", "radio": "handheld_radio.usda",
            "forceps": "plastic_forceps.usda", "tape": "tape_roll.usda", "cup": "specimen_cup.usda"}


def start_xy(name: str) -> np.ndarray:
    dx, dy, _ = ON_TRAY[name]
    return np.array([CART_XY[0] + dx, CART_XY[1] + dy])


ITEMS = {n: UsdPart(n, asset=n, xy=tuple(start_xy(n)), extent=0.2, mass=MASS[n], usd_path=str(ASSETS / USD_FILE[n]),
                    pos=(*start_xy(n), TRAY + REST_GAP), rot_xyzw=_yaw_xyzw(ON_TRAY[n][2])) for n in ON_TRAY}
OBJECTS = [CART, BIN, *ITEMS.values()]
NAMES = tuple(o.name for o in OBJECTS)

MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    pose = scene.Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(float(v) for v in spec.rot_xyzw))  # python floats: Isaac Lab rejects numpy float64 here
    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID, initial_pose=pose,
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    return scene.Object(
        name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID, initial_pose=pose,
        spawn_cfg_addon={"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)})


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
        """Each object's root pose and velocities, the grippers and the episode time."""
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
IN_BIN_MARGIN = 0.0  # m: every corner of a part's bounds within the tote's inner floor plan, shrunk by this
ON_CART_TILT = math.radians(10.0)  # a safe item standing on the tray is tilted less than this
ON_CART_UP = 0.008  # m: ... and its origin is within this of the tray's face (it has not been lifted or knocked up)
FALLEN_Z = -0.03  # m below the table top: a part that low has fallen off the table
CLEAR = 0.03  # m: a gripper's pads at least this far from every item is a clear release
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
STEPS = ("hemostat in the tote", "radio in the tote", "pulse oximeter in the tote",
         "cart left with only the safe items, grippers clear")
POINTS = 0.2  # per step; only success scores 1.0
WHOLE = RIGID  # every part whose stillness counts


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _corners(box) -> np.ndarray:
    lo, hi = box
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def part_points(objects, name: str, box_name: str | None = None) -> np.ndarray:
    """The corners of a part's bounds in the world frame."""
    s = objects[name]
    return _corners(BOX[box_name or name]) @ _rot(s).T + np.asarray(s.pos, float)


def _frames(objects, name: str):
    return [part_points(objects, name)]


def _in_bin_pts(pts: np.ndarray) -> bool:
    hx = BIN_X / 2 - BIN_WALL - IN_BIN_MARGIN
    hy = BIN_Y / 2 - BIN_WALL - IN_BIN_MARGIN
    return bool(np.all(np.abs(pts[:, 0] - BIN_XY[0]) <= hx) and np.all(np.abs(pts[:, 1] - BIN_XY[1]) <= hy)
                and np.all(pts[:, 2] >= BIN_FLOOR_TOP - 0.004) and pts[:, 2].min() <= BIN_TOP)


def in_tote(objects, name: str) -> bool:
    """The part is in the tote: every corner of its bounds inside the tote's floor plan, the lowest within its walls'
    height."""
    return all(_in_bin_pts(p) for p in _frames(objects, name))


def safe_on_cart(objects, name: str) -> bool:
    """A safe item still stands on the tray: its origin over the tray (inside the rim), no higher than ON_CART_UP above
    the tray's face, and upright within ON_CART_TILT."""
    s = objects[name]
    x, y, z = s.pos
    if abs(x - CART_XY[0]) > CART_X / 2 - RIM_T or abs(y - CART_XY[1]) > CART_Y / 2 - RIM_T:
        return False
    tilt = math.acos(max(-1.0, min(1.0, float(_rot(s)[2, 2]))))
    return bool(abs(z - (TRAY + REST_GAP)) <= ON_CART_UP + 0.002 and tilt <= ON_CART_TILT)


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def _box_near(pts: np.ndarray, state_pos, state_rot, box, slack: float) -> bool:
    q = (pts - np.asarray(state_pos, float)) @ state_rot
    lo, hi = box
    return bool(np.any(np.all((q >= np.asarray(lo) - slack) & (q <= np.asarray(hi) + slack), axis=1)))


def clear_of_items(objects) -> bool:
    """No gripper's pads are within CLEAR of any item."""
    for g in ("left_gripper", "right_gripper"):
        gr = objects.get(g)
        if gr is None:
            continue
        pads = _pads(gr)
        for name in RIGID:
            s = objects[name]
            if _box_near(pads, s.pos, _rot(s), BOX[name], CLEAR):
                return False
    return True


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in WHOLE)


def still(objects, anchor) -> bool:
    """Every part within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses, not from the
    simulator's instantaneous speeds."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


def cleared(objects) -> bool:
    """Everything steel or electronic is in the tote and off the cart."""
    return bool(all(in_tote(objects, n) for n in UNSAFE))


def safe_all_on_cart(objects) -> bool:
    return bool(all(safe_on_cart(objects, n) for n in SAFE))


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand."""
    a, b, c = (in_tote(objects, n) for n in UNSAFE)
    return [a, b, c, bool(a and b and c and safe_all_on_cart(objects) and clear_of_items(objects))]


class _Progress:
    """What has happened so far this episode, updated once per control step: since when every full-success condition
    has held without a break (``met_since``, ``None`` while one fails) with the poses when they began to hold
    (``anchor``), and whether a part has fallen off the table (kept for the whole episode). Starts over when the episode
    time goes back."""

    def __init__(self):
        self.time = -1.0
        self.fell: list[str] = []
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        stamp = -1.0 if t is None else t
        for name in RIGID:
            if objects[name].pos[2] < TOP + FALLEN_Z and name not in self.fell:
                self.fell.append(name)
                print(f"[{TAG}] t={stamp:.2f}s  {name} fell off the table (scores 0 from here on)", flush=True)
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not self.fell)
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'a part moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand: the hemostat, oximeter and radio each in the tote; the last one only with all three, every safe item still on the tray and every gripper clear."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = ("(" + "; ".join(f"{n}: {'in tote' if in_tote(objects, n) else 'out'}" for n in UNSAFE)
                + "; safe on cart: " + ",".join(n for n in SAFE if safe_on_cart(objects, n)) + ")")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: a part off the table, or a state that is not finite. Reads the progress without updating it."""
    values = [v for n in RIGID for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.fell)


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: the hemostat, oximeter and radio in the tote, the plastic forceps, tape and container
    still on the tray, every gripper clear, and all parts within STILL_MOVE and STILL_TURN of where they were when that
    second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 if a part fell off the table."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# One arm, the right. Every grip is straight down. Jaws close across y (yaw 0) or across x (yaw 90 deg).
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (about 20 N a finger)
OPEN_EXTRA = 0.016  # the pads open to the part's width plus this
CARRY_Z = TOP_Z + 0.075  # the TCP's height above the table top in transit (clears the tray's contents)
HOVER = 0.080  # the TCP's height above the pinch height coming down or leaving
SET_GAP = 0.004  # a part is set down this far above what it rests on before the fingers open

# what each grip is: the pinch point in the part's frame (x, y), the width across the jaws, how far the pad tips are above
# what the part rests on, and the gripper's yaw (degrees) relative to the part's own
GRIPS = {
    "hemostat": dict(pinch=(0.0, 0.0), width=0.014, tips=0.0015, yaw=0.0),
    "oximeter": dict(pinch=(0.0, 0.0), width=0.034, tips=0.0040, yaw=0.0),
    "radio": dict(pinch=(-0.010, 0.0), width=0.058, tips=0.0050, yaw=0.0),
}
# where each part is set down in the tote: its origin's xy relative to the tote's centre
IN_TOTE = {"hemostat": (0.06, -0.005), "radio": (-0.07, 0.07), "oximeter": (-0.06, -0.075)}


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

_START_TCP = yam_demo.ArmPlan([], BASES["right"]).start_pos
_START_ROT = yam_demo.ArmPlan([], BASES["right"]).start_rot


def _tcp(xy, support: float, tips: float, lift: float = 0.0) -> tuple[float, float, float]:
    return (float(xy[0]), float(xy[1]), support + tips + PAD_TIP_PAST_TCP + lift)


def _move(label: str, name: str, dst_rel, dst_support: float):
    """Pick the part from where it starts (resting on the tray) and set it down in the tote, its origin over ``dst_rel``
    from the tote's centre, in the same turn. The grip's pinch point is carried straight."""
    g = GRIPS[name]
    item_yaw = ON_TRAY[name][2]
    rot = np.array([[math.cos(math.radians(item_yaw)), -math.sin(math.radians(item_yaw))],
                    [math.sin(math.radians(item_yaw)), math.cos(math.radians(item_yaw))]])
    off = rot @ np.array(g["pinch"])
    src = start_xy(name) + off
    dst = np.array([BIN_XY[0] + dst_rel[0], BIN_XY[1] + dst_rel[1]]) + off
    yaw = item_yaw + g["yaw"]
    op = -(g["width"] + OPEN_EXTRA) / 2
    hd = -(g["width"] - SQUEEZE) / 2
    tips = g["tips"]
    s0 = TRAY + REST_GAP
    zc = TOP + CARRY_Z
    return [
        Waypoint(2.0, (*src, zc), op, f"over the {label}", yaw=yaw),
        Waypoint(1.6, _tcp(src, s0, tips, 0.02), op, f"down beside the {label}", yaw=yaw),
        Waypoint(1.0, _tcp(src, s0, tips), op, f"pads round the {label}", yaw=yaw),
        Waypoint(0.8, _tcp(src, s0, tips), hd, f"grip the {label}", yaw=yaw),
        Waypoint(0.4, _tcp(src, s0, tips), hd, f"hold the {label}", yaw=yaw),
        Waypoint(1.2, (*src, zc), hd, f"lift the {label}", yaw=yaw),
        Waypoint(3.0, (*dst, zc), hd, f"carry the {label} to the tote", yaw=yaw),
        Waypoint(1.6, _tcp(dst, dst_support, tips, SET_GAP), hd, f"lower the {label} into the tote", yaw=yaw),
        Waypoint(0.6, _tcp(dst, dst_support, tips, SET_GAP), op, f"let go of the {label}", yaw=yaw),
        Waypoint(1.0, _tcp(dst, dst_support, tips, HOVER), op, "lift away", yaw=yaw),
    ]


def right_plan() -> list[Waypoint]:
    wps = []
    for name in UNSAFE:
        wps += _move(name, name, IN_TOTE[name], BIN_FLOOR_TOP)
    wps.append(Waypoint(2.5, tuple(_START_TCP), FINGER_OPEN, "home", rot=_START_ROT))
    return wps


def demo():
    """The right arm picks the hemostat, the oximeter and the radio in turn and sets each in the tote, then goes home.
    The plastic forceps, the tape and the container are not touched."""
    return {"left": [], "right": right_plan()}
