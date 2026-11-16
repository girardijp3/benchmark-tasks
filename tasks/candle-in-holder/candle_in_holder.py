"""Setting a dinner table (at home, or before restaurant or hotel service): a free 4 in brass candlestick (340 g, 127 mm foot, 7/8 in socket) stands on the table and an
8 in taper-base dinner candle stands on the table 220 mm to the right of it. The task is to seat the candle in the candlestick: one hand
may pinch the candlestick's stem and steady it (the demo does; one hand is enough), the other presses the tapered base into the snug 22 mm cup and twists it in until it is
wedged to the cup's floor and stands upright by itself. Nothing is scripted: all effects are physical.

Run it:  inspect-robots run --embodiment yam_arena -E task=candle_in_holder.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "candle"
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


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: a free rigid body, an articulation
    (``articulation``: its root link is free) or a fixture (``kinematic``). ``Asset``'s fields only satisfy the
    task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False  # an articulation whose root link is fixed to the world
    no_gravity: bool = False  # a free part with gravity off (parked out of the way)

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


SOLVER_ITERATIONS = 64
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
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=True if spec.fixed else None)
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


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each part's root pose and velocity, the grippers and the episode time, plus what ``extra_states`` adds."""
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
        """yam_arena's environment settings, plus the task's reset events (``reset_events``)."""
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            env_cfg = configure(env_cfg)
            for name, func, params in reset_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="reset", params=params))
            for name, func, params in step_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="interval", interval_range_s=(0.0, 0.0),
                                                           is_global_time=False, params=params))
            return env_cfg

        return with_events

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


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
# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
STICK_XY = (0.30, -0.08)  # the socket's axis at the start
Z_RIM = 0.1016  # the cup's rim above the table (4 in)
SOCKET_DEPTH = 0.022
FLOOR_Z = Z_RIM - SOCKET_DEPTH
LEAD = 0.004
CANDLE_H = 0.203
CANDLE_MASS = 0.046
STICK_MASS = 0.340
CANDLE_START = (0.26, -0.20, TOP + 0.0003)  # the base end's middle
STICK_Z0 = 0.0004  # the foot starts this far above the table (settles onto it)

STICK = UsdPart("stick", asset="stick", xy=STICK_XY, extent=0.13, mass=STICK_MASS, usd_path=str(ASSETS / "candlestick.usda"),
                pos=(STICK_XY[0], STICK_XY[1], TOP + STICK_Z0), articulation=True)
CANDLE = UsdPart("candle", asset="candle", xy=CANDLE_START[:2], extent=0.03, mass=CANDLE_MASS, usd_path=str(ASSETS / "candle.usda"), pos=CANDLE_START)
OBJECTS = [STICK, CANDLE]
MOVERS = ("candle", "stick")


def extra_states(base, out, origin) -> None:
    """Nothing beyond the parts' poses."""


def reset_events() -> list:
    return []


def step_events() -> list:
    return []


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
# -- the rule and what is scored ----------------------------------------------------------------------------------------
LIFTED = 0.015  # the candle counts as picked up once it has been this far above the table
WEDGE_DEPTH = 0.016  # step 2: the candle's base end is this far below the rim: the taper has begun to wedge (it first touches the walls at about 16.4 mm)
SEAT_DEPTH = 0.0205  # step 3: wedged to within 1.5 mm of the floor (22 mm down) ...
SEAT_XY, SEAT_TILT = 0.0015, math.radians(2.0)  # ... base within 1.5 mm of the socket's axis, within 2 degrees of upright
ENTER_XY, ENTER_TILT = 0.005, math.radians(8.0)  # step 2: base end within 5 mm of the axis and 8 degrees of upright
STICK_TIP, STICK_SLIDE = math.radians(10.0), 0.010  # a rule event, kept for the run: the candlestick tips over 10 degrees or slides 10 mm
CLEAR = 0.03
CANDLE_LO = np.array([-0.0117, -0.0117, 0.0])
CANDLE_HI = np.array([0.0117, 0.0117, CANDLE_H])
STICK_LO = np.array([-0.0635, -0.0635, 0.0])
STICK_HI = np.array([0.0635, 0.0635, Z_RIM + LEAD])
STEPS = ("the candle picked up off the table", "its tapered base wedging in the socket", "wedged to the floor, upright",
         "both hands clear of the candle and the candlestick")


def _tilt(objects, name="candle"):
    return math.acos(max(-1.0, min(1.0, float(_rot(objects[name])[2, 2]))))


def _axis(objects):
    """The socket's axis (the free candlestick's origin) in xy."""
    return np.asarray(objects["stick"].pos, float)


def _off(objects):
    p, s = np.asarray(objects["candle"].pos, float), _axis(objects)
    return float(np.hypot(p[0] - s[0], p[1] - s[1]))


def _depth(objects):
    """How far the candle's base end is below the cup's rim (m)."""
    return float(_axis(objects)[2] + Z_RIM - np.asarray(objects["candle"].pos, float)[2])


def entered(objects) -> bool:
    return bool(_off(objects) <= ENTER_XY and _depth(objects) >= WEDGE_DEPTH and _tilt(objects) <= ENTER_TILT)


def seated(objects) -> bool:
    return bool(_off(objects) <= SEAT_XY and _depth(objects) >= SEAT_DEPTH and _tilt(objects) <= SEAT_TILT)


def held(objects) -> bool:
    return any(holds(objects, s, "candle", CANDLE_LO, CANDLE_HI) or holds(objects, s, "stick", STICK_LO, STICK_HI) for s in ("left", "right"))


def clear_of_candle(objects) -> bool:
    return not pads_near(objects, "candle", CANDLE_LO, CANDLE_HI, CLEAR)


def _steps(objects) -> list[bool]:
    lifted = bool(_PROGRESS.mem.get("lifted"))
    ent = bool(entered(objects))
    seat = bool(seated(objects))
    return [lifted, ent, seat, bool(seat and clear_of_candle(objects) and not held(objects))]


def events_now(objects, p) -> list[str]:
    """Rule events as things stand: the candle off the table, the candlestick tipped over 10 degrees or slid over 10 mm. Also remembers
    (``p.mem``) that the candle was picked up."""
    out = []
    if objects["candle"].pos[2] > CANDLE_START[2] + LIFTED:
        p.mem["lifted"] = True
    if objects["candle"].pos[2] < TOP - 0.03:
        out.append("the candle fell off the table")
    s = objects["stick"]
    p.mem.setdefault("stick0", np.array(s.pos[:2], float))
    if _tilt(objects, "stick") > STICK_TIP:
        out.append("the candlestick tipped over 10 degrees")
    if float(np.linalg.norm(np.array(s.pos[:2], float) - p.mem["stick0"])) > STICK_SLIDE:
        out.append("the candlestick slid more than 10 mm")
    if s.pos[2] < TOP - 0.03:
        out.append("the candlestick fell off the table")
    return out


def trace_line(objects) -> str:
    c, s = objects["candle"], _axis(objects)
    return (f"candle base at {(c.pos[0] - s[0]) * 1000:.1f},{(c.pos[1] - s[1]) * 1000:.1f} mm from the socket axis, "
            f"{_depth(objects) * 1000:.1f} mm below the rim (floor {SOCKET_DEPTH * 1000:.0f}), tilt {math.degrees(_tilt(objects)):.1f} deg"
            f"{'*' if held(objects) else ''}, stick tilt {math.degrees(_tilt(objects, 'stick')):.1f} deg, slid "
            f"{np.linalg.norm(np.array(objects['stick'].pos[:2], float) - np.array([STICK_XY[0], STICK_XY[1]])) * 1000:.1f} mm, "
            f"entered {entered(objects)}, seated {seated(objects)}")


# -- scoring (shared machinery) ---------------------------------------------------------------------------------------
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
POINTS = 0.2  # per step; only success scores 1.0
GRIP_SLACK = 0.004  # held: a pad's point within this of the part's bounds ...
GRIP_FORCE = 0.5  # ... both fingers pressing with at least this (N) ...
GRIP_OPENING = 0.003  # ... and the pads apart by at least this (not closed on nothing)


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def holds(objects, side: str, name: str, lo, hi) -> bool:
    """The ``side`` gripper holds the part: a point along its pads is within GRIP_SLACK of the part's bounds (``lo``,
    ``hi``, in the part's frame), both fingers press with GRIP_FORCE and the pads are apart by GRIP_OPENING."""
    g = objects.get(f"{side}_gripper")
    if g is None or g.contact < GRIP_FORCE or g.opening < GRIP_OPENING:
        return False
    s = objects[name]
    q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
    return bool(np.any(np.all((q >= np.asarray(lo) - GRIP_SLACK) & (q <= np.asarray(hi) + GRIP_SLACK), axis=1)))


def pads_near(objects, name: str, lo, hi, margin: float) -> bool:
    """Either gripper's pads are within ``margin`` of the part's bounds."""
    s = objects[name]
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
        if np.any(np.all((q >= np.asarray(lo) - margin) & (q <= np.asarray(hi) + margin), axis=1)):
            return True
    return False


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in MOVERS)


def still(objects, anchor) -> bool:
    """Every part within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses, not from the
    simulator's instantaneous speeds."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: the task's own memory (``mem``), every
    rule violation (``events``, they last for the whole episode), and since when every full-success condition has held
    without a break (``met_since``, ``None`` while one fails) with the poses when they began to hold (``anchor``).
    Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.mem: dict = {}
        self.events: list[str] = []
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def flag(self, t, what: str) -> None:
        if what not in self.events:
            self.events.append(what)
            print(f"[{TAG}] t={t:.2f}s  RULE BROKEN: {what} (scores 0 from here on)", flush=True)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        stamp = -1.0 if t is None else t
        for what in events_now(objects, self):
            self.flag(stamp, what)
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not self.events)
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


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand (see ``_steps``)."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = f"({trace_line(objects)}; rule events {len(p.events)})"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: any rule event this episode (kept for the whole run), or a state that is not finite. Reads the
    episode's progress without updating it."""
    values = [v for n in MOVERS for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.events)


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all steps done, no rule event so far, and the parts within STILL_MOVE and STILL_TURN of where they were when
    that second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise POINTS per step done (``STEPS``, at
    most 0.8, also while the second is being timed); 0 after any rule event."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)
# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand pinches the candlestick's waisted stem (24 mm across) from the side with the gripper leaned, and holds it. The right hand
# pinches the candle across its width 150 mm up, lifts it, carries it over the socket, lowers its tapered base end through the 4 mm
# lead-in and presses it into the wedge while turning it (twist 35 degrees), lets go; then the left hand lets go and both go home.
_grip = yam_demo.grip_rotation
SQUEEZE = 0.008
PINCH_H = 0.065  # the right pads' tip is this far up the candle
C_WIDTH = 0.0211  # the candle width there (about 21 mm: 23.4 mm at 25 mm up, 17 mm at the top)
HOLD = -(C_WIDTH - SQUEEZE) / 2
OPEN = -(C_WIDTH + 0.016) / 2
TWIST = math.radians(20)
CARRY = Z_RIM + 0.014  # a carried candle's base end this high above the table (the lead-in's mouth is 4 mm above the rim)
AIM = (0.0, 0.0)  # aim correction of the pinch (x, y)
PUSH = 0.025
PUSH_DY = -0.010  # the press is aimed this far to +y of the socket axis (cancels the arm's sideways give while it pushes)  # the arm commands the base end this far below the floor (its position control then pushes with its own stiffness)


def _frame(x_axis, z_axis) -> np.ndarray:
    x, z = np.asarray(x_axis, float), np.asarray(z_axis, float)
    return np.column_stack([x, np.cross(z, x), z])


RLEAN = math.radians(40)  # the right gripper leans toward the other arm so its wrist stays over its own side (reach)


def _rot_right(twist=0.0) -> np.ndarray:
    base = _frame(np.array([1.0, 0.0, 0.0]), np.array([0.0, math.sin(RLEAN), -math.cos(RLEAN)]))
    return base if twist == 0.0 else axis_angle_matrix(np.array([0.0, 0.0, 1.0]), twist) @ base


def _tcp(x, y, z_base, rot):
    """The TCP that puts the pads' tip on the candle's axis, PINCH_H up its length."""
    return tuple(np.array([x, y, z_base + PINCH_H]) - PAD_TIP_PAST_TCP * rot[:, 2])


LEAN = math.radians(50.0)
STEM_Z = 0.052  # the pinch's height on the stem (the waist is 24 mm across between the knop and the flare)
STEM_W = 0.024
S_HOLD = -(STEM_W - SQUEEZE) / 2
S_OPEN = -(STEM_W + 0.020) / 2
L_AXIS = np.array([0.0, -math.sin(LEAN), -math.cos(LEAN)])
L_ROT = _frame(np.array([-1.0, 0.0, 0.0]), L_AXIS)


def _stem_tcp(dz=0.0, back=0.0):
    c = np.array([STICK_XY[0], STICK_XY[1], TOP + STEM_Z + dz])
    return tuple(c + 0.006 * L_AXIS - back * L_AXIS)


def right_plan() -> list[Waypoint]:
    x0, y0, z0 = CANDLE_START[0], CANDLE_START[1], CANDLE_START[2] - TOP
    dx, dy = STICK_XY[0] + AIM[0], STICK_XY[1] + AIM[1]
    zf = FLOOR_Z - PUSH
    ROT, twisted = _rot_right(), _rot_right(TWIST)
    t = lambda x, y, z, r=ROT: _tcp(x, y, TOP + z, r)  # noqa: E731
    return [
        Waypoint(2.5, t(x0, y0, z0 + 0.04), OPEN, "over the candle", rot=ROT),
        Waypoint(1.5, t(x0, y0, z0), OPEN, "pads astride the candle", rot=ROT),
        Waypoint(0.8, t(x0, y0, z0), HOLD, "grip the candle", rot=ROT),
        Waypoint(0.4, t(x0, y0, z0), HOLD, "hold", rot=ROT),
        Waypoint(1.2, t(x0, y0, CARRY), HOLD, "lift the candle", rot=ROT),
        Waypoint(2.5, t(dx, dy, CARRY), HOLD, "over the socket", rot=ROT),
        Waypoint(1.5, t(dx, dy, Z_RIM + 0.003), HOLD, "down to the lead-in", rot=ROT),
        Waypoint(3.0, t(dx, dy + PUSH_DY, zf, twisted), HOLD, "press and twist the base in", rot=twisted),
        Waypoint(0.8, t(dx, dy, FLOOR_Z - 0.001, twisted), HOLD, "ease off the push", rot=twisted),
        Waypoint(0.6, t(dx, dy, FLOOR_Z - 0.001, twisted), OPEN, "let go", rot=twisted),
        Waypoint(1.2, t(dx, dy, CARRY, ROT), OPEN, "lift away", rot=ROT),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan() -> list[Waypoint]:
    right_release = sum(w.duration for w in right_plan()[:10])  # the right hand has let go
    pre = sum(w.duration for w in right_plan()[:6])
    up = 0.07
    return [
        Waypoint(2.5, _stem_tcp(up, 0.05), S_OPEN, "over the stem", rot=L_ROT),
        Waypoint(1.5, _stem_tcp(0.0, 0.0), S_OPEN, "pads astride the stem", rot=L_ROT),
        Waypoint(0.8, _stem_tcp(), S_HOLD, "steady the candlestick", rot=L_ROT),
        Waypoint(max(right_release - 4.8 + 0.5, 0.1), _stem_tcp(), S_HOLD, "hold it still", rot=L_ROT),
        Waypoint(0.6, _stem_tcp(), S_OPEN, "let go", rot=L_ROT),
        Waypoint(1.2, _stem_tcp(up, 0.05), S_OPEN, "lift away", rot=L_ROT),
        Waypoint(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


def demo():
    """The left hand steadies the candlestick by its stem; the right hand presses and twists the candle into it."""
    return {"left": left_plan(), "right": right_plan()}
