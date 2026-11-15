"""On the station table, in a stand: the sleeve of an adjustable walking stick (a 32 mm aluminium ring, bore 24 mm, 100 mm tall, with two axial
slots and two circumferential notches, one at the bottom for the stowed length and one 56 mm up at the red mark), and in it the stick's upper
tube (20 mm, 0.2 kg) with two lugs at its bottom, a red band and a T handle bar on top. It starts stowed: the lugs rest in the bottom notches.

The task is to lengthen the stick until its red band shows above the sleeve and lock it there: the lugs have to be turned back to the slots
(the ceiling of the notch holds the tube down), the tube drawn up the slots to the mark, and turned into the upper notches, where it rests on the
notch floor and cannot be drawn out. The lock is geometric (no spring); the tube is a free rigid body and everything is convex colliders.

Run it:  inspect-robots run --embodiment yam_arena -E task=walking_stick_twist_lock.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "stick"
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
SLEEVE_XY = (0.30, -0.12)  # the sleeve's axis
FLOOR_Z = 0.006  # the stowed notch floor (the base plate's top) above the table
MARK_Z = 0.056  # the tube's bottom when its lugs rest on the upper notch floor (the red mark shows)
NOTCH_H = 0.014
LUG_H = 0.008
SLEEVE_TOP = 0.100
TUBE_L, BAR_Z = 0.120, 0.010
STICK_TOTAL = TUBE_L + BAR_Z
STICK_R = 0.010
STICK_MASS = 0.20
START_YAW = math.radians(48.0)  # the lugs rest in the stowed notch, between its slot edge (about 43 deg) and its stop (about 52)
START_QUAT = (0.0, 0.0, math.sin(START_YAW / 2), math.cos(START_YAW / 2))
STICK_START = (SLEEVE_XY[0], SLEEVE_XY[1], TOP + FLOOR_Z + 0.0003)  # the tube's bottom middle

SLEEVE = UsdPart("sleeve", asset="sleeve", xy=SLEEVE_XY, extent=0.07, mass=None, usd_path=str(ASSETS / "stick_sleeve.usda"),
                 pos=(SLEEVE_XY[0], SLEEVE_XY[1], TOP), kinematic=True)
STICK = UsdPart("stick", asset="stick", xy=STICK_START[:2], extent=0.04, mass=STICK_MASS, usd_path=str(ASSETS / "stick_inner.usda"), pos=STICK_START,
                rot_xyzw=START_QUAT)
OBJECTS = [SLEEVE, STICK]
MOVERS = ("stick",)


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
LIFTED = 0.015  # the tube counts as freed once it has been this far above its stowed height
MARK_LO, MARK_HI = 0.004, 0.020  # drawn to the mark: the tube's bottom from 4 mm under to 20 mm over its locked rest height
LOCK_Z_LO, LOCK_Z_HI = 0.0015, 0.0065  # locked: bottom on the notch floor, at most 6.5 mm up (the notch's ceiling stops the lugs)
LOCK_MIN, LOCK_MAX = math.radians(44.0), math.radians(62.0)  # turned this far (counter-clockwise from above) the lugs are in the notch
AXIS_XY, TILT_MAX = 0.004, math.radians(6.0)
OUT_Z = SLEEVE_TOP + 0.002  # a tube whose bottom is above this has left the sleeve
CLEAR = 0.03
STICK_LO = np.array([-0.02, -0.02, 0.0])
STICK_HI = np.array([0.02, 0.02, STICK_TOTAL])
STEPS = ("the stick freed and lifted out of its stowed notch", "drawn up to the red mark", "locked: lugs in the upper notch, turned to the stop, resting on its floor",
         "grippers clear of the stick")


def _tilt(objects):
    return math.acos(max(-1.0, min(1.0, float(_rot(objects["stick"])[2, 2]))))


def _off(objects):
    p = np.asarray(objects["stick"].pos, float)
    return float(np.hypot(p[0] - SLEEVE_XY[0], p[1] - SLEEVE_XY[1]))


def turn(objects) -> float:
    """The stick's turn about the vertical (rad, counter-clockwise from above)."""
    r = _rot(objects["stick"])
    return math.atan2(float(r[1, 0]), float(r[0, 0]))


def _up(objects) -> float:
    return float(objects["stick"].pos[2]) - TOP - MARK_Z


def drawn(objects) -> bool:
    return bool(-MARK_LO <= _up(objects) <= MARK_HI and _off(objects) <= AXIS_XY and _tilt(objects) <= TILT_MAX)


def locked(objects) -> bool:
    return bool(LOCK_Z_LO - 0.003 <= _up(objects) <= LOCK_Z_HI and _off(objects) <= AXIS_XY and _tilt(objects) <= TILT_MAX
                and LOCK_MIN <= turn(objects) <= LOCK_MAX)


def held(objects) -> bool:
    return any(holds(objects, s, "stick", STICK_LO, STICK_HI) for s in ("left", "right"))


def clear_of_stick(objects) -> bool:
    return not pads_near(objects, "stick", STICK_LO, STICK_HI, CLEAR)


def _steps(objects) -> list[bool]:
    lifted = bool(_PROGRESS.mem.get("lifted"))
    lk = bool(locked(objects))
    return [lifted, bool(drawn(objects)), lk, bool(lk and clear_of_stick(objects) and not held(objects))]


def events_now(objects, p) -> list[str]:
    """Rule events as things stand: the stick off the table or drawn out of its sleeve. Also remembers (``p.mem``) that it was freed and lifted."""
    out = []
    z = float(objects["stick"].pos[2])
    if z > STICK_START[2] + LIFTED:
        p.mem["lifted"] = True
    if z < TOP - 0.03:
        out.append("the stick fell off the table")
    if z > TOP + OUT_Z:
        out.append("the stick was pulled right out of its sleeve")
    return out


def trace_line(objects) -> str:
    c = objects["stick"]
    return (f"stick bottom at {(c.pos[0] - SLEEVE_XY[0]) * 1000:.1f},{(c.pos[1] - SLEEVE_XY[1]) * 1000:.1f} mm from the axis, "
            f"{_up(objects) * 1000:.1f} mm from the mark height, tilt {math.degrees(_tilt(objects)):.1f} deg, "
            f"turn {math.degrees(turn(objects)):.0f} deg{'*' if held(objects) else ''}, drawn {drawn(objects)}, locked {locked(objects)}")


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
# The right arm pinches the T handle bar across its 12 mm width (closing along y), turns the stick back to the slots (about 0 degrees), draws it up
# to the mark, turns it counter-clockwise into the upper notch (the stop holds it at about 52), lets it rest on the notch floor, lets go and goes home.
_grip = yam_demo.grip_rotation
SQUEEZE = 0.008
HOLD = -(0.012 - SQUEEZE) / 2
OPEN = -(0.012 + 0.020) / 2
PAD_DEPTH = 0.006
FREE = math.radians(0.0)  # the lugs in line with the slots
LOCKTURN = math.radians(58.0)  # the wrist turns this far (the notch stop holds the stick at about 52)
ROT_S, ROT_F, ROT_L = _grip(START_YAW), _grip(FREE), _grip(LOCKTURN)
HOVER = 0.06
DRAW = MARK_Z + 0.0075  # the tube bottom is commanded this high; it hangs about 4 mm lower in the pads (Isaac), i.e. about 3.5 mm over the notch floor while it turns in


def _tcp(x, y, z_bottom):
    return (x, y, z_bottom + STICK_TOTAL - PAD_DEPTH + PAD_TIP_PAST_TCP)


def left_plan() -> list[Waypoint]:
    return []


def right_plan() -> list[Waypoint]:
    x0, y0 = SLEEVE_XY[0], SLEEVE_XY[1]
    z0 = FLOOR_Z
    t = lambda z: _tcp(x0, y0, TOP + z)  # noqa: E731
    return [
        Waypoint(2.5, t(z0 + HOVER), OPEN, "over the stick handle", rot=ROT_S),
        Waypoint(1.5, t(z0), OPEN, "pads astride the handle bar", rot=ROT_S),
        Waypoint(0.8, t(z0), HOLD, "grip the bar", rot=ROT_S),
        Waypoint(0.4, t(z0), HOLD, "hold", rot=ROT_S),
        Waypoint(2.0, t(z0), HOLD, "turn the stick back to the slots", rot=ROT_F),
        Waypoint(0.4, t(z0), HOLD, "hold", rot=ROT_F),
        Waypoint(2.5, t(DRAW), HOLD, "draw the stick up to the red mark", rot=ROT_F),
        Waypoint(1.4, t(DRAW), HOLD, "hold: the arm settles at the draw height before the turn", rot=ROT_F),
        Waypoint(2.0, t(DRAW), HOLD, "turn the lugs into the upper notch", rot=ROT_L),
        Waypoint(0.5, t(DRAW), HOLD, "hold", rot=ROT_L),
        Waypoint(0.6, t(MARK_Z + 0.0005), HOLD, "let the lugs settle on the notch floor", rot=ROT_L),
        Waypoint(0.6, t(MARK_Z + 0.0005), OPEN, "let go", rot=ROT_L),
        Waypoint(1.2, t(MARK_Z + 0.02), OPEN, "lift away", rot=ROT_L),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The right arm unlocks the stick, draws it to the red mark and locks it."""
    return {"right": right_plan()}
