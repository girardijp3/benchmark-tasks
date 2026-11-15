"""On the station table: a small sliding-sash window of the kind used in garden sheds and greenhouses: a pine frame in a 40 mm sill
(opening 480 x 270 mm) with a fixed pane in the back lane and, in the front lane, a swollen pine sash (250 x 270 x 20 mm, 1.5 kg) that
stands 150 mm from the left jamb, which is its end stop. The sash is a free body resting on the sill in its lane and held only by
the lane walls; its drag is the dry friction of pine on the sill (0.40: a pull of about 6 N, the break-away force of a swollen sash,
checked with a spring balance). Its only handle is a plastic pull tab (36 x 10 x 20 mm) on a spring hinge on the sash's front face:
the tab is brittle and bends if it is pulled by its tip.

The task is to close the window: slide the sash to the left jamb by its tab, without bending the tab and without pushing the sash
by anything but the tab. Nothing is scripted but the scene: the tab's flex is a spring hinge (0.8 N m/rad) and a bend of more than
10 degrees counts as a broken tab (task code).

Run it:  inspect-robots run --embodiment yam_arena -E task=sticky_sash_window.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "sash"
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
FRAME_XY = (0.36, 0.02)  # the window's middle on the table (the front lane's middle in x, the opening's middle in y)
SILL_H = 0.040
OPEN_HALF = 0.240  # the opening's half width: the left jamb's face (the end stop) is at FRAME_XY[1] + OPEN_HALF
SASH_W, SASH_H, SASH_T = 0.250, 0.270, 0.020
SASH_MASS = 1.5
TRAVEL = 0.150  # the sash's middle starts this far from where it ends
START_Y = OPEN_HALF - SASH_W / 2 - TRAVEL  # the sash's middle at the start (frame frame)
REST_Z = TOP + SILL_H + 0.0003  # the sash origin's height on the sill
SASH_START = (FRAME_XY[0], FRAME_XY[1] + START_Y, REST_Z)
TAB_HINGE = np.array([-0.010, 0.080, 0.030])  # in the sash frame
TAB_L, TAB_W, TAB_H = 0.036, 0.010, 0.020
TAB_LO = np.array([-0.010 - TAB_L, TAB_HINGE[1] - TAB_W / 2, TAB_HINGE[2] - TAB_H / 2])
TAB_HI = np.array([-0.010, TAB_HINGE[1] + TAB_W / 2, TAB_HINGE[2] + TAB_H / 2])
HANDLE_JOINT = "Handle"
ART_NAME = "sash"

FRAME = UsdPart("frame", asset="frame", xy=FRAME_XY, extent=0.35, mass=None, usd_path=str(ASSETS / "window_frame.usda"),
                pos=(FRAME_XY[0], FRAME_XY[1], TOP), kinematic=True)
SASH = UsdPart("sash", asset="sash", xy=FRAME_XY, extent=0.15, mass=SASH_MASS, usd_path=str(ASSETS / "sash.usda"),
               pos=SASH_START, articulation=True)
OBJECTS = [FRAME, SASH]
MOVERS = ("sash",)


def extra_states(base, out, origin) -> None:
    """The tab hinge's angle (rad, 0 straight)."""
    art = base.scene[ART_NAME]
    ids = art.find_joints([HANDLE_JOINT], preserve_order=True)[0]
    out["tab"] = float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])


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
STOP_Y = FRAME_XY[1] + OPEN_HALF  # the end stop's face
MOVED = 0.010  # step 1: the sash is this far along from where it started
HALF_WAY = 0.075  # step 2
AT_STOP = 0.003  # step 3: the sash's left edge within this of the stop ...
IN_LANE_X = 0.004  # ... in its lane,
SEATED_Z = 0.004  # ... on the sill,
LEAN = math.radians(4.0)  # ... and square
BENT = math.radians(10.0)  # a tab bent this far has broken
STEP_MOVE = 0.0015  # the sash moves this far (y) in one control step: it is being pushed
LEFT_TRACK_Z = 0.010  # lifted this far off the sill: out of its lane
CLEAR = 0.03
PUSH_NEAR = 0.006  # a gripper that does not hold the tab, with its pads this near the sash body while the sash moves, is pushing it
SASH_LO = np.array([-0.010, -SASH_W / 2, 0.0])
SASH_HI = np.array([0.010, SASH_W / 2, SASH_H])
STEPS = ("sash started along the sill", "sash over half way to the stop", "sash closed against the stop in its lane",
         "grippers let go of the tab")


def sash_y(objects) -> float:
    return float(objects["sash"].pos[1])


def sash_edge(objects) -> float:
    return sash_y(objects) + SASH_W / 2


def lean(objects) -> float:
    return math.acos(max(-1.0, min(1.0, float(_rot(objects["sash"])[2, 2]))))


def in_lane(objects) -> bool:
    p = np.asarray(objects["sash"].pos, float)
    return bool(abs(p[0] - FRAME_XY[0]) <= IN_LANE_X and abs(p[2] - REST_Z) <= SEATED_Z and lean(objects) <= LEAN)


def tab_bent(objects) -> bool:
    return abs(float(objects.get("tab", 0.0))) > BENT


def tab_held(objects) -> bool:
    return any(holds(objects, s, "sash", TAB_LO, TAB_HI) for s in ("left", "right"))


def clear_of_tab(objects) -> bool:
    return not pads_near(objects, "sash", TAB_LO, TAB_HI, CLEAR)


def _steps(objects) -> list[bool]:
    moved = sash_y(objects) - SASH_START[1]
    closed = bool(sash_edge(objects) >= STOP_Y - AT_STOP and in_lane(objects) and not tab_bent(objects))
    return [bool(moved >= MOVED), bool(moved >= HALF_WAY), closed,
            bool(closed and clear_of_tab(objects) and not tab_held(objects))]


def _pushed_by_other_gripper(objects) -> bool:
    """A gripper that is not holding the tab has its pads within PUSH_NEAR of the sash's body (not just its tab)."""
    s = objects["sash"]
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None or holds(objects, side, "sash", TAB_LO, TAB_HI):
            continue
        q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
        if np.any(np.all((q >= SASH_LO - PUSH_NEAR) & (q <= SASH_HI + PUSH_NEAR), axis=1)):
            return True
    return False


def events_now(objects, p) -> list[str]:
    """Rule events as things stand: a bent tab, a sash moved by anything but a grip on the tab, a sash out of its lane."""
    out = []
    if tab_bent(objects):
        out.append("the tab was bent past 10 degrees")
    y = sash_y(objects)
    last = p.mem.get("y")
    p.mem["y"] = y
    if last is not None and abs(y - last) > STEP_MOVE and not tab_held(objects):
        out.append("the sash was moved without a grip on its tab")
    if last is not None and abs(y - last) > STEP_MOVE and _pushed_by_other_gripper(objects):
        out.append("the sash was pushed by a gripper that does not hold its tab")
    if objects["sash"].pos[2] > REST_Z + LEFT_TRACK_Z or lean(objects) > math.radians(15.0):
        out.append("the sash came out of its lane")
    if objects["sash"].pos[2] < TOP - 0.03:
        out.append("the sash fell off the table")
    return out


def trace_line(objects) -> str:
    return (f"sash {1000 * (sash_y(objects) - SASH_START[1]):.0f} mm along, edge {1000 * (STOP_Y - sash_edge(objects)):.1f} mm from the stop, "
            f"tab {math.degrees(float(objects.get('tab', float('nan')))):.1f} deg, held {tab_held(objects)}")


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
# The left arm, pointing ahead and 25 degrees down (the tab is only 70 mm up), fingers closing along y across the tab 14 mm out
# from its hinge; grips it, pulls the sash 150 mm along the sill at the stop, lets go, backs off and goes home.
_grip = yam_demo.grip_rotation
_Y = np.array([0.0, 1.0, 0.0])
GRIP_TILT = math.radians(25.0)
GRIP_ROT = axis_angle_matrix(_Y, -math.pi / 2 + GRIP_TILT) @ _grip(0.0)
HOLD = -(TAB_W / 2 - 0.003)
OPEN = -(TAB_W + 0.030) / 2
GRIP_D = 0.014  # the TCP this far out from the hinge
OVERSHOOT = 0.030  # the arm lags the sash by about 27 mm under its 6 N drag


def _tab_tcp(origin) -> np.ndarray:
    return np.asarray(origin, float) + TAB_HINGE + np.array([-GRIP_D, 0.0, 0.0])


def left_plan() -> list[Waypoint]:
    rot = GRIP_ROT
    pick = _tab_tcp(SASH_START)
    end = pick + (0.0, TRAVEL + OVERSHOOT, 0.0)
    return [
        Waypoint(2.5, pick - 0.06 * rot[:, 2], FINGER_OPEN, "in front of the tab", rot=rot),
        Waypoint(1.5, pick, OPEN, "pads astride the tab", rot=rot),
        Waypoint(0.8, pick, HOLD, "grip the tab", rot=rot),
        Waypoint(5.0, end, HOLD, "pull the sash to the stop", rot=rot),
        Waypoint(0.5, end, HOLD, "hold", rot=rot),
        Waypoint(0.6, end, OPEN, "let go", rot=rot),
        Waypoint(1.0, end - 0.06 * rot[:, 2], OPEN, "back off", rot=rot),
        Waypoint(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


def demo():
    """The left arm pulls the sash shut by its tab."""
    return {"left": left_plan()}
