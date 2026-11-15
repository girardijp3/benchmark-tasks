"""On the station table: a tie-down board (200 x 340 x 8 mm) with a U-shaped steel staple fixed on it (two 6 mm legs 90 mm
apart, the bar between their tops 30 mm up) and, 90 mm behind the staple's near leg, a spring-gate key-ring carabiner (5 mm
wire, 24 g) lying flat on the board with its gate end toward the staple.

The task is to clip the carabiner to the staple: pick the carabiner up by its spine, push it forward so the staple's leg
pushes the spring gate inward and passes through the opening, let the gate close with the leg inside the loop, and put the
carabiner down. Being linked to the closed staple, it can no longer be lifted off. The carabiner is an articulation: a free
body and a gate on a revolute hinge (spring 0.04 N m/rad, 60 degrees inward, stopped when closed). Nothing is scripted: all
effects are physical.

Run it:  inspect-robots run --embodiment yam_arena -E task=carabiner_clip_bar.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "carabiner"
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
BOARD_XY = (0.31, -0.20)  # the board's middle
BOARD_T = 0.008
LEG_A = (BOARD_XY[0] + 0.035, BOARD_XY[1] - 0.02)  # the near leg's axis (the carabiner is clipped round this one)
LEG_B = (LEG_A[0], LEG_A[1] + 0.09)
LEG_TOP = 0.030  # the staple bar's axis above the table
LEG_R = 0.003
WIRE_R = 0.0025
CARAB_START = (0.255, LEG_A[1], TOP + BOARD_T + WIRE_R + 0.0003)  # the spine's middle; it lies flat, the loop pointing +x (toward the staple)
CARAB_MASS = 0.024
GATE_JOINT = "Gate"
ART_NAME = "carabiner"
# the carabiner's frame (x normal to its plane, y along the spine, z from the loop toward the spine) when it lies flat, loop toward +x:
_R_FLAT = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # columns: x_c = +z, y_c = +y, z_c = -x


def _quat(rot):
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.asarray(rot, float)))


CARAB = UsdPart("carabiner", asset="carabiner", xy=CARAB_START[:2], extent=0.07, mass=CARAB_MASS, usd_path=str(ASSETS / "carabiner.usda"),
                pos=CARAB_START, rot_xyzw=_quat(_R_FLAT), articulation=True)
ANCHOR = UsdPart("anchor", asset="anchor", xy=BOARD_XY, extent=0.34, mass=2.0, usd_path=str(ASSETS / "anchor.usda"),
                 pos=(BOARD_XY[0], BOARD_XY[1], TOP), kinematic=True)
OBJECTS = [ANCHOR, CARAB]
MOVERS = ("carabiner",)


def extra_states(base, out, origin) -> None:
    """The gate's angle (rad, 0 closed, negative open)."""
    art = base.scene[ART_NAME]
    ids = art.find_joints([GATE_JOINT], preserve_order=True)[0]
    out["gate"] = float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])


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
LIFTED = 0.008  # the carabiner counts as picked up once its spine has been this far above its start
PLANE_TOL = math.radians(45.0)  # the loop's plane must be within this of horizontal
GATE_CLOSED = math.radians(-3.0)  # the gate counts as closed above this angle
# the loop's inside, as seen by the leg's axis (y, z in the carabiner's frame, m): inside the wires' centre line (the leg can
# only get there by passing the gate; a closed gate then keeps it there)
INSIDE = np.array([(-0.013, -0.001), (0.013, -0.001), (0.021, -0.0085), (0.021, -0.0355), (0.013, -0.043), (-0.013, -0.043),
                   (-0.021, -0.0355), (-0.021, -0.0085)])
CLEAR = 0.03
CARAB_LO = np.array([-0.0035, -0.026, -0.050])
CARAB_HI = np.array([0.0035, 0.026, 0.0035])
STEPS = ("the carabiner lifted off the board", "the staple's near leg through the carabiner's loop",
         "clipped: the gate closed behind the leg and the carabiner lying on the board", "grippers clear of the carabiner")


def _inside(pt) -> bool:
    """``pt`` (y, z) lies inside the convex polygon INSIDE (counter-clockwise or clockwise)."""
    s = []
    for i in range(len(INSIDE)):
        a, b = INSIDE[i], INSIDE[(i + 1) % len(INSIDE)]
        s.append((b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0]))
    return bool(all(v >= 0 for v in s) or all(v <= 0 for v in s))


def threaded(objects) -> bool:
    """The near leg's axis (vertical) crosses the carabiner's plane at a point inside its loop, between the board and the staple's bar, the plane within PLANE_TOL of horizontal."""
    s = objects["carabiner"]
    p, r = np.asarray(s.pos, float), _rot(s)
    n = r[:, 0]
    if abs(n[2]) < math.cos(PLANE_TOL):
        return False
    z = float(n @ p - n[0] * LEG_A[0] - n[1] * LEG_A[1]) / n[2]
    if not (TOP + BOARD_T - 0.002 <= z <= TOP + LEG_TOP - 0.004):
        return False
    q = r.T @ (np.array([LEG_A[0], LEG_A[1], z]) - p)
    return _inside((q[1], q[2]))


def clipped(objects) -> bool:
    """Threaded on the leg, the gate closed and the carabiner lying on the board (its spine within 8 mm of the board, plane within 25 degrees of horizontal)."""
    s = objects["carabiner"]
    p = np.asarray(s.pos, float)
    flat = abs(_rot(s)[2, 0]) >= math.cos(math.radians(25.0))
    closed = float(objects.get("gate", -1.0)) >= GATE_CLOSED
    return bool(threaded(objects) and closed and flat and p[2] - (TOP + BOARD_T) <= 0.008)


def held(objects) -> bool:
    return any(holds(objects, s, "carabiner", CARAB_LO, CARAB_HI) for s in ("left", "right"))


def clear_of_carabiner(objects) -> bool:
    return not pads_near(objects, "carabiner", CARAB_LO, CARAB_HI, CLEAR)


def _steps(objects) -> list[bool]:
    lifted = bool(_PROGRESS.mem.get("lifted"))
    thr = bool(threaded(objects))
    clip = bool(clipped(objects))
    return [lifted, thr, clip, bool(clip and clear_of_carabiner(objects) and not held(objects))]


def events_now(objects, p) -> list[str]:
    """Rule events as things stand. Also remembers (``p.mem``) that the carabiner was lifted off the board."""
    out = []
    if objects["carabiner"].pos[2] > CARAB_START[2] + LIFTED:
        p.mem["lifted"] = True
    if objects["carabiner"].pos[2] < TOP - 0.03:
        out.append("the carabiner fell off the table")
    return out


def trace_line(objects) -> str:
    c = objects["carabiner"]
    return (f"carabiner spine at {(c.pos[0] - LEG_A[0]) * 1000:.0f},{(c.pos[1] - LEG_A[1]) * 1000:.0f} mm from the leg, "
            f"{(c.pos[2] - TOP) * 1000:.0f} mm up, gate {math.degrees(float(objects.get('gate', float('nan')))):.1f} deg"
            f"{'*' if held(objects) else ''}, threaded {threaded(objects)}, clipped {clipped(objects)}")


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
# The right arm pinches the spine across its 5 mm wire (closing along x), lifts the carabiner 12 mm, pushes it forward so the
# staple's near leg meets the gate near its free end, pushes the gate in and passes through until the leg is in the middle of the
# loop, lets the gate close, sets the carabiner down on the board, lets go and goes home.
_grip = yam_demo.grip_rotation
SQUEEZE = 0.006  # grips close the fingers this far inside the wire's thickness
WIRE = 2 * WIRE_R
HOLD = -(WIRE - SQUEEZE) / 2
OPEN = -(0.020) / 2
ROT = _grip(math.pi / 2)  # pointing down, the fingers closing along x (across the spine wire)
PAD_BELOW = 0.0015  # the pad tips go this far below the spine's middle
LIFT = 0.012  # the carabiner is carried this far above where it lies
PUSH_TO = LEG_A[0] - 0.022  # the spine's x when the leg is in the middle of the loop (the loop is 44 mm long)
Y_OFF = 0.006  # the carabiner's middle is this far toward the hinge side of the leg (the leg meets the gate near its free end)


def _tcp(x, y, z_spine):
    return (x, y, z_spine - PAD_BELOW + PAD_TIP_PAST_TCP)


def left_plan() -> list[Waypoint]:
    return []


def right_plan() -> list[Waypoint]:
    cx, cy, cz = CARAB_START[0], CARAB_START[1], CARAB_START[2] - TOP
    ly = LEG_A[1] + Y_OFF
    t = lambda x, y, z: _tcp(x, y, TOP + z)  # noqa: E731
    return [
        Waypoint(2.5, t(cx, cy, cz + 0.05), OPEN, "over the carabiner's spine", rot=ROT),
        Waypoint(1.5, t(cx, cy, cz), OPEN, "pads astride the spine", rot=ROT),
        Waypoint(0.8, t(cx, cy, cz), HOLD, "grip the spine", rot=ROT),
        Waypoint(0.4, t(cx, cy, cz), HOLD, "hold", rot=ROT),
        Waypoint(1.0, t(cx, cy, cz + LIFT), HOLD, "lift it off the board", rot=ROT),
        Waypoint(1.5, t(cx, ly, cz + LIFT), HOLD, "line the gate up with the leg", rot=ROT),
        Waypoint(1.5, t(LEG_A[0] - 0.052, ly, cz + LIFT), HOLD, "gate just short of the leg", rot=ROT),
        Waypoint(3.0, t(PUSH_TO, ly, cz + LIFT), HOLD, "push the gate in over the leg", rot=ROT),
        Waypoint(1.0, t(PUSH_TO, ly, cz + LIFT), HOLD, "gate closes behind the leg", rot=ROT),
        Waypoint(1.0, t(PUSH_TO, ly, cz + 0.0005), HOLD, "set it down", rot=ROT),
        Waypoint(0.6, t(PUSH_TO, ly, cz + 0.0005), OPEN, "let go", rot=ROT),
        Waypoint(1.2, t(PUSH_TO, ly, cz + 0.05), OPEN, "lift away", rot=ROT),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The right arm clips the carabiner to the staple."""
    return {"right": right_plan()}
