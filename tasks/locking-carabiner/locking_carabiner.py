"""On the station table: a fixed anchor eye (an eye-bolt post on a base) and a ~60 mm aluminium screw-lock
(SCREW-LOCK) locking carabiner lying flat beside it.  The robot picks the carabiner up by its spine, opens the
gate by pushing it against the post, hooks the post through the loop, lets the spring shut the gate, then grips
the knurled screw sleeve with the other hand and turns it a quarter turn until it is fully locked.  Finally it
pull-tests the connection (the eye slides to the gate end of the loop and holds) and lets go.  Idle 0; gate
open 0.25; hooked 0.5; gate closed 0.75; sleeve locked + pull-test held 1 s = 1.0.  Capped: gate not closed,
sleeve not locked, carabiner dropped.

The carabiner is a real articulated rigid body in its own USD (a free root and a spring-hinged gate), with real
contact physics for opening the gate and threading the post.  The screw sleeve's *thread* is declared, the way
the family declares latches and twists: while the left gripper holds the sleeve with the gate shut, the task
accumulates its wrist twist; a quarter turn sets the locked flag the scorer reads.  The sleeve's knurl is modelled
on the gate link in the USD; its turning is reported in the run trace and media rather than integrated as a helix.

Run it:  inspect-robots run --embodiment yam_arena -E task=locking_carabiner.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/locking-carabiner
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

TAG = "lockbiner"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: a free rigid body, an articulation
    (``articulation``: its root link is free) or a fixture (``kinematic``).  ``Asset``'s fields only satisfy the
    task-file checks."""

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
    """One gripper after a control step (world frame, m): its TCP and pad points, the span of its pads along its pointing
    axis, how far apart the pads are, the smaller finger's contact force (N) and the full TCP rotation."""

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
            tcp=tcp, back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])), rot=rot)
    return out


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_lockbiner_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

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
            from isaaclab.managers import EventTermCfg

            env_cfg = configure(env_cfg)
            for name, func, params in reset_events():
                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="reset", params=params))
            for name, func, params in step_events():
                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="interval", interval_range_s=(0.0, 0.0),
                                                           is_global_time=False, params=params))
            return env_cfg

        return with_events

    task_object._lockbiner_usd_parts = True
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


# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
BOARD_XY = (0.31, 0.0)
BOARD_T = 0.008
POST_XY = (BOARD_XY[0] + 0.035, BOARD_XY[1])   # the anchor eye's post axis (x 0.345, y 0)
POST_TOP = 0.040                                # ring axis above the table
POST_R = 0.004
WIRE_R = 0.003
CARAB_START = (0.255, POST_XY[1], TOP + BOARD_T + WIRE_R + 0.0003)  # spine middle; flat, loop pointing +x toward the post
CARAB_MASS = 0.050
GATE_JOINT = "Gate"
ART_NAME = "carabiner"
# carabiner frame (x normal to its plane, y along the spine, z from the loop toward the spine) when flat, loop toward +x:
_R_FLAT = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # x_c = +z, y_c = +y, z_c = -x


def _quat(rot):
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.asarray(rot, float)))

# The screw-lock sleeve centre in the carabiner's local frame (x, y, z), near the gate hinge (y +17, z -54 mm).
SLEEVE_LOCAL = np.array([0.0, 0.008, -0.054])

CARAB = UsdPart("carabiner", asset="carabiner", xy=CARAB_START[:2], extent=0.07, mass=CARAB_MASS,
                usd_path=str(ASSETS / "carabiner.usda"), pos=CARAB_START, rot_xyzw=_quat(_R_FLAT),
                articulation=True)
ANCHOR = UsdPart("anchor", asset="anchor", xy=BOARD_XY, extent=0.34, mass=2.0, usd_path=str(ASSETS / "anchor.usda"),
                 pos=(BOARD_XY[0], BOARD_XY[1], TOP), kinematic=True)
OBJECTS = [ANCHOR, CARAB]
MOVERS = ("carabiner",)


def extra_states(base, out, origin) -> None:
    art = base.scene[ART_NAME]
    ids = art.find_joints([GATE_JOINT], preserve_order=True)[0]
    out["gate"] = float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])
    out["sleeve_turns"] = _LOCK.turns
    out["sleeve_locked"] = _LOCK.locked


def reset_events() -> list:
    return []


def step_events() -> list:
    return []


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


# -- the rule and what is scored ----------------------------------------------------------------------------------------
LIFTED = 0.008
PLANE_TOL = math.radians(45.0)
GATE_CLOSED = math.radians(-3.0)          # gate counts as closed above this angle
GATE_OPENED = math.radians(-12.0)         # gate counts as opened below this angle (it swung inward)
INSIDE = np.array([(-0.017, -0.001), (0.017, -0.001), (0.027, -0.010), (0.027, -0.044), (0.017, -0.054),
                   (-0.017, -0.054), (-0.027, -0.044), (-0.027, -0.010)])
CLEAR = 0.03
CARAB_LO = np.array([-0.0045, -0.031, -0.061])
CARAB_HI = np.array([0.0045, 0.031, 0.0035])
TAUT_Z = -0.045                           # post this far down the loop (toward the gate) = pull-test taut
SLEEVE_R = 0.030                          # m: right TCP within this of the sleeve centre = on the sleeve
SLEEVE_CLOSE = 0.025                      # opening at/below which the right fingers are "gripping the sleeve"
LOCK_TURN = math.radians(75.0)            # the screw sleeve is fully shut after about a quarter turn
LOCK_TOL = math.radians(15.0)             # tolerance on the declared lock window (>= 60 deg counts)
STEPS = ("the gate was opened", "the anchor eye through the carabiner's loop", "gate closed behind the eye",
         "sleeve locked and the pull-test holds")


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _inside(pt) -> bool:
    s = []
    for i in range(len(INSIDE)):
        a, b = INSIDE[i], INSIDE[(i + 1) % len(INSIDE)]
        s.append((b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0]))
    return bool(all(v >= 0 for v in s) or all(v <= 0 for v in s))


def _post_local(objects):
    """The post axis's crossing of the carabiner's plane, in the carabiner's (y, z) frame, or None if the plane is not
    nearly horizontal / the crossing is not between the base and the ring."""
    s = objects["carabiner"]
    p, r = np.asarray(s.pos, float), _rot(s)
    n = r[:, 0]
    if abs(n[2]) < math.cos(PLANE_TOL):
        return None
    z = float(n @ p - n[0] * POST_XY[0] - n[1] * POST_XY[1]) / n[2]
    if not (TOP + BOARD_T - 0.002 <= z <= TOP + POST_TOP - 0.004):
        return None
    q = r.T @ (np.array([POST_XY[0], POST_XY[1], z]) - p)
    return (float(q[1]), float(q[2]))


def threaded(objects) -> bool:
    """The post's axis passes through the carabiner's loop (inside the wires' centre line, between the base and the ring,
    plane within PLANE_TOL of horizontal)."""
    loc = _post_local(objects)
    return bool(loc is not None and _inside(loc))


def taut(objects) -> bool:
    """Pull-test: the post is still inside the loop but has slid to the gate end (pulled against the shut gate)."""
    loc = _post_local(objects)
    return bool(loc is not None and _inside(loc) and loc[1] <= TAUT_Z)


def clipped(objects) -> bool:
    """Hooked on the post (threaded, plane within PLANE_TOL of horizontal) with the gate shut behind it.  The
    carabiner may be lying on the base or lifted for the pull-test; ``threaded`` already rejects one standing on edge."""
    closed = float(objects.get("gate", -1.0)) >= GATE_CLOSED
    return bool(threaded(objects) and closed)


def sleeve_point(objects) -> np.ndarray:
    s = objects["carabiner"]
    return np.asarray(s.pos, float) + _rot(s) @ SLEEVE_LOCAL


def _yaw(mat) -> float:
    return math.atan2(float(mat[1, 0]), float(mat[0, 0]))


def _wrap(a) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _pads(g) -> np.ndarray:
    return g.back + np.linspace(0.0, 1.0, 11)[:, None] * (g.tips - g.back)


def holds(objects, side: str, name: str, lo, hi) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None or g.contact < 0.5 or g.opening < 0.003:
        return False
    s = objects[name]
    q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
    return bool(np.any(np.all((q >= np.asarray(lo) - 0.004) & (q <= np.asarray(hi) + 0.004), axis=1)))


def held(objects) -> bool:
    return any(holds(objects, s, "carabiner", CARAB_LO, CARAB_HI) for s in ("left", "right"))


def clear_of_carabiner(objects) -> bool:
    s = objects["carabiner"]
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
        if np.any(np.all((q >= np.asarray(CARAB_LO) - CLEAR) & (q <= np.asarray(CARAB_HI) + CLEAR), axis=1)):
            return False
    return True


class _Lock:
    """The declared screw-sleeve twist, counted from the right wrist's rotation while it grips the sleeve with the gate
    shut.  Re-grips do not lose the count (last_yaw resets whenever the grip is broken)."""

    def __init__(self):
        self.turns = 0.0
        self.last_yaw = None
        self.locked = False

    def update(self, objects):
        g = objects.get("right_gripper")
        gate = float(objects.get("gate", -1.0))
        on = (g is not None and g.opening <= SLEEVE_CLOSE and gate >= GATE_CLOSED
              and float(np.linalg.norm(g.tcp - sleeve_point(objects))) <= SLEEVE_R)
        if on:
            yaw = _yaw(g.rot)
            if self.last_yaw is not None and clipped(objects):
                self.turns += _wrap(yaw - self.last_yaw)
            self.last_yaw = yaw
        else:
            self.last_yaw = None
        if abs(self.turns) >= LOCK_TURN - LOCK_TOL:
            self.locked = True


_LOCK = _Lock()


class _Progress:
    def __init__(self):
        self.time = -1.0
        self.mem: dict = {}
        self.events: list[str] = []
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def flag(self, t, what):
        if what not in self.events:
            self.events.append(what)
            print(f"[{TAG}] t={t:.2f}s  RULE BROKEN: {what} (scores 0 from here on)", flush=True)

    def update(self, objects):
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        stamp = -1.0 if t is None else t
        _LOCK.update(objects)
        for what in events_now(objects, self):
            self.flag(stamp, what)
        if t is None:
            return
        met = bool(all(_steps(objects)) and not self.events)
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed: timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing 1 s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def events_now(objects, p) -> list[str]:
    out = []
    c = objects["carabiner"]
    if float(objects.get("gate", 0.0)) <= GATE_OPENED:
        p.mem["gate_opened"] = True
    if c.pos[2] < TOP - 0.03:
        out.append("the carabiner fell off the table")
    return out


def _steps(objects) -> list[bool]:
    opened = bool(_PROGRESS.mem.get("gate_opened"))
    thr = bool(threaded(objects))
    clip = bool(clipped(objects))
    done4 = bool(_LOCK.locked and taut(objects) and clip)
    return [opened, thr, clip and opened, done4 and opened and thr and clip]


def trace_line(objects) -> str:
    c = objects["carabiner"]
    loc = _post_local(objects)
    return (f"carabiner spine {(c.pos[0] - POST_XY[0]) * 1000:+.0f},{(c.pos[1] - POST_XY[1]) * 1000:+.0f} mm from the post, "
            f"gate {math.degrees(float(objects.get('gate', float('nan')))):5.1f} deg, "
            f"post-local {'(none)' if loc is None else f'{loc[0]*1000:+.0f},{loc[1]*1000:+.0f}'}, "
            f"threaded {threaded(objects)}, clipped {clipped(objects)}, taut {taut(objects)}, "
            f"sleeve {abs(_LOCK.turns)/(2*math.pi):.2f} turns{' LOCKED' if _LOCK.locked else ''}"
            f"{' HELD' if held(objects) else ''}, events {len(_PROGRESS.events)}")


# -- scoring (shared machinery) ---------------------------------------------------------------------------------------
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
POINTS = 0.25


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in MOVERS)


def still(objects, anchor) -> bool:
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


def steps_done(objects) -> list[bool]:
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  ({trace_line(objects)})", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {trace_line(objects)}", flush=True)
        p.traced = t
    return done


def _caps(objects) -> bool:
    """Caps: a rule event scores 0; releasing a threaded carabiner with the gate open caps at 0.5; starting the
    pull-test before the sleeve is locked caps at 0.5."""
    if _PROGRESS.events:
        return True
    if threaded(objects) and not clipped(objects) and not held(objects):
        return True
    if taut(objects) and clipped(objects) and not _LOCK.locked:
        return True
    return False


def violation(objects) -> bool:
    values = [v for n in MOVERS for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    _PROGRESS.update(objects)
    return bool(_PROGRESS.events)


def success(objects) -> bool:
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6 and not _caps(objects))


def score(objects) -> float:
    done = steps_done(objects)
    if violation(objects) or _caps(objects):
        return 0.0 if _PROGRESS.events else min(round(POINTS * sum(done), 6), 0.5)
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
_grip = yam_demo.grip_rotation
SQUEEZE = 0.007
WIRE = 2 * WIRE_R
HOLD = -(WIRE - SQUEEZE) / 2
OPEN = -(0.022) / 2
ROT = _grip(math.pi / 2)
PAD_BELOW = 0.0015
LIFT = 0.012
PUSH_TO = POST_XY[0] - 0.027          # spine x when the post sits in the middle of the loop
TAUT_TO = PUSH_TO - 0.022             # pull back so the post slides to the gate end of the loop
Y_OFF = 0.007
SLEEVE_Z = TOP + BOARD_T + WIRE_R     # sleeve centre height when the carabiner lies flat
# where the left arm grips the sleeve (world): spine at the clipped pose plus the flat-frame sleeve offset.
SLEEVE_XY = (PUSH_TO + 0.054, POST_XY[1] + 0.015)
LOCK_TWIST = math.radians(100.0)      # wrist sweep for the quarter-turn lock (past the 90 deg threshold)


def _tcp(x, y, z_spine):
    return (x, y, z_spine - PAD_BELOW + PAD_TIP_PAST_TCP)


def left_plan() -> list[Waypoint]:
    return []  # one-handed: the right arm does the whole sequence


def right_plan() -> list[Waypoint]:
    cx, cy, cz = CARAB_START[0], CARAB_START[1], CARAB_START[2] - TOP
    ly = POST_XY[1] + Y_OFF
    sx, sy = SLEEVE_XY
    t = lambda x, y, z: _tcp(x, y, TOP + z)  # noqa: E731
    SLEEVE_ROT = _grip(math.radians(50.0))   # start yaw: a YAM wrist reaches 50-150 deg here with < 3 deg IK error
    LOCKED_ROT = _grip(math.radians(150.0))  # a 100 deg sweep -> the declared quarter-turn lock
    SZ = SLEEVE_Z - TOP + 0.018          # hover 18 mm above the sleeve: the declared turn needs no contact, so the
    #                                       clipped carabiner is not dragged round the post while the wrist turns
    return [
        # A. pick up and hook the carabiner over the anchor eye
        Waypoint(2.5, t(cx, cy, cz + 0.05), OPEN, "over the carabiner's spine", rot=ROT),
        Waypoint(1.5, t(cx, cy, cz), OPEN, "pads astride the spine", rot=ROT),
        Waypoint(0.8, t(cx, cy, cz), HOLD, "grip the spine", rot=ROT),
        Waypoint(0.4, t(cx, cy, cz), HOLD, "hold", rot=ROT),
        Waypoint(1.0, t(cx, cy, cz + LIFT), HOLD, "lift it off the base", rot=ROT),
        Waypoint(1.5, t(cx, ly, cz + LIFT), HOLD, "line the gate up with the post", rot=ROT),
        Waypoint(1.5, t(POST_XY[0] - 0.062, ly, cz + LIFT), HOLD, "gate just short of the post", rot=ROT),
        Waypoint(3.0, t(PUSH_TO, ly, cz + LIFT), HOLD, "push the gate in over the post", rot=ROT),
        Waypoint(1.0, t(PUSH_TO, ly, cz + LIFT), HOLD, "gate closes behind the post", rot=ROT),
        # B. let the clipped carabiner rest on the post; go to the screw sleeve
        Waypoint(0.6, t(PUSH_TO, ly, cz + LIFT), OPEN, "release the clipped carabiner", rot=ROT),
        Waypoint(0.8, t(PUSH_TO, ly, cz + 0.05), OPEN, "lift away", rot=ROT),
        Waypoint(1.6, t(sx, sy, cz + 0.05), OPEN, "over the screw sleeve", rot=SLEEVE_ROT),
        Waypoint(1.0, t(sx, sy, SZ), OPEN, "down to the sleeve", rot=SLEEVE_ROT),
        Waypoint(0.7, t(sx, sy, SZ), -0.007, "grip the sleeve", rot=SLEEVE_ROT),
        Waypoint(0.4, t(sx, sy, SZ), -0.007, "hold the sleeve", rot=SLEEVE_ROT),
        # C. turn the sleeve fully shut (a quarter turn) and let go
        Waypoint(2.4, t(sx, sy, SZ), -0.007, "turn the sleeve shut", rot=LOCKED_ROT),
        Waypoint(0.8, t(sx, sy, SZ), -0.007, "sleeve fully locked", rot=LOCKED_ROT),
        Waypoint(0.5, t(sx, sy, SZ), OPEN, "let go of the sleeve", rot=LOCKED_ROT),
        Waypoint(0.8, t(sx, sy, cz + 0.05), OPEN, "lift away", rot=LOCKED_ROT),
        # D. regrip the spine, pull-test and hold
        Waypoint(1.2, t(PUSH_TO, ly, cz + 0.05), OPEN, "back to the spine", rot=ROT),
        Waypoint(1.2, t(PUSH_TO, ly, cz), OPEN, "down to the spine", rot=ROT),
        Waypoint(0.8, t(PUSH_TO, ly, cz), HOLD, "regrip the spine", rot=ROT),
        Waypoint(0.8, t(PUSH_TO, ly, cz + LIFT), HOLD, "lift the locked carabiner", rot=ROT),
        Waypoint(1.2, t(TAUT_TO, ly, cz + LIFT), HOLD, "pull-test: slide the eye to the gate", rot=ROT),
        Waypoint(1.8, t(TAUT_TO, ly, cz + LIFT), HOLD, "hold the pull-test", rot=ROT),
        Waypoint(0.6, t(TAUT_TO, ly, cz + 0.05), OPEN, "let go", rot=ROT),
        Waypoint(1.2, t(TAUT_TO, ly, cz + 0.05), OPEN, "lift away", rot=ROT),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    return {"left": left_plan(), "right": right_plan()}


def trace(objects) -> str:
    return trace_line(objects)
