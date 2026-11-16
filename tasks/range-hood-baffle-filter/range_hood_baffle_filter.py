"""On a kitchen back wall: a range hood's filter panel leaning back 30 degrees above horizontal, with a bottom trough, a top spring latch
(a sliding hook plate with a pull handle) and, lying flat on the counter in front of it, a freshly washed stainless baffle filter
(16 x 10 x 1.75 in, 1.18 kg, drop handles on its face).

The robot has to refit the filter the way a kitchen worker does: carry it by its two handles, stand its bottom edge in the trough with
the top swung out, pull the spring latch's handle up (against its spring) so the top edge can swing in flat to the panel, and let the
latch drop back over the filter's top edge with a click. Two hands carry the 1.18 kg filter (more than one arm should hold); the
latch is pulled by one hand while the other steadies the filter.

The latch is a plate that slides along the panel on a prismatic joint with a spring (150 N/m, 3 N preload, 5.5 N at full travel: estimates in the light-force range of catalogue spring plungers, 3 to 10 N);
with it down, its hook stands 2.5 mm in front of the seated filter's top 14 mm and blocks the top edge from swinging in or out.

Run it:  inspect-robots run --embodiment yam_arena -E task=range_hood_baffle_filter.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "hoodfil"
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
# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases; z above the table) -------------------
FW, FH, FT = 0.4064, 0.254, 0.04445  # the filter: along y, up the panel, thickness (16 x 10 x 1.75 in)
F_MASS = 1.18  # Component Hardware FR51-1016, 2.6 lb
PHI = 60.0  # the panel leans back this far from vertical (30 degrees above horizontal)
XB, ZB = 0.27, 0.02  # the trough's inner corner (a = 0, b = 0)
SPH, CPH = math.sin(math.radians(PHI)), math.cos(math.radians(PHI))
U_H = np.array([SPH, 0.0, CPH])  # up the panel
M_H = np.array([-CPH, 0.0, SPH])  # out of the panel's face
P0 = np.array([XB, 0.0, TOP + ZB])
TH = math.radians(-(90.0 - PHI))  # the filter seated: turned -30 degrees about +y from lying flat
LIP_A, SLOT_B = 0.018, 0.0495
PLATE_Y = -0.15
TAB_Y = -0.235  # the latch's T handle, beyond the filter's end
LATCH_TRAVEL = 0.0165
HANDLE_Y, HANDLE_A, HANDLE_M = 0.16, 0.072, FT / 2 + 0.027  # the drop handles' crossbar centres (filter frame, from its centre)


def hood_pt(a, b, y=0.0):
    return P0 + a * U_H + b * M_H + np.array([0.0, y, 0.0])


def q_y(theta):
    return np.array([0.0, math.sin(theta / 2), 0.0, math.cos(theta / 2)])


FILTER_NOMINAL = np.array([0.115, 0.0, TOP + FT / 2])  # where the demo expects the filter
FILTER_START = np.array([0.115, 0.0, TOP + FT / 2])  # where it lies
HOOD = UsdPart("hood", asset="hood", xy=(XB, 0.0), extent=0.40, mass=3.0, usd_path=str(ASSETS / "hood.usda"),
               pos=(0.0, 0.0, TOP), articulation=True, fixed=True)
FILTER = UsdPart("filter", asset="filter", xy=(FILTER_START[0], 0.0), extent=0.30, mass=F_MASS, usd_path=str(ASSETS / "filter.usda"),
                 pos=tuple(float(v) for v in FILTER_START))
OBJECTS = [HOOD, FILTER]
MOVERS = ("filter",)
FILTER_LO = np.array([-FH / 2, -FW / 2, -FT / 2])
FILTER_HI = np.array([FH / 2, FW / 2, FT / 2 + 0.034])
SEAT_CENTRE = hood_pt(FH / 2, FT / 2)
SEAT_Q = q_y(TH)


def _frame(state):
    r = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))
    return np.asarray(state.pos, float), r[:, 0], r[:, 1], r[:, 2]


def hood_coords(p):
    d = np.asarray(p, float) - P0
    return float(d @ U_H), float(d @ M_H), float(d[1])


def bottom_edge(objects):
    """The filter's bottom back edge (centre of the line) in hood coordinates (a, b, y)."""
    p, uf, yf, mf = _frame(objects["filter"])
    return hood_coords(p - (FH / 2) * uf - (FT / 2) * mf)


def in_trough(objects) -> bool:
    """The filter's bottom edge stands in the trough: its back edge within 4 mm of the floor (a from -4 to 13 mm; a tilted filter
    stands on its front corner) and between the panel and the lip, centred within 30 mm along y."""
    p, uf, yf, mf = _frame(objects["filter"])
    ok = True
    for sb in (0.0, 1.0):  # back and front bottom edges
        a, b, y = hood_coords(p - (FH / 2) * uf + (sb - 0.5) * FT * mf)
        ok = ok and (-0.004 <= a <= 0.045) and (-0.003 <= b <= SLOT_B) and abs(y) <= 0.030
    return bool(ok)


def seated(objects) -> bool:
    """The filter lies flat on the panel with its bottom edge on the trough floor: centre within 3 mm of the panel's face offset,
    6 mm along the panel, 30 mm along y, and within 3 degrees of the panel."""
    p, uf, yf, mf = _frame(objects["filter"])
    a, b, y = hood_coords(p)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(mf @ M_H)))))
    return bool(abs(b - FT / 2) <= 0.003 and abs(a - FH / 2) <= 0.006 and abs(y) <= 0.030 and tilt <= 3.0
                and abs(float(yf[1])) >= math.cos(math.radians(3.0)))


def extra_states(base, out, origin) -> None:
    """The latch plate's slide along the panel (m, 0 = down, at its spring's stop)."""
    art = base.scene["hood"]
    ids = art.find_joints(["Latch"], preserve_order=True)[0]
    out["latch"] = float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])


def _reset_all(env, env_ids, asset_cfg) -> None:
    """Reset event: the latch plate down at its stop."""
    import torch

    if env_ids is None:
        return
    art = env.scene["hood"]
    ids = art.find_joints(["Latch"], preserve_order=True)[0]
    zero = torch.zeros((len(env_ids), len(ids)), device=art.device)
    art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)


def reset_events() -> list:
    from isaaclab.managers import SceneEntityCfg

    return [("hood_reset", _reset_all, {"asset_cfg": SceneEntityCfg("filter")})]


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
CLEAR = 0.03
LIFT_Z = 0.07  # the filter's centre this high above the table: it has been lifted
DROP_Z = 0.05  # ... and back this low (it was on the table flat at 0.022): it was dropped
LATCH_REST = 0.0015  # the latch plate down within this of its spring stop: the hook is over the filter
STEPS = ("the filter's bottom edge stands in the trough", "the filter lies flat on the panel, seated on the trough floor",
         "the spring latch is back down over the filter's top edge", "latched, let go, hands clear of the filter")


def filter_held(objects) -> bool:
    return any(holds(objects, s, "filter", FILTER_LO, FILTER_HI) for s in ("left", "right"))


def parts_free(objects) -> bool:
    if filter_held(objects):
        return False
    return not pads_near(objects, "filter", FILTER_LO, FILTER_HI, CLEAR)


def _steps(objects) -> list[bool]:
    s1 = in_trough(objects)
    s2 = bool(s1 and seated(objects))
    s3 = bool(s2 and float(objects.get("latch", 0.0)) <= LATCH_REST)
    s4 = bool(s3 and parts_free(objects))
    return [s1, s2, s3, s4]


def events_now(objects, p) -> list[str]:
    out = []
    z = float(objects["filter"].pos[2]) - TOP
    if z > LIFT_Z:
        p.mem["lifted"] = True
    if p.mem.get("lifted") and z < DROP_Z:
        out.append("the filter was dropped (below 5 cm after being carried)")
    return out


def trace_line(objects) -> str:
    p, uf, yf, mf = _frame(objects["filter"])
    a, b, y = hood_coords(p)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(mf @ M_H)))))
    return (f"filter at a {(a - FH / 2) * 1000:.1f} b {(b - FT / 2) * 1000:.1f} y {y * 1000:.1f} mm, tilt {tilt:.1f} deg{'*' if filter_held(objects) else ''}, "
            f"latch {float(objects.get('latch', 0.0)) * 1000:.1f} mm, trough {in_trough(objects)}, seated {seated(objects)}")


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
# Both arms take the filter by its two drop handles (pads closing along the panel's up direction across each 15 mm crossbar,
# fingers pointing into the filter's face), lift it off the counter, carry it to the hood with the top swung out 35 degrees and
# stand its bottom front corner in the trough. They swing the top in to 4 degrees; the right hand lets go, takes the latch's
# handle and pulls it up its 20 mm against the spring; the left hand swings the filter flat to the panel; the right hand lets the
# latch go (it drops over the filter's top edge); both hands let go and back away out of the filter's face.
_X, _Y, _Z = np.eye(3)
_grip = yam_demo.grip_rotation


YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # the wrist's roll: the pads close along the same line either way, this keeps each wrist away from its limit


def tilt(deg: float, side: str = "right") -> np.ndarray:
    """A grip straight down (pads closing along x), turned ``deg`` about the world y axis (negative: the fingers point toward +x)."""
    return axis_angle_matrix(_Y, math.radians(deg)) @ _grip(YAW[side])


def ry(theta: float) -> np.ndarray:
    return axis_angle_matrix(_Y, theta)


OPEN = FINGER_OPEN
SQUEEZE = 0.008
HOLD = -(0.015 - SQUEEZE) / 2  # across a 15 mm crossbar, about 20 N a finger
LIFT = 0.06
PSI0 = 35.0  # the top swung out this far when the bottom goes into the trough
PSI1 = 25.0  # the top swung out this far while the latch is pulled (the hook plate would stop it near 12 degrees)
EDGE_A, EDGE_B = 0.0015, 0.002  # the bottom back edge 1.5 mm above the floor and 2 mm off the panel
SEAT_PUSH = 0.0005  # at the end the hand holds the filter this far into the panel
PULL_EXTRA = 0.030  # the hand pulls beyond the latch's stop so that the arm's stiffness delivers the spring force
AIM_X = AIM_Y = AIM_Z = 0.0
SAGX = SAGZ = 0.0


def pose(psi_deg: float, da: float = 0.0, db: float = 0.0):
    """The filter swung out ``psi_deg`` from the panel with its bottom back edge on the trough floor (the front corner
    stands on the floor) plus (da, db) in panel coordinates: (centre, turn about y)."""
    psi = math.radians(psi_deg)
    th = TH - psi
    edge = hood_pt(FT * math.sin(psi) + EDGE_A + da, EDGE_B + db)
    return edge + ry(th) @ np.array([FH / 2, 0.0, FT / 2]), th


def handle(side, centre, th):
    sg = 1.0 if side == "left" else -1.0
    return centre + ry(th) @ np.array([HANDLE_A, sg * HANDLE_Y, HANDLE_M])


def _tcp(p):
    return np.array([p[0] + AIM_X + SAGX, p[1] + AIM_Y, p[2] + AIM_Z + SAGZ])


def tab_tcp():
    return hood_pt(0.2555, 0.079, TAB_Y)


def plan(side):
    c0, th0 = FILTER_NOMINAL, 0.0
    r = lambda th: tilt(math.degrees(th), side)
    H0 = handle(side, c0, th0)
    wps = [
        Waypoint(2.5, _tcp(H0 + np.array([0, 0, 0.07])), OPEN, "over the handle", rot=r(th0)),
        Waypoint(1.5, _tcp(H0), OPEN, "pads astride the crossbar", rot=r(th0)),
        Waypoint(0.8, _tcp(H0), HOLD, "grip the handle", rot=r(th0)),
        Waypoint(1.2, _tcp(handle(side, c0 + np.array([0, 0, LIFT]), th0)), HOLD, "lift the filter off the counter", rot=r(th0)),
    ]
    # carry it up and over the trough while swinging the top out to PSI0
    cM, thM = pose(PSI0, 0.10, 0.07)
    wps.append(Waypoint(3.0, _tcp(handle(side, cM, thM)), HOLD, "carry it to the hood, top swung out", rot=r(thM)))
    cA, thA = pose(PSI0, 0.035, 0.0)
    wps.append(Waypoint(1.5, _tcp(handle(side, cA, thA)), HOLD, "over the trough", rot=r(thA)))
    cB, thB = pose(PSI0, 0.0, 0.0)
    wps.append(Waypoint(2.0, _tcp(handle(side, cB, thB)), HOLD, "stand its bottom edge in the trough", rot=r(thB)))
    wps.append(Waypoint(1.0, _tcp(handle(side, cB, thB)), HOLD, "steady", rot=r(thB)))
    n = 8
    for i in range(1, n + 1):
        psi = PSI0 + (PSI1 - PSI0) * i / n
        c, th = pose(psi)
        wps.append(Waypoint(5.0 / n, _tcp(handle(side, c, th)), HOLD, "swing the top in", rot=r(th)))
    c1, th1 = pose(PSI1)
    if side == "right":
        wps.append(Waypoint(0.8, _tcp(handle(side, c1, th1)), OPEN, "let go of the right handle", rot=r(th1)))
        wps.append(Waypoint(1.2, _tcp(handle(side, c1, th1) + 0.05 * M_H), OPEN, "lift off the handle", rot=r(th1)))
        t = tab_tcp()
        rt = tilt(-(90.0 - PHI), side)
        wps.append(Waypoint(2.0, _tcp(t + 0.06 * M_H), OPEN, "over the latch handle", rot=rt))
        wps.append(Waypoint(1.2, _tcp(t), OPEN, "pads astride the latch handle", rot=rt))
        wps.append(Waypoint(0.8, _tcp(t), HOLD, "grip the latch handle", rot=rt))
        wps.append(Waypoint(2.5, _tcp(t + (LATCH_TRAVEL + PULL_EXTRA) * U_H), HOLD, "pull the latch up against its spring", rot=rt))
        wps.append(Waypoint(3.0, _tcp(t + (LATCH_TRAVEL + PULL_EXTRA) * U_H), HOLD, "hold it up while the filter swings in", rot=rt))
        wps.append(Waypoint(1.5, _tcp(t + (LATCH_TRAVEL + PULL_EXTRA) * U_H), HOLD, "hold", rot=rt))
        wps.append(Waypoint(2.0, _tcp(t + 0.002 * U_H), HOLD, "let the latch down slowly", rot=rt))
        wps.append(Waypoint(0.8, _tcp(t), OPEN, "let go of the latch handle", rot=rt))
        wps.append(Waypoint(1.5, _tcp(t + 0.08 * M_H), OPEN, "back away", rot=rt))
        wps.append(Waypoint(2.5, tuple(_START_TCP[side]), FINGER_OPEN, "home", rot=_START_ROT[side]))
    else:
        wps.append(Waypoint(0.8, _tcp(handle(side, c1, th1)), HOLD, "steady while the latch is pulled", rot=r(th1)))
        wps.append(Waypoint(8.2, _tcp(handle(side, c1, th1)), HOLD, "steady", rot=r(th1)))
        for i in range(1, 7):
            psi = PSI1 * (1.0 - i / 6)
            c, th = pose(psi, 0.0, (-SEAT_PUSH - EDGE_B) * (i / 6))
            wps.append(Waypoint(4.0 / 6, _tcp(handle(side, c, th)), HOLD, "swing the filter flat to the panel", rot=r(th)))
        c2, th2 = pose(0.0, 0.0, -SEAT_PUSH - EDGE_B)
        wps.append(Waypoint(1.0, _tcp(handle(side, c2, th2)), HOLD, "hold it flat", rot=r(th2)))
        wps.append(Waypoint(4.5, _tcp(handle(side, c2, th2)), HOLD, "hold while the latch drops", rot=r(th2)))
        wps.append(Waypoint(0.8, _tcp(handle(side, c2, th2)), OPEN, "let go", rot=r(th2)))
        wps.append(Waypoint(1.5, _tcp(handle(side, c2, th2) + 0.08 * M_H), OPEN, "back away", rot=r(th2)))
        wps.append(Waypoint(2.5, tuple(_START_TCP[side]), FINGER_OPEN, "home", rot=_START_ROT[side]))
    return wps


def demo():
    """Both arms carry the filter to the hood and stand it in the trough; the right hand pulls the latch up while the left swings
    the filter flat; the latch drops back and both let go."""
    return {"left": plan("left"), "right": plan("right")}
