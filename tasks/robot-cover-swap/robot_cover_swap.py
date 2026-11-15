"""On the station table: the electronics bay of a small mobile robot, a 190 x 140 mm deck 60 mm above the table on the
robot's body (fixed to the table), with three headed posts (5 mm shaft, 12 mm head). Its grey top cover (ABS, 150 x 100
mm, a 20 mm skirt, a 3 mm plate with a finger rib on top, 100 g) is cracked: it sits locked on the posts, its three
keyholes slid 10 mm toward the robot so each post is at the end of its 8 mm slot, the post heads holding the plate
down. A new white cover of the same size lies on a parts rack (30 mm tall, fixed) beside the robot, and an empty scrap
tray (174 x 124 mm inside, 35 mm walls, fixed) stands on a riser.

The right arm takes the cracked cover off (pushes it 10 mm away from the robot until the posts are in the keyholes'
big holes, lifts it clear), puts it in the scrap tray, then fits the new cover: lowers it onto the posts, slides it
10 mm toward the robot until the posts reach the slots' ends, lets go and goes back.

The covers are free rigid bodies from ``assets/cover_*.usda`` (box colliders); the deck, the rack and the tray are fixed
rigid bodies. Nothing is spring-held: the cover is held down only by the post heads over the 8 mm slots.

Run it:  inspect-robots run --embodiment yam_arena -E task=robot_cover_swap.py --scorer task_score ...

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

    @property
    def free(self) -> bool:
        return not self.kinematic


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
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False)
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
TAG = "cover_swap"

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
DECK_XY = (0.34, -0.10)
DECK_TOP = TOP + 0.060  # the deck's top face (the cover's skirt rests on it)
RACK_XY = (0.20, -0.30)
RACK_TOP = TOP + 0.030
TRAY_XY = (0.22, 0.07)
TRAY_FLOOR = TOP + 0.030 + 0.003  # the tray's floor top
SLIDE = 0.010  # the keyholes' slide: the cover goes this far toward the robot (-x) to lock
REST = 0.0003  # a cover starts this far above what it rests on
COVER_L, COVER_W, SKIRT_H, PLATE_TOP = 0.150, 0.100, 0.020, 0.023
RIB_TOP = PLATE_TOP + 0.014

DECK = UsdPart("deck", asset="deck", xy=DECK_XY, extent=0.19, mass=3.0, usd_path=str(ASSETS / "bay_deck.usda"),
               pos=(*DECK_XY, DECK_TOP), kinematic=True)
RACK = UsdPart("rack", asset="rack", xy=RACK_XY, extent=0.17, mass=2.0, usd_path=str(ASSETS / "parts_rack.usda"),
               pos=(*RACK_XY, RACK_TOP), kinematic=True)
TRAY = UsdPart("tray", asset="tray", xy=TRAY_XY, extent=0.18, mass=1.0, usd_path=str(ASSETS / "scrap_tray.usda"),
               pos=(*TRAY_XY, TOP + 0.030), kinematic=True)
OLD = UsdPart("old", asset="old", xy=DECK_XY, extent=0.15, mass=0.10, usd_path=str(ASSETS / "cover_cracked.usda"),
              pos=(*DECK_XY, DECK_TOP + REST))
NEW = UsdPart("new", asset="new", xy=RACK_XY, extent=0.15, mass=0.10, usd_path=str(ASSETS / "cover_new.usda"),
              pos=(*RACK_XY, RACK_TOP + REST))
OBJECTS = [DECK, RACK, TRAY, OLD, NEW]


def extra_states(base, out, origin) -> None:
    """Nothing beyond each part's pose."""


def reset_events() -> list:
    """No reset events."""
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


# -- scoring ----------------------------------------------------------------------------------------------------------
LOCK_X, LOCK_Y = 0.002, 0.0025  # a locked cover's middle within this of the deck's middle (the slots leave 0.5 mm in x)
LOCK_Z = (-0.001, 0.0015)  # its skirt's lower edge within this of the deck's top face
LOCK_YAW, LOCK_TILT = math.radians(4.0), math.radians(2.5)
AWAY = 0.17  # m: the old cover's middle at least this far from the deck's middle (the bay is clear)
IN_TRAY_X, IN_TRAY_Y = 0.011, 0.011  # the old cover's middle within this of the tray's middle (the tray leaves 12 mm)
IN_TRAY_Z = (-0.002, 0.012)  # its skirt's lower edge above the tray's floor
IN_TRAY_TILT = math.radians(6.0)
CLEAR = 0.03  # m: let go = no pad within this of either cover
COV_LO = np.array([-0.075, -0.050, -0.002])  # a cover's extent (its frame, m), rib included
COV_HI = np.array([0.075, 0.050, RIB_TOP])
GRIP_FORCE, GRIP_OPENING, GRIP_SLACK = 0.5, 0.008, 0.003
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
STEPS = ("cracked cover lifted off and clear of the bay", "cracked cover lying in the scrap tray",
         "new cover locked on the bay", "let go of both covers")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, float))


def _frame(objects, name) -> tuple[np.ndarray, np.ndarray]:
    c = objects[name]
    return np.asarray(c.pos, float), _rot(c.quat_xyzw)


def tilt(objects, name) -> float:
    _, r = _frame(objects, name)
    return math.acos(max(-1.0, min(1.0, float(r[2, 2]))))


def yaw(objects, name) -> float:
    _, r = _frame(objects, name)
    return math.atan2(float(r[1, 0]), float(r[0, 0]))


def locked(objects, name) -> bool:
    """The cover at ``name`` sits locked on the deck: its middle within LOCK_X, LOCK_Y of the deck's, its skirt on the
    deck's top face, within LOCK_YAW and LOCK_TILT of square. The posts' heads then hold it down."""
    pos, _ = _frame(objects, name)
    dz = pos[2] - DECK_TOP
    return bool(abs(pos[0] - DECK_XY[0]) <= LOCK_X and abs(pos[1] - DECK_XY[1]) <= LOCK_Y
                and LOCK_Z[0] <= dz <= LOCK_Z[1] and abs(yaw(objects, name)) <= LOCK_YAW
                and tilt(objects, name) <= LOCK_TILT)


def old_clear(objects) -> bool:
    pos, _ = _frame(objects, "old")
    return bool(math.hypot(pos[0] - DECK_XY[0], pos[1] - DECK_XY[1]) >= AWAY)


def old_in_tray(objects) -> bool:
    pos, _ = _frame(objects, "old")
    dz = pos[2] - TRAY_FLOOR
    return bool(abs(pos[0] - TRAY_XY[0]) <= IN_TRAY_X and abs(pos[1] - TRAY_XY[1]) <= IN_TRAY_Y
                and IN_TRAY_Z[0] <= dz <= IN_TRAY_Z[1] and tilt(objects, "old") <= IN_TRAY_TILT)


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def held(objects, name) -> bool:
    """A gripper holds the cover: both fingers press, the pads are apart, and a point of the pads is inside the
    cover's outline (its box, rib included)."""
    pos, r = _frame(objects, name)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - pos)
            if np.all(q >= COV_LO - GRIP_SLACK) and np.all(q <= COV_HI + GRIP_SLACK):
                return True
    return False


def let_go(objects) -> bool:
    for name in ("old", "new"):
        pos, r = _frame(objects, name)
        for g in ("left_gripper", "right_gripper"):
            gripper = objects.get(g)
            if gripper is None:
                continue
            for p in _pads(gripper):
                q = r.T @ (p - pos)
                if np.all(q >= COV_LO - CLEAR) and np.all(q <= COV_HI + CLEAR):
                    return False
    return True


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in ("old", "new"))


def still(objects, anchor) -> bool:
    """Both covers within STILL_MOVE and STILL_TURN of their poses in ``anchor``."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper has held the cracked
    cover and the new cover, and since when every full-success condition has held without a break (``met_since``,
    ``None`` while one fails) with the poses when they began to hold (``anchor``). Any lapse restarts the count.
    Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.old_held = False
        self.new_held = False
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
        self.old_held |= held(objects, "old")
        self.new_held |= held(objects, "new")
        if t is None:  # no clock: nothing can be timed
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
    """Each of ``STEPS`` as things stand, from the episode's progress so far."""
    p = _PROGRESS
    clear = bool(p.old_held and old_clear(objects))
    tray = bool(clear and old_in_tray(objects))
    new = bool(tray and p.new_held and locked(objects, "new"))
    return [clear, tray, new, bool(new and let_go(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand. The cracked cover counts as off the bay only after a gripper held it, in the
    tray only with that, the new cover as locked only after a gripper held it and with the cracked one in the tray, and
    step 4 only with step 3."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        po, _ = _frame(objects, "old")
        pn, _ = _frame(objects, "new")
        line = (f"(old from deck {(po[0] - DECK_XY[0]) * 1000:+.0f} {(po[1] - DECK_XY[1]) * 1000:+.0f} mm "
                f"{'held' if held(objects, 'old') else 'free'}, tray off {(po[0] - TRAY_XY[0]) * 1000:+.0f} "
                f"{(po[1] - TRAY_XY[1]) * 1000:+.0f} mm; new from deck {(pn[0] - DECK_XY[0]) * 1000:+.1f} "
                f"{(pn[1] - DECK_XY[1]) * 1000:+.1f} {(pn[2] - DECK_TOP) * 1000:+.1f} mm yaw "
                f"{math.degrees(yaw(objects, 'new')):+.1f} {'held' if held(objects, 'new') else 'free'})")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: a cover off the table, or a state that is not finite."""
    po, _ = _frame(objects, "old")
    pn, _ = _frame(objects, "new")
    if not np.all(np.isfinite([*po, *pn])):
        return True
    return bool(po[2] < TOP - 0.03 or pn[2] < TOP - 0.03)


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S: all four steps done, no violation, and both
    covers within STILL_MOVE and STILL_TURN of where they were when that second began."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 for a violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Right arm, straight down onto a cover's rib, the fingers closing across the rib (along y). The cracked cover is pushed
# SLIDE away from the robot, lifted clear of the posts, carried over the tray and lowered into it; the new cover is
# lifted off the rack, lowered onto the posts with its keyholes' big holes over them, pulled SLIDE toward the robot,
# let go, and the arm goes home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (about 20 N a finger)
RIB_W = 0.012
HOLD = -(RIB_W - SQUEEZE) / 2
OPEN = -(RIB_W + 0.022) / 2
GRIP_UP = 0.038  # the TCP this far above a cover's skirt bottom: the pad tips 25 mm up, 2 mm above the plate
OVER = 0.05  # hover / carry height above the resting height
POST_CLEAR = 0.03  # lifted this far the plate clears the post heads
INSERT_AIM = 0.0025
OVERSLIDE = 0.007  # the slide is commanded this far past the lock: the posts meet the slots' ends first
SLIDE_LIFT = 0.0007  # a cover is eased this far off the deck for the slide (the heads leave the plate 1 mm)


def _tcp(xy, z0) -> tuple:
    return (xy[0], xy[1], z0 + GRIP_UP)


def right_plan() -> list[Waypoint]:
    old0 = DECK_TOP + REST
    new0 = RACK_TOP + REST
    tray0 = TRAY_FLOOR + 0.001
    d = DECK_XY
    unlocked = (d[0] + SLIDE, d[1])
    lock = (d[0] - OVERSLIDE, d[1])
    aim = (d[0] + SLIDE + INSERT_AIM, d[1])  # the arm sags 2-3 mm short of its target here: aimed past the holes' middle
    return [
        # the cracked cover: push it off its slots' ends, lift it clear
        Waypoint(2.5, _tcp(d, old0 + OVER), OPEN, "over the cracked cover"),
        Waypoint(1.5, _tcp(d, old0), OPEN, "around its rib"),
        Waypoint(0.8, _tcp(d, old0), HOLD, "grip it"),
        Waypoint(0.5, _tcp(d, old0 + SLIDE_LIFT), HOLD, "ease it up off the deck"),
        Waypoint(1.2, _tcp(unlocked, old0 + SLIDE_LIFT), HOLD, "push it off the slots' ends"),
        Waypoint(1.0, _tcp(unlocked, old0 + POST_CLEAR), HOLD, "lift it off the posts"),
        Waypoint(1.0, _tcp(unlocked, old0 + OVER), HOLD, "lift clear"),
        Waypoint(2.0, _tcp(TRAY_XY, old0 + OVER), HOLD, "over the scrap tray"),
        Waypoint(1.8, _tcp(TRAY_XY, tray0), HOLD, "lower it into the tray"),
        Waypoint(0.6, _tcp(TRAY_XY, tray0), OPEN, "let go"),
        Waypoint(1.0, _tcp(TRAY_XY, tray0 + OVER), OPEN, "lift away"),
        # the new cover
        Waypoint(2.0, _tcp(RACK_XY, new0 + OVER), OPEN, "over the new cover"),
        Waypoint(1.5, _tcp(RACK_XY, new0), OPEN, "around its rib"),
        Waypoint(0.8, _tcp(RACK_XY, new0), HOLD, "grip it"),
        Waypoint(1.0, _tcp(RACK_XY, new0 + OVER), HOLD, "lift it"),
        Waypoint(2.0, _tcp(aim, old0 + OVER), HOLD, "over the posts"),
        Waypoint(2.0, _tcp(aim, old0), HOLD, "lower it onto the posts"),
        Waypoint(0.4, _tcp(aim, old0), HOLD, "settle"),
        Waypoint(0.5, _tcp(aim, old0 + SLIDE_LIFT), HOLD, "ease it up off the deck"),
        Waypoint(1.2, _tcp(lock, old0 + SLIDE_LIFT), HOLD, "slide it toward the robot to lock"),
        Waypoint(0.4, _tcp(lock, old0 + SLIDE_LIFT), HOLD, "hold"),
        Waypoint(0.6, _tcp(lock, old0 + SLIDE_LIFT), OPEN, "let go"),
        Waypoint(1.0, _tcp(lock, old0 + OVER), OPEN, "lift away"),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The right arm swaps the covers; the left arm stays at its start."""
    return {"left": [Waypoint(4.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"])],
            "right": right_plan()}
