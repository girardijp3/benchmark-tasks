"""On the station table: a robot head's camera mount, a plate 50 mm above the table (fixed) with a male dovetail rail
along y (14 mm wide at the plate, 23 mm at its top, 6 mm tall, 66 mm long), open at its front end and closed by a stop
block at the far end, and the camera module that has come off it: 36 x 40 x 22 mm, 80 g, a dovetail slot in its
underside with 2.5 mm of play round the rail's base, lying on a parts rack (70 x 70 mm, 30 mm tall, fixed) beside the robot.

The right arm picks the module up, sets it on the plate in front of the rail with its slot lined up on the rail's
axis, and pushes it along the rail until it meets the stop: the undercut walls of the slot then hold it on the rail.
It lets go and goes back.

The module is a free rigid body from ``assets/camera_module.usda`` (the slot is two convex cheek prisms and a roof); the
mount (plate, rail with its flanks, stop) and the rack are fixed rigid bodies. Nothing is spring-held: the module is held
on only by the rail's undercut.

Run it:  inspect-robots run --embodiment yam_arena -E task=camera_dovetail_seat.py --scorer task_score ...

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
TAG = "cam_seat"

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
MOUNT_XY = (0.34, -0.05)  # the rail's axis runs along y through x 0.34; y 0 is the plate's middle (the rail spans -30..+36 mm)
PLATE_TOP = TOP + 0.050
RAIL_Y0, RAIL_Y1 = -0.030, 0.036
MOD_W, MOD_L, MOD_H = 0.036, 0.040, 0.022
HOME_Y = RAIL_Y1 - MOD_L / 2  # the module's middle along y when it meets the stop (rel. to MOUNT_XY y)
ENTRY_Y = RAIL_Y0 - MOD_L / 2 - 0.006  # where the module is set down in front of the rail
RACK_XY = (0.20, -0.28)
RACK_TOP = TOP + 0.030
REST = 0.0003

MOUNT = UsdPart("mount", asset="mount", xy=MOUNT_XY, extent=0.15, mass=3.0, usd_path=str(ASSETS / "head_mount.usda"),
                pos=(*MOUNT_XY, PLATE_TOP), kinematic=True)
RACK = UsdPart("rack", asset="rack", xy=RACK_XY, extent=0.07, mass=2.0, usd_path=str(ASSETS / "parts_rack.usda"),
               pos=(*RACK_XY, RACK_TOP), kinematic=True)
MODULE = UsdPart("module", asset="module", xy=RACK_XY, extent=0.04, mass=0.08, usd_path=str(ASSETS / "camera_module.usda"),
                 pos=(*RACK_XY, RACK_TOP + REST))
OBJECTS = [MOUNT, RACK, MODULE]


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
LIFT = 0.01  # m: picked up = the module's underside is this far above the rack
ON_X = 0.0025  # m: the module's middle within this of the rail's axis (the slot leaves 2.5 mm)
ON_Z = (-0.0005, 0.0015)  # its underside on the plate
ON_YAW, ON_TILT = math.radians(4.0), math.radians(2.5)
STOP_TOL = 0.002  # m: seated = the module's middle within this of where it meets the stop
ENGAGED_Y = RAIL_Y0 + 0.012  # the module's middle is past this along the rail: the rail is inside the slot
CLEAR = 0.03  # m: let go = no pad within this of the module
MOD_LO = np.array([-MOD_W / 2, -MOD_L / 2, -0.002])  # the module's extent (its frame, m)
MOD_HI = np.array([MOD_W / 2, MOD_L / 2, MOD_H])
GRIP_FORCE, GRIP_OPENING, GRIP_SLACK = 0.5, 0.015, 0.004
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
STEPS = ("camera module picked up", "module on the dovetail rail", "module pushed home against the stop",
         "let go, module still on the rail")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, float))


def _frame(objects) -> tuple[np.ndarray, np.ndarray]:
    c = objects["module"]
    return np.asarray(c.pos, float), _rot(c.quat_xyzw)


def tilt(objects) -> float:
    _, r = _frame(objects)
    return math.acos(max(-1.0, min(1.0, float(r[2, 2]))))


def yaw(objects) -> float:
    _, r = _frame(objects)
    return math.atan2(float(r[1, 0]), float(r[0, 0]))  # 0: the module's slot along the rail (its y along the world y)


def rel(objects) -> np.ndarray:
    """The module's underside middle from the rail's axis at the plate's middle (x across, y along, z up), m."""
    pos, _ = _frame(objects)
    return np.array([pos[0] - MOUNT_XY[0], pos[1] - MOUNT_XY[1], pos[2] - PLATE_TOP])


def _aligned(objects) -> bool:
    """Flat on the plate, square to the rail, and on its axis: the slot's walls cannot be on the rail's flanks otherwise."""
    d = rel(objects)
    yw = yaw(objects)  # the module's y (its slot) along the world y
    return bool(abs(d[0]) <= ON_X and ON_Z[0] <= d[2] <= ON_Z[1] and abs(yw) <= ON_YAW and tilt(objects) <= ON_TILT)


def on_rail(objects) -> bool:
    """The rail is inside the slot: aligned, and the module's middle far enough along the rail that the slot's walls
    (module 40 mm long) are over its undercut."""
    d = rel(objects)
    return bool(_aligned(objects) and ENGAGED_Y <= d[1] <= HOME_Y + STOP_TOL)


def home(objects) -> bool:
    d = rel(objects)
    return bool(_aligned(objects) and abs(d[1] - HOME_Y) <= STOP_TOL)


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def held(objects) -> bool:
    pos, r = _frame(objects)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - pos)
            if np.all(q >= MOD_LO - GRIP_SLACK) and np.all(q <= MOD_HI + GRIP_SLACK):
                return True
    return False


def lifted(objects) -> bool:
    pos, _ = _frame(objects)
    return bool(pos[2] >= RACK_TOP + REST + LIFT)


def let_go(objects) -> bool:
    pos, r = _frame(objects)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - pos)
            if np.all(q >= MOD_LO - CLEAR) and np.all(q <= MOD_HI + CLEAR):
                return False
    return True


def _poses(objects) -> tuple:
    c = objects["module"]
    return (np.array(c.pos, float), np.array(c.quat_xyzw, float))


def still(objects, anchor) -> bool:
    pos, quat = _poses(objects)
    pos0, quat0 = anchor
    turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
    return bool(np.linalg.norm(pos - pos0) <= STILL_MOVE and turn <= STILL_TURN)


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper has held the module
    lifted off the rack, and since when every full-success condition has held without a break (``met_since``) with
    the pose when they began to hold (``anchor``). Any lapse restarts the count. Starts over when the time goes back."""

    def __init__(self):
        self.time = -1.0
        self.held = False
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
        self.held |= bool(held(objects) and lifted(objects))
        if t is None:
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'module moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    p = _PROGRESS
    on = bool(p.held and on_rail(objects))
    h = bool(on and home(objects))
    return [bool(p.held), on, h, bool(h and let_go(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand. Step 1 counts once a gripper has held the module off the rack; 2 to 4 only after
    that, each with the one before."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        d = rel(objects)
        line = (f"(module {'held' if held(objects) else 'free'}, from rail axis x {d[0] * 1000:+.1f} y {d[1] * 1000:+.1f} "
                f"z {d[2] * 1000:+.1f} mm (home y {HOME_Y * 1000:+.0f}), yaw {math.degrees(yaw(objects)):+.1f}, "
                f"tilt {math.degrees(tilt(objects)):.1f} deg)")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    pos, _ = _frame(objects)
    if not np.all(np.isfinite(pos)):
        return True
    return bool(pos[2] < TOP - 0.03)


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
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
# Right arm, straight down beside the module, the fingers closing across its 36 mm width (along x, so the gripper is
# turned a quarter turn about the vertical: the module's slot runs along y). It lifts the module off the rack, carries it
# over the plate in front of the rail, sets it down square on the rail's axis and pushes it along y until it meets the stop.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (about 20 N a finger)
HOLD = -(MOD_W - SQUEEZE) / 2
OPEN = -(MOD_W + 0.022) / 2
GRIP_UP = 0.017  # the TCP this far above the module's underside: the pad tips 4 mm above it
OVER = 0.05
OVERPUSH = 0.008  # the push is commanded this far past the stop, which meets the module first
LIFT_SKIM = 0.0006  # the module slides this far over the plate


def _tcp(x, y, z0) -> tuple:
    return (x, y, z0 + GRIP_UP)


def right_plan() -> list[Waypoint]:
    r = math.pi / 2
    mod0 = RACK_TOP + REST
    plate0 = PLATE_TOP + REST
    mx, my = MOUNT_XY
    W = Waypoint
    return [
        W(2.5, _tcp(*RACK_XY, mod0 + OVER), OPEN, "over the camera module", r),
        W(1.5, _tcp(*RACK_XY, mod0), OPEN, "beside its sides", r),
        W(0.8, _tcp(*RACK_XY, mod0), HOLD, "grip it", r),
        W(1.2, _tcp(*RACK_XY, mod0 + OVER), HOLD, "lift it", r),
        W(2.0, _tcp(mx, my + ENTRY_Y, plate0 + OVER), HOLD, "over the dovetail's entry", r),
        W(1.5, _tcp(mx, my + ENTRY_Y, plate0 + LIFT_SKIM), HOLD, "set it on the plate, slot over the rail's axis", r),
        W(0.4, _tcp(mx, my + ENTRY_Y, plate0 + LIFT_SKIM), HOLD, "settle", r),
        W(3.0, _tcp(mx, my + HOME_Y + OVERPUSH, plate0 + LIFT_SKIM), HOLD, "push it along the rail to the stop", r),
        W(0.4, _tcp(mx, my + HOME_Y + OVERPUSH, plate0 + LIFT_SKIM), HOLD, "hold", r),
        W(0.6, _tcp(mx, my + HOME_Y + OVERPUSH, plate0 + LIFT_SKIM), OPEN, "let go", r),
        W(1.0, _tcp(mx, my + HOME_Y + OVERPUSH, plate0 + OVER), OPEN, "lift away", r),
        W(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The right arm seats the module; the left arm stays at its start."""
    return {"left": [Waypoint(4.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"])],
            "right": right_plan()}
