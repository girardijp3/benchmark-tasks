"""On the station table: a robot arm link in a repair jig and the nylon cable clip that has fallen off it. The link is a
25 mm carbon-fibre tube, 300 mm long, held by its two aluminium joint housings in two saddles, with a 30 mm band painted
where the clip belongs; the jig is fixed to the table. The clip (24 mm wide, 38 mm across, 40 g with its two spring
jaws) stands in front of the jig on the lips of its jaws, opening down, its cable-tie loop on top.

The right arm picks the clip up by its loop, carries it over the link's painted band, and presses it straight down
over the link: the link spreads the two jaws, which are 0.1 N m/rad springs, and they snap shut behind it, 0.8 mm
inside the tube's surface. The right arm lets go and goes back. Only the final state is scored.

The clip is one articulation from ``assets/cable_clip.usda``, not fixed to the world: Clip (root, the ring with the loop),
HingeR - JawR, HingeL - JawL. The jig is one fixed rigid body from ``assets/link_jig.usda``.

Run it:  inspect-robots run --embodiment yam_arena -E task=arm_link_cable_clip.py --scorer task_score ...

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
TAG = "cable_clip"

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
JIG_XY = (0.30, -0.08)  # the tube's axis runs along y through this point
ZC = 0.060  # the tube's axis above the table
TUBE_R = 0.0125
CLIP_XY = (0.20, -0.20)
STAND = 0.01155  # the clip's bore centre above the table while it stands on its jaw lips (the assets' build_assets.py)
CLIP_Z = TOP + STAND + 0.0004
TUBE_Z = TOP + ZC
LOOP_LO, LOOP_HI = 0.018, 0.035  # the cable-tie loop's faces above the bore's centre (clip frame)
CLIP_W = 0.024  # the clip's width along the tube

JIG = UsdPart("jig", asset="jig", xy=JIG_XY, extent=0.38, mass=2.0, usd_path=str(ASSETS / "link_jig.usda"),
              pos=(*JIG_XY, TOP), kinematic=True)
CLIP = UsdPart("clip", asset="clip", xy=CLIP_XY, extent=0.047, mass=0.040, usd_path=str(ASSETS / "cable_clip.usda"),
               pos=(*CLIP_XY, CLIP_Z), articulation=True)
OBJECTS = [JIG, CLIP]

JOINTS = ("HingeR", "HingeL")  # the clip's jaws: right opens negative, left positive (rad)


def extra_states(base, out, origin) -> None:
    """The clip's two jaw angles (rad)."""
    clip = base.scene["clip"]
    ids = clip.find_joints(list(JOINTS), preserve_order=True)[0]
    q = clip.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
    out["jaw_r"], out["jaw_l"] = float(q[0]), float(q[1])


def reset_events() -> list:
    """No reset events: the clip's jaws start at rest (0), as Isaac Lab starts every joint."""
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
LIFT = 0.01  # m: picked up = the clip's lowest point is this far above the table (bore centre above its standing height)
SEAT_AXIS = 0.002  # m: the link's axis within this of the bore's centre (the bore leaves 1 mm all round the tube)
SEAT_TILT = math.radians(8.0)  # the clip's axis within this of the link's
BAND = 0.006  # m: the clip's centre within this of the painted band's middle, along the link
SNAPPED = math.radians(6.0)  # each jaw within this of rest (a jaw held open by the tube sits at about 2 degrees)
CLEAR = 0.03  # m: let go = no pad within this of the clip
CLIP_LO = np.array([-0.023, -0.016, -0.014])  # the clip's extent (clip frame, m), for the clear and held tests
CLIP_HI = np.array([0.023, 0.016, 0.040])
GRIP_FORCE, GRIP_OPENING, GRIP_SLACK = 0.5, 0.012, 0.004
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
STEPS = ("clip picked up", "clip over the link at the band", "jaws snapped shut behind the link",
         "let go, clip still on the link")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, float))


def _clip_frame(objects) -> tuple[np.ndarray, np.ndarray]:
    c = objects["clip"]
    return np.asarray(c.pos, float), _rot(c.quat_xyzw)


def seat_offset(objects) -> np.ndarray:
    """(across, along, up) of the clip's bore centre from the link's axis (world, m)."""
    pos, _ = _clip_frame(objects)
    return np.array([pos[0] - JIG_XY[0], pos[1] - JIG_XY[1], pos[2] - TUBE_Z])


def clip_tilt(objects) -> float:
    """The clip's axis (its y) off the link's axis (world y), rad."""
    _, r = _clip_frame(objects)
    return math.acos(max(-1.0, min(1.0, abs(float(r[1, 1])))))


def on_link(objects) -> bool:
    """The link's axis passes within SEAT_AXIS of the bore's centre, the clip's axis along the link's, and the clip's
    middle at the painted band."""
    off = seat_offset(objects)
    return bool(math.hypot(off[0], off[2]) <= SEAT_AXIS and abs(off[1]) <= BAND and clip_tilt(objects) <= SEAT_TILT)


def snapped(objects) -> bool:
    """Both jaws have swung back to within SNAPPED of rest (the lips are behind the link)."""
    return bool(-SNAPPED <= objects["jaw_r"] <= 0.001 and -0.001 <= objects["jaw_l"] <= SNAPPED)


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def clip_held(objects) -> bool:
    """A gripper holds the clip: its pads pass through the clip's box, both fingers press and the pads are apart."""
    pos, r = _clip_frame(objects)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - pos)
            if np.all(q >= CLIP_LO - GRIP_SLACK) and np.all(q <= CLIP_HI + GRIP_SLACK):
                return True
    return False


def lifted(objects) -> bool:
    pos, _ = _clip_frame(objects)
    return bool(pos[2] >= CLIP_Z + LIFT)


def let_go(objects) -> bool:
    """No gripper pad is within CLEAR of the clip."""
    pos, r = _clip_frame(objects)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - pos)
            if np.all(q >= CLIP_LO - CLEAR) and np.all(q <= CLIP_HI + CLEAR):
                return False
    return True


def _poses(objects) -> tuple:
    c = objects["clip"]
    return (np.array(c.pos, float), np.array(c.quat_xyzw, float), float(objects["jaw_r"]), float(objects["jaw_l"]))


def still(objects, anchor) -> bool:
    """The clip within STILL_MOVE and STILL_TURN and each jaw within STILL_TURN of their poses in ``anchor``."""
    pos, quat, jr, jl = _poses(objects)
    pos0, quat0, jr0, jl0 = anchor
    turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
    return bool(np.linalg.norm(pos - pos0) <= STILL_MOVE and turn <= STILL_TURN and abs(jr - jr0) <= STILL_TURN
                and abs(jl - jl0) <= STILL_TURN)


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper has held the clip
    lifted off the table, and since when every full-success condition has held without a break (``met_since``,
    ``None`` while one fails) with the poses when they began to hold (``anchor``). Any lapse restarts the count.
    Starts over when the episode time goes back."""

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
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.held |= bool(clip_held(objects) and lifted(objects))
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
    on = bool(p.held and on_link(objects))
    sn = bool(on and snapped(objects))
    return [bool(p.held), on, sn, bool(sn and let_go(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand. Step 1 counts once a gripper has held the clip off the table; steps 2 to 4
    count only after that, and each only with the one before."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        off = seat_offset(objects)
        line = (f"(clip {'held' if clip_held(objects) else 'free'}, bore from link axis across {off[0] * 1000:+.1f} "
                f"along {off[1] * 1000:+.1f} up {off[2] * 1000:+.1f} mm, tilt {math.degrees(clip_tilt(objects)):.1f} deg, "
                f"jaws {math.degrees(objects['jaw_r']):+.1f} {math.degrees(objects['jaw_l']):+.1f} deg)")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: the clip off the table, or a state that is not finite."""
    pos, _ = _clip_frame(objects)
    values = [objects["jaw_r"], objects["jaw_l"], *pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(pos[2] < TOP - 0.03)


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S: all four steps done, no violation, and the
    clip and its jaws within STILL_MOVE and STILL_TURN of where they were when that second began."""
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
# Right arm: straight down onto the clip's loop, its fingers closing across the clip's width (along the link); lifts
# the clip, carries it over the painted band, lowers it onto the link until the bore sits on it, lets go and lifts away.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (about 20 N a finger)
CLIP_HOLD = -(CLIP_W - SQUEEZE) / 2
CLIP_OPEN = -(CLIP_W + 0.022) / 2
GRIP_UP = 0.033  # the TCP this far above the clip's bore centre: the pad tips 20 mm above it, on the loop's faces
OVER = 0.07  # the clip is brought over / lifted this far above its place
SEAT_PRESS = 0.001  # the clip is pressed this far below where its bore sits on the link
CLIP_SHIFT = (0.0, 0.0)  # the clip's centre off the TCP in the grip (x, y), measured in the demo


def _tcp_for(clip_xy, clip_z) -> tuple:
    return (clip_xy[0] + CLIP_SHIFT[0], clip_xy[1] + CLIP_SHIFT[1], clip_z + GRIP_UP)


def right_plan() -> list[Waypoint]:
    seat_z = TUBE_Z - 0.001 - SEAT_PRESS  # the bore centre when pressed home (the ring top rests on the link)
    above = TUBE_Z + 0.05
    return [
        Waypoint(2.5, _tcp_for(CLIP_XY, CLIP_Z + OVER), CLIP_OPEN, "over the clip"),
        Waypoint(1.5, _tcp_for(CLIP_XY, CLIP_Z), CLIP_OPEN, "around the loop"),
        Waypoint(0.8, _tcp_for(CLIP_XY, CLIP_Z), CLIP_HOLD, "grip the clip"),
        Waypoint(1.2, _tcp_for(CLIP_XY, CLIP_Z + 0.05), CLIP_HOLD, "lift the clip"),
        Waypoint(2.0, _tcp_for(JIG_XY, above), CLIP_HOLD, "over the painted band"),
        Waypoint(3.0, _tcp_for(JIG_XY, seat_z), CLIP_HOLD, "press it onto the link"),
        Waypoint(0.5, _tcp_for(JIG_XY, seat_z), CLIP_HOLD, "hold"),
        Waypoint(0.6, _tcp_for(JIG_XY, seat_z), CLIP_OPEN, "let go"),
        Waypoint(1.0, _tcp_for(JIG_XY, seat_z + 0.08), CLIP_OPEN, "lift away"),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The right arm puts the clip on the link; the left arm stays at its start."""
    return {"left": [Waypoint(4.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"])],
            "right": right_plan()}
