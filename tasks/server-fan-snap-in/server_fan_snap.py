"""On the station table: a tool-less bracket for a 120 mm case fan (a 147 mm square base ring 6 mm high with four L-shaped corner
posts 46 mm tall, a 127 mm opening, and two spring latches on the middle of two opposite sides; one fixed articulation) and a
120 x 120 x 25 mm case fan lying on the table (130 g, a square frame with a hub).

The fan has to be lowered square into the four corner posts (3.5 mm a side, with 6 mm lead-in bevels) and pressed down until its
lower face rests on the ring: only then does its upper face clear the latch hooks (0.6 mm), which spring back over it. Each
latch is a 34 mm arm on a hinge with a 7.5 mm hook whose top is a 54 degree ramp: the fan's lower edge pushes the latches open
as it comes down and rubs along them, and they snap shut over its top face in the last 0.6 mm. A fan not pressed all the way down
is not seated (its lower face must be within 0.8 mm of the ring and both latches back over it).

The left arm lifts the fan by one side bar from above, carries it over the bracket, lowers it into the posts and presses
it home, lets go and backs off.

The bracket is one articulation (fixed base, two latch links on revolute hinges with spring drives, 0.09 N m/rad preloaded
2 degrees against their inner stops); the fan is a rigid body. Both come from USD files in ``assets/``.

Run it:  inspect-robots run --embodiment yam_arena -E task=server_fan_snap.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
Arena gives objects no actuators, so a reset event writes the latch springs' targets and joint angles (as in
extrusion-guard-door) and ``object_states`` reads the latch angles.
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
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body or a fixture
    (``kinematic``). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False
    articulation: bool = False  # an articulation (joints), its root link fixed to the world when ``fixed``
    fixed: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)




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


def _grip(yaw: float) -> np.ndarray:
    return yam_demo.grip_rotation(yaw)


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


_X, _Y, _Z = np.eye(3)
GRIP_SLACK = 0.005  # a gripper's pads this close to a part's box count as reaching it
GRIP_FORCE = 0.5  # N each finger: both fingers press on something
GRIP_OPENING = 0.002  # the fingers are not closed on nothing


def _rz(a: float) -> np.ndarray:
    return axis_angle_matrix(_Z, a)


def _xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(r))


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _in_box(point, origin, rot, lo, hi, slack: float) -> bool:
    q = rot.T @ (np.asarray(point, float) - origin)
    return bool(np.all(q >= lo - slack) and np.all(q <= hi + slack))


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def _grips(gripper) -> bool:
    return bool(gripper is not None and gripper.contact >= GRIP_FORCE and gripper.opening >= GRIP_OPENING)


def _grippers(objects):
    return [g for g in (objects.get("left_gripper"), objects.get("right_gripper")) if g is not None]


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
            solver_position_iteration_count=64, solver_velocity_iteration_count=1,
            enabled_self_collisions=True, fix_root_link=True if spec.fixed else None)
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


def _joint_ids(art):
    return art.find_joints(list(ART_JOINTS), preserve_order=True)[0]


def _set_targets(art, targets) -> None:
    """Writes the drive targets (rad or m, one per joint of ``ART_JOINTS``) to PhysX directly: Arena gives objects no
    actuators, so Isaac Lab's joint targets never reach the simulator."""
    import torch

    view = art.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    names = list(view.shared_metatype.dof_names)
    for name, v in zip(ART_JOINTS, targets, strict=True):
        t[:, names.index(name)] = float(v)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, art._ALL_INDICES)


def _reset_art(env, env_ids, asset_cfg) -> None:
    """Reset event: the joints at their start values, the mechanism's state back to the start."""
    import torch

    if env_ids is None:
        return
    art = env.scene[asset_cfg.name]
    ids = _joint_ids(art)
    start = torch.tensor([list(ART_START)], device=art.device, dtype=torch.float32).repeat(len(env_ids), 1)
    art.write_joint_position_to_sim_index(position=start, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(start), joint_ids=ids, env_ids=env_ids)
    art_reset()
    _set_targets(art, art_targets(list(ART_START)))


def _step_art(env, env_ids, asset_cfg) -> None:
    """Every control step: the task's catch or detent may move the drive targets."""
    art = env.scene[asset_cfg.name]
    q = art.data.joint_pos.torch[0, _joint_ids(art)].detach().cpu().numpy().astype(np.float64)
    t = float(env.episode_length_buf[0]) * float(env.step_dt)
    new = art_update(q, t)
    if new is not None:
        _set_targets(art, new)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root pose and speeds (Arena's geometry queries take stock objects only, so bounds are left
        at the root), the grippers and the episode time."""
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
        art = base.scene[ART_NAME]
        ids = art.find_joints(list(ART_JOINTS), preserve_order=True)[0]
        out["joints"] = tuple(float(v) for v in art.data.joint_pos.torch[0, ids].detach().cpu().numpy())
        out.update(art_extra())
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event (joints at their start values, targets, mechanism) and
        a per-step event (the task's catch or detent moves the drive targets)."""
        configure = stock_cfg_callback(control_hz)

        def with_start(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.art_reset = EventTermCfg(func=_reset_art, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg(ART_NAME)})
            env_cfg.events.art_step = EventTermCfg(func=_step_art, mode="interval", interval_range_s=(0.0, 0.0),
                                                   is_global_time=False, params={"asset_cfg": SceneEntityCfg(ART_NAME)})
            return env_cfg

        return with_start

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



TAG = "server_fan_snap"
PART = "fan"
FIXTURES = ("bracket",)
ART_NAME = "bracket"
ART_JOINTS = ("HingeP", "HingeN")
ART_START = (0.0, 0.0)  # both latches at their inner stops (hooks over where the fan's top will be)
PRELOAD = math.radians(2.0)  # the springs' targets are this far past the stops


def art_extra() -> dict:
    return {}


def art_reset() -> None:
    pass


def art_targets(q) -> list:
    """The latch drive targets: 2 degrees past each inner stop (the +y latch's inward turn is positive, the -y latch's
    negative)."""
    return [PRELOAD, -PRELOAD]


def art_update(q, t):
    """Nothing changes during the episode: the latches are plain springs."""
    return None


# -- the fan (its frame: origin at the middle of its lower face, axis along z) -----------------------------------------------
FAN_W, FAN_H, BAR = 120.0 * MM, 25.0 * MM, 8.0 * MM
FAN_MASS = 0.130
PART_LO = np.array([-FAN_W / 2, -FAN_W / 2, 0.0])
PART_HI = np.array([FAN_W / 2, FAN_W / 2, FAN_H])
LEDGE = 6.0 * MM  # the ring's top: the fan's lower face rests on it
WALL_TOP = 46.0 * MM  # the corner posts' tops above the table

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ------------------------------
BRACKET_XY = (0.30, 0.0)
BRACKET_POS = np.array([*BRACKET_XY, TOP])
FAN_XY = (0.30, 0.22)
FAN_START = np.array([FAN_XY[0], FAN_XY[1], TOP + 0.0003])
PART_START = FAN_START
NUDGE = np.zeros(3)
NUDGE_YAW = 0.0

BRACKET = UsdPart("bracket", asset="bracket", xy=BRACKET_XY, extent=0.2, mass=1.0,
                  usd_path=str(ASSETS / "fan_bracket.usda"), pos=tuple(float(v) for v in BRACKET_POS), articulation=True,
                  fixed=True)
FAN = UsdPart("fan", asset="fan", xy=FAN_XY, extent=0.2, mass=FAN_MASS, usd_path=str(ASSETS / "case_fan.usda"),
              pos=tuple(float(v) for v in (FAN_START + NUDGE)), rot_xyzw=_xyzw(_rz(NUDGE_YAW)))
OBJECTS = [BRACKET, FAN]


# -- scoring ----------------------------------------------------------------------------------------------------------
LIFTED = 0.02  # m: the fan at least this far above where it lay, held, counts as picked up
IN_Z = LEDGE + 0.014  # step 2: its lower face at most this low (8 mm inside the posts' walls)
IN_OFF, IN_YAW, IN_TILT = 0.0035, math.radians(3.0), math.radians(4.0)
SEAT_Z = 0.0008  # step 3: its lower face within this of the ring
SEAT_OFF, SEAT_YAW, SEAT_TILT = 0.003, math.radians(2.0), math.radians(1.5)
LATCH_SHUT = math.radians(2.5)  # both latches back within this of their stops (hooks over the fan's top face)
CLEAR = 0.03  # m: both grippers' pads at least this far from the fan
FALLEN = math.radians(45.0)
FALLEN_Z = -0.02
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)
TRACE_S = 1.0
STEPS = ("fan lifted off the table", "fan lowered into the corner posts", "fan pressed down onto the ring, both latches over it",
         "fan let go")
POINTS = 0.2  # per step; the four steps give 0.8: only success scores 1.0


def fan_pose(objects) -> dict:
    """The fan against the bracket: ``off`` its axis' distance from the bracket's (m), ``low`` its lower face above the
    table (m), ``yaw`` its turn about the vertical, ``tilt`` its axis from vertical (rad); ``open_p`` and ``open_n`` how far
    each latch is swung open (rad)."""
    pos, r = _part(objects)
    q = objects["joints"]
    out = {"open_p": float(-q[0]), "open_n": float(q[1])}
    if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(r)) and np.all(np.isfinite(q))):
        out.update({k: math.nan for k in ("off", "low", "yaw", "tilt")})
        return out
    out.update({"off": float(np.linalg.norm((pos - BRACKET_POS)[:2])), "low": float(pos[2] - TOP),
                "yaw": float(math.atan2(-r[0, 1], r[1, 1])), "tilt": float(math.acos(min(1.0, max(-1.0, r[2, 2]))))})
    return out


def _square(yaw: float) -> float:
    """The fan's yaw folded into -45..45 degrees (the frame is square)."""
    return (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4


def engaged(objects) -> bool:
    """The fan's lower face is inside the corner posts (at most IN_Z above the table), centred within IN_OFF, within IN_YAW
    of square and IN_TILT of level."""
    s = fan_pose(objects)
    return bool(np.isfinite(s["low"]) and s["low"] <= IN_Z and s["off"] <= IN_OFF and abs(_square(s["yaw"])) <= IN_YAW
                and s["tilt"] <= IN_TILT)


def seated(objects) -> bool:
    """Engaged and pressed home: the lower face within SEAT_Z of the ring, centred, square and level, and both latches back
    over its upper face (open at most LATCH_SHUT)."""
    s = fan_pose(objects)
    return bool(engaged(objects) and s["low"] <= LEDGE + SEAT_Z and s["off"] <= SEAT_OFF and abs(_square(s["yaw"])) <= SEAT_YAW
                and s["tilt"] <= SEAT_TILT and s["open_p"] <= LATCH_SHUT and s["open_n"] <= LATCH_SHUT)


def extra_violation(objects) -> bool:
    """The fan fallen over (more than 45 degrees), or the bracket moved from where it is fixed."""
    s = fan_pose(objects)
    return bool(s["tilt"] > FALLEN or np.linalg.norm(objects["bracket"].pos - BRACKET_POS) > 0.005)


def trace_line(objects) -> str:
    s = fan_pose(objects)
    return (f"fan: off {s['off'] * 1000:.1f} mm, lower face {s['low'] * 1000:.1f} mm up (ring {LEDGE * 1000:.0f}), yaw "
            f"{math.degrees(_square(s['yaw'])):.1f} tilt {math.degrees(s['tilt']):.1f} deg; latches open "
            f"{math.degrees(s['open_p']):.1f} / {math.degrees(s['open_n']):.1f} deg;")



# -- shared scoring (a free part lifted from its start, brought to a fixture, seated, let go) ---------------------------
def _part(objects):
    p = objects[PART]
    return np.array(p.pos, float), _rot(p)


def part_held(objects) -> bool:
    """A gripper holds the part: its pads reach the part's box and both fingers press."""
    pos, r = _part(objects)
    return any(_grips(g) and any(_in_box(q, pos, r, PART_LO, PART_HI, GRIP_SLACK) for q in _pads(g))
               for g in _grippers(objects))


def let_go(objects) -> bool:
    """No gripper holds the part or has its pads within CLEAR of it."""
    pos, r = _part(objects)
    return not (part_held(objects) or any(_in_box(q, pos, r, PART_LO, PART_HI, CLEAR)
                                          for g in _grippers(objects) for q in _pads(g)))


def _poses(objects):
    p = objects[PART]
    return np.array(p.pos, float), np.array(p.quat_xyzw, float)


def still(objects, anchor) -> bool:
    pos, quat = _poses(objects)
    pos0, quat0 = anchor
    turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
    return bool(np.linalg.norm(pos - pos0) <= STILL_MOVE and turn <= STILL_TURN)


def _violation(objects) -> bool:
    """Scores 0: a state that is not finite, the part off the table, or a violation of the task (``extra_violation``)."""
    p = objects[PART]
    values = [*p.pos, *p.quat_xyzw] + [v for n in FIXTURES for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    if p.pos[2] < TOP + FALLEN_Z:
        return True
    return bool(extra_violation(objects))


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper has lifted the part
    from its start, and since when every full-success condition has held without a break (``met_since``) with the
    part's pose then (``anchor``). Any lapse (a step undone, a violation, or the part moving more than STILL_MOVE or
    turning more than STILL_TURN from the anchor) restarts the count. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.picked = False
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
        self.picked |= bool(objects[PART].pos[2] > PART_START[2] + LIFTED and part_held(objects))
        if t is None:
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'part moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand, from the episode's progress so far (see ``steps_done``)."""
    p = _PROGRESS
    inn = bool(p.picked and engaged(objects))
    on = bool(inn and seated(objects))
    free = bool(on and let_go(objects))
    return [bool(p.picked), inn, on, free]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached (a gripper has lifted the part), steps 2 to 4 as things stand, each
    only with the one before."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = trace_line(objects) + f" held {part_held(objects)}"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  ({line})", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  ({line})", flush=True)
        p.traced = t
    return done


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all four steps done (the last one: both grippers clear), no violation, and the part within STILL_MOVE and
    STILL_TURN of where it was when that second began. The arms' return is not scored."""
    p = _PROGRESS
    steps_done(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)



# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from above, fingers closing along x across the fan's +x side bar (8 mm), pad tips 10 mm down from its top; lifts the
# fan, carries it over the bracket with its lower face 10 mm over the posts' tops, lowers it into the posts (the latches
# ride along its sides), presses it onto the ring until the hooks snap over, lets go, lifts off and goes home.
GRIP_YAW = -math.pi / 2  # fingers close along x
HOLD = 0.0  # the fingers closed on the 8 mm bar: about 20 N a finger
OPEN = -(BAR + 0.030) / 2
PAD_DEPTH = 14.0 * MM  # pad tips this far down the bar
LIFT = 0.056  # the fan's lower face this far off the table while it is carried (the corner posts top out at 46 mm)
PRESS = 0.0002  # m: the fan's lower face is driven to the ring's top less this
AIM = np.array([0.0, -1.0, 0.0]) * MM  # the TCP's aim is moved this much at the bracket (measured sag)
BACK_OFF = 0.05


def _tcp_for(origin) -> np.ndarray:
    """The left TCP gripping the +x bar of a fan whose lower-face middle is at ``origin`` (the fan flat)."""
    return np.asarray(origin, float) + np.array([FAN_W / 2 - BAR / 2, 0.0, FAN_H - PAD_DEPTH + PAD_TIP_PAST_TCP])


def left_plan() -> list[Waypoint]:
    rot = _grip(GRIP_YAW)
    start = FAN_START
    lifted = start + (0.0, 0.0, LIFT)
    over = np.array([BRACKET_POS[0], BRACKET_POS[1], TOP + LIFT])
    wide = np.array([BRACKET_POS[0], BRACKET_POS[1], TOP + 0.050])
    seat = np.array([BRACKET_POS[0], BRACKET_POS[1], TOP + LEDGE - PRESS])
    return [
        Waypoint(2.5, _tcp_for(start) + (0, 0, 0.04), FINGER_OPEN, "over the fan's side bar", rot=rot),
        Waypoint(1.2, _tcp_for(start), OPEN, "pads astride the bar", rot=rot),
        Waypoint(0.8, _tcp_for(start), HOLD, "grip the bar", rot=rot),
        Waypoint(1.5, _tcp_for(lifted), HOLD, "lift the fan", rot=rot),
        Waypoint(2.5, _tcp_for(over) + AIM, HOLD, "over the bracket", rot=rot),
        Waypoint(1.5, _tcp_for(wide) + AIM, HOLD, "lower it to the posts' mouths", rot=rot),
        Waypoint(3.5, _tcp_for(seat) + AIM, HOLD, "press it down into the posts and latches", rot=rot),
        Waypoint(0.8, _tcp_for(seat) + AIM, HOLD, "hold", rot=rot),
        Waypoint(0.6, _tcp_for(seat) + AIM, OPEN, "let go", rot=rot),
        Waypoint(1.0, _tcp_for(seat) + AIM + (0, 0, BACK_OFF), OPEN, "lift off", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


def demo():
    """The left arm lifts the fan, carries it over the bracket, presses it down until the latches snap over it, lets go and
    goes home."""
    return {"left": left_plan()}
