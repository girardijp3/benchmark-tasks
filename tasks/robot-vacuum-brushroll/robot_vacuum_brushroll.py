"""On the station table: the brush bay of a robot vacuum (a fixed base with two end walls 216 mm apart and the -y wall has a 12 mm
bearing slot, the +y wall a 9.5 mm square drive socket, both open at the top with lead-ins; a red drive-end retainer flap hinged outside the +y
wall, sprung open at 75 degrees) and, 115 mm toward the robot, the brush roll (28 x 200 mm, 0.10 kg) lying on the table with its square drive stub toward +y,
the same way round as the bay's drive socket.

The task is to refit the roll and close the drive-end retainer: pick the roll up by its body, carry it over the bay, lower its round stub into the bearing slot
and its square stub into the drive socket (the 11 mm round stub does not fit the 9.5 mm socket, and the square stub must be turned within
about 12 degrees of the socket's flats), let go, and push the retainer flap down over the drive socket until its latch catches. The retainer is an articulation (revolute,
0 closed .. 75 open); the latch (angle within 3 degrees) is declared task code that moves the drive target below closed. Everything else is
physical.

Run it:  inspect-robots run --embodiment yam_arena -E task=robot_vacuum_brushroll.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "brushroll"
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
BAY_XY = (0.34, -0.25)  # the bay's middle
BODY_R, BODY_L = 0.014, 0.200
STUB_L = 0.018
AXIS_Z = 0.026  # the roll's axis above the table when seated
WALL_TOP = 0.050
HINGE = (BAY_XY[0], BAY_XY[1] + 0.126, TOP + 0.0515)  # the retainer's hinge, on an ear outside the +y wall
GUARD_L = 0.030
OPEN_DEG = 75.0
LATCH_DEG = 3.0
GUARD_J = "Retainer"
ART_NAME = "chassis"
ROLL_START = (0.225, BAY_XY[1], TOP + BODY_R + 0.0003)  # the body's middle, lying on the table
_LATCH = {"on": False}

CHASSIS = UsdPart("chassis", asset="chassis", xy=BAY_XY, extent=0.26, mass=3.0, usd_path=str(ASSETS / "chassis.usda"),
                  pos=(BAY_XY[0], BAY_XY[1], TOP), articulation=True, fixed=True)
ROLL = UsdPart("roll", asset="roll", xy=ROLL_START[:2], extent=0.23, mass=0.10, usd_path=str(ASSETS / "roll.usda"), pos=ROLL_START,
               rot_xyzw=(0.0, 0.0, 0.0, 1.0))  # the square stub toward +y
OBJECTS = [CHASSIS, ROLL]
MOVERS = ("roll",)


def _set_targets(art, values: dict) -> None:
    """Writes drive targets (joint name -> rad) to PhysX directly: Arena gives objects no actuators, so Isaac Lab's joint targets never reach the simulator."""
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
    for k, v in values.items():
        t[:, names.index(k)] = float(v)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, art._ALL_INDICES)


def extra_states(base, out, origin) -> None:
    """The retainer's angle (rad, 0 closed, positive open) and whether the latch has caught."""
    art = base.scene[ART_NAME]
    ids = art.find_joints([GUARD_J], preserve_order=True)[0]
    out["retainer"] = -float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])  # the joint is negative when open: this is positive open
    out["latched"] = bool(_LATCH["on"])


def _reset_art(env, env_ids, asset_cfg=None) -> None:
    """Reset event: the retainer open, the latch off."""
    import torch

    if env_ids is None:
        return
    _LATCH["on"] = False
    art = env.scene[ART_NAME]
    ids = art.find_joints([GUARD_J], preserve_order=True)[0]
    q = torch.full((1, 1), -math.radians(OPEN_DEG), device=art.device, dtype=torch.float32)
    art.write_joint_position_to_sim_index(position=q, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids, env_ids=env_ids)
    _set_targets(art, {GUARD_J: -math.radians(OPEN_DEG)})


def _latch_step(env, env_ids, asset_cfg=None) -> None:
    """Every control step: a retainer pushed within LATCH_DEG of closed is caught (declared: its drive target moves below closed)."""
    if _LATCH["on"]:
        return
    art = env.scene[ART_NAME]
    ids = art.find_joints([GUARD_J], preserve_order=True)[0]
    q = -float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])  # positive open
    if q <= math.radians(LATCH_DEG):
        _LATCH["on"] = True
        _set_targets(art, {GUARD_J: math.radians(8.0)})
        t = float(env.episode_length_buf[0]) * float(env.step_dt)
        print(f"[{TAG}] t={t:.2f}s  the retainer reached {math.degrees(q):.1f} deg: latched", flush=True)


def reset_events() -> list:
    return [("art_reset", _reset_art, {})]


def step_events() -> list:
    return [("retainer_latch", _latch_step, {})]


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
LIFTED = 0.015  # the roll counts as picked up once its axis has been this far above its start
SEAT_X, SEAT_Y, SEAT_Z = 0.003, 0.004, 0.002  # seated: axis within these of the bay's slots (x across, y along, z up)
SEAT_YAW = math.radians(4.0)  # the axis within this of the bay's y axis, square stub toward +y
SEAT_TWIST = math.radians(12.0)  # the square stub's flats within this of the socket's
CLOSED = math.radians(4.0)
CLEAR = 0.03
ROLL_LO = np.array([-BODY_R, -BODY_L / 2 - STUB_L, -BODY_R])
ROLL_HI = np.array([BODY_R, BODY_L / 2 + STUB_L, BODY_R])
STEPS = ("the roll picked up off the table", "the roll seated, round stub in the bearing slot and square stub in the drive socket",
         "the drive-end retainer pushed down and latched over the seated roll", "grippers clear of the roll")


def seated(objects) -> bool:
    s = objects["roll"]
    p, r = np.asarray(s.pos, float), _rot(s)
    ax = r[:, 1]
    yaw = math.acos(max(-1.0, min(1.0, float(ax[1]))))  # angle from +y (the square stub toward +y)
    twist = abs(math.asin(max(-1.0, min(1.0, float(r[2, 0])))))
    return bool(abs(p[0] - BAY_XY[0]) <= SEAT_X and abs(p[1] - BAY_XY[1]) <= SEAT_Y and abs(p[2] - TOP - AXIS_Z) <= SEAT_Z
                and yaw <= SEAT_YAW and twist <= SEAT_TWIST)


def retainer_latched(objects) -> bool:
    return bool(objects.get("latched") and float(objects.get("retainer", 1.0)) <= CLOSED)


def held(objects) -> bool:
    return any(holds(objects, s, "roll", ROLL_LO, ROLL_HI) for s in ("left", "right"))


def clear_of_roll(objects) -> bool:
    return not pads_near(objects, "roll", ROLL_LO, ROLL_HI, CLEAR)


def _steps(objects) -> list[bool]:
    lifted = bool(_PROGRESS.mem.get("lifted"))
    seat = bool(seated(objects))
    done = bool(seat and retainer_latched(objects))
    return [lifted, seat, done, bool(done and clear_of_roll(objects) and not held(objects))]


def events_now(objects, p) -> list[str]:
    """Rule events as things stand: the roll off the table, or the retainer latched with the roll not seated. Also remembers (``p.mem``) the roll picked up."""
    out = []
    if objects["roll"].pos[2] > ROLL_START[2] + LIFTED:
        p.mem["lifted"] = True
    if objects["roll"].pos[2] < TOP - 0.03:
        out.append("the roll fell off the table")
    if retainer_latched(objects) and not seated(objects):
        out.append("the retainer was latched with the roll not seated")
    return out


def trace_line(objects) -> str:
    s = objects["roll"]
    r = _rot(s)
    return (f"roll at {(s.pos[0] - BAY_XY[0]) * 1000:.0f},{(s.pos[1] - BAY_XY[1]) * 1000:.0f} mm from the bay, {(s.pos[2] - TOP - AXIS_Z) * 1000:.1f} mm above its seat, "
            f"stub toward {'+y' if r[1, 1] > 0 else '-y'}, retainer {math.degrees(float(objects.get('retainer', float('nan')))):.1f} deg, "
            f"latched {bool(objects.get('latched'))}, seated {seated(objects)}{'*' if held(objects) else ''}")


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
# The right arm pinches the roll's body across its 28 mm diameter (closing along x), lifts it, carries it over the bay and lowers it until the stubs are in their slots, lets go; then pushes the bearing retainer down with its closed
# fingers, the pad tips riding on the retainer's top face along its arc, until the latch catches.
_grip = yam_demo.grip_rotation
SQUEEZE = 0.003  # each finger is commanded this far inside the body
HOLD = -(BODY_R - SQUEEZE)
OPEN = -(BODY_R + 0.012)
ROT_PICK = _grip(math.pi / 2)  # pointing down, the fingers closing along x
ROT_PLACE = ROT_PICK
TIP_BELOW = 0.004  # the pad tips go this far below the roll's axis
CARRY_Z = 0.075  # the roll's axis height while carried (the stubs clear the 50 mm walls)
DROP_Z = AXIS_Z + 0.006  # the roll's axis at the release, just above the slots' floors


def _tcp(x, y, z_axis):
    return (x, y, TOP + z_axis - TIP_BELOW + PAD_TIP_PAST_TCP)


def left_plan() -> list[Waypoint]:
    return []


def right_plan() -> list[Waypoint]:
    rx, ry = ROLL_START[0], ROLL_START[1]
    z0 = BODY_R + 0.0003
    bx, by = BAY_XY
    pivot = np.array([HINGE[0], HINGE[1], HINGE[2]])
    wps = [
        Waypoint(2.5, _tcp(rx, ry, z0 + 0.05), OPEN, "over the roll", rot=ROT_PICK),
        Waypoint(1.5, _tcp(rx, ry, z0), OPEN, "pads astride the roll", rot=ROT_PICK),
        Waypoint(0.8, _tcp(rx, ry, z0), HOLD, "grip the roll", rot=ROT_PICK),
        Waypoint(0.4, _tcp(rx, ry, z0), HOLD, "hold", rot=ROT_PICK),
        Waypoint(1.2, _tcp(rx, ry, CARRY_Z), HOLD, "lift the roll", rot=ROT_PICK),
        Waypoint(4.0, _tcp(bx, by, CARRY_Z), HOLD, "carry it over the bay", rot=ROT_PLACE),
        Waypoint(1.5, _tcp(bx, by, DROP_Z), HOLD, "lower the stubs into their slots", rot=ROT_PLACE),
        Waypoint(0.6, _tcp(bx, by, DROP_Z), OPEN, "let go", rot=ROT_PLACE),
        Waypoint(1.2, _tcp(bx, by, CARRY_Z), OPEN, "lift away", rot=ROT_PLACE),
        Waypoint(1.0, _tcp(bx, by, CARRY_Z), 0.0, "close the fingers", rot=ROT_PLACE),
        Waypoint(1.0, _tcp(bx, by, CARRY_Z), 0.0, "turn the hand a quarter turn", rot=_grip(0.0)),
    ]
    R_PUSH = GUARD_L - 0.010
    fingers = 0.0  # closed fingers
    ROT_G = _grip(0.0)  # closing along y: the pads are 27 mm wide along x, 12.8 along y

    def on_flap(deg, off=0.0):
        a = math.radians(deg)
        n = np.array([0.0, math.sin(a), math.cos(a)])  # the flap's upper face normal
        pt = pivot + R_PUSH * np.array([0.0, -math.cos(a), math.sin(a)]) + (0.006 + off) * n
        return (pt[0], pt[1], pt[2] + PAD_TIP_PAST_TCP)

    wps += [
        Waypoint(1.2, (bx, by + 0.02, TOP + 0.150), fingers, "raise the closed fingers clear of the open retainer", rot=ROT_G),
        Waypoint(2.0, on_flap(OPEN_DEG, 0.030), fingers, "over the retainer, behind it", rot=ROT_G),
        Waypoint(1.0, on_flap(OPEN_DEG, 0.018), fingers, "lower toward the retainer's back face", rot=ROT_G),
        Waypoint(1.0, on_flap(OPEN_DEG, 0.002), fingers, "pad tips on the retainer", rot=ROT_G),
        Waypoint(1.5, on_flap(35.0, 0.0), fingers, "push the retainer down", rot=ROT_G),
        Waypoint(1.5, on_flap(-8.0, -0.003), fingers, "to the latch", rot=ROT_G),
        Waypoint(1.0, on_flap(-8.0, -0.003), fingers, "hold it latched", rot=ROT_G),
        Waypoint(1.5, on_flap(0.0, 0.06), fingers, "lift away", rot=ROT_G),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return wps


def demo():
    """The right arm seats the roll and latches the retainer."""
    return {"right": right_plan()}
