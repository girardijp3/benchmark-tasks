"""On a workshop back board: a wall-mounted 600 mm T8 batten with two G13 rotor lampholders, and in front of it, lying in two bench
cradles, a new 18 W-class T8 tube (590 mm cap to cap, 26 mm, 0.13 kg, bi-pins 12.7 mm apart).

The robot has to fit the tube the way a maintenance worker does: carry it level to the batten, slide both pin pairs into the
notches and slots of both rotor lampholders at once, and turn the tube a quarter turn until both rotors click into their stops;
the tube then lights. Both hands are needed: the tube is 590 mm long and has to go into two holders at once.

Every holder is a fixed housing (a round pocket and a front entry notch with a lead-in) round a rotor (a 3.2 mm diametral slot,
on a revolute joint about the tube's axis). At 0 degrees the notch lines up with the slot, so the pins slide in from the front;
the pins then ride in the slots, the tube turns the rotors, and at 90 degrees the pins lie behind the pocket wall, where the
notch no longer lines up, so the tube cannot come out. The rotor's click is a declared detent (a drive that pulls a rotor
past 45 degrees on to 90 and a rotor short of 45 back to 0, peak 0.1 N m). The lamp is lit (four strips on the tube show) only while
both pin pairs are in both slots and both rotors are at 90 degrees, +-5.

Run it:  inspect-robots run --embodiment yam_arena -E task=t8_tube_rotor_lampholders.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "t8lamp"
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
XT = 0.42  # the holders' axis (the tube's axis when it is fitted)
AX = 0.115  # the tube's axis above the table
TUBE_D, TUBE_L, TUBE_MASS = 26.0 * MM, 590.0 * MM, 0.13
PIN_R, PIN_SP, PIN_OUT = 1.235 * MM, 12.7 * MM, 7.5 * MM
HALF_GAP = 298.0 * MM  # a holder's inner face, from the middle
CRADLE_DX = 0.090  # the tube lies this far in front of the holders
TUBE_XYZ = np.array([XT - CRADLE_DX, 0.0, TOP + AX])
TUBE_FITTED = np.array([XT, 0.0, TOP + AX])
BATTEN = UsdPart("batten", asset="batten", xy=(XT, 0.0), extent=0.40, mass=2.0, usd_path=str(ASSETS / "batten.usda"),
                 pos=(XT, 0.0, TOP), articulation=True, fixed=True)
CRADLES = UsdPart("cradles", asset="cradles", xy=(XT - CRADLE_DX, 0.0), extent=0.10, mass=0.5, usd_path=str(ASSETS / "cradles.usda"),
                  pos=(XT - CRADLE_DX, 0.0, TOP), kinematic=True)
TUBE = UsdPart("tube", asset="tube", xy=(TUBE_XYZ[0], 0.0), extent=0.30, mass=TUBE_MASS, usd_path=str(ASSETS / "t8_tube.usda"),
               pos=tuple(float(v) for v in TUBE_XYZ), articulation=True)
OBJECTS = [BATTEN, CRADLES, TUBE]
MOVERS = ("tube",)
TUBE_LO = np.array([-TUBE_D / 2, -TUBE_L / 2, -TUBE_D / 2])
TUBE_HI = np.array([TUBE_D / 2, TUBE_L / 2, TUBE_D / 2])

# -- the declared detent and the lamp -------------------------------------------------------------------------------------
ROT_JOINTS = ("RotL", "RotR")
GLOW_JOINTS = ("Glow0", "Glow1", "Glow2", "Glow3")
GLOW_LIT = (0.0035, -0.0035, 0.0035, -0.0035)
CLICK = math.radians(45.0)  # a rotor turned past this is pulled on to its stop ...
STOP = math.radians(90.0)  # ... at 90 degrees (turning is negative about +y)
STOP_TOL = math.radians(5.0)
DETENT_PEAK = 0.1  # N m: the click's peak torque (an estimate; set in the USD as the drive stiffness)


def _write_targets(art, names, values) -> None:
    """Writes joint drive targets (``names`` order) to PhysX directly: Arena gives task objects no actuators."""
    import torch

    view = art.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    dofs = list(view.shared_metatype.dof_names)
    cols = [dofs.index(n) for n in names]
    t[:, cols] = torch.as_tensor(np.asarray(values, np.float32), device=t.device)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, art._ALL_INDICES)


def _roll_of(state) -> float:
    """The tube's turn about its own axis (rad, about +y): where its pin line (local x) points."""
    r = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))
    return float(math.atan2(-r[2, 0], r[0, 0]))


def engaged(objects) -> bool:
    """Both pin pairs are in both holders: the tube's axis within 3.5 mm of the holders' axis at both ends, along y within +-2.9 mm
    (the pins 1.5 mm or more past each holder's face), and level within 3 degrees."""
    s = objects["tube"]
    r = quat_xyzw_to_matrix(np.asarray(s.quat_xyzw, float))
    ax = r[:, 1]
    p = np.asarray(s.pos, float)
    for sg in (1.0, -1.0):
        e = p + sg * (TUBE_L / 2) * ax
        if abs(e[0] - XT) > 0.0035 or abs(e[2] - (TOP + AX)) > 0.0035:
            return False
    return bool(abs(p[1]) <= 0.0029 and abs(ax[1]) >= math.cos(math.radians(3.0)))


class _Lamp:
    """The click detent and the lamp, once per control step: each rotor's drive target is 0 until it passes 45 degrees and 90 after;
    the strips show only while the tube is engaged and both rotors are at their stops."""

    def __init__(self):
        self.time = -1.0
        self.tgt = [0.0, 0.0]
        self.lit = False
        self.shown = None

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        a = [float(objects.get("rotL", 0.0)), float(objects.get("rotR", 0.0))]
        for i in range(2):
            self.tgt[i] = -STOP if a[i] <= -CLICK else 0.0
        self.lit = bool(engaged(objects) and all(abs(v + STOP) <= STOP_TOL for v in a))


_LAMP = _Lamp()


def extra_states(base, out, origin) -> None:
    """The rotors' angles, the detent targets and the lamp strips (once every control step)."""
    art = base.scene["batten"]
    ids = art.find_joints(list(ROT_JOINTS), preserve_order=True)[0]
    q = art.data.joint_pos.torch[0, ids].detach().cpu().numpy()
    out["rotL"], out["rotR"] = float(q[0]), float(q[1])
    _LAMP.update(out)
    key = (tuple(_LAMP.tgt), _LAMP.lit)
    if _LAMP.shown != key:
        _write_targets(art, list(ROT_JOINTS), list(_LAMP.tgt))
        _write_targets(base.scene["tube"], list(GLOW_JOINTS), list(GLOW_LIT) if _LAMP.lit else [0.0] * 4)
        _LAMP.shown = key


def _reset_all(env, env_ids, asset_cfg) -> None:
    """Reset event: rotors at 0 with drive target 0, strips hidden, the detent started over."""
    import torch

    if env_ids is None:
        return
    for name, joints in (("batten", ROT_JOINTS), ("tube", GLOW_JOINTS)):
        art = env.scene[name]
        ids = art.find_joints(list(joints), preserve_order=True)[0]
        zero = torch.zeros((len(env_ids), len(ids)), device=art.device)
        art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
        art.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
        _write_targets(art, list(joints), [0.0] * len(joints))
    _LAMP.__init__()
    _LAMP.shown = ((0.0, 0.0), False)


def reset_events() -> list:
    from isaaclab.managers import SceneEntityCfg

    return [("t8_reset", _reset_all, {"asset_cfg": SceneEntityCfg("tube")})]


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
DROP_Z = 0.035  # the tube falling this far below its cradles' height has been dropped (it would break)
STEPS = ("both pin pairs in both rotor slots (tube level on the holders' axis)", "both rotors turned 45 degrees or more",
         "both rotors at their 90 degree stops: click, tube lit", "lit, let go, grippers clear of the tube")


def rotors(objects) -> tuple[float, float]:
    return float(objects.get("rotL", 0.0)), float(objects.get("rotR", 0.0))


def turned(objects) -> bool:
    return bool(all(a <= -CLICK for a in rotors(objects)))


def at_stops(objects) -> bool:
    return bool(all(abs(a + STOP) <= STOP_TOL for a in rotors(objects)))


def tube_held(objects) -> bool:
    return any(holds(objects, s, "tube", TUBE_LO, TUBE_HI) for s in ("left", "right"))


def parts_free(objects) -> bool:
    if tube_held(objects):
        return False
    return not pads_near(objects, "tube", TUBE_LO, TUBE_HI, CLEAR)


def _steps(objects) -> list[bool]:
    _LAMP.update(objects)
    s1 = engaged(objects)
    s2 = bool(s1 and turned(objects))
    s3 = bool(s2 and at_stops(objects) and _LAMP.lit)
    s4 = bool(s3 and parts_free(objects))
    return [s1, s2, s3, s4]


def events_now(objects, p) -> list[str]:
    out = []
    if objects["tube"].pos[2] < TOP + AX - DROP_Z:
        out.append("the tube was dropped (a fluorescent tube breaks)")
    return out


def trace_line(objects) -> str:
    s = objects["tube"]
    ra, rb = rotors(objects)
    return (f"tube at ({(s.pos[0] - XT) * 1000:.1f},{s.pos[1] * 1000:.1f},{(s.pos[2] - TOP - AX) * 1000:.1f}) mm{'*' if tube_held(objects) else ''}, roll {math.degrees(_roll_of(s)):.0f} deg, "
            f"rotors {math.degrees(ra):.0f}/{math.degrees(rb):.0f} deg, engaged {engaged(objects)}, lit {_LAMP.lit}")


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
# Both arms take the tube from its cradles by pads closing along x (grip straight down, tilt 0), lift it clear, carry it to
# 40 mm in front of the holders, and push it in along +x: both pin pairs slide down both entry notches into both slots. Then both
# wrists tilt together from 0 to -90 degrees about the tube's axis (the pads stay on it), which turns both rotors to their
# stops (the lamp lights). They hold a moment, let go and back away along -x; the tube stays fitted.
_X, _Y, _Z = np.eye(3)
_grip = yam_demo.grip_rotation


def tilt(deg: float) -> np.ndarray:
    """A grip straight down (pads closing along x), tilted ``deg`` about the world y axis (negative: the fingers point toward +x)."""
    return axis_angle_matrix(_Y, math.radians(deg)) @ _grip(math.pi / 2)


OPEN = FINGER_OPEN
SQUEEZE = 0.008
HOLD = -(TUBE_D - SQUEEZE) / 2
LOOSE = -(TUBE_D - 0.0015) / 2  # a light hold while the pins go in (the lead-in turns the tube in the hand)
GRIP_Y = 0.20  # each pad pair this far either side of the middle
LIFT = 0.016
SAG = 0.0
INSERT_DX = 0.040
PUSH = 0.002  # the hands keep pushing the tube into the back of the pockets while they turn it
TURN_DEG = -92.0
TURN_S = 9.0
TURN_WP = 12
AIM_X = 0.0
AIM_Y = 0.0
AIM_Z = 0.0


ERR_X = (-0.090, -0.040, 0.0)  # where the held tube was measured (x from the holders' axis) ...
ERR_DZ = (0.0, 0.0, 0.0)  # ... how far above (+) its command it sat ... (measured: none worth correcting)
ERR_DX = (0.0, 0.0, 0.0)  # ... and how far in front (-)


TILT_ABS = (0.0, 8.0, 18.0, 29.0, 38.0, 49.0, 59.0, 68.0, 78.0, 85.0, 90.0, 95.0)  # the arms sag more the further the wrist is tilted (measured with the tube held in free air)
TILT_DX = np.array([-1.0, -4.5, -8.0, -6.7, -4.9, -7.3, -5.4, -3.1, -3.0, -0.8, -0.7, -0.7]) * MM
TILT_DZ = np.array([-1.9, -2.8, -4.3, -5.4, -5.1, -9.8, -9.1, -8.8, -16.0, -25.7, -24.0, -24.0]) * MM


def _tcp(side, x, z, th=0.0):
    """The TCP command that puts the held tube at (x, z) with the wrist tilted ``th`` degrees: the arms sag by measured amounts."""
    sg = 1.0 if side == "left" else -1.0
    dx = float(np.interp(abs(th), TILT_ABS, TILT_DX))
    dz = float(np.interp(abs(th), TILT_ABS, TILT_DZ))
    return np.array([x - dx + AIM_X, sg * GRIP_Y + AIM_Y, z - dz + AIM_Z])


def plan(side):
    x0, z0 = TUBE_XYZ[0], TUBE_XYZ[2]
    r0 = tilt(0.0)
    wps = [
        Waypoint(2.5, _tcp(side, x0, z0 + 0.07), OPEN, "over the tube", rot=r0),
        Waypoint(1.5, _tcp(side, x0, z0), OPEN, "pads astride the tube", rot=r0),
        Waypoint(0.8, _tcp(side, x0, z0), HOLD, "grip the tube", rot=r0),
        Waypoint(1.2, _tcp(side, x0, z0 + LIFT), HOLD, "lift it off the cradles", rot=r0),
        Waypoint(2.5, _tcp(side, XT - INSERT_DX, z0 + LIFT), HOLD, "carry it to the batten", rot=r0),
        Waypoint(2.0, _tcp(side, XT - INSERT_DX, z0 + SAG), HOLD, "down to the holders' height", rot=r0),
        Waypoint(1.5, _tcp(side, XT - INSERT_DX, z0 + SAG), HOLD, "steady", rot=r0),
        Waypoint(1.0, _tcp(side, XT - INSERT_DX, z0 + SAG), LOOSE, "ease the grip", rot=r0),
        Waypoint(4.0, _tcp(side, XT + 0.030, z0 + SAG), LOOSE, "slide the pins into the holders", rot=r0),
        Waypoint(1.0, _tcp(side, XT + PUSH, z0 + SAG), HOLD, "grip firmly again", rot=r0),
    ]
    for i in range(1, TURN_WP + 1):
        wps.append(Waypoint(TURN_S / TURN_WP, _tcp(side, XT + PUSH, z0 + SAG, TURN_DEG * i / TURN_WP), HOLD, "turn the tube", rot=tilt(TURN_DEG * i / TURN_WP)))
    end = tilt(TURN_DEG)
    wps += [
        Waypoint(1.5, _tcp(side, XT + PUSH, z0 + SAG, TURN_DEG), HOLD, "hold", rot=end),
        Waypoint(0.8, _tcp(side, XT + PUSH, z0 + SAG, TURN_DEG), OPEN, "let go", rot=end),
        Waypoint(1.5, _tcp(side, XT - 0.08, z0 + SAG, TURN_DEG), OPEN, "back away", rot=end),
        Waypoint(2.5, tuple(_START_TCP[side]), FINGER_OPEN, "home", rot=_START_ROT[side]),
    ]
    return wps


def demo():
    """Both arms lift the tube, carry it to the batten, push it in and turn it a quarter turn; then let go."""
    return {"left": plan("left"), "right": plan("right")}
