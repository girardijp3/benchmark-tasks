"""In an installed front wheel well: a worn, in-service Brembo 09.7012.11 280 mm vented brake rotor (22.0 mm new, 21.50 mm measured
at the inspection band here, 19 mm discard, ~5.74 kg worn) is bolted to its wheel hub and steering knuckle, with the caliper
unbolted and its carrier bracket still on the knuckle. Eight scribe marks at radius 135 mm on the outboard friction face show the
inspection points, a 6 mm paint dot marking the thinnest one (135 mm from the hub axis, 10 deg above the 9 o'clock line, facing the
robot). To its left, lying across a bench rest, is a 0-25 mm outside micrometer (Mitutoyo 103-137 class: 175 g, ratchet stop, lock
lever) with its spindle run out to the full 25 mm.

The robot has to measure the rotor the way a mechanic does: one hand holds the micrometer by its frame and slips it over the rotor's
outer edge so the two measuring faces sit square on the friction band and rocks the frame slightly across the face, the other hand turns
the ratchet knob until it clicks (the ratchet slips at the measuring force: 5-10 N on the real tool, 6 N here with the drive capped at
10 N), then the micrometer is taken off with the reading kept on the thimble and put back on its rest. The lock lever works but is not
scored and the instruction does not ask for it.

The rotor is an in-service part being inspected for wear: the simulator's worn thickness at the marked spot is 21.50 mm (the part
was 22.0 mm new and is discarded at 19 mm). A real trial first scans the eight inspection points to find the thinnest, marks it,
and scores the robot's reading against that spot's thickness measured with a calibrated micrometer (README: eight points, then five
readings at the thinnest, median, repeatability <= 0.005 mm, +-0.02 mm band).

The micrometer is a real articulation: the frame is a free root; the spindle slides along the measuring axis on a prismatic joint whose
drive is limited to 10 N (the top of the real 5-10 N measuring force); the thimble and the ratchet knob turn about that axis on revolute joints; the lock lever turns
about a horizontal axis. The thread is declared (Screw couplings are not joints in PhysX): every control step the task reads how far the
ratchet knob has turned and moves the spindle's drive target 0.5 mm per turn; the thimble follows the knob; once the spindle is held
back by the rotor the thimble stops and the knob slips (a torque-limited drive of 0.006 N m, a click every tooth of 18 degrees). The lever
is a declared detent (a drive that throws it on past its middle); thrown, it freezes the spindle where it stands.

Run it:  inspect-robots run --embodiment yam_arena -E task=brake_rotor_micrometer.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "mic"
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
XM, YA, ZA = 0.375, 0.1325, 0.152  # the measuring pose: anvil face centre (the micrometer's origin); axis along y, z above the table
XS, YS = 0.330, 0.165  # on the rest: the anvil face centre; the frame lies across the ribs
NEW_MM = 22.00  # the Brembo 09.7012.11's new thickness (catalogue); DISCARD_MM is the stamp on the part
DISCARD_MM = 19.00  # the minimum thickness stamped on the rotor: below this it is scrapped
REFERENCE_MM = 21.50  # the worn in-service rotor's measured thickness at the marked spot in this simulator
REFERENCE_TOL_MM = 0.02  # the simulator's reading band; a real trial uses it when its reference is measured to 0.001 mm (README)
REFERENCE_REPEAT_MM = 0.005  # the real reference's five-reading spread must stay within this
INSPECTION_R = 0.135  # radius of the eight real-trial inspection points (m; 5 mm inboard of the 140 mm friction rim)
INSPECTION_N = 8  # eight equally spaced circumferential points (every 45 deg) on each friction face
DOT_ANG = math.radians(10.0)  # the marked thinnest inspection point: 10 deg above the 9 o'clock line
THICK = REFERENCE_MM * MM  # the simulator's reference thickness (the real rotor's is measured per trial; README)
G_OPEN = 25.0 * MM
PITCH = 0.5 * MM
ROT_R = 0.140
XC, YR = XM + INSPECTION_R * math.cos(DOT_ANG), YA - 1.5 * MM - THICK / 2  # hub centre (axis along y at ZA): the faces sit 5 mm inside the 140 mm rim
BAND_LO, BAND_HI = 0.100, 0.136  # the faces' centre lies on the friction band between these radii (m from the hub axis)
DOT_X, DOT_Z = XM, TOP + ZA + INSPECTION_R * math.sin(DOT_ANG)  # 135 mm from the hub axis, on the outboard face
DOT_TOL = 0.005  # the anvil face centre must be this close to the dot (m), so the reference and the reading are the same spot
READ_LO, READ_HI = -REFERENCE_TOL_MM * MM, REFERENCE_TOL_MM * MM  # a reading counts when it is within this of the measured thickness (MuJoCo's soft contact sinks the faces a few hundredths of a mm at 6 N; PhysX sinks almost nothing)
MIC_START = np.array([XS, YS, TOP + ZA + 0.0003])
MIC = UsdPart("mic", asset="micrometer", xy=(XS, YS), extent=0.06, mass=0.176, usd_path=str(ASSETS / "micrometer.usda"),
              pos=tuple(float(v) for v in MIC_START), articulation=True)
ROTOR = UsdPart("rotor", asset="rotor", xy=(XC, YR), extent=0.15, mass=5.74, usd_path=str(ASSETS / "rotor.usda"),
                pos=(XC, YR, TOP + ZA), kinematic=True)  # worn disc; installed on the hub below
HUB_KNUCKLE = UsdPart("hub_knuckle", asset="hub_knuckle", xy=(XC, YR), extent=0.22, mass=12.0,
                      usd_path=str(ASSETS / "hub_knuckle.usda"), pos=(XC, YR, TOP + ZA), kinematic=True)
CALIPER_BRACKET = UsdPart("caliper_bracket", asset="caliper_bracket", xy=(XC, YR - 0.05), extent=0.16, mass=2.4,
                          usd_path=str(ASSETS / "caliper_bracket.usda"), pos=(XC, YR, TOP + ZA), kinematic=True)
REST = UsdPart("rest", asset="rest", xy=(XS, YS), extent=0.05, mass=0.75, usd_path=str(ASSETS / "rest.usda"),
               pos=(XS, YS, TOP), kinematic=True)
OBJECTS = [ROTOR, HUB_KNUCKLE, CALIPER_BRACKET, REST, MIC]  # the wheel-well hardware is fixed installed context
MOVERS = ("mic",)
MIC_LO = np.array([-39.3 * MM, -35.0 * MM, -6.0 * MM])
MIC_HI = np.array([9.5 * MM, 13.0 * MM, 6.0 * MM])  # the frame, in the micrometer's frame
RAT_LO = np.array([-7.5 * MM, -12.0 * MM, -7.5 * MM])
RAT_HI = np.array([7.5 * MM, 0.0, 7.5 * MM])  # the ratchet knob, in its link's frame

# -- the declared thread, ratchet and lock ---------------------------------------------------------------------------------
TURN = 2 * math.pi
JOINTS = ("Spindle", "Thimble", "Ratchet", "Lever")
LEV_X, LEV_Y = -0.027, -0.0305  # the lock lever's pivot in the micrometer's frame (on the spindle arm's top face)
LAG = 0.03 * MM  # the spindle is held back when it trails its command by this much (the drive's force there: 6 N of the 10 N limit)
TEETH = 20  # ratchet teeth: a click every 18 degrees of slip (an estimate)
TOOTH = TURN / TEETH
LEV_UNLOCK, LEV_LOCK = math.radians(35.0), math.radians(-35.0)  # lever angle about +x (positive: the top leans toward the thimble)
LEV_ON, LEV_OFF = math.radians(-20.0), math.radians(20.0)  # thrown past -20 degrees = locked; back past +20 = free
SPINDLE_HI = G_OPEN - 0.1 * MM


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


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def reading_m(objects) -> float:
    """The faces' separation (the micrometer's reading, m)."""
    return G_OPEN - float(objects.get("q_s", 0.0))


def reading_ok(objects) -> bool:
    d = reading_m(objects) - THICK
    return bool(READ_LO <= d <= READ_HI)


def on_rotor(objects) -> bool:
    """Both faces sit on the rotor's friction band, square: the anvil face centre on the band (BAND_LO .. BAND_HI from the hub axis) and
    within DOT_TOL of the reference dot (135 mm from the hub axis, 10 deg above 9 o'clock), the anvil face on the rotor's +y face (within
    1 mm), the measuring axis within 5 degrees of the hub axis."""
    s = objects["mic"]
    p = np.asarray(s.pos, float)
    r = math.hypot(p[0] - XC, p[2] - (TOP + ZA))
    at_dot = math.hypot(p[0] - DOT_X, p[2] - DOT_Z) <= DOT_TOL
    ay = _rot(s)[:, 1]
    return bool(BAND_LO <= r <= BAND_HI and at_dot and abs(p[1] - (YR + THICK / 2)) <= 0.001
                and ay[1] >= math.cos(math.radians(5.0)))


def gauged_now(objects) -> bool:
    """The micrometer is on the rotor, the spindle is held against it (the ratchet's measuring force) and the reading is its measured thickness."""
    return bool(on_rotor(objects) and reading_ok(objects))


class _Mic:
    """The declared thread, ratchet and lock lever, once per control step.

    ``t_cmd`` is the thimble's commanded angle (rad, closing positive: about +y); the spindle's drive target is ``PITCH * t_cmd / TURN``. The
    ratchet knob's angle ``th_r`` is read each step. Free: the thimble follows the knob both ways (``r_ref`` follows the knob). Held back (the
    spindle trails its command by LAG: the faces are on something, the drive pushes with its force limit): the thimble is held there and the
    knob slips, its drive (limited to the slip torque) pulling it back to ``r_ref``; each TOOTH of slip is a click and moves ``r_ref`` on;
    turning the knob back drives the thimble back. The lever, thrown past LEV_ON, freezes the spindle where it stands (``q_lock``); the
    thimble holds still and the knob only slips. The lever's drive target is the nearer stop (a detent)."""

    def __init__(self):
        self.time = -1.0
        self.last_r = None
        self.t_cmd = 0.0
        self.r_ref = 0.0
        self.locked = False
        self.q_lock = 0.0
        self.held = False  # the spindle is held back
        self.touched = False  # the faces have closed on the rotor: from then on the ratchet keeps its push and the thimble never backs off by itself
        self.clicks = 0  # clicks of the knob while the micrometer was gauged (on the rotor at its thickness)
        self.clicks_any = 0
        self.ever_gauged = False
        self.q_hold = None  # the spindle's position at the last good contact: the thimble stays there when the micrometer is lifted off
        self.lever_t = LEV_UNLOCK
        self.tgt = [0.0, 0.0, 0.0, LEV_UNLOCK]
        self.shown = None

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        if "q_s" not in objects:
            return
        q_s, th_r, phi = float(objects["q_s"]), float(objects["th_r"]), float(objects["phi_l"])
        self.lever_t = LEV_LOCK if phi < 0.0 else LEV_UNLOCK
        if phi <= LEV_ON and not self.locked:
            self.locked, self.q_lock = True, q_s
            self.t_cmd = TURN * q_s / PITCH
        elif phi >= LEV_OFF and self.locked:
            self.locked = False
            self.r_ref = th_r
        if self.last_r is None:
            self.last_r, self.r_ref = th_r, th_r
        e = th_r - self.r_ref
        gauged = gauged_now(objects)
        if self.locked:
            if e < 0.0:
                self.r_ref = th_r  # turning back: the thimble is locked, the knob just follows
            elif e >= TOOTH:
                self.r_ref += TOOTH
                self.clicks_any += 1
        else:
            if self.held and e > 0.0:
                if e >= TOOTH:  # a tooth slipped: a click (counted while the faces are on the rotor, the spindle held at the measuring force)
                    self.r_ref += TOOTH
                    self.clicks_any += 1
                    if on_rotor(objects):
                        self.clicks += 1
            else:
                self.t_cmd += e
                self.r_ref = th_r
                if e < 0.0:
                    self.touched = False  # the knob turned back: the spindle opens
            if gauged:
                self.q_hold = q_s  # the reading as the frame rocks through square: the thimble keeps the last good one when the micrometer is lifted off
            lag = PITCH * self.t_cmd / TURN - q_s
            if self.q_hold is not None and not on_rotor(objects):
                self.t_cmd = TURN * self.q_hold / PITCH  # off the rotor: the thimble holds its reading (thread friction), the spindle does not creep closed
                self.held = False
                self.touched = False
                lag = 0.0
            elif self.touched:
                # Rocking technique: the ratchet keeps pushing at the measuring force, so as the frame rocks toward square the spindle follows the
                # faces closer (the target follows the spindle closing, never opening): the reading settles at the minimum, the true thickness.
                self.t_cmd = max(self.t_cmd, TURN * (q_s + LAG) / PITCH)
                self.held = True
                lag = PITCH * self.t_cmd / TURN - q_s
            elif lag > LAG:
                self.t_cmd = TURN * (q_s + LAG) / PITCH
                self.held = True
                self.touched = bool(on_rotor(objects))
            elif lag < 0.5 * LAG:
                self.held = False
            if self.t_cmd < 0.0:
                self.t_cmd = 0.0
                self.r_ref = th_r
        self.last_r = th_r
        if gauged:
            self.ever_gauged = True
        q_cmd = self.q_lock if self.locked else min(max(PITCH * self.t_cmd / TURN, 0.0), SPINDLE_HI)
        self.tgt = [q_cmd, self.t_cmd, self.r_ref, self.lever_t]


_MIC = _Mic()


def extra_states(base, out, origin) -> None:
    """The micrometer's joints and links (spindle position, ratchet and lever angles, the ratchet knob as ``ratchet``), the declared thread,
    ratchet and lock (once every control step), and the drive targets it writes."""
    art = base.scene["mic"]
    ids = art.find_joints(list(JOINTS), preserve_order=True)[0]
    q = art.data.joint_pos.torch[0, ids].detach().cpu().numpy()
    out["q_s"], out["th_t"], out["th_r"], out["phi_l"] = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    idx = art.find_bodies("Ratchet")[0][0]
    pose = _arr(art.data.body_link_pose_w)[idx]
    pos = pose[:3] - origin
    out["ratchet"] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())
    _MIC.update(out)
    key = tuple(_MIC.tgt)
    if _MIC.shown != key:
        _write_targets(art, list(JOINTS), list(key))
        _MIC.shown = key


def _reset_mic(env, env_ids, asset_cfg) -> None:
    """Reset event: spindle fully open, thimble and knob at 0, lever unlocked, drive targets to match, the declared state started over."""
    import torch

    if env_ids is None:
        return
    art = env.scene[asset_cfg.name]
    ids = art.find_joints(list(JOINTS), preserve_order=True)[0]
    pos = torch.zeros((len(env_ids), len(ids)), device=art.device)
    pos[:, 3] = LEV_UNLOCK
    art.write_joint_position_to_sim_index(position=pos, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=env_ids)
    _write_targets(art, list(JOINTS), [0.0, 0.0, 0.0, LEV_UNLOCK])
    _MIC.__init__()
    _MIC.shown = (0.0, 0.0, 0.0, LEV_UNLOCK)


def reset_events() -> list:
    from isaaclab.managers import SceneEntityCfg

    return [("mic_reset", _reset_mic, {"asset_cfg": SceneEntityCfg("mic")})]


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
DROP_VZ = -0.6  # a micrometer that nothing holds falling faster than this (m/s: about 18 mm of free fall) has been dropped
MIN_CLICKS = 2  # clicks of the ratchet at the measuring force
STEPS = ("the micrometer closed square on the rotor's friction band, its spindle held against the rotor (reading = its measured thickness)",
         "the ratchet clicked at the measuring force (at least two clicks)",
         "the micrometer lifted off the rotor with its reading kept at the measured thickness",
         "the micrometer back on its rest still showing that reading, let go, grippers clear")


def mic_held(objects) -> bool:
    return any(holds(objects, s, "mic", MIC_LO, MIC_HI) for s in ("left", "right"))


def on_rest(objects) -> bool:
    s = objects["mic"]
    p = np.asarray(s.pos, float)
    r = _rot(s)
    return bool(abs(p[0] - XS) <= 0.012 and abs(p[1] - YS) <= 0.012 and abs(p[2] - MIC_START[2]) <= 0.004
                and r[2, 2] >= math.cos(math.radians(6.0)) and abs(r[1, 1]) >= math.cos(math.radians(6.0)))


def parts_free(objects) -> bool:
    if mic_held(objects) or any(holds(objects, s, "ratchet", RAT_LO, RAT_HI) for s in ("left", "right")):
        return False
    return not (pads_near(objects, "mic", MIC_LO, MIC_HI, CLEAR) or pads_near(objects, "ratchet", RAT_LO, RAT_HI, CLEAR))


def locked_reading(objects) -> bool:
    return bool(_MIC.locked and reading_ok(objects))


def _steps(objects) -> list[bool]:
    _MIC.update(objects)
    s1 = bool(_MIC.ever_gauged)
    s2 = bool(s1 and _MIC.clicks >= MIN_CLICKS)
    s3 = bool(s2 and not on_rotor(objects) and reading_ok(objects))
    s4 = bool(s3 and on_rest(objects) and parts_free(objects))
    return [s1, s2, s3, s4]


def events_now(objects, p) -> list[str]:
    _MIC.update(objects)
    out = []
    s = objects["mic"]
    if s.pos[2] < TOP - 0.03:
        out.append("the micrometer fell off the table")
    elif not mic_held(objects) and not on_rotor(objects) and float(s.lin_vel[2]) < DROP_VZ:
        out.append("the micrometer was dropped (a micrometer is a precision instrument)")
    return out


def trace_line(objects) -> str:
    s = objects["mic"]
    return (f"mic at ({(s.pos[0] - XM) * 1000:.0f},{(s.pos[1] - YA) * 1000:.1f},{(s.pos[2] - TOP - ZA) * 1000:.1f}) mm{'*' if mic_held(objects) else ''}, "
            f"reading {reading_m(objects) * 1000:.3f} mm, thimble {math.degrees(_MIC.t_cmd):.0f} deg, knob {math.degrees(float(objects.get('th_r', 0.0))):.0f} deg, "
            f"axis tilt {math.degrees(math.asin(max(-1.0, min(1.0, float(_rot(s)[2, 1]))))):.2f} deg up / {math.degrees(math.atan2(float(_rot(s)[0, 1]), float(_rot(s)[1, 1]))):.2f} deg yaw, "
            f"held {_MIC.held}, clicks {_MIC.clicks}/{_MIC.clicks_any}, on rotor {on_rotor(objects)}, lever {math.degrees(float(objects.get('phi_l', 0.0))):.0f} deg locked {_MIC.locked}")


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
# The left arm (level, pointing +x, pads closing up and down) takes the micrometer's frame by the spine, lifts it 25 mm off the rest,
# carries it to the rotor's outer edge and slides it on (+x) so the anvil and spindle faces straddle the friction band. The right
# arm (level, pointing +y, pads closing sideways) takes the ratchet knob end-on and turns it with wrist rolls, letting go and
# re-gripping between strokes, until the faces are on the rotor and the ratchet clicks, while the left arm rocks the frame about
# the anvil face (it does not throw the lock lever). The left arm takes the micrometer off the rotor and puts it back on its rest.
_X, _Y, _Z = np.eye(3)
_DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
_grip = yam_demo.grip_rotation


def side_grip(approach: float, roll: float = 0.0) -> np.ndarray:
    """The gripper pointing horizontally along ``approach`` (rad from +x), fingers closing horizontally at roll 0; ``roll`` turns it about its pointing axis."""
    forward = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN  # pointing +x, fingers closing along y
    return axis_angle_matrix(_Z, approach) @ axis_angle_matrix(_X, roll) @ forward


OPEN = FINGER_OPEN
FRAME_T = 0.012
FRAME_HOLD = -(FRAME_T - 0.008) / 2  # about 20 N a finger
RAT_D = 0.015
RAT_HOLD = -(RAT_D - 0.008) / 2
CLOSED = -0.001  # the right pads shut, as a pusher
TRIM_P, TRIM_Y = math.radians(-2.1), math.radians(3.55)  # the grip trails about 2 deg nose-up and 3.5 deg of yaw (Isaac) (measured, MuJoCo and Isaac): the left hand is turned this much the other way so the faces sit flat on the rotor (the micrometer's own squareness: 1 deg costs 0.11 mm of reading)
LEFT_ROT = axis_angle_matrix(_Z, TRIM_Y) @ axis_angle_matrix(_X, TRIM_P) @ side_grip(0.0, math.pi / 2)  # pointing +x, pads closing along z
RIGHT_R0 = side_grip(math.pi / 2, 0.0)  # pointing +y, pads closing along x
TIPS_X = -0.030  # the left pads' tips, in the micrometer's frame: flush with the spine's inner face
PINCH_Y = -0.008
LIFT = 0.025  # the lower pad is 10 mm or so deep: lift this far before sliding sideways
L_AIM = (0.0, 0.0, 0.0)  # aim trims (m)
PRESS_DY = 0.0035  # after sliding on, the frame is pushed this far toward the rotor: the anvil face goes against the rotor's face
L_SAG = np.array([0.001, -0.0002, 0.0115])  # the left arm trails its command this much when it carries the frame raised (measured in MuJoCo; re-measure on Isaac)
R_AIM = (0.0, 0.0, 0.0)
INSERT_DX = XM - XS
RAT_TIP_Y = -0.0895  # the right pad tips, in the micrometer's frame (y of the ratchet knob's end -100 mm plus 10.5 mm)
STROKE = math.radians(120.0)  # the wrist rolls this far a stroke
STROKES = 40
STROKE_S = 1.0
STROKE_WP = 5
LEV_TIP_X = LEV_X
LEV_Z = 0.023  # the closed pads' centre above the axis
LEV_TIP_Y0 = LEV_Y - 0.014  # the pad tips start this far behind the lever's pivot ...
LEV_TIP_Y1 = LEV_Y + 0.004  # ... and are pushed to here


def _left_tcp(p, dx=0.0, dy=0.0, dz=0.0):
    """The left TCP gripping the frame of a micrometer whose anvil face centre is at ``p`` (+ offsets)."""
    return np.array([p[0] + TIPS_X - PAD_TIP_PAST_TCP + dx + L_AIM[0], p[1] + PINCH_Y + dy + L_AIM[1], p[2] + dz + L_AIM[2]])


def _right_tcp(x, y_tip, z):
    return np.array([x + R_AIM[0], y_tip - PAD_TIP_PAST_TCP + R_AIM[1], z + R_AIM[2]])


Z_MEAS = ZA + 0.0235  # the micrometer's height while measured (lifted 25 mm; the arm trails it about 1.5 mm)
P_START = np.array([XS, YS, TOP + ZA])
P_MEAS = np.array([XM, YA, TOP + ZA])
P_APPR = np.array([XS, YA, TOP + ZA])  # beside the rotor, 45 mm short of the edge


def right_plan(start_t: float):
    """Wait, take the ratchet knob, turn it in strokes, let go and go home. Returns the plan and the time the hand backed off the knob."""
    knob = _right_tcp(XM, YA + RAT_TIP_Y, TOP + Z_MEAS)
    pre = knob + np.array([0.0, -0.05, 0.0])
    wps = [
        Waypoint(5.0, pre, OPEN, "ready behind the knob", rot=RIGHT_R0),
        Waypoint(max(start_t - 5.0, 0.1), pre, OPEN, "wait for the micrometer", rot=RIGHT_R0),
        Waypoint(1.5, knob, OPEN, "pads round the knob", rot=RIGHT_R0),
        Waypoint(0.6, knob, RAT_HOLD, "grip the knob", rot=RIGHT_R0),
    ]
    for k in range(STROKES):
        for i in range(1, STROKE_WP + 1):
            wps.append(Waypoint(STROKE_S / STROKE_WP, knob, RAT_HOLD, f"stroke {k + 1}", rot=side_grip(math.pi / 2, STROKE * i / STROKE_WP)))
        if k == STROKES - 1:
            break
        end = side_grip(math.pi / 2, STROKE)
        wps += [
            Waypoint(0.4, knob, OPEN, "let go", rot=end),
            Waypoint(0.5, knob, OPEN, "roll the wrist back", rot=RIGHT_R0),
            Waypoint(0.4, knob, RAT_HOLD, "grip again", rot=RIGHT_R0),
        ]
    end = side_grip(math.pi / 2, STROKE)
    wps += [Waypoint(0.4, knob, OPEN, "let go", rot=end), Waypoint(1.0, pre, OPEN, "back off", rot=RIGHT_R0)]
    out_t = sum(w.duration for w in wps)
    wps.append(Waypoint(3.0, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]))
    return wps, out_t


ROCK_DEG = 1.5  # demo rock amplitude; the worn 21.50 mm rotor leaves 1.25 mm frame clearance, so 1.5 deg sweeps the frame through square without jamming (the real technique is ~2.5 deg)
ROCK_PERIODS = (4.0, 5.3)  # s: the two rocking motions have different periods, so the frame sweeps through every attitude in a few seconds
ROCK_STROKE = 22  # rocking begins in this stroke of the right hand's turning (the spindle is then within about 1 mm of the rotor)
ROCK_STEP = 0.25  # s a waypoint
ROCK_RAMP = 3.0  # s: the rocking grows and dies away over this long


def rock_waypoints(press, t_from: float, t_to: float) -> list:
    """The left hand rocks the frame (about the anvil face's centre, so the faces stay on the rotor's friction band) while the right hand keeps
    turning the ratchet; the rocking dies away to the squared attitude at ``t_to``. Mitutoyo's micrometer guide: rock the micrometer
    slightly across the work to find the minimum reading."""
    centre = press - np.array([TIPS_X - PAD_TIP_PAST_TCP, PINCH_Y, 0.0]) - L_AIM - L_SAG
    out = []
    n = max(int(round((t_to - t_from) / ROCK_STEP)), 1)
    for i in range(1, n + 1):
        u = i * ROCK_STEP
        env = min(1.0, u / ROCK_RAMP, (n * ROCK_STEP - u) / ROCK_RAMP)
        pitch = math.radians(ROCK_DEG) * env * math.sin(2 * math.pi * u / ROCK_PERIODS[0])
        yaw = math.radians(ROCK_DEG) * env * math.sin(2 * math.pi * u / ROCK_PERIODS[1] + 1.0)
        rot_d = axis_angle_matrix(_Z, yaw) @ axis_angle_matrix(_X, pitch)
        out.append(Waypoint(ROCK_STEP, centre + rot_d @ (press - centre), FRAME_HOLD, "rock the frame across the rotor", rot=rot_d @ LEFT_ROT))
    return out


def left_plan(wait_until: float, rock_from: float | None = None):
    g0 = _left_tcp(P_START, dx=-0.05)
    g1 = _left_tcp(P_START)
    up = _left_tcp(P_START, dz=LIFT) + L_SAG
    side = _left_tcp(P_APPR, dz=LIFT) + L_SAG
    meas = _left_tcp(P_MEAS, dz=LIFT) + L_SAG
    press = meas + np.array([0.0, -PRESS_DY, 0.0])
    wps = [
        Waypoint(3.0, g0, OPEN, "ready behind the micrometer", rot=LEFT_ROT),
        Waypoint(1.5, g1, OPEN, "pads over and under the spine", rot=LEFT_ROT),
        Waypoint(0.8, g1, FRAME_HOLD, "grip the frame", rot=LEFT_ROT),
        Waypoint(1.0, up, FRAME_HOLD, "lift it off the rest", rot=LEFT_ROT),
        Waypoint(3.0, side, FRAME_HOLD, "carry it beside the rotor", rot=LEFT_ROT),
        Waypoint(1.0, side, FRAME_HOLD, "steady", rot=LEFT_ROT),
        Waypoint(5.0, meas, FRAME_HOLD, "slide it over the rotor's edge", rot=LEFT_ROT),
        Waypoint(1.0, press, FRAME_HOLD, "press the anvil against the rotor", rot=LEFT_ROT),
    ]
    held = sum(w.duration for w in wps)
    if rock_from is None:
        wps.append(Waypoint(max(wait_until - held, 0.1), press, FRAME_HOLD, "hold it square", rot=LEFT_ROT))
    else:
        wps.append(Waypoint(max(rock_from - held, 0.1), press, FRAME_HOLD, "hold it square", rot=LEFT_ROT))
        wps += rock_waypoints(press, max(rock_from, held), wait_until)
    wps += [
        Waypoint(1.0, meas, FRAME_HOLD, "ease off the rotor", rot=LEFT_ROT),
        Waypoint(4.0, side, FRAME_HOLD, "slide it off the rotor", rot=LEFT_ROT),
        Waypoint(3.0, up, FRAME_HOLD, "carry it back", rot=LEFT_ROT),
        Waypoint(1.0, g1 + np.array([0.0, 0.0, -0.0005]), FRAME_HOLD, "set it on the rest", rot=LEFT_ROT),
        Waypoint(0.8, g1 + np.array([0.0, 0.0, -0.0005]), OPEN, "let go", rot=LEFT_ROT),
        Waypoint(1.5, g0, OPEN, "back off", rot=LEFT_ROT),
        Waypoint(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps


def demo():
    """The left arm slides the micrometer onto the rotor and holds it; the right arm turns the ratchet until it clicks while
    the left arm rocks the frame; the left arm takes the micrometer off and sets it on its rest."""
    left_head = 3.0 + 1.5 + 0.8 + 1.0 + 3.0 + 1.0 + 5.0 + 1.0
    right, out_t = right_plan(start_t=left_head + 1.0)
    stroke_s = STROKE_S + 0.4 + 0.5 + 0.4
    rock_from = left_head + 1.0 + 1.5 + 0.6 + 5.0 * 0 + ROCK_STROKE * stroke_s
    return {"left": left_plan(wait_until=left_head + 1.0 + out_t - 1.0, rock_from=rock_from), "right": right}
