"""On the station table: a single-spool filament dryer (262 x 130 x 240 mm, its smoked clear hood hinged at the back
and resting fully open on its stop), and beside it, on the robot's right, a 1 kg spool of orange PLA standing on its
rims in a printed spool stand, axis level. The right arm lifts the spool off the stand, carries it over the dryer and
sets it down on the dryer's two rollers, lets go and goes back; then the left arm swings the hood over and presses it
shut until its catch clicks, and goes back.

The dryer is one articulation from ``assets/`` (the base, the hood on a friction hinge, two free-spinning rollers, the
catch's sprung slider that holds the hood shut by contact). The spool and the stand are their own rigid bodies.

Run it:  inspect-robots run --embodiment yam_arena -E task=filament_dryer.py --scorer task_score ...

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
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP, SHOULDER, WRIST
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
ASSETS = Path(__file__).resolve().parent / "assets"
TAG = "filament_dryer"
TRACE_S = 1.0  # s between state lines in a run's log

# -- the dryer (its own frame: origin on the table under the middle of its base, x along it, y across it, z up) -------
DRYER_HALF = np.array([0.131, 0.065])  # 262 x 130 mm
BASE_TOP = 0.160  # the base's rim, where the hood sits
HINGE = np.array([0.0, 0.065, 0.160])  # the hood's hinge axis (along x) on the base's back top edge
HOOD_TOP = 0.240  # the hood's top face, shut
LID_OPEN = math.radians(-100)  # the hinge's open stop (negative opens); the hood starts resting on it
ROLLER_X, ROLLER_Z, ROLLER_R = 0.065, 0.030, 0.011
# -- the spool (origin at its centre, axis along its y) and the stand ------------------------------------------------
SPOOL_R, SPOOL_HALF_WIDTH = 0.100, 0.0335  # 200 mm across, 67 mm wide (flanges 3 mm)
SPOOL_MASS = 1.18
SEAT_Z = ROLLER_Z + math.sqrt((SPOOL_R + ROLLER_R) ** 2 - ROLLER_X ** 2)  # the spool's centre on the rollers: 120 mm
STAND_RAIL_X, STAND_RAIL_Z, STAND_RAIL_R = 0.055, 0.020, 0.008
ON_STAND_Z = STAND_RAIL_Z + math.sqrt((SPOOL_R + STAND_RAIL_R) ** 2 - STAND_RAIL_X ** 2)  # 113 mm
STAND_TOP = STAND_RAIL_Z + STAND_RAIL_R  # the rails' tops: 28 mm

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
DRYER_XY = (0.30, 0.0)  # long side along x, hinge on the robot's left (+y), control panel toward the robot
STAND_XY = (0.30, -0.20)  # on the right arm's side, the spool's axis along y like the dryer's
DRYER_POS = np.array([*DRYER_XY, TOP + 0.0002])
STAND_POS = np.array([*STAND_XY, TOP + 0.0002])
SPOOL_START = STAND_POS + (0.0, 0.0, ON_STAND_Z + 0.0005)  # the 32-sided flange colliders settle 0.5 mm
SEAT = DRYER_POS + (0.0, 0.0, SEAT_Z)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


# Arena places an articulation by its root link: the dryer's is its base, whose origin is the asset's origin.
DRYER = UsdPart("dryer", asset="dryer", xy=DRYER_XY, extent=0.27, mass=1.2, usd_path=str(ASSETS / "filament_dryer.usda"),
                pos=tuple(float(v) for v in DRYER_POS), articulation=True)
STAND = UsdPart("stand", asset="stand", xy=STAND_XY, extent=0.15, mass=0.08, usd_path=str(ASSETS / "spool_stand.usda"),
                pos=tuple(float(v) for v in STAND_POS))
SPOOL = UsdPart("spool", asset="spool", xy=STAND_XY, extent=0.2, mass=SPOOL_MASS,
                usd_path=str(ASSETS / "filament_spool_1kg.usda"), pos=tuple(float(v) for v in SPOOL_START))
OBJECTS = [DRYER, STAND, SPOOL]

SOLVER_ITERATIONS = 32  # position iterations per step for the dryer's articulation (light links: rollers, catch)
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD; a kinematic one fixed in place."""
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
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1)
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


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root (the dryer's is its base), plus the grippers, the episode time, the hood's hinge angle and
        the catch slider's travel."""
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
        dryer = base.scene["dryer"]
        ids = dryer.find_joints(["LidHinge", "LatchSlide"], preserve_order=True)[0]
        lid, latch = dryer.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
        out["lid_angle"], out["latch_travel"] = float(lid), float(latch)  # rad (negative opens), m
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that opens the hood onto its stop and sets the catch's
        spring: Isaac Lab starts an articulation's joints at 0 (hood shut) and sets every drive's target to 0."""
        configure = stock_cfg_callback(control_hz)

        def with_lid(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.dryer_lid = EventTermCfg(func=_open_lid, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("dryer")})
            return env_cfg

        return with_lid

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


# The joints' start and drive targets: the hood open on its stop (the hinge has damping only), the catch home with its
# return spring's 1 mm preload, the rollers at 0.
START_JOINTS = {"LidHinge": LID_OPEN, "LatchSlide": 0.0, "RollerAAxle": 0.0, "RollerBAxle": 0.0}
DRIVE_TARGETS = {"LidHinge": LID_OPEN, "LatchSlide": -0.001, "RollerAAxle": 0.0, "RollerBAxle": 0.0}


def _open_lid(env, env_ids, asset_cfg) -> None:
    """Reset event: the hood open on its stop, the catch slider home, the rollers still, and the drives' targets as the
    dryer's USD has them."""
    import torch

    if env_ids is None:
        return
    dryer = env.scene[asset_cfg.name]
    ids = dryer.find_joints(list(START_JOINTS), preserve_order=True)[0]
    start = torch.tensor([list(START_JOINTS.values())], device=dryer.device).repeat(len(env_ids), 1)
    dryer.write_joint_position_to_sim_index(position=start, joint_ids=ids, env_ids=env_ids)
    dryer.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(start), joint_ids=ids, env_ids=env_ids)
    target = torch.tensor([list(DRIVE_TARGETS.values())], device=dryer.device).repeat(len(env_ids), 1)
    dryer.set_joint_position_target_index(target=target, joint_ids=ids, env_ids=env_ids)


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


# -- scoring ----------------------------------------------------------------------------------------------------------
LIFTED_CLEAR = 0.010  # m: the spool's lowest point at least this far above the stand's rails: off the stand
SEATED_XZ = (0.006, 0.004)  # m: the spool's centre within this of its seat on the rollers (along the dryer, height),
SEATED_Y = 0.012  # ... within this of the middle across the dryer (16.5 mm of play a side), and
ALIGNED = math.radians(8)  # ... its axis within this of the dryer's: it rests on both rollers
LID_SHUT = math.radians(-2)  # the hinge past this counts as shut, and
LATCH_HOME = 0.0003  # ... the catch slider back within this: the catch holds the hood
DRYER_MOVED = 0.03  # m: success needs the dryer within this of where it started, and
DRYER_TURNED = math.radians(10)  # ... turned or tilted less than this
FALLEN_Z = -0.02  # m above the table: a part whose origin is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads reach the part's bounding box (grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are at least GRIP_OPENING apart (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
SPOOL_BOX = (np.array([-SPOOL_R, -SPOOL_HALF_WIDTH, -SPOOL_R]), np.array([SPOOL_R, SPOOL_HALF_WIDTH, SPOOL_R]))
STEPS = ("spool taken off the stand", "spool on the rollers", "lid swung over", "lid shut and caught")
LID_OVER = math.radians(-60)  # step 3: a gripper has pushed the hood 40 deg toward shut (once reached); past -50 deg it
# falls shut by its own weight
POINTS = 0.25  # per step


@dataclass(frozen=True)
class PartPose:
    pos: np.ndarray
    quat_xyzw: np.ndarray


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose``."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ quat_xyzw_to_matrix(pose.quat_xyzw).T + pose.pos
    return w.min(0), w.max(0)


def _held(bounds, gripper) -> bool:
    """``gripper`` holds the part with world ``bounds``: its pads reach them and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = bounds[0] - GRIP_SLACK, bounds[1] + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def _held_by_any(objects, bounds) -> bool:
    return any(_held(bounds, objects.get(g)) for g in ("left_gripper", "right_gripper"))


def spool_held(objects) -> bool:
    return _held_by_any(objects, _box(objects["spool"], SPOOL_BOX))


def spool_bottom(objects) -> float:
    """The spool's lowest point (m, world z): its rims' lowest point, whatever its tilt."""
    s = objects["spool"]
    axis_z = abs(float(quat_xyzw_to_matrix(s.quat_xyzw)[2, 1]))
    return float(s.pos[2] - SPOOL_R * math.sqrt(max(0.0, 1.0 - axis_z ** 2)) - SPOOL_HALF_WIDTH * axis_z)


def spool_off_stand(objects) -> bool:
    """A gripper holds the spool lifted clear of the stand: its lowest point above the stand's rails."""
    return bool(spool_bottom(objects) > objects["stand"].pos[2] + STAND_TOP + LIFTED_CLEAR and spool_held(objects))


def spool_in_dryer(objects):
    """The spool's centre in the dryer's frame (m) and the angle between its axis and the dryer's (rad, 0 to pi/2)."""
    d = objects["dryer"]
    r = quat_xyzw_to_matrix(d.quat_xyzw)
    s = objects["spool"]
    local = r.T @ (s.pos - d.pos)
    axis = quat_xyzw_to_matrix(s.quat_xyzw)[:, 1]
    return local, math.acos(min(1.0, abs(float(axis @ r[:, 1]))))


def spool_seated(objects) -> bool:
    """The spool rests on both rollers, centred and aligned, and no gripper holds it."""
    local, angle = spool_in_dryer(objects)
    return bool(abs(local[0]) < SEATED_XZ[0] and abs(local[2] - SEAT_Z) < SEATED_XZ[1] and abs(local[1]) < SEATED_Y
                and angle < ALIGNED and not spool_held(objects))


def lid_caught(objects) -> bool:
    return bool(objects["lid_angle"] > LID_SHUT and objects["latch_travel"] < LATCH_HOME)


def dryer_in_place(objects) -> bool:
    """The dryer within DRYER_MOVED of where it started, upright and not turned more than DRYER_TURNED."""
    d = objects["dryer"]
    r = quat_xyzw_to_matrix(d.quat_xyzw)
    turned = math.acos(min(1.0, float(r[0, 0])))  # its long axis against the start's (x)
    tilted = math.acos(min(1.0, float(r[2, 2])))
    return bool(np.linalg.norm(d.pos[:2] - DRYER_POS[:2]) < DRYER_MOVED and turned < DRYER_TURNED
                and tilted < DRYER_TURNED)


def _lid_pushed(objects) -> bool:
    """A gripper's pads are at the hood's outer top face (within 2 cm of it), over the hood: it is pushing it."""
    d = objects["dryer"]
    r = quat_xyzw_to_matrix(d.quat_xyzw)
    a = objects["lid_angle"]
    hinge = d.pos + r @ HINGE
    lid_r = r @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), a)  # the hood's frame: x along the hinge
    for g in ("left_gripper", "right_gripper"):
        gs = objects.get(g)
        if gs is None:
            continue
        for f in np.linspace(0.0, 1.0, 5):
            p = lid_r.T @ (gs.back + f * (gs.tips - gs.back) - hinge)  # in the shut hood's frame, from the hinge
            if abs(p[0]) < DRYER_HALF[0] + 0.01 and -2 * DRYER_HALF[1] - 0.01 < p[1] < 0.01 \
                    and HOOD_TOP - HINGE[2] - 0.01 < p[2] < HOOD_TOP - HINGE[2] + 0.02:
                return True
    return False


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether the spool has been lifted off the stand, and whether a gripper
    has swung the hood past halfway shut. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.next_trace = 0.0
        self.taken_off = False
        self.swung = False
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.taken_off |= spool_off_stand(objects)
        self.swung |= bool(objects["lid_angle"] > LID_OVER and _lid_pushed(objects))


_PROGRESS = _Progress()


def _trace(objects) -> str:
    local, angle = spool_in_dryer(objects)
    d, s = objects["dryer"], objects["spool"]
    stand = objects["stand"].pos - STAND_POS
    return (f"spool {np.round((local - (0, 0, SEAT_Z)) * 1000, 1).tolist()} mm from its seat, axis "
            f"{math.degrees(angle):.1f} deg off, {np.linalg.norm(s.lin_vel) * 1000:.0f} mm/s, bottom "
            f"{(spool_bottom(objects) - TOP) * 1000:.0f} mm up; lid {math.degrees(objects['lid_angle']):.1f} deg, catch "
            f"{objects['latch_travel'] * 1000:.2f} mm; dryer moved {np.linalg.norm(d.pos[:2] - DRYER_POS[:2]) * 1000:.1f}"
            f" mm; stand moved {np.round(stand * 1000, 1).tolist()} mm")


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: 1 and 3 once reached; 2 and 4 as things stand. Steps 3 and 4 count only with the spool on the
    rollers."""
    p = _PROGRESS
    p.update(objects)
    seated = spool_seated(objects)
    done = [bool(p.taken_off), seated, bool(seated and p.swung), bool(seated and lid_caught(objects))]
    if "episode_time" in objects:  # in a run, log each change (the run's step timeline) and the state every 2 s
        t = objects["episode_time"]
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  ({_trace(objects)})", flush=True)
        elif t >= p.next_trace:
            print(f"[{TAG}] t={t:.2f}s  {_trace(objects)}", flush=True)
        if t >= p.next_trace:
            p.next_trace = t + TRACE_S
    return done


def _violation(objects) -> bool:
    """Scores 0: the dryer or the spool off the table, or a state that is not finite."""
    for name in ("dryer", "spool"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return not math.isfinite(objects["lid_angle"])


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the dryer where it was and the
    dryer and the spool still, and no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and dryer_in_place(objects) and objects["dryer"].resting()
                and objects["spool"].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)




# -- scripted demo ----------------------------------------------------------------------------------------------------
# Right arm: pointing forward and 38 deg down, fingers closing across the spool's width on the outsides of its two
# flanges at its top (the contact patch over the spool's axis, so it hangs straight; the gripper's housing, 54 mm and
# more behind the TCP and as wide as the spool, then clears the spool's rim). It takes the spool's weight, lifts it
# off the stand, carries it over the dryer's front wall and lowers it onto the rollers; it hands the weight to the
# rollers, opens a little, lifts out and goes home.
# Left arm: pointing forward, level, fingers closed, at the hood's near end; the side of the fingers follows the open
# hood's top face near its free edge, pushing it over (the gripper rolls with the face); the hood falls shut past
# about 50 deg open (friction hinge); the fingers follow it down and press it so the catch snaps over the hook, lift away
# and go home.
SQUEEZE = 0.014  # grips close the fingers this far inside the spool's width: about 35 N a finger
SPOOL_HOLD = -(2 * SPOOL_HALF_WIDTH - SQUEEZE) / 2
GRIP_TILT = math.radians(38)  # the right gripper points this far below level
PAD_CENTRE = np.array([-0.0035, 0.0, 0.094])  # the pads' middle from the spool's centre: its patch is over the axis
PAD_MID_BEHIND_TCP = 0.006  # the pads' middle is this far behind the TCP (they span 25 mm behind to 13 mm past it)
RIGHT_ROLL = -1  # +1 or -1: which way round the right gripper is about its pointing axis (its wrist's range)
OVER_RIM = 0.015  # m: the spool's bottom clears the dryer's front wall by this, carried over
RELEASE_OPEN = -0.038  # finger value when letting go (76 mm apart): clear of the flanges, short of the open hood
# The arms hold position through soft joint drives (80 N m/rad at the shoulder, 10 at the wrist) with gravity
# compensation for their own links only: the 1.18 kg spool would pull the right wrist down about 15 deg. While the
# spool is in the hand, the right arm's joint targets are raised by the sag its weight causes (each joint's torque
# from the spool's weight over the drive's stiffness), faded in as the arm takes the weight and out once the rollers
# have it.
# The hood: the closed fingers' outer face (FINGER_HALF off the gripper's axis) follows the hood's outer top face
# LID_PUSH_U from the hinge (near its free edge, 130 mm), LID_PUSH_GAP off it; the gripper's middle is LID_PUSH_X in
# from the hood's near end so the gripper's housing stays off it (right over the catch). At the end it presses
# LID_PRESS into the shut hood (the soft wrist turns that into about 2 N).
FINGER_HALF = 0.004
LID_PUSH_U = 0.110
LID_PUSH_GAP = 0.0015
LID_PUSH_X = 0.030
LID_PRESS = 0.008
LEFT_ROLL = -1


@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None  # a full gripper (TCP) rotation; replaces ``yaw`` when set
    load: float = 0.0  # how much of the spool's weight the arm carries at the end of this waypoint (0 to 1)


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


def _load(plan, t: float) -> float:
    """The share of the spool's weight the arm carries at time ``t`` (interpolated like the waypoints)."""
    prev = 0.0
    for w in plan.waypoints:
        load = getattr(w, "load", 0.0)
        if t < w.duration:
            return prev + ik.smoothstep(t / w.duration) * (load - prev)
        t -= w.duration
        prev = load
    return prev


K_JOINTS = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)  # N m/rad


def payload_sag(q, base_pos) -> np.ndarray:
    """How far (rad) each arm joint gives under the spool's weight held in the right grip at joints ``q``: the
    torque of its weight at its centre (fixed in the gripper) over the joint's drive stiffness."""
    def centre(qq):
        p, r = ik.tcp_pose(qq, base_pos)
        return p + r @ SPOOL_IN_HAND

    c0, jac = centre(q), np.zeros((3, 6))
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += 1e-6
        jac[:, i] = (centre(dq) - c0) / 1e-6
    torque = jac.T @ np.array([0.0, 0.0, -SPOOL_MASS * 9.81])
    return torque / K_JOINTS


_stock_action7 = yam_demo.ArmPlan.action7


def _action7(self, t: float):
    """``ArmPlan.action7`` with the joint targets raised by the spool's sag while the arm carries it."""
    out, label = _stock_action7(self, t)
    f = _load(self, t)
    if f > 0.0:
        out = np.array(out, float)
        out[:6] -= f * payload_sag(self.q, self.base_pos)
    return out, label


yam_demo.ArmPlan.target = _target
yam_demo.ArmPlan.action7 = _action7

_X = np.array([1.0, 0.0, 0.0])
_Y = np.array([0.0, 1.0, 0.0])
_Z = np.array([0.0, 0.0, 1.0])


def _grip(point, close) -> np.ndarray:
    """The gripper (TCP) rotation pointing along ``point`` with its fingers closing along ``close`` (made square to
    it): columns x = closing, y = pad width, z = pointing."""
    p = np.asarray(point, float) / np.linalg.norm(point)
    c = np.asarray(close, float) - (np.asarray(close, float) @ p) * p
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(p, c), p])


RIGHT_POINT = np.array([math.cos(GRIP_TILT), 0.0, -math.sin(GRIP_TILT)])
RIGHT_ROT = _grip(RIGHT_POINT, RIGHT_ROLL * _Y)
SPOOL_IN_HAND = RIGHT_ROT.T @ -(PAD_CENTRE + PAD_MID_BEHIND_TCP * RIGHT_POINT)  # the spool's centre, gripper frame


def _spool_tcp(centre) -> np.ndarray:
    """The right TCP holding the spool whose centre is at ``centre`` (its axis along y)."""
    return np.asarray(centre, float) + PAD_CENTRE + PAD_MID_BEHIND_TCP * RIGHT_POINT


def _lid_point(angle: float, push: float = 0.0):
    """The world point on the hood's outer top face LID_PUSH_U from the hinge, at the near end of the push, the hood at
    ``angle`` (rad, 0 shut), and the face's outward normal; ``push`` moves the point in along it."""
    c, s = math.cos(angle), math.sin(angle)
    y, z = -LID_PUSH_U, HOOD_TOP - HINGE[2] - push
    p = np.array([0.0, y * c - z * s, y * s + z * c])
    n = np.array([0.0, -s, c])
    near_end = -DRYER_HALF[0] + LID_PUSH_X
    return DRYER_POS + HINGE + np.array([near_end, 0.0, 0.0]) + p, n


def _lid_rot(angle: float) -> np.ndarray:
    _, n = _lid_point(angle)
    return _grip(_X, LEFT_ROLL * n)


def _lid_tcp(angle: float, gap: float = LID_PUSH_GAP, press: float = 0.0) -> np.ndarray:
    p, n = _lid_point(angle, press)
    return p + (FINGER_HALF + gap) * n


def right_plan() -> list[Waypoint]:
    """Lift the spool off the stand, set it on the rollers, let go, go home."""
    grip = _spool_tcp(SPOOL_START)
    carry_z = DRYER_POS[2] + BASE_TOP + OVER_RIM + SPOOL_R  # the spool's centre, carried over the front wall
    lifted = _spool_tcp((*SPOOL_START[:2], carry_z))
    over = _spool_tcp((*SEAT[:2], carry_z))
    seated = _spool_tcp(SEAT)
    r = RIGHT_ROT
    return [
        Waypoint(2.5, grip + (0, 0, 0.06), FINGER_OPEN, "over the spool", rot=r),
        Waypoint(1.5, grip, FINGER_OPEN, "down around its top", rot=r),
        Waypoint(0.8, grip, SPOOL_HOLD, "grip the flanges", rot=r),
        Waypoint(1.0, grip, SPOOL_HOLD, "take its weight", rot=r, load=1.0),
        Waypoint(2.0, lifted, SPOOL_HOLD, "lift it off the stand", rot=r, load=1.0),
        Waypoint(2.5, over, SPOOL_HOLD, "over the dryer", rot=r, load=1.0),
        Waypoint(2.5, seated, SPOOL_HOLD, "down onto the rollers", rot=r, load=1.0),
        Waypoint(1.0, seated, SPOOL_HOLD, "let the rollers take it", rot=r, load=0.0),
        Waypoint(0.8, seated, RELEASE_OPEN, "let go", rot=r),
        Waypoint(1.2, seated + (0, 0, 0.08), RELEASE_OPEN, "up out of the dryer", rot=r),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


LEFT_WAIT_S = 2.5 + 1.5 + 0.8 + 1.0 + 2.0 + 2.5 + 2.5 + 1.0 + 0.8 + 1.2 + 1.0  # the right arm is leaving the dryer
LID_SWEEP = (-100, -85, -70, -55, -40, -25, -10, 0)  # deg


def left_plan() -> list[Waypoint]:
    """Swing the hood over and press it shut until the catch clicks, go home."""
    start = math.radians(LID_SWEEP[0])
    wps = [
        Waypoint(LEFT_WAIT_S, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
        Waypoint(2.5, _lid_tcp(start, gap=0.03) + (0, 0, 0.03), 0.0, "beside the open hood", rot=_lid_rot(start)),
        Waypoint(1.0, _lid_tcp(start), 0.0, "against the hood", rot=_lid_rot(start)),
    ]
    wps += [Waypoint(0.9, _lid_tcp(math.radians(a)), 0.0, "swing the hood over", rot=_lid_rot(math.radians(a)))
            for a in LID_SWEEP[1:]]
    pressed = _lid_tcp(0.0, gap=0.0, press=LID_PRESS)
    wps += [
        Waypoint(1.0, pressed, 0.0, "press it shut", rot=_lid_rot(0.0)),
        Waypoint(0.6, pressed, 0.0, "hold", rot=_lid_rot(0.0)),
        Waypoint(0.8, pressed + (0, 0, 0.05), 0.0, "lift away", rot=_lid_rot(0.0)),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps


def demo():
    """The right arm puts the spool on the rollers and goes home; then the left closes the hood and goes home."""
    return {"left": left_plan(), "right": right_plan()}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
