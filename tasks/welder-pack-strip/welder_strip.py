"""On the station table: an assembled 2x2 18650 battery pack (both halves of its holder, four cells in parallel,
positive ends up), loose on the table; a nickel strip lying in a small tray; and a mini 18650 spot welder in its holder,
probes toward the robot. The right arm lays the strip on top of the pack and goes back; the left arm takes the welder
out of its holder, tilts it probes-down and touches the strip with both copper probe tips over each of the four cells (a
weld each time), puts the welder back in its holder and goes back too.

The welder is one articulation from ``assets/`` (its housing, the latched lid over its own cell, the latch slider, the
sprung + contact and the USB-C fingers); its cell, the pack, the strip and the two trays are rigid bodies (the trays
fixed to the table). The pack is one body: real holders are a press fit, and as six bodies the cells' contacts in the
holders' cups kept a jitter that walked the 1 g strip off the + nubs. A weld is both probe tips on the strip over a cell's + terminal
for 0.2 s, with the strip on the pack and the welder in a gripper; it leaves two dark spots on the strip where the tips
touched.

Run it:  inspect-robots run --embodiment yam_arena -E task=welder_strip.py --scorer task_score ...

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
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it first holds
ASSETS = Path(__file__).resolve().parent / "assets"
TAG = "welder_strip"

# -- the welder (its own frame: origin at the rear face's bottom centre, +x to the probes, z up) ----------------------
WELDER_WIDTH, WELDER_HEIGHT = 0.024, 0.022
WELDER_LENGTH = 0.1387  # with its probes
FEET = 0.0005  # the rubber feet reach this far below the housing's bottom
BODY_SHIFT = np.array([0.004, 0.0, 0.0])  # the housing body's own origin (its articulation root) is 4 mm in
CELL_IN_WELDER = np.array([0.0421, 0.0, 0.011])  # the welder's own 18650 cell, in its bay under the latched lid
CELL_IN_WELDER_XYZW = (-0.5, 0.5, -0.5, 0.5)
# The copper probes: 2 mm rods along x at mid-height, converging to tapered tips that end in balls of TIP_RADIUS, 3.2 mm
# apart. Tilted probes-down about the welder's y axis by more than ~35 degrees, the balls are the welder's lowest points.
TIP_BALLS = (np.array([0.13844, -0.00159, 0.011]), np.array([0.13844, 0.00159, 0.011]))
TIP_RADIUS = 0.00022
WELDER_GRIP_AT = 0.062  # along the welder from its rear face: its middle, in the holder's open sides

# -- the welder's holder (task 4's orange PLA tray), fixed to the table; the welder seated in it -----------------------
SEAT_IN_HOLDER = np.array([-0.062, 0.0, 0.0035])  # the welder's origin in the holder's frame (on its feet, centred)
HOLDER_WALL_TOP = 0.014
HOLDER_FEET = 0.0005

# -- the pack: two 2x2 holder halves (41.2 x 41.2 x 9.5 mm, 20 mm pitch) and four cells, + up, as one rigid body; its
# origin at its bottom face's centre ------------------------------------------------------------------------------
CELL_PITCH = 0.020
CELL_RADIUS = 0.009
CELL_HALF = 0.03325  # a cell's centre to its + nub's top
CELL_SEATED_Z = 0.03475  # a cell's centre above the pack's bottom, seated on the bottom holder's ledge
NUB_TOP_Z = 0.068  # the + nubs' tops above the pack's bottom
NUB_RADIUS = 0.0032
# The top holder (the bottom one flipped over) rests on the cells' shoulders: its outer face 67.8 mm up, the + nubs
# 0.2 mm proud of it (the stack's axial play).
PACK_TOP_Z = 0.0678
PACK_HALF = 0.0206
CELLS_IN_PACK = ((-1, -1), (-1, 1), (1, -1), (1, 1))  # cells 1 to 4, in half pitches (x toward the far side)
CELL_AXES = tuple(np.array([i * CELL_PITCH / 2, j * CELL_PITCH / 2]) for i, j in CELLS_IN_PACK)  # in the pack's frame

# -- the nickel strip (an H, 28 x 28 mm, its bars along its own y; origin at the plate's underside) and its tray ------
STRIP_TOP = 0.0003  # its colliders' top face above its origin (the plate is 0.15 mm, its colliders 0.3 mm)
STRIP_HALF = 0.014
BAR_INNER = 0.006  # the bars run from 6 to 14 mm either side of the strip's centre (its x)
BRIDGE_HALF = 0.004  # the bridge between them is 8 mm wide (its y)
TAB_TOP = 0.010  # the folded grip tab: 8 mm along the strip's x, 3 mm across (its y), 10 mm tall
TAB_ACROSS = 0.003
STRIP_YAW = math.pi / 2  # the strip's bars along x (front to back), over the cell pairs; its tab 3 mm across in x
TRAY_POCKET = 0.007  # the tray's pocket floor above the table
TRAY_RIM = 0.008

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
HOLDER_XY, HOLDER_YAW = (0.30, 0.26), math.pi  # the welder's holder on the left, probes toward the robot (task 4)
PACK_XY = (0.31, 0.03)  # the pack, in front of the robot
TRAY_XY = (0.27, -0.19)  # the strip's tray, on the right


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _ry(angle: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), angle)


def _qmul(a, b) -> tuple[float, float, float, float]:
    """Quaternion product a * b (xyzw)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


HOLDER_POS = (*HOLDER_XY, TOP + HOLDER_FEET)
WELDER_YAW = HOLDER_YAW
WELDER_POS = tuple(float(v) for v in np.array(HOLDER_POS) + _rz(HOLDER_YAW) @ SEAT_IN_HOLDER)  # its origin, seated
PACK_POS = (*PACK_XY, TOP + 0.0002)
CELL_POS = tuple(np.array([PACK_XY[0] + i * CELL_PITCH / 2, PACK_XY[1] + j * CELL_PITCH / 2,
                           PACK_POS[2] + CELL_SEATED_Z]) for i, j in CELLS_IN_PACK)
NUB_TOP = PACK_POS[2] + NUB_TOP_Z  # where the strip's underside lies on the pack (world z)
TRAY_POS = (*TRAY_XY, TOP)
STRIP_POS = (*TRAY_XY, TOP + TRAY_POCKET + 0.0001)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False  # a body that never moves (a tray fixed to the table)

    @property
    def free(self) -> bool:
        return not self.kinematic


# Arena places an articulation by its root link: the welder's is its housing body, 4 mm in from the rear face.
WELDER = UsdPart("welder", asset="welder", xy=HOLDER_XY, extent=0.14, mass=0.0797,
                 usd_path=str(ASSETS / "welder_task.usda"),
                 pos=tuple(float(v) for v in np.array(WELDER_POS) + _rz(WELDER_YAW) @ BODY_SHIFT + (0.0, 0.0, 0.0001)),
                 rot_xyzw=_yaw_xyzw(WELDER_YAW), articulation=True)
WELDER_CELL = UsdPart("welder_cell", asset="cell", xy=HOLDER_XY, extent=0.065, mass=0.047,
                      usd_path=str(ASSETS / "battery_18650.usda"),
                      pos=tuple(float(v) for v in np.array(WELDER_POS) + _rz(WELDER_YAW) @ CELL_IN_WELDER
                                + (0.0, 0.0, 0.0001)),
                      rot_xyzw=tuple(float(v) for v in _qmul(_yaw_xyzw(WELDER_YAW), CELL_IN_WELDER_XYZW)))
WELDER_HOLDER = UsdPart("welder_holder", asset="holder", xy=HOLDER_XY, extent=0.15, mass=0.0207,
                        usd_path=str(ASSETS / "spot_welder_holder.usda"), pos=HOLDER_POS,
                        rot_xyzw=_yaw_xyzw(HOLDER_YAW), kinematic=True)
PACK = UsdPart("pack", asset="pack", xy=PACK_XY, extent=0.068, mass=0.2009,
               usd_path=str(ASSETS / "battery_pack_2x2_rigid.usda"), pos=PACK_POS)
STRIP = UsdPart("strip", asset="strip", xy=TRAY_XY, extent=0.028, mass=0.00095,
                usd_path=str(ASSETS / "nickel_strip_2x2.usda"), pos=STRIP_POS, rot_xyzw=_yaw_xyzw(STRIP_YAW))
TRAY = UsdPart("strip_tray", asset="tray", xy=TRAY_XY, extent=0.04, mass=0.0044,
               usd_path=str(ASSETS / "strip_tray.usda"), pos=TRAY_POS, kinematic=True)
OBJECTS = [WELDER, WELDER_CELL, WELDER_HOLDER, PACK, STRIP, TRAY]

SOLVER_ITERATIONS = 32  # position iterations per step for the welder's articulation (light links: lid, latch, fingers)
MAX_DEPENETRATION = 0.2  # m/s: tight contacts with 20 N grippers stay stable


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
class PartPose:
    """One link of an articulation after a control step (world frame, m)."""

    pos: np.ndarray
    quat_xyzw: np.ndarray


def _link_pose(base, origin, asset: str, link: str) -> PartPose:
    art = base.scene[asset]
    i = art.find_bodies([link])[0][0]
    pose = art.data.body_link_pose_w.torch[0, i].detach().cpu().numpy().astype(np.float64)
    return PartPose(pos=pose[:3] - origin, quat_xyzw=pose[3:])


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
        """Each object's root (Arena's geometry queries take rigid objects only; an articulation's bounds are left at
        its root), plus the grippers, the episode time and the welder's housing."""
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
        out["housing"] = _link_pose(base, origin, "welder", "Body")
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that shuts the welder's lid and puts its latch home."""
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.welder_springs = EventTermCfg(func=_wind_springs, mode="reset",
                                                         params={"asset_cfg": SceneEntityCfg("welder")})
            return env_cfg

        return with_events

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


# The lid's hinge and the latch's slide. Their springs keep the targets written in the welder's USD (Arena gives task
# objects no actuators, so the drives are PhysX's own): the lid's torsion spring winds toward fully open (the latch
# holds it shut), the latch's return spring presses the slider back with a small preload.
SPRING_JOINTS = ("LidHinge", "LatchSlide")


def _wind_springs(env, env_ids, asset_cfg) -> None:
    """Reset event: the lid shut and the latch home (the simulator keeps an articulation's joints where they were)."""
    import torch

    if env_ids is None:
        return
    welder = env.scene[asset_cfg.name]
    ids = welder.find_joints(list(SPRING_JOINTS), preserve_order=True)[0]
    zero = torch.zeros((len(env_ids), len(ids)), device=welder.device)
    welder.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    welder.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)


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


# -- geometry of the parts in a state ---------------------------------------------------------------------------------
GRIP_SLACK = 0.005  # held: a gripper's pads span the part's box grown by this, both fingers press with at least
GRIP_FORCE = 0.5  # ... this (N), and the pads are at least GRIP_OPENING apart (not closed on nothing)
GRIP_OPENING = 0.002
WELDER_BOX = (np.array([0.0, -WELDER_WIDTH / 2, 0.0]), np.array([WELDER_LENGTH, WELDER_WIDTH / 2, WELDER_HEIGHT]))
STRIP_BOX = (np.array([-STRIP_HALF, -STRIP_HALF, 0.0]), np.array([STRIP_HALF, STRIP_HALF, TAB_TOP]))


def welder_origin(objects) -> np.ndarray:
    h = objects["housing"]
    return h.pos - quat_xyzw_to_matrix(h.quat_xyzw) @ BODY_SHIFT


def _welder_pose(objects) -> PartPose:
    return PartPose(pos=welder_origin(objects), quat_xyzw=objects["housing"].quat_xyzw)


def probe_tips(objects) -> list[np.ndarray]:
    """Where each probe tip meets a flat surface under it: the lowest point of its end ball (world)."""
    r = quat_xyzw_to_matrix(objects["housing"].quat_xyzw)
    o = welder_origin(objects)
    return [o + r @ c - (0.0, 0.0, TIP_RADIUS) for c in TIP_BALLS]


def _in_strip(objects, p) -> np.ndarray:
    s = objects["strip"]
    return quat_xyzw_to_matrix(s.quat_xyzw).T @ (np.asarray(p) - s.pos)


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


def welder_held(objects) -> bool:
    return _held_by_any(objects, _box(_welder_pose(objects), WELDER_BOX))


def strip_held(objects) -> bool:
    return _held_by_any(objects, _box(objects["strip"], STRIP_BOX))


# -- checks -----------------------------------------------------------------------------------------------------------
STRIP_OFF = 0.003  # m: the strip's centre within this of the pack's centre (sideways), and
STRIP_TURN = math.radians(10)  # ... its bars along a cell pair within this (any quarter turn), and flat within this,
STRIP_DOWN = 0.001  # ... and its underside within this of resting on the cells' + nubs
LIFTED = 0.005  # m: the strip counts as taken out of its tray once a gripper holds it this far above the tray's rim
WELDER_OUT = 0.02  # m: the welder counts as taken out of its holder once a gripper holds it this far above its seat
WELD_GAP = 0.0005  # m: both probe tips this close to the strip's top face (or pressed into it), and
WELD_DEEP = 0.001  # ... not deeper into it than this (through it: not a weld), and
WELD_REACH = 0.0035  # ... this close to a cell's axis (sideways: on its + terminal): a weld on that cell, once held
WELD_HOLD_S = 0.2  # s
SEATED_OFF = 0.005  # m: the welder's origin within this of its seat in the holder (sideways and along it), and
SEATED_UP = 0.002  # ... within this of the pocket floor's height, and
UPRIGHT = math.radians(10)  # ... upright within this
PACK_TILT = math.radians(20)  # the pack tilted further: knocked over
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home


def _tilt(quat) -> float:
    """How far a part's z axis is from vertical (rad)."""
    return math.acos(max(-1.0, min(1.0, float(quat_xyzw_to_matrix(quat)[2, 2]))))


def _yaw_off(a, b) -> float:
    """How far apart two parts' headings are about the vertical, modulo a quarter turn (rad)."""
    ra, rb = quat_xyzw_to_matrix(a.quat_xyzw), quat_xyzw_to_matrix(b.quat_xyzw)
    turn = math.atan2(ra[1, 0], ra[0, 0]) - math.atan2(rb[1, 0], rb[0, 0])
    return abs((turn + math.pi / 4) % (math.pi / 2) - math.pi / 4)


def _in_pack(objects, p) -> np.ndarray:
    pack = objects["pack"]
    return quat_xyzw_to_matrix(pack.quat_xyzw).T @ (np.asarray(p) - pack.pos)


def cell_terminal(objects, cell: int) -> np.ndarray:
    """The top of a cell's + nub (world)."""
    pack = objects["pack"]
    return pack.pos + quat_xyzw_to_matrix(pack.quat_xyzw) @ np.array([*CELL_AXES[cell - 1], NUB_TOP_Z])


def strip_on_pack(objects) -> bool:
    """The strip lies flat on the pack's + nubs, centred on the pack, its bars over the cell pairs."""
    s, pack = objects["strip"], objects["pack"]
    d = _in_pack(objects, s.pos)
    up = float(quat_xyzw_to_matrix(s.quat_xyzw)[:, 2] @ quat_xyzw_to_matrix(pack.quat_xyzw)[:, 2])
    return bool(np.hypot(d[0], d[1]) < STRIP_OFF and abs(d[2] - NUB_TOP_Z) < STRIP_DOWN
                and _yaw_off(s, pack) < STRIP_TURN and up > math.cos(STRIP_TURN))


def strip_lifted(objects) -> bool:
    """A gripper holds the strip with its underside above the tray's rim."""
    return bool(objects["strip"].pos[2] > TOP + TRAY_RIM + LIFTED and strip_held(objects))


def on_metal(p_strip) -> bool:
    """A point (in the strip's frame) lies over the strip's H: on a bar or on the bridge."""
    x, y = abs(float(p_strip[0])), abs(float(p_strip[1]))
    return bool(y <= STRIP_HALF and (BAR_INNER <= x <= STRIP_HALF or (x <= BAR_INNER and y <= BRIDGE_HALF)))


def touching_cell(objects) -> int | None:
    """The cell (1-4) both probe tips touch the strip over, on its + terminal, if any."""
    tips = probe_tips(objects)
    for t in tips:
        p = _in_strip(objects, t)
        h = p[2] - STRIP_TOP
        if h > WELD_GAP or h < -WELD_DEEP or not on_metal(p):
            return None
    for cell in range(1, 5):
        axis = _in_pack(objects, cell_terminal(objects, cell))
        if all(np.hypot(*(_in_pack(objects, t)[:2] - axis[:2])) < WELD_REACH for t in tips):
            return cell
    return None


def welder_in_holder(objects):
    """The welder's origin relative to its seat in the holder (m, in the holder's frame) and whether it is seated: on the
    pocket floor, centred within SEATED_OFF and upright within UPRIGHT."""
    holder = objects["welder_holder"]
    rr = quat_xyzw_to_matrix(holder.quat_xyzw)
    d = rr.T @ (welder_origin(objects) - (holder.pos + rr @ SEAT_IN_HOLDER))
    seated = np.hypot(d[0], d[1]) < SEATED_OFF and abs(d[2]) < SEATED_UP and _tilt(objects["housing"].quat_xyzw) < UPRIGHT
    return d, bool(seated)


def welder_taken_out(objects) -> bool:
    """A gripper holds the welder lifted out of its holder (its origin WELDER_OUT above the seat, or away from it)."""
    d, _ = welder_in_holder(objects)
    return bool((d[2] > WELDER_OUT or np.hypot(d[0], d[1]) > WELDER_OUT) and welder_held(objects))


def welder_put_back(objects) -> bool:
    """The welder sits in its holder and no gripper holds it."""
    return bool(welder_in_holder(objects)[1] and not welder_held(objects))


def pack_standing(objects) -> bool:
    """The pack stands upright (not knocked over)."""
    return bool(_tilt(objects["pack"].quat_xyzw) < PACK_TILT)


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


# -- welds and their marks ---------------------------------------------------------------------------------------------
SPOT_PATH = "/World/envs/env_0/strip/Marks/Spot_{quarter}{tip}"  # the strip's hidden weld-spot discs
SPOT_LIFT = 0.000165  # m: a spot's centre above the strip's origin (on the 0.15 mm plate's top)


def _stage():
    """The running simulator's USD stage, or None (offline)."""
    try:
        import omni.usd
    except ImportError:
        return None
    return omni.usd.get_context().get_stage()


def spot_names(spots) -> list[str]:
    """The strip's spot prims for one weld's two tip contacts (in the strip's frame): the strip's quarter they are in
    (1 to 4: -x-y, -x+y, +x-y, +x+y, as the asset numbers them) and a / b in the order of the tips along the strip's x
    (its y when they lie along it)."""
    mid = (spots[0] + spots[1]) / 2
    quarter = 1 + 2 * int(mid[0] > 0) + int(mid[1] > 0)
    d = spots[1] - spots[0]
    axis = 0 if abs(d[0]) >= abs(d[1]) else 1
    first = 0 if spots[0][axis] <= spots[1][axis] else 1
    return [f"Spot_{quarter}{'a' if i == first else 'b'}" for i in range(2)]


def _show_spots(spots) -> None:
    """Moves two of the strip's hidden weld spots to where the probe tips touched and shows them (in the running
    simulator only)."""
    stage = _stage()
    if stage is None:
        return
    from pxr import Gf, UsdGeom

    for name, at in zip(spot_names(spots), spots, strict=True):
        prim = stage.GetPrimAtPath(SPOT_PATH.format(quarter=name[5], tip=name[6]))
        if not prim.IsValid():
            print(f"[{TAG}] no weld spot prim at {prim.GetPath()}", flush=True)
            continue
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        (ops[0] if ops else xf.AddTranslateOp()).Set(Gf.Vec3d(float(at[0]), float(at[1]), SPOT_LIFT))
        UsdGeom.Imageable(prim).MakeVisible()


def _hide_spots() -> None:
    stage = _stage()
    if stage is None:
        return
    from pxr import UsdGeom

    for k in range(1, 5):
        for tip in "ab":
            prim = stage.GetPrimAtPath(SPOT_PATH.format(quarter=k, tip=tip))
            if prim.IsValid():
                UsdGeom.Imageable(prim).MakeInvisible()


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper has taken the strip out
    of its tray and the welder out of its holder, the welds made (cell -> where each tip touched, in the strip's
    frame), and since when every full-success condition (``conditions_met``) has held without a break (``None`` while
    one fails: any lapse, such as a part moving, restarts the count). Starts over (and hides the weld spots) when the
    episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.strip_taken = self.welder_taken = False
        self.welds: dict[int, tuple] = {}
        self.touching, self.since = None, 0.0
        self.met_since = None
        self.reported = None

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
                _hide_spots()
            self.time = t
        self.strip_taken |= strip_lifted(objects)
        self.welder_taken |= welder_taken_out(objects)
        cell = touching_cell(objects) if (strip_on_pack(objects) and welder_held(objects)) else None
        if cell != self.touching:
            self.touching, self.since = cell, (t or 0.0)
        if cell is not None and cell not in self.welds and (t is None or t - self.since >= WELD_HOLD_S - 1e-6):
            spots = tuple(_in_strip(objects, p) for p in probe_tips(objects))
            self.welds[cell] = spots
            _show_spots(spots)
            if t is not None:
                print(f"[{TAG}] t={t:.2f}s  weld on cell {cell} (spots at {np.round(spots[0][:2] * 1000, 1)} and "
                      f"{np.round(spots[1][:2] * 1000, 1)} mm on the strip)", flush=True)
        if t is None:  # no clock: nothing can be timed
            return
        met = conditions_met(objects, self)
        if met and self.met_since is None:
            self.met_since = t
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)
        elif not met and self.met_since is not None:
            self.met_since = None
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed: timing restarts", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()

# -- scoring ----------------------------------------------------------------------------------------------------------
STEPS = ("strip on the pack", "cell 1 welded", "cell 2 welded", "cell 3 welded", "cell 4 welded",
         "welder back in its holder")
POINTS = (0.2, 0.1, 0.1, 0.1, 0.1, 0.2)  # at most 0.8; only success scores 1.0
STILL_S = 1.0  # s: success needs every full-success condition held without a break this long, among them
STILL_SPEED, STILL_SPIN = 0.01, 0.5  # ... the welder, the strip and the pack still: under 1 cm/s and 0.5 rad/s


def _steps(objects, p) -> list[bool]:
    """Each of ``STEPS`` from the progress ``p`` and the parts as they stand (see ``steps_done``)."""
    strip = bool(p.strip_taken and strip_on_pack(objects) and not strip_held(objects))
    return [strip] + [k in p.welds for k in range(1, 5)] + [bool(p.welder_taken and welder_put_back(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: the strip on the pack (judged as things stand, and only once a gripper has taken it out of its
    tray, and not while one still holds it); each cell's weld once made; the welder back in its holder (as things
    stand, once a gripper has taken it out)."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects, p)
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        off = _in_pack(objects, objects["strip"].pos) * 1000
        seat, _ = welder_in_holder(objects)
        print(f"[{TAG}] t={objects['episode_time']:.2f}s  steps done: {got}  (strip {np.round(off[:2], 1)} mm off the "
              f"pack's centre, {off[2] - NUB_TOP_Z * 1000:.1f} mm over the nubs; welder {np.round(seat * 1000, 1)} mm "
              f"off its seat)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: a part off the table or not finite, or the pack knocked over."""
    parts = [objects["housing"].pos, objects["strip"].pos, objects["welder_cell"].pos, objects["pack"].pos]
    if any(not np.all(np.isfinite(p)) or p[2] < TOP + FALLEN_Z for p in parts):
        return True
    return not pack_standing(objects)


def _trace(objects) -> None:
    """Every 0.5 s of a run: the strip against the pack, the probe tips over the strip, the pack."""
    t = objects.get("episode_time")
    if t is None or round(t * 20) % 10:
        return
    tips = [_in_strip(objects, p)[2] - STRIP_TOP for p in probe_tips(objects)]
    s, pack = objects["strip"], objects["pack"]
    xw = quat_xyzw_to_matrix(objects["housing"].quat_xyzw)[:, 0]
    mid = _in_pack(objects, sum(probe_tips(objects)) / 2)[:2] * 1000
    print(f"[{TAG}] t={t:5.1f}s welder tilted {math.degrees(math.asin(max(-1.0, min(1.0, -xw[2])))):5.1f} deg, tips' "
          f"middle at {np.round(mid, 1)} mm on the pack", flush=True)
    print(f"[{TAG}] t={t:5.1f}s strip {np.round((_in_pack(objects, s.pos) - (0, 0, NUB_TOP_Z)) * 1000, 2)} mm off its "
          f"place, v {np.linalg.norm(s.lin_vel) * 1000:5.1f} mm/s | tips {tips[0] * 1000:6.1f} {tips[1] * 1000:6.1f} mm "
          f"over the strip | pack {np.round((pack.pos - PACK_POS) * 1000, 2)} mm moved, v "
          f"{np.linalg.norm(pack.lin_vel) * 1000:4.1f} mm/s | welder v {np.linalg.norm(objects['welder'].lin_vel) * 1000:5.1f}"
          f" mm/s | welds {sorted(_PROGRESS.welds)}", flush=True)


def conditions_met(objects, p) -> bool:
    """Every full-success condition, as things stand (``p``: the episode's progress): all six steps done, both arms
    back where they started with nothing in hand, the welder, the strip and the pack still (under ``STILL_SPEED`` and
    ``STILL_SPIN``), and no violation."""
    return bool(all(_steps(objects, p)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right")
                and all(objects[n].resting(STILL_SPEED, STILL_SPIN) for n in ("welder", "strip", "pack")))


def success(objects) -> bool:
    """Every full-success condition (``conditions_met``) has held without a break for ``STILL_S``, counted from the
    first control step it held: the welder, the strip and the pack still for that whole second, both arms home, all
    six steps."""
    _trace(objects)
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for ``STILL_S``); otherwise ``POINTS`` per step done
    (``STEPS``, at most 0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(sum(p for p, ok in zip(POINTS, done, strict=True) if ok), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Right arm: straight down over the strip's tab, fingers closing across its 3 mm loop; lift the strip out of its tray,
# carry it over the pack and lower it until it is just above the + nubs, let go, go home.
# Left arm: from above, across the welder's middle in the holder's open sides (leaned LEAN toward the probes); lift it,
# tilt it TILT probes-down about its own y axis (the probes keep pointing toward the robot, the housing rises away from
# it over the far side of the pack: clear of the strip's tab, which runs across the welder) and touch both tips to the
# strip over each cell's + terminal, from just above, for a moment; carry it back level over its holder, let it drop
# into the pocket from just above the walls, and go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width
WELDER_HOLD = -(WELDER_WIDTH - 1.5 * SQUEEZE) / 2  # ~30 N a finger
TAB_HOLD = -(TAB_ACROSS - 0.002) / 2  # ~5 N a finger on the 1 g strip's tab loop
TAB_OPEN = -0.008  # the fingers 16 mm apart around the tab (inside the tray's pocket)
TAB_TIPS_UP = 0.0045  # m: the right pad tips' middle this far above the strip's underside, on the tab (their lower
# corner 2 mm lower, the gripper tilted RIGHT_TILT)
RIGHT_TILT = math.radians(15)  # the right gripper leans this far about its closing axis, pointing to the robot's left
PLACE_GAP = 0.0005  # m: the strip let go this far above the + nubs
WELDER_TIP_PAST_MID = 0.003  # the left pad tips this far past the housing's mid-height
LEAN = math.radians(10)  # the left gripper leans this far toward the probes (from straight down onto the housing's top)
LEFT_ROLL = 1  # +1 or -1: which way round the left gripper is about its pointing axis (its wrist's range)
TILT = math.radians(60)  # the welder tilted probes-down for the welds
PRESS = 0.0002  # m: the probe tips aimed this far into the strip's top face
WELD_CLEAR = 0.02  # m: the tips carried from weld to weld this far over the strip, then lowered to
WELD_NEAR = 0.003  # ... this far over it, and brought down onto it slowly
WELD_FROM = 0.03  # m: the first weld's tips brought this far over the strip before they go down
WELD_S = 1.0  # s: the tips held on the strip
# The left arm's position drives (stiffness 80 N m/rad at the shoulder, 10 at the wrist; gravity is compensated for
# the arm only) carry the 127 g welder 4-6 mm low and a few millimetres short. Pressed onto the strip with that error,
# the tilted welder rests its weight on its tips, skids and drags the 1 g strip along. So each weld is aimed past its
# cell's terminal by the error measured in the demo (world, m): the tips then meet the strip with ~0.1 N.
WELD_TRIM = {1: np.array([0.0039, -0.0010, 0.0038]), 2: np.array([0.0040, -0.0009, 0.0039]),
             3: np.array([0.0048, -0.0008, 0.0052]), 4: np.array([0.0056, -0.0006, 0.0052])}
WELD_SETTLE_S = 1.0  # s: the tips held just over the strip before they touch it (the arm catches up)
PLACE_TRIM = np.array([0.0006, 0.0005, 0.0])  # m: the right arm lays the strip this much past the pack's centre
WELD_ORDER = (2, 4, 3, 1)  # near left, far left, far right, near right
DROP = 0.0015  # m: the welder's feet this far above the holder's walls when it is let go
WELDER_LIFT = 0.05  # m: the welder lifted this far straight up out of its holder
WAIT_ASIDE = np.array([-0.03, 0.15, 0.06])  # m: the tilted welder waits this far to the left of and above its first weld


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

_X = np.array([1.0, 0.0, 0.0])
_Y = np.array([0.0, 1.0, 0.0])
_Z = np.array([0.0, 0.0, 1.0])


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


# The left gripper in the welder's frame (x = closing, y = pad width, z = pointing): pointing down onto the housing's top
# leaned LEAN toward the probes, closing across the housing; and its TCP, the pad tips WELDER_TIP_PAST_MID past the
# middle of the housing.
_POINT_IN_WELDER = np.array([math.sin(LEAN), 0.0, -math.cos(LEAN)])
_CLOSE_IN_WELDER = LEFT_ROLL * _Y
GRIP_IN_WELDER = np.column_stack([_CLOSE_IN_WELDER, np.cross(_POINT_IN_WELDER, _CLOSE_IN_WELDER), _POINT_IN_WELDER])
TCP_IN_WELDER = _tcp_with_tips(np.array([WELDER_GRIP_AT, 0.0, WELDER_HEIGHT / 2]), WELDER_TIP_PAST_MID, GRIP_IN_WELDER)

WELDER_LEVEL = _rz(WELDER_YAW)  # the welder's rotation in its holder
WELDER_TILTED = _rz(WELDER_YAW) @ _ry(TILT)  # ... and tilted probes-down for the welds


def _left_grip(origin, rot) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP and rotation holding the welder whose origin is at ``origin``, turned ``rot``."""
    return np.asarray(origin, float) + rot @ TCP_IN_WELDER, rot @ GRIP_IN_WELDER


def weld_origin(cell: int, above: float = 0.0, trim: bool = False) -> np.ndarray:
    """The welder's origin, tilted, with both tips' lowest points PRESS into the strip lying on the pack, centred on
    ``cell``'s axis (as laid out), raised ``above``; with ``trim``, aimed past it by WELD_TRIM."""
    c = CELL_POS[cell - 1]
    aim = np.array([c[0], c[1], NUB_TOP + STRIP_TOP - PRESS + above]) + (WELD_TRIM[cell] if trim else 0.0)
    mid = (TIP_BALLS[0] + TIP_BALLS[1]) / 2
    return aim + (0.0, 0.0, TIP_RADIUS) - WELDER_TILTED @ mid


# The right gripper: down, closing across the tab's loop (along the strip's y), its pads' width along the tab, leaned
# RIGHT_TILT about the closing axis (straight down, the right wrist's joint 4 is at its limit over the pack).
RIGHT_GRIP = axis_angle_matrix(_X, RIGHT_TILT) @ yam_demo.grip_rotation(STRIP_YAW)


def _strip_grip(strip_origin) -> np.ndarray:
    """The right TCP holding the strip whose origin (its plate's underside centre, turned STRIP_YAW) is at
    ``strip_origin``."""
    return _tcp_with_tips(np.asarray(strip_origin, float) + (0.0, 0.0, TAB_TIPS_UP), 0.0, RIGHT_GRIP)


PLACED = np.array([*PACK_XY, NUB_TOP + PLACE_GAP]) + PLACE_TRIM  # the strip's origin when it is let go
RIGHT_CLEAR_S = 2.5 + 1.5 + 0.8 + 1.5 + 2.5 + 1.5 + 0.6 + 0.8 + 1.0 + 1.0  # the right arm has laid the strip, is leaving


def right_plan() -> list[Waypoint]:
    """Pick the strip up by its tab, lay it on the pack, go home."""
    rot = RIGHT_GRIP
    at = _strip_grip(STRIP_POS)
    place = _strip_grip(PLACED)
    return [
        Waypoint(2.5, at + (0, 0, 0.05), TAB_OPEN, "over the strip's tab", rot=rot),
        Waypoint(1.5, at, TAB_OPEN, "down to the tab", rot=rot),
        Waypoint(0.8, at, TAB_HOLD, "grip the tab", rot=rot),
        Waypoint(1.5, at + (0, 0, 0.06), TAB_HOLD, "lift the strip", rot=rot),
        Waypoint(2.5, place + (0, 0, 0.03), TAB_HOLD, "over the pack", rot=rot),
        Waypoint(1.5, place, TAB_HOLD, "down onto the cells", rot=rot),
        Waypoint(0.6, place, TAB_HOLD, "settle", rot=rot),
        Waypoint(0.8, place, TAB_OPEN, "let go", rot=rot),
        Waypoint(1.0, place + (0, 0, 0.05), TAB_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan() -> list[Waypoint]:
    """Take the welder out of its holder, tilt it, weld the strip to each cell, put the welder back, go home."""
    at, level = _left_grip(WELDER_POS, WELDER_LEVEL)
    lifted = at + (0, 0, WELDER_LIFT)
    aside, tilted = _left_grip(weld_origin(WELD_ORDER[0]) + WAIT_ASIDE, WELDER_TILTED)
    above_first, _ = _left_grip(weld_origin(WELD_ORDER[0], above=WELD_FROM, trim=True), WELDER_TILTED)
    wps = [
        Waypoint(2.5, at + (0, 0, 0.06), FINGER_OPEN, "over the welder", rot=level),
        Waypoint(1.5, at, FINGER_OPEN, "down to the welder", rot=level),
        Waypoint(0.8, at, WELDER_HOLD, "grip the welder", rot=level),
        Waypoint(1.5, lifted, WELDER_HOLD, "lift it out", rot=level),
        Waypoint(3.0, aside, WELDER_HOLD, "tilt it beside the pack", rot=tilted),
    ]
    wait = RIGHT_CLEAR_S - sum(w.duration for w in wps)
    if wait > 0:
        wps.append(Waypoint(wait, aside, WELDER_HOLD, "wait for the strip", rot=tilted))
    wps.append(Waypoint(1.5, above_first, WELDER_HOLD, "over the pack", rot=tilted))
    for cell in WELD_ORDER:
        over, _ = _left_grip(weld_origin(cell, above=WELD_CLEAR, trim=True), WELDER_TILTED)
        near, _ = _left_grip(weld_origin(cell, above=WELD_NEAR, trim=True), WELDER_TILTED)
        touch, _ = _left_grip(weld_origin(cell, trim=True), WELDER_TILTED)
        wps += [
            Waypoint(1.5, over, WELDER_HOLD, f"over cell {cell}", rot=tilted),
            Waypoint(1.0, near, WELDER_HOLD, f"down to cell {cell}", rot=tilted),
            Waypoint(WELD_SETTLE_S, near, WELDER_HOLD, f"just over cell {cell}", rot=tilted),
            Waypoint(1.2, touch, WELDER_HOLD, f"touch cell {cell}", rot=tilted),
            Waypoint(WELD_S, touch, WELDER_HOLD, f"weld cell {cell}", rot=tilted),
            Waypoint(0.8, over, WELDER_HOLD, f"off cell {cell}", rot=tilted),
        ]
    over_seat, _ = _left_grip(np.array(WELDER_POS) + (0.0, 0.0, HOLDER_WALL_TOP - SEAT_IN_HOLDER[2] + FEET + DROP),
                              WELDER_LEVEL)
    wps += [
        Waypoint(1.0, wps[-1].pos + (0, 0, 0.04), WELDER_HOLD, "up from the pack", rot=tilted),
        Waypoint(3.0, over_seat + (0, 0, 0.04), WELDER_HOLD, "over the holder, level", rot=level),
        Waypoint(1.5, over_seat, WELDER_HOLD, "down to the walls", rot=level),
        Waypoint(1.0, over_seat, FINGER_OPEN, "let go", rot=level),
        Waypoint(0.5, over_seat, FINGER_OPEN, "fingers open", rot=level),
        Waypoint(1.0, over_seat + (0, 0, 0.06), FINGER_OPEN, "up", rot=level),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(3.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]
    return wps


def demo():
    """Both arms: the right lays the strip on the pack while the left takes the welder out of its holder; then the left
    welds the strip to the four cells and puts the welder back. Both go home."""
    return {"left": left_plan(), "right": right_plan()}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
