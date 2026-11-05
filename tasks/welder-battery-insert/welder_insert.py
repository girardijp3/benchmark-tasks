"""On the station table: a mini 18650 spot welder (124 x 24 x 22 mm housing, black ABS) lying on its feet, empty, its
battery lid sprung fully open, and beside it the bottom half of a 2x2 18650 battery holder with its four cells standing
in it, positive ends up. The left arm holds the welder down on the table; the right arm takes one cell out of the
holder, lays it in the welder's bay positive end toward the probes, closes the lid until the latch catches, and goes
back; then the left arm lets go and goes back too.

The welder is one articulation from ``assets/`` (its housing, the hinged lid with its opening spring, the latch slider
that holds the lid shut by contact, the sprung + contact and the USB-C fingers). The holder and each cell are their own
rigid bodies.

Run it:  inspect-robots run --embodiment yam_arena -E task=welder_insert.py --scorer task_score ...

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
SUCCESS_HOLD_S = 1.0
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the welder (its own frame: origin at the rear face's bottom centre, +x to the probes, z up) ----------------------
WELDER_WIDTH, WELDER_HEIGHT = 0.024, 0.022
FEET = 0.0006  # the rubber feet reach this far below the housing's bottom
BODY_SHIFT = np.array([0.004, 0.0, 0.0])  # the housing body's own origin (its articulation root) is 4 mm in
LID_TOP = 0.022  # the lid's top face when shut
HINGE = np.array([0.00795, 0.0, 0.02125])  # the lid's hinge axis (along y), at the rear end of the lid
LID_HALF_THICK = 0.00075  # the lid plate is 1.5 mm thick; it reaches 72.25 mm from the hinge
LID_OPEN_ANGLE = math.radians(105)  # its spring opens it this far (the hinge's limit)
RIM = 0.0218  # the bay's rim height
CELL_IN_WELDER = np.array([0.0421, 0.0, 0.011])  # the 18650 cell seated in the bay
CELL_IN_WELDER_XYZW = (-0.5, 0.5, -0.5, 0.5)  # its positive end (+z of the cell) toward the probes (+x)
WELDER_GRIP_AT = 0.105  # along the welder from its rear face: its front end, clear of the lid and the bay

# -- the holder: the bottom half of a 2x2 18650 holder (41.2 x 41.2 x 9.5 mm, 20 mm pitch), origin at its bottom face --
CELL_PITCH = 0.020
CELL_RADIUS = 0.009
CELL_SEATED_Z = 0.03475  # a cell's centre above the holder's bottom face, seated on its ledge
CELL_LENGTH = 0.0664  # 18650 cell with its positive nub

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# Where the left arm sets the welder down and holds it: probes toward the robot's left and back, the heading at which
# the right wrist lays the cell in with the most margin. It starts level across the table (probes to the robot's left)
# with its middle at the same place.
WELDER_XY, WELDER_YAW = (0.25, -0.07), math.radians(135)
START_YAW = math.radians(90)
WELDER_MID = 0.062  # m: the housing's middle along it
HOLDER_XY = (0.35, -0.20)  # on the right arm's side, in front of the welder's rear end
HOLDER_FIXED = False  # loose on the table
CELLS_IN_HOLDER = ((-1, -1), (1, -1), (-1, 1), (1, 1))  # cell_1 .. cell_4, in pitch units; cell_1 is nearest the robot


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _qmul(a, b) -> tuple[float, float, float, float]:
    """Quaternion product a * b (xyzw)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


WELDER_POS = (*WELDER_XY, TOP + FEET)
_MID = np.array(WELDER_POS) + _rz(WELDER_YAW) @ np.array([WELDER_MID, 0.0, 0.0])
WELDER_START = tuple(float(v) for v in _MID - _rz(START_YAW) @ np.array([WELDER_MID, 0.0, 0.0]))
HOLDER_POS = (*HOLDER_XY, TOP + 0.0002)
CELL_POS = tuple(np.array([HOLDER_XY[0] + i * CELL_PITCH / 2, HOLDER_XY[1] + j * CELL_PITCH / 2,
                           TOP + 0.0002 + CELL_SEATED_Z]) for i, j in CELLS_IN_HOLDER)
SEAT = np.array(WELDER_POS) + _rz(WELDER_YAW) @ CELL_IN_WELDER  # where the cell's centre ends up, in the bay


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False  # a body that never moves (the holder, taped to the table)

    @property
    def free(self) -> bool:
        return not self.kinematic


# Arena places an articulation by its root link: the welder's is its housing body, 4 mm in from the rear face.
WELDER = UsdPart("welder", asset="welder", xy=WELDER_XY, extent=0.14, mass=0.0797,
                 usd_path=str(ASSETS / "welder_task.usda"),
                 pos=tuple(float(v) for v in np.array(WELDER_START) + _rz(START_YAW) @ BODY_SHIFT),
                 rot_xyzw=_yaw_xyzw(START_YAW), articulation=True)
HOLDER = UsdPart("holder", asset="holder", xy=HOLDER_XY, extent=0.042, mass=0.0065,
                 usd_path=str(ASSETS / "holder_18650_2x2.usda"), pos=HOLDER_POS, kinematic=HOLDER_FIXED)
CELLS = [UsdPart(f"cell_{k + 1}", asset="cell", xy=tuple(float(v) for v in p[:2]), extent=0.065, mass=0.047,
                 usd_path=str(ASSETS / "battery_18650_yellow.usda"), pos=tuple(float(v) for v in p))
         for k, p in enumerate(CELL_POS)]
OBJECTS = [WELDER, HOLDER, *CELLS]

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
        its root), plus the grippers, the episode time, the welder's housing and its lid and latch."""
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
        welder = base.scene["welder"]
        ids = welder.find_joints(["LidHinge", "LatchSlide"], preserve_order=True)[0]
        lid, latch = welder.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
        out["lid_angle"], out["latch_travel"] = float(lid), float(latch)  # rad (negative opens), m
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that opens the lid, puts the latch home and winds
        their springs: Isaac Lab sets every joint drive's target to 0, which would make the lid's spring close it."""
        configure = stock_cfg_callback(control_hz)

        def with_springs(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.welder_springs = EventTermCfg(func=_open_lid, mode="reset",
                                                         params={"asset_cfg": SceneEntityCfg("welder")})
            return env_cfg

        return with_springs

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


# The springs' targets, as in the welder's USD: the lid's torsion spring winds toward fully open, the latch's return
# spring presses the slider back with a small preload. The lid starts where its spring holds it: fully open.
SPRING_TARGETS = {"LidHinge": -LID_OPEN_ANGLE, "LatchSlide": -0.001}
START_JOINTS = {"LidHinge": -LID_OPEN_ANGLE, "LatchSlide": 0.0}


def _open_lid(env, env_ids, asset_cfg) -> None:
    """Reset event: the lid fully open and the latch home (the simulator keeps an articulation's joints where they
    were, and starts them at 0: lid shut), and the springs' drive targets as the welder's USD has them."""
    import torch

    if env_ids is None:
        return
    welder = env.scene[asset_cfg.name]
    ids = welder.find_joints(list(SPRING_TARGETS), preserve_order=True)[0]
    start = torch.tensor([[START_JOINTS[k] for k in SPRING_TARGETS]], device=welder.device).repeat(len(env_ids), 1)
    welder.write_joint_position_to_sim_index(position=start, joint_ids=ids, env_ids=env_ids)
    welder.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(start), joint_ids=ids, env_ids=env_ids)
    target = torch.tensor([list(SPRING_TARGETS.values())], device=welder.device).repeat(len(env_ids), 1)
    welder.set_joint_position_target_index(target=target, joint_ids=ids, env_ids=env_ids)


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
HOLDER_TOP = 0.0095  # m: the holder's top above its bottom face
SEATED_OFF = 0.003  # m: a cell's centre within this of its seat in the bay, and
ALIGNED = math.radians(10)  # ... its positive end toward the probes within this
LID_SHUT = math.radians(-2)  # the lid's hinge past this counts as shut, and
LATCH_HOME = 0.0003  # ... the latch slider back within this: the latch has caught the lid
ON_FEET = 0.003  # m: the welder's bottom within this of resting on its feet, and
UPRIGHT = math.radians(10)  # ... upright within this
TURNED = math.radians(30)  # the welder turned at least this far from its start (about the vertical)
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
# The parts' boxes in their own frames (m): a cell (centre origin, + end along z) and the welder housing (its origin
# frame: from the rear face to the probes' tips).
CELL_BOX = (np.array([-CELL_RADIUS, -CELL_RADIUS, -0.03325]), np.array([CELL_RADIUS, CELL_RADIUS, 0.03325]))
WELDER_BOX = (np.array([0.0, -WELDER_WIDTH / 2, 0.0]), np.array([0.1387, WELDER_WIDTH / 2, WELDER_HEIGHT]))
CELL_NAMES = tuple(c.name for c in CELLS)
STEPS = ("battery taken out", "battery in the bay", "lid latched", "welder turned and put down")
POINTS = 0.25  # per step


def welder_origin(objects) -> np.ndarray:
    h = objects["housing"]
    return h.pos - quat_xyzw_to_matrix(h.quat_xyzw) @ BODY_SHIFT


def _welder_pose(objects) -> PartPose:
    return PartPose(pos=welder_origin(objects), quat_xyzw=objects["housing"].quat_xyzw)


def cell_in_bay(objects, name: str):
    """How far the cell's centre is from its seat in the bay (m) and the angle between its positive end and the
    welder's probes (rad)."""
    r = quat_xyzw_to_matrix(objects["housing"].quat_xyzw)
    seat = welder_origin(objects) + r @ CELL_IN_WELDER
    cell = objects[name]
    plus = quat_xyzw_to_matrix(cell.quat_xyzw)[:, 2]
    return float(np.linalg.norm(cell.pos - seat)), math.acos(float(np.clip(plus @ r[:, 0], -1.0, 1.0)))


def seated_cell(objects) -> str | None:
    """The cell lying in the bay the right way round, if any."""
    for name in CELL_NAMES:
        off, angle = cell_in_bay(objects, name)
        if off < SEATED_OFF and angle < ALIGNED:
            return name
    return None


def lid_latched(objects) -> bool:
    return bool(objects["lid_angle"] > LID_SHUT and objects["latch_travel"] < LATCH_HOME)


def welder_turn(objects) -> float:
    """How far the welder has turned about the vertical from its start (rad, 0 to pi)."""
    x = quat_xyzw_to_matrix(objects["housing"].quat_xyzw)[:, 0]
    heading = math.atan2(x[1], x[0])
    return abs((heading - START_YAW + math.pi) % (2 * math.pi) - math.pi)


def welder_on_table(objects) -> bool:
    up = float(quat_xyzw_to_matrix(objects["housing"].quat_xyzw)[2, 2])
    return bool(abs(welder_origin(objects)[2] - (TOP + FEET)) < ON_FEET and up > math.cos(UPRIGHT))


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


def _cell_bottom(cell) -> float:
    """The cell's lowest point (m, world z)."""
    axis_z = abs(float(quat_xyzw_to_matrix(cell.quat_xyzw)[2, 2]))
    return float(cell.pos[2] - 0.03325 * axis_z - CELL_RADIUS * math.sqrt(max(0.0, 1.0 - axis_z ** 2)))


def battery_taken_out(objects) -> bool:
    """A gripper holds a cell lifted clear of the holder: the cell's lowest point above the holder's top."""
    top = objects["holder"].pos[2] + HOLDER_TOP
    return any(_cell_bottom(objects[n]) > top and _held_by_any(objects, _box(objects[n], CELL_BOX)) for n in CELL_NAMES)


def welder_put_down(objects) -> bool:
    """The welder stands on its feet on the table, turned at least TURNED from its start, and no gripper holds it."""
    return bool(welder_on_table(objects) and welder_turn(objects) >= TURNED
                and not _held_by_any(objects, _box(_welder_pose(objects), WELDER_BOX)))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether a cell has been taken out of the holder. Starts over when the
    episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.taken_out = False
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.taken_out |= battery_taken_out(objects)


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached; 2 to 4 as things stand. Step 3 counts only with a cell in the bay."""
    p = _PROGRESS
    p.update(objects)
    seated = seated_cell(objects) is not None
    done = [bool(p.taken_out), seated, bool(seated and lid_latched(objects)), welder_put_down(objects)]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        off = min(cell_in_bay(objects, n)[0] for n in CELL_NAMES)
        print(f"[welder_insert] t={objects['episode_time']:.2f}s  steps done: {got}  (nearest cell {off * 1000:.1f} mm "
              f"from the seat; lid {math.degrees(objects['lid_angle']):.0f} deg; welder turned "
              f"{math.degrees(welder_turn(objects)):.0f} deg)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the welder or a cell off the table, or a state that is not finite."""
    for pos in [objects["housing"].pos] + [objects[n].pos for n in CELL_NAMES]:
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the welder and the battery in it
    still, and no violation. (The cells left standing in the holder keep a sub-millimetre contact jitter in PhysX.)"""
    seated = seated_cell(objects)
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["welder"].resting() and seated is not None
                and objects[seated].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from above, across the welder's front end (clear of the bay and the lid); hold it down on the table until
# the lid is latched, let go and go home.
# Right arm: pointing sideways along the holder's diagonal, fingers closing across the corner cell's two free sides and
# their tips stopping short of its two neighbours; lift the cell out, turn the gripper down (the cell now lies flat,
# positive end forward), lay it in the bay from just above the rim; then push the open lid over and down from behind,
# press it shut so the latch catches, and go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width
WELDER_HOLD = -(WELDER_WIDTH - 1.5 * SQUEEZE) / 2  # ~30 N a finger
CELL_HOLD = -(2 * CELL_RADIUS - SQUEEZE) / 2  # ~20 N a finger
WELDER_TIP_PAST_MID = 0.003  # the left pad tips this far past the housing's mid-height
LEAN = math.radians(50)  # the left gripper leans toward +y (the probes) about its finger-closing axis
# The pick: the gripper points horizontally along the holder's diagonal, toward its centre. The fingers (15.5 mm wide,
# 4 mm thick each) close across the corner cell's free sides; the cell's two neighbours stand 14 mm ahead of its axis,
# so the pad tips stop PICK_TIPS_PAST past it (up to 5 mm clears them). The pads' width is vertical, PICK_HEIGHT up.
PICK_TIPS_PAST = 0.003
PICK_HEIGHT = 0.050  # m: the TCP above the table; the gripper's body hangs 34-40 mm below its axis
PICK_TILT = math.radians(15)  # the gripper points this far below level (the right wrist reaches the pose that way)
PICK_LIFT = 0.10
PICK_ROLL = 1  # +1 or -1: which way round the gripper is about its pointing axis (its wrist's range)
OVER_BAY = 0.03  # m: the cell is lowered straight from this far above where it is let go
OVER_LID = 0.12  # m: ... after being carried in this high, clear of the open lid (its top 93 mm up), turning flat on the
# way (the right wrist can't turn it above the holder)
# Laying the cell in the bay: the pad tips stop RELEASE_ABOVE_RIM above the bay's rim (the cell's lower part is then
# already between the bay walls, 0.5 mm a side) and the fingers open; the cell drops into its seat.
RELEASE_ABOVE_RIM = 0.0015
PAD_HALF_WIDTH = 0.00775
# Closing the lid: the lower front corner of the closed fingertips (TIP_HALF along the welder from the pads' middle)
# follows the lid's outer face, LID_PUSH_GAP off it, at LID_PUSH_R from the hinge (near its free end, so the pads above
# the corner stay clear of the lid leaning back past vertical); the spring keeps the lid against it. At the end it is
# pressed LID_PRESS below the shut lid, so the hook passes the latch tongue's ramp and catches.
TIP_HALF = 0.0057
LID_PUSH_R = 0.070
LID_PUSH_GAP = 0.0015
LID_PRESS = 0.002


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
_Z = np.array([0.0, 0.0, 1.0])
def _left_grip_rot(yaw: float) -> np.ndarray:
    """The left gripper across the housing of a welder heading ``yaw``, leaned toward its probes."""
    return _rz(yaw - math.pi / 2) @ axis_angle_matrix(_X, -LEAN) @ yam_demo.grip_rotation(-math.pi / 2)

WELDER_AXIS = _rz(WELDER_YAW) @ _X  # the welder's long axis, rear to probes
RIGHT_PUSH = yam_demo.grip_rotation(WELDER_YAW)  # upright, fingers closing across the welder, their width along it

# The pick's directions (world): ALONG the holder's diagonal toward its centre (level), POINT the gripper's pointing
# (ALONG tilted PICK_TILT down), CLOSE across the diagonal; the gripper's frame is x = closing, y = pad width,
# z = pointing.
ALONG = np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0)
POINT = ALONG * math.cos(PICK_TILT) - _Z * math.sin(PICK_TILT)
CLOSE = np.cross(ALONG, _Z) * PICK_ROLL
PICK_ROT = np.column_stack([CLOSE, np.cross(POINT, CLOSE), POINT])
# Turned a quarter turn about the closing axis: the cell (its + end was up) lies level along ALONG, the gripper points
# down (PICK_TILT off vertical); then turned about the vertical so the cell lies along the welder, + end toward the
# probes.
DOWN_ROT = axis_angle_matrix(np.cross(_Z, ALONG), math.pi / 2) @ PICK_ROT
INSERT_ROT = _rz(math.atan2(WELDER_AXIS[1], WELDER_AXIS[0]) - math.atan2(ALONG[1], ALONG[0])) @ DOWN_ROT


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ np.array([0.0, 0.0, 1.0]))


def _welder_grip(origin, yaw: float = WELDER_YAW) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP and rotation gripping across the front end of the welder whose origin is at ``origin``, heading
    ``yaw``."""
    grip = np.asarray(origin, float) + _rz(yaw) @ np.array([WELDER_GRIP_AT, 0.0, WELDER_HEIGHT / 2])
    rot = _left_grip_rot(yaw)
    return _tcp_with_tips(grip, WELDER_TIP_PAST_MID, rot), rot


# The cell in the right hand, in the gripper's frame (fixed from the pick on): its centre, and its + end's direction.
_CELL_IN_HAND = PICK_ROT.T @ (np.array([0.0, 0.0, TOP + 0.0002 + CELL_SEATED_Z])
                             - np.array([0.0, 0.0, TOP + PICK_HEIGHT])
                             + (PAD_TIP_PAST_TCP - PICK_TIPS_PAST) * POINT)
_PLUS_IN_HAND = PICK_ROT.T @ _Z


def _tcp_holding_cell_at(centre, rot) -> np.ndarray:
    return np.asarray(centre, float) - rot @ _CELL_IN_HAND


def _lid_corner(angle: float, press: float = 0.0) -> np.ndarray:
    """The world point for the fingertips' lower front corner on the lid's outer face, the lid opened ``angle`` (rad,
    0 shut), LID_PUSH_R from the hinge; ``press`` pushes it that much further in."""
    d = np.array([math.cos(angle), 0.0, math.sin(angle)])  # along the lid, hinge to free end (welder frame)
    n = np.array([-math.sin(angle), 0.0, math.cos(angle)])  # its outer face's normal
    p = HINGE + LID_PUSH_R * d + (LID_HALF_THICK + LID_PUSH_GAP - press) * n
    return np.asarray(WELDER_POS) + _rz(WELDER_YAW) @ p


def _push_tcp(corner) -> np.ndarray:
    return _tcp_with_tips(np.asarray(corner, float) - TIP_HALF * WELDER_AXIS, 0.0, RIGHT_PUSH)


LEFT_PLACED_S = 2.5 + 1.5 + 0.8 + 1.5 + 2.5 + 1.5 + 0.5  # the left arm has picked the welder up, set it down turned
WELDER_LIFT = 0.03  # m: the welder carried this high from its start to where it is held


def right_plan() -> list[Waypoint]:
    """Take cell_1 out of the holder, lay it in the bay, close the lid, go home."""
    cell = np.array(CELL_POS[0])
    pick = np.array([cell[0], cell[1], TOP + PICK_HEIGHT]) - (PAD_TIP_PAST_TCP - PICK_TIPS_PAST) * POINT
    lifted = pick + (0, 0, PICK_LIFT)
    plus_world = INSERT_ROT @ _PLUS_IN_HAND
    assert plus_world @ WELDER_AXIS > 0.99, plus_world  # + end toward the probes
    seated_tcp = _tcp_holding_cell_at(SEAT, INSERT_ROT)
    tips = seated_tcp + PAD_TIP_PAST_TCP * (INSERT_ROT @ _Z)  # the pad tips' middle, were the cell seated; their
    tips_z = tips[2] - PAD_HALF_WIDTH * abs(INSERT_ROT[2, 1])  # lowest corner, the gripper tilted
    release = seated_tcp + (0, 0, WELDER_POS[2] + RIM + RELEASE_ABOVE_RIM - tips_z)
    over_bay = release + (0, 0, OVER_BAY)
    behind_lid = _push_tcp(_lid_corner(LID_OPEN_ANGLE + math.radians(8)))
    sweep = [_push_tcp(_lid_corner(math.radians(a))) for a in (100, 85, 65, 45, 25, 8, 0)]
    pressed = _push_tcp(_lid_corner(0.0, press=LID_PUSH_GAP + LID_PRESS))
    wps = [
        Waypoint(2.5, pick - 0.05 * POINT + (0, 0, 0.12), FINGER_OPEN, "above the holder's corner", rot=PICK_ROT),
        Waypoint(1.0, pick - 0.05 * POINT, FINGER_OPEN, "level with the cell", rot=PICK_ROT),
        Waypoint(1.5, pick, FINGER_OPEN, "around the corner cell", rot=PICK_ROT),
        Waypoint(0.8, pick, CELL_HOLD, "grip the cell", rot=PICK_ROT),
        Waypoint(2.0, lifted, CELL_HOLD, "lift it out", rot=PICK_ROT),
        Waypoint(max(LEFT_PLACED_S - 7.8, 0.5), lifted, CELL_HOLD, "wait for the welder", rot=PICK_ROT),
        Waypoint(4.0, release + (0, 0, OVER_LID), CELL_HOLD, "over the bay, turned flat", rot=INSERT_ROT),
        Waypoint(2.0, over_bay, CELL_HOLD, "down over the bay", rot=INSERT_ROT),
        Waypoint(1.5, release, CELL_HOLD, "down to the rim", rot=INSERT_ROT),
        Waypoint(0.8, release, FINGER_OPEN, "let it drop in", rot=INSERT_ROT),
        Waypoint(1.0, release + (0, 0, 0.06), FINGER_OPEN, "up", rot=INSERT_ROT),
        Waypoint(2.0, behind_lid + (0, 0, 0.06), 0.0, "over the lid's back", rot=RIGHT_PUSH),
        Waypoint(1.0, behind_lid, 0.0, "behind the lid", rot=RIGHT_PUSH),
    ]
    wps += [Waypoint(1.0, p, 0.0, "close the lid", rot=RIGHT_PUSH) for p in sweep]
    wps += [
        Waypoint(1.0, pressed, 0.0, "press it shut", rot=RIGHT_PUSH),
        Waypoint(0.5, pressed, 0.0, "hold", rot=RIGHT_PUSH),
        Waypoint(0.8, pressed + (0, 0, 0.05), 0.0, "lift away", rot=RIGHT_PUSH),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return wps


def left_plan(hold_s: float) -> list[Waypoint]:
    """Grip the welder by its front end, lift it, set it down turned to where it is held, and hold it down on the
    table; let go once the lid is latched, go home."""
    start, start_rot = _welder_grip(WELDER_START, START_YAW)
    at, rot = _welder_grip(WELDER_POS)
    return [
        Waypoint(2.5, start + (0, 0, 0.06), FINGER_OPEN, "over the welder", rot=start_rot),
        Waypoint(1.5, start, FINGER_OPEN, "down to the welder", rot=start_rot),
        Waypoint(0.8, start, WELDER_HOLD, "grip the welder", rot=start_rot),
        Waypoint(1.5, start + (0, 0, WELDER_LIFT), WELDER_HOLD, "lift it", rot=start_rot),
        Waypoint(2.5, at + (0, 0, WELDER_LIFT), WELDER_HOLD, "turn it", rot=rot),
        Waypoint(1.5, at, WELDER_HOLD, "set it down", rot=rot),
        Waypoint(hold_s, at, WELDER_HOLD, "hold it down", rot=rot),
        Waypoint(1.0, at, FINGER_OPEN, "let go", rot=rot),
        Waypoint(1.0, at + (0, 0, 0.06), FINGER_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(3.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


def demo():
    """Both arms: the left turns the welder to where it holds it down while the right takes a cell out of the holder;
    the right lays the cell in it and closes the lid; then the left lets go. Both go home."""
    right = right_plan()
    held_until = sum(w.duration for w in right) - 2.5  # the right arm is lifting away from the shut lid
    return {"left": left_plan(hold_s=held_until - (LEFT_PLACED_S - 0.5)), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
