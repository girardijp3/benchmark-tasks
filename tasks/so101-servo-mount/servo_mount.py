"""On the station table: the shoulder bracket of an SO-101 robot arm (its 3D-printed "rotation pitch" part with the motor
holder screwed on), standing on its lower fork arm with its rectangular servo pocket opening upward and empty, and an
STS3215 servo motor standing on its rear end. The left arm picks the bracket up by its motor holder and holds it up;
the right arm picks the servo up, turns it a quarter turn so its horn faces the side the upper arm will go on, lowers
it into the pocket (0.25 mm a side) until it rests on the pocket's floor, lets go and goes back. Then the left arm sets
the bracket down standing, lets go and goes back too.

The bracket and the servo are rigid bodies from ``assets/so101_shoulder_bracket.usda`` and
``assets/sts3215_servo.usda``.

Run it:  inspect-robots run --embodiment yam_arena -E task=servo_mount.py --scorer task_score ...

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
TAG = "servo_mount"

# -- the bracket (its own frame: origin on its underside below the pocket's axis, z up through the pocket, +x toward
# its back (the motor holder's flat side), its fork arms toward -x, the servo's horn on its -y side) -----------------
# The servo seated in the pocket: its centre, and its rotation (its +x, the output-shaft end, up; its horn, +z, toward
# the bracket's -y, the side the upper arm goes on).
SEAT_POS = np.array([0.0, 0.0, 0.0879013])
SEAT_XYZW = (0.5, -0.5, 0.5, 0.5)
POCKET_DEPTH = 0.019  # m: from the pocket's floor (the servo's rear end, 22.7 mm below its centre) up to the rim
BRACKET_TOP = 0.0842  # m: the rim, the bracket's highest point
# The motor holder's ring around the pocket: its two long outer faces (y) and its height (z); the left hand pinches it.
RING_Y = (-0.0268, 0.0256)
RING_Z = (0.0562, 0.0842)
BRACKET_BOX = (np.array([-0.042, -0.0268, 0.0]), np.array([0.0202, 0.0256, 0.0842]))
CLEARANCE = 0.00025  # m a side

# -- the servo (its own frame: origin at the case's middle, +x to the output-shaft end, +z through the horn) ----------
SERVO_BOX = (np.array([-0.0227, -0.0124, -0.0177]), np.array([0.0227, 0.0124, 0.0187]))  # with the horn
SERVO_HALF = 0.0227  # m: the case from its centre to either end, along its +x
SERVO_WIDTH = 0.0248  # across its sides (y), where the gripper holds it

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The bracket stands with its back (the holder's flat side) toward the robot and its fork arms away: the left hand
# comes at the ring from behind, pointing forward. Seated, the servo's horn faces the robot's left.
BRACKET_XY, BRACKET_YAW = (0.30, 0.03), math.radians(180)
# The servo stands on its rear end on the right, horn away from the robot: it has to be turned a quarter turn.
SERVO_XY, SERVO_HORN_YAW = (0.27, -0.17), math.radians(0)
# Where the left hand holds the bracket up while the servo goes in (its underside this high, over this spot).
HOLD_XY, HOLD_LIFT = (0.30, 0.03), 0.05


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _matrix_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> quaternion (xyzw)."""
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(r))


_Z = np.array([0.0, 0.0, 1.0])


def standing_rot(horn_yaw: float) -> np.ndarray:
    """The servo standing on its rear end (its +x up), its horn (+z) heading ``horn_yaw``."""
    x = _Z
    z = _rz(horn_yaw) @ np.array([1.0, 0.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


BRACKET_ROT = _rz(BRACKET_YAW)
BRACKET_POS0 = np.array([*BRACKET_XY, TOP])
HOLD_POS = np.array([*HOLD_XY, TOP + HOLD_LIFT])
SEAT_ROT_B = quat_xyzw_to_matrix(np.array(SEAT_XYZW))  # the seated servo, bracket frame
SEAT_ROT = BRACKET_ROT @ SEAT_ROT_B  # ... world, the bracket standing or held (it is not turned)
SEAT_HELD = HOLD_POS + BRACKET_ROT @ SEAT_POS  # the seated servo's centre, the bracket held up
SERVO_ROT0 = standing_rot(SERVO_HORN_YAW)
SERVO_POS0 = (*SERVO_XY, TOP + SERVO_HALF + 0.0002)

# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body. ``Asset``'s fields only
    satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


BRACKET = UsdPart("bracket", asset="bracket", xy=BRACKET_XY, extent=0.065, mass=0.045,
                  usd_path=str(ASSETS / "so101_shoulder_bracket.usda"),
                  pos=(*BRACKET_XY, TOP + 0.0002), rot_xyzw=_yaw_xyzw(BRACKET_YAW))
SERVO = UsdPart("servo", asset="servo", xy=SERVO_XY, extent=0.046, mass=0.055,
                usd_path=str(ASSETS / "sts3215_servo.usda"), pos=SERVO_POS0, rot_xyzw=_matrix_xyzw(SERVO_ROT0))
OBJECTS = [BRACKET, SERVO]

MAX_DEPENETRATION = 0.2  # m/s: tight contacts with 20 N grippers stay stable


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon={"rigid_props": sim_utils.RigidBodyPropertiesCfg(
            max_depenetration_velocity=MAX_DEPENETRATION)},
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
    closing: np.ndarray = None  # the fingers' closing axis (unit)


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
        frame = runtime.link_poses(robot)["gripper"][:3, :3]
        axis = frame[:, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)  # fingers open to -q each; a closing finger is held back
        out[f"{side}_gripper"] = GripperState(
            back=tcp - PAD_BACK * axis,
            tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
            closing=frame[:, 0],
        )
    return out


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root pose and velocities, plus the grippers and the episode time."""
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
        return out

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states


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
LIFTED = 0.010  # m: picked up once the part's lowest point is this far above the table, held by a gripper
IN_POCKET = 0.005  # m: in the pocket once its rear end is at least this far below the rim, and
IN_LINE = 0.0015  # ... its axis within this of the pocket's (sideways), and
IN_TILT = math.radians(5)  # ... in line with the pocket within this, and
HORN_RIGHT = math.radians(30)  # ... its horn toward the upper arm's side within this
SEATED = 0.001  # m: fully seated once its rear end is within this of the pocket's floor, and
SEATED_TILT = math.radians(2)  # ... in line within this
ON_TABLE = 0.003  # m: the bracket's underside within this of the table, and
UPRIGHT = math.radians(10)  # ... upright within this: it stands on the table
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: both of a gripper's pads (their middles) are on the part (its box, grown by GRIP_SLACK), both fingers press on
# something with at least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing). Pads, not
# the gripper's axis: the left hand's axis runs through the pocket while it holds the bracket's ring.
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
STEPS = ("bracket picked up", "servo in the pocket the right way round", "servo pressed fully home",
         "bracket set down standing, let go")
POINTS = 0.25  # per step
TRACE_S = 1.0  # s between trace lines in a run


@dataclass(frozen=True)
class SeatError:
    """Where the servo is against its seated pose, in the pocket's frame (wherever the bracket is): how far its centre
    is above the seated centre along the pocket (m; its rear end's height over the floor), how far off the pocket's
    line sideways (m), how far its axis is tilted from the pocket's (rad), and how far its horn is turned from the
    upper arm's side (rad)."""

    height: float
    off_line: float
    tilt: float
    horn: float


def seat_error(objects) -> SeatError:
    br = objects["bracket"]
    r_br = quat_xyzw_to_matrix(br.quat_xyzw)
    r_seat = r_br @ SEAT_ROT_B
    seat = br.pos + r_br @ SEAT_POS
    servo = objects["servo"]
    r = quat_xyzw_to_matrix(servo.quat_xyzw)
    d = r_seat.T @ (servo.pos - seat)
    tilt = math.acos(float(np.clip(r[:, 0] @ r_seat[:, 0], -1.0, 1.0)))
    horn = math.acos(float(np.clip(r[:, 2] @ r_seat[:, 2], -1.0, 1.0)))
    return SeatError(height=float(d[0]), off_line=float(math.hypot(d[1], d[2])), tilt=tilt, horn=horn)


def in_pocket(objects) -> bool:
    """The servo's rear end at least IN_POCKET down in the pocket, in line with it, horn the right way."""
    e = seat_error(objects)
    return bool(e.height < POCKET_DEPTH - IN_POCKET and e.off_line < IN_LINE and e.tilt < IN_TILT
                and e.horn < HORN_RIGHT)


def fully_seated(objects) -> bool:
    e = seat_error(objects)
    return bool(in_pocket(objects) and abs(e.height) < SEATED and e.tilt < SEATED_TILT)


def bracket_standing(objects) -> bool:
    """The bracket stands on the table: its underside on it, upright."""
    br = objects["bracket"]
    up = float(quat_xyzw_to_matrix(br.quat_xyzw)[2, 2])
    return bool(abs(br.pos[2] - TOP) < ON_TABLE and up > math.cos(UPRIGHT))


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose``."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ quat_xyzw_to_matrix(pose.quat_xyzw).T + pose.pos
    return w.min(0), w.max(0)


def _held(pose, box, gripper) -> bool:
    """``gripper`` holds the part at ``pose`` (its ``box`` in its own frame): both pads on it, both fingers pressing."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    middle = (gripper.back + gripper.tips) / 2
    r = quat_xyzw_to_matrix(pose.quat_xyzw)
    lo, hi = box[0] - GRIP_SLACK, box[1] + GRIP_SLACK
    for side in (-1.0, 1.0):
        pad = r.T @ (middle + side * gripper.opening / 2 * np.asarray(gripper.closing) - pose.pos)
        if not (np.all(pad >= lo) and np.all(pad <= hi)):
            return False
    return True


def held(objects, name: str, sides=("left", "right")) -> bool:
    box = SERVO_BOX if name == "servo" else BRACKET_BOX
    return any(_held(objects[name], box, objects.get(f"{s}_gripper")) for s in sides)


def picked_up(objects, name: str) -> bool:
    """A gripper holds the part with its lowest point at least LIFTED above the table."""
    box = SERVO_BOX if name == "servo" else BRACKET_BOX
    return bool(_box(objects[name], box)[0][2] > TOP + LIFTED and held(objects, name))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether the bracket and the servo have been picked up. Starts over when
    the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.bracket = False
        self.servo = False
        self.reported = [False] * len(STEPS)
        self.next_trace = 0.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.bracket |= picked_up(objects, "bracket")
        self.servo |= picked_up(objects, "servo")


_PROGRESS = _Progress()


def _trace(objects) -> None:
    e, s, b = seat_error(objects), objects["servo"], objects["bracket"]
    tilt_b = math.degrees(math.acos(float(np.clip(quat_xyzw_to_matrix(b.quat_xyzw)[2, 2], -1, 1))))
    print(f"[{TAG}] t={objects['episode_time']:5.1f}s servo {np.round(s.pos * 1000, 1)} mm v "
          f"{np.linalg.norm(s.lin_vel) * 1000:5.1f} | bracket {np.round(b.pos * 1000, 1)} mm tilt {tilt_b:4.1f} deg v "
          f"{np.linalg.norm(b.lin_vel) * 1000:5.1f} | rear end {e.height * 1000:6.1f} mm over the floor, "
          f"{e.off_line * 1000:5.2f} mm off line, tilt {math.degrees(e.tilt):4.1f}, horn {math.degrees(e.horn):3.0f} "
          f"| held servo {held(objects, 'servo')} bracket {held(objects, 'bracket')}", flush=True)


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached; 2 to 4 as things stand, and 2 to 4 only after a gripper picked the servo
    up (a gripper did the work)."""
    p = _PROGRESS
    p.update(objects)
    if "episode_time" in objects and objects["episode_time"] >= p.next_trace:  # in a run: a trace every TRACE_S
        p.next_trace = objects["episode_time"] + TRACE_S
        _trace(objects)
    inside = bool(p.servo and in_pocket(objects))
    seated = bool(inside and fully_seated(objects))
    down = bool(inside and bracket_standing(objects) and not held(objects, "bracket") and not held(objects, "servo"))
    done = [bool(p.bracket), inside, seated, down]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        e = seat_error(objects)
        print(f"[{TAG}] t={objects['episode_time']:.2f}s  steps done: {got}  (servo's rear end "
              f"{e.height * 1000:.1f} mm above the pocket's floor, {e.off_line * 1000:.2f} mm off its line, tilted "
              f"{math.degrees(e.tilt):.1f} deg, horn {math.degrees(e.horn):.0f} deg off)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the bracket or the servo off the table, or a state that is not finite."""
    for name in ("bracket", "servo"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the bracket and the servo still,
    and no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["servo"].resting() and objects["bracket"].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from behind the bracket, pointing forward and LEFT_TILT down, the fingers close across the motor holder's
# ring (its two long sides, 52.4 mm apart); lift the bracket HOLD_LIFT and hold it until the servo is in; set it down
# where it stood, let go, back off and go home.
# Right arm: from above, tilted RIGHT_TILT about its closing axis toward the servo's horn side (the right wrist reaches
# high that way), the fingers closing across the servo's sides with their lowest corners GRIP_ABOVE_REAR its rear end,
# so they stay clear of the rim when it is seated; lift it clear of the held bracket, carry it over the pocket while
# turning it a quarter turn, lower it in line to just above the rim, then down to the floor and PRESS a little past
# it; let go and go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width
SERVO_HOLD = -(SERVO_WIDTH - SQUEEZE) / 2  # ~20 N a finger
RING_WIDTH = RING_Y[1] - RING_Y[0]
BRACKET_HOLD = -(RING_WIDTH - 1.5 * SQUEEZE) / 2  # ~30 N a finger: the push into the pocket must not slide it
PAD_HALF_WIDTH = 0.00775
PAD_LENGTH = PAD_BACK + PAD_TIP_PAST_TCP
LEFT_TILT = math.radians(10)  # the left gripper points this far below level
LEFT_PADS_PAST_AXIS = 0.0  # m: the left pads' middle this far behind the pocket's axis (bracket +x): on it, so the
# push into the pocket turns the bracket in the hand as little as possible
LEFT_BACK_OFF = 0.06  # m: the left gripper comes in and backs off along its pointing axis
RIGHT_TILT = math.radians(30)  # the right gripper leans this far from vertical, about its closing axis
GRIP_ABOVE_REAR = 0.024  # m: the right pads' lowest corner this far above the servo's rear end (the rim: 19 mm)
CARRY_CLEAR = 0.012  # m: the servo's rear end this far above the rim while carried over it
OVER_RIM = 0.003  # m: lowered in line to this far above the rim, then pushed in
RELEASE_AT = 0.008  # m: let go with its rear end this far above the floor (11 mm in): the walls line it up as it drops
PRESS = 0.002  # m: then the closed fingertips press its top this far past where it sits when seated
HELD_SAG = 0.0017  # m: the bracket hangs this much lower in the left hand than where the hand was sent (measured)


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


def _grip_frame(closing, pointing) -> np.ndarray:
    """A gripper rotation: x = closing, y = the pads' width, z = pointing."""
    c = np.asarray(closing, float) / np.linalg.norm(closing)
    p = np.asarray(pointing, float) / np.linalg.norm(pointing)
    return np.column_stack([c, np.cross(p, c), p])


def _tcp_for_pads(middle, rot) -> np.ndarray:
    """The TCP that puts the pads' middle (halfway along them, between the fingers) at ``middle``."""
    return np.asarray(middle, float) + (PAD_BACK - PAD_LENGTH / 2) * rot[:, 2]


# The left hand on the ring, bracket frame: pointing -x tilted down, closing across y.
_LEFT_P = np.array([-math.cos(LEFT_TILT), 0.0, -math.sin(LEFT_TILT)])
LEFT_ROT_B = _grip_frame((0.0, 1.0, 0.0), _LEFT_P)  # the left wrist's range: its fingers close toward world -y
LEFT_TCP_B = _tcp_for_pads((LEFT_PADS_PAST_AXIS, sum(RING_Y) / 2, sum(RING_Z) / 2), LEFT_ROT_B)


def _left_on_bracket(bracket_pos) -> tuple[np.ndarray, np.ndarray]:
    return np.asarray(bracket_pos, float) + BRACKET_ROT @ LEFT_TCP_B, BRACKET_ROT @ LEFT_ROT_B


# The right hand on the servo, servo frame: closing across its sides (y), pointing down along -x leaned RIGHT_TILT
# toward its horn side (+z); the pads' lowest corner GRIP_ABOVE_REAR above the rear end, the pads' middle on its axis.
_RIGHT_P = np.array([-math.cos(RIGHT_TILT), 0.0, math.sin(RIGHT_TILT)])
RIGHT_ROT_S = _grip_frame((0.0, 1.0, 0.0), _RIGHT_P)
_low = PAD_LENGTH / 2 * math.cos(RIGHT_TILT) + PAD_HALF_WIDTH * math.sin(RIGHT_TILT)  # the middle above the corner
RIGHT_TCP_S = _tcp_for_pads((-SERVO_HALF + GRIP_ABOVE_REAR + _low, 0.0, 0.0), RIGHT_ROT_S)


def _right_on_servo(servo_pos, servo_rot) -> tuple[np.ndarray, np.ndarray]:
    r = np.asarray(servo_rot)
    return np.asarray(servo_pos, float) + r @ RIGHT_TCP_S, r @ RIGHT_ROT_S


def _fingertips_on(point, rot) -> np.ndarray:
    """The TCP that puts the closed fingertips' lowest corner at ``point`` (the gripper at ``rot``)."""
    tips = PAD_TIP_PAST_TCP * rot[:, 2]
    corner = min((tips + s * PAD_HALF_WIDTH * rot[:, 1] for s in (-1.0, 1.0)), key=lambda c: c[2])
    return np.asarray(point, float) - corner


def right_plan() -> list[Waypoint]:
    """Pick the servo up, turn it over the held bracket's pocket and push it in, let go of it part-way in (it drops
    to the floor), press its top with the closed fingertips, go home."""
    pick, rot0 = _right_on_servo((*SERVO_XY, TOP + SERVO_HALF), SERVO_ROT0)
    rim = HOLD_POS[2] - HELD_SAG + BRACKET_TOP  # the held bracket's rim
    carry = rim + CARRY_CLEAR + SERVO_HALF  # the servo's centre while carried
    up, _ = _right_on_servo((*SERVO_XY, carry), SERVO_ROT0)
    axis = SEAT_HELD - (0, 0, HELD_SAG)  # the seated servo's centre, the bracket held
    over, rot = _right_on_servo(np.array([axis[0], axis[1], carry]), SEAT_ROT)
    at_rim, _ = _right_on_servo(axis + (0, 0, POCKET_DEPTH + OVER_RIM), SEAT_ROT)
    release, _ = _right_on_servo(axis + (0, 0, RELEASE_AT), SEAT_ROT)
    back = -rot[:, 2]  # straight back out along the gripper's axis
    top = axis + (0, 0, SERVO_HALF)  # the middle of the seated servo's top (its output-shaft end)
    above_top = _fingertips_on(top + (0, 0, 0.002), rot)
    pressed = _fingertips_on(top - (0, 0, PRESS), rot)
    return [
        Waypoint(2.5, pick + 0.06 * back, FINGER_OPEN, "over the servo", rot=rot0),
        Waypoint(1.5, pick, FINGER_OPEN, "down around it", rot=rot0),
        Waypoint(0.8, pick, SERVO_HOLD, "grip it", rot=rot0),
        Waypoint(1.5, up, SERVO_HOLD, "lift it", rot=rot0),
        Waypoint(3.0, over, SERVO_HOLD, "over the pocket, turned", rot=rot),
        Waypoint(1.5, at_rim, SERVO_HOLD, "down to the rim", rot=rot),
        Waypoint(2.0, release, SERVO_HOLD, "into the pocket", rot=rot),
        Waypoint(0.6, release, FINGER_OPEN, "let go", rot=rot),
        Waypoint(0.8, release + 0.015 * back, 0.0, "up, fingers closed", rot=rot),
        Waypoint(1.0, above_top, 0.0, "over its top", rot=rot),
        Waypoint(0.8, pressed, 0.0, "press it home", rot=rot),
        Waypoint(0.5, pressed, 0.0, "hold", rot=rot),
        Waypoint(1.0, pressed + 0.05 * back, 0.0, "back off", rot=rot),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan(hold_s: float) -> list[Waypoint]:
    """Pick the bracket up by its motor holder, hold it up while the servo goes in, set it down where it stood, let
    go, go home."""
    pick, rot = _left_on_bracket(BRACKET_POS0)
    hold, _ = _left_on_bracket(HOLD_POS)
    back = -rot[:, 2]
    return [
        Waypoint(2.5, pick + LEFT_BACK_OFF * back, FINGER_OPEN, "behind the bracket", rot=rot),
        Waypoint(1.5, pick, FINGER_OPEN, "around its holder", rot=rot),
        Waypoint(0.8, pick, BRACKET_HOLD, "grip it", rot=rot),
        Waypoint(2.0, hold, BRACKET_HOLD, "lift it", rot=rot),
        Waypoint(hold_s, hold, BRACKET_HOLD, "hold it up", rot=rot),
        Waypoint(2.0, pick + (0, 0, 0.001), BRACKET_HOLD, "set it down", rot=rot),
        Waypoint(0.8, pick + (0, 0, 0.001), FINGER_OPEN, "let go", rot=rot),
        Waypoint(1.2, pick + LEFT_BACK_OFF * back, FINGER_OPEN, "back off", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(1.5, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


LEFT_HELD_S = 2.5 + 1.5 + 0.8 + 2.0  # the left arm holds the bracket up from here


def demo():
    """The left arm picks the bracket up and holds it; the right arm puts the servo in it and goes home; the left arm
    sets the bracket down and goes home."""
    right = right_plan()
    released = sum(w.duration for w in right[:-1])  # the right hand has pressed the servo home and backed off
    return {"left": left_plan(hold_s=released - LEFT_HELD_S), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
