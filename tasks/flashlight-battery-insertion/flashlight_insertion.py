"""On the station table: an 18650 flashlight (110 mm long, black anodised aluminium) standing head-down on its bezel
midway between the arms, its tail open and its tube empty; beside it, on the right arm's side, its screw-on tail cap
lying open end down and an 18650 lithium-ion cell (18 x 65 mm) lying on its side, positive end pointing away from
the robot. The left arm holds the flashlight body; the right arm puts the cell into the tube positive end first
(toward the LED), then screws the tail cap back on (clockwise seen from above, about one turn).

Scoring: 0.25 for each of ``STEPS`` (battery picked up, battery in positive end first, cap on the thread, cap screwed
down); 0 with the cell in the wrong way round, a part off the table, or the cap forced down without turning. The
grippers' state comes from the simulator (``GripperState``), and the cap's turn on the thread from the parts'
orientations.

The flashlight is three rigid bodies (body, tail cap, battery) from ``assets/``; the cap's 4-start thread collides as
modelled, and the cell has 0.4 mm of radial play in the bore.

Run it:  inspect-robots run --embodiment yam_arena -E task=flashlight_insertion.py --scorer task_score ...

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
from yam_arena.task_file import TABLE_TOP, Asset
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the flashlight (its own frame: tail at z = 0, head at z = 0.110, axis z) -----------------------------------------
LENGTH = 0.110
LEAD = 0.010  # cap travel per turn: 4-start thread, 2.5 mm pitch
CAP_HEIGHT = 0.0208
CAP_DIAMETER = 0.026
TUBE_DIAMETER = 0.022
TUBE_MOUTH = 0.006  # the tube's open end, in the flashlight frame
CELL = (0.0080, 0.0745)  # the cell's flat negative end and its positive nub, in the flashlight frame (assembled)
CELL_DIAMETER = 0.018
CELL_MID = sum(CELL) / 2

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
FLASH_XY = (0.30, -0.02)  # the flashlight's axis
CAP_XY = (0.22, -0.20)  # the cap, open end down
CELL_XY = (0.38, -0.20)  # the cell's middle, lying along x, nub toward +x
# Head down: the flashlight's frame turned 180 deg about x, so a point at height h in it is at world z = Z0 - h.
Z0 = TOP + LENGTH + 0.0005
# The left arm lifts the flashlight, carries it toward the right arm's side (so the right arm works from its own side
# instead of crossing into the left arm's space) and stands it back on the table there, still holding it: standing
# on its head it is exactly upright and the table takes the push of the cell and the cap.
WORK_XY = (0.27, -0.07)
WORK_LIFT = 0.03  # carried this high
ZW = Z0  # the flashlight's frame height at the work place: standing on the table again
HEAD_DOWN_XYZW = (1.0, 0.0, 0.0, 0.0)
# Lying: the frame turned 90 deg about y, so its z axis (toward the nub) points along +x.
LYING_XYZW = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
CELL_Z = TOP + CELL_DIAMETER / 2 + 0.0005  # the lying cell's axis


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned as one rigid body at a full pose, with the physics authored in it. ``Asset``'s fields only
    satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


def _part(name: str, usd: str, mass: float, pos, rot) -> UsdPart:
    return UsdPart(name, asset=name, xy=tuple(pos[:2]), extent=0.034, mass=mass, usd_path=str(ASSETS / usd),
                   pos=tuple(pos), rot_xyzw=rot)


BODY = _part("flashlight_body", "flashlight_body.usda", 0.070, (*FLASH_XY, Z0), HEAD_DOWN_XYZW)
CAP = _part("flashlight_cap", "flashlight_cap.usda", 0.018, (*CAP_XY, TOP + CAP_HEIGHT + 0.0005), HEAD_DOWN_XYZW)
# The lying frame's origin sits CELL_MID behind the cell's middle along -x.
BATTERY = _part("flashlight_battery", "flashlight_battery.usda", 0.047,
                (CELL_XY[0] - CELL_MID, CELL_XY[1], CELL_Z), LYING_XYZW)
OBJECTS = [BODY, CAP, BATTERY]


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
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
    stock, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """The stock object states, plus the grippers and the episode time for this file's scoring."""
        return {**stock_states(env, names), **_gripper_states(env)}

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
# Positions along the flashlight's axis are relative to the body's centre, toward the head. Assembled, the cell's centre
# is 11 mm and the cap's 47 mm on the tail side of it; the cell's flat end is then 2 mm inside the tube's rim.
CELL_AT, CAP_AT = -0.011, -0.047
CELL_OFF_AXIS = 0.003  # m: the cell's centre within this of the tube's axis: it is in the bore
CELL_UPRIGHT = math.cos(math.radians(10))  # the cell's axis within 10 deg of the tube's
PICKED_UP = 0.02  # m: a gripper holds the cell with its lowest point this high above the table
CAP_OFF_AXIS = 0.0015  # m: the cap's centre within this of the tube's axis: it sits on the thread
CAP_NEAR = 0.0175  # m: the cap within this of its seated place along the axis (it meets the thread 12.5 mm up)
THREADED_TURN = math.radians(90)  # step 3: turned this far onto the thread (it finds a start within 90 deg)
SCREWED_TURN = math.radians(300)  # step 4: turned this far, and
CAP_SEATED = 0.001  # ... within this of its seated place: tight against the collar, no gap over 1 mm
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something
# with at least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
STEPS = ("battery picked up", "battery in, positive end first", "cap on the thread", "cap screwed down")
POINTS = 0.2  # per step; only success scores 1.0
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)  # settled: within 2 mm and 2 degrees over 1 s


def _axis(state) -> np.ndarray:
    """A part's flashlight-frame z axis (tail to head; for the cell, toward its nub) in the world."""
    return quat_xyzw_to_matrix(state.quat_xyzw)[:, 2]


def _along(objects, name: str):
    """``name``'s centre relative to the body's: along the body's axis (toward the head) and off it (m)."""
    body = objects["flashlight_body"]
    axis = _axis(body)
    d = objects[name].pos - body.pos
    s = float(d @ axis)
    return s, float(np.linalg.norm(d - s * axis))


def _held(part, gripper) -> bool:
    """``gripper`` holds ``part``: its pads reach the part's bounding box and both fingers press."""
    if gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = part.low - GRIP_SLACK, part.high + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def _in_bore(objects) -> bool:
    s, off = _along(objects, "flashlight_battery")
    return off < CELL_OFF_AXIS and CELL_AT - 0.03 < s < CELL_AT + 0.03


def battery_in(objects) -> bool:
    """The cell is fully in the tube (its flat end no higher than the rim), positive nub toward the head."""
    s, _ = _along(objects, "flashlight_battery")
    nub_to_head = float(_axis(objects["flashlight_battery"]) @ _axis(objects["flashlight_body"]))
    return bool(_in_bore(objects) and nub_to_head > CELL_UPRIGHT and s > CELL_AT - 0.002)


def battery_reversed(objects) -> bool:
    """The cell is in the tube the wrong way round: positive nub toward the tail."""
    nub_to_head = float(_axis(objects["flashlight_battery"]) @ _axis(objects["flashlight_body"]))
    return bool(_in_bore(objects) and nub_to_head < -CELL_UPRIGHT)


def _cap_near(objects) -> bool:
    s, off = _along(objects, "flashlight_cap")
    return off < CAP_OFF_AXIS and abs(s - CAP_AT) < CAP_NEAR


def _cap_seated(objects) -> bool:
    s, off = _along(objects, "flashlight_cap")
    return off < CAP_OFF_AXIS and abs(s - CAP_AT) < CAP_SEATED


class _Progress:
    """What has happened so far this episode: whether a gripper has picked the cell up, and how far the cap has turned
    on the tube's end (rad, tightening positive; back to 0 when the cap leaves it). Starts over when the episode time
    goes back."""

    def __init__(self):
        self.time, self.twist, self.last = -1.0, 0.0, None
        self.picked = False
        self.reported = [False] * len(STEPS)
        self.poses = []  # (time, {part: (pos, quat_xyzw)}) over the last STILL_S seconds

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
            self.poses.append((t, {o.name: (objects[o.name].pos.copy(), objects[o.name].quat_xyzw.copy())
                                   for o in OBJECTS}))
            while self.poses[0][0] < t - STILL_S - 1e-6:
                self.poses.pop(0)
        cell = objects["flashlight_battery"]
        grippers = [g for g in (objects.get("left_gripper"), objects.get("right_gripper")) if g is not None]
        self.picked |= cell.low[2] > TOP + PICKED_UP and any(_held(cell, g) for g in grippers)
        s, off = _along(objects, "flashlight_cap")
        if abs(s - CAP_AT) > 0.03 or off > 0.005:  # off the tube's end: start counting again when it is back
            self.twist, self.last = 0.0, None
            return
        rel = quat_xyzw_to_matrix(objects["flashlight_body"].quat_xyzw).T @ quat_xyzw_to_matrix(
            objects["flashlight_cap"].quat_xyzw)
        angle = math.atan2(rel[1, 0], rel[0, 0])
        if self.last is not None:
            self.twist += (angle - self.last + math.pi) % (2 * math.pi) - math.pi
        self.last = angle


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached; 2 to 4 as things stand (3 and 4 only over a cell put in right)."""
    p = _PROGRESS
    p.update(objects)
    cell = battery_in(objects)
    done = [
        bool(p.picked),
        cell,
        bool(cell and _cap_near(objects) and p.twist >= THREADED_TURN),
        bool(cell and _cap_seated(objects) and p.twist >= SCREWED_TURN),
    ]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, d in zip(STEPS, done, strict=True) if d) or "none"
        print(f"[flashlight_insertion] t={objects['episode_time']:.2f}s  steps done: {got}  "
              f"(cap turned {math.degrees(p.twist):.0f} deg)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: a part off the table, the cell in the wrong way round, or the cap forced down without turning."""
    if any(objects[o.name].pos[2] < TOP + FALLEN_Z or not np.all(np.isfinite(objects[o.name].pos)) for o in OBJECTS):
        return True
    return battery_reversed(objects) or (_cap_seated(objects) and _PROGRESS.twist < SCREWED_TURN)


def settled(objects) -> bool:
    """The three parts have stayed in place for the last ``STILL_S``: each within ``STILL_MOVE`` m and ``STILL_TURN``
    rad of where it was. Judged from the poses, not the simulator's instantaneous speeds: a cap screwed tight keeps a
    contact jitter (tens of mm/s, a few rad/s) while it stays within 0.2 mm and 0.5 degrees."""
    p = _PROGRESS
    p.update(objects)
    if not p.poses or p.poses[-1][0] - p.poses[0][0] < STILL_S - 1e-6:
        return False
    for o in OBJECTS:
        pos = np.array([poses[o.name][0] for _, poses in p.poses])
        if np.linalg.norm(pos.max(axis=0) - pos.min(axis=0)) > STILL_MOVE:
            return False
        q0 = p.poses[0][1][o.name][1]
        turns = [2 * math.acos(min(1.0, abs(float(np.dot(q0, poses[o.name][1]))))) for _, poses in p.poses]
        if max(turns) > STILL_TURN:
            return False
    return True


def success(objects) -> bool:
    """All four steps done, the three parts settled (``settled``), and no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and settled(objects))


def score(objects) -> float:
    """1.0 on success; otherwise 0.2 per step done (``STEPS``, at most 0.8); 0 for any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: a sideways grip on the body tube, held throughout. Right arm: pick the cell up from above, turn it nub-down
# and lower it into the tube until it is guided, let it drop in; then pick the cap up from above, set it on the thread
# and screw it down clockwise in grips of 200 deg, sinking with the thread.
LEFT_APPROACH = -math.pi / 2  # the left gripper points along -y, from its own side
GRIP_ON_BODY = 0.050  # left grip along the tube, from the tail: clear of the thread and the collar
BODY_GRIP_REACH = 0.035  # the left pad tips 3.5 cm past the body's axis
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width, pressing ~20 N each
BODY_HOLD = -(TUBE_DIAMETER - SQUEEZE) / 2
BODY_APPROACH = -0.050 / 2
CELL_HOLD = -(CELL_DIAMETER - SQUEEZE) / 2
CAP_HOLD = -(CAP_DIAMETER - SQUEEZE) / 2
CAP_GRIP_DEPTH = 0.018  # the right pad tips 1.8 cm down the cap's side
CELL_GRIP_DEPTH = 0.009  # the pad tips this far past the cell's axis: the whole pads on it, the tips ~2.5 mm off the table
# One 90 deg tip turns the cell nub-down. The gripper takes it from above, fingers across it, tilted GRASP_TILT about
# the finger axis (leaning toward the nub), so after the tip it points a little downward and the arm's other joints,
# not the wrist roll, do the turning; the tipped cell is then swung to TIP_HEADING about the vertical.
GRASP_TILT = math.radians(-50)
TIP_HEADING = math.radians(270)  # the tipped gripper points +y, from the right arm's side
CELL_REACH = PAD_TIP_PAST_TCP - CELL_GRIP_DEPTH  # the cell's axis sits this far out along the gripper's pointing axis
# Measured in simulation: closing on the lying cell rolls it toward the palm, so once tipped its middle sits ~5.5 mm
# further along +y (and ~0.6 mm along -x) than the nominal grasp puts it, nub to tube mouth. The inserting moves shift
# the hand to cancel it (the bore leaves 0.4 mm of play).
CELL_IN_HAND_FIX = np.array([0.00065, -0.0055, 0.0])
CELL_HOVER = 0.005  # the nub this far above the tube's mouth before going in
CELL_INSERT = 0.020  # the cell this far into the bore when the fingers let go
CAP_ENGAGE = 0.0125  # the cap this far above its seated place when its thread first meets the tube's
CAP_PRESS = 0.001  # pressed this far past touching, so the thread can catch
# Twists of 200 deg (the most the wrist turns in one grip), then a short one that seats the cap: about 470 deg in all,
# a little past where the cap meets the collar. Never past the seated height: pressing a tight cap further down or on
# around makes the thread contacts chatter.
TWIST_ANGLES = (math.radians(200), math.radians(200), math.radians(100))
SCREW_START = math.radians(130)  # yaw at the start of each clockwise twist (right joint6 ~ 44 deg - yaw)
TWIST_STEPS = 2


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

_DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
_X, _Y, _Z = np.eye(3)


def side_grip(approach: float, roll: float) -> np.ndarray:
    """The gripper pointing horizontally along ``approach`` (rad from +x), fingers closing horizontally at roll 0,
    turned ``roll`` about its pointing axis."""
    forward = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN  # pointing +x, fingers closing along y
    return axis_angle_matrix(_Z, approach) @ axis_angle_matrix(_X, roll) @ forward


def _tcp_for_pads(target_xy, approach: float, reach: float):
    """The TCP's x, y that put the pad tips ``reach`` past ``target_xy`` along the pointing direction."""
    d = np.array([math.cos(approach), math.sin(approach)])
    return tuple(np.asarray(target_xy) + d * (reach - PAD_TIP_PAST_TCP))


def cell_grips() -> tuple[np.ndarray, np.ndarray]:
    """The grip picking the lying cell (nub along +x) and the same grip tipped 90 deg about the finger axis (nub
    down) and swung to ``TIP_HEADING``."""
    pick = axis_angle_matrix(_Y, GRASP_TILT) @ yam_demo.grip_rotation(0.0)
    upright = axis_angle_matrix(_Z, TIP_HEADING) @ axis_angle_matrix(_Y, math.pi / 2) @ pick
    return pick, upright


def _tcp_holding(cell_mid, rot: np.ndarray) -> np.ndarray:
    """The TCP that holds the cell's middle at ``cell_mid`` with the gripper at ``rot``."""
    return np.asarray(cell_mid, float) - CELL_REACH * (rot @ _Z)


def _turn_steps(a: np.ndarray, b: np.ndarray, n: int) -> list[np.ndarray]:
    """``n`` rotations from ``a`` to ``b`` (exclusive of ``a``), so each step stays well under 180 deg."""
    return [ik.slerp_matrix(a, b, (k + 1) / n) for k in range(n)]


LEFT_GRIP_S = 2.5 + 2.0 + 0.8  # the left arm's approach and grip
LEFT_READY_S = LEFT_GRIP_S + 1.2 + 2.0 + 1.0  # ... then lifting the flashlight and carrying it to the work place


def right_plan() -> list[Waypoint]:
    """Wait for the left grip; pick the cell up, tip it nub-down and put it in; screw the cap on."""
    pick, upright = cell_grips()
    lift_mid = np.array([*CELL_XY, TOP + 0.12])
    mouth_z = ZW - TUBE_MOUTH
    half = (CELL[1] - CELL[0]) / 2
    hover_mid = np.array([*WORK_XY, mouth_z + CELL_HOVER + half])
    insert_mid = np.array([*WORK_XY, mouth_z - CELL_INSERT + half])
    at_cell = _tcp_holding((*CELL_XY, CELL_Z), pick)
    plan = [
        Waypoint(LEFT_GRIP_S, at_cell - 0.05 * (pick @ _Z), FINGER_OPEN, "over the cell", rot=pick),
        Waypoint(1.5, at_cell, FINGER_OPEN, "down to the cell", rot=pick),
        Waypoint(0.6, at_cell, CELL_HOLD, "grip the cell", rot=pick),
        Waypoint(1.5, _tcp_holding(lift_mid, pick), CELL_HOLD, "lift the cell", rot=pick),
    ]
    for k, rot in enumerate(_turn_steps(pick, upright, 3)):  # about the cell's middle
        plan.append(Waypoint(1.2, _tcp_holding(lift_mid, rot), CELL_HOLD, f"tip it nub-down {k + 1}", rot=rot))
    in_bore = _tcp_holding(insert_mid, upright) + CELL_IN_HAND_FIX
    back = in_bore + (0.0, 0.0, 0.07)  # open wide and lift straight up off the cell, now guided by the bore
    plan += [
        Waypoint(2.5, _tcp_holding(hover_mid + (0, 0, 0.03), upright) + CELL_IN_HAND_FIX, CELL_HOLD, "over the tube",
                 rot=upright),
        Waypoint(1.5, _tcp_holding(hover_mid, upright) + CELL_IN_HAND_FIX, CELL_HOLD, "line up on the bore",
                 rot=upright),
        Waypoint(1.0, _tcp_holding(hover_mid, upright) + CELL_IN_HAND_FIX, CELL_HOLD, "settle", rot=upright),
        Waypoint(3.5, in_bore, CELL_HOLD, "into the bore", rot=upright),
        Waypoint(0.6, in_bore, FINGER_OPEN, "let it drop in", rot=upright),
        Waypoint(1.5, back, FINGER_OPEN, "lift off the cell", rot=upright),
    ]
    # The cap: from above, open end down, carried at the twist's start yaw.
    a = SCREW_START
    cap_top = TOP + CAP_HEIGHT + 0.0005
    cap_grip_z = cap_top + PAD_TIP_PAST_TCP - CAP_GRIP_DEPTH
    seated_grip_z = ZW + PAD_TIP_PAST_TCP - CAP_GRIP_DEPTH  # the grip height with the cap screwed down
    x, y = WORK_XY
    z = seated_grip_z + CAP_ENGAGE - CAP_PRESS
    to_cap = (*CAP_XY, cap_grip_z + 0.05)
    # Back the way the cell came (poses known to be in reach): over the cell's spot, un-tip, point straight down, then
    # turn the wrist to the cap's yaw in steps under 90 deg on the way to the cap.
    plan.append(Waypoint(1.5, _tcp_holding(lift_mid, upright), FINGER_OPEN, "back over the cell's spot", rot=upright))
    for k, rot in enumerate(_turn_steps(upright, pick, 3)):
        plan.append(Waypoint(0.8, _tcp_holding(lift_mid, rot), FINGER_OPEN, f"un-tip {k + 1}", rot=rot))
    start = _tcp_holding(lift_mid, pick)
    yaws = np.linspace(0.0, a, 3)
    for k, yaw in enumerate(yaws):
        f = (k + 1) / len(yaws)
        plan.append(Waypoint(1.0, (1 - f) * start + f * np.asarray(to_cap), FINGER_OPEN, "to the cap",
                             rot=yam_demo.grip_rotation(yaw)))
    plan += [
        Waypoint(1.5, (*CAP_XY, cap_grip_z), FINGER_OPEN, "down to the cap", a),
        Waypoint(0.6, (*CAP_XY, cap_grip_z), CAP_HOLD, "grip the cap", a),
        Waypoint(1.5, (*CAP_XY, z + 0.04), CAP_HOLD, "lift the cap", a),
        Waypoint(2.5, (x, y, z + 0.02), CAP_HOLD, "over the tube", a),
        Waypoint(1.5, (x, y, z), CAP_HOLD, "onto the thread", a),
    ]
    floor = seated_grip_z  # never push past the seated cap
    for k, twist in enumerate(TWIST_ANGLES):
        if k:
            plan.append(Waypoint(0.6, (x, y, z), CAP_HOLD, f"grip {k + 1}", a))
        rise = LEAD * twist / (2 * math.pi)
        for i in range(1, TWIST_STEPS + 1):  # clockwise from above: the yaw goes down
            f = i / TWIST_STEPS
            plan.append(Waypoint(2.0 * twist / TWIST_ANGLES[0], (x, y, max(z - rise * f, floor)), CAP_HOLD,
                                 f"screw {k + 1}", a - twist * f))
        z = max(z - rise, floor)
        b = a - twist
        plan += [
            Waypoint(0.5, (x, y, z), FINGER_OPEN, f"let go {k + 1}", b),
            Waypoint(0.6, (x, y, z + 0.03), FINGER_OPEN, f"rise {k + 1}", b),
        ]
        if k < len(TWIST_ANGLES) - 1:
            plan += [
                Waypoint(0.6, (x, y, z + 0.03), FINGER_OPEN, f"turn back {k + 1}", (a + b) / 2),
                Waypoint(0.6, (x, y, z + 0.03), FINGER_OPEN, f"turn back {k + 1}", a),
                Waypoint(0.6, (x, y, z), FINGER_OPEN, f"down {k + 1}", a),
            ]
    plan.append(Waypoint(1.5, (x, y, z + 0.04), FINGER_OPEN, "clear away", b))
    return plan


def left_plan(hold_s: float) -> list[Waypoint]:
    """Grip the body from the side, lift it, carry it to the work place, stand it there and hold it."""
    grip_z = Z0 - GRIP_ON_BODY
    rot = side_grip(LEFT_APPROACH, 0.0)
    at = _tcp_for_pads(FLASH_XY, LEFT_APPROACH, BODY_GRIP_REACH)
    back = _tcp_for_pads(FLASH_XY, LEFT_APPROACH, -0.04)
    work = _tcp_for_pads(WORK_XY, LEFT_APPROACH, BODY_GRIP_REACH)
    return [
        Waypoint(2.5, (*back, grip_z), BODY_APPROACH, "beside the flashlight", rot=rot),
        Waypoint(2.0, (*at, grip_z), BODY_APPROACH, "around the body", rot=rot),
        Waypoint(0.8, (*at, grip_z), BODY_HOLD, "grip the body", rot=rot),
        Waypoint(1.2, (*at, grip_z + WORK_LIFT), BODY_HOLD, "lift it", rot=rot),
        Waypoint(2.0, (*work, grip_z + WORK_LIFT), BODY_HOLD, "toward the right arm", rot=rot),
        Waypoint(1.0, (*work, grip_z), BODY_HOLD, "stand it on the table", rot=rot),
        Waypoint(hold_s, (*work, grip_z), BODY_HOLD, "hold it still", rot=rot),
    ]


def demo():
    """Both arms: the right arm's plan, and the left arm's, which holds the body throughout."""
    right = right_plan()
    return {"left": left_plan(hold_s=sum(w.duration for w in right) - LEFT_READY_S + 3.0), "right": right}
