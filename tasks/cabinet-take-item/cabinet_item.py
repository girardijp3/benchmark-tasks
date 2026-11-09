"""On the station table: a small bench-top cabinet of birch plywood (220 x 250 x 356 mm) with its door shut, held by a
catch, facing the robot. Its door is hinged on the left (the robot's left) and has a vertical bar handle near its right
edge. Inside, a handheld multimeter in a yellow rubber boot stands on its end. The left arm opens the door by its
handle and holds it open; the right arm takes the multimeter out and stands it on the bench beside the cabinet, and
goes back; the left arm pushes the door shut until the catch holds it, lets go and goes back.

The cabinet is one articulation from ``assets/`` (the carcass, the door on its damped hinge, the catch body fixed under
the top panel: a strike rod on the door rides under the catch's lip rod, a compliant contact, and once past it holds
the door shut against the hinge's stop). The multimeter is its own rigid body.

Run it:  inspect-robots run --embodiment yam_arena -E task=cabinet_item.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations and ``Waypoint.pivot`` moves the gripper
along an arc about a vertical axis, the door's hinge (``yam_arena.demo.ArmPlan.target``).
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

# -- the cabinet (its own frame: origin at the middle of the carcass's front edge, on the table; +x into the cabinet,
# +y to the left seen from the front, z up). As in assets/bench_cabinet.usda. --------------------------------------
CAB_W, CAB_D, CAB_H = 0.220, 0.250, 0.356  # outside, with its 6 mm feet
FLOOR = 0.018  # the inside floor's top
SHELF = 0.240  # the shelf's underside
DOOR_T = 0.015  # the door's thickness; shut, it lies in front of the carcass (x -0.015..0)
HINGE_XY = np.array([-0.015, 0.110])  # the hinge axis, at the door's outer face on its left edge
HANDLE_IN_DOOR = np.array([-0.029, -0.187, 0.181])  # the bar's middle in the door's frame (origin on the hinge axis)
BAR_R, BAR_LEN = 0.006, 0.168
# The catch holds the door shut below this angle; the strike passes the catch's lip at 1.1 deg and leaves it at 2.5 deg.
CAUGHT = math.radians(0.8)

# -- the multimeter (its own frame: origin at the middle of its bottom end; x across its width, y through its thickness,
# display on +y, z up its length). 186 x 89 x 49 mm, 400 g. ----------------------------------------------------------
MM_W, MM_T, MM_H = 0.089, 0.049, 0.186
MM_IN_CABINET = np.array([0.030 + MM_W / 2, 0.0, FLOOR])  # standing on its end, 30 mm in, display to the left

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
CABINET_XY = (0.40, -0.03)  # the carcass's front edge, middle
CABINET_POS = (*CABINET_XY, TOP + 0.0002)
METER_POS = tuple(float(v) for v in np.array(CABINET_POS) + MM_IN_CABINET + (0.0, 0.0, 0.0003))
PLACE_XY = (0.40, -0.25)  # where the right arm stands the multimeter (its middle): right of the cabinet, clear of the door


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


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


# Arena places an articulation by its root link: the cabinet's is its carcass, at the cabinet's origin.
CABINET = UsdPart("cabinet", asset="cabinet", xy=CABINET_XY, extent=0.25, mass=4.08,
                  usd_path=str(ASSETS / "bench_cabinet.usda"), pos=CABINET_POS, articulation=True)
METER = UsdPart("multimeter", asset="multimeter", xy=tuple(METER_POS[:2]), extent=0.09, mass=0.40,
                usd_path=str(ASSETS / "multimeter.usda"), pos=METER_POS)
OBJECTS = [CABINET, METER]

SOLVER_ITERATIONS = 32
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

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
    """One body after a control step (world frame, m)."""

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
        """Each object's root (an articulation's bounds are left at its root), plus the grippers, the episode time,
        the door's body and its hinge angle."""
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
        out["door"] = _link_pose(base, origin, "cabinet", "Door")
        cab = base.scene["cabinet"]
        i = cab.find_joints(["DoorHinge"])[0][0]
        out["door_angle"] = float(cab.data.joint_pos.torch[0, i].detach().cpu())  # rad, + opens
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
OPENED = math.radians(45)  # the door opened at least this far by a gripper holding its handle
ON_BENCH = 0.004  # m: the multimeter's lowest point within this of the table top
IN_PLACE = 0.02  # m: the cabinet within this of where it stood, and
IN_PLACE_YAW = math.radians(5)  # ... turned less than this
UPRIGHT = math.radians(20)  # the cabinet tipped further than this is a violation
FALLEN_Z = -0.02  # m above the table: a part whose origin is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
METER_BOX = (np.array([-MM_W / 2, -MM_T / 2, 0.0]), np.array([MM_W / 2, MM_T / 2, MM_H]))
BAR_BOX = (HANDLE_IN_DOOR - (BAR_R, BAR_R, BAR_LEN / 2), HANDLE_IN_DOOR + (BAR_R, BAR_R, BAR_LEN / 2))
CARCASS_BOX = (np.array([0.0, -CAB_W / 2, 0.0]), np.array([CAB_D, CAB_W / 2, CAB_H]))
STEPS = ("door opened by its handle", "multimeter taken out", "multimeter on the bench", "door shut and caught")
POINTS = 0.25  # per step
TRACE_S = 0.0  # development: print the parts' state this often (s); 0 = off


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


def handle_held(objects) -> bool:
    return _held_by_any(objects, _box(objects["door"], BAR_BOX))


def meter_held(objects) -> bool:
    return _held_by_any(objects, _box(objects["multimeter"], METER_BOX))


def meter_out(objects) -> bool:
    """The multimeter's box lies wholly outside the carcass's (in the cabinet's frame, as the cabinet stands now)."""
    cab = objects["cabinet"]
    r = quat_xyzw_to_matrix(cab.quat_xyzw)
    m = objects["multimeter"]
    lo, hi = METER_BOX
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    in_cab = (corners @ quat_xyzw_to_matrix(m.quat_xyzw).T + m.pos - cab.pos) @ r
    clo, chi = CARCASS_BOX
    return bool(np.any(in_cab.max(0) < clo) or np.any(in_cab.min(0) > chi))


def meter_on_bench(objects) -> bool:
    """Out of the cabinet, resting on the table (its lowest point at the table top), and no gripper holds it."""
    low = _box(objects["multimeter"], METER_BOX)[0][2]
    return bool(meter_out(objects) and abs(low - TOP) < ON_BENCH and not meter_held(objects))


def cabinet_in_place(objects) -> bool:
    cab = objects["cabinet"]
    x = quat_xyzw_to_matrix(cab.quat_xyzw)[:, 0]
    turn = abs(math.atan2(x[1], x[0]))
    return bool(np.linalg.norm(cab.pos[:2] - np.array(CABINET_XY)) < IN_PLACE and turn < IN_PLACE_YAW)


def door_caught(objects) -> bool:
    """The door shut (the catch holds it) and nobody holds its handle."""
    return bool(objects["door_angle"] < CAUGHT and not handle_held(objects))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: the door opened by its handle, the multimeter taken out in a gripper.
    Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.opened = False
        self.taken_out = False
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
        self.opened |= bool(objects["door_angle"] >= OPENED and handle_held(objects))
        self.taken_out |= bool(meter_out(objects) and meter_held(objects))


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: 1 and 2 once reached; 3 and 4 as things stand. Step 4 counts only once the multimeter has
    been taken out, with the cabinet where it stood."""
    p = _PROGRESS
    p.update(objects)
    done = [p.opened, p.taken_out, meter_on_bench(objects),
            bool(p.taken_out and door_caught(objects) and cabinet_in_place(objects))]
    if TRACE_S and "episode_time" in objects and objects["episode_time"] >= p.next_trace:  # development trace
        p.next_trace = objects["episode_time"] + TRACE_S
        m, c = objects["multimeter"], objects["cabinet"]
        g = {k: objects[f"{k}_gripper"] for k in ("left", "right") if f"{k}_gripper" in objects}
        print(f"[cabinet_item] t={objects['episode_time']:.2f}s trace door {math.degrees(objects['door_angle']):.2f} "
              f"meter {np.round(m.pos - (0, 0, TOP), 4)} v {np.linalg.norm(m.lin_vel):.4f} w "
              f"{np.linalg.norm(m.ang_vel):.3f} cab {np.round(c.pos - (0, 0, TOP), 4)} v {np.linalg.norm(c.lin_vel):.4f} "
              + " ".join(f"{k} f {v.contact:.1f} open {v.opening * 1000:.1f}" for k, v in g.items())
              + f" | meter tilt {math.degrees(math.acos(min(1.0, quat_xyzw_to_matrix(m.quat_xyzw)[2, 2]))):.1f}"
              + "".join(f" {k} tcp {np.round(v.back + PAD_BACK * (v.tips - v.back) / np.linalg.norm(v.tips - v.back) - (0, 0, TOP), 3)}"
                        f" pitch {math.degrees(math.asin(-(v.tips - v.back)[2] / np.linalg.norm(v.tips - v.back))):.1f}"
                        for k, v in g.items()), flush=True)
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        m = objects["multimeter"].pos
        print(f"[cabinet_item] t={objects['episode_time']:.2f}s  steps done: {got}  (door "
              f"{math.degrees(objects['door_angle']):.1f} deg; multimeter at ({m[0]:.3f}, {m[1]:.3f}, "
              f"{m[2] - TOP:.3f}); cabinet moved {np.linalg.norm(objects['cabinet'].pos[:2] - CABINET_XY) * 1000:.1f}"
              f" mm)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the cabinet or the multimeter off the table, the cabinet tipped over, or a state that is not finite."""
    for name in ("cabinet", "multimeter"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    up = float(quat_xyzw_to_matrix(objects["cabinet"].quat_xyzw)[2, 2])
    return bool(up < math.cos(UPRIGHT) or not math.isfinite(objects["door_angle"]))


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the cabinet and the multimeter
    still, and no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["cabinet"].resting() and objects["multimeter"].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: grips the bar handle from the front, pointing level at the door, and pulls the door open along its arc (the
# gripper turns with the door), holds it open while the right arm works, pushes it back along the arc and a little past
# shut (the strike rides under the catch's lip), lets go, backs off and goes home.
# Right arm: waits at home until the door is open, reaches in level, fingers closing across the multimeter's thickness
# near its top, lifts it a little, draws it straight out, carries it right, past the cabinet's corner, and stands it on
# the bench beside the cabinet, lets go, rises off it and goes home. The arm sags about 20 mm under the 400 g meter (its
# wrist motors are soft), so lifts are commanded higher than needed.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width
HANDLE_OPEN = -0.017  # fingers open to 34 mm around the 12 mm bar (wide open, they would reach the door's face)
HANDLE_HOLD = -(2 * BAR_R - SQUEEZE) / 2  # ~20 N a finger
METER_OPEN = -0.036  # 72 mm around the 49 mm multimeter
METER_HOLD = -(MM_T - 1.5 * SQUEEZE) / 2  # ~30 N a finger
HANDLE_YAW = math.radians(-10)  # the left gripper points this far clockwise (seen from above) of straight at the door
HANDLE_ROLL = 1  # +1 or -1: which way round the gripper is about its pointing axis
METER_ROLL = 1
DOOR_OPEN = math.radians(80)  # the left arm opens the door this far and holds it there
PRESS = math.radians(0.5)  # ... and at the end pushes it this far past shut, into the hinge's stop
METER_GRIP_Z = 0.150  # the multimeter is gripped this high up its length (the pads span +-7.75 mm), across the LCD
METER_GRIP_IN = 0.028  # the TCP this far behind its front face (the pads reach 25 mm behind the TCP)
METER_LIFT = 0.035  # commanded: the arm sags about 20 mm under the 400 g meter, so it rises about 10 mm
APPROACH = 0.06  # m: grips are approached from this far back along the pointing axis
_Z = np.array([0.0, 0.0, 1.0])


@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None  # a full gripper (TCP) rotation; replaces ``yaw`` when set
    pivot: tuple[float, float] | None = None  # move along an arc about this vertical axis (x, y), not a straight line


def _target(self, t: float):
    """``ArmPlan.target`` that also follows a waypoint's ``rot`` and ``pivot``."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            pivot = getattr(w, "pivot", None)
            if pivot is None:
                pos = prev_pos + s * (goal - prev_pos)
            else:  # radius, angle and height about the pivot, each interpolated
                c = np.asarray(pivot, float)
                a0, a1 = prev_pos[:2] - c, goal[:2] - c
                r0, r1 = np.linalg.norm(a0), np.linalg.norm(a1)
                t0 = math.atan2(a0[1], a0[0])
                dt = (math.atan2(a1[1], a1[0]) - t0 + math.pi) % (2 * math.pi) - math.pi
                ang, r = t0 + s * dt, r0 + s * (r1 - r0)
                pos = np.array([c[0] + r * math.cos(ang), c[1] + r * math.sin(ang),
                                prev_pos[2] + s * (goal[2] - prev_pos[2])])
            return pos, ik.slerp_matrix(prev_rot, rot, s), prev_finger + s * (w.finger - prev_finger), w.label
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target


def _level_rot(point: np.ndarray, roll: int) -> np.ndarray:
    """A gripper pointing level along ``point`` (horizontal), its fingers closing level across it, pads' width
    vertical. Gripper frame: x = closing, y = pad width, z = pointing."""
    p = np.asarray(point, float) / np.linalg.norm(point)
    close = np.cross(p, _Z) * roll
    return np.column_stack([close, np.cross(p, close), p])


def door_hinge_world() -> np.ndarray:
    return np.array(CABINET_XY) + HINGE_XY


def _door_point(p_door, angle: float) -> np.ndarray:
    """A point given in the door's frame (origin on the hinge axis at table level), world, the door open ``angle``
    (rad; opening turns the door clockwise seen from above)."""
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = p_door
    h = door_hinge_world()
    return np.array([h[0] + c * x + s * y, h[1] - s * x + c * y, CABINET_POS[2] + z])


def handle_grip(angle: float) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP (on the bar's axis, at its middle) and rotation holding the handle of the door open ``angle``."""
    point = _rz(HANDLE_YAW - angle) @ np.array([1.0, 0.0, 0.0])
    return _door_point(HANDLE_IN_DOOR, angle), _level_rot(point, HANDLE_ROLL)


METER_ROT = _level_rot(np.array([1.0, 0.0, 0.0]), METER_ROLL)  # the right gripper pointing straight in


def left_plan(hold_s: float) -> list[Waypoint]:
    """Open the door by its handle, hold it open ``hold_s``, push it shut past the catch, let go, go home."""
    shut, rot0 = handle_grip(0.0)
    opened, rot_open = handle_grip(DOOR_OPEN)
    pressed, rot_pressed = handle_grip(-PRESS)
    back = -APPROACH * rot0[:, 2]
    hinge = tuple(door_hinge_world())
    return [
        Waypoint(3.0, shut + back, HANDLE_OPEN, "in front of the handle", rot=rot0),
        Waypoint(1.5, shut, HANDLE_OPEN, "around the bar", rot=rot0),
        Waypoint(0.8, shut, HANDLE_HOLD, "grip the handle", rot=rot0),
        Waypoint(4.5, opened, HANDLE_HOLD, "pull the door open", rot=rot_open, pivot=hinge),
        Waypoint(hold_s, opened, HANDLE_HOLD, "hold it open", rot=rot_open),
        Waypoint(4.5, shut, HANDLE_HOLD, "push it shut", rot=rot0, pivot=hinge),
        Waypoint(0.8, pressed, HANDLE_HOLD, "press it past the catch", rot=rot_pressed),
        Waypoint(0.5, pressed, HANDLE_HOLD, "hold", rot=rot_pressed),
        Waypoint(0.8, pressed, HANDLE_OPEN, "let go", rot=rot_pressed),
        Waypoint(1.0, pressed + back, HANDLE_OPEN, "back off", rot=rot0),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(1.5, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


LEFT_OPEN_S = 3.0 + 1.5 + 0.8 + 4.5  # the door is open


def right_plan() -> list[Waypoint]:
    """Wait for the door, take the multimeter out, stand it on the bench, go home."""
    meter = np.array(METER_POS)
    front = meter[0] - MM_W / 2
    grip = np.array([front + METER_GRIP_IN, meter[1], meter[2] + METER_GRIP_Z])
    lifted = grip + (0, 0, METER_LIFT)
    out = np.array([CABINET_XY[0] - 0.075, grip[1], lifted[2]])  # its back face just out of the carcass
    place = np.array([PLACE_XY[0] - MM_W / 2 + METER_GRIP_IN, PLACE_XY[1], TOP + 0.001 + METER_GRIP_Z])
    return [
        Waypoint(LEFT_OPEN_S - 2.5, _START_TCP["right"], FINGER_OPEN, "wait for the door", rot=_START_ROT["right"]),
        Waypoint(3.5, grip - (APPROACH + 0.05, 0, 0), METER_OPEN, "in front of the multimeter", rot=METER_ROT),
        Waypoint(1.5, grip, METER_OPEN, "around the multimeter", rot=METER_ROT),
        Waypoint(0.8, grip, METER_HOLD, "grip it", rot=METER_ROT),
        Waypoint(0.8, lifted, METER_HOLD, "lift it", rot=METER_ROT),
        Waypoint(2.0, out, METER_HOLD, "draw it out", rot=METER_ROT),
        Waypoint(2.0, np.array([out[0], place[1], place[2] + 0.04]), METER_HOLD, "beside the cabinet", rot=METER_ROT),
        Waypoint(1.0, place + (0, 0, 0.04), METER_HOLD, "over the bench", rot=METER_ROT),
        Waypoint(1.2, place, METER_HOLD, "stand it down", rot=METER_ROT),
        Waypoint(0.8, place, METER_OPEN, "let go", rot=METER_ROT),
        Waypoint(1.0, place + (0, 0, 0.06), METER_OPEN, "up off it", rot=METER_ROT),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """Both arms: the left opens the door and holds it while the right takes the multimeter out and stands it on the
    bench; the left closes the door once the right arm has drawn back. Both go home."""
    right = right_plan()
    clear = sum(w.duration for w in right) - 2.5 - 1.0  # the right arm has let go and backed off
    return {"left": left_plan(hold_s=clear - LEFT_OPEN_S), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
