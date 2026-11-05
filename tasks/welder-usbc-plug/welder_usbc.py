"""On the station table: a power bank (92 x 46 x 14 mm) fixed to the table on the right, its captive USB-C cable
(300 mm) lying straight toward the robot's left with the plug at its end; beside it, a mini 18650 spot welder (124 x
24 x 22 mm housing, black ABS) lying on its feet, its rear USB-C port facing the plug. The left arm picks the welder up
and brings it closer, port toward the right arm; the right arm picks the plug up by its plastic body, pushes it into
the port, lets go and goes back; then the left arm sets the welder down, the cable still plugged in, and goes back.

The power bank, cable and plug are one articulation (24 cable links on bending joints) from ``assets/``; the welder is
another (its housing, lid, latch and the two USB-C spring fingers that press on the plug). The port has real USB-C
clearances (0.10-0.23 mm a side, a 0.4 mm lead-in); README.md says how its colliders were made to hold that fit.

Run it:  inspect-robots run --embodiment yam_arena -E task=welder_usbc.py --scorer task_score ...

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

# -- the parts (their own frames) -------------------------------------------------------------------------------------
# Power bank asset: origin at the bank's bottom centre at its cable end; the cable leaves along +x at z = 7 mm and the
# plug's frame sits on its mating face (the overmold's front), 338 mm out, +x the insertion direction.
PLUG_IN_BANK = np.array([0.338, 0.0, 0.007])
OVERMOLD = (0.022, 0.012, 0.0062)  # length behind the mating face, width, thickness
# The right grip's centre this far behind the mating face: the YAM finger pads are ~28 mm wide, so their front edge ends
# ~0.8 mm behind the face and they never reach the welder's rear face, while gripping the whole 22 mm overmold.
PLUG_GRIP_BEHIND = 0.015
# Welder asset: origin at the rear end, bottom of the housing, centre; +x to the probes; the port is in the rear face.
PORT_IN_WELDER = np.array([0.0, 0.0, 0.011])
PORT_IN_BODY = PORT_IN_WELDER - (0.004, 0.0, 0.0)  # the housing body's own origin sits 4 mm in from the rear face
WELDER_WIDTH, WELDER_HEIGHT = 0.024, 0.022
WELDER_GRIP_AT = 0.062  # along the welder from its rear face: its middle, so it hangs level in the grip
FEET = 0.0006  # the rubber feet reach this far below the housing's bottom
# The 18650 cell in the welder's bay, under the closed lid (its place in the welder asset).
CELL_IN_WELDER = np.array([0.0421, 0.0, 0.011])
CELL_IN_WELDER_XYZW = (-0.5, 0.5, -0.5, 0.5)

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
BANK_XY, CABLE_YAW = (0.27, -0.37), math.pi / 2  # the cable runs along +y, the plug at its end points +y
WELDER_XY, WELDER_YAW = (0.27, 0.07), math.pi / 2  # its rear face at y = 0.07, port facing -y toward the plug
MEET = np.array([0.27, -0.07, TOP + 0.08])  # where the left arm holds the port (its mouth)


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


BANK_POS = (*BANK_XY, TOP + 0.0005)
WELDER_POS = (*WELDER_XY, TOP + FEET)
PLUG_START = np.array(BANK_POS) + _rz(CABLE_YAW) @ PLUG_IN_BANK  # before the cable settles onto the table


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation
    (``fixed_base`` pins its root link where it spawns). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed_base: bool = False


POWERBANK = UsdPart("powerbank", asset="powerbank", xy=BANK_XY, extent=0.1, mass=0.1356,
                    usd_path=str(ASSETS / "powerbank_task.usda"), pos=BANK_POS, rot_xyzw=_yaw_xyzw(CABLE_YAW),
                    articulation=True, fixed_base=True)
WELDER = UsdPart("welder", asset="welder", xy=WELDER_XY, extent=0.14, mass=0.1267,
                 usd_path=str(ASSETS / "welder_task.usda"), pos=WELDER_POS, rot_xyzw=_yaw_xyzw(WELDER_YAW),
                 articulation=True)
CELL = UsdPart("cell", asset="cell", xy=WELDER_XY, extent=0.065, mass=0.047,
               usd_path=str(ASSETS / "battery_18650.usda"),
               pos=tuple(float(v) for v in np.array(WELDER_POS) + _rz(WELDER_YAW) @ CELL_IN_WELDER),
               rot_xyzw=tuple(float(v) for v in _qmul(_yaw_xyzw(WELDER_YAW), CELL_IN_WELDER_XYZW)))
OBJECTS = [POWERBANK, WELDER, CELL]


# The welder housing's collider is a signed distance field of its whole shell; inside the USB-C opening it isn't
# exact to the 0.1 mm the fit needs, so the plug's metal shell meets only the port's own steel pieces (as in a real
# receptacle). The overmold still meets the housing, where it seats on the rear face.
PLUG_SHELL_COLLIDERS = "Plug/Colliders/Shell_Cap.*"
WELDER_HOUSING_COLLIDER = "Body/Body_Shell"


def _spawn_powerbank(prim_path: str, cfg, *args, **kwargs):
    """Spawn the power bank as Isaac Lab does, then keep its plug's shell clear of the welder housing's collider."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import UsdPhysics

    prim = spawn_from_usd(prim_path, cfg, *args, **kwargs)
    stage = prim.GetStage()
    for path in sim_utils.find_matching_prim_paths(f"{prim_path}/{PLUG_SHELL_COLLIDERS}"):
        env = path.split("/powerbank/")[0]
        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path))
        pairs.CreateFilteredPairsRel().AddTarget(f"{env}/welder/{WELDER_HOUSING_COLLIDER}")
    return prim


SOLVER_ITERATIONS = 32  # position iterations per step for the articulations: light links (0.3 g cable, 4 g plug) in
MAX_DEPENETRATION = 0.2  # m/s: ... tight contacts with 20 N grippers stay stable


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.name == "powerbank":
        addon["func"] = _spawn_powerbank
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=spec.fixed_base or None, solver_position_iteration_count=SOLVER_ITERATIONS,
            solver_velocity_iteration_count=1)
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
        """Each articulation's root (Arena's geometry queries take rigid objects only; its bounds are left at its
        root), plus the grippers, the episode time, the plug and the welder's housing."""
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
        out["plug"] = _link_pose(base, origin, "powerbank", "Plug")
        out["housing"] = _link_pose(base, origin, "welder", "Body")
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
PLUGGED_DEPTH = 0.0015  # m: the plug's front face (its body's, the overmold's) within this of the welder's rear face
PLUGGED_OFF = 0.0005  # m: the plug within this of the port's centre line
PLUGGED_ANGLE = math.radians(8)  # and tilted no more than this from the port's direction (a released plug sags a few
# degrees under its cable, in the port's clearance)
LIFTED = 0.02  # m: a part's lowest point at least this far above the table counts as lifted
ON_FEET = 0.003  # m: the housing's bottom within this of resting on its feet, and
UPRIGHT = math.radians(10)  # ... upright within this
CELL_SHIFT = 0.01  # m: the cell farther than this from its place in the welder's bay has come out (lid opened)
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
# The parts' boxes in their own frames (m): the welder housing (its Body frame starts 4 mm in from the rear face, at the
# bottom) and the plug (its frame on its body's front face: back to the cable boot, forward over the metal shell).
WELDER_BOX = (np.array([-0.004, -WELDER_WIDTH / 2, 0.0]), np.array([0.120, WELDER_WIDTH / 2, WELDER_HEIGHT]))
PLUG_BOX = (np.array([-0.030, -OVERMOLD[1] / 2, -OVERMOLD[2] / 2]), np.array([0.0066, OVERMOLD[1] / 2, OVERMOLD[2] / 2]))
CELL_IN_BODY = CELL_IN_WELDER - (0.004, 0.0, 0.0)
STEPS = ("welder lifted", "plug lifted", "plugged in", "welder set down, plug still in, both let go")
POINTS = 0.25  # per step


def plug_in_port(objects):
    """The plug's front face in the port's frame (m; x along the insertion direction, 0 at the welder's rear face) and
    the angle between the plug's and the port's directions (rad)."""
    h, p = objects["housing"], objects["plug"]
    rh = quat_xyzw_to_matrix(h.quat_xyzw)
    mouth = h.pos + rh @ PORT_IN_BODY
    d = rh.T @ (p.pos - mouth)
    angle = math.acos(float(np.clip(quat_xyzw_to_matrix(p.quat_xyzw)[:, 0] @ rh[:, 0], -1.0, 1.0)))
    return d, angle


def plugged_in(objects) -> bool:
    """The plug is in the port: fully in to within PLUGGED_DEPTH, on its centre line, not tilted past PLUGGED_ANGLE."""
    d, angle = plug_in_port(objects)
    return bool(abs(d[0]) < PLUGGED_DEPTH and np.hypot(d[1], d[2]) < PLUGGED_OFF and angle < PLUGGED_ANGLE)


def _box(pose: PartPose, box) -> tuple[np.ndarray, np.ndarray]:
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


def _grippers(objects):
    return [objects.get("left_gripper"), objects.get("right_gripper")]


def _lifted_and_held(objects, pose: PartPose, box) -> bool:
    bounds = _box(pose, box)
    return bool(bounds[0][2] > TOP + LIFTED and any(_held(bounds, g) for g in _grippers(objects)))


def welder_set_down(objects) -> bool:
    """The welder rests on its feet on the table: its bottom within ON_FEET of there and upright within UPRIGHT."""
    h = objects["housing"]
    up = float(quat_xyzw_to_matrix(h.quat_xyzw)[2, 2])
    return bool(abs(h.pos[2] - (TOP + FEET)) < ON_FEET and up > math.cos(UPRIGHT))


def _nothing_held(objects) -> bool:
    parts = (_box(objects["housing"], WELDER_BOX), _box(objects["plug"], PLUG_BOX))
    return not any(_held(b, g) for b in parts for g in _grippers(objects))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether the welder and the plug have been lifted. Starts over when the
    episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.welder_lifted = self.plug_lifted = False
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.welder_lifted |= _lifted_and_held(objects, objects["housing"], WELDER_BOX)
        self.plug_lifted |= _lifted_and_held(objects, objects["plug"], PLUG_BOX)


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: steps 1 and 2 once reached; 3 and 4 as things stand."""
    p = _PROGRESS
    p.update(objects)
    plugged = plugged_in(objects)
    done = [
        bool(p.welder_lifted),
        bool(p.plug_lifted),
        plugged,
        bool(plugged and welder_set_down(objects) and _nothing_held(objects)),
    ]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        d, angle = plug_in_port(objects)
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        print(f"[welder_usbc] t={objects['episode_time']:.2f}s  steps done: {got}  (plug {d[0] * 1000:+.2f} mm from "
              f"fully in, {np.hypot(d[1], d[2]) * 1000:.2f} mm off the port's line, tilted {math.degrees(angle):.1f} deg)",
              flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the welder or its cell off the table, the cell out of its bay (the lid opened), or a state that is not
    finite."""
    h = objects["housing"]
    for pos in (h.pos, objects["cell"].pos, objects["plug"].pos):
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    cell_in_body = quat_xyzw_to_matrix(h.quat_xyzw).T @ (objects["cell"].pos - h.pos)
    return bool(np.linalg.norm(cell_in_body - CELL_IN_BODY) > CELL_SHIFT)


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the welder and its cell still, and
    no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["welder"].resting() and objects["cell"].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from above, across the welder's middle; lift it and slide it toward the plug, port at MEET, and hold it.
# Right arm: from above, across the plug's overmold; lift it to the port's height a few cm short of it, line up, settle,
# push it straight in along +y until it seats, let go and back off.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width, pressing ~20 N each
WELDER_HOLD = -(WELDER_WIDTH - 1.5 * SQUEEZE) / 2  # ~30 N a finger: the welder hangs 3.3 cm off-centre
PLUG_HOLD = -(OVERMOLD[1] - SQUEEZE / 2) / 2  # ~10 N a finger: firmer pinches with the tapered tips and rolls the plug
WELDER_TIP_PAST_MID = 0.003  # the left pad tips this far past the housing's mid-height (8 mm above its bottom)
PLUG_TIP_PAST_AXIS = 0.0016  # the right pad tips this far past the plug's axis (2 mm above the table, lying)
# The grippers lean, each about its own finger axis: the left one's wrist away from the plug hand (toward +y), so the
# two wrist motors (joint 5) don't meet over the port; the right one's fingertips back (toward -y), so the plug sticks
# out in front of its fingers and they never reach the welder's rear face.
LEAN = math.radians(50)
PLUG_LEAN = 0.0
# Measured in simulation: held near its front, the welder hangs lower and a little short of where the left hand aims
# it (the arm sags under the load); the plug hand aims at where the port actually is (world frame).
PLUG_ALIGN_FIX = np.array([0.00081, 0.0015, -0.00728])
PORT_PITCH = math.radians(2.6)  # the right hand's pitch, at the pick and after (the plug lies flat in it)
PORT_AXIS_PITCH = math.radians(1.2)  # the held welder's port points this much up (nose-high): the plug goes in along it
# Once lifted, the plug is turned about its face to line up with the port (not at the pick, where turning the hand
# changes how the fingers close on the lying plug): pitched up to the port's axis and yawed back from how it sits.
PLUG_PITCH_FIX = math.radians(0.75)
PLUG_YAW_FIX = math.radians(0.72)
STANDOFF = 0.04  # the plug's face this far short of the mouth before lining up
ENTRY = 0.008  # ... then this far short (the metal shell ~2 mm short of the mouth), to settle before pushing
# The arms are soft position servos (a few hundred N/m at the hand): the push target goes this far past seated so the
# arm's spring builds up the force the port's spring fingers need (the welder face stops the overmold); then the hand
# eases back to the seated place while it opens, so it stops pressing.
PUSH_PAST = 0.008

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


LEFT_READY_S = 2.5 + 1.5 + 0.8 + 2.0 + 3.0  # the left arm's approach, grip, lift and carry


_X = np.array([1.0, 0.0, 0.0])
LEFT_GRIP = axis_angle_matrix(_X, -LEAN) @ yam_demo.grip_rotation(WELDER_YAW - math.pi)  # fingers across the housing
RIGHT_GRIP = axis_angle_matrix(_X, PORT_PITCH - PLUG_LEAN) @ yam_demo.grip_rotation(CABLE_YAW)  # fingers across the overmold
_TURN = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), PLUG_YAW_FIX) @ axis_angle_matrix(_X, PLUG_PITCH_FIX)
RIGHT_INSERT = _TURN @ RIGHT_GRIP
GRIP_AXIS = np.array([0.0, math.cos(PORT_PITCH), math.sin(PORT_PITCH)])  # the plug's axis as the hand sees it at the pick
PUSH_AXIS = np.array([0.0, math.cos(PORT_AXIS_PITCH), math.sin(PORT_AXIS_PITCH)])  # the port's axis, as the welder hangs


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ np.array([0.0, 0.0, 1.0]))


def _welder_grip_tcp(welder_origin) -> np.ndarray:
    """The left TCP gripping the welder whose frame origin (rear, bottom centre) is at ``welder_origin``."""
    grip = np.asarray(welder_origin, float) + _rz(WELDER_YAW) @ np.array([WELDER_GRIP_AT, 0.0, WELDER_HEIGHT / 2])
    return _tcp_with_tips(grip, WELDER_TIP_PAST_MID, LEFT_GRIP)


def left_plan(hold_s: float) -> list[Waypoint]:
    """Pick the welder up near its front, bring its port to ``MEET`` facing -y, hold it while the plug goes in and the
    right arm goes home, then set it down on the table right below, let go and go home."""
    rot = LEFT_GRIP
    at = _welder_grip_tcp(WELDER_POS)
    held_origin = MEET - _rz(WELDER_YAW) @ PORT_IN_WELDER
    held = _welder_grip_tcp(held_origin)
    down = _welder_grip_tcp((held_origin[0], held_origin[1], WELDER_POS[2]))
    return [
        Waypoint(2.5, at + (0, 0, 0.06), FINGER_OPEN, "over the welder", rot=rot),
        Waypoint(1.5, at, FINGER_OPEN, "down to the welder", rot=rot),
        Waypoint(0.8, at, WELDER_HOLD, "grip the welder", rot=rot),
        Waypoint(2.0, at + (0, 0, MEET[2] - TOP), WELDER_HOLD, "lift it", rot=rot),
        Waypoint(3.0, held, WELDER_HOLD, "bring the port over", rot=rot),
        Waypoint(hold_s, held, WELDER_HOLD, "hold it still", rot=rot),
        Waypoint(3.0, down + (0, 0, 0.002), WELDER_HOLD, "set it down", rot=rot),
        Waypoint(1.0, down + (0, 0, 0.002), FINGER_OPEN, "let go", rot=rot),
        Waypoint(0.5, down + (0, 0, 0.002), FINGER_OPEN, "fingers open", rot=rot),
        Waypoint(1.5, down + (0, 0, 0.06), FINGER_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(3.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


def right_plan() -> list[Waypoint]:
    """Pick the plug up across its overmold, line it up a few cm short of the port, push it in, let go."""
    rot = RIGHT_GRIP
    lying = np.array([PLUG_START[0], PLUG_START[1], TOP + OVERMOLD[2] / 2 + 0.0005])  # resting flat on the table

    def tcp(face, turned: bool = True) -> np.ndarray:
        """The right TCP holding the plug whose mating face is at ``face`` (pointing +y), the hand turned by the yaw
        correction about that face (or not, at the pick)."""
        grip = _TURN @ (PLUG_GRIP_BEHIND * GRIP_AXIS) if turned else PLUG_GRIP_BEHIND * GRIP_AXIS
        return _tcp_with_tips(np.asarray(face, float) - grip, PLUG_TIP_PAST_AXIS, RIGHT_INSERT if turned else rot)

    at = tcp(lying, turned=False)
    ins = RIGHT_INSERT
    aim = MEET + PLUG_ALIGN_FIX
    out, near, seated = aim - STANDOFF * PUSH_AXIS, aim - ENTRY * PUSH_AXIS, aim + PUSH_PAST * PUSH_AXIS
    return [
        Waypoint(1.5, at + (0, 0, 0.05), FINGER_OPEN, "over the plug", rot=rot),
        Waypoint(1.5, at, FINGER_OPEN, "down to the plug", rot=rot),
        Waypoint(0.8, at, PLUG_HOLD, "grip the plug", rot=rot),
        Waypoint(1.5, at + (0, 0, 0.03), PLUG_HOLD, "lift it", rot=rot),
        Waypoint(max(LEFT_READY_S - 5.3, 2.5), tcp(out), PLUG_HOLD, "in front of the port", rot=ins),
        Waypoint(1.0, tcp(out), PLUG_HOLD, "settle", rot=ins),
        Waypoint(2.0, tcp(near), PLUG_HOLD, "up to the mouth", rot=ins),
        Waypoint(1.5, tcp(near), PLUG_HOLD, "settle at the mouth", rot=ins),
        Waypoint(2.5, tcp(seated), PLUG_HOLD, "push it in", rot=ins),
        Waypoint(1.0, tcp(aim), FINGER_OPEN, "let go", rot=ins),
        Waypoint(0.5, tcp(aim), FINGER_OPEN, "fingers open", rot=ins),
        Waypoint(1.0, tcp(aim - 0.03 * PUSH_AXIS), FINGER_OPEN, "back off", rot=ins),
        Waypoint(1.0, tcp(aim - 0.06 * PUSH_AXIS) + (0, 0, 0.03), FINGER_OPEN, "up and away", rot=ins),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """Both arms: the left arm brings the welder and holds it, the right arm plugs the cable in and goes home, then the
    left arm sets the welder down and goes home."""
    right = right_plan()
    return {"left": left_plan(hold_s=sum(w.duration for w in right) - LEFT_READY_S + 0.5), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
