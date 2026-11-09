"""On the station table: a 4 V pen-style cordless screwdriver (27 mm across, 145 mm long) standing nose up in its
charging dock (fixed to the table), and an aluminium bit block with four 25 mm 1/4-inch hex bits standing in it: PH1,
PH2, SL5 and T20, each with a colour ring (PH2's is red) and its size printed on the block. The left arm holds the
screwdriver steady in its dock; the right arm takes the PH2 bit out of the block, turns it to line its hexagon up with
the hex socket of the screwdriver's magnetic bit holder, pushes it in until it seats and goes back; then the left arm
lets go and goes back too.

The socket is 6.45 mm across flats on a 6.35 mm bit (0.05 mm a side), 10.6 mm deep with a 0.6 mm lead-in: the bit
goes in only within about 1.6 degrees of the socket's hexagon and 0.3 degrees of its axis. The screwdriver, its dock,
the block and each bit are their own rigid bodies from ``assets/``.

Run it:  inspect-robots run --embodiment yam_arena -E task=bit_insert.py --scorer task_score ...

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

# -- the parts (their own frames, m) ----------------------------------------------------------------------------------
# A bit: origin at its hex shank's end face, +z toward the tip; its flats face 0, 60, ... deg about z.
BIT_AF = 0.00635
BIT_LEN = 0.025
BIT_RADIUS = BIT_AF / math.sqrt(3.0)  # across corners / 2
# The screwdriver: origin at its battery end's face centre, +z toward the nose; its hex socket's flats face 0, 60, ...
# deg about z (like the bit's), its mouth on the chuck's top face, its floor (the magnet) SOCKET_DEPTH below.
SOCKET_MOUTH = 0.145
SOCKET_DEPTH = 0.0106
SD_RADIUS = 0.0135
SD_LENGTH = 0.145
SD_GRIP_AT = 0.064  # the left hand's grip: the middle of the rubber grip sleeve (40 to 88 mm up)
# The dock: origin at its base's bottom centre; the screwdriver stands on the cup's floor, 3 mm up.
SD_IN_DOCK = 0.003
DOCK_TOP = 0.032
# The bit block: origin at its bottom centre; holes in a row along its y, 10 mm deep, floors 6 mm up, flats facing +-x.
BLOCK_TOP = 0.016
HOLE_FLOOR = 0.006
BIT_HOLES = {"bit_ph1": 0.027, "bit_ph2": 0.009, "bit_sl5": -0.009, "bit_t20": -0.027}
BIT_FILES = {"bit_ph1": "bit_ph1.usda", "bit_ph2": "bit_ph2.usda", "bit_sl5": "bit_sl5.usda", "bit_t20": "bit_t20.usda"}
BIT_MASSES = {"bit_ph1": 0.00554, "bit_ph2": 0.00563, "bit_sl5": 0.00588, "bit_t20": 0.00578}
TARGET = "bit_ph2"

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The dock on the right arm's side, where its wrist reaches straight down highest (about 0.21 m above the table), turned
# so the socket's hexagon is 25 deg off the bits' in the block (the right wrist turns the bit 25 deg on the way) and
# the screwdriver's switch faces the robot.
DOCK_XY, DOCK_YAW = (0.22, -0.10), math.radians(145)
BLOCK_XY, BLOCK_YAW = (0.30, -0.22), 0.0


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


DOCK_POS = (*DOCK_XY, TOP)
SD_POS = (*DOCK_XY, TOP + SD_IN_DOCK)
BLOCK_POS = (*BLOCK_XY, TOP + 0.00002)
BIT_POS = {n: tuple(float(v) for v in np.array(BLOCK_POS) + _rz(BLOCK_YAW) @ np.array([0.0, y, HOLE_FLOOR + 0.00002]))
           for n, y in BIT_HOLES.items()}
MOUTH = np.array(SD_POS) + np.array([0.0, 0.0, SOCKET_MOUTH])  # the socket's mouth (world)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body (``kinematic``: fixed where
    it spawns). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False


DOCK = UsdPart("dock", asset="dock", xy=DOCK_XY, extent=0.076, mass=0.15, usd_path=str(ASSETS / "screwdriver_dock.usda"),
               pos=DOCK_POS, rot_xyzw=_yaw_xyzw(DOCK_YAW), kinematic=True)
SCREWDRIVER = UsdPart("screwdriver", asset="screwdriver", xy=DOCK_XY, extent=0.03, mass=0.23,
                      usd_path=str(ASSETS / "screwdriver_4v.usda"), pos=SD_POS, rot_xyzw=_yaw_xyzw(DOCK_YAW))
BLOCK = UsdPart("bit_block", asset="bit_block", xy=BLOCK_XY, extent=0.084, mass=0.0775,
                usd_path=str(ASSETS / "bit_block.usda"), pos=BLOCK_POS, rot_xyzw=_yaw_xyzw(BLOCK_YAW))
BITS = [UsdPart(n, asset="bit", xy=tuple(float(v) for v in BIT_POS[n][:2]), extent=0.0074, mass=BIT_MASSES[n],
                usd_path=str(ASSETS / BIT_FILES[n]), pos=BIT_POS[n], rot_xyzw=_yaw_xyzw(BLOCK_YAW)) for n in BIT_HOLES]
OBJECTS = [DOCK, SCREWDRIVER, BLOCK, *BITS]
BIT_NAMES = tuple(BIT_HOLES)
OTHER_BITS = tuple(n for n in BIT_NAMES if n != TARGET)

MAX_DEPENETRATION = 0.2  # m/s: tight contacts with gripped parts stay stable


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD; a kinematic one fixed in place."""
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             kinematic_enabled=spec.kinematic or None)}
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
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
        """Each object's root (its bounds are left at its root: the scoring uses its own boxes), plus the grippers and
        the episode time."""
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
STARTED_DEPTH = 0.002  # m: the bit's end at least this far into the socket, and
STARTED_OFF = 0.001  # ... its end within this of the socket's axis, and
STARTED_TILT = math.radians(5)  # ... its axis within this of the socket's: the bit is started in the holder
SEATED_DEPTH = 0.009  # m: at least this far in (fully home is 10.6 mm), and
SEATED_OFF = 0.0005  # ... within this of the axis, and
SEATED_TILT = math.radians(2)  # ... within this of the axis's direction, and
SEATED_PHASE = math.radians(3)  # ... its hexagon within this of the socket's: the bit is seated, the hex engaged
DOCKED_OFF = 0.0015  # m: the screwdriver's battery end within this of its place on the dock's floor, and
DOCKED_TILT = math.radians(3)  # ... upright within this: the screwdriver is in its dock
IN_HOLE_OFF = 0.0015  # m: a bit's end within this of its hole's floor (block frame), and
IN_HOLE_TILT = math.radians(10)  # ... upright within this: the bit is in its hole
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
# The parts' boxes in their own frames (m).
BIT_BOX = (np.array([-BIT_RADIUS, -BIT_RADIUS, 0.0]), np.array([BIT_RADIUS, BIT_RADIUS, BIT_LEN]))
SD_BOX = (np.array([-SD_RADIUS, -SD_RADIUS, 0.0]), np.array([SD_RADIUS, SD_RADIUS, SD_LENGTH]))
STEPS = ("PH2 bit taken out", "PH2 bit started in the holder", "PH2 bit seated",
         "let go, screwdriver docked, other bits in the block")
POINTS = 0.25  # per step


@dataclass(frozen=True)
class InSocket:
    """A bit against the screwdriver's socket: how far its end is in past the mouth (m), how far its end is off the
    socket's axis (m), the angle between their axes (rad) and between their hexagons (rad, -30 to 30 deg)."""

    depth: float
    off: float
    tilt: float
    phase: float


def bit_in_socket(objects, name: str) -> InSocket:
    sd, bit = objects["screwdriver"], objects[name]
    rs, rb = quat_xyzw_to_matrix(sd.quat_xyzw), quat_xyzw_to_matrix(bit.quat_xyzw)
    d = rs.T @ (bit.pos - (sd.pos + rs @ np.array([0.0, 0.0, SOCKET_MOUTH])))  # the bit's end in the socket's frame
    tilt = math.acos(float(np.clip(rb[:, 2] @ rs[:, 2], -1.0, 1.0)))
    x = rs.T @ rb[:, 0]  # the bit's flat normal, in the socket's frame
    phase = (math.atan2(x[1], x[0]) + math.pi / 6) % (math.pi / 3) - math.pi / 6
    return InSocket(depth=float(-d[2]), off=float(np.hypot(d[0], d[1])), tilt=tilt, phase=phase)


def started(objects, name: str = TARGET) -> bool:
    s = bit_in_socket(objects, name)
    return bool(s.depth >= STARTED_DEPTH and s.off < STARTED_OFF and s.tilt < STARTED_TILT)


def seated(objects, name: str = TARGET) -> bool:
    s = bit_in_socket(objects, name)
    return bool(s.depth >= SEATED_DEPTH and s.off < SEATED_OFF and s.tilt < SEATED_TILT and abs(s.phase) < SEATED_PHASE)


def docked(objects) -> bool:
    """The screwdriver stands in its dock: its battery end on the cup's floor and upright."""
    sd = objects["screwdriver"]
    d = sd.pos - np.array(SD_POS)
    up = float(quat_xyzw_to_matrix(sd.quat_xyzw)[2, 2])
    return bool(np.linalg.norm(d) < DOCKED_OFF and up > math.cos(DOCKED_TILT))


def in_its_hole(objects, name: str) -> bool:
    """The bit stands in its own hole in the block: its end on the hole's floor (block frame) and upright."""
    blk, bit = objects["bit_block"], objects[name]
    rk = quat_xyzw_to_matrix(blk.quat_xyzw)
    d = rk.T @ (bit.pos - blk.pos) - np.array([0.0, BIT_HOLES[name], HOLE_FLOOR])
    up = float(rk[:, 2] @ quat_xyzw_to_matrix(bit.quat_xyzw)[:, 2])
    return bool(np.linalg.norm(d) < IN_HOLE_OFF and up > math.cos(IN_HOLE_TILT))


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


def _bit_bottom(bit) -> float:
    """The bit's lowest point (m, world z)."""
    axis = quat_xyzw_to_matrix(bit.quat_xyzw)[:, 2]
    side = BIT_RADIUS * math.sqrt(max(0.0, 1.0 - axis[2] ** 2))
    return float(min(bit.pos[2], bit.pos[2] + BIT_LEN * axis[2]) - side)


def taken_out(objects, name: str = TARGET) -> bool:
    """A gripper holds the bit lifted clear of the block: its lowest point above the block's top."""
    top = objects["bit_block"].pos[2] + BLOCK_TOP
    bit = objects[name]
    return bool(_bit_bottom(bit) > top and _held_by_any(objects, _box(bit, BIT_BOX)))


def let_go(objects) -> bool:
    """No gripper holds the PH2 bit or the screwdriver."""
    return not (_held_by_any(objects, _box(objects[TARGET], BIT_BOX))
                or _held_by_any(objects, _box(objects["screwdriver"], SD_BOX)))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether the PH2 bit has been taken out of the block by a gripper. Starts
    over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.taken_out = False
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.taken_out |= taken_out(objects)


_PROGRESS = _Progress()
TRACE_EVERY = 0.0  # s: in a run, print the PH2 bit against the socket this often (0: only at step changes)


def _describe(objects) -> str:
    s = bit_in_socket(objects, TARGET)
    sd = objects["screwdriver"]
    r = quat_xyzw_to_matrix(sd.quat_xyzw)
    d = (r.T @ (objects[TARGET].pos - sd.pos - r @ np.array([0.0, 0.0, SOCKET_MOUTH]))) * 1000
    fast = max(np.linalg.norm(objects[n].lin_vel) for n in ("screwdriver", "bit_block", *OTHER_BITS)) * 1000
    w = quat_xyzw_to_matrix(objects[TARGET].quat_xyzw)[:, 2]
    world = r @ d
    return (f"PH2 {s.depth * 1000:+.2f} mm in, {s.off * 1000:.2f} mm off the axis (world {world[0]:+.2f}, {world[1]:+.2f}), "
            f"tilted {math.degrees(s.tilt):.2f} deg (axis x {w[0] * 1000:+.1f} y {w[1] * 1000:+.1f} mrad), hexagon "
            f"{math.degrees(s.phase):+.2f} deg, speed {np.linalg.norm(objects[TARGET].lin_vel) * 1000:.1f} mm/s; "
            f"screwdriver tilted "
            f"{math.degrees(math.acos(min(1.0, r[2, 2]))):.2f} deg, turned "
            f"{math.degrees(math.atan2(r[1, 0], r[0, 0]) - DOCK_YAW):+.2f} deg, moved "
            f"{np.linalg.norm(sd.pos - np.array(SD_POS)) * 1000:.2f} mm; fastest other part {fast:.1f} mm/s")


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached; 2 to 4 as things stand. Steps 2 and 3 count only for the PH2 bit and
    once a gripper has taken it out; step 4 only with step 3."""
    p = _PROGRESS
    p.update(objects)
    in2 = bool(p.taken_out and started(objects))
    in3 = bool(p.taken_out and seated(objects))
    done = [bool(p.taken_out), in2, in3,
            bool(in3 and let_go(objects) and docked(objects) and all(in_its_hole(objects, n) for n in OTHER_BITS))]
    if "episode_time" in objects:  # in a run, log each change: the run's step timeline
        t = objects["episode_time"]
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[bit_insert] t={t:.2f}s  steps done: {got}  ({_describe(objects)})", flush=True)
        elif TRACE_EVERY and t - p.traced >= TRACE_EVERY - 1e-6:
            p.traced = t
            print(f"[bit_insert] t={t:.2f}s  {_describe(objects)}", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the screwdriver, the block or a bit off the table, or a state that is not finite."""
    for name in ("screwdriver", "bit_block", *BIT_NAMES):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the screwdriver and the PH2 bit
    in it still, and no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["screwdriver"].resting() and objects[TARGET].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: pointing at the screwdriver from its own side, 40 deg below level, fingers closing level across the rubber
# grip; holds it until the right arm has let go of the bit, then lets go and goes home.
# Right arm: pointing straight down, fingers closing across two flats of the PH2 bit near its top (the pinch lines the
# bit's hexagon up with the fingers); lift it out, carry it over the screwdriver turning it to the socket's hexagon,
# lower it to just above the mouth, settle, start it into the socket and let go (it slides in), press it home with
# the closed fingertips, go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width, pressing ~20 N each
BIT_SQUEEZE = 0.003  # ... a gentle pinch on the 6 g bit (~7 N a finger)
BIT_HOLD = -(BIT_AF - BIT_SQUEEZE) / 2
BIT_OPEN = -0.012  # the fingers 24 mm apart over the bit (the neighbours stand along the pads' width, 18 mm apart)
SD_SQUEEZE = 0.004
SD_HOLD = -(2 * SD_RADIUS - SD_SQUEEZE) / 2
PINCH_AT = 0.013  # m: the pad tips this far above the bit's end (its hex shank runs to 18.5 mm)
LIFT = 0.04  # m: the bit lifted this far out of its hole (10 mm deep)
CARRY_ABOVE = 0.01  # m: the bit's end carried this far above the mouth, then lowered to ENTRY above it
ENTRY = 0.002
# Pushing a pinched bit hard tilts it in the fingers (a few degrees: the hexagon then jams, it fits within 0.3 deg). So
# the hand starts the bit in, START_DEPTH past the mouth (into the lead-in, its hexagon lined up), and lets go: like a
# magnetic holder, the socket takes it, and it slides down to the magnet. Then the closed fingertips press on its tip,
# PRESS past where it sits fully home, to push it home.
START_DEPTH = 0.001
ABOVE_TOP = 0.004
PRESS = 0.0015
# Measured in simulation: where the bit actually is against the socket when the hand aims at it (world frame), and the
# hand's turn about the vertical.
INSERT_FIX = np.array([-0.00060, 0.00118, -0.0007])
YAW_FIX = math.radians(-0.72)
TILT_FIX = np.array([0.0095, 0.0202, 0.0])  # rad (world rotation vector): the bit leans 1.3 deg in the hand


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


def side_grip(approach: float, pitch: float = 0.0) -> np.ndarray:
    """The gripper pointing along ``approach`` (rad from +x) and ``pitch`` below level, fingers closing level."""
    forward = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN  # pointing +x, fingers closing along y
    r = axis_angle_matrix(_Z, approach) @ forward
    for sign in (1.0, -1.0):
        rot = axis_angle_matrix(r[:, 0], sign * pitch) @ r
        if rot[2, 2] <= 0.0:
            return rot
    return r


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


def _closing_yaw(rot: np.ndarray) -> float:
    """The direction (rad about z) of a downward gripper's finger-closing axis (its x)."""
    return math.atan2(rot[1, 0], rot[0, 0])


def _down_closing(yaw: float) -> np.ndarray:
    """The downward gripper whose fingers close along ``yaw`` (rad about z), as ``grip_rotation`` turns it."""
    for base in (math.pi / 2, 0.0, -math.pi / 2, math.pi):
        g = yam_demo.grip_rotation(yaw + base)
        if abs(math.cos(_closing_yaw(g) - yaw)) > 0.999:
            return g
    raise ValueError("grip_rotation does not close along the asked direction")


# The pick closes the fingers across the bit's flats facing the block's x; the insert turns the hand by the smallest
# angle that lines those flats up with the socket's (both hexagons repeat every 60 deg).
PICK_CLOSE = BLOCK_YAW
_TURN = ((DOCK_YAW - PICK_CLOSE + math.pi / 6) % (math.pi / 3)) - math.pi / 6
INSERT_CLOSE = PICK_CLOSE + _TURN + YAW_FIX
PICK_ROT = _down_closing(PICK_CLOSE)
INSERT_ROT = axis_angle_matrix(TILT_FIX / max(np.linalg.norm(TILT_FIX), 1e-12), float(np.linalg.norm(TILT_FIX))) \
    @ _down_closing(INSERT_CLOSE)
LEFT_APPROACH = math.atan2(DOCK_XY[1] - BASES["left"][1], DOCK_XY[0] - BASES["left"][0])
LEFT_PITCH = math.radians(40)
LEFT_GRIP = side_grip(LEFT_APPROACH, LEFT_PITCH)


def _bit_tcp(bit_end, rot) -> np.ndarray:
    """The right TCP pinching the bit whose end (its origin) is at ``bit_end``, standing upright."""
    return _tcp_with_tips(np.asarray(bit_end, float) + (0.0, 0.0, PINCH_AT), 0.0, rot)


def right_plan(wait_s: float) -> list[Waypoint]:
    """Take the PH2 bit out of the block, start it in the socket, let go, press it home, go home."""
    bit = np.array(BIT_POS[TARGET])
    pick = _bit_tcp(bit, PICK_ROT)
    up = pick + (0, 0, LIFT)
    aim = MOUTH + INSERT_FIX
    over = _bit_tcp(aim + (0, 0, CARRY_ABOVE), INSERT_ROT)
    entry = _bit_tcp(aim + (0, 0, ENTRY), INSERT_ROT)
    start = _bit_tcp(aim - (0, 0, START_DEPTH), INSERT_ROT)
    top = aim + (0, 0, BIT_LEN - SOCKET_DEPTH)  # the tip of the bit, fully home
    above = _tcp_with_tips(top + (0, 0, ABOVE_TOP), 0.0, INSERT_ROT)
    press = _tcp_with_tips(top - (0, 0, PRESS), 0.0, INSERT_ROT)
    return [
        Waypoint(2.5, pick + (0, 0, 0.06), BIT_OPEN, "over the PH2 bit", rot=PICK_ROT),
        Waypoint(1.5, pick, BIT_OPEN, "down around it", rot=PICK_ROT),
        Waypoint(0.8, pick, BIT_HOLD, "pinch it", rot=PICK_ROT),
        Waypoint(1.5, up, BIT_HOLD, "lift it out", rot=PICK_ROT),
        Waypoint(max(wait_s, 0.5), up, BIT_HOLD, "wait for the left hand", rot=PICK_ROT),
        Waypoint(3.0, over, BIT_HOLD, "over the socket, turned", rot=INSERT_ROT),
        Waypoint(1.5, entry, BIT_HOLD, "down to the mouth", rot=INSERT_ROT),
        Waypoint(1.0, entry, BIT_HOLD, "settle", rot=INSERT_ROT),
        Waypoint(1.5, start, BIT_HOLD, "start it in", rot=INSERT_ROT),
        Waypoint(0.8, start, BIT_OPEN, "let it go in", rot=INSERT_ROT),
        Waypoint(0.5, start, BIT_OPEN, "fingers open", rot=INSERT_ROT),
        Waypoint(1.0, above, BIT_OPEN, "over its tip", rot=INSERT_ROT),
        Waypoint(0.6, above, 0.0, "close the fingers", rot=INSERT_ROT),
        Waypoint(1.2, press, 0.0, "press it home", rot=INSERT_ROT),
        Waypoint(0.5, press, 0.0, "hold", rot=INSERT_ROT),
        Waypoint(0.8, press + (0, 0, 0.03), 0.0, "up", rot=INSERT_ROT),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


LEFT_READY_S = 2.5 + 1.5 + 0.8  # the left arm's approach and grip


def left_plan(hold_s: float) -> list[Waypoint]:
    """Grip the screwdriver's rubber grip from the side and hold it; let go, back off, go home."""
    rot = LEFT_GRIP
    at = np.array([*DOCK_XY, TOP + SD_IN_DOCK + SD_GRIP_AT])  # the TCP on the screwdriver's axis
    out = at - 0.06 * (rot @ _Z)  # backed off along the pointing direction
    return [
        Waypoint(2.5, out, FINGER_OPEN, "beside the screwdriver", rot=rot),
        Waypoint(1.5, at, FINGER_OPEN, "around its grip", rot=rot),
        Waypoint(0.8, at, SD_HOLD, "hold it", rot=rot),
        Waypoint(hold_s, at, SD_HOLD, "keep it steady", rot=rot),
        Waypoint(1.0, at, FINGER_OPEN, "let go", rot=rot),
        Waypoint(1.0, out, FINGER_OPEN, "back off", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(1.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


def demo():
    """Both arms: the left holds the screwdriver steady while the right takes the PH2 bit out of the block; the right
    pushes it into the socket, lets go and goes home; then the left lets go and goes home."""
    picked_s = 2.5 + 1.5 + 0.8 + 1.5
    wait = LEFT_READY_S + 0.5 - picked_s
    right = right_plan(wait)
    released = sum(w.duration for w in right[:-1])  # the right hand has pressed the bit home and lifted
    return {"left": left_plan(hold_s=released - LEFT_READY_S), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
