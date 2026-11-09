"""On the station table: a 2x2 18650 pack-assembly jig screwed to the bench, the two halves of a 2x2 cell holder
lying on the bench (one cups up, one cups down) and a tray with four 18650 cells standing in it, positive ends up. The
left arm puts the cups-up holder half into the jig; the right arm takes the four cells from the tray one by one and
pushes each into a cup of that holder, positive end up; then the left arm lowers the other holder half, cups down, onto
the tops of the four cells, and both arms go back.

The holder halves (the same part), the cells and the tray are rigid bodies from ``assets/``; the jig is fixed.

Run it:  inspect-robots run --embodiment yam_arena -E task=pack_assembly.py --scorer task_score ...

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
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the holder: one half of a 2x2 18650 holder (41.2 x 41.2 x 9.5 mm, 20 mm pitch); origin at the centre of its outer
# ledge face, +z into the cups. The top half is the same part turned over.
HOLDER_W, HOLDER_H = 0.0412, 0.0095
CELL_PITCH = 0.020
CELL_RADIUS = 0.009
CELL_HALF = 0.03325  # the cell's centre to its bottom face, and to the top of its + nub
CELL_BODY_TOP = 0.03155  # the cell's centre to the top of its body (the nub stands 1.7 mm above it)
LEDGE = 0.0015  # the cups' floor (a ledge with a window) above the holder's outer face
CELL_SEATED_Z = LEDGE + CELL_HALF  # a seated cell's centre above the bottom holder's outer face
TOP_ON_CELLS = LEDGE + CELL_HALF + CELL_BODY_TOP + LEDGE  # the top half's outer face, resting on the cells: 67.8 mm
CUPS = {"FL": (1, 1), "BL": (-1, 1), "FR": (1, -1), "BR": (-1, -1)}  # half pitches in the holder's frame; F = +x
# -- the jig (76 x 76 x 6 mm, its 42 x 42 mm pocket 3 mm deep) and the tray (80 x 80 x 14 mm, pockets 11 mm deep on a
# 36 mm pitch); both have their origin at the centre of their bottom face
JIG_FLOOR = 0.003
TRAY_FLOOR = 0.003
TRAY_H = 0.014
TRAY_PITCH = 0.036

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
JIG_XY = (0.30, -0.05)
TRAY_XY = (0.30, -0.26)
BOTTOM_XY = (0.24, 0.15)  # the holder half lying cups up
LID_XY = (0.34, 0.18)  # the holder half lying cups down
ORDER = ("FL", "FR", "BL", "BR")  # the right arm fills the cups in this order: cell_1 goes into FL, ...
# each cell stands in the tray's pocket on the same side as its cup (the tray's pockets in half pitches)


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


FLIPPED = (1.0, 0.0, 0.0, 0.0)  # turned over: 180 degrees about x
JIG_POS = (*JIG_XY, TOP)
FLOOR_Z = TOP + JIG_FLOOR  # the bottom holder's outer face, in the jig
TRAY_POS = (*TRAY_XY, TOP + 0.0002)


def cup_seat(cup: str, holder_xy=JIG_XY, floor_z: float = FLOOR_Z) -> np.ndarray:
    """The centre of a cell seated in ``cup`` of an upright holder at ``holder_xy`` (its outer face at ``floor_z``)."""
    sx, sy = CUPS[cup]
    return np.array([holder_xy[0] + sx * CELL_PITCH / 2, holder_xy[1] + sy * CELL_PITCH / 2, floor_z + CELL_SEATED_Z])


def tray_cell(cup: str) -> np.ndarray:
    """The centre of the cell standing in the tray's pocket for ``cup``."""
    sx, sy = CUPS[cup]
    return np.array([TRAY_XY[0] + sx * TRAY_PITCH / 2, TRAY_XY[1] + sy * TRAY_PITCH / 2,
                     TRAY_POS[2] + TRAY_FLOOR + CELL_HALF])


BOTTOM_POS = (*BOTTOM_XY, TOP + 0.0002)
CELL_START = {c: tray_cell(c) for c in ORDER}
LID_POS = (*LID_XY, TOP + 0.0002 + HOLDER_H)  # turned over: its outer face is on top


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body. ``Asset``'s fields only
    satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False  # a body that never moves (the jig, screwed to the bench)

    @property
    def free(self) -> bool:
        return not self.kinematic


JIG = UsdPart("jig", asset="jig", xy=JIG_XY, extent=0.076, mass=0.0185, usd_path=str(ASSETS / "pack_jig_2x2.usda"),
              pos=JIG_POS, kinematic=True)
TRAY = UsdPart("tray", asset="tray", xy=TRAY_XY, extent=0.08, mass=0.0474,
               usd_path=str(ASSETS / "cell_tray_18650_2x2.usda"), pos=TRAY_POS)
BOTTOM = UsdPart("bottom_holder", asset="holder", xy=BOTTOM_POS[:2], extent=0.042, mass=0.0065,
                 usd_path=str(ASSETS / "holder_18650_2x2.usda"), pos=tuple(float(v) for v in BOTTOM_POS))
LID = UsdPart("top_holder", asset="holder", xy=LID_XY, extent=0.042, mass=0.0065,
              usd_path=str(ASSETS / "holder_18650_2x2.usda"), pos=LID_POS, rot_xyzw=FLIPPED)
CELLS = [UsdPart(f"cell_{k + 1}", asset="cell", xy=tuple(float(v) for v in CELL_START[c][:2]), extent=0.065,
                 mass=0.047, usd_path=str(ASSETS / "battery_18650_yellow.usda"),
                 pos=tuple(float(v) for v in CELL_START[c])) for k, c in enumerate(ORDER)]
CELL_NAMES = tuple(c.name for c in CELLS)
HOLDER_NAMES = (BOTTOM.name, LID.name)
OBJECTS = [JIG, TRAY, BOTTOM, LID, *CELLS]

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
    return scene.Object(
        name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon={"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)},
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
        """Each object's root pose and velocity, plus the grippers and the episode time."""
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
IN_JIG_XY = 0.002  # m: the bottom holder's centre within this of the pocket's centre, and
IN_JIG_Z = 0.0015  # ... its outer face within this of the pocket floor, and
UPRIGHT = math.radians(10)  # ... cups up within this
SEAT_XY = 0.001  # m: a cell's bottom face's centre within this of a cup's axis, and
SEAT_Z = 0.0015  # ... within this of the cup's ledge, and
CELL_ALIGNED = math.radians(8)  # ... its + end along the holder's cups within this: seated, + up
LID_XY_OFF = 0.002  # m: the top holder's centre within this of the bottom one's axis, and
LID_Z_OFF = 0.0015  # ... its outer face within this of resting on the four cells, and turned over within UPRIGHT
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Still: the holders and cells keep a contact jitter of a few mm/s in PhysX (signed-distance-field holders, 0.3 mm cup
# clearance), so "still" for the success hold is under STILL_SPEED and STILL_SPIN: nothing sliding, tipping or falling.
STILL_SPEED = 0.03  # m/s
STILL_SPIN = 1.0  # rad/s
GRIP_SLACK = 0.005  # held: a gripper's pads span the part's box grown by this, both fingers press with at least
GRIP_FORCE = 0.5  # ... this (N), and the pads are at least GRIP_OPENING apart
GRIP_OPENING = 0.003
CELL_BOX = (np.array([-CELL_RADIUS, -CELL_RADIUS, -CELL_HALF]), np.array([CELL_RADIUS, CELL_RADIUS, CELL_HALF]))
HOLDER_BOX = (np.array([-HOLDER_W / 2, -HOLDER_W / 2, 0.0]), np.array([HOLDER_W / 2, HOLDER_W / 2, HOLDER_H]))
STEPS = ("bottom holder in the jig", "two cells seated", "four cells seated", "top holder on")
POINTS = 0.25  # per step


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(state.quat_xyzw)


def holder_in_jig(h) -> bool:
    """The holder lies in the jig's pocket, cups up."""
    return bool(_rot(h)[2, 2] > math.cos(UPRIGHT) and np.linalg.norm(h.pos[:2] - np.array(JIG_XY)) < IN_JIG_XY
                and abs(h.pos[2] - FLOOR_Z) < IN_JIG_Z)


def cell_in_cup(cell, h) -> tuple[str | None, np.ndarray]:
    """The cup of holder ``h`` the cell is in (seated: its bottom face's centre on the cup's ledge, + end into the
    cup's opening), or None; and that offset from the nearest cup's ledge centre in the holder's frame (m). A seated
    cell may lean a little in its 0.3 mm clearance cup; its bottom stays put."""
    r = _rot(h)
    local = r.T @ (cell.pos - CELL_HALF * _rot(cell)[:, 2] - h.pos)
    name, off = min(((c, local - np.array([sx * CELL_PITCH / 2, sy * CELL_PITCH / 2, LEDGE]))
                     for c, (sx, sy) in CUPS.items()), key=lambda t: np.linalg.norm(t[1]))
    aligned = float(_rot(cell)[:, 2] @ r[:, 2]) > math.cos(CELL_ALIGNED)
    seated = np.linalg.norm(off[:2]) < SEAT_XY and abs(off[2]) < SEAT_Z and aligned
    return (name if seated else None), off


def seated_cups(objects, holder: str) -> set:
    return {c for c in (cell_in_cup(objects[n], objects[holder])[0] for n in CELL_NAMES) if c is not None}


def lid_offset(lid, h) -> np.ndarray:
    """The top holder's outer face's centre relative to where it rests on four cells in ``h`` (m, ``h``'s frame)."""
    return _rot(h).T @ (lid.pos - h.pos) - np.array([0.0, 0.0, TOP_ON_CELLS])


def lid_on(lid, h) -> bool:
    """``lid`` sits turned over on the cells in ``h``, at the height where it rests on all four."""
    off = lid_offset(lid, h)
    return bool(float(_rot(lid)[:, 2] @ _rot(h)[:, 2]) < -math.cos(UPRIGHT) and np.linalg.norm(off[:2]) < LID_XY_OFF
                and abs(off[2]) < LID_Z_OFF)


def base_holder(objects) -> tuple[str, str]:
    """The holder half the pack is being built in (the one in the jig, else the cups-up one with more cells seated),
    and the other one."""
    def rank(n):
        h = objects[n]
        return (holder_in_jig(h), _rot(h)[2, 2] > math.cos(UPRIGHT), len(seated_cups(objects, n)))

    b = max(HOLDER_NAMES, key=rank)
    return b, next(n for n in HOLDER_NAMES if n != b)


def _box(state, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``state``'s pose."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ _rot(state).T + state.pos
    return w.min(0), w.max(0)


def _held(bounds, gripper) -> bool:
    """``gripper`` holds the part with world ``bounds``: its pads reach them and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = bounds[0] - GRIP_SLACK, bounds[1] + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """The steps last reported this episode; starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t < self.time:
                self.__init__()
            self.time = t


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand: the bottom holder in the jig; two, then four cells seated + up in it; the
    top holder turned over and resting on all four."""
    p = _PROGRESS
    p.update(objects)
    b, lid = base_holder(objects)
    n = len(seated_cups(objects, b))
    in_jig = holder_in_jig(objects[b])
    done = [in_jig, in_jig and n >= 2, in_jig and n == 4, in_jig and n == 4 and lid_on(objects[lid], objects[b])]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        off = lid_offset(objects[lid], objects[b]) * 1000
        print(f"[pack_assembly] t={objects['episode_time']:.2f}s  steps done: {got}  ({n} cells seated; bottom holder "
              f"{np.linalg.norm(objects[b].pos[:2] - np.array(JIG_XY)) * 1000:.1f} mm from the pocket's centre; top "
              f"holder {np.linalg.norm(off[:2]):.1f} mm off the pack's axis, {off[2]:.1f} mm above its seat)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: a part off the table, or a state that is not finite."""
    for n in (TRAY.name, *HOLDER_NAMES, *CELL_NAMES):
        pos = objects[n].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the pack still, no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right")
                and all(objects[n].resting(STILL_SPEED, STILL_SPIN) for n in (*HOLDER_NAMES, *CELL_NAMES)))


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: pinch the cups-up holder half across two opposite sides (the planar grip faces either side of their middle)
# from above, and lower it into the jig's pocket (the fingertips go down the jig's finger slots); later pinch the
# cups-down half the same way, carry it over the cells and lower it onto their tops.
# Right arm: the cells in the order FL, FR, BL, BR. The fingers widen from 15 mm at their tips to 26 mm level with the
# TCP and 40 mm 20 mm behind it, so they can't reach between two cells 2 mm apart: each of the first three cells is
# gripped from above at mid-height with the fingers closing toward cups still empty (FL across x; FR across x, the
# fingers shifted away from FL; BL across y, shifted away from FL and FR). The last cell (BR) is gripped from the side,
# like a cell taken out of the holder's corner: the gripper points along the holder's diagonal toward its centre,
# PICK_TILT below level, the fingers closing across the cell's two free sides with their tips PICK_TIPS_PAST its axis,
# short of its two neighbours. Each cell is lowered straight down to its cup (slowly between its neighbours), steadied
# at the cup's mouth and let go once its bottom is just inside the bore: it drops onto its seat. The fingers come up
# over its top, close, and press on its + nub to push it fully home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (~20 N a finger)
CELL_HOLD = -(2 * CELL_RADIUS - SQUEEZE) / 2
SIDE_HOLD = -(2 * CELL_RADIUS - 1.5 * SQUEEZE) / 2  # ~30 N: the sideways grip holds the cell straighter
CELL_OPEN = -(2 * CELL_RADIUS + 0.012) / 2  # 6 mm clear of the cell each side
CELL_LET_GO = -(2 * CELL_RADIUS + 0.004) / 2  # 2 mm clear each side
HOLDER_SQUEEZE = 0.004  # ~10 N a finger on the 6.5 g holder
HOLDER_HOLD = -(HOLDER_W - HOLDER_SQUEEZE) / 2
HOLDER_OPEN = -(HOLDER_W + 0.016) / 2  # 8 mm clear each side (the jig's slots run out to its edge)
HOLDER_TIPS_UP = 0.0015  # the pad tips this far above a holder's lower face
TOP_GRIP_DZ = 0.020  # top-down cell grips: the TCP this far above the cell's centre (the gripper's wide part clears
# the seated cells' tops)
TOP_GRIP_SHIFT = 0.006  # ... shifted this far away from seated neighbours along the pads' width
PICK_TILT = math.radians(30)  # the sideways grip points this far below level
PICK_TIPS_PAST = 0.001  # its pad tips this far past the cell's axis
SIDE_GRIP_DZ = -0.004  # its TCP's height above the cell's centre (before the tilt): the pads about its middle
CLEAR = 0.006  # m: a carried cell's bottom this far above what it passes over
ENTER = 0.003  # m: the cell comes down to this far above the cup's mouth, then slowly in
LEAD_IN = 0.0008  # the cups' 45 degree lead-in
RELEASE_IN = 0.0005  # the cell is let go with its bottom this far into the bore below the lead-in; it drops onto its
PRESS = 0.0005  # seat, and the closed fingertips then press this far below a seated cell's nub top
SEAT_GAP = 0.0005  # parts are let go this far above where they rest
SETTLE = 0.6  # s: the arm holds still this long before the fingers close or open (it lags a few mm after a move)
# The right arm droops under a held cell by a few tenths of a mm to 2 mm, differently at each pose (and the cell sits up
# to 0.8 mm off in the hand): without correction the FL and BR cells came down 1.6 and 2.3 mm off their cups (the
# physics is deterministic; the cups' lead-in catches only about 1.1 mm), so the demo shifts those cups' waypoints by
# AIM (world x, y; m), the opposite of the measured offsets.
AIM = {"FL": (0.0004, 0.0016), "FR": (0.0, 0.0), "BL": (0.0, 0.0), "BR": (-0.0002, 0.0023)}
RIGHT_ANGLE_RANGE = (-135.0, 45.0)  # deg: closing directions the right wrist reaches gripping from above
LEFT_ANGLE_RANGE = (135.0, 315.0)  # ... and the left one


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

_Z = np.array([0.0, 0.0, 1.0])


def _down(close_deg: float, reach=RIGHT_ANGLE_RANGE) -> np.ndarray:
    """The gripper pointing straight down, its fingers closing along the horizontal direction ``close_deg`` (or the
    opposite one, whichever the wrist reaches); its frame is x = closing, y = pad width, z = pointing."""
    a = close_deg
    while a < reach[0]:
        a += 180.0
    while a > reach[1]:
        a -= 180.0
    x = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0])
    z = -_Z
    return np.column_stack([x, np.cross(z, x), z])


@dataclass(frozen=True)
class CellGrip:
    """How the right gripper holds a cell for one cup: its rotation, and the TCP's offset from the cell's centre
    (world frame; the cell stays upright and the gripper's rotation fixed from the pick to the cup)."""

    rot: np.ndarray
    off: np.ndarray
    side: np.ndarray | None = None  # sideways grips: the horizontal direction the gripper points in


def _side_grip(cup: str) -> CellGrip:
    """From the side, along the holder's diagonal through ``cup`` toward its centre, PICK_TILT below level."""
    sx, sy = CUPS[cup]
    along = -np.array([sx, sy, 0.0]) / math.sqrt(2.0)
    point = along * math.cos(PICK_TILT) - _Z * math.sin(PICK_TILT)
    close = np.cross(along, _Z)
    rot = np.column_stack([close, np.cross(point, close), point])
    return CellGrip(rot, np.array([0.0, 0.0, SIDE_GRIP_DZ]) - (PAD_TIP_PAST_TCP - PICK_TIPS_PAST) * point, along)


GRIPS = {
    "FL": CellGrip(_down(0.0), np.array([0.0, 0.0, TOP_GRIP_DZ])),
    "FR": CellGrip(_down(0.0), np.array([0.0, -TOP_GRIP_SHIFT, TOP_GRIP_DZ])),
    "BL": CellGrip(_down(90.0), np.array([-TOP_GRIP_SHIFT, 0.0, TOP_GRIP_DZ])),
    "BR": _side_grip("BR"),
}
PACK_TOP = FLOOR_Z + LEDGE + 2 * CELL_HALF  # the seated cells' nub tops


def _insert(cup: str, pick_centre, wait_s: float = 0.0) -> list[Waypoint]:
    """Take the cell standing at ``pick_centre`` out of the tray and put it into ``cup``."""
    g = GRIPS[cup]
    hold = CELL_HOLD if g.side is None else SIDE_HOLD
    pick = np.asarray(pick_centre, float) + g.off
    seat = cup_seat(cup) + g.off + (*AIM[cup], 0.0)  # the TCP were the held cell seated

    def bottom_at(z):  # the TCP with the held cell's bottom at height z over the cup
        return seat + (0, 0, z - (FLOOR_Z + LEDGE))

    mouth = bottom_at(FLOOR_Z + HOLDER_H + ENTER)
    release = bottom_at(FLOOR_Z + HOLDER_H - LEAD_IN - RELEASE_IN)  # the cell's bottom just inside the bore
    if g.side is None:  # from above: lift clear of the tray, carry over the cells, straight down into the cup
        carry = bottom_at(PACK_TOP + CLEAR)
        approach = [Waypoint(2.0, pick + (0, 0, 0.04), CELL_OPEN, f"over the {cup} cell", rot=g.rot),
                    Waypoint(1.2, pick, CELL_OPEN, f"around the {cup} cell", rot=g.rot)]
        lifted = pick + (0, 0, (TRAY_POS[2] + TRAY_H + CLEAR if wait_s > 0 else PACK_TOP + CLEAR)
                         - (np.asarray(pick_centre)[2] - CELL_HALF))  # the first cell waits just above the tray
        to_cup = [Waypoint(2.0, carry, hold, f"over cup {cup}", rot=g.rot),
                  Waypoint(0.5, carry, hold, f"steady over cup {cup}", rot=g.rot),
                  Waypoint(1.0, bottom_at(PACK_TOP + 0.002), hold, f"down to the cells' tops at {cup}", rot=g.rot),
                  Waypoint(3.0, mouth, hold, f"down to cup {cup}", rot=g.rot),
                  Waypoint(0.8, mouth, hold, f"steady at cup {cup}", rot=g.rot)]
        up = CELL_HALF + PAD_TIP_PAST_TCP - g.off[2] + 0.004  # the open fingertips 4 mm over the cell's top
        leave = [Waypoint(1.0, release + (0, 0, up), CELL_LET_GO, f"fingers up past the {cup} cell", rot=g.rot)]
    else:  # from the side: in level with the cell, lift it just clear of the tray, in from outside the holder's corner
        out = 0.05 * g.side
        approach = [Waypoint(2.5, pick - out + (0, 0, 0.10), CELL_OPEN, f"beside the {cup} cell", rot=g.rot),
                    Waypoint(1.2, pick - out, CELL_OPEN, f"level with the {cup} cell", rot=g.rot),
                    Waypoint(1.5, pick, CELL_OPEN, f"around the {cup} cell", rot=g.rot)]
        low = bottom_at(max(TRAY_POS[2] + TRAY_H, FLOOR_Z + HOLDER_H) + CLEAR)
        lifted = pick + (0, 0, low[2] - seat[2] + cup_seat(cup)[2] - np.asarray(pick_centre)[2])  # its bottom as low
        to_cup = [Waypoint(2.0, low - out, hold, f"outside cup {cup}", rot=g.rot),
                  Waypoint(1.5, low, hold, f"over cup {cup}", rot=g.rot),
                  Waypoint(0.5, low, hold, f"steady over cup {cup}", rot=g.rot),
                  Waypoint(1.0, mouth, hold, f"down to cup {cup}", rot=g.rot),
                  Waypoint(0.8, mouth, hold, f"steady at cup {cup}", rot=g.rot)]
        leave = [Waypoint(1.2, release - out, CELL_LET_GO, f"out from {cup}", rot=g.rot),
                 Waypoint(1.0, release - out + (0, 0, 0.06), CELL_LET_GO, f"up from {cup}", rot=g.rot)]
    wps = approach + [Waypoint(SETTLE, pick, CELL_OPEN, f"settle at the {cup} cell", rot=g.rot),
                      Waypoint(0.6, pick, hold, f"grip the {cup} cell", rot=g.rot),
                      Waypoint(1.5, lifted, hold, f"lift the {cup} cell", rot=g.rot)]
    if wait_s > 0:
        wps.append(Waypoint(wait_s, lifted, hold, "wait for the holder", rot=g.rot))
    axis = cup_seat(cup) + (*AIM[cup], 0.0)
    above = axis + (0, 0, CELL_HALF + 0.006 + PAD_TIP_PAST_TCP)  # closed fingertips 6 mm over its nub
    pressed = axis + (0, 0, CELL_HALF - PRESS + PAD_TIP_PAST_TCP)
    press_rot = _down(-45.0)
    return wps + to_cup + [
        Waypoint(1.2, release, hold, f"into cup {cup}", rot=g.rot),
        Waypoint(SETTLE, release, hold, f"settle in {cup}", rot=g.rot),
        Waypoint(0.6, release, CELL_LET_GO, f"let go in {cup}", rot=g.rot),
    ] + leave + [
        Waypoint(1.5, above, 0.0, f"over the {cup} cell", rot=press_rot),
        Waypoint(1.0, pressed, 0.0, f"press {cup} home", rot=press_rot),
        Waypoint(0.4, pressed, 0.0, f"hold {cup} down", rot=press_rot),
        Waypoint(0.8, above + (0, 0, 0.01), 0.0, f"off {cup}", rot=press_rot),
    ]


def _holder_tcp(xy, lower_face_z: float) -> np.ndarray:
    """The left TCP pinching a holder half whose lower face is at ``lower_face_z``."""
    return np.array([xy[0], xy[1], lower_face_z + HOLDER_TIPS_UP + PAD_TIP_PAST_TCP])


LEFT_PLACED_S = 2.0 + 1.2 + SETTLE + 0.6 + 1.2 + 2.0 + 1.2 + 0.6 + 0.8  # the bottom holder is in the jig, hand up
LEFT_CLEAR_S = LEFT_PLACED_S + 1.5  # ... and the left arm on its way to the top holder


def right_plan() -> list[Waypoint]:
    """The cells, one by one, from the tray into their cups; home."""
    wps = []
    for k, cup in enumerate(ORDER):
        wait = max(LEFT_CLEAR_S - 5.3 - SETTLE, 0.5) if k == 0 else 0.0  # the first cell waits for the holder
        wps += _insert(cup, CELL_START[cup], wait)
    wps.append(Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]))
    return wps


def left_plan(lid_at_s: float) -> list[Waypoint]:
    """The bottom holder into the jig; the top holder picked up and held aside; onto the cells once the right arm is
    done (``lid_at_s``); home."""
    home = [Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
            Waypoint(3.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"])]  # for the 1 s success hold
    rot = _down(90.0, LEFT_ANGLE_RANGE)  # closing across y
    pick_b = _holder_tcp(BOTTOM_XY, BOTTOM_POS[2])
    place_b = _holder_tcp(JIG_XY, FLOOR_Z + SEAT_GAP)
    pick_t = _holder_tcp(LID_XY, TOP + 0.0002)
    over_cells = _holder_tcp(JIG_XY, PACK_TOP + CLEAR)
    on_cells = _holder_tcp(JIG_XY, FLOOR_Z + TOP_ON_CELLS - HOLDER_H + SEAT_GAP)
    lid_lifted = pick_t + (0, 0, 0.03)
    wps = [
        Waypoint(2.0, pick_b + (0, 0, 0.05), HOLDER_OPEN, "over the bottom holder", rot=rot),
        Waypoint(1.2, pick_b, HOLDER_OPEN, "down to the bottom holder", rot=rot),
        Waypoint(SETTLE, pick_b, HOLDER_OPEN, "settle at the bottom holder", rot=rot),
        Waypoint(0.6, pick_b, HOLDER_HOLD, "grip the bottom holder", rot=rot),
        Waypoint(1.2, pick_b + (0, 0, 0.03), HOLDER_HOLD, "lift it", rot=rot),
        Waypoint(2.0, place_b + (0, 0, 0.015), HOLDER_HOLD, "over the jig", rot=rot),
        Waypoint(1.2, place_b, HOLDER_HOLD, "into the jig", rot=rot),
        Waypoint(0.6, place_b, HOLDER_OPEN, "let go of it", rot=rot),
        Waypoint(0.8, place_b + (0, 0, 0.05), HOLDER_OPEN, "up from the jig", rot=rot),
        Waypoint(2.5, pick_t + (0, 0, 0.05), HOLDER_OPEN, "over the top holder", rot=rot),
        Waypoint(1.5, pick_t, HOLDER_OPEN, "down to the top holder", rot=rot),
        Waypoint(SETTLE, pick_t, HOLDER_OPEN, "settle at the top holder", rot=rot),
        Waypoint(0.8, pick_t, HOLDER_HOLD, "grip the top holder", rot=rot),
        Waypoint(2.0, lid_lifted, HOLDER_HOLD, "lift it", rot=rot),
    ]
    waited = sum(w.duration for w in wps)
    wps += [
        Waypoint(max(lid_at_s - 1.5 - waited, 0.5), lid_lifted, HOLDER_HOLD, "wait for the cells", rot=rot),
        Waypoint(1.5, lid_lifted + (0, 0, over_cells[2] - lid_lifted[2]), HOLDER_HOLD, "up above the cells' height",
                 rot=rot),
        Waypoint(2.0, over_cells, HOLDER_HOLD, "over the cells", rot=rot),
        Waypoint(3.0, on_cells, HOLDER_HOLD, "onto the cells", rot=rot),
        Waypoint(0.8, on_cells, HOLDER_OPEN, "let go of the top holder", rot=rot),
        Waypoint(1.0, on_cells + (0, 0, 0.03), HOLDER_OPEN, "up from the pack", rot=rot),
    ]
    return wps + home


def demo():
    """Both arms: the left puts the bottom holder into the jig while the right takes the first cell; the right fills
    the four cups; the left puts the top holder on. Both go home."""
    right = right_plan()
    clear_s = sum(w.duration for w in right) - 2.5  # the right arm has left the last cup
    return {"left": left_plan(lid_at_s=clear_s), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
