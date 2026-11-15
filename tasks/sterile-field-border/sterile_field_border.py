"""Sterile field with an unsterile border: four surgical instruments, laid out on the field and kept sterile.

A scrub nurse or surgical technologist lays a basic soft-tissue set out on a sterile back table. Four real instruments
at true size lie on a sterile rack beside the field: Metzenbaum scissors (7 in, 180 mm), Rochester-Pean forceps
(6.25 in, 160 mm, closed on the ratchet), a no. 4 scalpel handle (135 mm, no blade) and a Mayo-Hegar needle holder
(6 in, 152 mm). They have to end up on the field's blue sterile area, flat and apart from each other.

What is sterile at the start: the four instruments, both grippers' finger pads (as if in sterile covers), the rack's
tray and rests, and the field's blue inside. What is not: the tabletop (everywhere outside the panel and the rack) and
the field's orange border band (its outer inch; the drape's edge is not sterile). An instrument is contaminated, for
good, when any point of its surface touches a nonsterile surface (within 1.5 mm), or when a gripper that has itself
touched one (its pads, likewise) touches it. Contaminated instruments cannot score and 1.0 is out of reach; a red tag
appears on the instrument (a visual only). Carrying one over the border without touching it is fine.

The drape is cloth in the operating theatre; here it is a flat rigid panel with the border drawn on it (see the README).

Run it:  inspect-robots run --embodiment yam_arena -E task=sterile_field_border.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models. This file extends it without changing
yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with the physics authored there
(``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim starts), and ``Waypoint.rot``
gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
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
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "sterile_field_border"

# -- the field, the rack and the instruments (m; x forward, y to the robot's left, from the midpoint between the arms) --
FIELD_C = (0.40, 0.14)  # the sterile field panel's centre
FIELD_OUT = (0.30, 0.44)  # its outer size: x (depth), y (width); about a Mayo stand tray's 300 x 480 mm
BORDER = 25.4 * MM  # the unsterile border: 1 inch all round
DRAPE_T = 3 * MM  # the panel's thickness (its top face is the surface the instruments lie on)
DRAPE_TOP = TOP + DRAPE_T
INNER = (FIELD_OUT[0] / 2 - BORDER, FIELD_OUT[1] / 2 - BORDER)  # the sterile area's half sizes
RACK_C = (0.35, 0.50)  # the rack's centre
RACK_PLATE, REST_H = 4 * MM, 8 * MM  # the rack's tray, and its rests above the tray
REST_TOP = TOP + RACK_PLATE + REST_H  # the instruments' underside on the rack
PITCH = 58 * MM  # instrument spacing on the rack (x)
LANE = 84 * MM  # instrument spacing on the field (y)

# Instruments, in rack order (increasing x). The rack holds them across (long axis along y), tips away from the field
# (+y); on the field they lie along x with the tips away from the robot (+x), in the lanes below.
NAMES = ("needle_holder", "metzenbaum_scissors", "pean_forceps", "scalpel_handle")
RACK_X = {n: RACK_C[0] + (k - 1.5) * PITCH for k, n in enumerate(NAMES)}
LANE_ORDER = ("pean_forceps", "needle_holder", "metzenbaum_scissors", "scalpel_handle")  # lane k, from low y to high y
LANE_Y = {n: FIELD_C[1] + (k - 1.5) * LANE for k, n in enumerate(LANE_ORDER)}
MASS = {"metzenbaum_scissors": 0.050, "pean_forceps": 0.060, "scalpel_handle": 0.022, "needle_holder": 0.070}
START_NUDGE = (0.0, 0.0, 0.0)  # a check of the demo's tolerance: every instrument's start moved (m, m, rad); 0 in the task

# --- generated from the asset geometry (assets/*.usda): each instrument's convex pieces, in mm, in its own frame
# (origin at its centre of mass on the underside, +x toward the tip): (outline, bottom z, top z) ---
PIECES = {
    "metzenbaum_scissors": (
        (((-50.63, 17.35), (-54.27, 15.25), (-53.29, 11.60), (-49.09, 11.60)), 0.0, 3.8),
        (((-54.84, 21.56), (-56.94, 17.92), (-54.27, 15.25), (-50.63, 17.35)), 0.0, 3.8),
        (((-54.84, 21.56), (-60.59, 23.10), (-60.59, 18.90), (-56.94, 17.92)), 0.0, 3.8),
        (((-60.59, 23.10), (-66.34, 21.56), (-64.24, 17.92), (-60.59, 18.90)), 0.0, 3.8),
        (((-66.34, 21.56), (-70.55, 17.35), (-66.92, 15.25), (-64.24, 17.92)), 0.0, 3.8),
        (((-70.55, 17.35), (-72.09, 11.60), (-67.89, 11.60), (-66.92, 15.25)), 0.0, 3.8),
        (((-72.09, 11.60), (-70.55, 5.85), (-66.92, 7.95), (-67.89, 11.60)), 0.0, 3.8),
        (((-70.55, 5.85), (-66.34, 1.64), (-64.24, 5.28), (-66.92, 7.95)), 0.0, 3.8),
        (((-66.34, 1.64), (-60.59, 0.10), (-60.59, 4.30), (-64.24, 5.28)), 0.0, 3.8),
        (((-60.59, 4.30), (-60.59, 0.10), (-54.84, 1.64), (-56.94, 5.28)), 0.0, 3.8),
        (((-56.94, 5.28), (-54.84, 1.64), (-50.63, 5.85), (-54.27, 7.95)), 0.0, 3.8),
        (((-54.27, 7.95), (-50.63, 5.85), (-49.09, 11.60), (-53.29, 11.60)), 0.0, 3.8),
        (((-49.79, 12.51), (-50.69, 7.18), (7.46, -2.66), (8.36, 2.66)), 0.0, 6.4),
        (((-50.63, -5.85), (-54.27, -7.95), (-53.29, -11.60), (-49.09, -11.60)), 0.0, 3.8),
        (((-54.84, -1.64), (-56.94, -5.28), (-54.27, -7.95), (-50.63, -5.85)), 0.0, 3.8),
        (((-54.84, -1.64), (-60.59, -0.10), (-60.59, -4.30), (-56.94, -5.28)), 0.0, 3.8),
        (((-60.59, -0.10), (-66.34, -1.64), (-64.24, -5.28), (-60.59, -4.30)), 0.0, 3.8),
        (((-66.34, -1.64), (-70.55, -5.85), (-66.92, -7.95), (-64.24, -5.28)), 0.0, 3.8),
        (((-70.55, -5.85), (-72.09, -11.60), (-67.89, -11.60), (-66.92, -7.95)), 0.0, 3.8),
        (((-72.09, -11.60), (-70.55, -17.35), (-66.92, -15.25), (-67.89, -11.60)), 0.0, 3.8),
        (((-70.55, -17.35), (-66.34, -21.56), (-64.24, -17.92), (-66.92, -15.25)), 0.0, 3.8),
        (((-66.34, -21.56), (-60.59, -23.10), (-60.59, -18.90), (-64.24, -17.92)), 0.0, 3.8),
        (((-60.59, -18.90), (-60.59, -23.10), (-54.84, -21.56), (-56.94, -17.92)), 0.0, 3.8),
        (((-56.94, -17.92), (-54.84, -21.56), (-50.63, -17.35), (-54.27, -15.25)), 0.0, 3.8),
        (((-54.27, -15.25), (-50.63, -17.35), (-49.09, -11.60), (-53.29, -11.60)), 0.0, 3.8),
        (((-50.69, -7.18), (-49.79, -12.51), (8.36, -2.66), (7.46, 2.66)), 0.0, 6.4),
        (((3.29, 1.91), (3.29, -1.91), (5.99, -4.62), (9.82, -4.62), (12.53, -1.91), (12.53, 1.91), (9.82, 4.62), (5.99, 4.62)), 0.0, 7.2),
        (((7.91, -4.40), (52.91, -3.40), (52.91, 3.40), (7.91, 4.40)), 0.0, 6.4),
        (((52.91, 3.40), (52.91, -3.40), (105.91, -1.40), (107.91, -0.00), (105.91, 1.40)), 0.0, 6.4),
    ),
    "pean_forceps": (
        (((-51.17, 16.70), (-54.64, 14.70), (-53.70, 11.20), (-49.70, 11.20)), 0.0, 3.8),
        (((-55.20, 20.73), (-57.20, 17.26), (-54.64, 14.70), (-51.17, 16.70)), 0.0, 3.8),
        (((-55.20, 20.73), (-60.70, 22.20), (-60.70, 18.20), (-57.20, 17.26)), 0.0, 3.8),
        (((-60.70, 22.20), (-66.20, 20.73), (-64.20, 17.26), (-60.70, 18.20)), 0.0, 3.8),
        (((-66.20, 20.73), (-70.22, 16.70), (-66.76, 14.70), (-64.20, 17.26)), 0.0, 3.8),
        (((-70.22, 16.70), (-71.70, 11.20), (-67.70, 11.20), (-66.76, 14.70)), 0.0, 3.8),
        (((-71.70, 11.20), (-70.22, 5.70), (-66.76, 7.70), (-67.70, 11.20)), 0.0, 3.8),
        (((-70.22, 5.70), (-66.20, 1.67), (-64.20, 5.14), (-66.76, 7.70)), 0.0, 3.8),
        (((-66.20, 1.67), (-60.70, 0.20), (-60.70, 4.20), (-64.20, 5.14)), 0.0, 3.8),
        (((-60.70, 4.20), (-60.70, 0.20), (-55.20, 1.67), (-57.20, 5.14)), 0.0, 3.8),
        (((-57.20, 5.14), (-55.20, 1.67), (-51.17, 5.70), (-54.64, 7.70)), 0.0, 3.8),
        (((-54.64, 7.70), (-51.17, 5.70), (-49.70, 11.20), (-53.70, 11.20)), 0.0, 3.8),
        (((-50.40, 12.06), (-51.28, 6.94), (3.86, -2.56), (4.74, 2.56)), 0.0, 7.0),
        (((-51.17, -5.70), (-54.64, -7.70), (-53.70, -11.20), (-49.70, -11.20)), 0.0, 3.8),
        (((-55.20, -1.67), (-57.20, -5.14), (-54.64, -7.70), (-51.17, -5.70)), 0.0, 3.8),
        (((-55.20, -1.67), (-60.70, -0.20), (-60.70, -4.20), (-57.20, -5.14)), 0.0, 3.8),
        (((-60.70, -0.20), (-66.20, -1.67), (-64.20, -5.14), (-60.70, -4.20)), 0.0, 3.8),
        (((-66.20, -1.67), (-70.22, -5.70), (-66.76, -7.70), (-64.20, -5.14)), 0.0, 3.8),
        (((-70.22, -5.70), (-71.70, -11.20), (-67.70, -11.20), (-66.76, -7.70)), 0.0, 3.8),
        (((-71.70, -11.20), (-70.22, -16.70), (-66.76, -14.70), (-67.70, -11.20)), 0.0, 3.8),
        (((-70.22, -16.70), (-66.20, -20.73), (-64.20, -17.26), (-66.76, -14.70)), 0.0, 3.8),
        (((-66.20, -20.73), (-60.70, -22.20), (-60.70, -18.20), (-64.20, -17.26)), 0.0, 3.8),
        (((-60.70, -18.20), (-60.70, -22.20), (-55.20, -20.73), (-57.20, -17.26)), 0.0, 3.8),
        (((-57.20, -17.26), (-55.20, -20.73), (-51.17, -16.70), (-54.64, -14.70)), 0.0, 3.8),
        (((-54.64, -14.70), (-51.17, -16.70), (-49.70, -11.20), (-53.70, -11.20)), 0.0, 3.8),
        (((-51.28, -6.94), (-50.40, -12.06), (4.74, -2.56), (3.86, 2.56)), 0.0, 7.0),
        (((-2.70, -4.50), (11.30, -4.50), (11.30, 4.50), (-2.70, 4.50)), 0.0, 8.0),
        (((11.30, -4.80), (53.30, -4.80), (53.30, 4.80), (11.30, 4.80)), 0.0, 7.0),
        (((53.30, 4.80), (53.30, -4.80), (88.30, -1.20), (88.30, 1.20)), 0.0, 7.0),
    ),
    "scalpel_handle": (
        (((-58.80, 4.50), (-58.80, -4.50), (-58.30, -5.00), (39.70, -5.00), (40.20, -4.50), (40.20, 4.50), (39.70, 5.00), (-58.30, 5.00)), 0.0, 6.0),
        (((40.20, -5.00), (66.20, -3.00), (66.20, 3.00), (40.20, 5.00)), 0.0, 4.4),
        (((66.20, -2.20), (76.20, -2.20), (76.20, 2.20), (66.20, 2.20)), 0.0, 2.4),
    ),
    "needle_holder": (
        (((-51.60, 16.70), (-55.06, 14.70), (-54.12, 11.20), (-50.12, 11.20)), 0.0, 3.8),
        (((-55.62, 20.73), (-57.62, 17.26), (-55.06, 14.70), (-51.60, 16.70)), 0.0, 3.8),
        (((-55.62, 20.73), (-61.12, 22.20), (-61.12, 18.20), (-57.62, 17.26)), 0.0, 3.8),
        (((-61.12, 22.20), (-66.62, 20.73), (-64.62, 17.26), (-61.12, 18.20)), 0.0, 3.8),
        (((-66.62, 20.73), (-70.65, 16.70), (-67.19, 14.70), (-64.62, 17.26)), 0.0, 3.8),
        (((-70.65, 16.70), (-72.12, 11.20), (-68.12, 11.20), (-67.19, 14.70)), 0.0, 3.8),
        (((-72.12, 11.20), (-70.65, 5.70), (-67.19, 7.70), (-68.12, 11.20)), 0.0, 3.8),
        (((-70.65, 5.70), (-66.62, 1.67), (-64.62, 5.14), (-67.19, 7.70)), 0.0, 3.8),
        (((-66.62, 1.67), (-61.12, 0.20), (-61.12, 4.20), (-64.62, 5.14)), 0.0, 3.8),
        (((-61.12, 4.20), (-61.12, 0.20), (-55.62, 1.67), (-57.62, 5.14)), 0.0, 3.8),
        (((-57.62, 5.14), (-55.62, 1.67), (-51.60, 5.70), (-55.06, 7.70)), 0.0, 3.8),
        (((-55.06, 7.70), (-51.60, 5.70), (-50.12, 11.20), (-54.12, 11.20)), 0.0, 3.8),
        (((-50.78, 12.15), (-51.79, 6.64), (-0.63, -2.75), (0.38, 2.75)), 0.0, 7.4),
        (((-51.60, -5.70), (-55.06, -7.70), (-54.12, -11.20), (-50.12, -11.20)), 0.0, 3.8),
        (((-55.62, -1.67), (-57.62, -5.14), (-55.06, -7.70), (-51.60, -5.70)), 0.0, 3.8),
        (((-55.62, -1.67), (-61.12, -0.20), (-61.12, -4.20), (-57.62, -5.14)), 0.0, 3.8),
        (((-61.12, -0.20), (-66.62, -1.67), (-64.62, -5.14), (-61.12, -4.20)), 0.0, 3.8),
        (((-66.62, -1.67), (-70.65, -5.70), (-67.19, -7.70), (-64.62, -5.14)), 0.0, 3.8),
        (((-70.65, -5.70), (-72.12, -11.20), (-68.12, -11.20), (-67.19, -7.70)), 0.0, 3.8),
        (((-72.12, -11.20), (-70.65, -16.70), (-67.19, -14.70), (-68.12, -11.20)), 0.0, 3.8),
        (((-70.65, -16.70), (-66.62, -20.73), (-64.62, -17.26), (-67.19, -14.70)), 0.0, 3.8),
        (((-66.62, -20.73), (-61.12, -22.20), (-61.12, -18.20), (-64.62, -17.26)), 0.0, 3.8),
        (((-61.12, -18.20), (-61.12, -22.20), (-55.62, -20.73), (-57.62, -17.26)), 0.0, 3.8),
        (((-57.62, -17.26), (-55.62, -20.73), (-51.60, -16.70), (-55.06, -14.70)), 0.0, 3.8),
        (((-55.06, -14.70), (-51.60, -16.70), (-50.12, -11.20), (-54.12, -11.20)), 0.0, 3.8),
        (((-51.79, -6.64), (-50.78, -12.15), (0.38, -2.75), (-0.63, 2.75)), 0.0, 7.4),
        (((-7.12, -5.00), (6.88, -5.00), (6.88, 5.00), (-7.12, 5.00)), 0.0, 8.4),
        (((6.88, -5.20), (45.88, -5.20), (45.88, 5.20), (6.88, 5.20)), 0.0, 7.6),
        (((45.88, 5.20), (45.88, -5.20), (79.88, -3.00), (79.88, 3.00)), 0.0, 7.6),
    ),
}
# --- end generated ---


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(r))


def _extent(name: str) -> tuple[np.ndarray, np.ndarray]:
    pts = np.array([p for poly, _z0, _z1 in PIECES[name] for p in poly], float) * MM
    return pts.min(axis=0), pts.max(axis=0)


def _height(name: str) -> float:
    return max(z1 for _p, _z0, z1 in PIECES[name]) * MM


# Each instrument's bounds in its own frame (low, high corners), for the grip and release checks.
BOXES = {n: (np.array([*_extent(n)[0], 0.0]), np.array([*_extent(n)[1], _height(n)])) for n in NAMES}

START_YAW = math.pi / 2  # in the rack: the tip toward +y (away from the field)
START = {n: np.array([RACK_X[n], RACK_C[1], REST_TOP]) for n in NAMES}
# Where each instrument belongs (its origin, on the underside) with its footprint centred on the field's x, tip toward +x.
PLACE = {n: np.array([FIELD_C[0] - 0.5 * (BOXES[n][0][0] + BOXES[n][1][0]), LANE_Y[n], DRAPE_TOP]) for n in NAMES}
PLACE_YAW = 0.0


def _start_pose(name):
    pos = START[name] + np.array([START_NUDGE[0], START_NUDGE[1], 0.0])
    return pos, _xyzw(_rz(START_YAW + START_NUDGE[2]))


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body, or a fixture
    (``kinematic``). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed_base: bool = False
    kinematic: bool = False
    no_gravity: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed_base)


def _instrument(name: str) -> UsdPart:
    pos, rot = _start_pose(name)
    lo, hi = _extent(name)
    return UsdPart(name, asset=name, xy=(float(pos[0]), float(pos[1])), extent=float(max(hi - lo)), mass=MASS[name],
                   usd_path=str(ASSETS / f"{name}.usda"), pos=tuple(float(v) for v in pos), rot_xyzw=rot)


FLAG_HIDDEN = np.array([0.30, -0.05, TOP - 0.03])  # a flag's waiting place, inside the table top
FIELD = UsdPart("field", asset="field", xy=FIELD_C, extent=FIELD_OUT[1], mass=None,
                usd_path=str(ASSETS / "sterile_field.usda"), pos=(*FIELD_C, TOP), kinematic=True)
RACK = UsdPart("rack", asset="rack", xy=RACK_C, extent=0.25, mass=None, usd_path=str(ASSETS / "instrument_rack.usda"),
               pos=(*RACK_C, TOP), kinematic=True)
FLAGS = {n: UsdPart(f"flag_{n}", asset="flag", xy=(float(FLAG_HIDDEN[0]), float(FLAG_HIDDEN[1])), extent=0.03,
                    mass=0.001, usd_path=str(ASSETS / "contam_flag.usda"), pos=tuple(float(v) for v in FLAG_HIDDEN),
                    no_gravity=True) for n in NAMES}
INSTRUMENTS = [_instrument(n) for n in NAMES]
OBJECTS = [FIELD, RACK, *INSTRUMENTS, *FLAGS.values()]

MAX_DEPENETRATION = 0.2  # m/s: thin steel pinched by its flanks stays calm


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    if spec.no_gravity:
        props = sim_utils.RigidBodyPropertiesCfg(disable_gravity=True)
    else:
        props = sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon={"rigid_props": props},
    )


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): the span of its finger pads along its pointing axis (``back``
    to the pad ``tips``), the axis its fingers close along (``closing``, a unit vector), how far apart the pads are,
    and the smaller of the two fingers' grip forces (N): each finger's drive stiffness times how far something holds it
    back from its commanded position."""

    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float
    closing: np.ndarray = None


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
            closing=np.asarray(frame[:, 0], float),
        )
    return out


def _show_flags(base, origin, objects) -> None:
    """Keeps each contamination flag on its instrument (a visual only: no collider, no gravity) once that instrument has
    been contaminated, and out of sight under the table before."""
    import torch

    for n in NAMES:
        want = flag_pose(objects, n)
        flag = objects[f"flag_{n}"]
        if (np.linalg.norm(flag.pos - want[0]) < 1e-4
                and abs(float(np.dot(np.asarray(flag.quat_xyzw, float), want[1]))) > 1 - 1e-8):
            continue
        asset = base.scene[f"flag_{n}"]
        pose = asset.data.root_link_pose_w.torch.clone()
        pose[:, :3] = torch.as_tensor(np.asarray(want[0], float) + origin, dtype=pose.dtype, device=pose.device)
        pose[:, 3:] = torch.as_tensor(np.asarray(want[1], float), dtype=pose.dtype, device=pose.device)
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(asset.data.default_root_vel.torch))


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root pose and velocities, plus the grippers and the episode time. Also keeps the contamination
        flags where they belong."""
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
        if out["episode_time"] < _PROGRESS.time:  # a new episode: forget the last one's contacts before showing them
            _PROGRESS.__init__()
        _show_flags(base, origin, out)
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
TOUCH_Z = 1.5 * MM  # a surface point this close to a nonsterile surface (tabletop, border band) touches it
PAD_TOUCH = 2 * MM  # a finger pad's face this close to an instrument's box touches it
RACK_HALF = (0.125, 0.10)  # the rack tray's half sizes (x, y): its tray and rests are sterile
LIFTED = 0.01  # m: an instrument held by a gripper this far above where it started has been picked up
FLAT = math.radians(5)  # lying flat: its upright axis within this of vertical
APART = 8 * MM  # each instrument at least this far from the others
RESTING = 2 * MM  # lying on the panel: its lowest point within this of the top face
FALLEN_Z = -0.02  # m above the table: an instrument whose origin is lower has fallen off the table
CLEAR = 0.03  # m: both grippers at least this far from every instrument (a clean release)
GRIP_SLACK = 0.0025
GRIP_FORCE = 0.5
GRIP_OPENING = 0.002
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)  # settled: within 2 mm and 2 degrees over 1 s
TRACE_S = 1.0  # s between trace lines in a run
STEPS = tuple(f"{n.replace('_', ' ')} laid on the sterile field, still sterile" for n in NAMES)
POINTS = 0.2  # per instrument; only success scores 1.0


def _surface(name: str) -> tuple[np.ndarray, np.ndarray]:
    """Points on an instrument's surface (instrument frame, m): ``all`` over every piece's top and bottom faces (a grid
    about 4 mm apart, and the outline every 2 mm) and its vertical edges; ``low`` those on the underside."""
    allp, low = [], []
    for poly, z0, z1 in PIECES[name]:
        p = np.asarray(poly, float)
        m = len(p)
        lo, hi = p.min(axis=0), p.max(axis=0)
        gx, gy = np.meshgrid(np.arange(lo[0], hi[0] + 1e-9, 4.0), np.arange(lo[1], hi[1] + 1e-9, 4.0))
        grid = np.column_stack([gx.ravel(), gy.ravel()])
        inside = np.ones(len(grid), bool)
        for i in range(m):  # outline is counter-clockwise: inside is to the left of every edge
            a, b = p[i], p[(i + 1) % m]
            inside &= (b[0] - a[0]) * (grid[:, 1] - a[1]) - (b[1] - a[1]) * (grid[:, 0] - a[0]) >= -1e-9
        pts = [grid[inside]]
        for i in range(m):
            a, b = p[i], p[(i + 1) % m]
            n = max(1, int(np.ceil(np.linalg.norm(b - a) / 2.0)))
            pts.append(a + (b - a) * (np.arange(n)[:, None] / n))
        xy = np.vstack(pts)
        low.append(np.column_stack([xy, np.full(len(xy), z0)]))
        allp.append(np.column_stack([xy, np.full(len(xy), z0)]))
        allp.append(np.column_stack([xy, np.full(len(xy), z1)]))
        for i in range(m):  # vertical edges
            allp.append(np.array([[p[i][0], p[i][1], z] for z in np.linspace(z0, z1, 4)]))
    return np.vstack(allp) * MM, np.vstack(low) * MM


SURF = {n: _surface(n) for n in NAMES}


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _world(objects, name, pts) -> np.ndarray:
    s = objects[name]
    return pts @ _rot(s).T + np.asarray(s.pos, float)


def in_inner(xy, margin: float = 0.0) -> np.ndarray:
    """Points (x, y) lying on the sterile area (the panel inside its border band), ``margin`` inside its edge."""
    d = np.abs(np.asarray(xy, float) - np.asarray(FIELD_C))
    return (d[:, 0] <= INNER[0] - margin) & (d[:, 1] <= INNER[1] - margin)


def on_band(xy) -> np.ndarray:
    """Points (x, y) over the border's band: on the panel but outside its sterile area."""
    d = np.abs(np.asarray(xy, float) - np.asarray(FIELD_C))
    on_panel = (d[:, 0] <= FIELD_OUT[0] / 2) & (d[:, 1] <= FIELD_OUT[1] / 2)
    return on_panel & ~in_inner(xy)


def on_rack(xy) -> np.ndarray:
    """Points (x, y) over the sterile rack's tray."""
    d = np.abs(np.asarray(xy, float) - np.asarray(RACK_C))
    return (d[:, 0] <= RACK_HALF[0]) & (d[:, 1] <= RACK_HALF[1])


def nonsterile_hit(points) -> np.ndarray:
    """Which of ``points`` (world, N x 3) touch a nonsterile surface: the border band's top face (within TOUCH_Z over
    the band), or the bare tabletop (within TOUCH_Z of it anywhere outside the panel and the rack tray). The field's
    blue inside and the rack are sterile."""
    w = np.asarray(points, float)
    xy = w[:, :2]
    band = on_band(xy) & (w[:, 2] <= DRAPE_TOP + TOUCH_Z)
    d = np.abs(xy - np.asarray(FIELD_C))
    off_panel = ~((d[:, 0] <= FIELD_OUT[0] / 2) & (d[:, 1] <= FIELD_OUT[1] / 2))
    table = off_panel & ~on_rack(xy) & (w[:, 2] <= TOP + TOUCH_Z)
    return band | table


def touching_nonsterile(objects, name) -> bool:
    """Some point of the instrument's surface touches a nonsterile surface (the border band or the bare tabletop)."""
    return bool(np.any(nonsterile_hit(_world(objects, name, SURF[name][0]))))


def pad_points(gripper) -> np.ndarray:
    """Points on a gripper's two finger pad faces (world, m), along the pads from their back to their tips."""
    if gripper is None or gripper.closing is None:
        return np.zeros((0, 3))
    half = 0.5 * gripper.opening * np.asarray(gripper.closing, float)
    pts = []
    for f in np.linspace(0.0, 1.0, 11):
        mid = gripper.back + f * (gripper.tips - gripper.back)
        pts += [mid + half, mid - half]
    return np.array(pts)


def gripper_touching_nonsterile(gripper) -> bool:
    """A finger pad touches the border band or the bare tabletop."""
    p = pad_points(gripper)
    return bool(len(p) and np.any(nonsterile_hit(p)))


def pad_touches(name, state, gripper) -> bool:
    """A finger pad's face touches the instrument (comes within PAD_TOUCH of its box)."""
    p = pad_points(gripper)
    if not len(p):
        return False
    q = (p - np.asarray(state.pos, float)) @ _rot(state)
    low, high = BOXES[name][0] - PAD_TOUCH, BOXES[name][1] + PAD_TOUCH
    return bool(np.any(np.all((q >= low) & (q <= high), axis=1)))


def _held_by(name, state, gripper) -> bool:
    """``gripper`` holds the instrument: both finger pads' faces lie on its box (grown by GRIP_SLACK) at the same place
    along the pads, and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING or gripper.closing is None:
        return False
    r, pos = _rot(state), np.asarray(state.pos, float)
    low, high = BOXES[name][0] - GRIP_SLACK, BOXES[name][1] + GRIP_SLACK
    half = 0.5 * gripper.opening * np.asarray(gripper.closing, float)
    for f in np.linspace(0.0, 1.0, 11):
        mid = gripper.back + f * (gripper.tips - gripper.back)
        faces = [r.T @ (mid + s * half - pos) for s in (1.0, -1.0)]
        if all(bool(np.all(q >= low) and np.all(q <= high)) for q in faces):
            return True
    return False


def held(objects, name) -> bool:
    return any(_held_by(name, objects[name], objects.get(g)) for g in ("left_gripper", "right_gripper"))


def laid(objects, name) -> dict:
    """How instrument ``name`` lies: ``inside`` (every surface point on the sterile area), ``flat``, ``resting`` (its lowest point within RESTING of the panel's top face) and ``gap``
    (its 3-D distance from the nearest other instrument's underside, m)."""
    w = _world(objects, name, SURF[name][1])
    r = _rot(objects[name])
    upright = float(np.arccos(np.clip(r[2, 2], -1.0, 1.0)))
    gap = 1e9
    for other in NAMES:
        if other != name:
            o = _world(objects, other, SURF[other][1])
            d2 = ((w[::3, None, :] - o[None, ::3, :]) ** 2).sum(axis=2)
            gap = min(gap, float(np.sqrt(d2.min())))
    return {"inside": bool(np.all(in_inner(w[:, :2]))), "flat": upright,
            "resting": float(w[:, 2].min() - DRAPE_TOP), "gap": gap}


def placed(objects, name) -> bool:
    """The instrument lies on the sterile area: all of it inside, flat, resting on the panel, not in a gripper, and clear
    of the other instruments."""
    o = laid(objects, name)
    if held(objects, name):
        return False  # still in a gripper
    return bool(o["inside"] and o["flat"] <= FLAT and o["resting"] <= RESTING
                and o["gap"] >= APART)


def released(objects) -> bool:
    """Both grippers are at least CLEAR from every instrument (their pads' span outside each instrument's box grown by
    CLEAR)."""
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None:
            continue
        for name in NAMES:
            r, pos = _rot(objects[name]), np.asarray(objects[name].pos, float)
            low, high = BOXES[name][0] - CLEAR, BOXES[name][1] + CLEAR
            for f in np.linspace(0.0, 1.0, 11):
                q = r.T @ (gripper.back + f * (gripper.tips - gripper.back) - pos)
                if bool(np.all(q >= low) and np.all(q <= high)):
                    return False
    return True


def _poses(objects) -> dict:
    return {n: (np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in NAMES}


def still(objects, anchor) -> bool:
    """Every instrument within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses, not the
    simulator's instantaneous speeds."""
    for n, (pos, quat) in _poses(objects).items():
        pos0, quat0 = anchor[n]
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: which instruments a gripper has picked up,
    which instruments and grippers have been contaminated by a nonsterile surface (latched), and since when every full-success condition has held without a break
    (``met_since``, ``None`` while one fails) with the poses when they began to hold (``anchor``). Any lapse restarts the
    count. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.picked = {n: False for n in NAMES}
        self.touched = {n: False for n in NAMES}  # instrument contaminated (latched)
        self.pad_soiled = {"left_gripper": False, "right_gripper": False}  # gripper pads have touched a nonsterile surface
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        for n in NAMES:
            if objects[n].pos[2] > START[n][2] + LIFTED and held(objects, n):
                self.picked[n] = True
        tt = t if t is not None else 0.0
        for g in self.pad_soiled:
            if not self.pad_soiled[g] and gripper_touching_nonsterile(objects.get(g)):
                self.pad_soiled[g] = True
                print(f"[{TAG}] t={tt:.2f}s  {g.replace('_', ' ')} pads touched a nonsterile surface: "
                      "contaminated for good", flush=True)
        for n in NAMES:
            if self.touched[n]:
                continue
            why = ("touched a nonsterile surface" if touching_nonsterile(objects, n) else
                   "was touched by a contaminated gripper" if any(
                       self.pad_soiled[g] and pad_touches(n, objects[n], objects.get(g)) for g in self.pad_soiled)
                   else "")
            if why:
                self.touched[n] = True
                print(f"[{TAG}] t={tt:.2f}s  {n.replace('_', ' ')} {why}: contaminated for good", flush=True)
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not _violation(objects) and released(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def flag_pose(objects, name) -> tuple[np.ndarray, np.ndarray]:
    """Where instrument ``name``'s contamination flag belongs (world position, quaternion xyzw): on the instrument, once
    it has been contaminated, or out of sight under the table before."""
    if not _PROGRESS.touched[name]:
        return FLAG_HIDDEN.copy(), np.array([0.0, 0.0, 0.0, 1.0])
    s = objects[name]
    r = _rot(s)
    yaw = _rz(math.atan2(float(r[1, 0]), float(r[0, 0])))
    return (np.asarray(s.pos, float) + np.array([0.0, 0.0, _height(name) + 0.001]),
            np.array(_xyzw(yaw), float))


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand, from the episode's progress so far (see ``steps_done``)."""
    p = _PROGRESS
    return [bool(p.picked[n] and not p.touched[n] and placed(objects, n)) for n in NAMES]


def steps_done(objects) -> list[bool]:
    """Instrument by instrument: it counts once a gripper has picked it up, only while it is still sterile
    (contamination by a nonsterile surface, directly or through a contaminated gripper, is for good), and only while it
    lies on the sterile area as ``placed`` says."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        parts = []
        for n in NAMES:
            o = laid(objects, n)
            parts.append(f"{n.replace('_', ' ')} {'in' if o['inside'] else 'OUT'}, gap {min(o['gap'], 0.99) * 1000:.0f} mm, "
                         f"{'contaminated' if p.touched[n] else 'sterile'}"
                         f"{', held' if held(objects, n) else ''}")
        line = f"({'; '.join(parts)})"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: an instrument off the table, or a state that is not finite."""
    for n in NAMES:
        pos = np.asarray(objects[n].pos, float)
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S: all four instruments laid on the sterile area,
    none contaminated at any time, both grippers clear, nothing off the table, and every instrument within
    STILL_MOVE and STILL_TURN of where it was when that second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per instrument laid clean (at most
    0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left arm, one instrument at a time: from above, its fingers closing across the instrument's shank at its centre of
# mass (the pads reach down between the rack's rests), lifts it 8 cm clear, carries it over the border to its lane while
# the wrist turns the instrument a quarter turn (so it lies along x in its lane; the heading is not scored), lowers it to
# 3 mm above the sterile area, lets go and lifts away. The pads and the instrument never come near the tabletop or the band. Then the next. At the end the arm goes home. Open loop: no pose is read.
SQUEEZE = 0.008  # the grip closes the fingers this far inside the instrument's width (about 20 N a finger)
PICK_OPEN = 0.012  # m: each finger opens this far beyond the instrument before the grip
PAD_BELOW = 0.002  # m: the pad tips reach this far below the instrument's underside
LIFT = 0.06  # m: carry height above the underside's start
RELEASE_GAP = 0.003  # m: the instrument is let go with its underside this far above the panel
LET_GO = 0.006  # m: each finger opens this far to let go


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


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


def _finger(width: float, extra: float = 0.0) -> float:
    """The finger target that holds ``width`` (``SQUEEZE`` inside it), or ``extra`` open beyond it."""
    return min(0.0, -(width - SQUEEZE) / 2) if extra == 0.0 else max(FINGER_OPEN, -(width / 2 + extra))


GRASP_DX = {"metzenbaum_scissors": 0.008, "pean_forceps": 0.0043}  # m along the instrument from its centre of mass: the grip is on the pivot screw


def _grip_width(name: str) -> float:
    """The instrument's width where it is gripped (m): the span of the pieces' outlines at its grasp point."""
    x0 = GRASP_DX.get(name, 0.0) / MM
    ys = []
    for poly, _z0, _z1 in PIECES[name]:
        p = np.asarray(poly, float)
        for i in range(len(p)):
            a, b = p[i], p[(i + 1) % len(p)]
            if (a[0] <= x0 <= b[0] or b[0] <= x0 <= a[0]) and abs(b[0] - a[0]) > 1e-9:
                ys.append(a[1] + (b[1] - a[1]) * (x0 - a[0]) / (b[0] - a[0]))
    return (max(ys) - min(ys)) * MM


GRIP_W = {n: _grip_width(n) for n in NAMES}
# In the rack the instrument lies along y (tip toward +y) and the fingers close along x; on the field it lies along x
# and they close along y: a quarter turn of the gripper about the vertical.
PICK_ROT = yam_demo.grip_rotation(math.pi / 2)
PLACE_ROT = yam_demo.grip_rotation(0.0)
CARRY_ROT = yam_demo.grip_rotation(math.pi / 4)
ORDER = ("needle_holder", "scalpel_handle", "metzenbaum_scissors", "pean_forceps")  # the order the demo lays them out in

_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


class _Arm:
    """A waypoint list being written, with its running time."""

    def __init__(self, side: str):
        self.side, self.wps, self.t = side, [], 0.0
        self.pos = yam_demo.ArmPlan([], BASES[side]).start_pos
        self.rot = yam_demo.ArmPlan([], BASES[side]).start_rot
        self.finger = FINGER_OPEN

    def go(self, duration, pos=None, finger=None, label="", rot=None):
        self.pos = self.pos if pos is None else np.asarray(pos, float)
        self.finger = self.finger if finger is None else finger
        self.rot = self.rot if rot is None else rot
        self.wps.append(Waypoint(duration, self.pos.copy(), self.finger, label, rot=self.rot))
        self.t += duration
        return self.t


def plan(order=ORDER, home: bool = True) -> dict:
    """The left arm's waypoints for laying the instruments in ``order`` out on their lanes."""
    left = _Arm("left")
    for k, name in enumerate(order):
        w = GRIP_W[name]
        label = name.replace("_", " ")
        pick = _tcp_with_tips(START[name] + _rz(START_YAW) @ np.array([GRASP_DX.get(name, 0.0), 0.0, 0.0]), PAD_BELOW, PICK_ROT)
        left.go(3.0 if k == 0 else 2.0, pick + (0, 0, 0.07), _finger(w, PICK_OPEN), f"over the {label}", PICK_ROT)
        left.go(1.5, pick, label=f"down to the {label}")
        left.go(0.6, finger=_finger(w), label=f"grip the {label}")
        left.go(1.2, pick + (0, 0, LIFT), label=f"lift the {label}")
        place = _tcp_with_tips(PLACE[name] + np.array([GRASP_DX.get(name, 0.0), 0.0, RELEASE_GAP]), PAD_BELOW, PLACE_ROT)
        mid = (pick + place) / 2 + np.array([0.0, 0.0, LIFT])
        left.go(1.8, mid, label="carry over the border", rot=CARRY_ROT)
        left.go(1.8, place + (0, 0, LIFT), label=f"over the {label}'s lane", rot=PLACE_ROT)
        left.go(1.5, place, label=f"lower the {label} onto the field")
        left.go(0.4, label="steady")
        left.go(0.5, finger=_finger(w) - LET_GO, label=f"let go of the {label}")
        left.go(1.0, place + (0, 0, 0.06), label="lift away")
    if home:
        left.go(3.0, _START_TCP["left"], FINGER_OPEN, "home", _START_ROT["left"])
    return {"left": left.wps, "right": [Waypoint(3.0, tuple(_START_TCP["right"]), FINGER_OPEN, "idle",
                                                 rot=_START_ROT["right"])]}


def demo():
    """The left arm lays the four instruments out on the sterile field, one at a time, carrying each over the border."""
    return plan()
