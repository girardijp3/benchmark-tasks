"""On the station table: a fuse block lying flat (three MAXI blade fuses 28 mm apart standing upright in it, the middle one the one to remove,
the outer two fixed), a small tray beside it, and a tweezer-type fuse puller lying on the table in front, its jaws pointing away from the robot.

The task is to take the middle fuse out of the block with the puller and put it in the tray. The fuse stands upright (its axis vertical), so the puller
has to be carried horizontally, perpendicular to the fuse's axis, its jaws slid over the fuse's two faces between the neighbouring fuses, squeezed shut and
lifted straight up against the grip of the block's two spring clips (about 4 N). Taking the fuse out without the puller holding it limits the score.

Run it:  inspect-robots run --embodiment yam_arena -E task=fuse_puller_crosswise.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "fusepull"
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


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: a free rigid body, an articulation
    (``articulation``: its root link is free) or a fixture (``kinematic``). ``Asset``'s fields only satisfy the
    task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False  # an articulation whose root link is fixed to the world
    no_gravity: bool = False  # a free part with gravity off (parked out of the way)

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=True if spec.fixed else None)
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


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each part's root pose and velocity, the grippers and the episode time, plus what ``extra_states`` adds."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            data = base.scene[name].data
            pose = _arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=_arr(data.root_lin_vel_w),
                                    ang_vel=_arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        extra_states(base, out, origin)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus the task's reset events (``reset_events``)."""
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            env_cfg = configure(env_cfg)
            for name, func, params in reset_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="reset", params=params))
            for name, func, params in step_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="interval", interval_range_s=(0.0, 0.0),
                                                           is_global_time=False, params=params))
            return env_cfg

        return with_events

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


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
# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
BLOCK_XY = (0.38, -0.12)  # the fuse block's middle (the middle fuse stands here)
POCKET_FLOOR, BASE_Z = 0.008, 0.020
FUSE_W, FUSE_T, FUSE_H = 0.0292, 0.0085, 0.034
FUSE_MASS = 0.008
FUSE_START = (BLOCK_XY[0], BLOCK_XY[1], TOP + POCKET_FLOOR + 0.0001)  # the fuse's underside middle, on the pocket floor
TRAY_XY = (BLOCK_XY[0] - 0.050, BLOCK_XY[1] + 0.110)
TRAY_IN, TRAY_T = 0.054, 0.003
PULLER_START = (0.16, BLOCK_XY[1], TOP + 0.0003)  # the puller's hinge line, middle, on the table
ARM_L = 0.100
JAW_X0, JAW_X1 = 0.074, 0.102  # the jaw plates along the puller's x
ART_NAME = "puller"
ARM_JOINTS = ("ArmP", "ArmM")

BLOCK = UsdPart("block", asset="block", xy=BLOCK_XY, extent=0.20, mass=None, usd_path=str(ASSETS / "fuse_block.usda"),
                pos=(BLOCK_XY[0], BLOCK_XY[1], TOP), articulation=True, fixed=True)
FUSE = UsdPart("fuse", asset="fuse", xy=BLOCK_XY, extent=0.04, mass=FUSE_MASS, usd_path=str(ASSETS / "fuse.usda"), pos=FUSE_START)
PULLER = UsdPart("puller", asset="puller", xy=PULLER_START[:2], extent=0.12, mass=0.03, usd_path=str(ASSETS / "puller.usda"), pos=PULLER_START,
                 articulation=True)
PITCH_Y = 0.028  # the fuses stand 28 mm apart
NEIGHBOUR_START = {"fuse_a": (BLOCK_XY[0], BLOCK_XY[1] - PITCH_Y, TOP + POCKET_FLOOR + 0.0001),  # the 50 A fuse (red), -y side
                   "fuse_b": (BLOCK_XY[0], BLOCK_XY[1] + PITCH_Y, TOP + POCKET_FLOOR + 0.0001)}  # the 20 A fuse (yellow), +y side
NEIGHBOUR_MOVED = 0.003  # a neighbouring fuse moved this far from its pocket (or lifted) scores 0
FUSE_A = UsdPart("fuse_a", asset="fuse", xy=NEIGHBOUR_START["fuse_a"][:2], extent=0.04, mass=FUSE_MASS, usd_path=str(ASSETS / "fuse_50.usda"), pos=NEIGHBOUR_START["fuse_a"])
FUSE_B = UsdPart("fuse_b", asset="fuse", xy=NEIGHBOUR_START["fuse_b"][:2], extent=0.04, mass=FUSE_MASS, usd_path=str(ASSETS / "fuse_20.usda"), pos=NEIGHBOUR_START["fuse_b"])
OBJECTS = [BLOCK, FUSE, FUSE_A, FUSE_B, PULLER]
MOVERS = ("fuse", "puller")


def extra_states(base, out, origin) -> None:
    """The two puller arms' angles (rad; 0 open, ArmP negative and ArmM positive when closing)."""
    art = base.scene[ART_NAME]
    for key, joint in (("arm_p", "ArmP"), ("arm_m", "ArmM")):
        ids = art.find_joints([joint], preserve_order=True)[0]
        out[key] = float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])


def reset_events() -> list:
    return []


def step_events() -> list:
    return []


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

# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
# -- the rule and what is scored ----------------------------------------------------------------------------------------
CLOSED_MIN, CLOSED_MAX = math.radians(1.2), math.radians(3.2)  # the arms' angle with the jaws closed on the fuse (about 1.8 degrees; empty they close to 4.5)
OPEN_MAX = math.radians(0.8)  # the jaws count as open (released) below this
JAW_OVERLAP_X, JAW_OVERLAP_Z = 0.020, 0.008  # the fuse's body overlaps the jaw plates at least this much along the puller's x and z
JAW_Y = 0.0035  # the fuse's middle within this of the jaws' midline
POCKET_TOP = TOP + BASE_Z
OUT_Z = POCKET_TOP + FUSE_H / 2 + 0.0005  # the fuse's middle at least this high: its underside is above the pocket's rim
OUT_CLAMP_WINDOW = 0.5  # s: the jaws must have been closed on the fuse this recently when it first leaves the pocket
RISE = 0.003  # the fuse lifted this much (or slid sideways) without the puller clamped on it = taken out bare
TRAY_BOUND = TRAY_IN / 2 - 0.002
TRAY_Z = TOP + TRAY_T + 0.024  # the fuse's middle within this height: it rests in the tray
CLEAR = 0.03  # no gripper pad and no part of the puller within this of the fuse (step 4)
FUSE_LO = np.array([-FUSE_W / 2, -FUSE_T / 2, 0.0])
FUSE_HI = np.array([FUSE_W / 2, FUSE_T / 2, FUSE_H])
STEPS = ("the puller's jaws closed on the fuse", "the fuse out of its pocket, taken out by the puller",
         "the fuse resting in the tray", "the jaws open, the puller set down and let go, the grippers and the puller clear of the fuse")


def _centre(objects):
    s = objects["fuse"]
    return np.asarray(s.pos, float) + _rot(s)[:, 2] * (FUSE_H / 2)


def clamped(objects) -> bool:
    """The puller's jaws are closed on the fuse: both arms closed about 1.8 degrees, the fuse's body between the jaw plates."""
    ap, am = float(objects.get("arm_p", 0.0)), float(objects.get("arm_m", 0.0))
    if not (-CLOSED_MAX <= ap <= -CLOSED_MIN and CLOSED_MIN <= am <= CLOSED_MAX):
        return False
    pu = objects["puller"]
    r = _rot(pu)
    q = (_centre(objects) - np.asarray(pu.pos, float)) @ r
    if abs(q[1]) > JAW_Y or float(r[2, 2]) < math.cos(math.radians(20.0)):
        return False
    ox = min(q[0] + FUSE_W / 2, JAW_X1) - max(q[0] - FUSE_W / 2, JAW_X0)
    oz = min(q[2] + FUSE_H / 2, 0.012) - max(q[2] - FUSE_H / 2, 0.0)
    return bool(ox >= JAW_OVERLAP_X and oz >= JAW_OVERLAP_Z)


def in_pocket(objects) -> bool:
    c = _centre(objects)
    return bool(abs(c[0] - BLOCK_XY[0]) <= FUSE_W / 2 + 0.002 and abs(c[1] - BLOCK_XY[1]) <= 0.0065 and c[2] < OUT_Z)


def in_tray(objects) -> bool:
    c = _centre(objects)
    return bool(abs(c[0] - TRAY_XY[0]) <= TRAY_BOUND and abs(c[1] - TRAY_XY[1]) <= TRAY_BOUND and c[2] <= TRAY_Z)


def jaws_open(objects) -> bool:
    return bool(abs(float(objects.get("arm_p", 0.0))) <= OPEN_MAX and abs(float(objects.get("arm_m", 0.0))) <= OPEN_MAX)


def held(objects) -> bool:
    return any(holds(objects, s, "fuse", FUSE_LO, FUSE_HI) for s in ("left", "right"))


HEAD_BOX = ((-0.006, 0.0, 0.006), (0.016, 0.034, 0.012))  # the puller's head (centre, size) in its own frame
ARM_BOX = ((0.050, 0.0, 0.006), (0.100, 0.003, 0.012))  # an arm, in its own frame (origin on the hinge)
JAW_BOX = ((0.086, 0.0, 0.006), (0.028, 0.0025, 0.012))  # a jaw plate, in the arm's frame; its y is set below (it stands just inside the arm)
ARM_HINGES = ((0.002, 0.0105, 1), (0.002, -0.0105, -1))  # x, y of each arm's hinge (``ArmP``, ``ArmM``) and its side
SAMPLE = 0.002  # m: the puller's boxes are sampled this finely


def _box_points(centre, size, step=SAMPLE) -> np.ndarray:
    axes = [np.linspace(-s / 2, s / 2, max(2, int(math.ceil(s / step)) + 1)) for s in size]
    grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    return grid + np.asarray(centre)


def puller_points(objects) -> np.ndarray:
    """Points over every part of the puller (head, both arms and their jaw plates, the arms at their current angles), in the world frame."""
    pu = objects["puller"]
    pts = [_box_points(*HEAD_BOX)]
    for (hx, hy, side), key in zip(ARM_HINGES, ("arm_p", "arm_m"), strict=True):
        q = float(objects.get(key, 0.0))
        local = np.vstack([_box_points(*ARM_BOX), _box_points((JAW_BOX[0][0], -side * 0.0025, JAW_BOX[0][2]), JAW_BOX[1])])
        c, sn = math.cos(q), math.sin(q)
        turned = np.stack([c * local[:, 0] - sn * local[:, 1], sn * local[:, 0] + c * local[:, 1], local[:, 2]], axis=1)
        pts.append(turned + np.array([hx, hy, 0.0]))
    return np.vstack(pts) @ _rot(pu).T + np.asarray(pu.pos, float)


def puller_near_fuse(objects, margin: float = CLEAR) -> bool:
    """Any part of the puller (head, arms, jaw plates) is within ``margin`` of the fuse's body."""
    fu = objects["fuse"]
    q = (puller_points(objects) - np.asarray(fu.pos, float)) @ _rot(fu)
    d = np.linalg.norm(q - np.clip(q, FUSE_LO, FUSE_HI), axis=1)
    return bool(np.any(d <= margin))


def clear_of_fuse(objects) -> bool:
    """Truly clear: no gripper pad and no part of the puller within CLEAR of the fuse."""
    return not pads_near(objects, "fuse", FUSE_LO, FUSE_HI, CLEAR) and not puller_near_fuse(objects)


PULLER_LO = np.array([-0.014, -0.017, 0.0])
PULLER_HI = np.array([0.102, 0.017, 0.012])
DOWN_TILT = math.radians(20.0)  # a puller set down lies within this of flat
ON_TABLE = 0.02  # and its hinge line is this close to the table


def puller_set_down(objects) -> bool:
    """The puller lies on the table (within DOWN_TILT of flat, its hinge line within ON_TABLE of the table) and no gripper holds it."""
    pu = objects["puller"]
    if float(pu.pos[2]) > TOP + ON_TABLE or float(_rot(pu)[2, 2]) < math.cos(DOWN_TILT):
        return False
    return not any(holds(objects, side, "puller", PULLER_LO, PULLER_HI) for side in ("left", "right"))


def _steps(objects) -> list[bool]:
    p = _PROGRESS
    pulled = bool(p.mem.get("clamped")) and not p.mem.get("bare") and not in_pocket(objects)
    tray = bool(in_tray(objects))
    return [bool(p.mem.get("clamped")), bool(pulled), bool(tray and pulled),
            bool(tray and pulled and jaws_open(objects) and puller_set_down(objects) and clear_of_fuse(objects) and not held(objects))]


def events_now(objects, p) -> list[str]:
    """Rule events as things stand: the fuse or puller off the table. Also remembers (``p.mem``) that the jaws were closed on the fuse,
    and whether the fuse was taken out bare (lifted or slid by something else than the closed jaws)."""
    out = []
    cl = clamped(objects)
    if cl:
        p.mem["clamped"] = True
    c = _centre(objects)
    start_c = np.array([FUSE_START[0], FUSE_START[1], FUSE_START[2] + FUSE_H / 2])
    if not cl and not p.mem.get("clamped") and (np.linalg.norm(c - start_c) > RISE):
        if not p.mem.get("bare"):
            print(f"[{TAG}] the fuse was moved {np.linalg.norm(c - start_c) * 1000:.1f} mm without the puller's jaws clamped on it: score limited", flush=True)
        p.mem["bare"] = True
    if cl:
        p.mem["clamped_t"] = p.time
    if not p.mem.get("out_seen") and not in_pocket(objects):  # the moment the fuse first leaves the pocket, the jaws must have been closed on it (within 0.5 s)
        p.mem["out_seen"] = True
        if p.time - p.mem.get("clamped_t", -99.0) > OUT_CLAMP_WINDOW:
            p.mem["bare"] = True
            print(f"[{TAG}] the fuse left its pocket without the puller's jaws clamped on it: score limited", flush=True)
    for name, start in NEIGHBOUR_START.items():
        n = objects.get(name)
        if n is not None and np.linalg.norm(np.asarray(n.pos, float) - np.asarray(start, float)) > NEIGHBOUR_MOVED:
            out.append(f"a neighbouring fuse ({name[-1]}) was moved out of its place")
    if objects["fuse"].pos[2] < TOP - 0.03:
        out.append("the fuse fell off the table")
    if objects["puller"].pos[2] < TOP - 0.03:
        out.append("the puller fell off the table")
    return out


def trace_line(objects) -> str:
    c = _centre(objects)
    return (f"fuse middle {(c[0] - BLOCK_XY[0]) * 1000:.1f},{(c[1] - BLOCK_XY[1]) * 1000:.1f} mm from the pocket, {(c[2] - TOP) * 1000:.1f} mm up, "
            f"arms {math.degrees(float(objects.get('arm_p', float('nan')))):.1f}/{math.degrees(float(objects.get('arm_m', float('nan')))):.1f} deg, "
            f"clamped {clamped(objects)}, bare {bool(_PROGRESS.mem.get('bare'))}{', held' if held(objects) else ''}")


# -- scoring (shared machinery) ---------------------------------------------------------------------------------------
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
POINTS = 0.2  # per step; only success scores 1.0
GRIP_SLACK = 0.004  # held: a pad's point within this of the part's bounds ...
GRIP_FORCE = 0.5  # ... both fingers pressing with at least this (N) ...
GRIP_OPENING = 0.003  # ... and the pads apart by at least this (not closed on nothing)


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def holds(objects, side: str, name: str, lo, hi) -> bool:
    """The ``side`` gripper holds the part: a point along its pads is within GRIP_SLACK of the part's bounds (``lo``,
    ``hi``, in the part's frame), both fingers press with GRIP_FORCE and the pads are apart by GRIP_OPENING."""
    g = objects.get(f"{side}_gripper")
    if g is None or g.contact < GRIP_FORCE or g.opening < GRIP_OPENING:
        return False
    s = objects[name]
    q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
    return bool(np.any(np.all((q >= np.asarray(lo) - GRIP_SLACK) & (q <= np.asarray(hi) + GRIP_SLACK), axis=1)))


def pads_near(objects, name: str, lo, hi, margin: float) -> bool:
    """Either gripper's pads are within ``margin`` of the part's bounds."""
    s = objects[name]
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
        if np.any(np.all((q >= np.asarray(lo) - margin) & (q <= np.asarray(hi) + margin), axis=1)):
            return True
    return False


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in MOVERS)


def still(objects, anchor) -> bool:
    """Every part within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses, not from the
    simulator's instantaneous speeds."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: the task's own memory (``mem``), every
    rule violation (``events``, they last for the whole episode), and since when every full-success condition has held
    without a break (``met_since``, ``None`` while one fails) with the poses when they began to hold (``anchor``).
    Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.mem: dict = {}
        self.events: list[str] = []
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def flag(self, t, what: str) -> None:
        if what not in self.events:
            self.events.append(what)
            print(f"[{TAG}] t={t:.2f}s  RULE BROKEN: {what} (scores 0 from here on)", flush=True)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        stamp = -1.0 if t is None else t
        for what in events_now(objects, self):
            self.flag(stamp, what)
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not self.events)
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand (see ``_steps``)."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = f"({trace_line(objects)}; rule events {len(p.events)})"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: any rule event this episode (kept for the whole run), or a state that is not finite. Reads the
    episode's progress without updating it."""
    values = [v for n in (*MOVERS, "fuse_a", "fuse_b") for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.events)


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all steps done, no rule event so far, and the parts within STILL_MOVE and STILL_TURN of where they were when
    that second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise POINTS per step done (``STEPS``, at
    most 0.8, also while the second is being timed); 0 after any rule event."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)
# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right arm, pointing down, takes the puller by its two arms (pads closing on their outer faces beside the jaw plates) with a
# light grip (squeezing is what shuts the jaws), lifts it, tips the jaws down toward the fuse (PITCH), carries it forward and lowers the jaws either side
# of the fuse's two faces, squeezes the arms so the jaws clamp it, draws it straight up out of the clips and clear of its neighbours, carries it over the
# tray, lowers it and lets the arms open so the fuse drops in. Then it lifts the open puller straight up off the fuse, carries it back beside the block, lays
# it flat on the table where it started, lets go and goes home.
_grip = yam_demo.grip_rotation
ROT = _grip(0.0)
PAD_TIP_Z = 0.003  # the pad tips above the puller's underside
HANDLE_X = 0.079  # the pads' lowest point on the arms this far from the hinge line (the arms' front part, beside the jaw plates, which start at 0.074)
LEAN = 0.0  # the gripper holds the puller square to its arms, straight down when the puller lies flat (a lean along it made the pads pitch the puller)
PITCH = math.radians(5.0)  # the jaws tipped down this far (5 degrees) while the puller is carried; the scorer allows 20; 10 made the arms close unevenly on the station
PAD_WIDE = -0.030  # fingers wide open (per finger)
PAD_LIGHT = -0.0110  # pads 1 mm inside the arms' outer faces (24 mm apart): about 5 N a finger, less than the arms' spring so they stay open
PAD_SQUEEZE = -0.0055  # squeezing the arms shut on the fuse (the arms' outer faces 18.9 mm apart, about 4 mm further per finger; a softer squeeze let the puller pitch in the pads under the pull)
CARRY = 0.034  # the jaw plates' middle this high above the table when the jaws are level with the fuse's exposed body
LIFT = 0.040  # then the fuse is lifted this far (its underside clears the neighbours' tops)
APPROACH_UP = 0.014  # the jaws come in this much higher, to clear the block's edge, then drop either side of the fuse
CLEAR_UP = 0.0  # the open puller is lifted this far above the carry height off the fuse in the tray (the jaw tips are then 23 mm over the fuse's top)
AIM_Y = -0.0020  # the jaws aimed 2 mm to -y of the fuse: on the station the arms then close about evenly (aimed at 0 or +2 mm the -y-side arm closed under 1.2 degrees)
SIDE_REST = (PULLER_START[0], PULLER_START[1])  # the puller is laid back on the table here, beside the block


def _ry(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), a)


def _rot_for(p: float) -> np.ndarray:
    """The gripper's rotation when the puller is pitched ``p`` (jaws down)."""
    return _ry(p - LEAN) @ ROT


def _tcp(o, p: float = 0.0):
    """The TCP that puts the pads at the handle of a puller whose hinge-line origin is ``o`` (x, y, height of its underside over the table when flat)
    and that is pitched ``p`` (jaws down), when the gripper holds it as it does at the pick."""
    tip = np.array([o[0], o[1], TOP + o[2]]) + _ry(p) @ np.array([HANDLE_X, 0.0, PAD_TIP_Z])
    axis = _rot_for(p)[:, 2]
    return tuple(float(v) for v in tip - PAD_TIP_PAST_TCP * axis)


def left_plan() -> list[Waypoint]:
    return []


def right_plan() -> list[Waypoint]:
    px, py = PULLER_START[0], PULLER_START[1]
    jaw_mid = 0.5 * (JAW_X0 + JAW_X1)
    sp, cp = math.sin(PITCH), math.cos(PITCH)

    def origin(fuse_xy, z_mid):
        """The puller origin that puts the jaw plates' middle over ``fuse_xy`` at height ``z_mid`` over the table, pitched PITCH."""
        return (fuse_xy[0] - (jaw_mid * cp + 0.006 * sp), fuse_xy[1] + AIM_Y, z_mid + jaw_mid * sp - 0.006 * cp)

    high_fuse = origin(BLOCK_XY, CARRY + APPROACH_UP)
    over_fuse = origin(BLOCK_XY, CARRY)
    up_fuse = origin(BLOCK_XY, CARRY + LIFT)
    over_tray = origin(TRAY_XY, CARRY + LIFT)
    in_tray = origin(TRAY_XY, TRAY_T + 0.012 + 0.020 + 0.006)  # the fuse's underside 12 mm over the tray's floor
    off_tray = origin(TRAY_XY, CARRY + LIFT + CLEAR_UP)
    rest_up = (SIDE_REST[0], SIDE_REST[1], 0.060)
    rest_dn = (SIDE_REST[0], SIDE_REST[1], 0.0)
    w = lambda d, o, p, f, label: Waypoint(d, _tcp(o, p), f, label, rot=_rot_for(p))  # noqa: E731
    return [
        w(2.5, (px, py, 0.06), 0.0, PAD_WIDE, "over the puller"),
        w(1.5, (px, py, 0.0), 0.0, PAD_WIDE, "pads astride the arms"),
        w(0.8, (px, py, 0.0), 0.0, PAD_LIGHT, "take the puller lightly"),
        w(0.4, (px, py, 0.0), 0.0, PAD_LIGHT, "hold"),
        w(1.5, (px, py, 0.05), 0.0, PAD_LIGHT, "lift the puller"),
        w(1.5, (px + 0.03, py, 0.05), PITCH, PAD_LIGHT, "tip the jaws down toward the fuse"),
        w(3.0, high_fuse, PITCH, PAD_LIGHT, "carry it forward, jaws over the fuse"),
        w(1.0, over_fuse, PITCH, PAD_LIGHT, "lower the jaws either side of the fuse"),
        w(0.6, over_fuse, PITCH, PAD_LIGHT, "hold"),
        w(1.2, over_fuse, PITCH, PAD_SQUEEZE, "squeeze the jaws shut on the fuse"),
        w(0.4, over_fuse, PITCH, PAD_SQUEEZE, "hold"),
        w(2.0, up_fuse, PITCH, PAD_SQUEEZE, "draw the fuse straight up out of the clips"),
        w(3.5, over_tray, PITCH, PAD_SQUEEZE, "carry it over the tray"),
        w(1.5, in_tray, PITCH, PAD_SQUEEZE, "lower it"),
        w(0.8, in_tray, PITCH, PAD_LIGHT, "let the jaws open"),
        w(1.0, in_tray, PITCH, PAD_LIGHT, "hold"),
        w(1.8, off_tray, PITCH, PAD_LIGHT, "lift the open puller clear of the fuse"),
        w(3.0, (rest_up[0], rest_up[1], rest_up[2] + 0.02), PITCH * 0.5, PAD_LIGHT, "carry it back beside the block"),
        w(1.5, rest_up, 0.0, PAD_LIGHT, "level it"),
        w(1.5, rest_dn, 0.0, PAD_LIGHT, "lay it on the table"),
        w(0.6, rest_dn, 0.0, PAD_WIDE, "let go of the puller"),
        w(1.2, rest_up, 0.0, PAD_WIDE, "lift away"),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The right arm takes the puller by the arms, clamps the fuse with the jaws tipped down, draws it out and puts it in the tray, then lays the puller back
    on the table."""
    return {"right": right_plan()}
