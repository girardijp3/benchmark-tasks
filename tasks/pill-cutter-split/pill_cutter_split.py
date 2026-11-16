"""On the station table: a typical pill splitter (80 x 40 mm base with a 20 mm wide V cradle, hinged lid with a blade, 30 g in
all, fixed to the table by its rubber feet), one scored white caplet (20 x 9 x 7.5 mm, 1.8 g) lying beside it, and a dosing
tray with two 30 mL medicine cups (an orange and a blue one).

The task is to split the tablet and put one half in each cup: the tablet goes in the V cradle under the blade (its score line
at the blade), the lid is pulled shut by its fin, which cuts the tablet, the lid is opened again by the fin, and one of the
two halves goes into each cup.

The cut is declared task code: when the lid reaches shut (within 3 degrees) with the tablet in the cradle, within 2.5 mm of the
blade, the whole tablet is swapped for two half tablets lying in the cradle, 5 mm apart (the blade wedges them apart). The
cutter's lid hinge is a real articulated joint (0.6 N m s/rad of friction, stop at 95 degrees open, past upright, so the lid
stays open on its own and must be pulled shut); nothing else is scripted.

Run it:  inspect-robots run --embodiment yam_arena -E task=pill_cutter_split.py --scorer task_score ...

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
    finger's drive stiffness times how far something holds it back from its commanded position. ``closing`` is the
    unit vector of the finger-closing axis (the pads' inner faces are at ``tips +/- opening/2 * closing``)."""

    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float
    closing: np.ndarray | None = None


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
        poses = runtime.link_poses(robot)
        axis = poses["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)  # fingers open to -q each; a closing finger is held back
        out[f"{side}_gripper"] = GripperState(
            back=tcp - PAD_BACK * axis,
            tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
            closing=poses["gripper"][:3, 0],
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
TAG = "pill_cutter_split"

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
SP_XY = (0.36, 0.21)  # the splitter's base middle (its cradle runs along y; the hinge is on the robot's side, -x)
TB_XY = (0.28, 0.33)  # the tablet's middle at the start, lying with its length along y
TRAY_XY = (0.30, 0.44)  # the dosing tray's middle (cups at y -0.03 and +0.03 from it)
SP_POS = np.array([*SP_XY, TOP])
HINGE_REL = np.array([-0.040, 0.0, 0.011])  # the lid hinge axis, from the splitter's origin
OPEN_DEG = 95.0  # the lid's stop
TAB_W, TAB_L, TAB_T = 0.009, 0.020, 0.0075
V_DEPTH, V_SLOPE = 0.0055, 0.55  # the cradle: 5.5 mm deep, walls rising 0.55 mm per mm
SEAT_Z = V_DEPTH + V_SLOPE * (TAB_W / 2) + TAB_T / 2  # a tablet's centre above the table when it lies in the cradle
HALF_L = TAB_L / 2
HALF_GAP = 0.005  # the cut halves lie this far apart
CUP_R_IN, CUP_H, TRAY_T = 0.019, 0.030, 0.004
CUP_DY = 0.030
CUPS = {"orange": (TRAY_XY[0], TRAY_XY[1] - CUP_DY), "blue": (TRAY_XY[0], TRAY_XY[1] + CUP_DY)}
PARK = (0.30, 0.0, 0.06)  # the halves wait here, on the floor under the table, until the cut
POCKET_FLOOR = 0.001  # the dish pocket's floor above the table
TABLET_Z = TOP + POCKET_FLOOR + TAB_T / 2 + 0.0004

SPLITTER = UsdPart("cutter", asset="cutter", xy=SP_XY, extent=0.12, mass=None, usd_path=str(ASSETS / "pill_cutter.usda"),
                   pos=tuple(float(v) for v in SP_POS), articulation=True, fixed=True)
TRAY = UsdPart("tray", asset="tray", xy=TRAY_XY, extent=0.12, mass=None, usd_path=str(ASSETS / "cup_tray.usda"),
               pos=(*TRAY_XY, TOP), kinematic=True)
DISH = UsdPart("dish", asset="dish", xy=TB_XY, extent=0.04, mass=None, usd_path=str(ASSETS / "pill_dish.usda"),
               pos=(*TB_XY, TOP), kinematic=True)
TABLET = UsdPart("tablet", asset="tablet", xy=TB_XY, extent=0.03, mass=0.0018, usd_path=str(ASSETS / "tablet.usda"),
                 pos=(*TB_XY, TABLET_Z))
HALF_A = UsdPart("half_a", asset="half_a", xy=PARK[:2], extent=0.02, mass=0.0009, usd_path=str(ASSETS / "half_tablet.usda"),
                 pos=PARK)
HALF_B = UsdPart("half_b", asset="half_b", xy=PARK[:2], extent=0.02, mass=0.0009, usd_path=str(ASSETS / "half_tablet.usda"),
                 pos=(PARK[0] + 0.05, PARK[1], PARK[2]))
OBJECTS = [SPLITTER, TRAY, DISH, TABLET, HALF_A, HALF_B]
MOVERS = ("tablet", "half_a", "half_b")
LID_JOINT = "Hinge"
_SPLIT = {"done": False}
CUT_DEG = 3.0  # the lid counts as shut within this
CUT_CENTRE = 0.0025  # the tablet's middle within this of the blade (along y) and the cradle's line (along x)


def _quat_xyzw_id():
    return (0.0, 0.0, 0.0, 1.0)


def cradle_frame(pos) -> np.ndarray:
    """A world point in the splitter's frame (x away from the robot, y along the cradle, z up from the table)."""
    return np.asarray(pos, float) - SP_POS


def tablet_in_cradle(pos, rot) -> bool:
    """The whole tablet lies in the V cradle with its middle at the blade: within CUT_CENTRE of the blade and the cradle's
    line, at the height a tablet lies there (2 mm), its length along the cradle within 8 degrees, level within 8 degrees."""
    q = cradle_frame(pos)
    axis = np.asarray(rot, float)[:, 1]  # the tablet's length axis
    tilt = math.acos(min(1.0, max(-1.0, float(np.asarray(rot, float)[2, 2]))))
    return bool(abs(q[1]) <= CUT_CENTRE and abs(q[0]) <= CUT_CENTRE and abs(q[2] - SEAT_Z) <= 0.002
                and abs(axis[1]) >= math.cos(math.radians(8.0)) and tilt <= math.radians(8.0))


def _lid_state(env):
    art = env.scene["cutter"]
    ids = art.find_joints([LID_JOINT], preserve_order=True)[0]
    return art, ids


def extra_states(base, out, origin) -> None:
    """The lid's opening (degrees) and whether the tablet has been cut."""
    art, ids = _lid_state(base)
    out["lid_deg"] = float(-math.degrees(float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0])))
    out["split"] = bool(_SPLIT["done"])


def _write_pose(env, env_ids, name, pos, quat=None):
    """Writes a part's pose (table frame, m) and stops it."""
    import torch

    quat = _quat_xyzw_id() if quat is None else quat
    origin = env.scene.env_origins[0].detach().cpu().numpy()
    part = env.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *quat]], device=part.device, dtype=torch.float32)
    part.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    part.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=part.device), env_ids=env_ids)


def _reset_cutter(env, env_ids, asset_cfg=None) -> None:
    """Reset event: the lid open on its stop, the cut undone."""
    import torch

    if env_ids is None:
        return
    _SPLIT["done"] = False
    art, ids = _lid_state(env)
    q = torch.tensor([[-math.radians(OPEN_DEG)]], device=art.device, dtype=torch.float32)
    art.write_joint_position_to_sim_index(position=q, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids, env_ids=env_ids)


def _cut_step(env, env_ids, asset_cfg=None) -> None:
    """Every control step: when the lid is shut with the whole tablet in the cradle at the blade, the tablet is swapped for
    its two halves (declared: the blade is not simulated)."""
    from yam_arena.urdf import quat_xyzw_to_matrix

    if _SPLIT["done"]:
        return
    art, ids = _lid_state(env)
    lid = -math.degrees(float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy()[0]))
    if lid > CUT_DEG:
        return
    origin = env.scene.env_origins[0].detach().cpu().numpy()
    pose = env.scene["tablet"].data.root_link_pose_w.torch[0].detach().cpu().numpy().astype(np.float64)
    pos, rot = pose[:3] - origin, quat_xyzw_to_matrix(pose[3:])
    if not tablet_in_cradle(pos, rot):
        return
    _SPLIT["done"] = True
    z = SEAT_Z + 0.0004
    c = SP_POS
    off = HALF_L / 2 + HALF_GAP / 2
    _write_pose(env, env_ids, "half_a", (c[0], c[1] - off, c[2] + z))
    _write_pose(env, env_ids, "half_b", (c[0], c[1] + off, c[2] + z))
    _write_pose(env, env_ids, "tablet", (PARK[0] - 0.05, PARK[1], PARK[2]))
    t = float(env.episode_length_buf[0]) * float(env.step_dt)
    print(f"[{TAG}] t={t:.2f}s  the lid shut on the tablet in the cradle: cut into two halves", flush=True)


def reset_events() -> list:
    return [("cutter_reset", _reset_cutter, {})]


def step_events() -> list:
    return [("cutter_step", _cut_step, {})]


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
LID_OPEN_DEG = 80.0  # the lid counts as opened again past this
IN_CUP_MARGIN = 0.004  # a half is in a cup: its middle within the cup's inside radius less this
CLEAR = 0.03  # m: let go = no pad within this of a half or of the lid's fin
FIN_REL = np.array([0.068, 0.0, 0.027])  # the fin's middle, from the hinge axis, with the lid shut
TAB_LO, TAB_HI = np.array([-TAB_W / 2, -TAB_L / 2, -TAB_T / 2]), np.array([TAB_W / 2, TAB_L / 2, TAB_T / 2])
HALF_LO, HALF_HI = np.array([-TAB_W / 2, -HALF_L / 2, -TAB_T / 2]), np.array([TAB_W / 2, HALF_L / 2, TAB_T / 2])
STEPS = ("tablet lying in the cradle at the blade", "lid shut on it: tablet cut in two", "lid opened again",
         "one half in each cup, grippers clear")


def fin_point(lid_deg: float) -> np.ndarray:
    """The lid fin's middle in the table frame for a lid opened ``lid_deg`` degrees (a turn about the hinge axis, along y)."""
    a = math.radians(lid_deg)
    x, z = FIN_REL[0], FIN_REL[2]
    return SP_POS + HINGE_REL + np.array([x * math.cos(a) - z * math.sin(a), 0.0, x * math.sin(a) + z * math.cos(a)])


def _near_point(objects, point, margin: float) -> bool:
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is not None and np.min(np.linalg.norm(_pads(g) - np.asarray(point, float), axis=1)) <= margin:
            return True
    return False


def half_cup(objects, name: str):
    """Which cup (``orange``, ``blue``) a half lies in, or None."""
    pos = np.asarray(objects[name].pos, float)
    for cup, (cx, cy) in CUPS.items():
        if (math.hypot(pos[0] - cx, pos[1] - cy) <= CUP_R_IN - IN_CUP_MARGIN
                and TOP + TRAY_T - 0.002 <= pos[2] <= TOP + TRAY_T + CUP_H):
            return cup
    return None


def one_each(objects) -> bool:
    a, b = half_cup(objects, "half_a"), half_cup(objects, "half_b")
    return bool(a is not None and b is not None and a != b)


def halves_clear(objects) -> bool:
    return not any(pads_near(objects, n, HALF_LO, HALF_HI, CLEAR) for n in ("half_a", "half_b"))


def _seated_now(objects) -> bool:
    s = objects["tablet"]
    return bool(not objects.get("split") and tablet_in_cradle(np.array(s.pos, float), _rot(s)))


def _steps(objects) -> list[bool]:
    p = _PROGRESS
    split = bool(objects.get("split"))
    seated = bool(p.mem.get("seated") or _seated_now(objects))
    opened = bool(split and objects.get("lid_deg", 0.0) >= LID_OPEN_DEG)
    cups = bool(split and one_each(objects) and halves_clear(objects)
                and not _near_point(objects, fin_point(objects.get("lid_deg", 0.0)), CLEAR))
    return [seated, split, opened, cups]


def _pad_inside(objects, name: str, lo, hi, margin: float = 0.0005) -> bool:
    """Pass-through latch: a pad's inner-face centre at the tips is inside the part's solid by more than ``margin``."""
    s = objects.get(name)
    if s is None:
        return False
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None or getattr(g, "closing", None) is None:
            continue
        pts = np.array([g.tips - 0.5 * g.opening * g.closing, g.tips + 0.5 * g.opening * g.closing])
        q = (pts - np.asarray(s.pos, float)) @ _rot(s)
        if np.any(np.all((q > np.asarray(lo) + margin) & (q < np.asarray(hi) - margin), axis=1)):
            return True
    return False


def events_now(objects, p) -> list[str]:
    """Rule events as things stand: the tablet or a half off the table or a pad inside the held part. Also remembers
    (``p.mem``) that the tablet has lain in the cradle at the blade."""
    out = []
    if _seated_now(objects):
        p.mem["seated"] = True
    active = ("half_a", "half_b") if objects.get("split") else ("tablet",)
    for n in active:
        if objects[n].pos[2] < TOP - 0.03:
            out.append(f"the {n.replace('_', ' ')} left the table")
    for n, lo, hi in (("tablet", TAB_LO, TAB_HI), ("half_a", HALF_LO, HALF_HI), ("half_b", HALF_LO, HALF_HI)):
        if n == "tablet" and objects.get("split"):
            continue
        if n != "tablet" and not objects.get("split"):
            continue
        if _pad_inside(objects, n, lo, hi):
            out.append(f"a gripper pad is inside the {n.replace('_', ' ')}")
    return out


def trace_line(objects) -> str:
    cups = {n: half_cup(objects, n) for n in ("half_a", "half_b")}
    t = objects["tablet"]
    return (f"lid {objects.get('lid_deg', float('nan')):.1f} deg, cut {bool(objects.get('split'))}, tablet at "
            f"{(np.array(t.pos) - SP_POS)[0] * 1000:.1f},{(np.array(t.pos) - SP_POS)[1] * 1000:.1f},"
            f"{(np.array(t.pos) - SP_POS)[2] * 1000:.1f} mm from the cradle, halves in {cups['half_a']}/{cups['half_b']}")


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
    values = [v for n in MOVERS for v in objects[n].pos]
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
# The left arm does everything: it pinches the tablet by its sides and sets it in the cradle, pinches the lid's fin and
# pulls the lid shut (the cut), pinches the fin again and swings the lid open, then moves one half to each cup.
_X, _Y, _Z = np.eye(3)
_grip = yam_demo.grip_rotation
SQUEEZE = 0.008  # the fin: close the fingers this far inside its 6 mm width (about 15 N a finger; the lid needs it)
TAB_SQUEEZE = 0.001  # the caplet and halves: about 2.5 N a finger; 20 N ejected the 1.8 g caplet on PhysX


def _hold(width: float, squeeze: float = SQUEEZE) -> float:
    return -(width - squeeze) / 2


def _open(width: float, extra: float = 0.014) -> float:
    return -(width + extra) / 2


PAD_ABOVE = 0.0041  # at a pinch the pad tips are this far above the part's underside: 0.55 mm of finger-carriage
# clearance over the caplet in the dish (at 3.6 mm it was only 0.05 mm and the 20 N pinch rode the caplet into it) and
# 1.1 mm over the 11 mm cradle rim when the caplet is set down. The caplet is still pinched over the top 3.5 mm of its
# 7.5 mm height.
HOVER = 0.05
TAB_YAW = math.pi / 2  # pointing down with the fingers closing along x (across the tablet's width)
MID_DEG = 50.0  # the lid is let go here and taken again with a different wrist angle
FIN_W = 0.006


def _tcp_down(x, y, z_under):
    """TCP pinching a part whose underside is at ``z_under``, pointing down."""
    return np.array([x, y, z_under + PAD_ABOVE + PAD_TIP_PAST_TCP])


def _fin_rot0(tilt_deg: float) -> np.ndarray:
    """The gripper that holds the fin of a shut lid, its pointing axis ``tilt_deg`` below +x (over 90: down and back), the
    fingers closing along y."""
    return axis_angle_matrix(_Y, -math.pi / 2 + math.radians(tilt_deg)) @ yam_demo.grip_rotation(0.0)


def _lid_pose(deg: float, tilt_deg: float, rel=np.zeros(3)):
    """The TCP and rotation that hold the fin of a lid opened ``deg`` degrees (a turn about the hinge axis, along y), with the
    grip that points ``tilt_deg`` below +x on a shut lid."""
    a = math.radians(deg)
    h = SP_POS + HINGE_REL
    r = axis_angle_matrix(_Y, -a)
    p0 = SP_POS + HINGE_REL + FIN_REL + rel
    return h + r @ (p0 - h), r @ _fin_rot0(tilt_deg)


def _to_cup(cup: str):
    return np.array([*CUPS[cup], 0.0])


def _fin_swing(a0: float, a1: float, tilt: float, step: float, secs: float, label: str):
    """Waypoints that carry the held fin from lid angle ``a0`` to ``a1`` degrees (a waypoint every ``step`` degrees)."""
    n = max(1, int(round(abs(a1 - a0) / step)))
    out = []
    for i in range(1, n + 1):
        deg = a0 + (a1 - a0) * i / n
        p, r = _lid_pose(deg, tilt)
        out.append(Waypoint(secs, tuple(p), _hold(FIN_W), f"{label} ({deg:.0f} deg)", rot=r))
    return out


HIGH = 0.17  # m: the hand travels over the lid at this height above the table
APPROACH = 0.035  # m: the gripper comes to the fin from this far back along its pointing axis


def _fin_move(a0: float, a1: float, tilt: float, label: str, secs_per_8deg: float = 0.7):
    """Pinch the fin of a lid at ``a0`` with the grip ``tilt``, carry it to ``a1`` and let go."""
    p, r = _lid_pose(a0, tilt)
    back = r[:, 2]
    app = p - APPROACH * back
    wps = [
        Waypoint(2.0, (app[0], app[1], max(app[2], TOP + HIGH) if a0 > 60.0 else app[2] + 0.05), FINGER_OPEN, "clear of the lid", rot=r),
        Waypoint(2.0, tuple(app), FINGER_OPEN, f"in front of the fin ({label})", rot=r),
        Waypoint(1.5, tuple(p), _open(FIN_W, 0.03), "pads astride the fin", rot=r),
        Waypoint(0.8, tuple(p), _hold(FIN_W), "grip the fin", rot=r),
        Waypoint(0.4, tuple(p), _hold(FIN_W), "hold", rot=r),
    ]
    wps += _fin_swing(a0, a1, tilt, 8.0, secs_per_8deg, label)
    p2, r2 = _lid_pose(a1, tilt)
    wps += [
        Waypoint(1.5 if a1 > 90.0 else 0.6, tuple(p2), _hold(FIN_W), "hold", rot=r2),
        Waypoint(0.6, tuple(p2), _open(FIN_W, 0.03), "let go of the fin", rot=r2),
        Waypoint(1.5, tuple(p2 - APPROACH * r2[:, 2]), FINGER_OPEN, "back off", rot=r2),
    ]
    return wps


def left_plan() -> list[Waypoint]:
    rot = _grip(TAB_YAW)
    wps = []
    # 1. the tablet into the cradle
    tx, ty = TB_XY
    pick = _tcp_down(tx, ty, TOP + POCKET_FLOOR)
    seat_under = SP_POS[2] + V_DEPTH + V_SLOPE * (TAB_W / 2)  # the tablet's underside edges in the cradle
    place = _tcp_down(SP_XY[0], SP_XY[1], seat_under)
    wps += [
        Waypoint(2.5, pick + (0, 0, HOVER), _open(TAB_W, 0.020), "over the tablet", rot=rot),
        Waypoint(1.5, pick, _open(TAB_W, 0.020), "pads beside the tablet", rot=rot),
        Waypoint(0.8, pick, _hold(TAB_W, TAB_SQUEEZE), "grip the tablet", rot=rot),
        Waypoint(0.4, pick, _hold(TAB_W, TAB_SQUEEZE), "hold", rot=rot),
        Waypoint(1.2, pick + (0, 0, 0.04), _hold(TAB_W, TAB_SQUEEZE), "lift", rot=rot),
        Waypoint(2.5, place + (0, 0, 0.04), _hold(TAB_W, TAB_SQUEEZE), "over the cradle", rot=rot),
        Waypoint(1.5, place + (0, 0, 0.0008), _hold(TAB_W, TAB_SQUEEZE), "lower into the cradle", rot=rot),
        Waypoint(0.6, place + (0, 0, 0.0008), _open(TAB_W, 0.006), "let go", rot=rot),
        Waypoint(1.0, place + (0, 0, 0.05), _open(TAB_W, 0.006), "lift away", rot=rot),
    ]
    wps.append(Waypoint(1.2, tuple(place + (0, 0, HIGH - 0.02)), _open(TAB_W, 0.006), "up over the splitter", rot=rot))
    # 2. the lid shut by its fin (the cut): carried to MID_DEG with one wrist angle, then taken again for the rest
    wps += _fin_move(OPEN_DEG, MID_DEG, 115.0, "pull the lid down")
    wps += _fin_move(MID_DEG, 0.0, 95.0, "pull the lid shut")
    # 3. the lid opened again by its fin
    wps += _fin_move(0.0, MID_DEG, 90.0, "swing the lid up")
    wps += _fin_move(MID_DEG, OPEN_DEG + 3.0, 115.0, "swing the lid open")
    # 4. a half to each cup
    off = HALF_L / 2 + HALF_GAP / 2
    cup_floor = TOP + TRAY_T + 0.002
    hi = _lid_pose(OPEN_DEG, 115.0)[0]
    wps.append(Waypoint(1.5, (hi[0], hi[1], TOP + HIGH), FINGER_OPEN, "up and clear", rot=wps[-1].rot))
    for sy, cup in ((-off, "orange"), (off, "blue")):
        pick = _tcp_down(SP_XY[0], SP_XY[1] + sy, seat_under)
        cx, cy = CUPS[cup]
        drop = _tcp_down(cx, cy, cup_floor + 0.010)
        wps += [
            Waypoint(2.5, pick + (0, 0, HOVER), _open(TAB_W, 0.020), f"over the half ({cup} cup)", rot=rot),
            Waypoint(1.5, pick, _open(TAB_W, 0.020), "pads beside the half", rot=rot),
            Waypoint(0.8, pick, _hold(TAB_W, TAB_SQUEEZE), "grip the half", rot=rot),
            Waypoint(0.4, pick, _hold(TAB_W, TAB_SQUEEZE), "hold", rot=rot),
            Waypoint(1.2, pick + (0, 0, 0.04), _hold(TAB_W, TAB_SQUEEZE), "lift", rot=rot),
            Waypoint(2.5, drop + (0, 0, 0.06), _hold(TAB_W, TAB_SQUEEZE), "over the cup", rot=rot),
            Waypoint(1.5, drop, _hold(TAB_W, TAB_SQUEEZE), "down into the cup", rot=rot),
            Waypoint(0.6, drop, _open(TAB_W, 0.010), "let go", rot=rot),
            Waypoint(1.0, drop + (0, 0, 0.07), _open(TAB_W, 0.010), "lift away", rot=rot),
        ]
    wps.append(Waypoint(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]))
    return wps


def right_plan() -> list[Waypoint]:
    return []


def demo():
    """The left arm seats the tablet, shuts and opens the lid by its fin, and puts a half in each cup."""
    return {"left": left_plan()}
