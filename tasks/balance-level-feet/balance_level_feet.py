"""On the station table: a Shimadzu AUX220 analytical balance (220 g x 0.1 mg, 217 x 356 x 338 mm, 7.0 kg), standing
on its two front level screws and its fixed centre-rear foot, its circular level at the front.  The balance is tilted
in two axes; the bubble is off centre.  The robot turns the two large knurled front level screws, one at a time, until
the body is level and the bubble is in the middle of its red circle, then lets go and leaves it still.

The body is free: a free articulation root on three contact points (the two front feet and the rear foot).  Each front
foot is a prismatic link; its knurled wheel is a revolute link.  A wheel turned by the gripper drives its foot's slide
by the real screw pitch (1.75 mm a revolution, EST); the body tilts in both axes as the three support heights change.
The bubble is a separate gravity-free visual body whose position is computed from the body's real tilt; the scored
state is the body tilt, not the bubble.

Run it:  inspect-robots run --embodiment yam_arena -E task=balance_level_feet.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down.  This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with the
physics authored there, ``Waypoint.rot`` gives the scripted demo full gripper rotations, and a step event runs the
declared screw (the wheel's turn -> its foot's height) and the bubble.
"""

TAG = "balance"
import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from types import SimpleNamespace
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
SUCCESS_HOLD_S = 0.0  # success() itself times the 1 s full-score hold
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001

# -- the balance (its own frame: origin at the middle of the base's underside; front toward the robot at -x) ----------
BODY_W, BODY_D = 0.217, 0.356  # published 217 x 356 mm
BODY_H = 0.298  # base 60 + draft shield 238 (EST from the published 338 overall height and 40 mm feet)
FOOT_NOM = 0.040  # nominal foot extension; the three support heights equal = level (EST 40 mm)
FRONT_X, FRONT_Y = -0.168, 0.085
REAR_X = 0.150
WHEEL_R, WHEEL_Z = 0.017, 0.024
PITCH = 0.00175  # m per wheel revolution (EST: a large M12 x 1.75-class level screw; Shimadzu does not publish it)
E_MIN, E_MAX = 0.030, 0.052  # thread travel: 10 mm down, 12 mm up from nominal (EST)
BAL_XY = (0.528, 0.0)  # the base's centre on the table (the wheels are at x 0.360)
BAL_POS = (*BAL_XY, TOP + 0.044)  # spawned 4 mm high; it settles onto its three feet
E_L0, E_R0 = 0.04390, 0.04220  # initial front-foot extensions: pitch 0.55 deg, roll 0.55 deg
# The body's level is both front extensions = FOOT_NOM (the rear foot is fixed at FOOT_NOM).  The demo's turns:
TURN_L = (E_L0 - FOOT_NOM) / PITCH * 360.0  # 802.3 deg
TURN_R = (E_R0 - FOOT_NOM) / PITCH * 360.0  # 452.6 deg
BODY_MASS = 7.0  # published net weight, kg

# -- layout of the two wheels in world space (x forward, y to the robot's left) ------------------------------------
WHEEL_X = BAL_XY[0] + FRONT_X  # 0.490
WHEEL_TOP = TOP + 0.040 - FOOT_NOM + WHEEL_Z  # wheel centre when level: 0.7715
WHEEL_XY = {1: (WHEEL_X, FRONT_Y), -1: (WHEEL_X, -FRONT_Y)}
try:  # aim only: the Isaac arm's TCP tracking is corrected here, never the scoring
    import isaaclab
    _ON_ISAAC = getattr(isaaclab, "__file__", None) is not None
except ImportError:
    _ON_ISAAC = False
ISAAC_AIM = (0.0, 0.0, 0.0)  # world x, y, z added to every wheel grasp (tuned on Isaac if the grip misses)
print(f"[{TAG}] on Isaac: {_ON_ISAAC}", flush=True)

# -- gripper / hand ------------------------------------------------------------------------------------------------
PAD_BACK = 0.025  # m: the pads reach this far behind the TCP
GRIP_FORCE = 3.0  # N: a wheel grip needs at least this much finger force
GRIP_OPEN = (0.028, 0.040)  # the 34 mm wheel between the pads
GRIP_REACH = 0.025  # the TCP must be this close to the wheel's axis
OPEN_WHEEL = -(0.034 + 0.020) / 2  # pads 54 mm apart: 10 mm of clearance each side
HOLD_WHEEL = -(0.034 - 0.012) / 2  # 6 mm of squeeze a finger on the 34 mm wheel (about 20 N each)
RESET_WHEEL = -(0.034 + 0.012) / 2  # pads 46 mm apart: 6 mm each side, enough to break the grip and reset in place

# -- scoring -------------------------------------------------------------------------------------------------------
WINDOW_PART, WINDOW_FULL = 0.5, 0.1  # deg, per axis
BUBBLE_FULL = 0.0005  # m: the bubble within 0.5 mm of the vial's centre (0.125 deg at 4 mm/deg)
BUBBLE_K = 0.2292  # m of bubble travel per unit of in-plane gravity (= 4 mm per degree, EST for a 30 mm vial)
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(0.5)
CLEAR_DIST = 0.10  # m: both grippers at least this far from the body and the wheels for full score
POINTS = 0.25


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False
    no_gravity: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


BALANCE = UsdPart("balance", asset="balance", xy=BAL_XY, extent=0.36, mass=BODY_MASS,
                  usd_path=str(ASSETS / "balance_aux220.usda"), pos=BAL_POS, articulation=True)
BUBBLE = UsdPart("bubble", asset="bubble", xy=(BAL_XY[0] - 0.160, BAL_XY[1] - 0.073), extent=0.01, mass=0.0005,
                 usd_path=str(ASSETS / "level_bubble.usda"), pos=(BAL_XY[0] - 0.160, BAL_XY[1] - 0.073, TOP + 0.1224),
                 no_gravity=True)
OBJECTS = [BALANCE, BUBBLE]
MOVERS = ("balance",)
SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2


def _spawn(scene, spec):
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False)
    return scene.Object(
        name=spec.name, usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw), spawn_cfg_addon=addon)


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): the TCP, the pad span (``back`` to ``tips``), how far apart
    the pads are, the smaller finger's grip force (N) and the gripper's rotation (3x3)."""

    tcp: np.ndarray
    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float
    rot: np.ndarray

    @property
    def yaw(self) -> float:
        return math.atan2(float(self.rot[1, 0]), float(self.rot[0, 0]))


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        rot = runtime.link_poses(robot)["gripper"][:3, :3]
        axis = rot[:, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)
        out[f"{side}_gripper"] = GripperState(
            tcp=tcp, back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
            rot=np.asarray(rot, float))
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
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.balance_reset = EventTermCfg(func=_reset_balance, mode="reset",
                                                        params={"asset_cfg": SceneEntityCfg("balance")})
            env_cfg.events.balance_step = EventTermCfg(func=_balance_step, mode="interval",
                                                       interval_range_s=(0.0, 0.0), is_global_time=False,
                                                       params={"asset_cfg": SceneEntityCfg("balance")})
            return env_cfg

        return with_events

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


class _AfterSceneImport(importlib.abc.MetaPathFinder):
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

# -- joints, declared screw and bubble ------------------------------------------------------------------------------
FOOT_JOINTS = ("FootL_Slide", "FootR_Slide")
WHEEL_JOINTS = ("WheelL_Spin", "WheelR_Spin")
JOINTS = FOOT_JOINTS + WHEEL_JOINTS
WHEEL_SIDE = {"WheelL_Spin": 1, "WheelR_Spin": -1}
_FOOT_Y = {"FootL_Slide": FRONT_Y, "FootR_Slide": -FRONT_Y}


def _joint_ids(bal):
    return bal.find_joints(list(JOINTS), preserve_order=True)[0]


def _dof_names(bal):
    return list(bal.root_view.shared_metatype.dof_names)


def _set_targets(bal, values: dict) -> None:
    """Set the articulation's drive targets (m for the slides, rad for the wheels) in PhysX directly."""
    import torch

    view = bal.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    names = _dof_names(bal)
    for name, value in values.items():
        t[:, names.index(name)] = float(value)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, bal._ALL_INDICES)


class _Mechanism:
    """The declared screw and the episode's latches.  The wheel is a real revolute link; while a gripper holds it its
    angle follows the hand's yaw (a knurled wheel does not slip under a 20 N pinch), and its foot's slide follows the
    wheel at PITCH a revolution.  The body itself is free on the three feet."""

    def __init__(self):
        self.wheel_target = {1: 0.0, -1: 0.0}
        self.grab = {1: None, -1: None}  # (wheel angle at the grab, hand yaw at the grab)
        self.damaged = False
        self.tipped = False
        self.off_table = False
        self.last = None
        self.held_s = 0.0
        self.t = 0.0

    def reset(self):
        self.__init__()


_MECH = _Mechanism()


def _wrap(d: float) -> float:
    return math.atan2(math.sin(d), math.cos(d))


def _tilt(quat) -> tuple[float, float]:
    """Body pitch and roll in degrees from the body's z axis (the same two axes the vial reads)."""
    R = quat_xyzw_to_matrix(np.asarray(quat, float))
    p = math.degrees(math.atan2(float(R[0, 2]), float(R[2, 2])))
    r = math.degrees(math.atan2(float(-R[1, 2]), float(R[2, 2])))
    return p, r


def _bubble_xy(quat) -> tuple[float, float]:
    """The bubble's offset from the vial centre in the body's x-y plane (m).  The bubble runs to the high side."""
    R = quat_xyzw_to_matrix(np.asarray(quat, float))
    g = -R[2, :]  # world down in the body frame
    return -BUBBLE_K * float(g[0]), -BUBBLE_K * float(g[1])


def _wheel_center(pos, quat, side: int, ext: float):
    R = quat_xyzw_to_matrix(np.asarray(quat, float))
    local = np.array([FRONT_X, FRONT_Y * side, -ext + WHEEL_Z])
    return np.asarray(pos, float) + R @ local


def _feet_down(objects) -> bool:
    """All three support points on the bench: the two front pads within 2 mm of it and the rear pad too."""
    pos = np.asarray(objects["balance"].pos, float)
    R = quat_xyzw_to_matrix(np.asarray(objects["balance"].quat_xyzw, float))
    e = objects["foot_ext"]
    for side, yy, ext in ((1, FRONT_Y, e[0]), (-1, -FRONT_Y, e[1])):
        if (pos + R @ np.array([FRONT_X, yy, -ext]))[2] > TOP + 0.002:
            return False
    if (pos + R @ np.array([REAR_X, 0.0, -FOOT_NOM]))[2] > TOP + 0.002:
        return False
    return True


def _grip_wheel(g, pos, quat, side: int, ext: float) -> bool:
    if g.contact < GRIP_FORCE or not (GRIP_OPEN[0] <= g.opening <= GRIP_OPEN[1]):
        return False
    return float(np.linalg.norm(g.tcp - _wheel_center(pos, quat, side, ext))) <= GRIP_REACH


def _reset_balance(env, env_ids, asset_cfg=None):
    import torch

    if env_ids is None:
        return
    bal = env.scene["balance"]
    ids = _joint_ids(bal)
    q = torch.tensor([[-E_L0, -E_R0, 0.0, 0.0]], device=bal.device, dtype=torch.float32)
    bal.write_joint_position_to_sim_index(position=q, joint_ids=ids, env_ids=env_ids)
    bal.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids, env_ids=env_ids)
    pose = torch.tensor([[BAL_POS[0], BAL_POS[1], BAL_POS[2], 0.0, 0.0, 0.0, 1.0]],
                        device=bal.device, dtype=torch.float32)
    bal.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    bal.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=bal.device), env_ids=env_ids)
    _set_targets(bal, {"FootL_Slide": -E_L0, "FootR_Slide": -E_R0,
                       "WheelL_Spin": 0.0, "WheelR_Spin": 0.0})
    _MECH.reset()
    bubble = env.scene["bubble"]
    q0 = np.array([0, 0, 0, 1.0])
    bx, by = _bubble_xy(q0)
    p = np.array([BAL_POS[0] - 0.160 + bx, BAL_POS[1] - 0.073 + by, BAL_POS[2] + 0.0784])
    bubble.write_root_pose_to_sim_index(
        root_pose=torch.tensor([[p[0], p[1], p[2], 0.0, 0.0, 0.0, 1.0]], device=bubble.device, dtype=torch.float32),
        env_ids=env_ids)


def _balance_step(env, env_ids, asset_cfg=None):
    """Every control step: the two knurled wheels drive their feet at PITCH a revolution, the bubble follows the body,
    and the damage latches watch the feet's thread travel and the body's pose."""
    import torch

    bal = env.scene["balance"]
    base = env.unwrapped
    ids = _joint_ids(bal)
    q = bal.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
    e = {1: -q[0], -1: -q[1]}
    th = {1: q[2], -1: q[3]}
    pose = _arr(bal.data.root_link_pose_w)
    pos = pose[:3] - base.scene.env_origins[0].detach().cpu().numpy()
    quat = pose[3:]
    grips = _gripper_states(env)
    targets = {}
    for side, joint in ((1, "FootL_Slide"), (-1, "FootR_Slide")):
        g = grips["left_gripper" if side == 1 else "right_gripper"]
        wheel_joint = "WheelL_Spin" if side == 1 else "WheelR_Spin"
        if _grip_wheel(g, pos, quat, side, e[side]):
            if _MECH.grab[side] is None:
                _MECH.grab[side] = (th[side], g.yaw)
                print(f"[{TAG}] t={_MECH.t:.2f}s  {'left' if side == 1 else 'right'} grab at hand yaw "
                      f"{math.degrees(g.yaw):+.1f} deg, wheel {math.degrees(th[side]):+.1f} deg", flush=True)
            th0, yaw0 = _MECH.grab[side]
            _MECH.wheel_target[side] = th0 + _wrap(g.yaw - yaw0)
        else:
            _MECH.grab[side] = None
        target_e = E_L0 if side == 1 else E_R0
        target_e -= PITCH * _MECH.wheel_target[side] / (2 * math.pi)
        if target_e < E_MIN or target_e > E_MAX:
            if not _MECH.damaged:
                _MECH.damaged = True
                print(f"[{TAG}] t={_MECH.t:.2f}s  {'left' if side == 1 else 'right'} foot at "
                      f"{target_e * 1000:.1f} mm: {'off its thread' if target_e < E_MIN else 'over-tightened'} "
                      f"(score capped at 0.5)", flush=True)
        targets[joint] = -target_e
        targets[wheel_joint] = _MECH.wheel_target[side]
    _set_targets(bal, targets)

    p_deg, r_deg = _tilt(quat)
    if abs(p_deg) > 5.0 or abs(r_deg) > 5.0:
        _MECH.tipped = True
    if pos[2] < TOP - 0.01 or pos[2] > TOP + 0.10:
        _MECH.off_table = True
    # A foot that is off the bench at the scored state is handled by _feet_down(), not latched here: the body
    # can rock for a moment while a screw is turned, and the operator keeps it on all three feet at the end.
    _MECH.t += float(base.step_dt)

    # the bubble sits on the vial's glass, at the tilt the body really has
    bx, by = _bubble_xy(quat)
    R = quat_xyzw_to_matrix(np.asarray(quat, float))
    vial = np.array([-0.160, -0.073, 0.0784])
    world = np.asarray(pos, float) + R @ (vial + np.array([bx, by, 0.0]))
    bubble = env.scene["bubble"]
    bubble.write_root_pose_to_sim_index(
        root_pose=torch.tensor([[world[0], world[1], world[2], *quat]], device=bubble.device, dtype=torch.float32),
        env_ids=env_ids)
    bubble.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=bubble.device), env_ids=env_ids)

    # what the film's scoring panel shows (same four conditions as steps_done)
    _PROGRESS.time = _MECH.t
    objs = {"balance": SimpleNamespace(pos=pos, quat_xyzw=quat),
            "foot_ext": (e[1], e[-1]), "bubble_mm": (bx * 1000.0, by * 1000.0),
            "left_gripper": grips["left_gripper"], "right_gripper": grips["right_gripper"]}
    done = list(steps_done(objs))
    _PROGRESS.reported = done
    if all(done):
        if _PROGRESS.met_since is None:
            _PROGRESS.met_since = _MECH.t
    else:
        _PROGRESS.met_since = None

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={_MECH.t:.2f}s  pitch {p_deg:+.2f} deg roll {r_deg:+.2f} deg, bubble "
              f"{math.hypot(bx, by) * 1000:.2f} mm, feet {e[1] * 1000:.2f}/{e[-1] * 1000:.2f} mm, "
              f"wheels {math.degrees(th[1]):+.1f}/{math.degrees(th[-1]):+.1f} deg"
              f"{'  DAMAGED' if _MECH.damaged else ''}", flush=True)
        gt = []
        for ss in (1, -1):
            gg = grips["left_gripper" if ss == 1 else "right_gripper"]
            cc = _wheel_center(pos, quat, ss, e[ss])
            gt.append(f"{'L' if ss == 1 else 'R'} open {gg.opening * 1000:.1f}mm force {gg.contact:.1f}N "
                      f"TCP-gap {np.linalg.norm(gg.tcp - cc) * 1000:.1f}mm target {math.degrees(_MECH.wheel_target[ss]):+.0f}deg")
        print(f"[{TAG}] t={_MECH.t:.2f}s  grip: " + " | ".join(gt), flush=True)


def extra_states(env, out, origin):
    bal = env.scene["balance"]
    ids = _joint_ids(bal)
    q = bal.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
    out["foot_ext"] = (-float(q[0]), -float(q[1]))
    out["wheel_angle"] = (float(q[2]), float(q[3]))
    p, r = _tilt(out["balance"].quat_xyzw)
    out["tilt"] = (p, r)
    bx, by = _bubble_xy(out["balance"].quat_xyzw)
    out["bubble_mm"] = (bx * 1000.0, by * 1000.0)
    out["damaged"] = _MECH.damaged
    out["tipped"] = _MECH.tipped
    out["off_table"] = _MECH.off_table
    out["wheel_target"] = (_MECH.wheel_target[1], _MECH.wheel_target[-1])


# -- scoring -------------------------------------------------------------------------------------------------------
STEPS = ("both within 0.5 deg", "both within 0.1 deg", "bubble centred", "grippers clear")


def _clear(objects) -> bool:
    body = objects["balance"].pos
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.contact >= 0.5:
            return False
        for s in (1, -1):
            c = _wheel_center(body, objects["balance"].quat_xyzw, s, objects["foot_ext"][0 if s == 1 else 1])
            if float(np.linalg.norm(g.back - c)) < CLEAR_DIST or float(np.linalg.norm(g.tips - c)) < CLEAR_DIST:
                return False
        # the body's front face and the base's sides
        rel = g.back - body
        if abs(rel[0] + 0.10) < 0.20 and abs(rel[1]) < 0.16 and -0.02 < rel[2] < 0.30:
            return False
    return True


def _level(objects):
    p, r = _tilt(objects["balance"].quat_xyzw)
    m = math.hypot(*objects["bubble_mm"]) / 1000.0
    return p, r, m


def steps_done(objects) -> tuple[bool, ...]:
    p, r, m = _level(objects)
    down = _feet_down(objects)
    return (down and abs(p) <= WINDOW_PART and abs(r) <= WINDOW_PART,
            down and abs(p) <= WINDOW_FULL and abs(r) <= WINDOW_FULL,
            down and m <= BUBBLE_FULL,
            _clear(objects))


def score(objects) -> float:
    if objects.get("off_table") or objects.get("tipped"):
        _reset_hold()
        return 0.0
    p, r, m = _level(objects)
    if not _feet_down(objects):
        return 0.0
    part = abs(p) <= WINDOW_PART and abs(r) <= WINDOW_PART
    full = abs(p) <= WINDOW_FULL and abs(r) <= WINDOW_FULL
    pts = POINTS if (abs(p) <= WINDOW_PART or abs(r) <= WINDOW_PART) else 0.0
    if part:
        pts = 2 * POINTS
    if full and m <= BUBBLE_FULL:
        pts = 3 * POINTS
    if objects.get("damaged"):
        return min(pts, 2 * POINTS)
    return pts


class _Progress:
    """What the film addon writes into steps.jsonl: the time, the four step flags and the timing latch."""

    def __init__(self):
        self.time = 0.0
        self.reported = []
        self.met_since = None


_PROGRESS = _Progress()


_HOLD = {"t": None, "pos": None, "quat": None, "e": None}


def _reset_hold():
    _HOLD.update(t=None, pos=None, quat=None, e=None)


def success(objects) -> bool:
    p, r, m = _level(objects)
    if objects.get("damaged") or objects.get("off_table") or objects.get("tipped"):
        _reset_hold()
        return False
    if not (abs(p) <= WINDOW_FULL and abs(r) <= WINDOW_FULL and m <= BUBBLE_FULL and _feet_down(objects)):
        _reset_hold()
        return False
    if not _clear(objects):
        _reset_hold()
        return False
    pos = np.asarray(objects["balance"].pos, float)
    quat = np.asarray(objects["balance"].quat_xyzw, float)
    e = np.asarray(objects["foot_ext"], float)
    if _HOLD["pos"] is None or _HOLD["quat"] is None:
        _HOLD.update(t=0.0, pos=pos.copy(), quat=quat.copy(), e=e.copy())
        return False
    dot = min(1.0, abs(float(np.dot(quat, _HOLD["quat"]))))
    angle = 2.0 * math.acos(dot)
    if (float(np.linalg.norm(pos - _HOLD["pos"])) > STILL_MOVE or angle > STILL_TURN
            or float(np.linalg.norm(e - _HOLD["e"])) > 0.001):
        _HOLD.update(t=0.0, pos=pos.copy(), quat=quat.copy(), e=e.copy())
        return False
    _HOLD["t"] += 0.05
    return _HOLD["t"] >= STILL_S - 1e-9


# -- demo -----------------------------------------------------------------------------------------------------------
@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


def _grip_frame(yaw_deg: float) -> np.ndarray:
    """A 15-degree-down forward grip, yawed about vertical.  The TCP's z axis points +x and 15 deg down; its x axis
    (the finger-closing direction) is horizontal -y at yaw 0."""
    a = math.radians(15.0)
    z = np.array([math.cos(a), 0.0, -math.sin(a)])
    x = np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    R0 = np.column_stack([x, y, z])
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(yaw_deg)) @ R0


# The wheels sit low at the balance's front edge; at that reach the wrist's reliable yaw range about the vertical
# is about -100..+40 deg on the left and -40..+100 deg on the right (outside it q6 saturates at +-120 deg and the
# IK loses the wheel).  Each plan turns its wheel in 120 deg arcs from the side with the most margin.  Between arcs
# the pads stay at the wheel: they ease off to 46 mm (6 mm clear each side), the wrist resets in place and the pads
# pinch the knurled rim again.  No retreat to the pre-grasp, so a few mm of tracking lag still regrips.
GRIP_YAW = {1: (-80.0, 40.0), -1: (-40.0, 50.0)}


def _wheel_plan(side: int, turn_deg: float) -> list[Waypoint]:
    y = FRONT_Y * side + ISAAC_AIM[1]
    x, z = WHEEL_X + ISAAC_AIM[0], WHEEL_TOP - 0.001 + ISAAC_AIM[2]
    pre = (x - 0.045, y, z + 0.012)
    back = (x - 0.030, y, z + 0.008)
    y0, y1 = GRIP_YAW[side]
    arc = y1 - y0
    full = int(turn_deg // arc)
    rem = turn_deg - arc * full
    strokes = [(y0, y1)] * full
    if rem > 5.0:
        strokes.append((y0, y0 + rem))
    wps = [Waypoint(2.0, pre, OPEN_WHEEL, "to the wheel", rot=_grip_frame(0.0)),
           Waypoint(0.8, (x, y, z), OPEN_WHEEL, "pads astride the wheel", rot=_grip_frame(0.0)),
           Waypoint(0.8, (x, y, z), OPEN_WHEEL, "turn the open pads to the start", rot=_grip_frame(y0)),
           Waypoint(1.0, (x, y, z), OPEN_WHEEL, "settle at the start", rot=_grip_frame(y0)),
           Waypoint(0.6, (x, y, z), HOLD_WHEEL, "pinch the knurled wheel", rot=_grip_frame(y0)),
           Waypoint(0.4, (x, y, z), HOLD_WHEEL, "settle the pinch", rot=_grip_frame(y0))]
    for k, (a0, a1) in enumerate(strokes):
        if k > 0:
            wps.append(Waypoint(0.4, (x - 0.012, y, z + 0.004), RESET_WHEEL, "draw back a step", rot=_grip_frame(a0)))
            wps.append(Waypoint(0.5, (x, y, z), RESET_WHEEL, "re-approach the wheel", rot=_grip_frame(a0)))
            wps.append(Waypoint(0.3, (x, y, z), RESET_WHEEL, "settle at the wheel", rot=_grip_frame(a0)))
            wps.append(Waypoint(0.6, (x, y, z), HOLD_WHEEL, "pinch again", rot=_grip_frame(a0)))
            wps.append(Waypoint(0.4, (x, y, z), HOLD_WHEEL, "settle the pinch", rot=_grip_frame(a0)))
        wps.append(Waypoint(1.8, (x, y, z), HOLD_WHEEL, f"turn the wheel (+{a1 - a0:.0f} deg)", rot=_grip_frame(a1)))
        wps.append(Waypoint(0.3, (x, y, z), HOLD_WHEEL, "settle at the end", rot=_grip_frame(a1)))
        if k < len(strokes) - 1:
            wps.append(Waypoint(0.4, (x, y, z), RESET_WHEEL, "ease the pads off", rot=_grip_frame(a1)))
            wps.append(Waypoint(0.2, (x, y, z), RESET_WHEEL, "hold the pads off", rot=_grip_frame(a1)))
            wps.append(Waypoint(1.8, (x, y, z), RESET_WHEEL, "reset the wrist at the wheel", rot=_grip_frame(a0)))
            wps.append(Waypoint(0.8, (x, y, z), RESET_WHEEL, "settle at the start", rot=_grip_frame(a0)))
    last = strokes[-1][1]
    wps.append(Waypoint(0.5, (x, y, z), OPEN_WHEEL, "open at the end", rot=_grip_frame(last)))
    wps.append(Waypoint(0.6, back, OPEN_WHEEL, "back off", rot=_grip_frame(last)))
    wps.append(Waypoint(0.4, back, OPEN_WHEEL, "turn the open pads to 0", rot=_grip_frame(0.0)))
    wps.append(Waypoint(2.0, tuple(_START_TCP["left" if side == 1 else "right"]), FINGER_OPEN, "home",
                        rot=_START_ROT["left" if side == 1 else "right"]))
    return wps

def demo():
    """Open loop, known geometry: the left arm turns the left wheel 802.3 deg (six 120 deg arcs and an 82.3 deg trim),
    then the right arm turns the right wheel 452.6 deg (three 120 deg arcs and a 92.6 deg trim); both go home.  The
    wheels' angles follow the hands; the feet lower to 40.0 mm and the body comes level.  Between arcs the pads ease
    off to 46 mm and the wrist resets at the wheel, so the grip is never carried away from the knurled rim."""
    left = _wheel_plan(1, TURN_L)
    wait = Waypoint(sum(w.duration for w in left), tuple(_START_TCP["right"]), FINGER_OPEN,
                    "wait for the left arm", rot=_START_ROT["right"])
    right = [wait] + _wheel_plan(-1, TURN_R)
    return {"left": left, "right": right}


# -- optional closed-loop trace for tuning --------------------------------------------------------------------------
def trace(objects) -> str:
    p, r, m = _level(objects)
    return (f"pitch {p:+.2f} roll {r:+.2f} deg, bubble {m * 1000:.2f} mm, feet "
            f"{objects['foot_ext'][0] * 1000:.2f}/{objects['foot_ext'][1] * 1000:.2f} mm, "
            f"damaged {objects.get('damaged')}")
