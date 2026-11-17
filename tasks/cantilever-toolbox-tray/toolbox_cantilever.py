"""On the station table: a STANLEY 1-94-738 18 in. metal cantilever tool box (460 x 208 x 208 mm, 4.8 kg, five
trays, two metal latches), closed, with a claw hammer in its upper fixed tray, a 150 mm adjustable wrench in the left
moving tray and a 120 mm screwdriver in the right moving tray.  The robot pulls both front draw latches open, then
lifts the lid's wire tray handle; the parallelogram arms swing the two moving trays out and up until they are level and
spread, and the box stays on its four rubber feet with the tools still in their trays.

The box is one articulation from ``assets/toolbox.usda``: Base (free on four feet), the two arms (ArmL/ArmR), their
level trays (WingL/WingR), the lid (Top, a vertical prismatic joint) and the two latches (LatchL/LatchR).  The tools
are separate rigid bodies.

Run it:  inspect-robots run --embodiment yam_arena -E task=toolbox_cantilever.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down.  This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with the
physics authored there, ``Waypoint.rot`` gives the scripted demo full gripper rotations, and an interval event runs
the declared latch detents and the lid-to-arm coupling.
"""

TAG = "toolbox"
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

# -- the box (published 460 x 208 x 208 mm, 4.8 kg; local frame: middle of the base's underside, +x away from the robot,
#    +y to the robot's left) ------------------------------------------------------------------------------------------
Y_LEN, X_DEP = 0.460, 0.208
HALF_Y, HALF_X = Y_LEN / 2, X_DEP / 2
BASE_TOP, LID_BOT = 0.105, 0.195
BOX_XY = (0.43, 0.0)
BOX_POS = (*BOX_XY, TOP)
BOX_MASS = 4.74  # base 3.20 + arms 0.30 + wings 0.80 + lid 0.40 + latches 0.04 - the tools are separate
BOX_TILT_MAX = 5.0  # deg: over this the box has tipped
BOX_MOVE_MAX = 0.030  # m: the base's centre may settle this far and no more
STABLE_TILT, STABLE_MOVE = 2.0, 0.010  # deg, m, for the final state

# -- mechanism --------------------------------------------------------------------------------------------------------
ARM_CLOSED = 0.0
ARM_L_OPEN, ARM_R_OPEN = math.radians(-130.0), math.radians(135.0)
ARM_L_PART, ARM_R_PART = math.radians(-120.0), math.radians(130.0)
LIFT_START, LIFT_OPEN, LIFT_MAX = 0.030, 0.105, 0.115
LATCH_CLOSED = 0.0
LATCH_PART, LATCH_OPEN = math.radians(-10.0), math.radians(-18.0)  # -10: moved; -18: released
LATCH_FULL = math.radians(-25.0)
LATCH_PULL = 0.012  # m of TCP travel that takes the latch from closed to its open stop
JOINTS = ("ArmL_Swing", "WingL_Level", "ArmR_Swing", "WingR_Level", "Top_Lift", "LatchL_Swing", "LatchR_Swing")

# -- layout (world frame, m; x forward away from the arms, y to the robot's left, table top z 0) ----------------------
LATCH_KNOB = {1: np.array([BOX_XY[0] - 0.122, 0.160, TOP + 0.165]),
              -1: np.array([BOX_XY[0] - 0.122, -0.160, TOP + 0.165])}
LATCH_OPEN_KNOB = {1: np.array([BOX_XY[0] - 0.134, 0.160, TOP + 0.171]),
                   -1: np.array([BOX_XY[0] - 0.134, -0.160, TOP + 0.171])}
HANDLE_BAR = np.array([BOX_XY[0] - 0.060, 0.0, TOP + 0.216])  # the wire handle's grip bar
HANDLE_TOP = HANDLE_BAR + np.array([0.0, 0.0, LIFT_MAX])
WRENCH_POS = (BOX_XY[0], 0.001, TOP + 0.106)
SCREWDRIVER_POS = (BOX_XY[0], -0.001, TOP + 0.154)
HAMMER_POS = (BOX_XY[0], -0.090, TOP + 0.078)

# -- gripper ----------------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0  # N: the smaller finger's force for a real hold
GRIP_OPEN = (0.004, 0.024)  # the 10 mm latch knob / 8 mm handle bar between the pads
GRIP_REACH = 0.018  # m: the TCP within this of the knob or bar
OPEN_FINGER = -0.020
LATCH_HOLD = -0.003  # 3 mm squeeze a finger on the 10 mm knob (about 15 N)
HANDLE_HOLD = 0.0  # fingers command closed on the 8 mm bar (about 4 mm squeeze a finger)
CLEAR_DIST = 0.050  # m: both grippers at least this far from the box sides at the scored state

# -- tools (real sizes; masses in the files) ---------------------------------------------------------------------------
WRENCH_MASS, SCREWDRIVER_MASS, HAMMER_MASS = 0.35, 0.15, 0.50
TOOL_SLACK = 0.012
TRAY_L = {"WingL": (-0.160, 0.0), "WingR": (0.0, 0.160)}


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed: bool = False
    no_gravity: bool = False

    @property
    def free(self) -> bool:
        return not self.fixed


TOOLBOX = UsdPart("toolbox", asset="toolbox", xy=BOX_XY, extent=0.50, mass=BOX_MASS,
                  usd_path=str(ASSETS / "toolbox.usda"), pos=BOX_POS, articulation=True)
HAMMER = UsdPart("hammer", asset="hammer", xy=(HAMMER_POS[0], HAMMER_POS[1]), extent=0.30, mass=HAMMER_MASS,
                 usd_path=str(ASSETS / "hammer.usda"), pos=HAMMER_POS)
WRENCH = UsdPart("wrench", asset="wrench", xy=(WRENCH_POS[0], WRENCH_POS[1]), extent=0.16, mass=WRENCH_MASS,
                 usd_path=str(ASSETS / "wrench.usda"), pos=WRENCH_POS)
SCREWDRIVER = UsdPart("screwdriver", asset="screwdriver", xy=(SCREWDRIVER_POS[0], SCREWDRIVER_POS[1]), extent=0.13,
                      mass=SCREWDRIVER_MASS, usd_path=str(ASSETS / "screwdriver.usda"), pos=SCREWDRIVER_POS)
OBJECTS = [TOOLBOX, HAMMER, WRENCH, SCREWDRIVER]
MOVERS = ("toolbox",)
SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2


def _spawn(scene, spec):
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
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
    tcp: np.ndarray
    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float
    rot: np.ndarray


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
            tcp=tcp, back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, opening=-float(q.sum()),
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
            env_cfg.events.toolbox_reset = EventTermCfg(func=_reset_toolbox, mode="reset",
                                                        params={"asset_cfg": SceneEntityCfg("toolbox")})
            env_cfg.events.toolbox_step = EventTermCfg(func=_toolbox_step, mode="interval",
                                                       interval_range_s=(0.0, 0.0), is_global_time=False,
                                                       params={"asset_cfg": SceneEntityCfg("toolbox")})
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


# -- joint helpers ----------------------------------------------------------------------------------------------------
def _joint_ids(tb):
    return tb.find_joints(list(JOINTS), preserve_order=True)[0]


def _dof_names(tb):
    return list(tb.root_view.shared_metatype.dof_names)


def _set_targets(tb, values: dict) -> None:
    import torch

    view = tb.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp
        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    names = _dof_names(tb)
    for name, value in values.items():
        t[:, names.index(name)] = float(value)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, tb._ALL_INDICES)


def _pose(asset, body: str | None = None):
    """A body's world pose (pos, quat_xyzw); ``None`` is the root link."""
    if body is None or body == "Base":
        p = _arr(asset.data.root_link_pose_w)
    else:
        idx = asset.find_bodies(body)[0][0]
        p = _arr(asset.data.body_link_pose_w)[idx]
    return p[:3], p[3:]


def _rot(quat) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat, float))


class _Mechanism:
    """The declared latch detents and the lid-to-arm coupling; the setbacks and the tool checks."""

    def __init__(self):
        self.t = 0.0
        self.box0 = None
        self.forced = False
        self.dropped = False
        self.pinched = False
        self.tipped = False
        self.off_table = False
        self.opened_by_hand = [False, False]
        self.latch_open = [False, False]
        self.last_lift = 0.0

    def reset(self):
        self.__init__()


_MECH = _Mechanism()


def _grip_near(g, point, radius: float = GRIP_REACH) -> bool:
    return (g is not None and g.contact >= GRIP_FORCE and GRIP_OPEN[0] <= g.opening <= GRIP_OPEN[1]
            and float(np.linalg.norm(g.tcp - point)) <= radius)


def _knob_world(tb, side: int) -> np.ndarray:
    p, q = _pose(tb, "Latch" + ("L" if side == 1 else "R"))
    return p + _rot(q) @ np.array([-0.008, 0.0, -0.030])


def _bar_world(tb) -> np.ndarray:
    p, q = _pose(tb, "Top")
    return p + _rot(q) @ np.array([-0.060, 0.0, 0.021])


def _reset_toolbox(env, env_ids, asset_cfg=None):
    import torch

    base = env.unwrapped
    if env_ids is None:
        return
    tb = env.scene["toolbox"]
    ids = _joint_ids(tb)
    q = torch.zeros((1, len(ids)), device=tb.device, dtype=torch.float32)
    tb.write_joint_position_to_sim_index(position=q, joint_ids=ids, env_ids=env_ids)
    tb.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids, env_ids=env_ids)
    pose = torch.tensor([[BOX_POS[0], BOX_POS[1], BOX_POS[2] + 0.001, 0.0, 0.0, 0.0, 1.0]],
                        device=tb.device, dtype=torch.float32)
    tb.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    tb.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=tb.device), env_ids=env_ids)
    _set_targets(tb, {n: 0.0 for n in JOINTS})
    for name, p in (("hammer", HAMMER_POS), ("wrench", WRENCH_POS), ("screwdriver", SCREWDRIVER_POS)):
        tool = env.scene[name]
        tp = torch.tensor([[p[0], p[1], p[2] + 0.001, 0.0, 0.0, 0.0, 1.0]],
                          device=tool.device, dtype=torch.float32)
        tool.write_root_pose_to_sim_index(root_pose=tp, env_ids=env_ids)
        tool.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=tool.device), env_ids=env_ids)
    _MECH.reset()
    p, q0 = _pose(tb, "Base")
    _MECH.box0 = (p.copy(), q0.copy())


def _tool_local(env, tool: str, frame_body: str | None) -> np.ndarray:
    """The tool root in the frame of a box body (``None``: the box root)."""
    tb = env.scene["toolbox"]
    if frame_body is None:
        fp, fq = _pose(tb, "Base")
    else:
        fp, fq = _pose(tb, frame_body)
    tp, _ = _pose(env.scene[tool], None)
    return _rot(fq).T @ (tp - fp)


def _tool_ok(env, tool: str, frame_body: str | None, bounds) -> bool:
    q = _tool_local(env, tool, frame_body)
    if q[2] < bounds["z"][0] - 0.025:
        return False
    x_ok = bounds["x"][0] - TOOL_SLACK <= q[0] <= bounds["x"][1] + TOOL_SLACK
    y_ok = bounds["y"][0] - TOOL_SLACK <= q[1] <= bounds["y"][1] + TOOL_SLACK
    z_ok = bounds["z"][0] - 0.020 <= q[2] <= bounds["z"][1] + 0.030
    return bool(x_ok and y_ok and z_ok)


WRENCH_BOUNDS = {"x": (-0.073, 0.073), "y": (-0.158, -0.002), "z": (0.002, 0.030)}
SCREW_BOUNDS = {"x": (-0.083, 0.083), "y": (0.002, 0.158), "z": (0.042, 0.070)}
HAMMER_BOUNDS = {"x": (-0.088, 0.088), "y": (-0.208, 0.208), "z": (0.067, 0.090)}


def _tool_in_tray(env) -> bool:
    return (_tool_ok(env, "wrench", "WingL", WRENCH_BOUNDS)
            and _tool_ok(env, "screwdriver", "WingR", SCREW_BOUNDS)
            and _tool_ok(env, "hammer", None, HAMMER_BOUNDS))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _in_box_local(q, lo, hi) -> bool:
    return bool(np.all(q >= lo) and np.all(q <= hi))


def _toolbox_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    tb = env.scene["toolbox"]
    ids = _joint_ids(tb)
    q = tb.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
    j = dict(zip(JOINTS, q, strict=True))
    p, quat = _pose(tb, "Base")
    R = _rot(quat)
    grips = _gripper_states(env)
    lift = float(j["Top_Lift"])

    # 1. the lid: the hand holds it where it is; otherwise the over-centre detent holds it open or lets it fall shut
    bar = _bar_world(tb)
    handle_grip = any(_grip_near(grips[f"{s}_gripper"], bar) for s in ("left", "right"))
    if handle_grip:
        top_target = lift
    elif lift > 0.100:
        top_target = LIFT_MAX
    else:
        top_target = 0.0

    # 2. the two latches: while a hand holds the knob the latch follows the pull; otherwise its spring detent holds it
    latch_targets = {}
    for side, nm in ((1, "LatchL_Swing"), (-1, "LatchR_Swing")):
        g = grips["left_gripper" if side == 1 else "right_gripper"]
        knob = _knob_world(tb, side)
        gripped = _grip_near(g, knob)
        angle = float(j[nm])
        if gripped:
            pull = float(np.clip((LATCH_KNOB[side][0] - g.tcp[0]) / LATCH_PULL, 0.0, 1.0))
            latch_targets[nm] = LATCH_FULL * pull
            if angle < LATCH_PART:
                _MECH.opened_by_hand[0 if side == 1 else 1] = True
                _MECH.latch_open[0 if side == 1 else 1] = True
        else:
            latch_targets[nm] = (LATCH_FULL if (_MECH.latch_open[0 if side == 1 else 1] or angle < LATCH_OPEN)
                                 else LATCH_CLOSED)
            if angle < LATCH_PART and not _MECH.opened_by_hand[0 if side == 1 else 1]:
                _MECH.forced = True

    # 3. the declared parallelogram coupling: the lid's lift drives both arms; the level joints cancel the arm's roll
    f = float(np.clip(lift / 0.110, 0.0, 1.0))
    arm_l, arm_r = ARM_L_OPEN * f, ARM_R_OPEN * f
    targets = {**latch_targets, "Top_Lift": top_target, "ArmL_Swing": arm_l, "WingL_Level": -arm_l,
               "ArmR_Swing": arm_r, "WingR_Level": -arm_r}
    _set_targets(tb, targets)

    # 4. the damage / drop / pinch latches
    if _MECH.box0 is not None:
        tilt = _tilt(quat)
        move = float(np.linalg.norm(p - _MECH.box0[0]))
        if _MECH.box0[1] is not None:
            dot = min(1.0, abs(float(np.dot(quat, _MECH.box0[1]))))
            rot = 2.0 * math.acos(dot)
        else:
            rot = 0.0
        if tilt > BOX_TILT_MAX or move > BOX_MOVE_MAX or rot > math.radians(8.0):
            _MECH.tipped = True
        if p[2] < TOP - 0.02:
            _MECH.off_table = True
    if not _tool_in_tray(env):
        _MECH.dropped = True
    for side in ("left", "right"):
        g = grips[f"{side}_gripper"]
        for pt in _pad_points(g):
            lq = R.T @ (pt - p)
            if (-0.100 <= lq[0] <= 0.100 and -0.220 <= lq[1] <= 0.220 and 0.100 <= lq[2] <= 0.200):
                _MECH.pinched = True
    _MECH.last_lift = lift
    _MECH.t += float(base.step_dt)

    if int(base.episode_length_buf[0]) % 40 == 0:
        g = grips["left_gripper"]
        print(f"[{TAG}] t={_MECH.t:.2f}s lift {lift * 1000:.1f} mm, arms {math.degrees(arm_l):+.0f}/"
              f"{math.degrees(arm_r):+.0f} deg, latches {math.degrees(float(j['LatchL_Swing'])):+.0f}/"
              f"{math.degrees(float(j['LatchR_Swing'])):+.0f} deg, tools {'in' if _tool_in_tray(env) else 'OUT'}"
              f"{'  DROPPED' if _MECH.dropped else ''}{'  PINCHED' if _MECH.pinched else ''}"
              f"{'  TIPPED' if _MECH.tipped else ''}  L grip {g.contact:.1f} N", flush=True)


def _tilt(quat) -> float:
    R = _rot(quat)
    return math.degrees(math.acos(min(1.0, max(-1.0, float(R[2, 2])))))


def extra_states(env, out, origin):
    tb = env.scene["toolbox"]
    ids = _joint_ids(tb)
    q = tb.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
    j = dict(zip(JOINTS, q, strict=True))
    out["joints"] = j
    out["latch"] = (float(j["LatchL_Swing"]), float(j["LatchR_Swing"]))
    out["lift"] = float(j["Top_Lift"])
    for body in ("WingL", "WingR", "Top", "LatchL", "LatchR"):
        bp, bq = _pose(tb, body)
        out[body.lower()] = (bp, bq)
    out["base"] = _pose(tb, "Base")
    out["handle_bar"] = _bar_world(tb)
    out["knobs"] = (_knob_world(tb, 1), _knob_world(tb, -1))
    out["box0"] = _MECH.box0
    out["forced"] = _MECH.forced
    out["dropped"] = _MECH.dropped
    out["pinched"] = _MECH.pinched
    out["tipped"] = _MECH.tipped
    out["off_table"] = _MECH.off_table


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("both latches released", "lid lifted", "trays unfolded", "trays/tools held, box stable, hands clear")
POINTS = 0.25


def _latches_open(objects) -> bool:
    return bool(objects["latch"][0] <= LATCH_OPEN and objects["latch"][1] <= LATCH_OPEN)


def _lifted(objects) -> bool:
    return bool(objects["lift"] >= LIFT_START)


def _unfolded(objects) -> bool:
    j = objects["joints"]
    return bool(objects["lift"] >= LIFT_OPEN and j["ArmL_Swing"] <= ARM_L_PART and j["ArmR_Swing"] >= ARM_R_PART)


def _tilt_deg(objects) -> float:
    return _tilt(objects["toolbox"].quat_xyzw)


def _moved(objects) -> float:
    b0 = objects.get("box0")
    if b0 is None:
        return 0.0
    return float(np.linalg.norm(np.asarray(objects["toolbox"].pos) - np.asarray(b0[0])))


def _stable(objects) -> bool:
    if _tilt_deg(objects) > STABLE_TILT or _moved(objects) > STABLE_MOVE:
        return False
    # all four feet on the bench: the base's underside is the root, the feet are at +-0.080, +-0.200 and z 0
    p = np.asarray(objects["toolbox"].pos, float)
    R = _rot(objects["toolbox"].quat_xyzw)
    for sx in (-0.080, 0.080):
        for sy in (-0.200, 0.200):
            if float((p + R @ np.array([sx, sy, 0.0]))[2]) > TOP + 0.004:
                return False
    return True


def _tool_inside(objects, name: str, frame_body: str | None, bounds) -> bool:
    """The tool root in the tray frame from the state's body poses (used by the scorer; the step event reads the
    actual assets, this reads the same poses the tests construct)."""
    key = (frame_body or "base").lower()
    fp, fq = objects[key]
    tp = np.asarray(objects[name].pos, float)
    q = _rot(fq).T @ (tp - np.asarray(fp, float))
    return bool(bounds["x"][0] - TOOL_SLACK <= q[0] <= bounds["x"][1] + TOOL_SLACK
                and bounds["y"][0] - TOOL_SLACK <= q[1] <= bounds["y"][1] + TOOL_SLACK
                and bounds["z"][0] - 0.020 <= q[2] <= bounds["z"][1] + 0.030)


def tools_in_place(objects) -> bool:
    return (_tool_inside(objects, "wrench", "wingl", WRENCH_BOUNDS)
            and _tool_inside(objects, "screwdriver", "wingr", SCREW_BOUNDS)
            and _tool_inside(objects, "hammer", "base", HAMMER_BOUNDS))


def _clear(objects) -> bool:
    """Both grippers are away from the box and everything on it: no grip, and no pad point within CLEAR_DIST of
    any body."""
    tb = objects["toolbox"]
    p = np.asarray(tb.pos, float)
    R = _rot(tb.quat_xyzw)
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.contact >= 0.5:
            return False
        for pt in _pad_points(g):
            lq = R.T @ (pt - p)
            if (-0.230 - CLEAR_DIST <= lq[0] <= 0.230 + CLEAR_DIST
                    and -0.240 - CLEAR_DIST <= lq[1] <= 0.240 + CLEAR_DIST
                    and -0.02 - CLEAR_DIST <= lq[2] <= 0.32 + CLEAR_DIST):
                return False
    return True


def steps_done(objects) -> tuple[bool, ...]:
    return (_latches_open(objects), _latches_open(objects) and _lifted(objects),
            _latches_open(objects) and _unfolded(objects),
            _latches_open(objects) and _unfolded(objects) and tools_in_place(objects)
            and _stable(objects) and _clear(objects))


def _bad(objects) -> bool:
    return bool(objects.get("tipped") or objects.get("off_table") or objects.get("forced")
                or objects.get("dropped") or objects.get("pinched"))


_HOLD = {"t": None}


def _reset_hold():
    _HOLD["t"] = None


def _full(objects) -> bool:
    """Open, tools in their trays, box stable, both hands clear, no damage latch."""
    return bool(all(steps_done(objects)[:3]) and tools_in_place(objects) and _stable(objects) and _clear(objects)
                and not _bad(objects))


def success(objects) -> bool:
    """The full state held for 1 s (the task times this hold itself; SUCCESS_HOLD_S is 0)."""
    if not _full(objects):
        _reset_hold()
        return False
    if _HOLD["t"] is None:
        _HOLD["t"] = 0.0
        return False
    _HOLD["t"] += 0.05
    return _HOLD["t"] >= 1.0 - 1e-9


def score(objects) -> float:
    if objects.get("tipped") or objects.get("off_table"):
        _reset_hold()
        return 0.0
    done = steps_done(objects)
    pts = 0.0
    if done[0]:
        pts = POINTS
    if done[1]:
        pts = 2 * POINTS
    if done[2]:
        pts = 3 * POINTS
    if objects.get("forced") or objects.get("dropped") or objects.get("pinched"):
        pts = min(pts, 2 * POINTS)
    if _full(objects) and _HOLD["t"] is not None and _HOLD["t"] >= 1.0 - 1e-9:
        return 1.0
    return round(pts, 6)


# -- demo ----------------------------------------------------------------------------------------------------------
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


def _latch_frame() -> np.ndarray:
    """The gripper frame at the latch: pointing forward (+x) and 20 degrees down, fingers closing vertically (one
    above the round knob, one below).  The knob is a cylinder along y, so it turns under the pads as the latch
    swings; the hand keeps this orientation through the pull."""
    a = math.radians(20.0)
    z = np.array([math.cos(a), 0.0, -math.sin(a)])
    x = np.array([math.sin(a), 0.0, math.cos(a)])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _handle_frame() -> np.ndarray:
    """The gripper at the wire handle: pointing forward (+x) and 45 degrees down, fingers closing along y across the
    8 mm bar (the bar runs along x)."""
    a = math.radians(45.0)
    z = np.array([math.cos(a), 0.0, -math.sin(a)])
    x = np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _latch_plan(side: int, wait_s: float = 0.0):
    s = 1 if side == 1 else -1
    pre = (LATCH_KNOB[side][0] - 0.033, s * 0.160, LATCH_KNOB[side][2])
    grip = tuple(LATCH_KNOB[side])
    pull = tuple(LATCH_OPEN_KNOB[side])
    wps = []
    if wait_s:
        wps.append(Waypoint(wait_s, tuple(_START_TCP["left" if side == 1 else "right"]), FINGER_OPEN, "wait",
                            rot=_START_ROT["left" if side == 1 else "right"]))
    wps += [
        Waypoint(1.6, pre, OPEN_FINGER, "to the latch", rot=_latch_frame()),
        Waypoint(0.8, grip, OPEN_FINGER, "pads astride the knob", rot=_latch_frame()),
        Waypoint(0.7, grip, LATCH_HOLD, "grip the knob", rot=_latch_frame()),
        Waypoint(1.4, pull, LATCH_HOLD, "pull the latch", rot=_latch_frame()),
        Waypoint(0.3, pull, LATCH_HOLD, "hold", rot=_latch_frame()),
        Waypoint(0.4, pull, OPEN_FINGER, "let go", rot=_latch_frame()),
        Waypoint(0.7, pre, OPEN_FINGER, "back off", rot=_latch_frame()),
    ]
    return wps


def _handle_plan():
    pre = (HANDLE_BAR[0] - 0.040, 0.0, HANDLE_BAR[2] + 0.042)
    grip = tuple(HANDLE_BAR)
    top = tuple(HANDLE_TOP)
    mid1 = (HANDLE_BAR[0], 0.0, HANDLE_BAR[2] + 0.035)
    mid2 = (HANDLE_BAR[0], 0.0, HANDLE_BAR[2] + 0.070)
    return [
        Waypoint(1.7, pre, OPEN_FINGER, "over the tray handle", rot=_handle_frame()),
        Waypoint(0.9, grip, OPEN_FINGER, "around the grip bar", rot=_handle_frame()),
        Waypoint(0.7, grip, HANDLE_HOLD, "grip the handle", rot=_handle_frame()),
        Waypoint(1.2, mid1, HANDLE_HOLD, "lift the lid", rot=_handle_frame()),
        Waypoint(1.2, mid2, HANDLE_HOLD, "open the arms", rot=_handle_frame()),
        Waypoint(1.2, top, HANDLE_HOLD, "trays open", rot=_handle_frame()),
        Waypoint(0.4, top, HANDLE_HOLD, "hold it open", rot=_handle_frame()),
        Waypoint(0.4, top, OPEN_FINGER, "let go", rot=_handle_frame()),
        Waypoint(0.8, pre, OPEN_FINGER, "lift away", rot=_handle_frame()),
        Waypoint(2.2, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


def demo():
    """The right arm releases the right latch; the left releases the left latch, then lifts the lid's wire handle
    until the arms are fully open.  The lid's detent holds it open after the left hand lets go.  The box is never
    pushed or lifted: only the two latch pulls (about 7 N) and the lid's 0.5 kg act on it."""
    right = _latch_plan(-1)
    right += [Waypoint(2.0, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    latch_pre = (LATCH_KNOB[1][0] - 0.033, 0.160, LATCH_KNOB[1][2])
    left = _latch_plan(1) + [
        Waypoint(0.5, latch_pre, OPEN_FINGER, "wait for the right latch", rot=_latch_frame())] + _handle_plan()
    return {"left": left, "right": right}


# -- optional closed-loop trace for tuning --------------------------------------------------------------------------
def trace(objects) -> str:
    return (f"lift {objects['lift'] * 1000:.1f} mm, latches "
            f"{math.degrees(objects['latch'][0]):+.0f}/{math.degrees(objects['latch'][1]):+.0f} deg, "
            f"tilt {_tilt_deg(objects):.2f} deg, moved {_moved(objects) * 1000:.1f} mm, "
            f"tools {'in' if tools_in_place(objects) else 'OUT'}")
