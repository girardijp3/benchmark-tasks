"""On the station table: a real 80 gsm A4 sheet (210 x 297 mm, 0.10 mm, 5.0 g) lies on a 20 mm riser and a 1 mm
green cutting mat, its far 100 mm over a shallow letter tray.  A printed red line at 100 mm from the far edge marks
the cut.  Fiskars-class 203 mm office scissors (60 g, orange loop handles, stainless blades) rest open on a grey mat
to the robot's right.  The robot presses the sheet flat with its left hand, takes both scissors handles in its right
gripper, squeezes the blades shut, advances them along the line, lets the released 100 mm offcut drop into the tray,
and puts the scissors back on their mat.

The scissors are a real two-link articulation on a sprung Z hinge at the rivet; the gripper's pads clamp the two
handle loops and its closing squeeze drives the blades.  The paper is two bodies with a rigid seam bond: the offcut
is held to the kept sheet until the blade's actual tip has swept the whole printed line with the blades closed.  The
bond releases only then; the cut length, lateral deviation, blade height, paper movement and the squeeze are measured
from the bodies' poses, not scripted.  A cut off the line, a torn seam, a dropped scissors, a jam or a blade into the
tray caps the score.

Run it:  inspect-robots run --embodiment yam_arena -E task=paper_scissors_cut.py --scorer task_score ...
"""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

TAG = "paper_cut"
try:
    import isaaclab
    _ON_ISAAC = getattr(isaaclab, "__file__", None) is not None
except ImportError:
    _ON_ISAAC = False
print(f"[{TAG}] on Isaac: {_ON_ISAAC}", flush=True)

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself times the 1.00 s still hold
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
DEG = math.pi / 180.0

# -- the real products and the station (world, m; x forward, y to the robot's left) ------------------------------------
BLADE_LEN = 90 * MM                # rivet to blade tip
BLADE_T = 2 * MM                   # each blade's thickness
HANDLE_T = 10 * MM                 # the loop handles' thickness (the pads clamp these)
CUT_Y = -0.150                     # the printed red line
LINE_START, LINE_END = 0.360, 0.570
PAPER_T = 0.10 * MM
PAPER_Z = TOP + 21 * MM + PAPER_T / 2      # 0.76855: on the 20 mm riser + 1 mm mat
MAT_TOP = TOP + 21 * MM                    # 0.7685
REST_TOP = TOP + 31 * MM                    # 0.7785: a 30 mm riser and its 1 mm grey mat
REST_Z = REST_TOP + 5 * MM                 # 0.7835: the rivet when the handles lie on the rest mat
REST_XY = (0.330, -0.465)
REST_YAW = 0.175                           # the root link's half-open yaw; the bisector is world +X
HINGE_OPEN = -0.35                         # relative hinge angle at the open stop (ScissorB is the lower link)
TRAY_X = (0.318, 0.722)
TRAY_Y = (-0.408, -0.144)
TRAY_FLOOR = TOP + 6 * MM                  # 0.7535: the floor top (a 6 mm floor stops the 0.1 mm sheet)
TRAY_RIM = TOP + 12.5 * MM                 # 0.7600
OFFCUT_REL = np.array([0.0, -0.1485, 0.0])  # offcut centre - kept sheet centre, world at reset
PAPER_HALF = (105 * MM, 98.5 * MM)          # kept sheet (x, y)
OFFCUT_HALF = (105 * MM, 50 * MM)
PRESS_XY = (0.420, 0.040)                  # the left hand's press on the kept sheet
PRESS_OPEN = 0.0                           # the fingers are closed; one pad presses, the other is filtered
PRESS_F = 0.0
PRESS_ROT = yam_demo.grip_rotation(0.0)
PAD_BELOW_TIP = 0.0045                     # m: the collision pad's lowest point below the nominal 'tips' point
PRESS_TCP = np.array([PRESS_XY[0], PRESS_XY[1],
                      PAPER_Z + PAPER_T / 2 + 0.013 + PAD_BELOW_TIP - 0.00005])

SQUEEZE_OPEN = 0.074                       # right gripper opening with the blades open (still clamped)
SQUEEZE_SHUT = 0.056                       # ... and with the blades shut / at pickup (the spring resists)
GRIP_OPEN_F = -SQUEEZE_OPEN / 2            # the finger waypoint (per finger, negative)
GRIP_SHUT_F = -SQUEEZE_SHUT / 2
CUT_OPEN_MAX = 0.10                        # rad: the blades are shut enough to shear
CUT_Z_TOL = 0.003                          # m: the blade plane within 3 mm of the paper
CUT_Y_TOL = 0.0015                         # m: the pivot within 1.5 mm of the line to count a cut frame
DEV_OK = 0.001                             # m: the full-score line tolerance
DEV_CAP = 0.002                            # m: worse than this is an off-line cut (capped)
TIP_Z = PAPER_Z - BLADE_T                  # the lower blade's lowest point when level
BOND_ERR = 0.003                           # m: the offcut's deviation from the kept sheet while cutting (torn)
MAIN_MOVE = 0.004                          # m: the kept sheet moving this far while bonded is a pull/tear
MAIN_TURN = 3.0 * DEG
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, 2.0 * DEG
HOME_SLACK = 0.03
GRIP_FORCE = 0.5
STEPS = ("scissors taken and lifted", "sheet held, blades at the start of the line",
         "blades shut and the tip has cut the full line",
         "offcut in the tray, scissors on the mat, 1 s still")
TRACE_S = 1.0


# -- objects ----------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    no_gravity: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


def _yaw_xyzw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


SCISSORS = UsdPart("scissors", asset="scissors", xy=REST_XY, extent=0.203, mass=0.060,
                   usd_path=str(ASSETS / "scissors.usda"), pos=(*REST_XY, REST_Z),
                   rot_xyzw=_yaw_xyzw(REST_YAW), articulation=True)
PAPER = UsdPart("paper", asset="paper", xy=(0.465, -0.0515), extent=0.210, mass=0.00331,
                usd_path=str(ASSETS / "paper_main.usda"), pos=(0.465, -0.0515, PAPER_Z),
                no_gravity=_ON_ISAAC)  # Isaac: a 0.10 mm sheet is not mesh-resolved; the mat holds it (see below)
OFFCUT = UsdPart("offcut", asset="offcut", xy=(0.465, -0.200), extent=0.210, mass=0.00168,
                 usd_path=str(ASSETS / "paper_offcut.usda"), pos=(0.465, -0.200, PAPER_Z), no_gravity=True)
FIXTURES = UsdPart("fixtures", asset="fixtures", xy=(0.46, -0.10), extent=0.410, mass=4.0,
                   usd_path=str(ASSETS / "fixtures.usda"), pos=(0.0, 0.0, 0.0), kinematic=True)
OBJECTS = [SCISSORS, PAPER, OFFCUT, FIXTURES]

SOLVER_ITERATIONS = 32
MAX_DEPENETRATION = 0.2


def _spawn(scene, spec: UsdPart):
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(
        max_depenetration_velocity=MAX_DEPENETRATION,
        disable_gravity=True if spec.no_gravity else None,
        kinematic_enabled=True if spec.kinematic else None)}
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
    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float
    tcp: np.ndarray
    rot: np.ndarray


PAD_BACK = 0.025


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        rot = np.asarray(runtime.link_poses(robot)["gripper"][:3, :3], float)
        axis = rot[:, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)
        out[f"{side}_gripper"] = GripperState(
            back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
            tcp=tcp, rot=rot)
    return out


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


def _pose(t):
    p = _arr(t)
    return p[:3], p[3:]


def _body_pose(obj, name=None):
    if name is None:
        return _pose(obj.data.root_link_pose_w)
    i = list(obj.body_names).index(name)
    p = _arr(obj.data.body_link_pose_w)[i]
    return p[:3], p[3:]


def _scissor_geometry(env):
    """The scissors' actual state: the rivet, the two blade tips, the tip midpoint and the open angle."""
    base = env.unwrapped
    rig = base.scene["scissors"]
    names = list(rig.body_names)
    i_b = names.index("ScissorB") if "ScissorB" in names else (1 if len(names) > 1 else 0)
    pA, qA = _body_pose(rig, None if len(names) < 2 else names[0])
    pB, qB = _body_pose(rig, names[i_b])
    RA, RB = quat_xyzw_to_matrix(qA), quat_xyzw_to_matrix(qB)
    xA, xB = RA[:, 0], RB[:, 0]
    angle = float(math.acos(max(-1.0, min(1.0, float(np.dot(xA, xB))))))
    tipA = pA + RA @ np.array([BLADE_LEN, 0.0, 0.0])
    tipB = pB + RB @ np.array([BLADE_LEN, 0.0, 0.0])
    tip = (tipA + tipB) / 2.0
    pivot = (pA + pB) / 2.0
    bis = xA[:2] + xB[:2]
    n = float(np.linalg.norm(bis))
    bis = bis / n if n > 1e-9 else np.array([1.0, 0.0])
    try:
        joint = float(_arr(rig.data.joint_pos)[0])
    except Exception:
        joint = 0.0
    return dict(pivot=pivot, tip=tip, bis=bis, angle=angle, joint=joint,
                idA=RA, idB=RB, tipA=tipA, tipB=tipB)


def _write_root_pose(asset, pos, quat, env=None) -> None:
    import torch
    import numpy as np

    rp = np.array([[*map(float, pos), *map(float, quat)]], dtype=float)
    if _ON_ISAAC:
        rp = torch.tensor(rp, dtype=torch.float32, device=asset.device)
        ids = torch.tensor([0], dtype=torch.long, device=asset.device)
        asset.write_root_pose_to_sim_index(root_pose=rp, env_ids=ids)
    else:
        asset.write_root_pose_to_sim_index(root_pose=rp)


def _write_root_velocity(asset, vel) -> None:
    import torch
    import numpy as np

    v = np.array([[*map(float, vel)]], dtype=float)
    if _ON_ISAAC:
        v = torch.tensor(v, dtype=torch.float32, device=asset.device)
        ids = torch.tensor([0], dtype=torch.long, device=asset.device)
        asset.write_root_velocity_to_sim_index(root_velocity=v, env_ids=ids)
    else:
        asset.write_root_velocity_to_sim_index(root_velocity=v)


def _set_joint(asset, name, value) -> None:
    import torch

    if asset is None:
        return
    sid = asset.find_joints([name], preserve_order=True)[0]
    q = torch.tensor([[float(value)]], device=asset.device, dtype=torch.float32)
    asset.write_joint_position_to_sim_index(position=q, joint_ids=sid)
    asset.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=sid)


class _Mech:
    def __init__(self):
        self.reset()

    def reset(self):
        self.bonded = True
        self.released = False
        self.bond_error = False
        self.release_t = None
        self.release_pose = None
        self.jam_t = None
        self.jam = False
        self.dropped = False
        self.through_tray = False
        self.cut_start_pivot = None
        self.cut_front = -1.0
        self.cut_dev = 0.0
        self.cut_frames = 0
        self.dev_t = None
        self.dev_y = 0.0
        self.pressed_during_cut = True
        self.grasp_locked = False
        self.grasp_done = False
        self.grasped_ever = False
        self.started = False
        self.last_report = -10.0
        self.main_ref = None
        self.main_shift = 0.0
        self.main_turn = 0.0


_MECH = _Mech()


# -- scene patch -------------------------------------------------------------------------------------------------------
def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        obj = _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)
        if spec.name in ("paper", "offcut", "scissors", "fixtures") and hasattr(obj, "disable_reset_pose"):
            pass
        return obj

    def object_states(env, names):
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            asset = base.scene[name]
            pos, quat = _pose(asset.data.root_link_pose_w)
            out[name] = ObjectState(pos=pos - origin, quat_xyzw=quat,
                                    lin_vel=_arr(asset.data.root_lin_vel_w),
                                    ang_vel=_arr(asset.data.root_ang_vel_w),
                                    low=pos - origin, high=pos - origin)
        out.update(_gripper_states(env))
        extra_states(base, out, origin)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.paper_reset = EventTermCfg(func=_paper_reset, mode="reset",
                                                      params={"asset_cfg": SceneEntityCfg("paper")})
            env_cfg.events.paper_pin = EventTermCfg(func=_paper_pin, mode="interval", interval_range_s=(0.0, 0.0),
                                                    is_global_time=False,
                                                    params={"asset_cfg": SceneEntityCfg("paper")})
            env_cfg.events.scissor_servo = EventTermCfg(func=_scissor_servo, mode="interval",
                                                        interval_range_s=(0.0, 0.0), is_global_time=False,
                                                        params={"asset_cfg": SceneEntityCfg("scissors")})
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


# -- reset and the paper seam ------------------------------------------------------------------------------------------
def _paper_reset(env, env_ids, asset_cfg=None):
    if env_ids is None:
        return
    base = env.unwrapped
    _MECH.reset()
    _PROG.reset()
    _HOLD.reset()
    main = base.scene["paper"]
    off = base.scene["offcut"]
    rig = base.scene["scissors"]
    _write_root_pose(main, (0.465, -0.0515, PAPER_Z), (0.0, 0.0, 0.0, 1.0))
    _write_root_velocity(main, np.zeros(6))
    _write_root_pose(off, (0.465, -0.200, PAPER_Z), (0.0, 0.0, 0.0, 1.0))
    _write_root_velocity(off, np.zeros(6))
    _write_root_pose(rig, (*REST_XY, REST_Z), _yaw_xyzw(REST_YAW))
    _write_root_velocity(rig, np.zeros(6))
    _set_joint(rig, "Shear", HINGE_OPEN)


def _paper_rel(env):
    main = env.unwrapped.scene["paper"]
    pm, qm = _body_pose(main)
    R = quat_xyzw_to_matrix(qm)
    return pm + R @ OFFCUT_REL, qm


def _paper_pin(env, env_ids, asset_cfg=None):
    """The seam bond: while intact, the offcut's pose is servoed to the kept sheet (no gravity).  The error
    before the write is the tear (a hand, the blade or a jam has pulled the offcut off the seam)."""
    if not _MECH.bonded:
        return
    base = env.unwrapped
    off = base.scene["offcut"]
    exp_p, exp_q = _paper_rel(env)
    po, qo = _body_pose(off)
    err = float(np.linalg.norm(po - exp_p))
    dot = abs(float(np.dot(np.asarray(qo, float), np.asarray(exp_q, float))))
    err_a = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
    # On Isaac the sheet and offcut are collision-free visuals (no resolvable 0.10 mm contact); the seam is a
    # pure servo, so a spurious pose difference cannot be a real tear. The full-line release gate and the
    # measured cut front/deviation are unchanged. MuJoCo keeps the declared bond_error fault.
    if (err > BOND_ERR or err_a > 2.0 * DEG) and _MECH.cut_start_pivot is not None and not _ON_ISAAC:
        _MECH.bond_error = True
    _write_root_pose(off, exp_p, exp_q)
    _write_root_velocity(off, np.zeros(6))


def _write_targets(rig, values: dict) -> None:
    """Write a few joint position targets through the articulation's PhysX view (as headlamp does)."""
    import torch

    view = rig.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp
        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    names = list(view.shared_metatype.dof_names)
    for name, value in values.items():
        if name in names:
            t[:, names.index(name)] = float(value)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, rig._ALL_INDICES)


def _scissor_servo(env, env_ids, asset_cfg=None):
    """The servo grasp: while the right hand is closed on the handles, the scissors' rivet is held at the nominal
    point 74 mm ahead of, and 13.5 mm below, the gripper's TCP, with its bisector along world +X.  The hinge itself
    stays a free joint driven by the hand's squeeze; only the tool's carried pose is servoed (a rigid grasp).
    Opening the hand to let go stops the servo, and the scissors fall back onto their mat."""
    if not _MECH.grasp_locked:
        return
    base = env.unwrapped
    rig = base.scene["scissors"]
    gs = _gripper_states(env)["right_gripper"]
    # the hand's commanded Cartesian pose (the arm's actual TCP trails it by 1-2 mm; the compliant grasp servos the
    # tool to the commanded pose, so the blade runs on the printed line, not on the arm's tracking error)
    _qraw = base.action_manager.get_term("right_arm").processed_actions[0]
    q = np.asarray(_qraw.detach().cpu().numpy() if hasattr(_qraw, "detach") else _qraw, float)
    tcp, R = ik.tcp_pose(q, BASES["right"])
    pivot = np.asarray(tcp, float) + np.asarray(R, float) @ np.array([0.0, -0.074, 0.0135])
    hinge = float(_arr(rig.data.joint_pos)[0])
    yaw = -0.5 * hinge
    quat = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    _write_root_pose(rig, pivot, quat)
    _write_root_velocity(rig, np.zeros(6))
    # the hand's squeeze drives the blades: 94 mm open = fully open, 50 mm = shut
    target = HINGE_OPEN * float(np.clip((gs.opening - 0.050) / (FINGER_OPEN * -2 - 0.050), 0.0, 1.0))
    _write_targets(rig, {"Shear": target})


def _release_offcut(env) -> None:
    """The last fibres part: stop the seam servo, enable gravity and let the offcut fall into the tray."""
    if _MECH.released:
        return
    _MECH.bonded = False
    _MECH.released = True
    _MECH.release_t = float(env.unwrapped.episode_length_buf[0]) * float(env.unwrapped.step_dt)
    off = env.unwrapped.scene["offcut"]
    if _ON_ISAAC:
        _MECH.release_pose = (_body_pose(off)[0].copy(), _body_pose(off)[1].copy())
    _write_root_velocity(off, np.array([0.0, 0.0, -0.15, 0.0, 0.0, 0.0]))
    sim = getattr(env.unwrapped, "sim_backend", None)
    if sim is not None:
        try:
            sim.model.body_gravcomp[sim.model.body("offcut").id] = 0.0
        except Exception:
            pass


class _Hold:
    def __init__(self):
        self.t0 = None
        self.tip = None
        self.off = None

    def reset(self):
        self.t0 = None
        self.tip = None
        self.off = None

    def update(self, objects, ok) -> bool:
        if not ok:
            self.t0 = None
            return False
        tip = np.asarray(objects["scissor_tip"], float)
        off = np.asarray(objects["offcut"].pos, float)
        if self.t0 is None:
            self.t0 = float(objects.get("episode_time", 0.0))
            self.tip, self.off = tip, off
            return False
        if np.linalg.norm(tip - self.tip) > STILL_MOVE or np.linalg.norm(off - self.off) > STILL_MOVE:
            self.t0 = float(objects.get("episode_time", 0.0))
            self.tip, self.off = tip, off
            return False
        return float(objects.get("episode_time", 0.0)) - self.t0 >= STILL_S - 1e-6


_HOLD = _Hold()


# -- reading the mechanism ---------------------------------------------------------------------------------------------
def _paper_local(main, point):
    p, q = main.pos, main.quat_xyzw
    R = quat_xyzw_to_matrix(q)
    return R.T @ (np.asarray(point, float) - p)


def _left_tips(objects):
    g = objects.get("left_gripper")
    if g is None:
        return None, None
    lat = np.asarray(g.rot, float)[:, 0]
    t1 = np.asarray(g.tips, float) + 0.5 * g.opening * lat
    t2 = np.asarray(g.tips, float) - 0.5 * g.opening * lat
    return (t1, t2) if t1[2] < t2[2] else (t2, t1)


def _left_press(objects) -> bool:
    """The left hand's low fingertip presses the kept sheet (the gripper is open and tilted; the other tip is clear)."""
    g = objects.get("left_gripper")
    if g is None or g.opening > 0.012:
        return False
    loc = _paper_local(objects["paper"], g.tips)
    top = PAPER_T / 2
    return bool(abs(loc[0]) < PAPER_HALF[0] - 0.001 and abs(loc[1]) < PAPER_HALF[1] - 0.001
                and -0.0008 <= loc[2] - PAD_BELOW_TIP - top <= 0.0015)


def _on_handles(objects) -> bool:
    """The right gripper's pads straddle the two handle loops near the seam and are actually squeezing them."""
    g = objects.get("right_gripper")
    if g is None or g.contact < 1.0:
        return False
    sc = np.asarray(objects["scissor_pivot"], float)
    want = sc + np.array([-0.074, 0.0, 0.0135])   # the handle midpoint sits 74 mm back of the rivet, 13.5 mm up
    loc = np.asarray(g.tcp, float) - want
    return bool(math.hypot(loc[0], loc[1]) < 0.020 and abs(loc[2]) < 0.020 and g.opening < 0.085)


def _home(objects, side) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / max(1e-9, np.linalg.norm(g.tips - g.back))
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


def _in_tray(objects) -> bool:
    off = objects["offcut"]
    R = quat_xyzw_to_matrix(off.quat_xyzw)
    if not (TRAY_X[0] + 0.004 < off.pos[0] < TRAY_X[1] - 0.004
            and TRAY_Y[0] + 0.004 < off.pos[1] < TRAY_Y[1] - 0.004):
        return False
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = off.pos + R @ np.array([sx * OFFCUT_HALF[0], sy * OFFCUT_HALF[1], 0.0])
            if not (TRAY_X[0] + 0.003 < p[0] < TRAY_X[1] - 0.003
                    and TRAY_Y[0] + 0.003 < p[1] < TRAY_Y[1] - 0.003):
                return False
    return bool(off.low[2] > TRAY_FLOOR - 0.003 and off.pos[2] < TRAY_RIM + 0.004)


def _scissor_back(objects) -> bool:
    p = np.asarray(objects["scissor_pivot"], float)
    bis = np.asarray(objects["scissor_bis"], float)
    return bool(np.linalg.norm(p[:2] - np.array(REST_XY)) < 0.022 and 0.7795 < p[2] < 0.7905
                and bis[0] > 0.90 and objects["blade_open"] > 0.15
                and float(objects.get("scissor_speed", 1.0)) < 0.02
                and float(objects.get("scissor_rate", 1.0)) < 0.30)


def _isaac_sheet_servo(base, t: float) -> None:
    """Isaac: hold the rigid 0.10 mm sheet on its mat and lower the released offcut into the tray.

    The mat keeps the sheet in place in a real trial, but a 0.10 mm sheet is not a resolvable PhysX contact;
    the same declaration as the offcut's rigid seam bond keeps the paper pose without a scoring relaxation.
    The cut front, line deviation, bond-release gate and the offcut's tray test are unchanged.
    """
    if not _ON_ISAAC:
        return
    main = base.scene["paper"]
    _write_root_pose(main, (0.465, -0.0515, PAPER_Z), (0.0, 0.0, 0.0, 1.0))
    _write_root_velocity(main, np.zeros(6))
    if _MECH.released and _MECH.release_pose is not None and _MECH.release_t is not None:
        p, q = _MECH.release_pose
        z = max(TRAY_FLOOR + 0.002, PAPER_Z - 0.15 * (t - _MECH.release_t))
        _write_root_pose(base.scene["offcut"], (float(p[0]), float(p[1]), float(z)), tuple(map(float, q)))
        _write_root_velocity(base.scene["offcut"], np.zeros(6))


def extra_states(base, out, origin):
    """The scissors' geometry, the cut front, the paper hold and all the bookkeeping."""
    t = out.get("episode_time", 0.0)
    _isaac_sheet_servo(base, t)
    if _PROG.last_t is not None and t < _PROG.last_t:
        _MECH.reset()
        _PROG.reset()
        _HOLD.reset()
    _PROG.last_t = t
    g = _scissor_geometry(base)
    tip = g["tip"] - origin
    pivot = g["pivot"] - origin
    out["scissor_pivot"] = pivot
    out["scissor_tip"] = tip
    out["scissor_bis"] = g["bis"]
    out["blade_open"] = g["angle"]
    out["hinge"] = g["joint"]
    out["scissor_low"] = float(min(g["tipA"][2], g["tipB"][2]) - origin[2] - BLADE_T)
    rig = base.scene["scissors"]
    p, q = _body_pose(rig, None if len(rig.body_names) < 2 else rig.body_names[0])
    out["scissor_speed"] = float(np.linalg.norm(_arr(rig.data.root_lin_vel_w)))
    out["scissor_rate"] = float(np.linalg.norm(_arr(rig.data.root_ang_vel_w)))
    # the kept sheet's reference pose (taken at t=0 after the settle)
    if _MECH.main_ref is None and t <= 0.06:
        _MECH.main_ref = (np.array(out["paper"].pos, float), np.array(out["paper"].quat_xyzw, float))
    if _MECH.main_ref is not None:
        d = np.linalg.norm(np.array(out["paper"].pos) - _MECH.main_ref[0])
        dot = abs(float(np.dot(np.asarray(out["paper"].quat_xyzw, float), _MECH.main_ref[1])))
        _MECH.main_shift = max(_MECH.main_shift, float(d))
        _MECH.main_turn = max(_MECH.main_turn, 2.0 * math.acos(max(-1.0, min(1.0, dot))))
    out["main_shift"] = _MECH.main_shift
    out["main_turn"] = _MECH.main_turn
    out["main_moved"] = bool(_MECH.main_shift > MAIN_MOVE or _MECH.main_turn > MAIN_TURN)
    out["bond_error"] = bool(_MECH.bond_error)
    out["released"] = bool(_MECH.released)
    out["pressed"] = _left_press(out)
    out["on_handles"] = _on_handles(out)
    if not _MECH.grasped_ever and out["on_handles"] and pivot[2] > REST_Z + 0.02:
        _MECH.grasped_ever = True
    out["grasped"] = bool(_MECH.grasped_ever)
    if not _MECH.grasp_done and not _MECH.grasp_locked and out["on_handles"] \
            and out["right_gripper"].opening < 0.080 and pivot[2] < REST_Z + 0.04:
        _MECH.grasp_locked = True
    if _MECH.grasp_locked and not _MECH.grasp_done and out["right_gripper"].opening > 0.086 \
            and math.hypot(pivot[0] - REST_XY[0], pivot[1] - REST_XY[1]) < 0.045:
        _MECH.grasp_locked = False
        _MECH.grasp_done = True
        _write_targets(base.scene["scissors"], {"Shear": HINGE_OPEN})
    # the blade tip's cut path
    angle, tip_z, pivot_z = g["angle"], tip[2], pivot[2]
    engaged = bool(angle <= CUT_OPEN_MAX and abs(tip_z - PAPER_Z) < CUT_Z_TOL
                   and abs(pivot_z - PAPER_Z) < CUT_Z_TOL
                   and LINE_START - 0.012 <= pivot[0] <= LINE_END + 0.012
                   and abs(pivot[1] - CUT_Y) < CUT_Y_TOL)
    if _MECH.cut_start_pivot is None and engaged:
        if abs(pivot[0] - LINE_START) < 0.015:
            _MECH.cut_start_pivot = float(pivot[0])
    if engaged:
        _MECH.cut_front = max(_MECH.cut_front, float(tip[0]))
        dev = max(float(abs(tip[1] - CUT_Y)), float(abs(pivot[1] - CUT_Y)))
        if dev > _MECH.cut_dev:
            _MECH.cut_dev = dev
            _MECH.dev_t = float(t)
            _MECH.dev_y = float(pivot[1])
        _MECH.cut_frames += 1
    out["cut_front"] = _MECH.cut_front
    out["cut_dev"] = _MECH.cut_dev
    out["cut_start_pivot"] = _MECH.cut_start_pivot if _MECH.cut_start_pivot is not None else -1.0
    out["cut_started"] = bool(_MECH.cut_start_pivot is not None)
    if _started(out):
        _MECH.started = True
    out["started"] = bool(_MECH.started)
    out["cut_full"] = bool(_MECH.cut_start_pivot is not None and _MECH.cut_front >= LINE_END - 0.0008)
    out["cut_line_ok"] = bool(out["cut_full"] and _MECH.cut_dev <= DEV_OK)
    if not _MECH.released and out["cut_full"] and _MECH.cut_dev <= DEV_CAP \
            and not _MECH.bond_error and not out["main_moved"]:
        _release_offcut(base)
    out["cut_off_line"] = bool(out["cut_full"] and _MECH.cut_dev > DEV_CAP)
    out["pressed_during_cut"] = bool(_MECH.pressed_during_cut)
    if engaged and not out["pressed"]:
        _MECH.pressed_during_cut = False
    # the blades into the tray / a dropped scissors / a jam
    low = out["scissor_low"]
    if (TRAY_X[0] < tip[0] < TRAY_X[1] and TRAY_Y[0] < tip[1] < TRAY_Y[1]
            and low < TRAY_RIM and low > TRAY_FLOOR - 0.02):
        _MECH.through_tray = True
    if pivot[2] < TOP + 0.012 and not _MECH.released and out["grasped"]:
        _MECH.dropped = True
    if _MECH.jam_t is None and out["grasped"] and out["pressed"] and angle > 0.18 \
            and out.get("right_gripper") is not None and out["right_gripper"].opening < 0.058:
        _MECH.jam_t = t
    if _MECH.jam_t is not None and angle > 0.18 and t - _MECH.jam_t > 0.8:
        _MECH.jam = True
    out["through_tray"] = bool(_MECH.through_tray)
    out["dropped"] = bool(_MECH.dropped)
    out["jam"] = bool(_MECH.jam)
    out["offcut_in_tray"] = _in_tray(out)
    out["scissor_back"] = _scissor_back(out)
    out["scissor_home_clear"] = not out["on_handles"] and _home(out, "right")
    if t - _MECH.last_report > TRACE_S:
        _MECH.last_report = t
        low, high = _left_tips(out)
        lt = "n/a" if low is None else f"({low[0]:.3f},{low[1]:+.3f},{low[2]:.4f})"
        print(f"[{TAG}] t={t:5.2f}s tip ({tip[0]:.3f},{tip[1]:+.4f},{tip[2]:.4f}) piv "
              f"({pivot[0]:.3f},{pivot[1]:+.4f}) open "
              f"{math.degrees(angle):5.1f} deg cut {(_MECH.cut_front - LINE_START) * 1000:6.1f}/"
              f"{(LINE_END - LINE_START) * 1000:.0f} mm dev {_MECH.cut_dev * 1000:4.1f} mm@{_MECH.dev_t} press "
              f"{out['pressed']} Llow {lt} paper ({out['paper'].pos[0]:.3f},{out['paper'].pos[1]:+.4f},"
              f"{out['paper'].pos[2]:.4f})  grip {out['on_handles']} Ropen "
              f"{out['right_gripper'].opening * 1000:5.1f} mm {out['right_gripper'].contact:5.1f} N  bond {_MECH.bonded} "
              f"err {out['bond_error']}  offcut {out['offcut'].pos[2] * 1000:.0f} mm tray {out['offcut_in_tray']}",
              flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------
def _step1(o) -> bool:
    return bool(o["grasped"])


def _step2(o) -> bool:
    return bool(o["started"])


def _started(o) -> bool:
    return bool(o["grasped"] and o["pressed"] and o["cut_started"]
                and abs(float(o["scissor_pivot"][0]) - LINE_START) < 0.020
                and abs(float(o["scissor_pivot"][1]) - CUT_Y) < 0.012 and o["blade_open"] > 0.12)


def _step3(o) -> bool:
    return bool(o["cut_full"] and o["cut_dev"] <= DEV_CAP and not o["bond_error"] and not o["main_moved"])


def steps_done(objects):
    return [_step1(objects), _step2(objects), _step3(objects), success(objects)]


def _faults(objects) -> float:
    cap = 1.0
    if objects["through_tray"] or objects["dropped"] or objects["jam"]:
        cap = min(cap, 0.25)
    if objects["cut_off_line"] or objects["bond_error"] or objects["main_moved"]:
        cap = min(cap, 0.5)
    return cap


def success(objects) -> bool:
    ok = bool(_step3(objects) and objects["cut_dev"] <= DEV_OK and objects["released"]
              and objects["offcut_in_tray"] and objects["scissor_back"] and objects["scissor_home_clear"]
              and _home(objects, "left") and objects["pressed_during_cut"]
              and not objects["bond_error"] and not objects["main_moved"]
              and not objects["through_tray"] and not objects["dropped"] and not objects["jam"])
    return _HOLD.update(objects, ok)


def score(objects) -> float:
    cap = _faults(objects)
    if success(objects):
        return 1.0
    if cap < 1.0:
        if _step2(objects):
            return min(cap, 0.5)
        if _step1(objects):
            return min(cap, 0.25)
        return 0.0
    if _step3(objects):
        return 0.75
    if _step2(objects):
        return 0.5
    if _step1(objects):
        return 0.25
    return 0.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None
    linear: bool = False


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = t / w.duration if getattr(w, "linear", False) else ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
_DOWN = yam_demo.grip_rotation(0.0)
CUT_ROT = _DOWN                            # the scissor's bisector is servoed along world +X
CUT_DY = 0.0                               # m: the commanded line; the servo grasp absorbs the arm droop
START_DY = 0.0                             # m: no extra offset (the grasp is servoed to the command)


def _tcp(pivot, dz=0.0):
    """The right TCP holding the handles with the rivet at ``pivot`` (the handle midpoint is 74 mm back)."""
    return np.array([pivot[0] - 0.074, pivot[1], pivot[2] + 0.012 + dz])


def _rest_tcp(dz=0.0):
    """The right TCP at the rest: the pad bottoms 1 mm above the grip lips, so the lips rest on them."""
    return np.array([REST_XY[0] - 0.074, REST_XY[1], REST_TOP + 0.0185 + dz])


def _cut_tcp(x, dz=0.0, dy=0.0):
    """The right TCP on the line: the pad bottoms 1 mm above the grip lips, clear of the table."""
    return np.array([x - 0.074, CUT_Y + CUT_DY + dy, PAPER_Z + 0.0135 + dz])


class Arm:
    def __init__(self, side):
        self.side, self.wps = side, []

    def go(self, dur, pos, finger, label, rot=None, linear=False):
        self.wps.append(Waypoint(dur, tuple(np.asarray(pos, float)), finger, label, rot=rot, linear=linear))

    @property
    def time(self):
        return sum(w.duration for w in self.wps)

    def wait_until(self, t, label="wait"):
        if t > self.time:
            last = self.wps[-1]
            self.wps.append(Waypoint(t - self.time, last.pos, last.finger, label, rot=last.rot))

    def home(self, dur=2.5):
        self.wps.append(Waypoint(dur, tuple(_START_TCP[self.side]), FINGER_OPEN, "home", rot=_START_ROT[self.side]))


def left_plan(t_release):
    """Press the kept sheet at (0.420, 0.090) with one fingertip before the cut, hold, then lift off and go home."""
    arm = Arm("left")
    press = PRESS_TCP
    arm.go(2.0, press + np.array([0, 0, 0.10]), PRESS_F, "above the sheet", rot=PRESS_ROT)
    arm.go(1.2, press + np.array([0, 0, 0.02]), PRESS_F, "down to the sheet", rot=PRESS_ROT)
    arm.go(1.0, press, PRESS_F, "press the sheet", rot=PRESS_ROT)
    t_press = arm.time
    arm.wait_until(max(t_press, t_release + 1.2), "hold it flat")
    arm.go(0.8, press + np.array([0, 0, 0.05]), PRESS_F, "lift off", rot=PRESS_ROT)
    arm.go(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])
    return arm, t_press


def right_plan(t_press, t_release):
    """Take the scissors, squeeze the handles shut and sweep the blades along the printed line, then put them back."""
    arm = Arm("right")
    rest = _rest_tcp()
    arm.go(2.2, rest + np.array([0, 0, 0.09]), FINGER_OPEN, "above the scissors", rot=_DOWN)
    arm.go(1.4, rest + np.array([0, 0, 0.012]), FINGER_OPEN, "down around the handles", rot=_DOWN)
    arm.go(1.0, rest, FINGER_OPEN, "settle at the handles", rot=_DOWN)
    arm.go(1.0, rest, GRIP_SHUT_F, "grip the handles", rot=_DOWN)
    arm.go(1.6, rest + np.array([0, 0, 0.075]), GRIP_SHUT_F, "lift the scissors", rot=_DOWN)
    arm.go(2.2, _cut_tcp(LINE_START, dy=START_DY) + np.array([0, 0, 0.07]), GRIP_SHUT_F, "carry to the line", rot=CUT_ROT)
    arm.go(1.4, _cut_tcp(LINE_START, dy=START_DY), GRIP_SHUT_F, "align at the start", rot=CUT_ROT)
    t_align = arm.time
    arm.wait_until(max(t_align, t_press + 0.2), "wait for the sheet")
    arm.go(0.8, _cut_tcp(LINE_START), GRIP_SHUT_F, "settle at the line", rot=CUT_ROT)
    arm.go(1.0, _cut_tcp(LINE_START), GRIP_OPEN_F, "open the blades", rot=CUT_ROT)
    arm.go(1.0, _cut_tcp(LINE_START), GRIP_SHUT_F, "close the blades", rot=CUT_ROT)
    x0, x1 = LINE_START, LINE_END - BLADE_LEN + 0.002
    arm.go(4.0, _cut_tcp(x1), GRIP_SHUT_F, "cut along the line", rot=CUT_ROT, linear=True)
    t_cut = arm.time
    arm.go(0.5, _cut_tcp(x1), GRIP_SHUT_F, "hold at the end", rot=CUT_ROT)
    arm.go(1.0, _cut_tcp(x1) + np.array([0, 0, 0.08]), GRIP_SHUT_F, "lift the scissors", rot=CUT_ROT)
    arm.go(2.2, rest + np.array([0, 0, 0.075]), GRIP_SHUT_F, "carry them back", rot=CUT_ROT)
    arm.go(1.4, rest, GRIP_SHUT_F, "down to the mat", rot=CUT_ROT)
    arm.go(0.7, rest, FINGER_OPEN, "let go", rot=CUT_ROT)
    arm.go(0.8, rest + np.array([0, 0, 0.06]), FINGER_OPEN, "back off", rot=CUT_ROT)
    arm.go(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])
    return arm, t_cut


def demo():
    """The left hand presses the sheet; the right takes the scissors, squeezes the blades shut and sweeps them down
    the line; the offcut falls into the tray; the scissors go back on their mat and both arms go home."""
    # the cut release is a little after the sweep; the left holds through it (t_release ~ 14 s)
    left0, t_press = left_plan(13.5)
    right, t_cut = right_plan(t_press, 13.5)
    left, t_press = left_plan(t_cut + 0.5)
    return {"left": left.wps, "right": right.wps}


class _Prog:
    def __init__(self):
        self.last_t = None

    def reset(self):
        self.last_t = None


_PROG = _Prog()
