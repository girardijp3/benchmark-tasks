"""On the station table: a Hella 1058 / 1L6 002 395-131 178 mm H4 headlamp on a bench gimbal 3.00 m from an
aiming screen, with two real aim screws (8 mm hex socket) on the top rear of the housing. The lamp starts
low and left of the ECE cut-off; the beam screen carries the 36 mm target line and the +-15 mm / +-26 mm
aiming band. The robot grips the T-handle 8 mm hex key left in each aim screw with one hand and steadies
the lamp housing with the other, turns the vertical screw and then the horizontal screw, releases, and holds
the cut-off on the line for 1 s.

The lamp's pitch and yaw are real revolute joints on the gimbal; each aim screw has a visible head and a
coaxial shaft that follows it through a 0.9 N m torque fuse (the adjuster's strip limit). A shaft drives the
lamp's aim through a gear ratio, so turning the screw moves the reflector/housing. The cut-off on the screen
is the projection of the lamp's optical axis and its 0.69 deg down-angle from the actual lamp pose, not a
flag: a wrong-direction turn, a stripped screw, a pushed housing or a beam off the screen is measured.

Run it:  inspect-robots run --embodiment yam_arena -E task=headlamp_aim_adjustment.py --scorer task_score ...
"""

TAG = "headlamp"
try:
    import isaaclab
    _ON_ISAAC = getattr(isaaclab, "__file__", None) is not None
except ImportError:
    _ON_ISAAC = False
print(f"[{TAG}] on Isaac: {_ON_ISAAC}", flush=True)

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

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself times the 1.00 s hold
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
DEG = math.pi / 180.0

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) --------------------------
LAMP_Z = 0.860
LAMP_PIVOT = np.array([0.420, -0.130, LAMP_Z])
LAMP_DEPTH = 0.088
LENS_X = LAMP_PIVOT[0] + LAMP_DEPTH                 # 0.620
SCREEN_X = LENS_X + 3.000                           # 3.508: exactly 3.00 m from the lens
TARGET_DROP = 0.036                                 # ECE 0.69 deg / 1.2 % at 3.00 m
TARGET_H = LAMP_Z - TARGET_DROP                     # 0.824
BAND_V, BAND_H = 0.015, 0.026                       # +-15 mm vertical, +-26 mm horizontal (real aim limits)
SCREEN_W, SCREEN_H, SCREEN_Z = 1.600, 1.200, LAMP_Z
SCREEN_Y = -0.130
SCREW_X, SCREW_Z = 0.366, 0.900
SCREW_Y = {"V": -0.175, "H": -0.085}
STEADY_PAD = np.array([0.458, -0.0395, 0.860])      # the pad's outer face (local 0.038, 0.0885, 0 on the lamp)
STEADY_TCP = np.array([0.458, -0.0285, 0.860])

# -- the real adjuster geometry -------------------------------------------------------------------------------------
K_V = -1.20 * DEG / (2 * math.pi)                  # lamp pitch per radian of V screw turn (CCW raises)
K_H = 1.40 * DEG / (2 * math.pi)                   # lamp yaw per radian of H screw turn
PITCH0 = +0.55 * DEG                               # start: the cut-off is low (positive pitch is nose-down)
YAW0 = +0.72 * DEG                                 # start: the kink is left of the centreline
TARGET_PITCH = 0.0
TARGET_YAW = 0.0
ALPHA0 = math.atan(0.012)                           # the ECE cut-off is 1.2 % below the lamp axis
STRIP_TORQUE = 0.9                                 # N m: the adjuster's strip torque (the shaft drive's fuse)
STRIP_SLIP = 20.0 * DEG                              # rad (20 deg) of head-shaft slip means the socket stripped
TURN_ENGAGED = 15.0 * DEG                          # a screw turned this far has been engaged
WRONG_TURN = 8.0 * DEG                             # a wrong-direction turn past this latches the wrong_way fault

SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2


# -- objects ----------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class UsdPart(Asset):
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


RIG = UsdPart("rig", asset="rig", xy=(0.40, -0.13), extent=0.55, mass=2.4,
              usd_path=str(ASSETS / "headlamp_rig.usda"), pos=(0.0, 0.0, 0.0), articulation=True, fixed=True)
SCREEN = UsdPart("screen", asset="screen", xy=(SCREEN_X, SCREEN_Y), extent=SCREEN_W, mass=8.0,
                 usd_path=str(ASSETS / "beam_screen.usda"), pos=(SCREEN_X, SCREEN_Y, SCREEN_Z), kinematic=True)
PATCH = UsdPart("beam_patch", asset="beam_patch", xy=(SCREEN_X, SCREEN_Y), extent=1.3, mass=0.002,
                usd_path=str(ASSETS / "beam_patch.usda"), pos=(SCREEN_X - 0.004, SCREEN_Y, TARGET_H - 0.5),
                no_gravity=True)
OBJECTS = [RIG, SCREEN, PATCH]


def _spawn(scene, spec: UsdPart):
    import isaaclab.sim as sim_utils

    # Isaac: the fixed-root joint makes PhysX drop the articulation (the thermal-paste/headlamp nested-root bug),
    # so the fixed rig keeps a free root that extra_states pins to its spawn pose; gravity is off because the
    # stand carries the lamp.  MuJoCo keeps the fixed root and gravity, unchanged.
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(
        max_depenetration_velocity=MAX_DEPENETRATION,
        disable_gravity=True if (spec.no_gravity or (spec.fixed and _ON_ISAAC)) else None,
        kinematic_enabled=True if spec.kinematic else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=True if (spec.fixed and not _ON_ISAAC) else None)
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


RIG_JOINTS = ("AimPitch", "AimYaw", "ScrewHeadV", "ScrewHeadH", "ScrewV", "ScrewH")


def _joint_ids(rig):
    return rig.find_joints(list(RIG_JOINTS), preserve_order=True)[0]


def _rig_joints(rig):
    q = rig.data.joint_pos.torch[0, _joint_ids(rig)].detach().cpu().numpy().astype(np.float64)
    return dict(zip(RIG_JOINTS, (float(v) for v in q)))


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        obj = _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)
        # The task writes the rig root and the beam patch itself; Arena's own pose reset writes a float64
        # root pose that PhysX rejects (the transformf kernel), so drop those two reset events.
        if spec.name in ("rig", "beam_patch", "screen") and hasattr(obj, "disable_reset_pose"):
            obj.disable_reset_pose()
        return obj

    def object_states(env, names):
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            data = base.scene[name].data
            if name == "rig":
                i = list(base.scene[name].body_names).index("AimPitch")
                pose = _arr(data.body_link_pose_w)[i]
                lin = _arr(data.body_link_vel_w)[i, :3]
                ang = _arr(data.body_link_vel_w)[i, 3:]
            else:
                pose = _arr(data.root_link_pose_w)
                lin, ang = _arr(data.root_lin_vel_w), _arr(data.root_ang_vel_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=lin, ang_vel=ang, low=pos.copy(),
                                    high=pos.copy())
        out.update(_gripper_states(env))
        extra_states(base, out, origin)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.headlamp_reset = EventTermCfg(func=_reset_headlamp, mode="reset",
                                                         params={"asset_cfg": SceneEntityCfg("rig")})
            env_cfg.events.headlamp_step = EventTermCfg(func=_aim_step, mode="interval", interval_range_s=(0.0, 0.0),
                                                        is_global_time=False,
                                                        params={"asset_cfg": SceneEntityCfg("rig")})
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


# -- writing targets and the mechanism ---------------------------------------------------------------------------------
def _write_targets(rig, values: dict) -> None:
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
        t[:, names.index(name)] = float(value)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, rig._ALL_INDICES)


def _set_joints(rig, values: dict, velocity=True) -> None:
    import torch

    ids = _joint_ids(rig)
    order = list(RIG_JOINTS)
    q = torch.tensor([[float(values.get(n, 0.0)) for n in order]], device=rig.device, dtype=torch.float32)
    rig.write_joint_position_to_sim_index(position=q, joint_ids=ids, env_ids=rig._ALL_INDICES)
    if velocity:
        rig.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids, env_ids=rig._ALL_INDICES)


def _write_patch_pose(env, u: float, h: float):
    import torch

    patch = env.scene["beam_patch"]
    rp = np.array([[SCREEN_X - 0.004, u, h - 0.5, 0.0, 0.0, 0.0, 1.0]], dtype=float)
    if _ON_ISAAC:
        rp = torch.tensor(rp, dtype=torch.float32, device=patch.device)
        ids = torch.tensor([0], dtype=torch.long, device=patch.device)
        patch.write_root_pose_to_sim_index(root_pose=rp, env_ids=ids)
    else:
        patch.write_root_pose_to_sim_index(root_pose=rp)


def _pin_rig(env) -> None:
    """Isaac: hold the free-root rig at its spawn pose (the fixed-root joint is unavailable; see _spawn)."""
    if not _ON_ISAAC:
        return
    import torch

    origin = env.scene.env_origins[0].detach().cpu().numpy()
    rig = env.scene["rig"]
    pose = torch.tensor([[origin[0] + RIG.pos[0], origin[1] + RIG.pos[1], origin[2] + RIG.pos[2],
                          0.0, 0.0, 0.0, 1.0]], device=rig.device, dtype=torch.float32)
    rig.write_root_pose_to_sim_index(root_pose=pose, env_ids=None)
    rig.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=rig.device), env_ids=None)


def _reset_headlamp(env, env_ids, asset_cfg=None):
    if env_ids is None:
        return
    rig = env.scene["rig"]
    _set_joints(rig, {"AimPitch": PITCH0, "AimYaw": YAW0, "ScrewHeadV": 0.0, "ScrewHeadH": 0.0,
                      "ScrewV": 0.0, "ScrewH": 0.0})
    _write_targets(rig, {"AimPitch": PITCH0, "AimYaw": YAW0, "ScrewV": 0.0, "ScrewH": 0.0})
    _pin_rig(env)
    _MECH.reset()
    _write_patch_pose(env, SCREEN_Y, TARGET_H)



def _aim_pose(base):
    rig = base.scene["rig"]
    i = list(rig.body_names).index("AimPitch")
    pose = _arr(rig.data.body_link_pose_w)[i]
    pos, quat = pose[:3], pose[3:]
    R = quat_xyzw_to_matrix(quat)
    lens = pos + R @ np.array([LAMP_DEPTH, 0.0, 0.0])
    return lens, R


def beam_geometry(base):
    """The cut-off on the beam screen from the lamp's actual pose: the center ray of the horizontal cut-off
    (1.2 % below the lamp axis) projected onto the x=SCREEN_X plane. Returns (u, h, on_screen, axis)."""
    lens, R = _aim_pose(base)
    axis = R @ np.array([1.0, 0.0, 0.0])
    up = R @ np.array([0.0, 0.0, 1.0])
    d = axis * math.cos(ALPHA0) - up * math.sin(ALPHA0)
    if d[0] <= 1e-6:
        return 0.0, 0.0, False, axis
    t = (SCREEN_X - lens[0]) / d[0]
    p = lens + t * d
    u, h = float(p[1]), float(p[2])
    # the horizontal part of the cut-off (to +y) must be on the screen too
    yl = R @ np.array([0.0, 1.0, 0.0])
    on = (abs(u - SCREEN_Y) < SCREEN_W / 2 - 0.05 and SCREEN_Z - SCREEN_H / 2 + 0.03 < h < SCREEN_Z + SCREEN_H / 2 - 0.03
          and float(np.dot(yl, np.array([0.0, 1.0, 0.0]))) > 0.3)
    return u, h, bool(on), axis


@dataclass
class _Mech:
    time: float = 0.0
    v_turn: float = 0.0
    h_turn: float = 0.0
    v_engaged: bool = False
    h_engaged: bool = False
    wrong_way: bool = False
    stripped: bool = False
    forced: bool = False
    unsteady: bool = False
    steady_contact: float = 0.0
    last_report: float = -10.0
    last_u: float = 0.0
    last_h: float = 0.0

    def reset(self):
        self.__init__()


_MECH = _Mech()


def _aim_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    rig = env.scene["rig"]
    st = _rig_joints(rig)
    _MECH.time = float(base.episode_length_buf[0]) * float(base.step_dt)
    # the lamp's aim follows its shaft through the gearbox; each shaft follows its head through the 0.9 N m fuse
    _write_targets(rig, {
        "AimPitch": PITCH0 + K_V * st["ScrewV"],
        "AimYaw": YAW0 + K_H * st["ScrewH"],
        "ScrewV": st["ScrewHeadV"],
        "ScrewH": st["ScrewHeadH"],
    })
    _MECH.v_turn = st["ScrewV"]
    _MECH.h_turn = st["ScrewH"]
    if abs(st["ScrewV"]) > TURN_ENGAGED:
        _MECH.v_engaged = True
    if abs(st["ScrewH"]) > TURN_ENGAGED:
        _MECH.h_engaged = True
    # a wrong-direction turn: V above its start, H below its start (these move the beam away from the target)
    if st["ScrewV"] < -WRONG_TURN or st["ScrewH"] > WRONG_TURN:
        _MECH.wrong_way = True
    # the head turning while the shaft does not = the hex socket has stripped (the 0.9 N m fuse slipped)
    if abs(st["ScrewHeadV"] - st["ScrewV"]) > STRIP_SLIP or abs(st["ScrewHeadH"] - st["ScrewH"]) > STRIP_SLIP:
        _MECH.stripped = True
    # steadying: the left hand must be on the pad while a screw is actually being turned. The pad is a real
    # contact (its rubber face is at STEADY_PAD; the closed finger tips press 2 mm into it); the screw rates
    # come from the rig's joint velocities, so releasing after the turns is not a fault.
    g = _gripper_states(base)
    left = g["left_gripper"]
    on = bool(float(np.linalg.norm(left.tips - STEADY_PAD)) < 0.005 and left.opening < 0.010)
    _MECH.steady_contact = 1.0 if on else 0.0
    qv = rig.data.joint_vel.torch[0, _joint_ids(rig)].detach().cpu().numpy().astype(np.float64)
    vels = dict(zip(RIG_JOINTS, (float(v) for v in qv)))
    turning = max(abs(vels["ScrewV"]), abs(vels["ScrewH"])) > 0.02
    if turning and not on:
        _MECH.unsteady = True
    # a forced cut-off: the lamp pose has moved away from what the screw target asks for
    if abs(float(st["AimPitch"]) - (PITCH0 + K_V * st["ScrewV"])) > 1.50 * DEG or \
       abs(float(st["AimYaw"]) - (YAW0 + K_H * st["ScrewH"])) > 1.50 * DEG:
        _MECH.forced = True
    u, h, on, axis = beam_geometry(base)
    _MECH.last_u, _MECH.last_h = u, h
    _write_patch_pose(env, u, h)
    if _MECH.time - _MECH.last_report > 0.5:
        _MECH.last_report = _MECH.time
        print(f"[{TAG}] t={_MECH.time:6.2f}s  V {math.degrees(st['ScrewV']):+7.2f} deg (head "
              f"{math.degrees(st['ScrewHeadV']):+7.2f})  H {math.degrees(st['ScrewH']):+7.2f} deg (head "
              f"{math.degrees(st['ScrewHeadH']):+7.2f})  cut u {u * 1000:+6.1f} h {h * 1000:7.1f} mm  "
              f"steady {_MECH.steady_contact:4.1f} N  strip {_MECH.stripped} wrong {_MECH.wrong_way}", flush=True)


def extra_states(base, out, origin):
    _pin_rig(base)
    rig = base.scene["rig"]
    st = _rig_joints(rig)
    u, h, on, axis = beam_geometry(base)
    out["v_turn"] = st["ScrewV"]
    out["h_turn"] = st["ScrewH"]
    out["v_head"] = st["ScrewHeadV"]
    out["h_head"] = st["ScrewHeadH"]
    out["aim_pitch"] = st["AimPitch"]
    out["aim_yaw"] = st["AimYaw"]
    out["cut_u"] = u
    out["cut_h"] = h
    out["v_err"] = (h - TARGET_H) * 1000.0
    out["h_err"] = (u - SCREEN_Y) * 1000.0
    out["on_screen"] = bool(on)
    out["beam_on"] = bool(on)  # the 12 V supply is on; an off-screen beam is not a usable beam
    out["steady_contact"] = _MECH.steady_contact
    out["v_engaged"] = bool(_MECH.v_engaged)
    out["h_engaged"] = bool(_MECH.h_engaged)
    out["wrong_way"] = bool(_MECH.wrong_way)
    out["stripped"] = bool(_MECH.stripped)
    out["forced"] = bool(_MECH.forced)
    out["unsteady"] = bool(_MECH.unsteady)
    rig_i = list(rig.body_names).index("AimPitch")
    out["lamp_speed"] = float(np.linalg.norm(_arr(rig.data.body_link_vel_w)[rig_i]))
    out["lamp_rate"] = float(np.linalg.norm(_arr(rig.data.body_link_vel_w)[rig_i, 3:]))


# -- scoring ----------------------------------------------------------------------------------------------------------
CLEAR = 0.05
STILL_S = 1.0
LAMP_BOX = (np.array([0.28, -0.24, 0.76]), np.array([0.56, 0.02, 0.98]))  # the lamp, screws and stand head


def clear(objects) -> bool:
    """Both grippers clear of the lamp, stand head, screws and screen; no grip force."""
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        for p in g.back + np.linspace(0.0, 1.0, 11)[:, None] * (g.tips - g.back):
            if np.all(p >= LAMP_BOX[0] - CLEAR) and np.all(p <= LAMP_BOX[1] + CLEAR):
                return False
    return True


def v_in_band(objects) -> bool:
    return bool(abs(float(objects["v_err"])) <= BAND_V * 1000.0 and objects["on_screen"])


def h_in_band(objects) -> bool:
    return bool(abs(float(objects["h_err"])) <= BAND_H * 1000.0)


def _fault(objects) -> bool:
    return bool(objects["wrong_way"] or objects["stripped"] or objects["forced"] or objects["unsteady"]
                or not objects["on_screen"])


class _Hold:
    def __init__(self):
        self.t0 = None
        self.u = None
        self.h = None

    def update(self, objects, ok: bool) -> bool:
        if not ok:
            self.t0 = None
            return False
        u, h = float(objects["cut_u"]), float(objects["cut_h"])
        if self.t0 is None:
            self.t0, self.u, self.h = float(objects.get("episode_time", 0.0)), u, h
            return False
        if abs(u - self.u) > 0.002 or abs(h - self.h) > 0.002:
            self.t0, self.u, self.h = float(objects.get("episode_time", 0.0)), u, h
            return False
        return float(objects.get("episode_time", 0.0)) - self.t0 >= STILL_S - 1e-6


_HOLD = _Hold()


def steps_done(objects):
    return [bool(objects.get("v_engaged")), v_in_band(objects), bool(v_in_band(objects) and h_in_band(objects)),
            success(objects)]


def success(objects) -> bool:
    ok = bool(v_in_band(objects) and h_in_band(objects) and clear(objects) and not _fault(objects)
              and float(objects.get("lamp_speed", 1.0)) < 0.010 and float(objects.get("lamp_rate", 1.0)) < 0.05)
    return _HOLD.update(objects, ok)


def score(objects) -> float:
    if objects["stripped"] or objects["wrong_way"] or objects["forced"] or objects["unsteady"] \
            or not objects["on_screen"]:
        return 0.25 if objects.get("v_engaged") else 0.0
    if success(objects):
        return 1.0
    if v_in_band(objects) and h_in_band(objects):
        return 0.75
    if v_in_band(objects):
        return 0.5
    if objects.get("v_engaged"):
        return 0.25
    return 0.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
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
_DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
_Y, _Z = np.eye(3)[1], np.eye(3)[2]


def side_grip(approach: float, roll: float = 0.0) -> np.ndarray:
    forward = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN
    return axis_angle_matrix(_Z, approach) @ axis_angle_matrix(np.eye(3)[0], roll) @ forward


def _grip_rot(roll: float = 0.0) -> np.ndarray:
    return yam_demo.grip_rotation(roll)


TUCK = np.array([0.05, 0.55, 0.85])
DRIVER_HOLD = -0.010
DRIVER_OPEN = FINGER_OPEN


class Arm:
    def __init__(self, side):
        self.side, self.wps = side, []

    def go(self, dur, pos, finger, label, rot=None):
        self.wps.append(Waypoint(dur, tuple(np.asarray(pos, float)), finger, label, rot=rot))

    @property
    def time(self):
        return sum(w.duration for w in self.wps)

    def wait_until(self, t, label="wait"):
        if t > self.time:
            last = self.wps[-1] if self.wps else Waypoint(0.01, tuple(_START_TCP[self.side]), FINGER_OPEN,
                                                          "wait", rot=_START_ROT[self.side])
            self.wps.append(Waypoint(t - self.time, last.pos, last.finger, label, rot=last.rot))

    def home(self, dur=3.0):
        self.wps.append(Waypoint(dur, tuple(_START_TCP[self.side]), FINGER_OPEN, "home",
                                 rot=_START_ROT[self.side]))


TOOL0 = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])  # +Z = +X (forward), T-bar along +Z
STEADY_ROT = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.pi) @ np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


def tool_rot(roll_deg: float) -> np.ndarray:
    """The gripper holding the T-handle key, turned ``roll_deg`` about the key's +X shaft."""
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(roll_deg)) @ TOOL0


def demo():
    """Left hand braces the housing; right hand grips the T-handle key, turns the V screw +165 deg and the
    H screw -180 deg (one stroke each), lets go, and both arms clear.  The beam is then held on the ECE
    line for 1 s."""
    left = Arm("left")
    left.go(1.8, (0.458, 0.10, 0.86), FINGER_OPEN, "to the steady pad", rot=STEADY_ROT)
    left.go(1.2, STEADY_TCP, 0.0, "brace the housing", rot=STEADY_ROT)
    left.wait_until(20.0, "hold the housing steady")
    left.go(0.8, (0.458, 0.10, 0.88), FINGER_OPEN, "let go", rot=STEADY_ROT)
    left.home(3.0)
    right = Arm("right")
    right.go(3.0, tuple(_START_TCP["right"]), FINGER_OPEN, "wait for the brace", rot=_START_ROT["right"])
    right.go(1.2, (0.260, -0.175, 0.900), FINGER_OPEN, "behind the V key", rot=tool_rot(0))
    right.go(0.8, (0.331, -0.175, 0.900), FINGER_OPEN, "around the V key", rot=tool_rot(0))
    right.go(0.7, (0.331, -0.175, 0.900), DRIVER_HOLD, "grip the V key", rot=tool_rot(0))
    right.go(2.6, (0.331, -0.175, 0.900), DRIVER_HOLD, "turn V +165 deg", rot=tool_rot(165))
    right.go(0.7, (0.331, -0.175, 0.900), FINGER_OPEN, "let go of the V key", rot=tool_rot(165))
    right.go(0.7, (0.260, -0.175, 0.900), FINGER_OPEN, "clear the V key", rot=tool_rot(165))
    right.go(1.1, (0.260, -0.085, 0.900), FINGER_OPEN, "behind the H key", rot=tool_rot(180))
    right.go(0.8, (0.331, -0.085, 0.900), FINGER_OPEN, "around the H key", rot=tool_rot(180))
    right.go(0.7, (0.331, -0.085, 0.900), DRIVER_HOLD, "grip the H key", rot=tool_rot(180))
    right.go(2.9, (0.331, -0.085, 0.900), DRIVER_HOLD, "turn H -180 deg", rot=tool_rot(0))
    right.go(0.7, (0.331, -0.085, 0.900), FINGER_OPEN, "let go of the H key", rot=tool_rot(0))
    right.go(0.7, (0.260, -0.085, 0.900), FINGER_OPEN, "clear the H key", rot=tool_rot(0))
    right.go(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])
    if right.time > left.time:
        left.wait_until(right.time, "wait for the right arm")
    return {"left": left.wps, "right": right.wps}
