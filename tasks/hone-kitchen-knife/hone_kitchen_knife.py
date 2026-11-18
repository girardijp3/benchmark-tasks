"""Hone a real 200 mm chef's knife against a 300 mm honing steel in a bench clamp.

The robot takes the knife by its handle, sets the blade at the correct ~20 deg angle at the steel heel, sweeps
heel-to-tip along the steel with light pressure, alternates to the other side and sweeps again, parks the knife
on its wooden rest and holds still.  The knife is a real gravity-free rigid body (declared carry); the steel and
its holder are a welded, collider-free fixture.  Contact is a real geometric measurement of the ground edge
against the rod surface (declared light pressure), and the scorer reads angle, pressure, travel and park pose.

Run:  inspect-robots run --embodiment yam_arena -E task=hone_kitchen_knife.py --scorer task_score
Gate: cd local_sim && .venv/bin/python -m mjkit.run ../tasks/hone-kitchen-knife
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

TAG = "hk"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the steel: rod heel at this point, rod axis along world +X --------------------------------------------------------
STEEL_ORIGIN = (0.250, -0.150, TOP + 0.028)     # world z 0.7755
STEEL_R = 0.008                                   # 16 mm rod diameter (EST)
ROD_LEN = 0.200                                   # exposed rod length (EST of a 300 mm steel)
ROD_TOP = STEEL_ORIGIN[2] + STEEL_R              # 0.7835, the rod's upper surface
SWEEP_X0 = STEEL_ORIGIN[0]                        # blade at the steel heel (guard)
SWEEP_X1 = SWEEP_X0 + 0.180                       # blade at the steel tip
PROGRESS_OK = 0.170                               # 170 of the 180 mm sweep counts as full

# -- the knife: local +X heel->tip along the blade, +Z spine (edge at z=-0.028), Y = blade thickness --------------------
EDGE_PROBE = np.array([0.0, 0.0, -0.028])        # blade-heel edge point used for rod contact
GRASP_LOCAL = np.array([-0.065, 0.0, 0.0])        # handle centre, behind the blade heel
KNIFE_REST = np.array([0.330, -0.410, TOP + 0.040])
HOLD = -0.010                                     # gripper finger target, about 20 mm opening on the 22 mm handle

# -- angle and pressure --------------------------------------------------------------------------------------------------
HONE_ANGLE = math.radians(20.0)
ANGLE_WINDOW = math.radians(5.0)                 # 15-25 deg scores the sweep
ANGLE_CAP = math.radians(10.0)                   # outside 10-30 deg while on the rod caps at 0.5
PRESS_TARGET = 0.0015                            # 1.5 mm edge overlap: a light honing touch
PRESS_LIGHT_MAX = 0.0030                         # 0-3 mm overlap counts as light
PRESS_HEAVY = 0.0050                             # > 5 mm overlap caps at 0.5
CONTACT_GAP = 0.004                              # edge within 4 mm above the rod is 'near'
LATERAL_TOL = 0.012                              # edge probe within 12 mm of the rod's y

# -- grip -----------------------------------------------------------------------------------------------------------------
GRIP_CLOSE = 0.034
GRIP_RELEASE = 0.045
GRIP_REACH = 0.020
HOME_SLACK = 0.020
PAD_BACK = 0.025
GRIP_FORCE = 1.0
TRACE_S = 0.5


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


STEEL = UsdPart("steel", asset="steel", xy=STEEL_ORIGIN[:2], extent=0.30, mass=None, fixed=True,
                usd_path=str(ASSETS / "steel.usda"), pos=STEEL_ORIGIN)
REST = UsdPart("rest", asset="rest", xy=(0.330, -0.410), extent=0.25, mass=None, fixed=True,
               usd_path=str(ASSETS / "rest.usda"), pos=(0.330, -0.410, TOP))
KNIFE = UsdPart("knife", asset="knife", xy=KNIFE_REST[:2], extent=0.20, mass=0.205,
                usd_path=str(ASSETS / "knife.usda"), pos=tuple(KNIFE_REST))
OBJECTS = [STEEL, REST, KNIFE]
MOVEABLE = ("knife",)
SOLVER_ITERATIONS, MAX_DEPENETRATION = 64, 0.2


def _spawn(scene, spec):
    import isaaclab.sim as sim_utils
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.fixed:
        addon["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)
    return scene.Object(name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
                        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=(0, 0, 0, 1)),
                        spawn_cfg_addon=addon)


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
    if getattr(scene.task_object, "_hk_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        obj = _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)
        if isinstance(spec, UsdPart) and hasattr(obj, "disable_reset_pose"):
            obj.disable_reset_pose()
        return obj

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
        extra_states(env, out, origin)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.hk_reset = EventTermCfg(func=_reset_hone, mode="reset",
                                                   params={"asset_cfg": SceneEntityCfg("steel")})
            env_cfg.events.hk_step = EventTermCfg(func=_hone_step, mode="interval",
                                                  interval_range_s=(0.0, 0.0), is_global_time=False,
                                                  params={"asset_cfg": SceneEntityCfg("steel")})
            return env_cfg

        return with_events

    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback
    task_object._hk_usd_parts = True


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


def _rot(quat):
    return quat_xyzw_to_matrix(np.asarray(quat, float))


def _rz(a):
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), a)
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class _Mechanism:
    def __init__(self):
        self.pos = np.array(KNIFE_REST, float)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.held = False
        self.grasped_ever = False
        self.offset = np.zeros(3)
        self.rot_off = np.eye(3)
        self.angle_set = False
        self.prog_a = 0.0
        self.prog_b = 0.0
        self.sweep_a = False
        self.sweep_b = False
        self.clean = False
        self.parked = False
        self.wrong_angle = False
        self.heavy_press = False
        self.dropped = False
        self.angle = 0.0
        self.side = 1
        self.press = 0.0
        self.edge = np.zeros(3)
        self.t = 0.0
        self.report = 0.0

    def reset(self):
        self.__init__()


_MECH = _Mechanism()


def _write_knife(env, pos, quat, env_ids):
    import torch
    part = env.scene["knife"]
    pose = torch.tensor([[pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]],
                        device=part.device, dtype=torch.float32)
    part.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    part.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=part.device,
                                                                    dtype=torch.float32), env_ids=env_ids)


def _in_park(pos):
    return float(np.linalg.norm(pos - KNIFE_REST)) < 0.030


def _reset_hone(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_knife(env, _MECH.pos, _MECH.quat, env_ids)
    _reset_hold()
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: 200 mm chef's knife on the rest, 300 mm steel clamped at the heel", flush=True)


def _hone_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    g = _gripper_states(env)["right_gripper"]

    # 1. declared carry: close on the handle and the knife follows the gripper
    gp = m.pos + _rot(m.quat) @ GRASP_LOCAL
    if not m.held:
        if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - gp)) <= GRIP_REACH:
            m.held = True
            m.grasped_ever = True
            m.offset = g.rot.T @ (m.pos - g.tcp)
            m.rot_off = g.rot.T @ _rot(m.quat)
    else:
        if g.opening > GRIP_RELEASE:
            m.held = False
            if not _in_park(m.pos):
                m.dropped = True
        else:
            m.pos = g.tcp + g.rot @ m.offset
            m.quat = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off)

    # 2. physical measurements from the real knife pose
    R = _rot(m.quat)
    u = R @ np.array([1.0, 0.0, 0.0])
    m.angle = abs(math.atan2(float(u[1]), float(u[0])))
    m.side = 1 if u[1] >= 0 else -1
    m.edge = m.pos + R @ EDGE_PROBE
    ex, ey, ez = m.edge
    on_rod = (STEEL_ORIGIN[0] - 0.005 < ex < STEEL_ORIGIN[0] + ROD_LEN + 0.005
              and abs(ey - STEEL_ORIGIN[1]) < LATERAL_TOL)
    m.press = ROD_TOP - ez                       # > 0 means the edge is pressed into the rod surface
    near = on_rod and m.press > -CONTACT_GAP

    if near and abs(m.angle - HONE_ANGLE) > ANGLE_CAP:
        m.wrong_angle = True
    if near and m.press > PRESS_HEAVY:
        m.heavy_press = True

    # 3. scored milestones
    heel_x = STEEL_ORIGIN[0] + float(EDGE_PROBE[0] * math.cos(HONE_ANGLE))
    if (m.held and abs(m.angle - HONE_ANGLE) <= ANGLE_WINDOW and on_rod
            and abs(ex - heel_x) < 0.030 and abs(m.press) < 0.006):
        m.angle_set = True
    if m.held and near and abs(m.angle - HONE_ANGLE) <= ANGLE_WINDOW and m.press <= PRESS_LIGHT_MAX:
        progress = float(m.pos[0] - SWEEP_X0)
        if m.side == 1:
            m.prog_a = max(m.prog_a, progress)
        else:
            m.prog_b = max(m.prog_b, progress)
    m.sweep_a = bool(m.angle_set and m.prog_a >= PROGRESS_OK)
    m.sweep_b = bool(m.angle_set and m.prog_b >= PROGRESS_OK)
    m.clean = bool(m.sweep_a and m.sweep_b)
    m.parked = bool(m.grasped_ever and not m.held and _in_park(m.pos) and R[2, 2] > 0.90)

    _write_knife(env, m.pos, m.quat, env_ids)
    m.t += float(base.step_dt)
    if m.t - m.report >= TRACE_S:
        m.report = m.t
        print(f"[{TAG}] t={m.t:5.2f}s held {int(m.held)} angle {math.degrees(m.angle):5.1f} deg "
              f"side {'A' if m.side == 1 else 'B'} press {m.press*1000:5.2f} mm "
              f"edge ({ex:.3f},{ey:+.3f},{ez:.4f}) sweep "
              f"{m.prog_a*1000:5.0f}/{m.prog_b*1000:5.0f} mm steps "
              f"{int(m.angle_set)}{int(m.sweep_a)}{int(m.clean)}{int(m.parked)} "
              f"caps A{int(m.wrong_angle)}H{int(m.heavy_press)}D{int(m.dropped)}", flush=True)


def extra_states(env, out, origin):
    m = _MECH
    out["knife"] = ObjectState(pos=m.pos.copy(), quat_xyzw=m.quat.copy(), lin_vel=np.zeros(3),
                               ang_vel=np.zeros(3), low=m.pos.copy(), high=m.pos.copy())
    for k in ("angle_set", "sweep_a", "sweep_b", "clean", "parked", "grasped_ever", "wrong_angle", "heavy_press",
              "dropped", "held", "angle", "side", "press", "prog_a", "prog_b"):
        out[k] = getattr(m, k)
    out["edge"] = m.edge.copy()


# -- scoring -----------------------------------------------------------------------------------------------------------
STEPS = ("angle set", "heel-to-tip sweep", "both sides, clean edge", "knife parked")
POINTS = 0.25


def steps_done(objects) -> tuple:
    return (bool(objects.get("angle_set")), bool(objects.get("sweep_a")),
            bool(objects.get("clean")), bool(objects.get("parked")))


def _caps(objects) -> bool:
    return bool(objects.get("wrong_angle") or objects.get("heavy_press") or objects.get("dropped"))


def _pad_points(g):
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _home(objects, side) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / max(1e-9, np.linalg.norm(g.tips - g.back))
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


def _clear(objects) -> bool:
    """No pad touches the rod or the parked knife, and both grippers are open and unloaded."""
    x0, x1 = STEEL_ORIGIN[0], STEEL_ORIGIN[0] + ROD_LEN
    parked = KNIFE_REST + np.array([0.0, 0.0, 0.010])
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.opening < GRIP_RELEASE or g.contact >= GRIP_FORCE:
            return False
        for pt in _pad_points(g):
            dx = max(x0 - pt[0], 0.0, pt[0] - x1)
            on_rod = math.sqrt(dx * dx + (pt[1] - STEEL_ORIGIN[1]) ** 2
                               + (pt[2] - ROD_TOP) ** 2) < 0.045
            if on_rod or np.linalg.norm(pt - parked) < 0.060:
                return False
    return True


def _full(objects) -> bool:
    return bool(all(steps_done(objects)) and not _caps(objects) and _clear(objects)
                and _home(objects, "left") and _home(objects, "right"))


_HOLD = {"t": None}


def _reset_hold():
    _HOLD["t"] = None


def success(objects) -> bool:
    if not _full(objects):
        _reset_hold()
        return False
    if _HOLD["t"] is None:
        _HOLD["t"] = 0.0
        return False
    _HOLD["t"] += 0.05
    return _HOLD["t"] >= 1.0 - 1e-9


def score(objects) -> float:
    pts = POINTS * sum(steps_done(objects))
    if _caps(objects):
        pts = min(pts, 2 * POINTS)
    if _full(objects) and _HOLD["t"] is not None and _HOLD["t"] >= 1.0 - 1e-9:
        return 1.0
    return round(pts, 6)


# -- scripted demo -------------------------------------------------------------------------------------------------------
@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, float)
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
_OPEN = FINGER_OPEN
_DOWN = yam_demo.grip_rotation(0.0)     # gripper points down; fingers close across world y
_PICK_ROT = _DOWN


def _hone_target(side, x):
    """Return (knife origin, gripper TCP, gripper rot) for a light ~20 deg touch at rod position x."""
    th = side * HONE_ANGLE
    R = _rz(th)
    ep = R @ EDGE_PROBE
    oy = STEEL_ORIGIN[1] - float(ep[1])
    oz = ROD_TOP - float(EDGE_PROBE[2]) - PRESS_TARGET
    origin = np.array([x, oy, oz])
    tcp = origin - R @ np.array([-GRASP_LOCAL[0], 0.0, 0.0])
    return origin, tcp, _rz(th) @ _DOWN


def _w(dur, tcp, finger, label, rot):
    return Waypoint(dur, tuple(tcp), finger, label, rot=rot)


GRASP_REST = KNIFE_REST + GRASP_LOCAL
def demo():
    """One arm: take the knife, set 20 deg at the steel heel, sweep heel-to-tip on both sides with light
    pressure, park the knife on its rest and go home.  The left arm stays home.  Every approach is from above."""
    right = []
    # A. pick the knife up by its handle
    right += [
        _w(1.6, GRASP_REST + np.array([0, 0, 0.060]), _OPEN, "over the knife", _PICK_ROT),
        _w(1.2, GRASP_REST, _OPEN, "down to the handle", _PICK_ROT),
        _w(0.5, GRASP_REST, _OPEN, "settle on the handle", _PICK_ROT),
        _w(0.8, GRASP_REST, HOLD, "grip the handle", _PICK_ROT),
        _w(1.4, GRASP_REST + np.array([0, 0, 0.050]), HOLD, "lift the knife", _PICK_ROT),
    ]
    # B. set the correct ~20 deg angle at the steel heel, first side
    _, tcp_a0, rot_a = _hone_target(1, SWEEP_X0)
    right += [
        _w(1.8, tcp_a0 + np.array([0, 0, 0.050]), HOLD, "carry to the steel heel, side A", rot_a),
        _w(1.0, tcp_a0, HOLD, "set 20 deg at the heel, side A", rot_a),
        _w(0.6, tcp_a0, HOLD, "seat the edge on the rod, side A", rot_a),
    ]
    # C. heel-to-tip sweep, side A
    for i, x in enumerate(np.linspace(SWEEP_X0 + 0.045, SWEEP_X1, 4)):
        _, tcp, _ = _hone_target(1, float(x))
        right.append(_w(0.85, tcp, HOLD, f"sweep side A {i + 1}/4", rot_a))
    _, tcp_a1, _ = _hone_target(1, SWEEP_X1)
    right += [_w(0.9, tcp_a1 + np.array([0, 0, 0.055]), HOLD, "lift off the steel, side A", rot_a)]
    # D. alternate sides: present the other bevel at ~20 deg and sweep again
    _, tcp_b0, rot_b = _hone_target(-1, SWEEP_X0)
    right += [
        _w(1.8, tcp_b0 + np.array([0, 0, 0.060]), HOLD, "carry across to side B", rot_b),
        _w(1.0, tcp_b0, HOLD, "set 20 deg at the heel, side B", rot_b),
        _w(0.6, tcp_b0, HOLD, "seat the edge on the rod, side B", rot_b),
    ]
    for i, x in enumerate(np.linspace(SWEEP_X0 + 0.045, SWEEP_X1, 4)):
        _, tcp, _ = _hone_target(-1, float(x))
        right.append(_w(0.85, tcp, HOLD, f"sweep side B {i + 1}/4", rot_b))
    _, tcp_b1, _ = _hone_target(-1, SWEEP_X1)
    right += [_w(0.9, tcp_b1 + np.array([0, 0, 0.055]), HOLD, "lift off the steel, side B", rot_b)]
    # E. park the knife on its wooden rest and go home
    right += [
        _w(1.8, GRASP_REST + np.array([0, 0, 0.060]), HOLD, "carry the knife to its rest", _PICK_ROT),
        _w(1.2, GRASP_REST, HOLD, "lower the knife onto the rest", _PICK_ROT),
        _w(0.6, GRASP_REST, _OPEN, "park the knife", _PICK_ROT),
        _w(0.9, GRASP_REST + np.array([0, 0, 0.055]), _OPEN, "lift clear of the rest", _PICK_ROT),
        _w(2.2, tuple(_START_TCP["right"]), _OPEN, "home", _START_ROT["right"]),
    ]
    left = [_w(2.2, tuple(_START_TCP["left"]), _OPEN, "home", _START_ROT["left"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"angle {math.degrees(objects.get('angle', 0)):4.1f} deg press "
            f"{objects.get('press', 0) * 1000:4.1f} mm steps "
            f"{''.join('1' if x else '0' for x in steps_done(objects))} "
            f"caps {'/'.join(k for k in ('wrong_angle', 'heavy_press', 'dropped') if objects.get(k)) or '-'}")
