"""On the station table: a real CAT6 cable with its jacket stripped and eight wires straightened in T568B order, a
real 8P8C RJ45 plug in a staging pocket, and a ratcheting crimper fixed to the bench.  The robot picks the cable,
aligns wire 1 (orange/white, red index band) to the plug's pin-1 notch and pushes all eight wires into the plug in
order, carries the wired plug to the crimper seat and seats it fully, then the free arm squeezes the ratcheting handle
with a 4-7 N grip until the contacts pierce the insulation and the strain-relief jaw grips the jacket, and finally
pull-tests the cable axially with 4-7 N for 1 s.  A flipped cable (wrong wire order), a plug not fully in the seat,
or a squeeze below 4 N (weak crimp) are scored caps.

The plug, cable and crimper handles are real, gravity-free rigid bodies in their own USD files.  While a gripper is
closed on a body's grasp point that body follows the gripper (a declared carry, the family's device); once the plug is
seated the wired assembly is locked in the crimper so the squeeze and the pull-test cannot drag it out.  The contacts,
jaw, ratchet, plug channels and wire colours are real geometry.  Force is declared from finger closure (4-7 N range),
not integrated.  The scorer reads the mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=rj45_crimp.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/rj45-crimp/rj45_crimp.py
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

TAG = "rj45"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- crimper fixture (local frame at the crimper base centre; world z = TOP + local z) -------------------------------
CRIMPER_XY = (0.250, -0.030)
CRIMPER_POS = (CRIMPER_XY[0], CRIMPER_XY[1], TOP)
SEAT_XY = (0.270, -0.030)             # plug centre XY when seated in the seat rail
SEAT_CZ = 0.018                       # plug centre z when seated (local)
HANDLE_PIVOT = np.array([0.210, -0.100, TOP + 0.020])   # world position of the handle hinge (handles body root)
GRIP_ZONE = np.array([0.190, -0.130, TOP + 0.050])      # centre of the handle-grip squeeze zone (within YAM reach)
HANDLE_ANGLE = math.radians(25.0)     # handle sweep from open to fully ratcheted closed

# -- staging pockets (left arm side) -------------------------------------------------------------------------------
PLUG_STAGE = (0.220, 0.060)
PLUG_STAGE_CZ = 0.005
CABLE_STAGE = (0.150, 0.060)
CABLE_STAGE_CZ = 0.004
PLUG_REL = np.array([0.006, 0.0, 0.001])   # plug centre relative to cable centre once the wires are inserted

# -- product geometry (real size, metres) --------------------------------------------------------------------------
PITCH = 0.00102               # 8P8C contact pitch (0.040 in, 1.02 mm)
PLUG_L, PLUG_W, PLUG_H = 0.0116, 0.01168, 0.0066
CABLE_OD = 0.0062

# -- tolerance / threshold geometry ---------------------------------------------------------------------------------
INSERT_TOL = 0.008            # cable must be within this of the insert pose for the wires to count as in
CABLE_INSERT = np.array([PLUG_STAGE[0] - PLUG_REL[0], PLUG_STAGE[1], TOP + CABLE_STAGE_CZ])
SEAT_CABLE = np.array([SEAT_XY[0] - PLUG_REL[0], SEAT_XY[1], TOP + SEAT_CZ - PLUG_REL[2]])
ORDER_TOL = math.radians(15.0)       # wire-1 index must be within this of pin 1 (no 180-deg folding: order matters)
SEAT_XY_TOL = 0.005
SEAT_Z_TOL = 0.005
SEAT_ROT_TOL = math.radians(12.0)
HANDLE_REACH = 0.045           # radius of the handle-grip squeeze zone (absorbs the arm's pose error)
SQUEEZE_ATTEMPT_OPEN = 0.060  # finger opening at/below which a squeeze attempt is detected
CRIMP_FORCE_MIN = 4.0          # N: 4-7 N declared squeeze pierces the contacts + grips the strain relief
CRIMP_DWELL = 0.35             # s: squeeze held at force before the ratchet locks
PULL_REACH = 0.018             # TCP within this of the cable grasp point counts as taking the cable
PULL_CLOSE = 0.030             # finger opening at/below which the cable is taken for the pull test
PULL_DIST = 0.012              # the arm must draw the cable grasp point back this far along -x (the cable axis)
PULL_MIN, PULL_MAX = 4.0, 7.0  # N: 4-7 N declared axial pull

# -- gripper / carry -----------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_CLOSE = 0.034            # opening at/below which a closed finger pair may take a part
GRIP_RELEASE = 0.045          # opening above which a held part is let go
GRIP_REACH = 0.016            # m: TCP within this of the part's grasp point
GRASP_OFF = {                 # grasp point in each body's local frame (body axis: +x wire/tip direction, +z up)
    "cable": np.array([-0.020, 0.0, 0.008]),   # on the jacket, 20 mm behind the stripped wires
    "plug": np.array([0.0, 0.0, 0.006]),       # on top of the plug body
}
OPEN_REF = 0.0939             # fully-open finger gap (m, two joints), used for the declared force mapping
FORCE_K = 86.0                # N/m: declared force = (OPEN_REF - finger_gap) * FORCE_K
HOLD_CABLE = -0.014           # finger target while carrying / pulling the cable (gap ~0.014 -> ~5.8 N)
HOLD_SQUEEZE = -0.012         # finger target while squeezing the handle (gap ~0.012 -> ~6.1 N)
HOME_SLACK = 0.020
PARTS = ("cable", "plug")
START_POS = {
    "cable": np.array([CABLE_STAGE[0], CABLE_STAGE[1], TOP + CABLE_STAGE_CZ]),
    "plug": np.array([PLUG_STAGE[0], PLUG_STAGE[1], TOP + PLUG_STAGE_CZ]),
}
START_QUAT = {p: np.array([0.0, 0.0, 0.0, 1.0]) for p in PARTS}


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


CRIMPER = UsdPart("crimper", asset="crimper", xy=CRIMPER_XY, extent=0.12, mass=None, fixed=True,
                  usd_path=str(ASSETS / "crimper.usda"), pos=CRIMPER_POS)
HANDLES = UsdPart("handles", asset="handles", xy=CRIMPER_XY, extent=0.10, mass=0.120,
                  usd_path=str(ASSETS / "handles.usda"), pos=tuple(HANDLE_PIVOT))
CABLE = UsdPart("cable", asset="cable", xy=CABLE_STAGE, extent=0.04, mass=0.010,
                usd_path=str(ASSETS / "cable.usda"), pos=tuple(START_POS["cable"]))
PLUG = UsdPart("plug", asset="plug", xy=PLUG_STAGE, extent=0.02, mass=0.002,
               usd_path=str(ASSETS / "plug.usda"), pos=tuple(START_POS["plug"]))
OBJECTS = [CRIMPER, HANDLES, CABLE, PLUG]
MOVERS = ()
SOLVER_ITERATIONS, MAX_DEPENETRATION = 64, 0.2


def _spawn(scene, spec):
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.fixed:
        addon["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)
    return scene.Object(name=spec.name, usd_path=spec.usd_path,
                        object_type=scene.ObjectType.RIGID,
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
    if getattr(scene.task_object, "_rj45_usd_parts", False):
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
            env_cfg.events.rj45_reset = EventTermCfg(func=_reset_parts, mode="reset",
                                                     params={"asset_cfg": SceneEntityCfg("crimper")})
            env_cfg.events.rj45_step = EventTermCfg(func=_rj45_step, mode="interval",
                                                    interval_range_s=(0.0, 0.0), is_global_time=False,
                                                    params={"asset_cfg": SceneEntityCfg("crimper")})
            return env_cfg

        return with_events

    task_object._rj45_usd_parts = True
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


def _rot(quat):
    return quat_xyzw_to_matrix(np.asarray(quat, float))


def _ang(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return math.acos(float(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0)))


def _declared_force(opening):
    return max(0.0, (OPEN_REF - float(opening)) * FORCE_K)


class _Mechanism:
    def __init__(self):
        self.pos = {p: START_POS[p].copy() for p in PARTS}
        self.quat = {p: START_QUAT[p].copy() for p in PARTS}
        self.held = {p: None for p in PARTS}
        self.offset = {p: np.zeros(3) for p in PARTS}
        self.rot_off = {p: np.eye(3) for p in PARTS}
        self.squeeze_t = 0.0
        self.weak_t = 0.0
        self.pull_start_x = None
        self.t = 0.0
        # latches
        self.wires_in = False
        self.seated = False
        self.locked = False
        self.crimped = False
        self.pulling = False
        # caps
        self.wrong_order = False
        self.not_seated = False
        self.weak_crimp = False

    def reset(self):
        self.__init__()


_MECH = _Mechanism()


def _write_part(env, name, pos, quat, env_ids):
    import torch

    part = env.scene[name]
    pose = torch.tensor([[pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]],
                        device=part.device, dtype=torch.float32)
    part.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    part.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=part.device,
                                                                     dtype=torch.float32), env_ids=env_ids)


def _reset_parts(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    for p in PARTS:
        _write_part(env, p, START_POS[p], START_QUAT[p], env_ids)
    _write_part(env, "handles", HANDLE_PIVOT, START_QUAT["cable"], env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: stripped CAT6 cable and 8P8C plug staged; crimper seat bare, handle open", flush=True)


def _in_allowed_zone(pos):
    x, y, z = pos
    if z < TOP - 0.01 or x < 0.10 or x > 0.40 or abs(y) > 0.18:
        return False
    return True


def _rj45_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)
    dt = float(base.step_dt)

    # 1. grasp / carry / release the cable and (before wiring) the plug
    for name in PARTS:
        if name == "plug" and m.wires_in:
            continue                      # once wired, the plug is slaved to the cable
        R = _rot(m.quat[name])
        gp = m.pos[name] + R @ GRASP_OFF[name]
        if m.held[name] is None:
            for side in ("left", "right"):
                g = grips[f"{side}_gripper"]
                if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - gp)) <= GRIP_REACH:
                    m.held[name] = side
                    m.offset[name] = g.rot.T @ (m.pos[name] - g.tcp)
                    m.rot_off[name] = g.rot.T @ R
                    break
        else:
            g = grips[f"{m.held[name]}_gripper"]
            if g.opening > GRIP_RELEASE:
                m.held[name] = None
                if not _in_allowed_zone(m.pos[name]):
                    m.wrong_order = True   # dropped off the bench: cannot be a good termination
            elif not m.locked:             # the seated assembly cannot be dragged out
                m.pos[name] = g.tcp + g.rot @ m.offset[name]
                m.quat[name] = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off[name])

    # the wired plug travels rigidly with the cable (fixed relative pose)
    if m.wires_in:
        m.pos["plug"] = m.pos["cable"] + PLUG_REL
        m.quat["plug"] = m.quat["cable"].copy()

    Rc = _rot(m.quat["cable"])
    Rp = _rot(m.quat["plug"])

    # 2. wire insertion: all eight wires into the plug, wire 1 on pin 1 (T568B order)
    at_insert = float(np.linalg.norm(m.pos["cable"] - CABLE_INSERT)) < INSERT_TOL
    wire_ax = _ang(Rc @ np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    index_ax = _ang(Rc @ np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]))
    order_ok = wire_ax <= ORDER_TOL and index_ax <= ORDER_TOL
    if at_insert and order_ok:
        m.wires_in = True
    elif at_insert and not order_ok:
        m.wrong_order = True

    # 3. seat the wired plug fully in the crimper seat rail
    plug_at_seat = (float(np.linalg.norm(m.pos["plug"][:2] - np.asarray(SEAT_XY))) < SEAT_XY_TOL
                    and abs(m.pos["plug"][2] - (TOP + SEAT_CZ)) < SEAT_Z_TOL)
    plug_square = (_ang(Rp @ np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])) <= SEAT_ROT_TOL
                   and _ang(Rp @ np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])) <= SEAT_ROT_TOL)
    if plug_at_seat and plug_square and m.wires_in:
        m.seated = True
        m.locked = True

    # 4. squeeze the ratcheting handle with the free (right) arm: 4-7 N declared from finger closure
    g = grips["right_gripper"]
    at_handle = float(np.linalg.norm(g.tcp - GRIP_ZONE)) < HANDLE_REACH
    squeeze_force = _declared_force(g.opening)
    attempting = at_handle and g.opening <= SQUEEZE_ATTEMPT_OPEN
    if attempting and not m.seated:
        m.not_seated = True
        m.squeeze_t = 0.0
        m.weak_t = 0.0
    elif attempting and m.seated:
        if squeeze_force < CRIMP_FORCE_MIN:
            m.weak_t += dt                 # a weak squeeze must be held, not just passed through while closing
            m.squeeze_t = 0.0
            if m.weak_t >= 0.60:
                m.weak_crimp = True
        else:
            m.weak_t = 0.0
            m.squeeze_t += dt
    else:
        m.squeeze_t = 0.0
        m.weak_t = 0.0
    if m.squeeze_t >= CRIMP_DWELL:
        m.crimped = True

    # animate the handle: it closes as the squeeze builds and stays shut once ratcheted
    frac = 1.0 if m.crimped else (min(m.squeeze_t / CRIMP_DWELL, 1.0) if m.seated else 0.0)
    hrot = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -HANDLE_ANGLE * frac)
    _write_part(env, "handles", HANDLE_PIVOT, ik.matrix_to_quat_xyzw(hrot), env_ids)

    # 5. pull-test: the left arm draws the cable back along its axis with a 4-7 N declared axial pull
    g = grips["left_gripper"]
    gp = m.pos["cable"] + Rc @ GRASP_OFF["cable"]
    pull_force = _declared_force(g.opening)
    if m.crimped and m.held["cable"] == "left" and m.pull_start_x is None:
        m.pull_start_x = float(g.tcp[0])          # remember where the cable was first taken after the crimp
    behind = (m.pull_start_x is not None
              and float(g.tcp[0]) < m.pull_start_x - PULL_DIST)
    near_cable = float(np.linalg.norm(g.tcp - gp)) < PULL_REACH + 0.030
    m.pulling = bool(
        m.held["cable"] == "left" and near_cable and behind and m.seated and m.crimped
        and PULL_MIN <= pull_force <= PULL_MAX
        and not (m.wrong_order or m.not_seated or m.weak_crimp)
    )

    # 6. write the posed bodies (the plug is slaved; nothing is dynamically carried)
    for p in PARTS:
        _write_part(env, p, m.pos[p].copy(), m.quat[p].copy(), env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        caps = "".join("  " + k.upper() for k in ("wrong_order", "not_seated", "weak_crimp") if getattr(m, k))
        print(f"[{TAG}] t={m.t:.2f}s cable {np.round(m.pos['cable'], 4)} plug {np.round(m.pos['plug'], 4)} "
              f"wire/in {int(m.wires_in)} seat {int(m.seated)} squeeze {m.squeeze_t:.2f}/{CRIMP_DWELL} "
              f"crimp {int(m.crimped)} pull {int(m.pulling)}"
              f"{caps or ''}", flush=True)
    m.t += dt


def extra_states(env, out, origin):
    m = _MECH
    for p in PARTS:
        out[p] = ObjectState(pos=m.pos[p].copy(), quat_xyzw=m.quat[p].copy(), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=m.pos[p].copy(), high=m.pos[p].copy())
    for key in ("wires_in", "seated", "locked", "crimped", "pulling",
                "wrong_order", "not_seated", "weak_crimp"):
        out[key] = getattr(m, key)
    out["held"] = dict(m.held)
    out["squeeze_t"] = m.squeeze_t


# -- scoring --------------------------------------------------------------------------------------------------------
STEPS = ("all eight wires inserted in order", "plug seated in crimper", "crimped + strain relief gripped",
         "pull-test holds 1 s")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("wires_in"))
    s1 = s0 and bool(objects.get("seated"))
    s2 = s1 and bool(objects.get("crimped"))
    s3 = s2 and bool(objects.get("pulling"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("wrong_order") or objects.get("not_seated") or objects.get("weak_crimp"))


def _full(objects) -> bool:
    return bool(all(steps_done(objects)) and not _caps(objects))


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
    done = steps_done(objects)
    pts = POINTS * sum(done[:3])                 # 0 / 0.25 / 0.5 / 0.75 for the first three steps
    if _caps(objects):
        pts = min(pts, 2 * POINTS)
    if done[3] and not _caps(objects):
        pts = max(pts, 3 * POINTS)                # pulling holds at 0.75 until the 1 s hold completes
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
_DOWN = yam_demo.grip_rotation(0.0)
_FINGER_OPEN = FINGER_OPEN


def _w(dur, obj_pos, off, finger, label, obj_rot=None):
    """Waypoint whose TCP holds the body at ``obj_pos`` with grasp offset ``off`` and the body at ``obj_rot``."""
    r = np.eye(3) if obj_rot is None else obj_rot
    tcp = np.asarray(obj_pos, float) + r @ np.asarray(off, float)
    return Waypoint(dur, tuple(tcp), finger, label, rot=r @ _DOWN)


def _plain(dur, tcp, finger, label, rot=None):
    return Waypoint(dur, tuple(tcp), finger, label, rot=_DOWN if rot is None else rot)


def demo():
    """Left arm wires the plug and seats it; right arm squeezes the ratchet; left arm pull-tests the cable."""
    left = []
    right = []
    coff = GRASP_OFF["cable"]
    cs = START_POS["cable"]
    chover = (cs[0], cs[1], cs[2] + 0.045)

    # A. pick the stripped cable from its staging pocket
    left += [_w(1.1, chover, coff, FINGER_OPEN, "over the cable"),
             _w(1.0, cs, coff, FINGER_OPEN, "down to the cable"),
             _w(0.4, cs, coff, FINGER_OPEN, "settle at the cable"),
             _w(0.5, cs, coff, HOLD_CABLE, "grip the cable"),
             _w(1.0, chover, coff, HOLD_CABLE, "lift the cable")]

    # B. carry it to the plug, align wire 1 with pin 1, push all eight wires fully in
    ins_hover = (CABLE_INSERT[0] - 0.030, CABLE_INSERT[1], CABLE_INSERT[2] + 0.040)
    left += [_w(1.2, ins_hover, coff, HOLD_CABLE, "carry cable over the plug"),
             _w(0.9, (CABLE_INSERT[0] - 0.012, CABLE_INSERT[1], CABLE_INSERT[2]), coff, HOLD_CABLE,
                "align wire 1 to pin 1"),
             _w(1.0, CABLE_INSERT, coff, HOLD_CABLE, "insert the eight wires"),
             _w(0.4, CABLE_INSERT, coff, HOLD_CABLE, "seat the wires fully")]

    # C. carry the wired plug to the crimper and seat it fully in the rail
    lift = (CABLE_INSERT[0] - 0.020, CABLE_INSERT[1], CABLE_INSERT[2] + 0.040)
    seat_hover = (SEAT_CABLE[0] - 0.035, SEAT_CABLE[1], SEAT_CABLE[2] + 0.045)
    left += [_w(0.9, lift, coff, HOLD_CABLE, "lift the wired plug clear"),
             _w(1.3, seat_hover, coff, HOLD_CABLE, "carry plug to the crimper"),
             _w(1.0, (SEAT_CABLE[0] - 0.012, SEAT_CABLE[1], SEAT_CABLE[2] + 0.010), coff, HOLD_CABLE,
                "align plug with the seat"),
             _w(0.9, SEAT_CABLE, coff, HOLD_CABLE, "seat the plug in the crimper"),
             _w(0.4, SEAT_CABLE, coff, HOLD_CABLE, "hold plug fully seated"),
             _w(0.4, SEAT_CABLE, coff, FINGER_OPEN, "release the cable"),
             _w(0.7, (SEAT_CABLE[0] - 0.020, SEAT_CABLE[1], SEAT_CABLE[2] + 0.030), coff, FINGER_OPEN,
                "lift clear of the crimper")]

    # D. right arm: approach the handle, squeeze with 4-7 N until the ratchet locks, release, go home
    gz = GRIP_ZONE
    gzh = gz + np.array([0.0, 0.0, 0.018])
    right += [_plain(12.0, tuple(_START_TCP["right"]), FINGER_OPEN, "wait for the plug to seat",
                     rot=_START_ROT["right"]),
              _plain(1.0, tuple(gzh), FINGER_OPEN, "approach the crimper handle"),
              _plain(0.9, tuple(gz), FINGER_OPEN, "down onto the handle grips"),
              _plain(0.7, tuple(gz), HOLD_SQUEEZE, "squeeze the ratchet"),
              _plain(1.0, tuple(gz), HOLD_SQUEEZE, "hold the crimp"),
              _plain(0.5, tuple(gzh), FINGER_OPEN, "release the handle"),
              _plain(1.2, tuple(_START_TCP["right"]), FINGER_OPEN, "right arm home", rot=_START_ROT["right"])]

    # E. left arm: take the cable at the jacket, draw it back along its axis with 4-7 N, hold 1 s
    gp = SEAT_CABLE + GRASP_OFF["cable"]
    back = gp - np.array([0.016, 0.0, 0.0])
    retreat = gp - np.array([0.024, 0.0, 0.0])
    left += [_w(0.8, back - GRASP_OFF["cable"], coff, FINGER_OPEN, "draw back off the plug"),
             _w(2.4, back - GRASP_OFF["cable"], coff, FINGER_OPEN, "wait for the ratchet to lock"),
             _w(0.7, gp - GRASP_OFF["cable"], coff, FINGER_OPEN, "reach the cable jacket"),
             _w(0.6, gp - GRASP_OFF["cable"], coff, HOLD_CABLE, "grip the cable jacket"),
             _plain(0.9, tuple(retreat), HOLD_CABLE, "pull-test the crimp"),
             _plain(2.6, tuple(retreat), HOLD_CABLE, "hold the pull-test")]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('wrong_order','not_seated','weak_crimp') if objects.get(k)) or '-'}")
