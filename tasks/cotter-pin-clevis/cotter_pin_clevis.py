"""On the station table: a real 3 mm steel split (cotter) pin staged in a white holder beside an 8 mm clevis joint.
The joint's upright clevis pin carries a 3.2 mm cross-hole.  The robot must lift the cotter, align its axis with the
cross-hole, push it fully through with a light 4-7 N push, bend its two legs apart to opposite sides so it cannot back
out, then pull the joint for 1 s to prove it holds.  A cotter that is not through, legs not spread, or a cotter dropped
off the bench are scored caps.

The cotter is three real gravity-free rigid bodies (eye+shank, long leg, short leg) in their own USD files.  While the
gripper is closed on the eye/shank grasp point the cotter follows the gripper (a declared carry, like the family's
detents); the legs bend about their roots as a closed finger sweeps them outward (declared geometry, the way soft
steel stays bent).  The scorer reads the mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=cotter_pin_clevis.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/cotter-pin-clevis/cotter_pin_clevis.py
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

TAG = "cpc"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- clevis joint fixture (local frame: clevis-pin axis XY, table top z 0; world z = TOP + local z) ----------------------
PIVOT_XY = (0.250, -0.055)
PIN_R = 0.005                          # 8 mm-class clevis pin (10 mm shank in this fixture)
HOLE_Z_LOCAL = 0.055                   # cross-hole centre height above the table
HOLE_Z = TOP + HOLE_Z_LOCAL
INSERT_X = PIVOT_XY[0] - PIN_R - 0.0005  # cotter eye seat face just outside the near face of the pin
HOLE_CENTER = np.array([INSERT_X, PIVOT_XY[1], HOLE_Z])

# -- cotter (real 3 mm nominal ISO 1234 split pin) ------------------------------------------------------------------------
WIRE_R = 0.0014                        # 2.8 mm wire for a 3 mm nominal pin (ISO 1234 wire ~0.9 d)
SHANK = 0.012                          # straight part from the eye's seat face to the legs' roots
LEG_GAP = 0.0006                       # gap between the two legs at the root
LONG = 0.036                           # long (extended-prong) leg past the shank
SHORT = 0.028                          # short leg past the shank
LEG_ROOT_LONG = np.array([SHANK, LEG_GAP / 2, 0.0])    # long leg, +y side, bends toward +y
LEG_ROOT_SHORT = np.array([SHANK, -LEG_GAP / 2, 0.0])  # short leg, -y side, bends toward -y
LEGS = {"long": (LEG_ROOT_LONG, LONG, +1.0), "short": (LEG_ROOT_SHORT, SHORT, -1.0)}

# -- staging -------------------------------------------------------------------------------------------------------------
STAGE_XY = (0.250, 0.045)
STAGE_CZ = 0.010                       # cotter centre above the table in the white holder

# -- pull tab -------------------------------------------------------------------------------------------------------------
TAB_X = PIVOT_XY[0] + 0.058
PULL_TAB = np.array([TAB_X, PIVOT_XY[1], HOLE_Z])
PULL_DIST = 0.008                      # outward (-x) pull that tests the joint
PULL_CMD = 0.012                       # demo commands a little further than the gate, so arm sag still clears it

# -- mechanism windows ----------------------------------------------------------------------------------------------------
AXIS_TOL = math.radians(10.0)          # cotter wire axis within 10 deg of the cross-hole axis (+x)
XY_TOL = 0.006                         # cotter centre within 6 mm of the hole centre line in x/y
Z_TOL = 0.006                          # ... and 6 mm in z; the insertion is a geometry gate
ABOVE_HOLE = 0.005                     # centre this far above the hole counts as hovering over it
SPREAD_DEG = 55.0                      # a bent leg's outward angle that counts as spread
MAX_BEND_RAD = math.radians(70.0)
LEG_PUSH_REACH = 0.010                 # a closed finger within this of a leg tip bends that leg
SPREAD_RAD = math.radians(SPREAD_DEG)
PULL_REACH = 0.012                     # a closed finger within this of the pull tab can pull it

# -- gripper / grasp ------------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0              # N: a real closed grip
GRIP_CLOSE = 0.034            # opening at/below which a closed finger pair may take the cotter
GRIP_RELEASE = 0.045          # opening above which a held cotter is let go
GRIP_REACH = 0.016            # m: TCP within this of the cotter's grasp point
GRIP_CLEAR = 0.045            # m: no pad within this of the joint at the scored state
HOME_SLACK = 0.020
GRASP_UP = 0.012              # the TCP sits this far above the cotter centre, along cotter local z
HOLD = -0.013                 # finger target while carrying the cotter

START_POS = np.array([STAGE_XY[0], STAGE_XY[1], TOP + STAGE_CZ])
START_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


CLEVIS = UsdPart("clevis", asset="clevis", xy=PIVOT_XY, extent=0.11, mass=None, fixed=True,
                 usd_path=str(ASSETS / "clevis.usda"), pos=(PIVOT_XY[0], PIVOT_XY[1], TOP))
COTTER = UsdPart("cotter", asset="cotter", xy=STAGE_XY, extent=0.06, mass=0.0008,
                 usd_path=str(ASSETS / "cotter.usda"), pos=tuple(START_POS))
LEG_LONG = UsdPart("leg_long", asset="leg_long", xy=STAGE_XY, extent=0.04, mass=0.0004,
                   usd_path=str(ASSETS / "leg_long.usda"), pos=tuple(START_POS))
LEG_SHORT = UsdPart("leg_short", asset="leg_short", xy=STAGE_XY, extent=0.03, mass=0.0004,
                    usd_path=str(ASSETS / "leg_short.usda"), pos=tuple(START_POS))
OBJECTS = [CLEVIS, COTTER, LEG_LONG, LEG_SHORT]
PART_NAMES = ("cotter", "leg_long", "leg_short")
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
    if getattr(scene.task_object, "_cpc_usd_parts", False):
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
            env_cfg.events.cpc_reset = EventTermCfg(func=_reset_cotter, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("clevis")})
            env_cfg.events.cpc_step = EventTermCfg(func=_cotter_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("clevis")})
            return env_cfg

        return with_events

    task_object._cpc_usd_parts = True
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


class _Mechanism:
    def __init__(self):
        self.pos = START_POS.copy()
        self.quat = START_QUAT.copy()
        self.held = None
        self.offset = np.zeros(3)
        self.rot_off = np.eye(3)
        self.bend = {leg: 0.0 for leg in LEGS}
        # progress latches
        self.aligned = False
        self.through = False
        self.spread = False
        self.pull_ok = False
        # caps
        self.not_through = False
        self.not_spread = False
        self.dropped = False
        # pull gesture tracking
        self.pull_x0 = {side: None for side in ("left", "right")}
        self.pull_applied = False
        # current state
        self.seated = False
        self.t = 0.0

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


def _reset_cotter(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_part(env, "cotter", START_POS, START_QUAT, env_ids)
    for leg in LEGS:
        _write_part(env, f"leg_{leg}", START_POS, START_QUAT, env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: 3 mm split pin staged in holder; clevis cross-hole bare, both arms READY", flush=True)


def _grasp_point():
    return _MECH.pos + _rot(_MECH.quat) @ np.array([0.0, 0.0, GRASP_UP])


def _in_allowed_zone(pos):
    x, y, z = pos
    if z < TOP - 0.01 or x < 0.10 or x > 0.40 or abs(y) > 0.18:
        return False
    return True


def _leg_tip(pos, R, leg, bend):
    root, length, sign = LEGS[leg]
    root_w = pos + R @ root
    Rz = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), sign * bend)
    return root_w + R @ Rz @ np.array([length, 0.0, 0.0]), root_w


def _bend_from_finger(pos, R, leg, bend, tcp):
    """A closed finger at a leg's tip sweeps the leg outward; the bend latches (soft steel stays bent)."""
    tip_w, root_w = _leg_tip(pos, R, leg, bend)
    sign = LEGS[leg][2]
    if float(np.linalg.norm(np.asarray(tcp, float) - tip_w)) > LEG_PUSH_REACH:
        return bend
    v = R.T @ (np.asarray(tcp, float) - root_w)
    if v[0] <= 1e-6:
        return bend
    ang = math.atan2(sign * v[1], v[0])
    return max(bend, min(max(ang, 0.0), MAX_BEND_RAD))


def _cotter_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)

    # 1. grasp / carry / release the cotter body (the legs follow its pose at their current bend)
    gp = _grasp_point()
    if m.held is None:
        for side in ("left", "right"):
            g = grips[f"{side}_gripper"]
            if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - gp)) <= GRIP_REACH:
                m.held = side
                m.offset = g.rot.T @ (m.pos - g.tcp)
                m.rot_off = g.rot.T @ _rot(m.quat)
                break
    else:
        g = grips[f"{m.held}_gripper"]
        if g.opening > GRIP_RELEASE:
            m.held = None
            if not _in_allowed_zone(m.pos):
                m.dropped = True
        else:
            m.pos = g.tcp + g.rot @ m.offset
            m.quat = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off)

    # 2. static geometry of the cotter at the cross-hole
    R = _rot(m.quat)
    axis_err = math.acos(float(np.clip(R[:, 0] @ np.array([1.0, 0.0, 0.0]), -1.0, 1.0)))
    on_axis = axis_err <= AXIS_TOL
    p = m.pos
    at_xy = float(np.linalg.norm(p[:2] - HOLE_CENTER[:2])) <= XY_TOL
    at_z = abs(p[2] - HOLE_Z) <= Z_TOL
    above = p[2] > HOLE_Z + ABOVE_HOLE
    m.seated = bool(at_xy and at_z and on_axis and m.through)

    # 3. progress latches
    if m.held is not None and at_xy and above and on_axis:
        m.aligned = True
    if at_xy and at_z and on_axis:
        m.through = True

    # 4. bend each leg: a closed finger at the leg tip sweeps it outward; soft steel stays bent (latched)
    for leg in LEGS:
        for side in ("left", "right"):
            g = grips[f"{side}_gripper"]
            if g.opening <= GRIP_CLOSE:
                m.bend[leg] = _bend_from_finger(m.pos, R, leg, m.bend[leg], g.tcp)
    if all(m.bend[leg] >= SPREAD_RAD - 1e-6 for leg in LEGS):
        m.spread = True

    # 5. pull test: a closed finger grips the tab and drags it outward (-x) by PULL_DIST
    for side in ("left", "right"):
        g = grips[f"{side}_gripper"]
        near = g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - PULL_TAB)) <= PULL_REACH
        if near:
            if m.pull_x0[side] is None:
                m.pull_x0[side] = float(g.tcp[0])
            if float(g.tcp[0]) <= m.pull_x0[side] - PULL_DIST:
                m.pull_applied = True
        else:
            m.pull_x0[side] = None
    if m.pull_applied:
        if not m.through:
            m.not_through = True
        elif not m.spread:
            m.not_spread = True
        elif not (m.dropped):
            m.pull_ok = True

    # 6. write the three cotter bodies (posed by the declared carry / declared bend; no dynamics)
    _write_part(env, "cotter", m.pos.copy(), m.quat.copy(), env_ids)
    for leg in LEGS:
        root, length, sign = LEGS[leg]
        root_w = m.pos + R @ root
        Rz = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), sign * m.bend[leg])
        _write_part(env, f"leg_{leg}", root_w, ik.matrix_to_quat_xyzw(R @ Rz), env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s cotter at {np.round(p, 4)} (hole {np.round(HOLE_CENTER, 4)}) "
              f"axis {math.degrees(axis_err):+5.1f} deg; bend long/short "
              f"{math.degrees(m.bend['long']):+5.1f}/{math.degrees(m.bend['short']):+5.1f}; "
              f"align {int(m.aligned)} thru {int(m.through)} spread {int(m.spread)} pull {int(m.pull_ok)}"
              f"{'  NOT_THROUGH' if m.not_through else ''}{'  NOT_SPREAD' if m.not_spread else ''}"
              f"{'  DROPPED' if m.dropped else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    out["cotter"] = ObjectState(pos=m.pos.copy(), quat_xyzw=m.quat.copy(), lin_vel=np.zeros(3),
                                ang_vel=np.zeros(3), low=m.pos.copy(), high=m.pos.copy())
    for key in ("aligned", "through", "spread", "pull_ok", "seated", "not_through", "not_spread", "dropped"):
        out[key] = getattr(m, key)
    out["bend"] = dict(m.bend)
    out["held"] = m.held


# -- scoring -------------------------------------------------------------------------------------------------------------
STEPS = ("cotter aligned with holes", "cotter through the cross-hole", "legs spread apart", "pull-test holds")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("aligned"))
    s1 = s0 and bool(objects.get("through"))
    s2 = s1 and bool(objects.get("spread"))
    s3 = s2 and bool(objects.get("pull_ok"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("not_through") or objects.get("not_spread") or objects.get("dropped"))


def _current_ok(objects) -> bool:
    """The joint is still secure as things stand: cotter seated, legs spread, no damage latch."""
    return bool(objects.get("seated") and objects.get("spread") and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([PIVOT_XY[0], PIVOT_XY[1], TOP + HOLE_Z_LOCAL])
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.opening < GRIP_RELEASE or g.contact >= GRIP_FORCE:
            return False
        for pt in _pad_points(g):
            if float(np.linalg.norm(pt - axis)) < GRIP_CLEAR + 0.06:
                return False
    return True


def _home(objects, side) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


def _full(objects) -> bool:
    return bool(all(steps_done(objects)) and _current_ok(objects) and _clear(objects)
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
    done = steps_done(objects)
    pts = POINTS * sum(done)
    if _caps(objects):
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
_DOWN = yam_demo.grip_rotation(0.0)   # the cotter stays horizontal, so the grip rotation is constant
_OPEN = FINGER_OPEN


def _grip_for(bearing_rot):
    return bearing_rot @ _DOWN


def _tcp_for(centre, bearing_rot):
    return np.asarray(centre, float) + bearing_rot @ np.array([0.0, 0.0, GRASP_UP])


def _w(dur, centre, finger, label, bearing_rot=None):
    """Waypoint from a cotter-centre local z (world z = TOP + z) and a cotter rotation."""
    r = np.eye(3) if bearing_rot is None else bearing_rot
    tcp = _tcp_for((centre[0], centre[1], TOP + centre[2]), r)
    return Waypoint(dur, tuple(tcp), finger, label, rot=_grip_for(r))


def _direct(dur, tcp, finger, label):
    """Waypoint straight from a world TCP (used for the leg-push and pull-test gestures)."""
    return Waypoint(dur, tuple(np.asarray(tcp, float)), finger, label, rot=_DOWN)


def _hover(centre, finger, label, bearing_rot=None):
    return _w(1.6, centre, finger, label, bearing_rot)


def _pick(centre, finger, label):
    """Open above the part, descend, close on it, lift clear (all upright)."""
    gz = centre[2] + GRASP_UP
    return [_hover((centre[0], centre[1], gz + 0.045), _OPEN, "over the " + label),
            _w(1.2, (centre[0], centre[1], centre[2]), _OPEN, "down to the " + label),
            _w(0.5, (centre[0], centre[1], centre[2]), _OPEN, "settle at the " + label),
            _w(0.7, (centre[0], centre[1], centre[2]), finger, "grip the " + label),
            _w(1.4, (centre[0], centre[1], gz + 0.045), finger, "lift the " + label)]


def _straight_tip(leg):
    root, length, _ = LEGS[leg]
    return HOLE_CENTER + root + np.array([length, 0.0, 0.0])


def _bent_station(leg, deg=SPREAD_DEG):
    root, length, sign = LEGS[leg]
    Rz = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), sign * math.radians(deg))
    return HOLE_CENTER + root + Rz @ np.array([length, 0.0, 0.0])


def demo():
    """The left arm lifts the cotter, aligns it with the cross-hole, pushes it through, bends both legs apart with
    two fingers, then pulls the joint for a second and goes home.  The right arm stays home.  All waypoints are
    reached with known geometry and every approach is from above."""
    left = []
    # A. pick the straight cotter from its white holder
    stage_centre = (STAGE_XY[0], STAGE_XY[1], STAGE_CZ)
    left += _pick(stage_centre, HOLD, "cotter")

    # B. carry over the cross-hole and align the cotter's axis with it
    hz = HOLE_Z_LOCAL + 0.010
    left += [_hover((INSERT_X, PIVOT_XY[1], hz), HOLD, "cotter over the cross-hole"),
             _w(1.4, (INSERT_X, PIVOT_XY[1], hz), HOLD, "align cotter with hole")]

    # C. push it fully through and let go
    left += [_w(1.4, (INSERT_X, PIVOT_XY[1], HOLE_Z_LOCAL), HOLD, "push cotter through the hole"),
             _w(0.8, (INSERT_X, PIVOT_XY[1], HOLE_Z_LOCAL), HOLD, "seat the cotter eye"),
             _w(0.6, (INSERT_X, PIVOT_XY[1], HOLE_Z_LOCAL), _OPEN, "release the cotter"),
             _w(0.8, (INSERT_X, PIVOT_XY[1], hz), _OPEN, "lift away from the cotter")]

    # D. bend each leg apart: close on the straight tip, sweep it outward to the spread mark, lift away
    for leg in ("long", "short"):
        tip = _straight_tip(leg)
        # Aim 4 deg past the declared spread mark: Isaac's arm tracking otherwise lands ~0.5 deg short.
        mark = _bent_station(leg, SPREAD_DEG + 4.0)
        left += [_direct(1.6, tip + np.array([0, 0, 0.030]), _OPEN, f"over the {leg} leg"),
                 _direct(1.2, tip, _OPEN, f"down to the {leg} leg"),
                 _direct(0.6, tip, HOLD, f"grip the {leg} leg"),
                 _direct(1.4, mark, HOLD, f"spread the {leg} leg out"),
                 _direct(0.8, mark + np.array([0, 0, 0.030]), _OPEN, f"lift from the {leg} leg")]

    # E. pull-test the joint at the red tab (pull outward along -x), hold, release
    pulled = PULL_TAB + np.array([-PULL_CMD, 0, 0])
    left += [_direct(1.6, PULL_TAB + np.array([0, 0, 0.030]), _OPEN, "over the pull tab"),
             _direct(1.2, PULL_TAB, _OPEN, "down to the pull tab"),
             _direct(0.7, PULL_TAB, HOLD, "grip the pull tab"),
             _direct(1.2, pulled, HOLD, "pull-test the joint"),
             _direct(1.0, pulled, HOLD, "hold the pull test"),
             _direct(0.8, pulled, _OPEN, "release the pull tab"),
             _direct(0.8, PULL_TAB + np.array([0, 0, 0.030]), _OPEN, "lift from the pull tab")]

    # F. go home
    left += [Waypoint(2.2, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]
    right = [Waypoint(2.2, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    b = objects.get("bend", {})
    return (f"bend {math.degrees(b.get('long', 0)):+.0f}/{math.degrees(b.get('short', 0)):+.0f}, "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('not_through','not_spread','dropped') if objects.get(k)) or '-'}")
