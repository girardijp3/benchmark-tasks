"""On the station table: a real 20 mm spherical-OD bearing insert staged beside a pressed pillow-block-style housing.
The housing carries a concave spherical seat whose axis is inclined 8 deg from vertical (it is bolted to a sloped
conveyor frame) and a grease port at a fixed azimuth on the seat rim.  The robot picks the bearing, carries it over
the seat, tilts its axis to the seat axis, rotates it about that axis to register its grease hole with the housing
grease port, lowers it flush into the spherical seat with a light 4-7 N press, releases it and goes home.  A bearing
entered too steeply jams; one pressed with its grease hole unregistered scores the housing bore; one pushed below
flush is a forced press.  Those three damage latches are scored caps.

The bearing is a real gravity-free rigid body in its own USD file.  While the gripper is closed on the bearing's grasp
point it follows the gripper (a declared carry, like the family's detents); the robot must still reach, close on,
orient, tilt, rotate, carry and seat it.  The spherical OD, spherical seat, grease port and index hole are real
geometry.  The scorer reads the mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=bearing_insert_tilt.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/bearing-insert-tilt/bearing_insert_tilt.py
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

TAG = "bit"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the housing fixture (local frame: seat axis origin at x,y = 0, bench top z 0; world z = TOP + local z) --------------
HOUSING_XY = (0.250, 0.000)
HOUSING_POS = (HOUSING_XY[0], HOUSING_XY[1], TOP)
SEAT_TILT = math.radians(8.0)          # the seat axis leans +x from vertical (sloped conveyor mounting)
PORT_PSI = math.radians(60.0)          # grease-port azimuth about the seat axis from the minimal-tilt index direction
SEAT_XY = (0.250, 0.000)               # bearing centre XY when seated flush
SEAT_CZ = 0.024                        # bearing centre z when seated flush (local)
BALL_R = 0.010                         # spherical OD radius = 20 mm / 2 (true size)
BEARING_W = 0.007                      # bearing axial width
BORE_R = 0.004                         # 8 mm shaft bore

# -- staging pocket -------------------------------------------------------------------------------------------------------
STAGE_XY = (0.250, 0.062)
STAGE_CZ = 0.010

# -- orientation targets ----------------------------------------------------------------------------------------------------
TILT_ROT = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), SEAT_TILT)          # minimal tilt: axis -> seat axis
SEAT_AXIS = TILT_ROT @ np.array([0.0, 0.0, 1.0])                            # world direction of the seat axis
SEAT_ROT = axis_angle_matrix(SEAT_AXIS, PORT_PSI) @ TILT_ROT                 # axis on seat, index on grease port
GREASE_DIR = SEAT_ROT @ np.array([1.0, 0.0, 0.0])                          # world direction of the port / registered index

TILT_TOL = math.radians(4.0)           # bearing axis must be within 4 deg of the seat axis to align
JAM_TILT = math.radians(6.0)           # entering the seat with more than 6 deg of tilt jams the rim
YAW_TOL = math.radians(15.0)           # index within 15 deg of the port (two holes 180 deg apart -> mod 180)
OVER_SEAT = 0.030                      # horizontal window for the tilt / rotate work above the seat
ENTRY_Z_BAND = 0.008                   # centre this far above flush counts as entering the seat
SEAT_XY_TOL = 0.007
SEAT_Z_TOL = 0.005
FORCE_TOL = 0.004                      # centre pushed this far below flush is a forced press
PRESS_DWELL = 0.35                     # s: light press held at flush before it counts

# -- gripper / grasp -------------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0              # N: a real closed grip
GRIP_CLOSE = 0.034            # opening at/below which a closed finger pair may take the bearing
GRIP_RELEASE = 0.045          # opening above which a held bearing is let go
GRIP_REACH = 0.016            # m: TCP within this of the bearing's grasp point
GRIP_CLEAR = 0.045            # m: no pad within this of the housing at the scored state
HOME_SLACK = 0.020
GRASP_UP = 0.012              # the TCP sits this far above the bearing centre, along the bearing axis
HOLD = -0.013                 # finger target while carrying the bearing (about its 7 mm width)

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


HOUSING = UsdPart("housing", asset="housing", xy=HOUSING_XY, extent=0.09, mass=None, fixed=True,
                  usd_path=str(ASSETS / "housing.usda"), pos=HOUSING_POS)
BEARING = UsdPart("bearing", asset="bearing", xy=STAGE_XY, extent=0.02, mass=0.030,
                  usd_path=str(ASSETS / "bearing.usda"), pos=tuple(START_POS))
OBJECTS = [HOUSING, BEARING]
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
    if getattr(scene.task_object, "_bit_usd_parts", False):
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
            env_cfg.events.bit_reset = EventTermCfg(func=_reset_bearing, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("housing")})
            env_cfg.events.bit_step = EventTermCfg(func=_bearing_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("housing")})
            return env_cfg

        return with_events

    task_object._bit_usd_parts = True
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


def _tilt_error(R):
    """Angle between the bearing's bore axis (local z) and the housing's inclined seat axis."""
    c = float(np.clip(np.asarray(R)[:, 2] @ SEAT_AXIS, -1.0, 1.0))
    return math.acos(c)


def _yaw_error(R):
    """Azimuth of the bearing's grease hole about the seat axis, folded to [0, 90] deg.

    The outer ring has two lubrication holes 180 deg apart, so orientations a half turn apart are equivalent.
    """
    i = np.asarray(R)[:, 0]
    ip = i - (i @ SEAT_AXIS) * SEAT_AXIS
    gp = GREASE_DIR - (GREASE_DIR @ SEAT_AXIS) * SEAT_AXIS
    ni, ng = np.linalg.norm(ip), np.linalg.norm(gp)
    if ni < 1e-9 or ng < 1e-9:
        return math.pi
    ip, gp = ip / ni, gp / ng
    ang = math.acos(float(np.clip(ip @ gp, -1.0, 1.0)))
    return min(ang, math.pi - ang)


class _Mechanism:
    def __init__(self):
        self.pos = START_POS.copy()
        self.quat = START_QUAT.copy()
        self.held = None
        self.offset = np.zeros(3)
        self.rot_off = np.eye(3)
        # current state (recomputed each step)
        self.tilt_err = math.radians(90.0)
        self.yaw_err = math.pi
        self.seated_flush = False
        # progress / damage latches
        self.lifted = False
        self.tilted_ok = False
        self.registered_ok = False
        self.pressed = False
        self.jammed = False
        self.scored = False
        self.forced = False
        self.press_t = 0.0
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


def _reset_bearing(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_part(env, "bearing", START_POS, START_QUAT, env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: 20 mm spherical-OD bearing staged; housing spherical seat idle, grease port open",
              flush=True)


def _grasp_point():
    return _MECH.pos + _rot(_MECH.quat) @ np.array([0.0, 0.0, GRASP_UP])


def _in_allowed_zone(pos):
    x, y, z = pos
    if z < TOP - 0.01 or x < 0.10 or x > 0.40 or abs(y) > 0.18:
        return False
    return True


def _bearing_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)

    # 1. grasp / carry / release the bearing
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
                m.forced = True
        else:
            m.pos = g.tcp + g.rot @ m.offset
            m.quat = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off)

    # 2. geometry at this instant
    R = _rot(m.quat)
    m.tilt_err = _tilt_error(R)
    m.yaw_err = _yaw_error(R)
    p = m.pos
    at_seat = float(np.linalg.norm(p[:2] - np.asarray(SEAT_XY))) < SEAT_XY_TOL
    over_seat = float(np.linalg.norm(p[:2] - np.asarray(SEAT_XY))) < OVER_SEAT
    above_entry = p[2] > TOP + SEAT_CZ + ENTRY_Z_BAND
    aligned = m.tilt_err <= TILT_TOL
    registered = aligned and m.yaw_err <= YAW_TOL
    m.seated_flush = bool(at_seat and abs(p[2] - (TOP + SEAT_CZ)) < SEAT_Z_TOL and registered)

    # 3. progress latches
    if p[2] > TOP + STAGE_CZ + 0.025 and float(np.linalg.norm(p[:2] - np.asarray(STAGE_XY))) > 0.020:
        m.lifted = True
    if m.lifted and over_seat and above_entry and aligned:
        m.tilted_ok = True
    if m.tilted_ok and over_seat and above_entry and registered:
        m.registered_ok = True

    # 4. damage caps: a steep entry jams, an unregistered press scores the bore, an over-deep press is forced
    entering = at_seat and p[2] <= TOP + SEAT_CZ + ENTRY_Z_BAND
    if entering and m.tilt_err > JAM_TILT:
        m.jammed = True
    if entering and m.yaw_err > YAW_TOL and p[2] <= TOP + SEAT_CZ + 0.002:
        m.scored = True
    if at_seat and p[2] < TOP + SEAT_CZ - FORCE_TOL:
        m.forced = True

    # 5. light press: a closed gripper dwelling at flush with the bearing registered (4-7 N declared, not integrated)
    if m.seated_flush and m.held is not None and not (m.jammed or m.scored or m.forced):
        m.press_t += float(base.step_dt)
    else:
        m.press_t = 0.0
    if m.press_t >= PRESS_DWELL:
        m.pressed = True

    # 6. write the bearing (posed by the declared carry; no dynamics)
    _write_part(env, "bearing", m.pos.copy(), m.quat.copy(), env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s bearing at {np.round(p, 4)} (seat {np.round([SEAT_XY[0], SEAT_XY[1], TOP+SEAT_CZ], 4)}) "
              f"tilt_err {math.degrees(m.tilt_err):+5.1f} yaw_err {math.degrees(m.yaw_err):+5.1f} "
              f"lift {int(m.lifted)} tilt {int(m.tilted_ok)} reg {int(m.registered_ok)} flush {int(m.seated_flush)} "
              f"press {int(m.pressed)}"
              f"{'  JAM' if m.jammed else ''}{'  SCORE' if m.scored else ''}{'  FORCED' if m.forced else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    out["bearing"] = ObjectState(pos=m.pos.copy(), quat_xyzw=m.quat.copy(), lin_vel=np.zeros(3),
                                 ang_vel=np.zeros(3), low=m.pos.copy(), high=m.pos.copy())
    for key in ("tilt_err", "yaw_err", "seated_flush", "lifted", "tilted_ok", "registered_ok", "pressed",
                "jammed", "scored", "forced"):
        out[key] = getattr(m, key)
    out["held"] = m.held


# -- scoring -------------------------------------------------------------------------------------------------------------
STEPS = ("bearing lifted clear", "tilted to seat axis", "grease hole registered", "seated flush + pressed")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("lifted"))
    s1 = s0 and bool(objects.get("tilted_ok"))
    s2 = s1 and bool(objects.get("registered_ok"))
    s3 = s2 and bool(objects.get("seated_flush")) and bool(objects.get("pressed"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("jammed") or objects.get("scored") or objects.get("forced"))


def _current_ok(objects) -> bool:
    """The bearing is still seated flush and registered, with no damage latch."""
    return bool(objects.get("seated_flush") and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([SEAT_XY[0], SEAT_XY[1], TOP + SEAT_CZ + 0.010])
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
_DOWN = yam_demo.grip_rotation(0.0)
_TILT_GRIP = TILT_ROT @ _DOWN
_SEAT_GRIP = SEAT_ROT @ _DOWN
_OPEN = FINGER_OPEN


def _grip_for(bearing_rot):
    """Gripper rotation that poses the carried bearing at ``bearing_rot`` (the carry offset is fixed at grasp)."""
    return bearing_rot @ _DOWN


def _tcp_for(centre, bearing_rot):
    """TCP that puts the bearing centre at ``centre`` (world xyz) with ``bearing_rot``."""
    return np.asarray(centre, float) + bearing_rot @ np.array([0.0, 0.0, GRASP_UP])


def _w(dur, centre, finger, label, bearing_rot=None):
    """Waypoint from a bearing-centre local z (world z = TOP + z) and a bearing rotation."""
    r = np.eye(3) if bearing_rot is None else bearing_rot
    tcp = _tcp_for((centre[0], centre[1], TOP + centre[2]), r)
    return Waypoint(dur, tuple(tcp), finger, label, rot=_grip_for(r))


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


def demo():
    """The left arm picks the bearing, tilts its axis to the inclined seat, rotates its grease hole to the housing
    port, lowers it flush with a light press, releases it and goes home.  The right arm stays home.  All waypoints are
    reached with known geometry and every approach is from above."""
    left = []
    # A. pick the upright bearing from its staging pocket
    stage_centre = (STAGE_XY[0], STAGE_XY[1], STAGE_CZ)
    left += _pick(stage_centre, HOLD, "bearing")

    # B. carry over the seat, then tilt the bore axis to the seat axis (8 deg)
    hz = SEAT_CZ + 0.025                       # hover height above the seat, clear of the entry band
    left += [_hover((SEAT_XY[0], SEAT_XY[1], hz), HOLD, "bearing over the spherical seat")]
    left += [_w(1.6, (SEAT_XY[0], SEAT_XY[1], hz), HOLD, "tilt axis to the seat", TILT_ROT)]

    # C. rotate the bearing about the seat axis to register its grease hole with the housing port (60 deg)
    left += [_w(1.6, (SEAT_XY[0], SEAT_XY[1], hz), HOLD, "rotate grease hole to port", SEAT_ROT)]

    # D. lower flush into the spherical seat and hold a light 4-7 N press, then release
    left += [_w(1.4, (SEAT_XY[0], SEAT_XY[1], SEAT_CZ), HOLD, "press bearing flush", SEAT_ROT),
             _w(0.8, (SEAT_XY[0], SEAT_XY[1], SEAT_CZ), HOLD, "hold the press", SEAT_ROT),
             _w(0.5, (SEAT_XY[0], SEAT_XY[1], SEAT_CZ), _OPEN, "release the bearing", SEAT_ROT),
             _w(0.8, (SEAT_XY[0], SEAT_XY[1], hz), _OPEN, "lift away from the bearing", SEAT_ROT)]

    # E. go home
    left += [Waypoint(2.2, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]
    right = [Waypoint(2.2, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"tilt {math.degrees(objects.get('tilt_err', 0.0)):+.0f} yaw {math.degrees(objects.get('yaw_err', 0.0)):+.0f}, "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('jammed','scored','forced') if objects.get(k)) or '-'}")
