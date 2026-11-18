"""On the station bench: a real 15 mm eccentric furniture cam, its mating connecting bolt and a screwdriver.
The flat face panel lies on the bench with a 15 mm blind cam bore; the mating panel lies beside it with an 8 mm
bolt hole in its joining edge and starts 3 mm short of the face panel.  The robot drops the cam into its pocket,
inserts the bolt square into the mating panel, engages the screwdriver in the cam's cross recess and turns the cam
about its (vertical) bore axis ~180 deg with a declared 0.3-0.6 N m hand torque.  The eccentric ramp draws the bolt
head in and pulls the two panels flush; the joint is scored once it has held flush for 1 s and both grippers are home.
A cam turned while still out of its pocket, a bolt driven in crooked (cross-threaded), or panels forced shut before
the joint is captured are damage latches and scored caps.

The cam, bolt and screwdriver are real gravity-free rigid bodies in their own USD files.  A closed gripper within
16 mm of a part's grasp point carries it (a declared carry, like the family's detents); the robot must still reach,
close on, orient, place, engage and turn it.  The 15 mm cam bore, 8 mm bolt hole, eccentric ramp, cross recess and
panel gap are real geometry; the 0.3-0.6 N m torque and the draw are declared, not integrated.  The scorer reads
the mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=flat_pack_cam_lock.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/flat-pack-cam-lock/flat_pack_cam_lock.py
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

TAG = "fcl"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- face panel (fixed, lying flat): 16 mm particleboard, 160 x 140 mm --------------------------------------------------
PANEL_A_XY = (0.250, 0.030)
PANEL_A_TOP = 0.016
A_EDGE_Y = -0.040                       # joining edge of the face panel (nearest the robot)

# -- cam bore: 15 mm diameter, 13 mm deep, centre 24 mm in from the joining edge (Minifix drilling distance B = 24) ---
CAM_XY = (0.250, A_EDGE_Y + 0.024)      # (0.250, -0.016)
CAM_BORE_R = 0.0075                     # 15 mm / 2 (true size)
CAM_R = 0.0075                          # cam outer radius = 15 mm diameter
CAM_H = 0.012                           # cam height
CAM_SEAT_CZ = 0.009                     # cam centre z when seated (top 0.015, bottom 0.003)
BORE_BOTTOM = 0.003

# -- mating panel (slides +y as the cam draws it flush): 16 mm, 160 x 100 mm -------------------------------------------
PANEL_B_X = 0.250
PANEL_B_LEN_Y = 0.100
PANEL_B_CY_START = -0.093               # far edge at -0.043, i.e. a 3 mm gap to the face panel's -0.040 edge
PANEL_B_CY_FLUSH = -0.090               # far edge at -0.040 (panels touching)
PANEL_B_CZ = 0.008
PANEL_GAP_START = 0.003
PANEL_FLUSH_TOL = 0.0005                # 0.5 mm feeler-gauge test

# -- mating bolt: M6 shank, 7 mm head, 8 mm hole, axis along +y, collar flush with the panel's far edge ---------------
BOLT_XY = (0.250, -0.030)
BOLT_CZ = 0.008
BOLT_R = 0.003                          # M6 shank
BOLT_HEAD_R = 0.0035                    # 7 mm head, passes the 8 mm hole
BOLT_HOLE_R = 0.004                     # 8 mm hole
BOLT_TO_Z = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -math.pi / 2)   # local +z -> world +y

# -- staging -----------------------------------------------------------------------------------------------------------
CAM_STAGE = (0.250, 0.065, TOP + 0.010)
BOLT_STAGE = (0.210, 0.135, TOP + 0.006)
SCREW_STAGE = (0.325, 0.135, TOP + 0.030)

# -- orientation / engagement windows -----------------------------------------------------------------------------------
CAM_XY_TOL = 0.0025                     # cam centre must land within 2.5 mm of the bore centre
CAM_Z_TOL = 0.0025
CAM_TILT_TOL = math.radians(9.0)        # upright in the pocket
BOLT_XY_TOL = 0.008
BOLT_Z_TOL = 0.004
BOLT_TILT_TOL = math.radians(8.0)       # square to the hole
CROSS_TILT = math.radians(20.0)         # beyond this at the hole = cross-threaded cap
DRIVE_POINT = np.array([CAM_XY[0], CAM_XY[1], TOP + 0.015])
SCREW_TIP_LOCAL = np.array([0.0, 0.0, -0.048])   # PZ2 tip at local -z; handle at +z
ENGAGE_REACH = 0.012                    # screwdriver tip within 12 mm of the cam top
TURN_TARGET = math.radians(180.0)       # ~half a turn clockwise
TURN_DONE = math.radians(165.0)         # counts as turned from 165 deg
TURN_CAPTURE = math.radians(90.0)       # eccentric begins to draw the bolt after capture
TURN_STRIP = math.radians(215.0)        # over-rotation past ~210 deg strips the cam -> panels_not_flush cap
OVER_HOLE = 0.012                       # bolt this close to the hole is an insertion attempt

# -- gripper / grasp ----------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0              # N: a real closed grip
GRIP_CLOSE = 0.034            # opening at/below which a closed finger pair may take a part
GRIP_RELEASE = 0.045          # opening above which a held part is let go
GRIP_REACH = 0.011            # m: TCP within this of a part's grasp point (keeps adjacent parts from being grabbed)
GRIP_CLEAR = 0.045            # m: no pad within this of the joint at the scored state
HOME_SLACK = 0.020
GRASP_UP = 0.012              # TCP sits this far along the part's local +z from its centre
HOLD = -0.013                 # finger target while carrying a part
BOLT_HOLD = -0.010
SCREW_HOLD = -0.016
PRESS_DWELL = 0.35            # s: the declared 0.3-0.6 N m hand torque held at flush

START_POS = np.array(CAM_STAGE, dtype=float)   # unused by the sim, kept for parity with the family
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


FIXTURE = UsdPart("fixture", asset="fixture", xy=PANEL_A_XY, extent=0.20, mass=None, fixed=True,
                  usd_path=str(ASSETS / "fixture.usda"),
                  pos=(PANEL_A_XY[0], PANEL_A_XY[1], TOP))
PANEL_B = UsdPart("panel_b", asset="panel_b", xy=(PANEL_B_X, PANEL_B_CY_START), extent=0.16, mass=None, fixed=False,
                  usd_path=str(ASSETS / "panel_b.usda"),
                  pos=(PANEL_B_X, PANEL_B_CY_START, TOP))
CAM = UsdPart("cam", asset="cam", xy=CAM_STAGE[:2], extent=0.015, mass=0.015,
              usd_path=str(ASSETS / "cam.usda"), pos=tuple(CAM_STAGE))
BOLT = UsdPart("bolt", asset="bolt", xy=BOLT_STAGE[:2], extent=0.030, mass=0.010,
               usd_path=str(ASSETS / "bolt.usda"), pos=tuple(BOLT_STAGE))
SCREW = UsdPart("screwdriver", asset="screwdriver", xy=SCREW_STAGE[:2], extent=0.060, mass=0.060,
                usd_path=str(ASSETS / "screwdriver.usda"), pos=tuple(SCREW_STAGE))
OBJECTS = [FIXTURE, PANEL_B, CAM, BOLT, SCREW]
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
    if getattr(scene.task_object, "_fcl_usd_parts", False):
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
            env_cfg.events.fcl_reset = EventTermCfg(func=_reset_joint, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("fixture")})
            env_cfg.events.fcl_step = EventTermCfg(func=_joint_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("fixture")})
            return env_cfg

        return with_events

    task_object._fcl_usd_parts = True
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


def _yaw(R):
    R = np.asarray(R, float)
    return math.atan2(R[1, 0], R[0, 0])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _tilt_error(R):
    """Angle between a part's local +z axis and world vertical."""
    c = float(np.clip(np.asarray(R)[:, 2] @ np.array([0.0, 0.0, 1.0]), -1.0, 1.0))
    return math.acos(c)


def _bolt_axis_error(R):
    """Angle between the bolt's local +z (its shank) and the bolt-hole axis (+y)."""
    c = float(np.clip(np.asarray(R)[:, 2] @ np.array([0.0, 1.0, 0.0]), -1.0, 1.0))
    return math.acos(c)


class _Mechanism:
    def __init__(self):
        self.cam_pos = np.array(CAM_STAGE, float)
        self.cam_quat = START_QUAT.copy()
        self.bolt_pos = np.array(BOLT_STAGE, float)
        self.bolt_quat = np.array([*ik.matrix_to_quat_xyzw(BOLT_TO_Z)], float)
        self.screw_pos = np.array(SCREW_STAGE, float)
        self.screw_quat = START_QUAT.copy()
        self.cam_held = self.bolt_held = self.screw_held = None
        self.cam_off = np.zeros(3); self.cam_rot_off = np.eye(3)
        self.bolt_off = np.zeros(3); self.bolt_rot_off = np.eye(3)
        self.screw_off = np.zeros(3); self.screw_rot_off = np.eye(3)
        self.panel_cy = PANEL_B_CY_START
        # live geometry
        self.cam_tilt = math.pi
        self.bolt_axis_err = math.pi
        self.engaged = False
        self.turn = 0.0
        self.last_yaw = None
        self.gap = PANEL_GAP_START
        # progress / damage latches
        self.cam_seated = False
        self.seated_latched = False
        self.bolt_engaged = False
        self.bolt_latched = False
        self.turned = False
        self.flushed = False
        self.torqued = False
        self.cap_cam = False
        self.cap_bolt = False
        self.cap_panel = False
        self.torque_t = 0.0
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


def _reset_joint(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_part(env, "cam", _MECH.cam_pos, _MECH.cam_quat, env_ids)
    _write_part(env, "bolt", _MECH.bolt_pos, _MECH.bolt_quat, env_ids)
    _write_part(env, "screwdriver", _MECH.screw_pos, _MECH.screw_quat, env_ids)
    _write_part(env, "panel_b", (PANEL_B_X, PANEL_B_CY_START, TOP), START_QUAT, env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: 15 mm cam staged, M6 bolt staged, screwdriver in holder; "
              f"panels {PANEL_GAP_START * 1000:.0f} mm apart", flush=True)


def _grasp_point(centre, quat):
    return np.asarray(centre, float) + _rot(quat) @ np.array([0.0, 0.0, GRASP_UP])


def _in_allowed_zone(pos):
    x, y, z = pos
    if z < TOP - 0.01 or x < 0.10 or x > 0.40 or abs(y) > 0.20:
        return False
    return True


def _carry(m, key, g, close, release, graspable=True):
    """Generic declared carry. Returns True while the part is held.  A part already seated in its bore or hole is
    no longer grabbable by proximity: a real cam/bolt sits down in its seat, not loose on the bench."""
    if getattr(m, key + "_held") is None and graspable:
        if g.opening <= close and float(np.linalg.norm(g.tcp - getattr(m, key + "_gp"))) <= GRIP_REACH:
            setattr(m, key + "_held", "left")
            setattr(m, key + "_off", g.rot.T @ (getattr(m, key + "_pos") - g.tcp))
            setattr(m, key + "_rot_off", g.rot.T @ _rot(getattr(m, key + "_quat")))
    if getattr(m, key + "_held") is not None:
        if g.opening > release:
            setattr(m, key + "_held", None)
            if not _in_allowed_zone(getattr(m, key + "_pos")):
                m.cap_panel = True
        else:
            setattr(m, key + "_pos", g.tcp + g.rot @ getattr(m, key + "_off"))
            setattr(m, key + "_quat", ik.matrix_to_quat_xyzw(g.rot @ getattr(m, key + "_rot_off")))
    return getattr(m, key + "_held") is not None


def _joint_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)
    gl = grips["left_gripper"]

    # grasp points at this instant
    m.cam_gp = _grasp_point(m.cam_pos, m.cam_quat)
    m.bolt_gp = _grasp_point(m.bolt_pos, m.bolt_quat)
    m.screw_gp = _grasp_point(m.screw_pos, m.screw_quat)

    # 1. declared carries (one arm does the whole task; the right stays home)
    _carry(m, "cam", gl, GRIP_CLOSE, GRIP_RELEASE, graspable=not m.seated_latched)
    _carry(m, "bolt", gl, GRIP_CLOSE, GRIP_RELEASE, graspable=not m.bolt_latched)
    _carry(m, "screw", gl, GRIP_CLOSE, GRIP_RELEASE)

    # 2. geometry at this instant
    Rcam = _rot(m.cam_quat)
    Rbolt = _rot(m.bolt_quat)
    m.cam_tilt = _tilt_error(Rcam)
    m.bolt_axis_err = _bolt_axis_error(Rbolt)
    at_bore = float(np.linalg.norm(m.cam_pos[:2] - np.asarray(CAM_XY))) < CAM_XY_TOL
    cam_up = m.cam_tilt < CAM_TILT_TOL
    at_cam_z = abs(m.cam_pos[2] - (TOP + CAM_SEAT_CZ)) < CAM_Z_TOL
    m.cam_seated = bool(at_bore and cam_up and at_cam_z)
    at_hole = float(np.linalg.norm(m.bolt_pos[:2] - np.asarray(BOLT_XY))) < BOLT_XY_TOL
    bolt_square = m.bolt_axis_err < BOLT_TILT_TOL
    at_bolt_z = abs(m.bolt_pos[2] - (TOP + BOLT_CZ)) < BOLT_Z_TOL
    m.bolt_engaged = bool(at_hole and bolt_square and at_bolt_z)

    # 3. cross-thread cap: a crooked bolt driven into the hole
    if float(np.linalg.norm(m.bolt_pos[:2] - np.asarray(BOLT_XY))) < OVER_HOLE and m.bolt_axis_err > CROSS_TILT:
        m.cap_bolt = True

    # 4. screwdriver engagement with the cam's cross recess (needs a seated cam and an upright tool)
    m.screw_tip = m.screw_pos + _rot(m.screw_quat) @ SCREW_TIP_LOCAL
    m.engaged = bool(m.screw_held is not None and m.cam_seated and m.cam_tilt < CAM_TILT_TOL
                     and _tilt_error(_rot(m.screw_quat)) < CAM_TILT_TOL
                     and float(np.linalg.norm(m.screw_tip - DRIVE_POINT)) < ENGAGE_REACH)
    # turning the cam while it is not seated cracks the bore -> cam_not_seated cap
    ynow = _yaw(_rot(m.screw_quat))
    dyaw = 0.0
    if m.last_yaw is not None:
        dyaw = _wrap(ynow - m.last_yaw)
    m.last_yaw = ynow
    if m.engaged:
        m.turn += -dyaw  # clockwise seen from above is negative yaw; report a positive turn
        # over-rotation strips the die-cast zinc cam
        if m.turn > TURN_STRIP:
            m.cap_panel = True
        m.cam_quat = np.array([*ik.matrix_to_quat_xyzw(
            axis_angle_matrix(np.array([0.0, 0.0, 1.0]), m.turn))], float)
    elif abs(dyaw) > 1e-4 and m.screw_held is not None and not m.cam_seated:
        m.cap_cam = True

    # 5. the eccentric draws the mating panel in once the bolt is captured (declared draw)
    if m.bolt_engaged and m.cam_seated and m.turn > TURN_CAPTURE and not m.cap_panel:
        frac = float(np.clip((m.turn - TURN_CAPTURE) / (TURN_TARGET - TURN_CAPTURE), 0.0, 1.0))
        m.panel_cy = PANEL_B_CY_START + frac * (PANEL_B_CY_FLUSH - PANEL_B_CY_START)
    # panel B's joining edge is cy + len/2; the face panel's edge is A_EDGE_Y
    m.gap = abs(A_EDGE_Y - (m.panel_cy + PANEL_B_LEN_Y / 2.0))
    m.flushed = bool(m.gap <= PANEL_FLUSH_TOL and m.bolt_engaged and m.cam_seated and m.turn >= TURN_DONE)
    # forcing the panels shut before the joint is captured leaves them gaped/stripped
    if m.gap <= PANEL_FLUSH_TOL and not (m.bolt_engaged and m.turn >= TURN_DONE):
        m.cap_panel = True

    # 6. progress latches
    if m.cam_seated:
        m.seated_latched = True
    if getattr(m, "seated_latched", False) and m.bolt_engaged:
        m.bolt_latched = True
    if getattr(m, "bolt_latched", False) and m.turn >= TURN_DONE and m.cam_seated:
        m.turned = True

    # 7. declared 0.3-0.6 N m hand torque: a closed engaged tool dwelling at the flush joint
    if m.flushed and m.engaged and m.screw_held is not None and not _caps_state(m):
        m.torque_t += float(base.step_dt)
    else:
        m.torque_t = 0.0
    if m.torque_t >= PRESS_DWELL:
        m.torqued = True

    # 8. write the moving bodies (poses are posed by the declared carry / draw; no dynamics)
    _write_part(env, "cam", m.cam_pos.copy(), m.cam_quat.copy(), env_ids)
    _write_part(env, "bolt", m.bolt_pos.copy(), m.bolt_quat.copy(), env_ids)
    _write_part(env, "screwdriver", m.screw_pos.copy(), m.screw_quat.copy(), env_ids)
    _write_part(env, "panel_b", (PANEL_B_X, m.panel_cy, TOP), START_QUAT, env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s cam {np.round(m.cam_pos, 4)} bolt {np.round(m.bolt_pos, 4)} "
              f"turn {math.degrees(m.turn):+5.1f} gap {m.gap * 1000:4.1f} mm "
              f"seat {int(m.cam_seated)} bolt {int(m.bolt_engaged)} turnok {int(m.turned)} flush {int(m.flushed)} "
              f"torque {int(m.torqued)}"
              f"{'  CAM!' if m.cap_cam else ''}{'  THREAD!' if m.cap_bolt else ''}{'  PANEL!' if m.cap_panel else ''}",
              flush=True)
    m.t += float(base.step_dt)


def _caps_state(m):
    return bool(m.cap_cam or m.cap_bolt or m.cap_panel)


def extra_states(env, out, origin):
    m = _MECH
    out["cam"] = ObjectState(pos=m.cam_pos.copy(), quat_xyzw=m.cam_quat.copy(), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=m.cam_pos.copy(), high=m.cam_pos.copy())
    out["bolt"] = ObjectState(pos=m.bolt_pos.copy(), quat_xyzw=m.bolt_quat.copy(), lin_vel=np.zeros(3),
                              ang_vel=np.zeros(3), low=m.bolt_pos.copy(), high=m.bolt_pos.copy())
    out["screwdriver"] = ObjectState(pos=m.screw_pos.copy(), quat_xyzw=m.screw_quat.copy(), lin_vel=np.zeros(3),
                                     ang_vel=np.zeros(3), low=m.screw_pos.copy(), high=m.screw_pos.copy())
    for key in ("cam_tilt", "bolt_axis_err", "engaged", "turn", "gap", "cam_seated", "seated_latched",
                "bolt_engaged", "bolt_latched", "turned", "flushed", "torqued",
                "cap_cam", "cap_bolt", "cap_panel"):
        out[key] = getattr(m, key)
    out["held"] = (m.cam_held, m.bolt_held, m.screw_held)


# -- scoring -------------------------------------------------------------------------------------------------------------
STEPS = ("cam seated in its bore", "bolt engaged in the mating panel", "cam turned ~180 deg",
         "panels flush + torque held")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("cam_seated") or objects.get("seated_latched", False))
    s1 = bool(objects.get("bolt_engaged") or objects.get("bolt_latched", False))
    s2 = bool(objects.get("turned"))
    s3 = bool(objects.get("flushed")) and bool(objects.get("torqued"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("cap_cam") or objects.get("cap_bolt") or objects.get("cap_panel"))


def _current_ok(objects) -> bool:
    return bool(objects.get("flushed") and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([CAM_XY[0], CAM_XY[1], TOP + 0.012])
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.opening < GRIP_RELEASE or g.contact >= GRIP_FORCE:
            return False
        for pt in _pad_points(g):
            if float(np.linalg.norm(pt - axis)) < GRIP_CLEAR + 0.04:
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
_OPEN = FINGER_OPEN

# gripper rotation that poses a carried part with body rotation ``part_rot`` (carry offset captured at grasp)
def _grip_for(part_rot):
    return part_rot @ _DOWN


def _tcp_for(centre, part_rot):
    return np.asarray(centre, float) + part_rot @ np.array([0.0, 0.0, GRASP_UP])


def _w(dur, centre, finger, label, part_rot=None):
    r = np.eye(3) if part_rot is None else part_rot
    return Waypoint(dur, tuple(_tcp_for(centre, r)), finger, label, rot=_grip_for(r))


def _hover(centre, finger, label, part_rot=None):
    return _w(1.2, centre, finger, label, part_rot)


def _wb(dur, centre, finger, label):
    """Bolt waypoint: the bolt lies along +y but the wrist stays near-vertical (reachable); the fixed carry offset
    captured at grasp holds the bolt's axis.  ``centre`` is the desired bolt centre, not the TCP."""
    tcp = np.asarray(centre, float) + BOLT_TO_Z @ np.array([0.0, 0.0, GRASP_UP])
    return Waypoint(dur, tuple(tcp), finger, label, rot=_DOWN)


def _pick(centre, finger, label, part_rot=None, hold=HOLD):
    return [_hover((centre[0], centre[1], centre[2] + 0.045), _OPEN, "over the " + label, part_rot),
            _w(1.2, centre, _OPEN, "down to the " + label, part_rot),
            _w(0.5, centre, _OPEN, "settle at the " + label, part_rot),
            _w(0.7, centre, finger, "grip the " + label, part_rot),
            _w(1.4, (centre[0], centre[1], centre[2] + 0.045), finger, "lift the " + label, part_rot)]


def _place(centre, finger, label, part_rot=None):
    return [_w(1.4, centre, finger, label, part_rot),
            _w(0.5, centre, finger, "settle " + label, part_rot),
            _w(0.5, centre, _OPEN, "release " + label, part_rot),
            _w(1.0, (centre[0], centre[1], centre[2] + 0.045), _OPEN, "lift away from " + label, part_rot)]


def demo():
    """One arm: drop the cam in its bore, insert the bolt square into the mating panel, engage the screwdriver and
    turn the cam ~180 deg to draw the panels flush, hold the declared hand torque, retract and go home."""
    left = []
    # A. pick the cam and drop it upright into the 15 mm bore (arrow/slot toward the bolt edge)
    left += _pick(CAM_STAGE, HOLD, "cam")
    left += _place((CAM_XY[0], CAM_XY[1], TOP + CAM_SEAT_CZ), HOLD, "the cam in its bore")

    # B. pick the bolt (axis along +y, held by a vertical wrist) and insert it square into the mating panel's hole
    bhole = (BOLT_XY[0], BOLT_XY[1], TOP + BOLT_CZ)
    left += [_wb(1.2, (BOLT_STAGE[0], BOLT_STAGE[1], BOLT_STAGE[2] + 0.045), _OPEN, "over the bolt"),
             _wb(1.2, BOLT_STAGE, _OPEN, "down to the bolt"),
             _wb(0.5, BOLT_STAGE, _OPEN, "settle at the bolt"),
             _wb(0.7, BOLT_STAGE, BOLT_HOLD, "grip the bolt"),
             _wb(1.4, (BOLT_STAGE[0], BOLT_STAGE[1], BOLT_STAGE[2] + 0.045), BOLT_HOLD, "lift the bolt"),
             _wb(1.4, bhole, BOLT_HOLD, "the bolt in the hole"),
             _wb(0.5, bhole, BOLT_HOLD, "settle the bolt in the hole"),
             _wb(0.5, bhole, _OPEN, "release the bolt in the hole"),
             _wb(1.0, (bhole[0], bhole[1], bhole[2] + 0.045), _OPEN, "lift away from the bolt in the hole")]

    # C. pick the screwdriver (upright, tip down) and bring its PZ2 tip onto the cam recess
    left += _pick(SCREW_STAGE, SCREW_HOLD, "screwdriver")
    screw_cz = DRIVE_POINT[2] - SCREW_TIP_LOCAL[2]      # centre height that puts the tip on the cam top
    left += [_hover((DRIVE_POINT[0], DRIVE_POINT[1], screw_cz + 0.03), SCREW_HOLD,
                    "screwdriver over the cam"),
             _w(1.2, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz), SCREW_HOLD,
                "engage the cam recess")]

    # D. turn the cam clockwise (world -z yaw) to draw the panels flush.  The wrist cannot yaw a full 180 deg at
    # this height, so the turn is done the way a person does it: turn to 150 deg, lift, straighten the wrist,
    # re-engage the recess and turn the last 30 deg.  The cam keeps its accumulated turn across the regrip.
    R150 = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.radians(150.0))
    turns = []
    for k in range(1, 6):
        turns.append(_w(1.0, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz), SCREW_HOLD,
                        f"turn cam {k * 30} deg", axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.radians(k * 30))))
    left += turns
    left += [_w(0.8, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz + 0.03), SCREW_HOLD,
                "regrip: lift screwdriver", R150),
             _w(1.4, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz + 0.03), SCREW_HOLD,
                "regrip: straighten wrist", np.eye(3)),
             _w(1.0, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz), SCREW_HOLD, "regrip: re-engage the recess"),
             _w(1.0, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz), SCREW_HOLD, "turn cam 180 deg",
                axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.radians(30.0))),
             _w(0.6, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz), SCREW_HOLD, "hold the flush joint",
                axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.radians(30.0)))]

    # E. lift the screwdriver away, straighten the wrist, set it back in its holder, go home
    left += [_w(0.8, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz + 0.03), SCREW_HOLD, "lift screwdriver away",
                axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.radians(30.0))),
             _w(1.2, (DRIVE_POINT[0], DRIVE_POINT[1], screw_cz + 0.03), SCREW_HOLD, "reset wrist for the holder",
                np.eye(3))]
    left += _place(SCREW_STAGE, SCREW_HOLD, "the screwdriver in its holder")
    left += [Waypoint(2.2, tuple(_START_TCP["left"]), _OPEN, "home", rot=_START_ROT["left"])]
    right = [Waypoint(2.2, tuple(_START_TCP["right"]), _OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"turn {math.degrees(objects.get('turn', 0.0)):+.0f} gap {objects.get('gap', 0.0) * 1000:.1f}mm, "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('cap_cam','cap_bolt','cap_panel') if objects.get(k)) or '-'}")
