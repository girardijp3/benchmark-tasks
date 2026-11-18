"""On the station bench: a rotary fixture plate locked by a spring-loaded hand index plunger.  The plate has a ring
of index holes in its rim; the plunger's hardened pin sits in the hole at the home stop.  To move to the next stop
the robot must (1) pull the 16 mm knob straight out against the 4-7 N spring to retract the pin, (2) keep it pulled
and rotate the plate one hole pitch (45 degrees) about its central hub, (3) release the knob so the spring drives the
pin into the next hole, and (4) leave it indexed with both grippers home.  Turning the plate while the pin is still
engaged (a forced pull), turning more than one pitch (overshoot), or releasing the pin between holes (missed hole)
are capped failures.

The plate and plunger are real gravity-free rigid bodies posed by this task file from YAM gripper measurements:
the plunger translates axially with the grip that holds its knob; the plate's yaw follows the wrist yaw of the grip
that holds its central hub.  The robot must still reach, close on, pull, turn and release both.  The scorer reads the
mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=index_plunger_fixture.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/index-plunger-fixture
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

TAG = "ipf"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"
IDENT = np.array([0.0, 0.0, 0.0, 1.0])

# -- fixture / plate local frame (centre at x,y = 0; bench top z 0; world z = TOP + local z) ----------------------------
CXY = (0.250, 0.000)
BASE_POS = (CXY[0], CXY[1], TOP)
PLATE_CZ = 0.110                # plate disc centre local z
PLATE_R = 0.075                 # plate disc radius (150 mm table)
PLATE_H = 0.020                 # plate thickness
N_HOLES = 8                     # index holes in the rim
PITCH = 2.0 * math.pi / N_HOLES  # 45 degrees between stops
HOLE_R = 0.0035                 # index hole / pin radius (7 mm)
HOLE_RING = 0.072               # hole centre radius (opens on the plate rim)
HUB_R = 0.012                   # central hub knob radius (24 mm)
HUB_CZ = 0.130                  # hub centre local z (0.120 .. 0.140)

# -- index plunger: axis along +y at x 0.25, local z 0.110 --------------------------------------------------------------
PLUNGER_Z = 0.110               # plunger axis local z (world 0.8575)
KNOB_R = 0.008                  # 16 mm knob diameter
KNOB_SEATED_Y = 0.150           # knob centre global y with the pin in the hole
PIN_LEN = 0.072                 # pin from the knob's inner face to the plate rim (tip seated at y 0.070)
PULL_MAX = 0.016                # full retraction (16 mm): pin tip clears the rim
PULL_RETRACT = 0.012            # pull at/above which the pin is clear of the plate rim
SEAT_TOL = 0.0025               # pull at/below which the pin is back in the hole
ROT_TOL = math.radians(8.0)     # plate angle tolerance at the next stop
ALIGN_TOL = math.radians(8.0)   # hole alignment tolerance at release
OVER_TOL = math.radians(12.0)   # plate beyond one pitch + this = overshoot

# -- gripper / grasp -----------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0                # N: a real closed grip
GRIP_CLOSE = 0.034              # opening at/below which a closed finger pair may take a part
GRIP_RELEASE = 0.045            # opening above which a held part is let go
GRIP_REACH = 0.016              # m: TCP within this of the grasp point
GRIP_CLEAR = 0.045              # m: no pad within this of the fixture at the scored state
HOME_SLACK = 0.020
GRASP_UP = 0.012                # TCP sits this far above a grasp point
HOLD_KNOB = -0.008              # finger target across the 16 mm knob (opening 0.016)
HOLD_HUB = -0.012               # finger target across the 24 mm hub (opening 0.024)
KNOB_GRIP_ROT = yam_demo.grip_rotation(math.pi / 4.0)  # wrist angle that reaches cleanly across the knob


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


BASE = UsdPart("base", asset="base", xy=CXY, extent=0.16, mass=None, fixed=True,
               usd_path=str(ASSETS / "base.usda"), pos=BASE_POS)
PLATE = UsdPart("plate", asset="plate", xy=CXY, extent=0.14, mass=0.600,
                usd_path=str(ASSETS / "plate.usda"),
                pos=(CXY[0], CXY[1], TOP + PLATE_CZ))
PLUNGER = UsdPart("plunger", asset="plunger", xy=(CXY[0], KNOB_SEATED_Y), extent=0.05, mass=0.120,
                  usd_path=str(ASSETS / "plunger.usda"),
                  pos=(CXY[0], KNOB_SEATED_Y, TOP + PLUNGER_Z))
OBJECTS = [BASE, PLATE, PLUNGER]
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
    if getattr(scene.task_object, "_ipf_usd_parts", False):
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
            env_cfg.events.ipf_reset = EventTermCfg(func=_reset_fixture, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("base")})
            env_cfg.events.ipf_step = EventTermCfg(func=_fixture_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("base")})
            return env_cfg

        return with_events

    task_object._ipf_usd_parts = True
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


def _yaw(mat):
    return math.atan2(float(mat[1, 0]), float(mat[0, 0]))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _hole_error(yaw):
    """Signed angular error from the nearest of the 8 stop angles (a hole at the plunger)."""
    return (yaw + PITCH / 2.0) % PITCH - PITCH / 2.0


class _Mechanism:
    def __init__(self):
        self.plunger_pos = np.array([CXY[0], KNOB_SEATED_Y, TOP + PLUNGER_Z])
        self.plunger_held = None            # side holding the knob, or None
        self.plunger_grab_tcp = None
        self.pull = 0.0                     # axial retraction (m)
        self.prev_pull = 0.0
        self.plate_held = None              # side holding the hub, or None
        self.plate_yaw0 = 0.0
        self.plate_yaw = 0.0                # radians from the home stop
        # milestones (once reached, stay reached, like the family's detents)
        self.pulled = False
        self.rotated = False
        self.pin_seated = False
        self.indexed = False
        # caps
        self.overshoot = False
        self.miss_hole = False
        self.forced = False
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


def _reset_fixture(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_part(env, "base", np.array(BASE_POS), IDENT, env_ids)
    _write_part(env, "plate", np.array([CXY[0], CXY[1], TOP + PLATE_CZ]), IDENT, env_ids)
    _write_part(env, "plunger", _MECH.plunger_pos, IDENT, env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: pin in the home hole, plate at 0 deg, both arms at READY", flush=True)


def _plate_grasp_point():
    return np.array([CXY[0], CXY[1], TOP + HUB_CZ + GRASP_UP])


def _knob_grasp_point(name_pos):
    return name_pos + np.array([0.0, 0.0, GRASP_UP])


def _fixture_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)
    sides = ("left", "right")

    # 1. the plunger: grip its knob and slide it axially (+y); spring returns it on release.
    kgp = _knob_grasp_point(m.plunger_pos)
    if m.plunger_held is None:
        for side in sides:
            g = grips[f"{side}_gripper"]
            if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - kgp)) <= GRIP_REACH:
                if m.plate_held == side:
                    continue
                m.plunger_held = side
                m.plunger_grab_tcp = g.tcp.copy()
                break
    else:
        g = grips[f"{m.plunger_held}_gripper"]
        if g.opening > GRIP_RELEASE:
            m.plunger_held = None
            m.pull = 0.0                    # the 4-7 N spring seats the pin
        else:
            m.pull = float(np.clip((g.tcp - m.plunger_grab_tcp)[1], -0.002, PULL_MAX))
    m.plunger_pos = np.array([CXY[0], KNOB_SEATED_Y + m.pull, TOP + PLUNGER_Z])

    # 2. the plate: grip the central hub and turn it; its yaw follows the wrist yaw while held.
    pgp = _plate_grasp_point()
    if m.plate_held is None:
        for side in sides:
            g = grips[f"{side}_gripper"]
            if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - pgp)) <= GRIP_REACH:
                if m.plunger_held == side:
                    continue
                m.plate_held = side
                m.plate_yaw0 = _yaw(g.rot)
                break
    else:
        g = grips[f"{m.plate_held}_gripper"]
        if g.opening > GRIP_RELEASE:
            m.plate_held = None
        else:
            m.plate_yaw = _wrap(_yaw(g.rot) - m.plate_yaw0)

    # 3. caps.
    if (m.plate_yaw - PITCH) > OVER_TOL or m.plate_yaw < -OVER_TOL:
        m.overshoot = True
    if m.plate_held is not None and abs(m.plate_yaw) > ROT_TOL and m.pull < PULL_RETRACT:
        m.forced = True
    released = m.prev_pull > SEAT_TOL and m.pull <= SEAT_TOL
    if released and m.pulled and abs(_hole_error(m.plate_yaw)) > ALIGN_TOL:
        m.miss_hole = True
    m.prev_pull = m.pull

    # 4. milestones.
    if m.pull >= PULL_RETRACT:
        m.pulled = True
    if m.pulled and abs(m.plate_yaw - PITCH) <= ROT_TOL:
        m.rotated = True
    if m.rotated and m.pull <= SEAT_TOL and abs(_hole_error(m.plate_yaw)) <= ALIGN_TOL:
        m.pin_seated = True
    if m.pin_seated and not _caps_state() and abs(m.plate_yaw - PITCH) <= ROT_TOL and m.pull <= SEAT_TOL:
        m.indexed = True

    # 5. write the two moving bodies.
    plate_quat = ik.matrix_to_quat_xyzw(axis_angle_matrix(np.array([0.0, 0.0, 1.0]), m.plate_yaw))
    _write_part(env, "plate", np.array([CXY[0], CXY[1], TOP + PLATE_CZ]), plate_quat, env_ids)
    _write_part(env, "plunger", m.plunger_pos, IDENT, env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s pull {m.pull*1000:4.1f} mm "
              f"plate {math.degrees(m.plate_yaw):+6.1f} deg (hole err {math.degrees(_hole_error(m.plate_yaw)):+5.1f}) "
              f"steps {int(m.pulled)}{int(m.rotated)}{int(m.pin_seated)}{int(m.indexed)}"
              f"{'  OVER' if m.overshoot else ''}{'  MISS' if m.miss_hole else ''}"
              f"{'  FORCED' if m.forced else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    out["plate"] = ObjectState(pos=np.array([CXY[0], CXY[1], TOP + PLATE_CZ]), quat_xyzw=IDENT,
                               lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                               low=np.array([CXY[0], CXY[1], TOP + PLATE_CZ]),
                               high=np.array([CXY[0], CXY[1], TOP + PLATE_CZ]))
    out["plunger"] = ObjectState(pos=m.plunger_pos.copy(), quat_xyzw=IDENT.copy(), lin_vel=np.zeros(3),
                                 ang_vel=np.zeros(3), low=m.plunger_pos.copy(), high=m.plunger_pos.copy())
    out["pull"] = m.pull
    out["plate_yaw"] = m.plate_yaw
    out["hole_error"] = _hole_error(m.plate_yaw)
    out["pulled"] = m.pulled
    out["rotated"] = m.rotated
    out["pin_seated"] = m.pin_seated
    out["indexed"] = m.indexed
    out["overshoot"] = m.overshoot
    out["miss_hole"] = m.miss_hole
    out["forced"] = m.forced
    out["plunger_held"] = m.plunger_held
    out["plate_held"] = m.plate_held


# -- scoring -------------------------------------------------------------------------------------------------------------
STEPS = ("plunger pulled", "plate at next stop", "pin released into hole", "left indexed + hold")
POINTS = 0.25


def _caps_state():
    m = _MECH
    return m.overshoot or m.miss_hole or m.forced


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("pulled"))
    s1 = s0 and bool(objects.get("rotated"))
    s2 = s1 and bool(objects.get("pin_seated"))
    s3 = s2 and bool(objects.get("indexed"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("overshoot") or objects.get("miss_hole") or objects.get("forced"))


def _current_ok(objects) -> bool:
    """The fixture is indexed as things stand: next stop, pin seated, no damage latch."""
    return bool(abs(objects.get("plate_yaw", 0.0) - PITCH) <= ROT_TOL
                and objects.get("pull", PULL_MAX) <= SEAT_TOL
                and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    for p in (np.array([CXY[0], CXY[1], TOP + PLATE_CZ]),
              np.array([CXY[0], KNOB_SEATED_Y, TOP + PLUNGER_Z])):
        for side in ("left", "right"):
            g = objects.get(f"{side}_gripper")
            if g is None:
                continue
            if g.opening < GRIP_RELEASE or g.contact >= GRIP_FORCE:
                return False
            for pt in _pad_points(g):
                if float(np.linalg.norm(pt - p)) < GRIP_CLEAR:
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
    pts = POINTS * sum(steps_done(objects))
    if _caps(objects):
        pts = min(pts, 2 * POINTS)
    if _full(objects) and _HOLD["t"] is not None and _HOLD["t"] >= 1.0 - 1e-9:
        return 1.0
    return round(pts, 6)


# -- demo -----------------------------------------------------------------------------------------------------------------
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
_TURN = yam_demo.grip_rotation(PITCH)         # one hole pitch, 45 degrees
_OPEN = FINGER_OPEN
PULL_TCP_Y = KNOB_SEATED_Y + PULL_MAX


def _w(dur, x, y, z, finger, label, rot=_DOWN):
    return Waypoint(dur, (x, y, TOP + z), finger, label, rot=rot)


def _hover(x, y, z, finger, label, rot=_DOWN):
    return _w(1.5, x, y, z, finger, label, rot)


def demo():
    """The left arm pulls and releases the plunger knob; the right arm turns the plate one pitch at its hub.
    Open loop, known geometry.  Left: grip the knob, pull 14 mm, HOLD while the right arm turns, then return the pin
    and open.  Right: grip the hub, wait for the pull, rotate the wrist 45 deg, open and home.  Every approach is from
    above; the plate is still held by nobody when the pin is released, so no forced/overshoot/miss cap is triggered."""
    kz = PLUNGER_Z + GRASP_UP
    hz = HUB_CZ + GRASP_UP

    # LEFT: grip the plunger knob and pull it clear, then HOLD while the plate turns.
    left = [
        _hover(CXY[0], KNOB_SEATED_Y, kz + 0.045, _OPEN, "over the knob", KNOB_GRIP_ROT),
        _w(1.3, CXY[0], KNOB_SEATED_Y, kz, _OPEN, "down to the knob", KNOB_GRIP_ROT),
        _w(0.5, CXY[0], KNOB_SEATED_Y, kz, _OPEN, "settle at the knob", KNOB_GRIP_ROT),
        _w(0.7, CXY[0], KNOB_SEATED_Y, kz, HOLD_KNOB, "grip the knob", KNOB_GRIP_ROT),
        _w(1.5, CXY[0], PULL_TCP_Y, kz, HOLD_KNOB, "pull the plunger out", KNOB_GRIP_ROT),
        _w(6.0, CXY[0], PULL_TCP_Y, kz, HOLD_KNOB, "hold plunger clear", KNOB_GRIP_ROT),
        # right arm has turned the plate and let go by now; return the pin to the next hole
        _w(1.2, CXY[0], KNOB_SEATED_Y, kz, HOLD_KNOB, "return the pin to the hole", KNOB_GRIP_ROT),
        _w(0.6, CXY[0], KNOB_SEATED_Y, kz, _OPEN, "release the plunger", KNOB_GRIP_ROT),
        _w(0.8, CXY[0], KNOB_SEATED_Y, kz + 0.045, _OPEN, "lift away from the knob", KNOB_GRIP_ROT),
        Waypoint(2.0, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]

    # RIGHT: grip the hub, wait for the pull, rotate one pitch, open and home.
    right = [
        _w(1.5, CXY[0], CXY[1], hz + 0.006, _OPEN, "over the hub"),
        _w(1.3, CXY[0], CXY[1], hz, _OPEN, "down to the hub"),
        _w(0.5, CXY[0], CXY[1], hz, _OPEN, "settle at the hub"),
        _w(0.7, CXY[0], CXY[1], hz, HOLD_HUB, "grip the hub"),
        _w(2.0, CXY[0], CXY[1], hz, HOLD_HUB, "wait for the pull"),
        _w(2.5, CXY[0], CXY[1], hz, HOLD_HUB, "rotate plate one stop", _TURN),
        _w(0.8, CXY[0], CXY[1], hz, HOLD_HUB, "settle at the stop", _TURN),
        _w(0.6, CXY[0], CXY[1], hz, _OPEN, "release the hub", _TURN),
        _w(0.8, CXY[0], CXY[1], hz + 0.006, _OPEN, "lift away from the hub", _TURN),
        Waypoint(2.0, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"pull {objects.get('pull', 0.0)*1000:+.1f} mm, "
            f"plate {math.degrees(objects.get('plate_yaw', 0.0)):+5.1f} deg, "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('overshoot','miss_hole','forced') if objects.get(k)) or '-'}")
