"""On the station table: a real 100 mm steel angle-bracket gusset staged in a tray, a bench fixture with a locator
nest (back wall, end stop and two pins) and two hold-down toggle clamps standing open.  The robot seats the gusset
on its locators square, then flips both toggle-clamp levers over-centre (4-7 N each) so each jaw presses the gusset
foot down, and leaves the gusset clamped square.  The fixture is one welded visual-only body; the gusset and the two
clamp levers are real, gravity-free rigid bodies posed by a declared carry, exactly like the family's detents.

Run it:  inspect-robots run --embodiment yam_arena -E task=toggle_clamp_gusset.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../tasks/toggle-clamp-gusset
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

TAG = "tcg"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- fixture / gusset / clamps (local frame: fixture centre x,y; bench top z 0; world z = TOP + local z) --------------
FX, FY = 0.280, 0.000               # locator nest centre (YAM reach gate: h 0.15, reach 0.40, 100 mm gusset)
FIXTURE_POS = (FX, FY, TOP)
FIX_TOP = 0.045                     # fixture top / gusset foot bottom (local z)
GUSSET_STAGE = (FX, FY + 0.100)     # staging tray centre
GUSSET_STAGE_CZ = FIX_TOP           # gusset body origin (inner corner) height in the tray
GUSSET_SEAT_CZ = FIX_TOP            # ... and at the locator nest
SEAT_XY_TOL = 0.007                 # 7 mm seating tolerance
SQUARE_MIN = 0.90                   # foot along x and upright along z (cos > 0.90)

CLAMP_X = (FX - 0.035, FX + 0.035)  # clamp A / B handle-grip x (70 mm apart: open levers do not touch)
CLAMP_Y = FY - 0.037                # handle-grip y (jaw swings onto the foot at y ~ +0.005 when flipped)
CLAMP_CZ = TOP + 0.080              # handle-grip world z
CLAMP_POS_OK = 0.012                # grip must stay at its pivot for the over-centre latch
CLAMP_XY_TOL = 0.012

# -- the declared over-centre flip (a 4-7 N press over the toggle point; yaw of the lever about its pivot) ------------
OVER_YAW = math.radians(70.0)       # the lever yaw that counts as flipped over-centre
OVER_TOL = math.radians(15.0)       # +/- window
RELEASE_MIN = 0.25 * OVER_YAW       # a clamp released between this and over-centre = "not over-centre" cap

# -- gripper / grasp ---------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0
GRIP_CLOSE = 0.034
GRIP_RELEASE = 0.045
GRIP_REACH = 0.016
GRIP_CLEAR = 0.045
HOME_SLACK = 0.020

HOLD = {"gusset": -0.007, "clamp_a": -0.008, "clamp_b": -0.008}
GRASP_UP = 0.012               # clamp levers: the pads close just above the small handle
GUSSET_GRASP_UP = 0.112        # gusset: grasp the top edge of the 100 mm upright, clear of the foot/web
PARTS = ("gusset", "clamp_a", "clamp_b")
CLAMP_OF = {"clamp_a": CLAMP_X[0], "clamp_b": CLAMP_X[1]}

START_POS = {
    "gusset": np.array([GUSSET_STAGE[0], GUSSET_STAGE[1], TOP + GUSSET_STAGE_CZ]),
    "clamp_a": np.array([CLAMP_X[0], CLAMP_Y, CLAMP_CZ]),
    "clamp_b": np.array([CLAMP_X[1], CLAMP_Y, CLAMP_CZ]),
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


FIXTURE = UsdPart("fixture", asset="fixture", xy=(FX, FY), extent=0.24, mass=None, fixed=True,
                  usd_path=str(ASSETS / "fixture.usda"), pos=FIXTURE_POS)
GUSSET = UsdPart("gusset", asset="gusset", xy=GUSSET_STAGE, extent=0.12, mass=0.210,
                 usd_path=str(ASSETS / "gusset.usda"), pos=tuple(START_POS["gusset"]))
CLAMPA = UsdPart("clamp_a", asset="clamp", xy=(CLAMP_X[0], CLAMP_Y), extent=0.06, mass=0.090,
                 usd_path=str(ASSETS / "clamp.usda"), pos=tuple(START_POS["clamp_a"]))
CLAMPB = UsdPart("clamp_b", asset="clamp", xy=(CLAMP_X[1], CLAMP_Y), extent=0.06, mass=0.090,
                 usd_path=str(ASSETS / "clamp.usda"), pos=tuple(START_POS["clamp_b"]))
OBJECTS = [FIXTURE, GUSSET, CLAMPA, CLAMPB]
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
    if getattr(scene.task_object, "_tcg_usd_parts", False):
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
            env_cfg.events.tcg_reset = EventTermCfg(func=_reset_fixture, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("fixture")})
            env_cfg.events.tcg_step = EventTermCfg(func=_gusset_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("fixture")})
            return env_cfg

        return with_events

    task_object._tcg_usd_parts = True
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


def _yaw(mat):
    return math.atan2(float(mat[1, 0]), float(mat[0, 0]))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class _Mechanism:
    def __init__(self):
        self.pos = {p: START_POS[p].copy() for p in PARTS}
        self.quat = {p: START_QUAT[p].copy() for p in PARTS}
        self.held = {p: None for p in PARTS}
        self.offset = {}
        self.rot_off = {}
        self.twist0 = {}
        self.twist = {p: 0.0 for p in PARTS}
        # progress latches (a milestone, once reached, stays reached, like the family's detents)
        self.gusset_seated = False
        self.clamp_a_done = False
        self.clamp_b_done = False
        # caps
        self.gusset_shift = False
        self.clamp_not_over = False
        self.dropped = False
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
    for p in PARTS:
        _write_part(env, p, START_POS[p], START_QUAT[p], env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: gusset in staging tray, both toggle clamps open, locator nest bare", flush=True)


def _grasp_point(name):
    m = _MECH
    up = GUSSET_GRASP_UP if name == "gusset" else GRASP_UP
    return m.pos[name] + _rot(m.quat[name]) @ np.array([0.0, 0.0, up])


def _in_allowed_zone(name, pos):
    x, y, z = pos
    if z < TOP - 0.01 or x < 0.10 or x > 0.42 or abs(y) > 0.18:
        return False
    return True


def _gusset_at_seat(m):
    gq = _rot(m.quat["gusset"])
    square = float(gq[2, 2]) > SQUARE_MIN and float(gq[0, 0]) > SQUARE_MIN
    gp = m.pos["gusset"]
    at = (float(np.linalg.norm(gp[:2] - np.array([FX, FY]))) < SEAT_XY_TOL
          and abs(gp[2] - (TOP + GUSSET_SEAT_CZ)) < SEAT_XY_TOL)
    return bool(at and square)


def _gusset_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)

    # 1. grasp / carry / release each part
    sides = ("left", "right")
    for name in PARTS:
        gp = _grasp_point(name)
        if m.held[name] is None:
            for side in sides:
                g = grips[f"{side}_gripper"]
                if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - gp)) <= GRIP_REACH:
                    if any(m.held[q] == side for q in PARTS):
                        continue
                    m.held[name] = side
                    m.offset[name] = g.rot.T @ (m.pos[name] - g.tcp)
                    m.rot_off[name] = g.rot.T @ _rot(m.quat[name])
                    m.twist0[name] = _yaw(g.rot)
                    break
        else:
            g = grips[f"{m.held[name]}_gripper"]
            if g.opening > GRIP_RELEASE:
                m.held[name] = None
                if not _in_allowed_zone(name, m.pos[name]):
                    m.dropped = True
                if name in CLAMP_OF:
                    tw = _wrap(m.twist[name])
                    if RELEASE_MIN < tw < OVER_YAW - OVER_TOL:
                        m.clamp_not_over = True
            else:
                m.pos[name] = g.tcp + g.rot @ m.offset[name]
                m.quat[name] = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off[name])
                m.twist[name] = _wrap(_yaw(g.rot) - m.twist0[name])

    # 2. static mechanism measurements
    if _gusset_at_seat(m):
        m.gusset_seated = True

    for name in ("clamp_a", "clamp_b"):
        over = _wrap(m.twist[name]) >= OVER_YAW - OVER_TOL
        cp = m.pos[name]
        at_pivot = (float(np.linalg.norm(cp[:2] - np.array([CLAMP_OF[name], CLAMP_Y]))) < CLAMP_XY_TOL
                    and abs(cp[2] - CLAMP_CZ) < CLAMP_POS_OK)
        if over and at_pivot:
            if m.gusset_seated:
                if name == "clamp_a":
                    m.clamp_a_done = True
                else:
                    m.clamp_b_done = True
            else:
                # flipping a clamp onto an unseated (or crooked) gusset knocks it off its locators
                m.gusset_shift = True

    # 3. write the parts (their poses are owned by this declared mechanism, not by the physics solver)
    for p in PARTS:
        _write_part(env, p, m.pos[p], m.quat[p], env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s gusset {'SEAT' if m.gusset_seated else '---'} "
              f"at {np.round(m.pos['gusset'],4)}; "
              f"clamp A {math.degrees(m.twist['clamp_a']):+5.0f}deg {'LOCK' if m.clamp_a_done else '---'}; "
              f"clamp B {math.degrees(m.twist['clamp_b']):+5.0f}deg {'LOCK' if m.clamp_b_done else '---'}"
              f"{'  SHIFT' if m.gusset_shift else ''}{'  NOT-OVER' if m.clamp_not_over else ''}"
              f"{'  DROP' if m.dropped else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    for p in PARTS:
        out[p] = ObjectState(pos=m.pos[p].copy(), quat_xyzw=m.quat[p].copy(), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=m.pos[p].copy(), high=m.pos[p].copy())
    out["gusset_seated"] = m.gusset_seated
    out["clamp_a_done"] = m.clamp_a_done
    out["clamp_b_done"] = m.clamp_b_done
    out["gusset_shift"] = m.gusset_shift
    out["clamp_not_over"] = m.clamp_not_over
    out["dropped"] = m.dropped
    out["twist"] = dict(m.twist)
    out["held"] = dict(m.held)


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("gusset seated square", "clamp A over-centre", "clamp B over-centre", "clamped square + hold")
POINTS = 0.25


def _mech_all(objects) -> bool:
    return bool(objects.get("gusset_seated") and objects.get("clamp_a_done") and objects.get("clamp_b_done"))


def _held() -> bool:
    return _HOLD["t"] is not None and _HOLD["t"] >= 1.0 - 1e-9


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("gusset_seated"))
    s1 = s0 and bool(objects.get("clamp_a_done"))
    s2 = s1 and bool(objects.get("clamp_b_done"))
    s3 = s2 and _held()   # the fourth step is the timed 1 s hold, not another mechanism latch
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("gusset_shift") or objects.get("clamp_not_over") or objects.get("dropped"))


def _current_ok(objects) -> bool:
    """The clamped assembly is intact as things stand: gusset still seated square, both clamps still over-centre."""
    if not (objects.get("gusset_seated") and objects.get("clamp_a_done") and objects.get("clamp_b_done")):
        return False
    if _caps(objects):
        return False
    gp = objects.get("gusset")
    if gp is not None:
        pos = np.asarray(getattr(gp, "pos", gp), float)
        if (float(np.linalg.norm(pos[:2] - np.array([FX, FY]))) >= SEAT_XY_TOL
                or abs(float(pos[2]) - (TOP + GUSSET_SEAT_CZ)) >= SEAT_XY_TOL):
            return False
        quat = getattr(gp, "quat_xyzw", None)
        if quat is not None:
            gr = _rot(quat)
            if float(gr[2, 2]) <= SQUARE_MIN or float(gr[0, 0]) <= SQUARE_MIN:
                return False
    tw = objects.get("twist", {})
    for c in ("clamp_a", "clamp_b"):
        if _wrap(float(tw.get(c, 0.0))) < OVER_YAW - OVER_TOL:
            return False
    return True


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([FX, FY, TOP + 0.115])
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
    """The gusset is clamped square, intact, and both grippers are clear and home (the precondition for the hold)."""
    return bool(_mech_all(objects) and _current_ok(objects) and _clear(objects)
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
    if _full(objects) and _held():
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
_FLIP = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), OVER_YAW) @ _DOWN   # 70 deg yaw = the over-centre flip
_OPEN = FINGER_OPEN


def _w(dur, x, y, z, finger, label, rot=_DOWN):
    """Waypoint from local z (world z = TOP + z)."""
    return Waypoint(dur, (x, y, TOP + z), finger, label, rot=rot)


def _hover(x, y, z, finger, label, rot=_DOWN):
    return _w(1.6, x, y, z, finger, label, rot)


def _pick(centre, cz, hold, label, rot=_DOWN, lift=0.045, grasp_up=GRASP_UP):
    x, y = centre
    gz = cz + grasp_up
    return [_hover(x, y, gz + lift, _OPEN, "over the " + label, rot),
            _w(1.2, x, y, gz, _OPEN, "down to the " + label, rot),
            _w(0.5, x, y, gz, _OPEN, "settle at the " + label, rot),
            _w(0.7, x, y, gz, hold, "grip the " + label, rot),
            _w(1.4, x, y, gz + lift, hold, "lift the " + label, rot)]


def _place(centre, cz, label, rot=_DOWN, lift=0.045, grasp_up=GRASP_UP):
    x, y = centre
    gz = cz + grasp_up
    hold = HOLD.get(label, -0.007)
    return [_hover(x, y, gz + lift, hold, "carry the " + label, rot),
            _w(0.5, x, y, gz + lift, hold, "steady over the " + label, rot),
            _w(1.4, x, y, gz, hold, "lower the " + label, rot),
            _w(0.6, x, y, gz, _OPEN, "release the " + label, rot),
            _w(0.8, x, y, gz + lift, _OPEN, "lift away from the " + label, rot)]


def demo():
    """Left arm seats the gusset square on the locators; the right arm then picks each toggle-clamp lever at its open
    pivot and flips it over-centre (a 70 deg yaw about the pivot, 8.6 deg past the 55 deg threshold).  The right arm
    waits at home until the gusset is seated.  Every approach is from above with the fingers closing on the grasp
    point: the gusset by the top edge of its upright, the clamps by their handles."""
    left = []
    left += _pick(GUSSET_STAGE, GUSSET_STAGE_CZ, HOLD["gusset"], "gusset", lift=0.010,
                  grasp_up=GUSSET_GRASP_UP)
    left += _place((FX, FY), GUSSET_SEAT_CZ, "gusset", lift=0.010, grasp_up=GUSSET_GRASP_UP)
    left += [Waypoint(2.2, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]

    # right arm: wait for the gusset to be seated, then throw both clamps
    right = [Waypoint(10.0, tuple(_START_TCP["right"]), FINGER_OPEN, "wait for gusset", rot=_START_ROT["right"])]
    for name, cx in (("clamp A", CLAMP_X[0]), ("clamp B", CLAMP_X[1])):
        key = "clamp_a" if name == "clamp A" else "clamp_b"
        gz = CLAMP_CZ - TOP + GRASP_UP
        right += _pick((cx, CLAMP_Y), CLAMP_CZ - TOP, HOLD[key], name, lift=0.030)
        right += [_w(1.4, cx, CLAMP_Y, gz, HOLD[key], "flip " + name + " over-centre", _FLIP),
                  _w(0.8, cx, CLAMP_Y, gz, _OPEN, "release " + name, _FLIP),
                  _w(0.8, cx, CLAMP_Y, gz + 0.030, _OPEN, "lift away from " + name, _FLIP)]
    right += [Waypoint(2.2, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    tw = objects.get("twist", {})
    return (f"gusset {'SEAT' if objects.get('gusset_seated') else '---'} "
            f"A {math.degrees(tw.get('clamp_a', 0)):+.0f} B {math.degrees(tw.get('clamp_b', 0)):+.0f} deg, "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('gusset_shift','clamp_not_over','dropped') if objects.get(k)) or '-'}")
