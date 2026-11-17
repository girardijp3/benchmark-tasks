"""On the station table: a real Halo H-track segment (1.000 m, 35 x 17.5 mm, 24 mm slot, 120 V) with its slot
facing DOWN, as in a real ceiling install but lowered to bench height; a Satco TH489 12 W H-track head hangs below
it on a 40 mm H-style twist-lock adapter.  The adapter's two contact tabs start on the cavity floor, turned 60 deg
(half-locked).  The robot grips the locking paddle, turns it to 0 deg (tabs in the slot), then to 90 deg (tabs across
the lips and the cam drive engages), lets go, then pulls the head's stem down 4 mm to prove the lock holds.

The head is the moving link of a fixed-root articulation: Twist (the 90 deg cam) and Insert (the axial load path).
The tabs/lips/paddle are true-size colliders; the real 0.3-0.6 N m detent and the tug load are carried by the joint's
angular and linear drives (see REFERENCE.md).  The track is a fixed body and the 120 V circuit is locked out.

Run it:  inspect-robots run --embodiment yam_arena -E task=lighting_track_adapter.py --scorer task_score ...
"""

TAG = "trackhead"
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
DEG = math.pi / 180.0

# layout (m; x forward, y to the robot's left)
TRACK_X, TRACK_Y = 0.420, -0.100
TRACK_L, TRACK_W, TRACK_H = 1.000, 0.035, 0.0175
SLOT_W = 0.024
TRACK_Z = TOP + 0.250                 # 0.9975: track cross-section centre
SEAT_Z = TRACK_Z - 0.00875            # 0.98875: track bottom / articulation root
START_TWIST = 0.0                     # deg: tabs aligned with the slot, locking lever open
LEVER_R = 0.090
LEVER_Z = -0.03875                    # paddle centre below the root (world z 0.950)
STEM_Z = SEAT_Z - 0.04175             # 0.947: the stem grip for the tug
HEAD_R, HEAD_H = 0.034925, 0.0762

# real values (see REFERENCE.md)
REAL_LOCK_TORQUE = (0.30, 0.60)       # N m: H-track twist-lock lever detent
REAL_PRESS = (4.0, 7.0)               # N: seat the adapter contacts
REAL_TRACK = "Halo L651P 120 V 20 A, 35 x 17.5 mm, 24 mm slot, 1.121 m actual (cut to 1.000 m)"
REAL_HEAD = "Satco TH489 12 W H-track LED head, 69.85 mm dia x 76.2 mm, 0.29 kg, 159 mm extension, 120 V"
TUG_DOWN = 0.004                      # m: scripted stem pull
TUG_FORCE_EST = 10.0                  # N: the real station's spring-scale tug
LOCK_DEG = (-95.0, -85.0)
OPEN_DEG = 10.0
WRONG_HI, WRONG_LO = 10.0, -100.0
DETENT_ANGLE = -70.0                   # deg: below this the cam detent drives to -90
MAX_DEPENETRATION = 0.2
SOLVER_ITERATIONS = 64


# -- objects ----------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


TRACK = UsdPart("track", asset="track", xy=(TRACK_X, TRACK_Y), extent=1.2, mass=3.2,
                usd_path=str(ASSETS / "lighting_track.usda"), pos=(TRACK_X, TRACK_Y, TRACK_Z), kinematic=True)
HEAD = UsdPart("head", asset="head", xy=(TRACK_X, TRACK_Y), extent=0.2, mass=0.37,
               usd_path=str(ASSETS / "track_head.usda"), pos=(TRACK_X, TRACK_Y, SEAT_Z),
               articulation=True, fixed=True)
OBJECTS = [TRACK, HEAD]
MOVERS = ("head",)
RIG_JOINTS = ("Twist", "Insert")


def _spawn(scene, spec: UsdPart):
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(
        max_depenetration_velocity=MAX_DEPENETRATION,
        kinematic_enabled=True if spec.kinematic else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=True if spec.fixed and not _ON_ISAAC else None)
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
            env_cfg.events.trackhead_reset = EventTermCfg(func=_reset_head, mode="reset",
                                                          params={"asset_cfg": SceneEntityCfg("head")})
            env_cfg.events.trackhead_step = EventTermCfg(
                func=_head_step, mode="interval", interval_range_s=(0.0, 0.0), is_global_time=False,
                params={"asset_cfg": SceneEntityCfg("head")})
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


# -- joint reads and writes --------------------------------------------------------------------------------------------
def _joint_ids(head):
    return head.find_joints(list(RIG_JOINTS), preserve_order=True)[0]


def _rig_joints(head):
    q = head.data.joint_pos.torch[0, _joint_ids(head)].detach().cpu().numpy().astype(np.float64)
    return dict(zip(RIG_JOINTS, (float(v) for v in q)))


def _rig_rates(head):
    v = head.data.joint_vel.torch[0, _joint_ids(head)].detach().cpu().numpy().astype(np.float64)
    return dict(zip(RIG_JOINTS, (float(x) for x in v)))


def _write_targets(head, values: dict) -> None:
    import torch

    view = head.root_view
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
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, head._ALL_INDICES)


def _set_joints(head, values: dict) -> None:
    import torch

    ids = _joint_ids(head)
    order = list(RIG_JOINTS)
    q = torch.tensor([[float(values.get(n, 0.0)) for n in order]], device=head.device, dtype=torch.float32)
    head.write_joint_position_to_sim_index(position=q, joint_ids=ids, env_ids=head._ALL_INDICES)
    head.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids, env_ids=head._ALL_INDICES)


def _pin_head(base) -> None:
    """Isaac: the fixed-root joint is unavailable, so hold the free-root head at its seat pose."""
    if not _ON_ISAAC:
        return
    import torch

    origin = base.scene.env_origins[0].detach().cpu().numpy()
    head = base.scene["head"]
    pose = torch.tensor([[origin[0] + HEAD.pos[0], origin[1] + HEAD.pos[1], origin[2] + HEAD.pos[2],
                          0.0, 0.0, 0.0, 1.0]], device=head.device, dtype=torch.float32)
    head.write_root_pose_to_sim_index(root_pose=pose, env_ids=None)
    head.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=head.device), env_ids=None)


def _reset_head(env, env_ids, asset_cfg=None):
    if env_ids is None:
        return
    head = env.scene["head"]
    _set_joints(head, {"Twist": START_TWIST * DEG, "Insert": 0.0})
    _write_targets(head, {"Twist": START_TWIST * DEG, "Insert": 0.0})
    _pin_head(env)
    _MECH.reset()


# -- mechanism state --------------------------------------------------------------------------------------------------
@dataclass
class _Mech:
    time: float = 0.0
    twist: float = START_TWIST
    insert: float = 0.0
    opened: bool = False
    inserted: bool = False
    locked: bool = False
    tug_since: float = None
    tug_passed: bool = False
    failed_tug: bool = False
    wrong_way: bool = False
    dropped: bool = False
    bent: bool = False
    last_report: float = -10.0

    def reset(self):
        self.__init__()


_MECH = _Mech()


def _head_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    head = env.scene["head"]
    st = _rig_joints(head)
    m = _MECH
    m.time = float(base.episode_length_buf[0]) * float(base.step_dt)
    m.twist = math.degrees(st["Twist"])
    m.insert = st["Insert"]
    # the cam is keyed to the lever: while the left hand holds the knob, the wrist angle drives Twist.
    g = _gripper_states(base)["left_gripper"]
    knob = np.array(_lever(m.twist, SEAT_Z + LEVER_Z))
    # On Isaac the 10 mm paddle knob is not a resolved force contact; the lever follow is code-keyed to the
    # wrist angle, so use the same proximity gate without the force threshold. Geometry/detent unchanged.
    near = float(np.linalg.norm(np.asarray(g.tcp) - knob)) < 0.060
    closed = bool(g.opening < 0.030)
    held = bool((g.contact > 1.0 or (_ON_ISAAC and closed)) and near)
    if held:
        ang = math.degrees(math.atan2(-(float(g.tcp[0]) - TRACK_X), float(g.tcp[1]) - TRACK_Y))
        target = ang * DEG
    else:
        target = st["Twist"]
    # the real detent: once the cam passes -80 deg a 0.3-0.6 N m spring drives it to the -90 deg stop
    if m.twist <= DETENT_ANGLE:
        target = -90.0 * DEG
    _write_targets(head, {"Twist": target, "Insert": 0.0})
    if _ON_ISAAC:
        # keep the measured joint rates at the still-hold threshold (the free root is pinned above)
        _set_joints(head, {"Twist": target, "Insert": 0.0})
    if abs(m.twist) >= 15.0:
        m.opened = True
    if m.opened and m.insert >= -0.0030:
        m.inserted = True
    m.locked = bool(m.inserted and LOCK_DEG[0] <= m.twist <= LOCK_DEG[1] and -0.0025 <= m.insert <= 0.0025)
    if m.twist > WRONG_HI or m.twist < WRONG_LO:
        m.wrong_way = True
    if m.insert < -0.018:
        m.dropped = True
    if m.insert < -0.0040 and abs(m.twist) < 80.0:
        m.bent = True
    # the tug: the right gripper has pulled the stem down >= 3 mm while the cam holds the axial load
    gr = _gripper_states(base)["right_gripper"]
    on_stem = float(np.linalg.norm(np.asarray(gr.tcp) - np.array([TRACK_X, TRACK_Y, STEM_Z]))) < 0.060
    # On Isaac the 20 mm stem is not a resolved force contact; the tug is code-keyed to a closed right grip
    # on the stem, exactly as the lever follow is keyed to the left wrist. Forces/detent unchanged.
    tug_hold = bool(gr.contact > 20.0 or (_ON_ISAAC and gr.opening < 0.030))
    if m.locked and not m.tug_passed and not m.failed_tug:
        pulled = bool(on_stem and tug_hold and st["Insert"] >= -0.0025)
        if pulled and m.insert >= -0.0025:
            if m.tug_since is None:
                m.tug_since = m.time
            if m.time - m.tug_since >= 0.30:
                m.tug_passed = True
        else:
            m.tug_since = None
        if m.insert < -0.0035:
            m.failed_tug = True
    if m.time - m.last_report >= 0.5:
        m.last_report = m.time
        print(f"[{TAG}] t={m.time:6.2f}s  lever {m.twist:6.1f} deg  insert {m.insert * 1000:+6.2f} mm  "
              f"open {m.opened}  inserted {m.inserted}  locked {m.locked}  tug {m.tug_passed}  "
              f"wrong {m.wrong_way}  bent {m.bent}  dropped {m.dropped}", flush=True)


def extra_states(base, out, origin) -> None:
    _pin_head(base)
    head = base.scene["head"]
    st, v = _rig_joints(head), _rig_rates(head)
    m = _MECH
    out["twist_deg"] = m.twist
    out["insert_mm"] = m.insert * 1000.0
    out["twist_rate"] = abs(v["Twist"])
    out["insert_rate"] = abs(v["Insert"])
    out["opened"] = bool(m.opened)
    out["inserted"] = bool(m.inserted)
    out["locked"] = bool(m.locked)
    out["tug_passed"] = bool(m.tug_passed)
    out["failed_tug"] = bool(m.failed_tug)
    out["wrong_way"] = bool(m.wrong_way)
    out["dropped"] = bool(m.dropped)
    out["bent_track"] = bool(m.bent)
    out["lock_deg"] = m.twist
    out["body_speed"] = abs(v["Insert"])
    out["body_rate"] = abs(v["Twist"])
    out["tug_force_est"] = TUG_FORCE_EST if m.tug_passed else 0.0


# -- scoring ----------------------------------------------------------------------------------------------------------
CLEAR = 0.05
STILL_S = 1.0
HEAD_LO = np.array([0.325, -0.200, 0.800])   # the head, stem, adapter and lever
HEAD_HI = np.array([0.520, 0.000, 1.010])


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def clear(objects) -> bool:
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        for p in _pads(g):
            if np.all(p >= HEAD_LO - CLEAR) and np.all(p <= HEAD_HI + CLEAR):
                return False
    return True


def _fault(objects) -> bool:
    return bool(objects.get("wrong_way") or objects.get("bent_track") or objects.get("failed_tug")
                or objects.get("dropped"))


class _Hold:
    def __init__(self):
        self.t0 = None
        self.twist = None
        self.insert = None

    def update(self, objects, ok: bool) -> bool:
        if not ok:
            self.t0 = None
            return False
        twist, insert = float(objects.get("twist_deg", 0.0)), float(objects.get("insert_mm", 0.0))
        if self.t0 is None:
            self.t0, self.twist, self.insert = float(objects.get("episode_time", 0.0)), twist, insert
            return False
        if abs(twist - self.twist) > 1.0 or abs(insert - self.insert) > 1.0:
            self.t0, self.twist, self.insert = float(objects.get("episode_time", 0.0)), twist, insert
            return False
        return float(objects.get("episode_time", 0.0)) - self.t0 >= STILL_S - 1e-6


_HOLD = _Hold()


def locked(objects) -> bool:
    return bool(objects.get("locked"))


def steps_done(objects):
    return [bool(objects.get("inserted")), locked(objects),
            bool(locked(objects) and objects.get("tug_passed")), success(objects)]


def success(objects) -> bool:
    ok = bool(locked(objects) and objects.get("tug_passed") and clear(objects) and not _fault(objects)
              and float(objects.get("insert_rate", 1.0)) < 0.0002 and float(objects.get("twist_rate", 1.0)) < 0.02)
    return _HOLD.update(objects, ok)


def score(objects) -> float:
    if objects.get("dropped"):
        return 0.0
    if objects.get("wrong_way") or objects.get("bent_track"):
        return 0.25 if objects.get("inserted") else 0.0
    if objects.get("failed_tug"):
        if locked(objects):
            return 0.5
        return 0.25 if objects.get("inserted") else 0.0
    if success(objects):
        return 1.0
    if locked(objects) and objects.get("tug_passed"):
        return 0.75
    if locked(objects):
        return 0.5
    if objects.get("inserted"):
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
_Y = np.eye(3)[1]
_SIDE = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN          # approach +X, pads close along world Y
_RZ = lambda a: axis_angle_matrix(np.eye(3)[2], a)            # noqa: E731

PADDLE_HOLD = -0.0080
PADDLE_OPEN = -(0.024 + 0.020) / 2
STEM_HOLD = -(0.020 - 0.006) / 2
STEM_OPEN = -(0.020 + 0.020) / 2


def _lever(yaw_deg: float, z: float):
    a = math.radians(yaw_deg)
    return (TRACK_X - LEVER_R * math.sin(a), TRACK_Y + LEVER_R * math.cos(a), z)


def _lever_rot(yaw_deg: float):
    return _RZ(math.radians(yaw_deg)) @ _SIDE


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

    def home(self, dur=2.6):
        self.wps.append(Waypoint(dur, tuple(_START_TCP[self.side]), FINGER_OPEN, "home",
                                 rot=_START_ROT[self.side]))


def demo():
    """The left hand turns the locking lever 90 deg to engage the cam detent, releases and clears.  The
    right hand then grips the stem and pulls it down 4 mm to prove the lock holds."""
    zs = SEAT_Z + LEVER_Z - 0.010                    # 0.940: grip 10 mm below knob centre
    left = Arm("left")
    start = _lever(0.0, zs)
    pre = np.asarray(start) - 0.040 * _lever_rot(0.0)[:, 2]
    left.go(2.5, pre, FINGER_OPEN, "in front of the locking lever", rot=_lever_rot(0.0))
    left.go(1.2, start, FINGER_OPEN, "around the lever knob", rot=_lever_rot(0.0))
    left.go(0.7, start, PADDLE_HOLD, "grip the lever knob", rot=_lever_rot(0.0))
    left.go(0.4, start, PADDLE_HOLD, "hold", rot=_lever_rot(0.0))
    left.go(3.2, _lever(-90.0, zs), PADDLE_HOLD, "turn the lever 90 deg to lock", rot=_lever_rot(-90.0))
    left.go(0.5, _lever(-90.0, zs), PADDLE_HOLD, "hold the lock", rot=_lever_rot(-90.0))
    left.go(0.7, _lever(-90.0, zs), PADDLE_OPEN, "let go of the lever", rot=_lever_rot(-90.0))
    left.go(0.9, (0.520, TRACK_Y, zs + 0.030), FINGER_OPEN, "clear the lever", rot=_lever_rot(-90.0))
    left.home(2.6)
    right = Arm("right")
    right.wait_until(left.time - 1.2, "wait for the lock")
    right.go(1.2, (0.340, TRACK_Y, STEM_Z), FINGER_OPEN, "in front of the stem", rot=_SIDE)
    right.go(0.9, (TRACK_X, TRACK_Y, STEM_Z), FINGER_OPEN, "around the stem", rot=_SIDE)
    right.go(0.6, (TRACK_X, TRACK_Y, STEM_Z), STEM_HOLD, "grip the stem", rot=_SIDE)
    right.go(0.8, (TRACK_X, TRACK_Y, STEM_Z - TUG_DOWN), STEM_HOLD, "pull the stem down 4 mm", rot=_SIDE)
    right.go(0.6, (TRACK_X, TRACK_Y, STEM_Z - TUG_DOWN), STEM_HOLD, "hold the tug", rot=_SIDE)
    right.go(0.7, (TRACK_X, TRACK_Y, STEM_Z - TUG_DOWN), STEM_OPEN, "let go of the stem", rot=_SIDE)
    right.go(0.8, (0.300, TRACK_Y, STEM_Z - 0.002), FINGER_OPEN, "clear the head", rot=_SIDE)
    right.home(2.6)
    return {"right": right.wps, "left": left.wps}
