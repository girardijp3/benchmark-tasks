"""On the station table: a 20 mm vial adapter on a vial held in a small bench fixture, its female luer-lock port
upright, and a 10 mL luer-lock syringe staged beside it.  The robot picks the syringe up by its 20 mm barrel, seats
its 6 mm male luer tip in the adapter's female port, then twists the collar a cumulative ~1.5 turns (0.3-0.6 N m at
the real bench) until the luer lock is seated: no cross-thread (turning before the tip is seated) and no loose
connection (releasing before the lock is made).  It lets go and goes home.

The syringe is a real, gravity-free rigid body in its own USD file.  While a gripper is closed on the barrel the
syringe follows the gripper (a declared carry, like the family's declared latch detents); the robot must still reach,
close on, carry, seat and twist it.  A wrist yaw cannot turn a full 1.5 revolutions in one grip, so the demo regrips
twice and the task file accumulates the twist across the three grips, the way a real hand regrips a luer collar.  The
scorer reads the mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=luer_lock_twist_connect.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/luer-lock-twist-connect
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

TAG = "luer"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the vial adapter and its bench fixture (local frame: port axis at x,y = 0, bench top z 0; world z = TOP + local z)
PORT_XY = (0.280, 0.000)
PORT_Z = 0.100                     # female port lip: the work height h = 0.10 m
STAGE_XY = (0.280, 0.070)          # staged syringe (within the 0.30 m forward reach gate)
STAGE_CZ = 0.056                   # staged syringe centre; its tip bottom sits at 0.015, just above the holder
BARREL_D = 0.020                   # 20 mm syringe barrel
TIP_DROP = 0.041                   # syringe centre to the male luer tip bottom (barrel half + collar + tip)
SEAT_CZ = PORT_Z + TIP_DROP        # 0.141: centre when the tip is seated in the port

# -- gripper / grasp ---------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0              # N: a real closed grip
GRIP_CLOSE = 0.034            # opening at/below which a closed finger pair may take the barrel
GRIP_RELEASE = 0.045          # opening above which a held syringe is let go
GRIP_REACH = 0.016            # m: TCP within this of the barrel's grasp point
GRIP_CLEAR = 0.045            # m: no pad within this of the port axis at the scored state
HOME_SLACK = 0.020

HOLD = -0.009                 # finger target: pads close to 18 mm across the 20 mm barrel (2 mm squeeze)
GRASP_UP = 0.012              # TCP sits this far above the syringe centre when grasping
BENCH_ZONE = (0.10, 0.40, 0.18)   # x min/max, |y| max: released inside the bench

# -- the luer-lock thread (declared twist, counted from the syringe's rotation while its tip is seated) ----------------
LOCK_TURN = 1.5 * 2.0 * math.pi    # ~1.5 turns, the brief's lock twist (0.3-0.6 N m at the real bench)
ENGAGE_TURN = math.pi              # 0.5 turn: the thread is engaged (step 3) but not yet locked
LOCK_TOL = math.radians(40.0)      # lock window: 500-580 deg counts as the 1.5-turn lock
CROSS_THRESH = math.radians(25.0)  # turning this far while the tip is NOT seated = cross-thread
ZONE_Z = -0.010                    # a part below this is off the bench

PARTS = ("syringe",)
START_POS = {"syringe": np.array([STAGE_XY[0], STAGE_XY[1], TOP + STAGE_CZ])}
START_QUAT = {"syringe": np.array([0.0, 0.0, 0.0, 1.0])}


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


ADAPTER = UsdPart("adapter", asset="adapter", xy=PORT_XY, extent=0.14, mass=None, fixed=True,
                  usd_path=str(ASSETS / "adapter.usda"),
                  pos=(PORT_XY[0], PORT_XY[1], TOP))
SYRINGE = UsdPart("syringe", asset="syringe", xy=STAGE_XY, extent=0.07, mass=0.012,
                  usd_path=str(ASSETS / "syringe.usda"), pos=tuple(START_POS["syringe"]))
OBJECTS = [ADAPTER, SYRINGE]
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
    if getattr(scene.task_object, "_luer_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        obj = _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)
        # The task writes every task object's root pose itself; Arena's float64 reset pose is rejected by
        # PhysX's transformf kernel on Isaac (the same nested-root issue as the other bench tasks).
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
            env_cfg.events.luer_reset = EventTermCfg(func=_reset_connect, mode="reset",
                                                     params={"asset_cfg": SceneEntityCfg("adapter")})
            env_cfg.events.luer_step = EventTermCfg(func=_connect_step, mode="interval",
                                                    interval_range_s=(0.0, 0.0), is_global_time=False,
                                                    params={"asset_cfg": SceneEntityCfg("adapter")})
            return env_cfg

        return with_events

    task_object._luer_usd_parts = True
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
        # live state
        self.seated = False
        self.turns = 0.0            # signed, unwrapped yaw of the syringe about the port axis
        self.last_yaw = None
        self.cross_turn = 0.0       # turning accumulated while the tip was NOT seated
        # progress latches
        self.picked_up = False
        self.seated_done = False
        self.engaged = False
        self.locked = False
        # caps
        self.cross_thread = False
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


def _reset_connect(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    for p in PARTS:
        _write_part(env, p, START_POS[p], START_QUAT[p], env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: syringe staged beside the 20 mm vial adapter, port empty", flush=True)


def _grasp_point(name):
    m = _MECH
    return m.pos[name] + _rot(m.quat[name]) @ np.array([0.0, 0.0, GRASP_UP])


def _in_allowed_zone(pos):
    x, y, z = pos
    xmin, xmax, ymax = BENCH_ZONE
    return bool(z >= TOP + ZONE_Z and xmin <= x <= xmax and abs(y) <= ymax)


def _seated(m):
    """The male tip is down in the female port, coaxial with it and at the seated height."""
    s = m.pos["syringe"]
    axis = _rot(m.quat["syringe"])[:, 2]
    coax = float(axis[2]) > 0.97
    off = float(np.linalg.norm(s[:2] - np.asarray(PORT_XY)))
    return bool(coax and off < 0.006 and abs(s[2] - (TOP + SEAT_CZ)) < 0.006)


def _connect_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)

    # 1. grasp / carry / release the syringe
    name = "syringe"
    gp = _grasp_point(name)
    if m.held[name] is None:
        for side in ("left", "right"):
            g = grips[f"{side}_gripper"]
            if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - gp)) <= GRIP_REACH:
                m.held[name] = side
                m.offset[name] = g.rot.T @ (m.pos[name] - g.tcp)
                m.rot_off[name] = g.rot.T @ _rot(m.quat[name])
                break
    else:
        g = grips[f"{m.held[name]}_gripper"]
        if g.opening > GRIP_RELEASE:
            m.held[name] = None
            if not _in_allowed_zone(m.pos[name]):
                m.dropped = True
        else:
            m.pos[name] = g.tcp + g.rot @ m.offset[name]
            m.quat[name] = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off[name])

    # 2. static mechanism measurements
    s = m.pos["syringe"]
    m.seated = _seated(m)
    if s[2] > TOP + STAGE_CZ + 0.015:
        m.picked_up = True
    if m.seated:
        m.seated_done = True

    # 3. the luer-lock twist: accumulate the syringe's unwrapped yaw about the port axis.  It only changes while a
    #    gripper holds the syringe (the part is gravity-free and otherwise static), so regrasps do not lose the count.
    yaw = _yaw(_rot(m.quat["syringe"]))
    if m.last_yaw is not None:
        d = _wrap(yaw - m.last_yaw)
        m.turns += d
        if not m.seated:
            m.cross_turn += abs(d)
            if m.cross_turn > CROSS_THRESH:
                m.cross_thread = True
    m.last_yaw = yaw

    if m.seated and abs(m.turns) >= ENGAGE_TURN:
        m.engaged = True
    if m.seated and abs(m.turns) >= LOCK_TURN - LOCK_TOL:
        m.locked = True

    # 4. write the syringe
    _write_part(env, name, m.pos[name], m.quat[name], env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s syringe z {s[2]:.4f} "
              f"{'SEATED' if m.seated else '----- '} {'HELD' if m.held[name] else 'free '} "
              f"turn {math.degrees(m.turns):+5.0f} deg ({abs(m.turns)/(2*math.pi):.2f} turns) "
              f"engaged {int(m.engaged)} locked {int(m.locked)}"
              f"{'  CROSS-THREAD' if m.cross_thread else ''}{'  DROP' if m.dropped else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    for p in PARTS:
        out[p] = ObjectState(pos=m.pos[p].copy(), quat_xyzw=m.quat[p].copy(), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=m.pos[p].copy(), high=m.pos[p].copy())
    out["seated"] = m.seated
    out["picked_up"] = m.picked_up
    out["seated_done"] = m.seated_done
    out["engaged"] = m.engaged
    out["locked"] = m.locked
    out["cross_thread"] = m.cross_thread
    out["dropped"] = m.dropped
    out["turns"] = m.turns
    out["held"] = dict(m.held)


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("syringe picked up", "tip seated in the adapter", "thread engaged (0.5 turn)",
         "luer locked (1.5 turns, held 1 s)")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("picked_up"))
    s1 = s0 and bool(objects.get("seated_done"))
    s2 = s1 and bool(objects.get("engaged"))
    s3 = s2 and bool(objects.get("locked"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("cross_thread") or objects.get("dropped") or _loose(objects))


def _loose(objects) -> bool:
    """A loose connection: the tip is in the port, the lock was never made, and the hands are clear of it."""
    return bool(objects.get("seated") and not objects.get("locked") and _clear(objects))


def _current_ok(objects) -> bool:
    return bool(objects.get("seated") and objects.get("locked")
                and not objects.get("cross_thread") and not objects.get("dropped"))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([PORT_XY[0], PORT_XY[1], TOP + PORT_Z])
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

# Three regrips of half a turn each = the 1.5-turn lock.  At the seated height the left wrist's joint6 is about
# -47 deg - yaw, so a -150 -> +30 deg sweep (j6 +103 -> -77 deg) stays inside its +-120 deg limit.
TWIST_START = math.radians(-150.0)
TWIST = math.pi                 # 180 deg per grip
TWISTS = 3
_OPEN = FINGER_OPEN


def _w(dur, x, y, z, finger, label, yaw=0.0):
    """Waypoint from local z (world z = TOP + z), turning a straight-down grip by yaw about the vertical."""
    return Waypoint(dur, (x, y, TOP + z), finger, label, yaw=yaw)


def demo():
    """Pick the syringe, seat its tip in the vial adapter and twist it 1.5 turns in three regrips, then go home.
    The right arm stays home.  Every approach is from above with the fingers closing across the 20 mm barrel."""
    a = TWIST_START
    stage_gz = STAGE_CZ + GRASP_UP
    seat_gz = SEAT_CZ + GRASP_UP
    travel_gz = 0.160           # a reachable travel height over the port at the twist yaw (joint 4 margin)
    px, py = PORT_XY
    sx, sy = STAGE_XY
    left = []

    # 0. wind the wrist to the twist's start yaw while the syringe is untouched (so no twist is counted yet)
    left += [_w(1.6, sx, sy, stage_gz + 0.055, _OPEN, "to the syringe", a / 2),
             _w(1.4, sx, sy, stage_gz + 0.055, _OPEN, "wind the wrist", a)]

    # A. pick the syringe up by its barrel
    left += [_w(1.4, sx, sy, stage_gz, _OPEN, "down to the syringe", a),
             _w(0.6, sx, sy, stage_gz, _OPEN, "settle at the syringe", a),
             _w(0.7, sx, sy, stage_gz, HOLD, "grip the syringe", a),
             _w(1.4, sx, sy, stage_gz + 0.045, HOLD, "lift the syringe", a)]

    # B. carry it over the port and seat the male tip in the female luer port
    left += [_w(1.8, px, py, travel_gz, HOLD, "carry to the adapter", a),
             _w(0.6, px, py, travel_gz, HOLD, "steady over the port", a),
             _w(1.4, px, py, seat_gz, HOLD, "seat the tip", a)]

    # C. twist 1.5 turns in three half-turn grips, regripping between them
    for k in range(TWISTS):
        if k:  # the previous cycle released and wound back; close on the barrel again
            left += [_w(0.6, px, py, seat_gz, HOLD, f"regrip {k + 1}", a)]
        for i in (1, 2):
            left += [_w(2.0, px, py, seat_gz, HOLD, f"twist {k + 1}.{i}", a + TWIST * i / 2)]
        if k < TWISTS - 1:
            left += [_w(0.5, px, py, seat_gz, _OPEN, f"let go {k + 1}", a + TWIST),
                     _w(0.6, px, py, travel_gz, _OPEN, f"lift {k + 1}", a + TWIST),
                     _w(0.6, px, py, travel_gz, _OPEN, f"turn back {k + 1}.1", a + TWIST / 2),
                     _w(0.6, px, py, travel_gz, _OPEN, f"turn back {k + 1}.2", a),
                     _w(0.6, px, py, seat_gz, _OPEN, f"down {k + 1}", a)]

    # D. release the locked syringe, lift away and go home
    left += [_w(0.6, px, py, seat_gz, _OPEN, "release the syringe", a + TWIST),
             _w(0.8, px, py, travel_gz, _OPEN, "lift away", a + TWIST),
             Waypoint(2.2, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]
    right = [Waypoint(2.2, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"turn {math.degrees(objects.get('turns', 0.0)):+.0f} deg "
            f"({abs(objects.get('turns', 0.0)) / (2 * math.pi):.2f} turns), "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('cross_thread', 'dropped') if objects.get(k)) or '-'}"
            f"{'/loose' if _loose(objects) else ''}")
