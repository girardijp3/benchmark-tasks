"""On the station table: a 20 mm brass spring check valve held in a small bench fixture, with its threaded cap
seated, its seat bare and its new poppet and compression spring staged in pockets beside it.  The robot unscrews
and lifts the cap off (opening the body), picks the poppet and seats it seal-face-down on the seat, picks the spring
and seats it on the poppet, threads the cap back down, then presses the inlet and outlet test buttons to verify
one-way flow: forward flow lifts the poppet, backflow seats and seals it.

The three loose parts are real, gravity-free rigid bodies in their own USD files.  While a gripper is closed on a
part's grasp point the part follows the gripper (a declared carry, like the declared latch detents of the family);
the robot must still reach, close on and place every part, and the seat, spring, cap thread and test buttons are
real geometry.  The scorer reads the mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=check_valve_rebuild.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../tasks/check-valve-rebuild
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

TAG = "cv"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the valve and its fixture (local frame: valve axis at x,y = 0, bench top z 0; world z = TOP + local z) ------------
VALVE_XY = (0.250, 0.000)
VALVE_POS = (VALVE_XY[0], VALVE_XY[1], TOP)
FIX_H = 0.050                 # fixture top / valve body foot (local z)
BODY_TOP = 0.090              # top of the brass body, where the cap seats
SEAT_Z = 0.062                # the seat's sealing ledge
POPPET_SEAT_CZ = SEAT_Z + 0.0015   # poppet disc centre when seated
SPRING_SEAT_CZ = 0.065 + 0.012     # spring centre when seated (bottom on the disc top)
CAP_SEAT_CZ = 0.097           # cap centre when threaded down
CAP_PEG_XY = (0.250, -0.045)
CAP_PEG_CZ = 0.067            # cap centre when parked on the peg
POPPET_STAGE = (0.250, 0.045)
POPPET_STAGE_CZ = 0.0115
SPRING_STAGE = (0.250, 0.070)
SPRING_STAGE_CZ = 0.0220

# -- test buttons (world x,y; local z; the robot presses the TCP down PRESS_DEPTH below the cap) -----------------------
IN_BUTTON = (0.218, -0.025, 0.0305)
OUT_BUTTON = (0.218, 0.025, 0.0305)
BUTTON_PRESS = 0.006
FLOW_LIFT = 0.006             # the poppet lifts this far under forward flow

# -- gripper / grasp ---------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0              # N: a real closed grip
GRIP_CLOSE = 0.034            # opening at/below which a closed finger pair may take a part
GRIP_RELEASE = 0.045          # opening above which a held part is let go
GRIP_REACH = 0.016            # m: TCP within this of the part's grasp point
GRIP_CLEAR = 0.045            # m: no pad within this of the valve at the scored state
THREAD_YAW = math.radians(60.0)   # cap twist that counts as the thread engaged
THREAD_TOL = math.radians(15.0)
HOME_SLACK = 0.020

HOLD = {"cap": -0.012, "poppet": -0.007, "spring": -0.0065}   # finger target per part (about its width)
GRASP_UP = 0.012              # the TCP sits this far above the part centre when grasping
PARTS = ("cap", "poppet", "spring")
START_POS = {"cap": np.array([VALVE_XY[0], VALVE_XY[1], TOP + CAP_SEAT_CZ]),
             "poppet": np.array([POPPET_STAGE[0], POPPET_STAGE[1], TOP + POPPET_STAGE_CZ]),
             "spring": np.array([SPRING_STAGE[0], SPRING_STAGE[1], TOP + SPRING_STAGE_CZ])}
START_QUAT = {p: np.array([0.0, 0.0, 0.0, 1.0]) for p in PARTS}
SEAT_POS = {"poppet": np.array([VALVE_XY[0], VALVE_XY[1], TOP + POPPET_SEAT_CZ]),
            "spring": np.array([VALVE_XY[0], VALVE_XY[1], TOP + SPRING_SEAT_CZ]),
            "cap": np.array([VALVE_XY[0], VALVE_XY[1], TOP + CAP_SEAT_CZ])}


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


VALVE = UsdPart("valve", asset="valve", xy=VALVE_XY, extent=0.14, mass=None, fixed=True,
                usd_path=str(ASSETS / "valve.usda"), pos=VALVE_POS)
CAP = UsdPart("cap", asset="cap", xy=VALVE_XY, extent=0.04, mass=0.030,
              usd_path=str(ASSETS / "cap.usda"), pos=tuple(START_POS["cap"]))
POPPET = UsdPart("poppet", asset="poppet", xy=POPPET_STAGE, extent=0.03, mass=0.008,
                 usd_path=str(ASSETS / "poppet.usda"), pos=tuple(START_POS["poppet"]))
SPRING = UsdPart("spring", asset="spring", xy=SPRING_STAGE, extent=0.04, mass=0.004,
                 usd_path=str(ASSETS / "spring.usda"), pos=tuple(START_POS["spring"]))
OBJECTS = [VALVE, CAP, POPPET, SPRING]
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
    if getattr(scene.task_object, "_cv_usd_parts", False):
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
            env_cfg.events.cv_reset = EventTermCfg(func=_reset_valve, mode="reset",
                                                   params={"asset_cfg": SceneEntityCfg("valve")})
            env_cfg.events.cv_step = EventTermCfg(func=_valve_step, mode="interval",
                                                  interval_range_s=(0.0, 0.0), is_global_time=False,
                                                  params={"asset_cfg": SceneEntityCfg("valve")})
            return env_cfg

        return with_events

    task_object._cv_usd_parts = True
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
        self.body_opened = False
        self.poppet_done = False
        self.spring_done = False
        self.cap_done = False
        self.forward_done = False
        self.reverse_done = False
        # caps
        self.reversed_poppet = False
        self.missing_spring = False
        self.cross_thread = False
        self.dropped = False
        self.leak = False
        # current state
        self.flow_lift = 0.0
        self.pressed = {"in": False, "out": False}
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


def _reset_valve(env, env_ids, asset_cfg=None):
    import torch

    base = env.unwrapped
    _MECH.reset()
    for p in PARTS:
        _write_part(env, p, START_POS[p], START_QUAT[p], env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: cap seated, seat bare, poppet + spring staged", flush=True)


def _grasp_point(name):
    m = _MECH
    return m.pos[name] + _rot(m.quat[name]) @ np.array([0.0, 0.0, GRASP_UP])


def _in_allowed_zone(name, pos):
    x, y, z = pos
    if z < TOP - 0.01 or x < 0.10 or x > 0.40 or abs(y) > 0.18:
        return False
    return True


def _button_pressed(btn, grips):
    """A button is pressed when a gripper's TCP dwells at the button point pushed BUTTON_PRESS below rest."""
    bx, by, bz = btn
    bp = np.array([bx, by, TOP + bz - BUTTON_PRESS])
    for side in ("left", "right"):
        g = grips[f"{side}_gripper"]
        if float(np.linalg.norm(g.tcp - bp)) <= 0.010:
            return True
    return False


def _valve_step(env, env_ids, asset_cfg=None):
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
            else:
                m.pos[name] = g.tcp + g.rot @ m.offset[name]
                m.quat[name] = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off[name])
                m.twist[name] = _wrap(_yaw(g.rot) - m.twist0[name])

    # 2. static mechanism measurements
    cap = m.pos["cap"]
    cap_off_axis = float(np.linalg.norm(cap[:2] - np.asarray(VALVE_XY)))
    cap_high = cap[2] > TOP + BODY_TOP + 0.013
    if cap_high and cap_off_axis > 0.015:
        m.body_opened = True

    pq = _rot(m.quat["poppet"])
    upright = float(pq[2, 2]) > 0.90
    pp = m.pos["poppet"]
    poppet_seated = (float(np.linalg.norm(pp[:2] - np.asarray(VALVE_XY))) < 0.005
                     and abs(pp[2] - (TOP + POPPET_SEAT_CZ)) < 0.005)
    sp = m.pos["spring"]
    spring_seated = (float(np.linalg.norm(sp[:2] - np.asarray(VALVE_XY))) < 0.007
                     and abs(sp[2] - (TOP + SPRING_SEAT_CZ)) < 0.007)
    over_cap = cap_off_axis < 0.010
    threaded = abs(_wrap(m.twist["cap"] - THREAD_YAW)) <= THREAD_TOL
    cap_seated = (cap_off_axis < 0.010 and abs(cap[2] - (TOP + CAP_SEAT_CZ)) < 0.006)

    if poppet_seated and upright:
        m.poppet_done = True
    if poppet_seated and not upright:
        m.reversed_poppet = True
    if spring_seated:
        m.spring_done = True
    if cap_seated and threaded:
        m.cap_done = True
    if cap_seated and not threaded and abs(m.twist["cap"]) < THREAD_YAW - THREAD_TOL and m.body_opened:
        m.cross_thread = True
    if m.body_opened and cap_seated and not spring_seated:
        m.missing_spring = True

    internals_ok = m.poppet_done and m.spring_done and m.cap_done and not m.reversed_poppet and not m.cross_thread
    # 3. one-way flow test
    m.pressed["in"] = _button_pressed(IN_BUTTON, grips)
    m.pressed["out"] = _button_pressed(OUT_BUTTON, grips)
    m.flow_lift = 0.0
    if m.pressed["in"]:
        if internals_ok:
            m.forward_done = True
            m.flow_lift = FLOW_LIFT
        else:
            m.leak = True
    if m.pressed["out"]:
        if internals_ok and poppet_seated and upright:
            m.reverse_done = True
        else:
            m.leak = True

    # 4. write the parts (the poppet visibly lifts under forward flow)
    for p in PARTS:
        pos = m.pos[p].copy()
        if p == "poppet":
            pos[2] += m.flow_lift
        _write_part(env, p, pos, m.quat[p], env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s cap {'off' if m.body_opened else 'on'} twist {math.degrees(m.twist['cap']):+5.0f} "
              f"deg; poppet {'seat' if poppet_seated else '---'} {'up' if upright else 'FLIPPED'} "
              f"at {np.round(m.pos['poppet'],4)} (need z {TOP+POPPET_SEAT_CZ:.4f}); "
              f"spring {'seat' if spring_seated else '---'} at {np.round(m.pos['spring'],4)}; "
              f"cap at {np.round(cap,4)}; flow fwd {int(m.forward_done)} rev {int(m.reverse_done)}"
              f"{'  DROP' if m.dropped else ''}{'  LEAK' if m.leak else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    for p in PARTS:
        out[p] = ObjectState(pos=m.pos[p].copy(), quat_xyzw=m.quat[p].copy(), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=m.pos[p].copy(), high=m.pos[p].copy())
    out["body_opened"] = m.body_opened
    out["poppet_done"] = m.poppet_done
    out["spring_done"] = m.spring_done
    out["cap_done"] = m.cap_done
    out["forward_done"] = m.forward_done
    out["reverse_done"] = m.reverse_done
    out["reversed_poppet"] = m.reversed_poppet
    out["missing_spring"] = m.missing_spring
    out["cross_thread"] = m.cross_thread
    out["dropped"] = m.dropped
    out["leak"] = m.leak
    out["twist"] = dict(m.twist)
    out["pressed"] = dict(m.pressed)
    out["flow_lift"] = m.flow_lift
    out["held"] = dict(m.held)


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("body opened", "poppet + spring seated", "cap threaded + forward flow", "backflow sealed + hold")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("body_opened"))
    s1 = s0 and bool(objects.get("poppet_done")) and bool(objects.get("spring_done"))
    s2 = s1 and bool(objects.get("cap_done")) and bool(objects.get("forward_done"))
    s3 = s2 and bool(objects.get("reverse_done"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("reversed_poppet") or objects.get("missing_spring") or objects.get("cross_thread")
                or objects.get("dropped") or objects.get("leak"))


def _current_ok(objects) -> bool:
    """The rebuilt valve is intact as things stand: internals seated, cap threaded, no damage latch."""
    return bool(objects.get("poppet_done") and objects.get("spring_done") and objects.get("cap_done")
                and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([VALVE_XY[0], VALVE_XY[1], TOP + 0.070])
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
_TWIST = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(70.0)) @ _DOWN
_OPEN = FINGER_OPEN


def _w(dur, x, y, z, finger, label, rot=_DOWN):
    """Waypoint from local z (world z = TOP + z)."""
    return Waypoint(dur, (x, y, TOP + z), finger, label, rot=rot)


def _hover(x, y, z, finger, label, rot=_DOWN):
    return _w(1.6, x, y, z, finger, label, rot)


def _pick(centre, cz, hold, label, rot=_DOWN):
    """Open over the part, descend, close on it, lift clear."""
    x, y = centre
    gz = cz + GRASP_UP
    return [_hover(x, y, gz + 0.045, _OPEN, "over the " + label, rot),
            _w(1.2, x, y, gz, _OPEN, "down to the " + label, rot),
            _w(0.5, x, y, gz, _OPEN, "settle at the " + label, rot),
            _w(0.7, x, y, gz, hold, "grip the " + label, rot),
            _w(1.4, x, y, gz + 0.045, hold, "lift the " + label, rot)]


def _place(centre, cz, label, rot=_DOWN):
    """Carry over the seat, lower to the seated grasp height, open, retreat."""
    x, y = centre
    gz = cz + GRASP_UP
    hold = HOLD.get(label, -0.007)
    return [_hover(x, y, gz + 0.045, hold, "carry the " + label, rot),
            _w(0.5, x, y, gz + 0.045, hold, "steady over the " + label, rot),
            _w(1.4, x, y, gz, hold, "lower the " + label, rot),
            _w(0.6, x, y, gz, _OPEN, "release the " + label, rot),
            _w(0.8, x, y, gz + 0.045, _OPEN, "lift away from the " + label, rot)]


def demo():
    """The left arm removes the cap, seats the poppet and spring, threads the cap back on and runs both flow tests.
    The right arm stays home.  All parts are picked from known pockets; the cap is threaded with an 8 degree margin
    over the 85 degree requirement, and every approach is from above with the fingers closing across the part."""
    left = []
    # A. open the body: lift the seated cap and park it on the peg
    left += _pick(VALVE_XY, CAP_SEAT_CZ, HOLD["cap"], "cap")
    left += [_hover(CAP_PEG_XY[0], CAP_PEG_XY[1], CAP_PEG_CZ + GRASP_UP + 0.040, HOLD["cap"], "over the cap peg"),
             _w(1.2, CAP_PEG_XY[0], CAP_PEG_XY[1], CAP_PEG_CZ + GRASP_UP, HOLD["cap"], "down to the cap peg"),
             _w(0.6, CAP_PEG_XY[0], CAP_PEG_XY[1], CAP_PEG_CZ + GRASP_UP, _OPEN, "park the cap"),
             _w(0.8, CAP_PEG_XY[0], CAP_PEG_XY[1], CAP_PEG_CZ + GRASP_UP + 0.040, _OPEN, "up from the peg")]
    # B. seat the poppet, seal face down
    left += _pick(POPPET_STAGE, POPPET_STAGE_CZ, HOLD["poppet"], "poppet")
    left += _place(VALVE_XY, POPPET_SEAT_CZ, "poppet")
    # C. seat the spring on the poppet
    left += _pick(SPRING_STAGE, SPRING_STAGE_CZ, HOLD["spring"], "spring")
    left += _place(VALVE_XY, SPRING_SEAT_CZ, "spring")
    # D. pick the cap from the peg, twist the thread in, and screw it down
    left += _pick(CAP_PEG_XY, CAP_PEG_CZ, HOLD["cap"], "parked cap")
    left += [_hover(VALVE_XY[0], VALVE_XY[1], CAP_SEAT_CZ + GRASP_UP + 0.050, HOLD["cap"], "cap over the bore"),
             _w(1.4, VALVE_XY[0], VALVE_XY[1], CAP_SEAT_CZ + GRASP_UP + 0.050, HOLD["cap"], "twist the thread", _TWIST),
             _w(1.5, VALVE_XY[0], VALVE_XY[1], CAP_SEAT_CZ + GRASP_UP, HOLD["cap"], "thread the cap down", _TWIST),
             _w(0.6, VALVE_XY[0], VALVE_XY[1], CAP_SEAT_CZ + GRASP_UP, _OPEN, "release the cap", _TWIST),
             _w(0.8, VALVE_XY[0], VALVE_XY[1], CAP_SEAT_CZ + GRASP_UP + 0.050, _OPEN, "lift away", _TWIST)]
    # E. verify one-way flow at the two buttons
    for btn, label in ((IN_BUTTON, "inlet"), (OUT_BUTTON, "outlet")):
        bx, by, bz = btn
        left += [_hover(bx, by, bz + 0.045, _OPEN, "over the " + label + " button"),
                 _w(1.2, bx, by, bz - BUTTON_PRESS, _OPEN, "press the " + label + " button"),
                 _w(1.0, bx, by, bz - BUTTON_PRESS, _OPEN, "hold the " + label + " test"),
                 _w(0.8, bx, by, bz + 0.045, _OPEN, "release the " + label + " button")]
    left += [Waypoint(2.2, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]
    right = [Waypoint(2.2, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"cap twist {math.degrees(objects.get('twist', {}).get('cap', 0)):+.0f} deg, "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('reversed_poppet','cross_thread','dropped','leak') if objects.get(k)) or '-'}")
