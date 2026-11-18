"""On the station table: a care-training manikin lying supine, its bare neck exposed, and a two-piece rigid
cervical collar staged on the bench beside it: a front chin-support shell and a back occipital-support shell, each
with two 30 mm side tabs.  The robot fits the front shell under the manikin's chin, fits the back shell behind the
neck, then presses each side tab inward with a 4-7 N hand press until it seats with a click, leaving the collar
correctly fitted.

The two shells are real, gravity-free rigid bodies in their own USD files.  While a gripper's fingers close across a
shell's grasp point the shell follows the gripper (a declared carry, like the family's detents); the robot must
still reach, close on, orient, carry and seat each shell.  The manikin is a fixed, welded visual-only body, so the
fit is a two-hand manipulation task at a work height below 0.30 m.  The tabs seat on a declared compliant travel:
K_TAB = 700 N/m, so 4 N just seats a tab (5.7 mm travel), 7 N is the forced-fit limit (10 mm), and the press is
measured from the gripper TCP's real displacement in MuJoCo/Isaac.  Neck moves, tab misses and a forced fit are
explicit score caps.  The scorer reads the mechanism flags maintained here, exactly as the Isaac scorer does.

Run it:  inspect-robots run --embodiment yam_arena -E task=cervical_collar_shells.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../tasks/cervical-collar-shells
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

TAG = "ccs"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the manikin and collar (local frame: neck axis at x,y = 0; bench top z 0; world z = TOP + local z) ---------------
NECK_XY = (0.250, 0.000)
FRONT_SEAT = (0.285, 0.000, 0.120)   # front shell centre, under the chin (+x = front of the manikin)
BACK_SEAT = (0.215, 0.000, 0.120)    # back shell centre, behind the neck (occiput)
FRONT_STAGE = (0.205, 0.095)
BACK_STAGE = (0.205, -0.095)
SHELL_STAGE_CZ = 0.012               # shell centre rest height on the bench

# -- the side tabs (30 mm long): a gripper presses them DOWN to seat; force = K_TAB * travel ---------------------------
TAB_Y = 0.050                        # tab centre +/- y from the neck axis
TAB_REST_Z = 0.132                   # local z of the tab top at rest
K_TAB = 700.0                        # N/m: declared compliant tab (0.005714 m = 4 N, 0.010 m = 7 N)
DEPTH_SEAT = 4.0 / K_TAB             # 5.714 mm travel: just seats (4 N)
DEPTH_FORCE = 7.0 / K_TAB            # 10 mm travel: the forced-fit limit (7 N)
DEPTH_TOL = 0.0015
PRESS_RADIAL = 0.018                 # horizontal radius within which a press counts for a tab
PRESS_HI = 0.004                     # tab zone extends this far above rest
PRESS_LO = 0.030                     # and this far below rest

# -- seating / grip ---------------------------------------------------------------------------------------------------
SEAT_XY_TOL = 0.012
SEAT_Z_TOL = 0.010
SHELL_TILT_MAX = 20.0               # degrees (tilt() returns degrees)
HOME_SLACK = 0.020
PAD_BACK = 0.025
GRIP_FORCE = 1.0                     # N: a real closed grip
GRIP_CLOSE = 0.070                   # opening at/below which a closed finger pair may take a shell
GRIP_RELEASE = 0.085                 # opening above which a held shell is let go
GRIP_REACH = 0.020                   # m: TCP within this of the shell's grasp point
GRIP_CLEAR = 0.050                   # m: no pad within this of the neck axis at the scored state
GRASP_UP = 0.010                     # TCP sits this far above the shell centre when grasping
HOLD = -0.033                        # finger target when holding a shell (opening ~ 0.066 m)
PARTS = ("front", "back")
START_POS = {"front": np.array([FRONT_STAGE[0], FRONT_STAGE[1], TOP + SHELL_STAGE_CZ]),
             "back": np.array([BACK_STAGE[0], BACK_STAGE[1], TOP + SHELL_STAGE_CZ])}
START_QUAT = {p: np.array([0.0, 0.0, 0.0, 1.0]) for p in PARTS}
SEAT_POS = {"front": np.array([FRONT_SEAT[0], FRONT_SEAT[1], TOP + FRONT_SEAT[2]]),
            "back": np.array([BACK_SEAT[0], BACK_SEAT[1], TOP + BACK_SEAT[2]])}
TAB_CENTRE = {"left": np.array([NECK_XY[0], TAB_Y, TOP + TAB_REST_Z]),
              "right": np.array([NECK_XY[0], -TAB_Y, TOP + TAB_REST_Z])}


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


MANIKIN = UsdPart("manikin", asset="manikin", xy=NECK_XY, extent=0.20, mass=None, fixed=True,
                  usd_path=str(ASSETS / "manikin.usda"), pos=(NECK_XY[0], NECK_XY[1], TOP))
FRONT = UsdPart("front", asset="front", xy=FRONT_STAGE, extent=0.10, mass=0.140,
                usd_path=str(ASSETS / "front.usda"), pos=tuple(START_POS["front"]))
BACK = UsdPart("back", asset="back", xy=BACK_STAGE, extent=0.10, mass=0.160,
               usd_path=str(ASSETS / "back.usda"), pos=tuple(START_POS["back"]))
OBJECTS = [MANIKIN, FRONT, BACK]
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
    if getattr(scene.task_object, "_ccs_usd_parts", False):
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
            env_cfg.events.ccs_reset = EventTermCfg(func=_reset_collar, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("manikin")})
            env_cfg.events.ccs_step = EventTermCfg(func=_collar_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("manikin")})
            return env_cfg

        return with_events

    task_object._ccs_usd_parts = True
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


def _tilt(mat) -> float:
    return math.degrees(math.acos(min(1.0, max(-1.0, float(mat[2, 2])))))


class _Mechanism:
    def __init__(self):
        self.pos = {p: START_POS[p].copy() for p in PARTS}
        self.quat = {p: START_QUAT[p].copy() for p in PARTS}
        self.held = {p: None for p in PARTS}
        self.offset = {}
        self.rot_off = {}
        # progress latches
        self.front_done = False
        self.back_done = False
        self.left_tab_done = False
        self.right_tab_done = False
        # caps
        self.neck_moves = False
        self.tab_miss = False
        self.forced_fit = False
        # tab press tracking
        self.tab_inzone = {s: False for s in ("left", "right")}
        self.tab_maxdepth = {s: 0.0 for s in ("left", "right")}
        # live readings
        self.tilt = {p: 0.0 for p in PARTS}
        self.depth = {s: 0.0 for s in ("left", "right")}
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


def _reset_collar(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    for p in PARTS:
        _write_part(env, p, START_POS[p], START_QUAT[p], env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: two shells staged beside the manikin; tabs unseated", flush=True)


def _grasp_point(name):
    m = _MECH
    return m.pos[name] + _rot(m.quat[name]) @ np.array([0.0, 0.0, GRASP_UP])


def _in_allowed_zone(pos):
    x, y, z = pos
    return bool(z >= TOP - 0.01 and 0.10 <= x <= 0.44 and abs(y) <= 0.20)


def _collar_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)
    sides = ("left", "right")

    # 1. grasp / carry / release each shell
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
                    break
        else:
            g = grips[f"{m.held[name]}_gripper"]
            if g.opening > GRIP_RELEASE:
                m.held[name] = None
                if not _in_allowed_zone(m.pos[name]):
                    m.neck_moves = True            # a shell knocked off the bench disturbs the manikin
            else:
                m.pos[name] = g.tcp + g.rot @ m.offset[name]
                m.quat[name] = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off[name])

    # 2. shell seat measurements
    for name, target, flag in (("front", SEAT_POS["front"], "front_done"), ("back", SEAT_POS["back"], "back_done")):
        p = m.pos[name]
        m.tilt[name] = _tilt(_rot(m.quat[name]))
        off = float(np.linalg.norm(p[:2] - target[:2]))
        in_seat = off < SEAT_XY_TOL and abs(p[2] - target[2]) < SEAT_Z_TOL
        if in_seat and m.tilt[name] <= SHELL_TILT_MAX:
            setattr(m, flag, True)
        if in_seat and m.tilt[name] > SHELL_TILT_MAX:
            m.neck_moves = True                    # shell jammed against the neck at an angle

    # 3. tab presses: force = K_TAB * downward travel; seat in [4 N, 7 N], over 7 N is a forced fit
    for side in sides:
        g = grips[f"{side}_gripper"]
        c = TAB_CENTRE[side]
        horiz = float(np.linalg.norm(g.tcp[:2] - c[:2]))
        depth = float(c[2] - g.tcp[2])             # positive = pressed below the tab rest height
        m.depth[side] = depth
        in_zone = horiz <= PRESS_RADIAL and -PRESS_HI <= depth <= PRESS_LO
        if in_zone:
            m.tab_maxdepth[side] = max(m.tab_maxdepth[side], depth)
            if DEPTH_SEAT - DEPTH_TOL <= depth <= DEPTH_FORCE + DEPTH_TOL:
                if side == "left":
                    m.left_tab_done = True
                else:
                    m.right_tab_done = True
            if depth > DEPTH_FORCE + DEPTH_TOL:
                m.forced_fit = True
        else:
            done = m.left_tab_done if side == "left" else m.right_tab_done
            if m.tab_inzone[side] and not done and m.tab_maxdepth[side] < DEPTH_SEAT - DEPTH_TOL:
                m.tab_miss = True                  # the press never caught the tab
        m.tab_inzone[side] = in_zone

    # 4. write the shells
    for p in PARTS:
        _write_part(env, p, m.pos[p], m.quat[p], env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s front {'seat' if m.front_done else '---'} back {'seat' if m.back_done else '---'} "
              f"tabs L {'in' if m.left_tab_done else '--'} R {'in' if m.right_tab_done else '--'} "
              f"press {m.depth['left'] * 1000:5.1f}/{m.depth['right'] * 1000:5.1f} mm "
              f"({K_TAB * m.depth['left']:4.1f}/{K_TAB * m.depth['right']:4.1f} N) "
              f"caps neck {int(m.neck_moves)} miss {int(m.tab_miss)} force {int(m.forced_fit)}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    for p in PARTS:
        out[p] = ObjectState(pos=m.pos[p].copy(), quat_xyzw=m.quat[p].copy(), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=m.pos[p].copy(), high=m.pos[p].copy())
    out["front_done"] = m.front_done
    out["back_done"] = m.back_done
    out["left_tab_done"] = m.left_tab_done
    out["right_tab_done"] = m.right_tab_done
    out["neck_moves"] = m.neck_moves
    out["tab_miss"] = m.tab_miss
    out["forced_fit"] = m.forced_fit
    out["depth"] = dict(m.depth)
    out["tilt"] = dict(m.tilt)
    out["held"] = dict(m.held)


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("front shell under chin", "back shell behind neck", "left tab engaged", "right tab engaged + hold")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("front_done"))
    s1 = s0 and bool(objects.get("back_done"))
    s2 = s1 and bool(objects.get("left_tab_done"))
    s3 = s2 and bool(objects.get("right_tab_done"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("neck_moves") or objects.get("tab_miss") or objects.get("forced_fit"))


def _current_ok(objects) -> bool:
    """The collar is intact as things stand: both shells seated, no damage latch."""
    return bool(objects.get("front_done") and objects.get("back_done") and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([NECK_XY[0], NECK_XY[1], TOP + 0.195])
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
_OPEN = FINGER_OPEN
PRESS_DEPTH = 0.008                    # 5.6 N, mid-band (4 N seats, 7 N is the limit)


def _w(dur, x, y, z, finger, label, rot=_DOWN):
    return Waypoint(dur, (x, y, TOP + z), finger, label, rot=rot)


def _hover(x, y, z, finger, label, rot=_DOWN):
    return _w(1.6, x, y, z, finger, label, rot)


def _pick(centre, cz, label):
    x, y = centre
    gz = cz + GRASP_UP
    return [_hover(x, y, gz + 0.010, _OPEN, "over the " + label),
            _w(1.2, x, y, gz, _OPEN, "down to the " + label),
            _w(0.5, x, y, gz, _OPEN, "settle at the " + label),
            _w(0.7, x, y, gz, HOLD, "grip the " + label),
            _w(1.4, x, y, gz + 0.010, HOLD, "lift the " + label)]


def _place(centre, cz, label):
    x, y = centre
    gz = cz + GRASP_UP
    return [_hover(x, y, gz + 0.010, HOLD, "carry the " + label),
            _w(0.5, x, y, gz + 0.010, HOLD, "steady over the " + label),
            _w(1.4, x, y, gz, HOLD, "lower the " + label),
            _w(0.6, x, y, gz, _OPEN, "release the " + label),
            _w(0.8, x, y, gz + 0.010, _OPEN, "lift away from the " + label)]


def _press_tab(side):
    c = TAB_CENTRE[side]
    x, y, z = c[0], c[1], c[2] - TOP
    return [_hover(x, y, z + 0.018, _OPEN, "over the " + side + " tab"),
            _w(1.2, x, y, z - PRESS_DEPTH + 0.004, HOLD, "approach the " + side + " tab"),
            _w(0.8, x, y, z - PRESS_DEPTH, HOLD, "press the " + side + " tab"),
            _w(0.8, x, y, z - PRESS_DEPTH, HOLD, "hold the " + side + " tab seated"),
            _w(0.8, x, y, z + 0.018, _OPEN, "release the " + side + " tab")]


def _home_wp(side, dur=2.2):
    return Waypoint(dur, tuple(_START_TCP[side]), _OPEN, "home", rot=_START_ROT[side])


def demo():
    """The two arms work one at a time (they collide if both reach the neck together).

    Left: pick and seat the front shell under the chin, go home, wait for the right hand, then press the left tab.
    Right: wait, pick and seat the back shell behind the neck, go home, wait for the left tab, then press the right tab.
    Each tab is pressed 8 mm (5.6 N, mid-band), held, and released; the final state holds 1 s with both grippers home.
    """
    SEAT_PHASE = 10.3 + 2.2     # pick + place + home (one arm)
    TAB_PHASE = 5.2 + 2.2       # press + home (one arm)

    left = []
    left += _pick(FRONT_STAGE, SHELL_STAGE_CZ, "front shell")
    left += _place(FRONT_SEAT[:2], FRONT_SEAT[2], "front shell")
    left += [_home_wp("left")]
    left += [_home_wp("left", SEAT_PHASE)]          # the right hand seats the back shell
    left += _press_tab("left")
    left += [_home_wp("left")]

    right = []
    right += [_home_wp("right", SEAT_PHASE)]        # the left hand seats the front shell
    right += _pick(BACK_STAGE, SHELL_STAGE_CZ, "back shell")
    right += _place(BACK_SEAT[:2], BACK_SEAT[2], "back shell")
    right += [_home_wp("right")]
    right += [_home_wp("right", TAB_PHASE)]          # the left hand presses the left tab
    right += _press_tab("right")
    right += [_home_wp("right")]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"press {K_TAB * objects.get('depth', {}).get('left', 0):.1f}/"
            f"{K_TAB * objects.get('depth', {}).get('right', 0):.1f} N, "
            f"caps {'/'.join(k for k in ('neck_moves','tab_miss','forced_fit') if objects.get(k)) or '-'}")
