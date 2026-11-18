"""On the station table: a fixed trim panel with an 8 mm mounting hole, and a real 8 mm nylon panel push-clip
staged upright in a small tray beside it.  The robot picks the clip by its 20 mm cap, carries it over the hole,
aligns the stem with the hole, presses it straight down with a 4-7 N hand press until the barbs snap under the
panel and the cap is flush, then pulls up on the seated cap with a 4-7 N pull and holds for 1 s.  The barbs keep
the clip in the panel, so a held pull tests retention rather than extracting it.

The clip is a real, gravity-free rigid body in its own USD file.  While a gripper's fingers close across its cap
the clip follows the gripper (a declared carry, like the family's detents); once the barbs seat the clip is locked
in the panel and no longer follows the gripper, so an upward pull is measured as gripper travel against a fixed
clip.  The panel and tray are fixed, welded, visual-only bodies with no colliders (as in the family's fixtures),
which lets the gripper close at the cap and the clip pass through the hole.  Insertion force and pull force are
declared compliant travels from the real gripper TCP displacement: K = 350 N/m, 4 N = 11.43 mm, 7 N = 20 mm.
A cross-insert (tilted press), a clip left unseated, a dropped clip and a forced fit are explicit score caps.
The scorer reads the mechanism flags maintained here, exactly as the Isaac scorer does.

Run it:  inspect-robots run --embodiment yam_arena -E task=panel_push_clip.py --scorer task_score ...
Fast local gate:  cd local_sim && PYTHONPATH=. .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/panel-push-clip
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
from yam_arena.urdf import quat_xyzw_to_matrix

TAG = "ppc"
SUCCESS_HOLD_S = 0.0  # the 1 s pull-test hold is timed in this task file, not by the harness
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the panel hole and the staged clip (local frame: bench top z 0; world z = TOP + local z) -------------------------
HOLE_XY = (0.280, 0.000)
CLIP_STAGE = (0.200, 0.090)
STAGE_CAP_Z = 0.030           # cap-top height of the clip standing in its tray

# -- real 8 mm push-clip: 20 mm cap, 8 mm stem, ~18 mm stem length, barbs near the tip, ~2 g ---------------------------
CAP_R = 0.010                 # 20 mm head diameter
CAP_T = 0.003                 # 3 mm head thickness
STEM_R = 0.004                # 8 mm stem, fits the 8 mm hole
STEM_LEN = 0.018
TIP_Z = -0.023                # stem tip local z below the cap top
CLIP_MASS = 0.002             # 2 g (a real one is about 2 g)

# -- seat / press geometry (cap-top local z) -------------------------------------------------------------------------
SEAT_CAP_Z = 0.003            # when the cap underside (-0.003) is flush with the panel top (z 0)
HOVER_CAP_Z = 0.026           # carried above the hole, before pressing
PRESS_REF_Z = 0.020           # cap height when the stem first enters the hole; press measured from here
PRESS_ENTER_Z = 0.013         # below this cap height the barbs are at the hole
K_CLIP = 350.0                # N/m declared insertion force: 4 N = 11.43 mm, 7 N = 20 mm travel
DEPTH_4N = 4.0 / K_CLIP
DEPTH_7N = 7.0 / K_CLIP
CAP4_Z = PRESS_REF_Z - DEPTH_4N
CAP7_Z = PRESS_REF_Z - DEPTH_7N

# -- seating / alignment tolerances ----------------------------------------------------------------------------------
SEAT_XY_TOL = 0.004
SEAT_Z_TOL = 0.0025
SEAT_TILT_MAX = 8.0           # degrees
ALIGN_XY_TOL = 0.010
ALIGN_TILT_MAX = 12.0
CROSS_TILT = 12.0             # pressing tilted past this catches the barbs crosswise
ALIGN_Z_LO, ALIGN_Z_HI = 0.014, 0.032

# -- pull-test (declared retention pull, same compliant band) --------------------------------------------------------
K_PULL = 350.0
PULL_MIN = 4.0 / K_PULL
PULL_MAX = 7.0 / K_PULL
PULL_REACH = 0.035
HOLD_S = 1.0


def _in_pull_band(pull: float) -> bool:
    """A retention pull in the 4-7 N band (K_PULL * travel), with a small tolerance."""
    return bool(PULL_MIN - 0.002 <= pull <= PULL_MAX + 0.002)


def _advance_pull_hold(hold_s: float, dt: float, pull: float, holding: bool, seated: bool) -> tuple[float, bool]:
    """The pull-test's 1 s hold, as a pure step: accumulate while the pull is held in band against a seated clip;
    any lapse restarts the count. Returns (new_hold_seconds, verified_now)."""
    if seated and holding and _in_pull_band(pull):
        new = hold_s + dt
        return new, bool(new >= HOLD_S - 1e-9)
    return 0.0, False

# -- grasp / carry ---------------------------------------------------------------------------------------------------
HOME_SLACK = 0.020
PAD_BACK = 0.025
GRIP_FORCE = 1.0
GRIP_CLOSE = 0.070            # opening at/below which a closed finger pair may take the clip
GRIP_RELEASE = 0.085          # opening above which a held clip is let go
GRIP_REACH = 0.020            # TCP within this of the cap grasp point picks the clip up
GRIP_CLEAR = 0.050
GRASP_UP = 0.010              # TCP sits this far above the cap top when grasping
HOLD = -0.033
START_POS = np.array([CLIP_STAGE[0], CLIP_STAGE[1], TOP + STAGE_CAP_Z])
START_QUAT = np.array([0.0, 0.0, 0.0, 1.0])
SEAT_POS = np.array([HOLE_XY[0], HOLE_XY[1], TOP + SEAT_CAP_Z])
NAME = "clip"
ALLOW_LO, ALLOW_HI = 0.10, 0.44
ALLOW_Y = 0.20


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


PANEL = UsdPart("panel", asset="panel", xy=HOLE_XY, extent=0.16, mass=None, fixed=True,
                usd_path=str(ASSETS / "panel.usda"), pos=(HOLE_XY[0], HOLE_XY[1], TOP))
TRAY = UsdPart("tray", asset="tray", xy=CLIP_STAGE, extent=0.05, mass=None, fixed=True,
               usd_path=str(ASSETS / "tray.usda"), pos=(CLIP_STAGE[0], CLIP_STAGE[1], TOP))
CLIP = UsdPart("clip", asset="clip", xy=CLIP_STAGE, extent=0.03, mass=CLIP_MASS,
               usd_path=str(ASSETS / "clip.usda"), pos=tuple(START_POS))
OBJECTS = [PANEL, TRAY, CLIP]
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
            tcp=tcp, back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
            rot=np.asarray(rot, float))
    return out


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_ppc_usd_parts", False):
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
            env_cfg.events.ppc_reset = EventTermCfg(func=_reset_clip, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("clip")})
            env_cfg.events.ppc_step = EventTermCfg(func=_clip_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("clip")})
            return env_cfg

        return with_events

    task_object._ppc_usd_parts = True
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
        self.pos = START_POS.copy()
        self.quat = START_QUAT.copy()
        self.held = None
        self.offset = None
        self.rot_off = None
        self.aligned_done = False
        self.pressed_done = False
        self.seated_done = False
        self.pull_verified = False
        self.cross_insert = False
        self.not_seated = False
        self.dropped = False
        self.forced_fit = False
        self.was_pressed_zone = False
        self.pull_hold = 0.0
        self.pull_depth = 0.0
        self.depth = 0.0
        self.tilt = 0.0
        self.t = 0.0

    def reset(self):
        self.__init__()


_MECH = _Mechanism()


def _write_clip(env, pos, quat, env_ids):
    import torch

    part = env.scene[NAME]
    pose = torch.tensor([[pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]],
                        device=part.device, dtype=torch.float32)
    part.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    part.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=part.device,
                                                                     dtype=torch.float32), env_ids=env_ids)


def _reset_clip(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_clip(env, START_POS, START_QUAT, env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: clip upright in the tray, panel hole clear", flush=True)


def _grasp_point():
    return _MECH.pos + _rot(_MECH.quat) @ np.array([0.0, 0.0, GRASP_UP])


def _in_allowed_zone(pos):
    x, y, z = pos
    return bool(z >= TOP - 0.01 and ALLOW_LO <= x <= ALLOW_HI and abs(y) <= ALLOW_Y)


def _clip_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    dt = float(base.step_dt)
    grips = _gripper_states(env)
    sides = ("left", "right")

    m.tilt = _tilt(_rot(m.quat))
    cap_z = float(m.pos[2] - TOP)
    depth = PRESS_REF_Z - cap_z
    m.depth = depth
    off = float(np.linalg.norm(m.pos[:2] - np.asarray(HOLE_XY)))

    # 1. declared carry: only before the barbs seat; a seated clip is locked in the panel
    if not m.seated_done:
        gp = _grasp_point()
        if m.held is None:
            for side in sides:
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
                elif m.was_pressed_zone and not m.seated_done:
                    m.not_seated = True       # left in the hole with the barbs not fully seated
            else:
                m.pos = g.tcp + g.rot @ m.offset
                m.quat = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off)
    else:
        # once seated the barbs hold the clip; a release off the bench is still a drop
        if m.held is not None:
            g = grips[f"{m.held}_gripper"]
            if g.opening > GRIP_RELEASE:
                m.held = None

    # 2. latches from the real clip pose
    if (off <= ALIGN_XY_TOL and m.tilt <= ALIGN_TILT_MAX and ALIGN_Z_LO <= cap_z <= ALIGN_Z_HI):
        m.aligned_done = True
    in_press_zone = cap_z <= PRESS_ENTER_Z and off <= ALIGN_XY_TOL
    if in_press_zone:
        m.was_pressed_zone = True
        if m.tilt > CROSS_TILT:
            m.cross_insert = True            # barbs catching at an angle
        if depth >= DEPTH_4N and depth <= DEPTH_7N + 0.003:
            m.pressed_done = True
        if depth > DEPTH_7N + 0.003:
            m.forced_fit = True
    if (off <= SEAT_XY_TOL and abs(cap_z - SEAT_CAP_Z) <= SEAT_Z_TOL
            and m.tilt <= SEAT_TILT_MAX and DEPTH_4N <= depth <= DEPTH_7N + 0.003):
        m.seated_done = True
    if m.pos[2] < TOP - 0.02 or m.tilt > 60.0:
        m.dropped = True

    # 3. pull-test: a closed gripper pulls up on the seated cap; the clip stays put (barbs hold)
    if m.seated_done:
        gp = _grasp_point()
        pull, holding = 0.0, False
        for side in sides:
            g = grips[f"{side}_gripper"]
            horiz = float(np.linalg.norm(g.tcp[:2] - gp[:2]))
            if g.opening <= GRIP_CLOSE and horiz <= PULL_REACH and g.tcp[2] >= gp[2] - 0.004:
                pull = max(pull, float(g.tcp[2] - gp[2]))
                holding = True
        m.pull_depth = pull
        m.pull_hold, verified_now = _advance_pull_hold(m.pull_hold, dt, pull, holding, True)
        if verified_now:
            m.pull_verified = True
        if holding and pull > PULL_MAX + 0.006:
            m.not_seated = True              # yanked out past the barbs' retention

    _write_clip(env, m.pos, m.quat, env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s cap {cap_z * 1000:5.1f} mm off {off * 1000:5.1f} mm tilt {m.tilt:4.1f} "
              f"ins {K_CLIP * depth:4.1f} N pull {K_PULL * m.pull_depth:4.1f} N "
              f"aligned {int(m.aligned_done)} press {int(m.pressed_done)} seat {int(m.seated_done)} "
              f"verified {int(m.pull_verified)} caps "
              f"cross {int(m.cross_insert)} unseat {int(m.not_seated)} drop {int(m.dropped)} force {int(m.forced_fit)}",
              flush=True)
    m.t += dt


def extra_states(env, out, origin):
    m = _MECH
    out[NAME] = ObjectState(pos=m.pos.copy(), quat_xyzw=m.quat.copy(), lin_vel=np.zeros(3),
                            ang_vel=np.zeros(3), low=m.pos.copy(), high=m.pos.copy())
    out["aligned_done"] = m.aligned_done
    out["pressed_done"] = m.pressed_done
    out["seated_done"] = m.seated_done
    out["pull_verified"] = m.pull_verified
    out["cross_insert"] = m.cross_insert
    out["not_seated"] = m.not_seated
    out["dropped"] = m.dropped
    out["forced_fit"] = m.forced_fit
    out["depth"] = m.depth
    out["pull_depth"] = m.pull_depth
    out["tilt"] = m.tilt
    out["held"] = m.held
    out["pull_hold"] = m.pull_hold


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("clip aligned with the mounting hole", "pressed with a 4-7 N insertion", "barbs seated, cap flush",
         "pull-test held 1 s")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("aligned_done"))
    s1 = s0 and bool(objects.get("pressed_done"))
    s2 = s1 and bool(objects.get("seated_done"))
    s3 = s2 and bool(objects.get("pull_verified"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("cross_insert") or objects.get("not_seated")
                or objects.get("dropped") or objects.get("forced_fit"))


def _clip_intact(objects) -> bool:
    p = np.asarray(objects[NAME].pos, float)
    off = float(np.linalg.norm(p[:2] - np.asarray(HOLE_XY)))
    return bool(off <= SEAT_XY_TOL and abs(p[2] - SEAT_POS[2]) <= SEAT_Z_TOL and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([HOLE_XY[0], HOLE_XY[1], TOP + 0.06])
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
    return bool(all(steps_done(objects)) and _clip_intact(objects) and _clear(objects)
                and _home(objects, "left") and _home(objects, "right"))


def success(objects) -> bool:
    """All four steps, the clip still seated and intact, no cap, both grippers clear and home. The pull's 1 s hold is
    timed in _clip_step (pull_hold/pull_verified), so this gate is immediate once the final state is reached."""
    return _full(objects)


def score(objects) -> float:
    done = steps_done(objects)
    pts = POINTS * sum(done)
    if _caps(objects):
        pts = min(pts, 2 * POINTS)
    if _full(objects):
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
PRESS_CAP_Z = 0.007            # 4.55 N insertion, in the 4-7 N band
PULL_RISE = 0.016              # 5.6 N retention pull, in the 4-7 N band


def _w(dur, x, y, z, finger, label, rot=_DOWN):
    return Waypoint(dur, (x, y, TOP + z), finger, label, rot=rot)


def _hover(x, y, z, finger, label, rot=_DOWN):
    return _w(1.6, x, y, z, finger, label, rot)


def _home_wp(side, dur=2.2):
    return Waypoint(dur, tuple(_START_TCP[side]), _OPEN, "home", rot=_START_ROT[side])


def _tcp_at(x, y, cap_z):
    """TCP that grasps a clip whose cap top is at local z ``cap_z`` (GRASP_UP above it)."""
    return (x, y, cap_z + GRASP_UP)


def demo():
    """Right arm only: pick the clip, align it over the hole, press to 4.55 N, press home to 5.95 N (barbs seated,
    cap flush), let go, re-grip the seated cap, pull up 5.6 N and hold 1 s, let go and go home. The left arm stays home."""
    sx, sy = CLIP_STAGE
    hx, hy = HOLE_XY
    wp = []
    # pick the staged clip by its cap
    wp += [_hover(sx, sy, STAGE_CAP_Z + GRASP_UP + 0.014, _OPEN, "over the staged clip")]
    wp += [_w(1.2, sx, sy, STAGE_CAP_Z + GRASP_UP, _OPEN, "down to the clip cap")]
    wp += [_w(0.4, sx, sy, STAGE_CAP_Z + GRASP_UP, _OPEN, "settle on the cap")]
    wp += [_w(0.6, sx, sy, STAGE_CAP_Z + GRASP_UP, HOLD, "grip the clip cap")]
    wp += [_w(1.2, sx, sy, STAGE_CAP_Z + GRASP_UP + 0.014, HOLD, "lift the clip")]
    # carry over the hole and align (stem vertical, over the hole, above the barbs)
    wp += [_hover(hx, hy, HOVER_CAP_Z + GRASP_UP, HOLD, "carry over the mounting hole")]
    wp += [_w(1.0, hx, hy, HOVER_CAP_Z + GRASP_UP, HOLD, "align the stem with the hole")]
    # press with 4-7 N to engage the barbs, then press home until the cap is flush
    wp += [_w(1.2, hx, hy, PRESS_CAP_Z + GRASP_UP, HOLD, "press with 4-7 N into the hole")]
    wp += [_w(0.8, hx, hy, PRESS_CAP_Z + GRASP_UP, HOLD, "hold the pressed clip")]
    wp += [_w(0.9, hx, hy, SEAT_CAP_Z + GRASP_UP, HOLD, "press the barbs home, cap flush")]
    wp += [_w(0.6, hx, hy, SEAT_CAP_Z + GRASP_UP, HOLD, "hold the seated clip")]
    wp += [_w(0.5, hx, hy, SEAT_CAP_Z + GRASP_UP, _OPEN, "release the seated clip")]
    wp += [_w(0.8, hx, hy, SEAT_CAP_Z + GRASP_UP + 0.020, _OPEN, "lift away from the cap")]
    # re-grip the seated cap and pull-test: 5.6 N retention pull held 1 s, then release and go home
    wp += [_w(1.4, hx, hy, SEAT_CAP_Z + GRASP_UP + 0.020, _OPEN, "over the seated cap")]
    wp += [_w(0.8, hx, hy, SEAT_CAP_Z + GRASP_UP, _OPEN, "down to the seated cap")]
    wp += [_w(0.6, hx, hy, SEAT_CAP_Z + GRASP_UP, HOLD, "grip the seated cap")]
    wp += [_w(1.0, hx, hy, SEAT_CAP_Z + GRASP_UP + PULL_RISE, HOLD, "pull up on the cap")]
    wp += [_w(1.6, hx, hy, SEAT_CAP_Z + GRASP_UP + PULL_RISE, HOLD, "hold the pull for 1 s")]
    wp += [_w(0.6, hx, hy, SEAT_CAP_Z + GRASP_UP + PULL_RISE, _OPEN, "release the pull")]
    wp += [_w(0.8, hx, hy, SEAT_CAP_Z + GRASP_UP + 0.030, _OPEN, "lift clear of the panel")]
    wp += [_home_wp("right")]
    return {"right": wp}


def trace(objects) -> str:
    return (f"cap {(np.asarray(objects[NAME].pos)[2] - TOP) * 1000:.1f} mm "
            f"ins {K_CLIP * objects.get('depth', 0):.1f} N pull {K_PULL * objects.get('pull_depth', 0):.1f} N "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('cross_insert','not_seated','dropped','forced_fit') if objects.get(k)) or '-'}")
