"""On the station table: a steel machine-tool spindle nose (65 mm diameter, with a shoulder at its base) at
x 0.30 y 0, and an 80 mm magnetic dust boot (oil-resistant NBR elastomer with a moulded-in magnet ring) staged in
a pocket at x 0.25 y -0.06.  The robot picks the boot up, brings its 65 mm bore concentrically over the nose
(the embedded magnets must centre, not grab an off-centre bite of the steel rim), presses it down with a 4-7 N
hand force until its bottom face seats against the shoulder, releases it, and leaves it seated with both
grippers clear and home.

The boot is a real gravity-free rigid body in its own USD file.  While a gripper's fingers close across the
boot's grasp point the boot follows the gripper (a declared carry, like the family's detents).  The concentric
alignment, the press stroke, the seat against the shoulder and all three failure caps (magnets grab off-centre,
boot not seated, forced press past the shoulder) are real geometry read by the scorer exactly the way the Isaac
scorer reads them.  A 1 s hold of the final, seated, gripper-free state completes the task.

Run it:  inspect-robots run --embodiment yam_arena -E task=magnetic_dust_boot.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../tasks/magnetic-dust-boot
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

TAG = "mdb"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- spindle fixture and boot (local frame: spindle axis at x,y = 0, world z = TOP + local z) -----------------------------
SPINDLE_XY = (0.300, 0.000)
SPINDLE_POS = (SPINDLE_XY[0], SPINDLE_XY[1], TOP)
NOSE_D = 0.065                   # real 65 mm spindle nose diameter
SHOULDER_Z = 0.050               # local z of the shoulder face the boot seats against
NOSE_TOP_Z = 0.095               # local z of the spindle nose top
BOOT_OD = 0.080                  # real 80 mm boot outer diameter
BOOT_BORE = 0.065                # boot bore fits the 65 mm nose
BOOT_H = 0.040                   # 40 mm boot body height
BOOT_CZ_SEAT = SHOULDER_Z + BOOT_H / 2        # 0.070: boot centre when its bottom is on the shoulder
GRASP_UP = 0.026                 # TCP sits at the small central grasp boss (boot local z +0.026) when grasping
BOOT_STAGE = (0.250, -0.060)
BOOT_STAGE_CZ = BOOT_H / 2                    # 0.020: boot resting on the pocket base
TRAVEL_Z = 0.150                 # high carry height, clear of the nose and inside the arm's IK reach
APPROACH_CZ = BOOT_CZ_SEAT + 0.035            # 0.105: boot centre at the top of the press stroke
SEAT_TCP_Z = BOOT_CZ_SEAT + GRASP_UP          # 0.090: TCP local z when the boot is seated
APPROACH_TCP_Z = APPROACH_CZ + GRASP_UP       # 0.125: TCP local z at the start of the press

# -- alignment / press / seat -------------------------------------------------------------------------------------------
ALIGN_TOL = 0.006                # 6 mm: bore centre must be this close to the spindle axis to be concentric
SEAT_Z_TOL = 0.005               # 5 mm: boot bottom must be this close to the shoulder to be seated
OVERPRESS_TOL = 0.004            # driving the boot >4 mm past the shoulder is a forced press
PRESS_STROKE = APPROACH_CZ - BOOT_CZ_SEAT     # 0.035 m rated press stroke
PRESS_LOCK_FRAC = 0.90           # fraction of the stroke that counts as a full press
PRESS_XY = (0.300, 0.000)
PRESS_R = 0.030                  # XY window for a press
PRESS_Z_LO = SEAT_TCP_Z - 0.006
PRESS_Z_HI = APPROACH_TCP_Z + 0.006
FORCE_N = (4.0, 7.0)             # rated 4-7 N hand press (stated, see REFERENCE.md)

# -- gripper / grasp -----------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0              # N: a real closed grip
GRIP_CLOSE = 0.034            # opening at/below which a closed finger pair may take a part
GRIP_RELEASE = 0.045          # opening above which a held part is let go
GRIP_REACH = 0.016            # m: TCP within this of the part's grasp point
GRIP_CLEAR = 0.045            # m: no pad within this of the work area at the scored state
HOME_SLACK = 0.020
HOLD = -0.012                 # finger target for carrying the boot
GRASP_UP_TCP = GRASP_UP       # (alias used by the tests)

START_POS = {"boot": np.array([BOOT_STAGE[0], BOOT_STAGE[1], TOP + BOOT_STAGE_CZ])}
START_QUAT = {"boot": np.array([0.0, 0.0, 0.0, 1.0])}


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self):
        return not self.fixed


SPINDLE = UsdPart("spindle", asset="spindle", xy=SPINDLE_XY, extent=0.12, mass=None, fixed=True,
                  usd_path=str(ASSETS / "spindle.usda"), pos=SPINDLE_POS)
BOOT = UsdPart("boot", asset="boot", xy=BOOT_STAGE, extent=0.08, mass=0.090,
               usd_path=str(ASSETS / "boot.usda"), pos=tuple(START_POS["boot"]))
OBJECTS = [SPINDLE, BOOT]
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
    if getattr(scene.task_object, "_mdb_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        obj = _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)
        # The task writes the boot/spindle root poses itself; Arena's float64 reset pose is rejected by
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
            env_cfg.events.mdb_reset = EventTermCfg(func=_reset_boot, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("spindle")})
            env_cfg.events.mdb_step = EventTermCfg(func=_boot_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("spindle")})
            return env_cfg

        return with_events

    task_object._mdb_usd_parts = True
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
        self.pos = START_POS["boot"].copy()
        self.quat = START_QUAT["boot"].copy()
        self.held = None
        self.offset = {}
        self.rot_off = {}
        # progress latches
        self.aligned = False
        self.pressed = False
        self.seated = False
        self.released = False
        # caps
        self.off_centre = False
        self.not_seated = False
        self.forced_press = False
        # current state
        self.pressing = False
        self.press_depth = 0.0
        self.dxy = 0.0
        self.bottom_z = 0.0
        self.centred = False
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


def _reset_boot(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_part(env, "boot", START_POS["boot"], START_QUAT["boot"], env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: 80 mm magnetic dust boot staged at x 0.25 y -0.06, spindle nose bare", flush=True)


def _grasp_point():
    m = _MECH
    return m.pos + _rot(m.quat) @ np.array([0.0, 0.0, GRASP_UP])


def _boot_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)

    # live boot geometry
    m.dxy = float(np.linalg.norm(m.pos[:2] - np.asarray(SPINDLE_XY)))
    m.centred = m.dxy <= ALIGN_TOL
    m.bottom_z = float(m.pos[2] - TOP - BOOT_H / 2)          # local z of the boot bottom face
    on_shoulder = abs(m.bottom_z - SHOULDER_Z) <= SEAT_Z_TOL
    in_zone = m.dxy <= PRESS_R + 0.02

    side = "left"
    g = grips[f"{side}_gripper"]

    # 1. grasp / carry / release the boot
    if m.held is None:
        if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - _grasp_point())) <= GRIP_REACH:
            m.held = side
            m.offset = g.rot.T @ (m.pos - g.tcp)
            m.rot_off = g.rot.T @ _rot(m.quat)
    else:
        gh = grips[f"{m.held}_gripper"]
        if gh.opening > GRIP_RELEASE:
            m.held = None
            # a release is good only if the boot is seated concentrically against the shoulder
            if on_shoulder and m.centred:
                m.released = True
            else:
                if not on_shoulder:
                    m.not_seated = True
                if in_zone and not m.centred:
                    m.off_centre = True
        else:
            m.pos = gh.tcp + gh.rot @ m.offset
            m.quat = ik.matrix_to_quat_xyzw(gh.rot @ m.rot_off)

    # 2. concentric alignment: the bore centred over the nose while approaching at low height
    if m.held is not None and m.centred and m.bottom_z >= SHOULDER_Z and m.bottom_z <= SHOULDER_Z + 0.045:
        m.aligned = True

    # 3. press: a real closed-grip downward stroke at the boot, over the spindle axis
    press = False
    press_depth = 0.0
    if m.held is not None:
        gh = grips[f"{m.held}_gripper"]
        zloc = float(gh.tcp[2] - TOP)
        if (gh.opening <= GRIP_CLOSE and float(np.linalg.norm(gh.tcp[:2] - np.asarray(PRESS_XY))) <= PRESS_R
                and PRESS_Z_LO <= zloc <= PRESS_Z_HI):
            press = True
            press_depth = float(np.clip((APPROACH_TCP_Z - zloc) / PRESS_STROKE, 0.0, 1.0))
    m.pressing = bool(press and press_depth > 0.25)
    m.press_depth = press_depth if m.pressing else 0.0
    if m.pressing:
        if not m.centred:
            m.off_centre = True                     # the magnets grabbed an off-centre bite of the rim
        if m.bottom_z <= SHOULDER_Z - OVERPRESS_TOL:
            m.forced_press = True                   # jammed past the shoulder (through the solid stop)
        if press_depth >= PRESS_LOCK_FRAC and m.centred and m.bottom_z >= SHOULDER_Z - OVERPRESS_TOL:
            m.pressed = True

    # 4. seat: the boot bottom face against the shoulder, bore concentric with the nose
    if on_shoulder and m.centred:
        m.seated = True

    # 5. write the boot free body
    _write_part(env, "boot", m.pos, m.quat, env_ids)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s boot at {np.round(m.pos,4)} bottom z {m.bottom_z:.4f} "
              f"dxy {m.dxy:.4f} {'CENTRED' if m.centred else '---'} "
              f"press {m.press_depth:4.2f} steps {int(m.aligned)}{int(m.pressed)}{int(m.seated)}{int(m.released)}"
              f"{'  OFFCENTRE' if m.off_centre else ''}{'  NOTSEATED' if m.not_seated else ''}"
              f"{'  FORCED' if m.forced_press else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    out["boot"] = ObjectState(pos=m.pos.copy(), quat_xyzw=m.quat.copy(),
                              lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                              low=m.pos.copy(), high=m.pos.copy())
    for key in ("aligned", "pressed", "seated", "released", "off_centre", "not_seated", "forced_press",
                "pressing", "press_depth", "dxy", "bottom_z", "centred"):
        out[key] = getattr(m, key)
    out["held"] = m.held


# -- scoring -------------------------------------------------------------------------------------------------------------
STEPS = ("boot aligned concentrically on the nose", "press applied against the shoulder",
         "boot seated against the shoulder", "released and held")
POINTS = 0.25


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("aligned"))
    s1 = s0 and bool(objects.get("pressed"))
    s2 = s1 and bool(objects.get("seated"))
    s3 = s2 and bool(objects.get("released"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return bool(objects.get("off_centre") or objects.get("not_seated") or objects.get("forced_press"))


def _current_ok(objects) -> bool:
    """The boot is left seated and centred as things stand (not just a past latch)."""
    return bool(objects.get("seated") and objects.get("centred") and objects.get("released") and not _caps(objects))


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([SPINDLE_XY[0], SPINDLE_XY[1], TOP + 0.070])
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


# -- demo ----------------------------------------------------------------------------------------------------------------
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


def _w(dur, x, y, z, finger, label, rot=_DOWN):
    """Waypoint from local z (world z = TOP + z)."""
    return Waypoint(dur, (x, y, TOP + z), finger, label, rot=rot)


def _hover(x, y, z, finger, label, rot=_DOWN):
    return _w(1.6, x, y, z, finger, label, rot)


def _pick(centre, cz, label):
    """Open over the part, descend, close on it, lift clear."""
    x, y = centre
    gz = cz + GRASP_UP
    return [_hover(x, y, gz + 0.045, _OPEN, "over the " + label),
            _w(1.2, x, y, gz, _OPEN, "down to the " + label),
            _w(0.5, x, y, gz, _OPEN, "settle at the " + label),
            _w(0.7, x, y, gz, HOLD, "grip the " + label),
            _w(1.4, x, y, gz + 0.045, HOLD, "lift the " + label)]


def demo():
    """The left arm picks the boot, centres its bore on the spindle nose, presses it onto the shoulder,
    releases it and goes home.  The right arm stays home.  All approaches are from above with the fingers
    closing across the part."""
    left = []
    # A. pick the boot from its staging pocket
    left += _pick(BOOT_STAGE, BOOT_STAGE_CZ, "boot")
    # B. carry at high altitude to the spindle, then descend concentrically: the magnets must centre, not grab
    left += [_hover(BOOT_STAGE[0], BOOT_STAGE[1], TRAVEL_Z, HOLD, "lift the boot clear"),
             _w(1.2, SPINDLE_XY[0], SPINDLE_XY[1], TRAVEL_Z, HOLD, "carry the boot to the spindle"),
             _w(1.2, SPINDLE_XY[0], SPINDLE_XY[1], APPROACH_TCP_Z, HOLD, "descend over the spindle axis"),
             _w(0.8, SPINDLE_XY[0], SPINDLE_XY[1], APPROACH_TCP_Z, HOLD, "align the magnets concentrically")]
    # C. press the boot down through the rated stroke until its bottom seats against the shoulder, then release
    left += [_w(1.4, SPINDLE_XY[0], SPINDLE_XY[1], SEAT_TCP_Z, HOLD, "press the boot onto the nose"),
             _w(0.8, SPINDLE_XY[0], SPINDLE_XY[1], SEAT_TCP_Z, HOLD, "hold the boot on the shoulder"),
             _w(0.6, SPINDLE_XY[0], SPINDLE_XY[1], SEAT_TCP_Z, _OPEN, "release the seated boot"),
             _w(0.8, SPINDLE_XY[0], SPINDLE_XY[1], TRAVEL_Z, _OPEN, "lift away from the spindle")]
    left += [Waypoint(2.2, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]
    right = [Waypoint(2.2, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"dxy {objects.get('dxy', 0):.4f} bottom {objects.get('bottom_z', 0):.4f} "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in ('off_centre','not_seated','forced_press') if objects.get(k)) or '-'}")
