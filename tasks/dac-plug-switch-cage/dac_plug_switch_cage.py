"""On the station bench: a short 1U-style switch on a riser, two DAC/SFP port cages, a cable comb over the near
port, and a 40 mm DAC plug staged in a cradle to the left.  The robot picks the plug, brings it tip-first to the
near (target) port, aligns it with the keyed cage, pushes it in until the spring latch clicks, seats the rear
strain-relief boot between the comb teeth, then tugs the plug back to prove the latch holds.

The plug is a real, gravity-free rigid body in its own USD file.  While a gripper is closed on its grasp point the
plug follows the gripper (a declared carry, like the family's detents); once the boot is seated the latch pins it,
so a pull cannot extract it.  The cage, latch notch, comb and boot are real geometry; the scorer reads the
mechanism flags maintained here, exactly the way the Isaac scorer reads them.

Run it:  inspect-robots run --embodiment yam_arena -E task=dac_plug_switch_cage.py --scorer task_score ...
Fast local gate:  cd local_sim && .venv/bin/python -m mjkit.run ../tasks/dac-plug-switch-cage
"""
import importlib.abc
import importlib.util
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

TAG = "dps"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = Path(__file__).resolve().parent / "assets"

# -- switch / ports (local frame; world x = switch origin x + local x, world z = TOP + local z) -------------------------
SW_XY = (0.350, 0.000)
SW_POS = (SW_XY[0], SW_XY[1], TOP)
PORT_X = 0.310                  # the port mouth plane (world x)
PORT_Z = 0.100                  # port centre (local z; world z = TOP + 0.100)
TARGET_Y = 0.015                # near / target port
DECOY_Y = -0.015                # the other port: inserting here caps the score
PORT_W, PORT_H = 0.0145, 0.0095
COMB_X = 0.312                  # the comb slot / boot flange plane (world x)

# -- the plug (local frame: long axis +x, tip +0.020, rear -0.020, origin at centre, local +z up) ------------------------
PLUG_LEN, PLUG_W, PLUG_H = 0.040, 0.0135, 0.0086
PLUG_TIP = 0.020
BOOT_X = -0.018
BOOT_W = 0.012
PLUG_MASS = 0.020
STAGE_XY = (0.250, 0.060)       # staging cradle (world x,y)
STAGE_CZ = 0.100                # plug centre local z when staged

# insertion depth = tip world x - PORT_X.  Centre x = PORT_X - PLUG_TIP + depth = 0.290 + depth.
DEPTH_ALIGN = 0.004             # tip 4 mm into the mouth: "aligned at port"
DEPTH_LATCH = 0.036             # latch tab reaches the cage notch: "latched" (click)
DEPTH_SEAT = 0.040              # boot flange between the comb teeth: "boot seated"
ALIGN_X = PORT_X - PLUG_TIP + DEPTH_ALIGN     # 0.294
LATCH_X = PORT_X - PLUG_TIP + DEPTH_LATCH     # 0.326
SEAT_X = PORT_X - PLUG_TIP + DEPTH_SEAT       # 0.330

# -- tug: grip the boot tail behind the cage mouth (world x ~ 0.301) and draw it back 4 mm -----------------------------
#    The boot tail ends at local x -0.030, i.e. world x 0.300 when seated; gripping there keeps the fingers clear
#    of the cage walls and the comb while still applying the pull to the plug through its strain relief.
BOOT_GRASP_X = SEAT_X - 0.029                 # 0.301
TUG_PULL = 0.004
TUG_X = BOOT_GRASP_X - TUG_PULL               # 0.297

# -- gripper / grasp ---------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 1.0
GRIP_CLOSE = 0.034              # opening at/below which a closed gripper may take the plug
GRIP_RELEASE = 0.045            # opening above which a held plug is let go
GRIP_REACH = 0.016              # m: TCP within this of the grasp point
GRIP_CLEAR = 0.045              # m: no pad within this of the plug at the scored state
HOME_SLACK = 0.020
GRASP_UP = 0.012                # the TCP sits this far above the plug centre when grasping

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


SWITCH = UsdPart("switch", asset="switch", xy=SW_XY, extent=0.20, mass=None, fixed=True,
                 usd_path=str(ASSETS / "switch.usda"), pos=SW_POS)
PLUG = UsdPart("plug", asset="plug", xy=STAGE_XY, extent=0.05, mass=PLUG_MASS,
               usd_path=str(ASSETS / "plug.usda"), pos=tuple(START_POS))
OBJECTS = [SWITCH, PLUG]
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
    if getattr(scene.task_object, "_dps_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        obj = _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)
        # The task writes the switch and plug root poses itself; Arena's float64 reset pose is rejected by
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
            env_cfg.events.dps_reset = EventTermCfg(func=_reset_plug, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("plug")})
            env_cfg.events.dps_step = EventTermCfg(func=_plug_step, mode="interval",
                                                   interval_range_s=(0.0, 0.0), is_global_time=False,
                                                   params={"asset_cfg": SceneEntityCfg("plug")})
            return env_cfg

        return with_events

    task_object._dps_usd_parts = True
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
        self.pos = START_POS.copy()
        self.quat = START_QUAT.copy()
        self.held = None
        self.offset = None
        self.rot_off = None
        # progress milestones (a milestone, once reached, stays reached, like the family's detents)
        self.aligned = False
        self.latched = False
        self.boot_seated = False
        self.tug_held = False
        self.seat_pos = None
        self.seat_quat = None
        # caps
        self.wrong_port = False
        self.forced = False
        self.latch_miss = False
        self.boot_missing = False
        self.tug_failed = False
        # current measurements
        self.depth = 0.0
        self.keyed = False
        self.t = 0.0

    def reset(self):
        self.__init__()


_MECH = _Mechanism()


def _write_plug(env, pos, quat, env_ids):
    import torch

    plug = env.scene["plug"]
    pose = torch.tensor([[pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]],
                        device=plug.device, dtype=torch.float32)
    plug.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    plug.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=plug.device,
                                                                    dtype=torch.float32), env_ids=env_ids)


def _reset_plug(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    _MECH.reset()
    _write_plug(env, START_POS, START_QUAT, env_ids)
    if int(base.episode_length_buf[0]) == 0:
        print(f"[{TAG}] reset: plug staged at x {STAGE_XY[0]}, y {STAGE_XY[1]}; target port y {TARGET_Y}",
              flush=True)


def _grasp_point():
    m = _MECH
    return m.pos + _rot(m.quat) @ np.array([0.0, 0.0, GRASP_UP])


def _plug_step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    m = _MECH
    grips = _gripper_states(env)

    # 1. grasp / carry / release the plug (a declared carry; the latch pins it once the boot is seated)
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
        else:
            m.pos = g.tcp + g.rot @ m.offset
            m.quat = ik.matrix_to_quat_xyzw(g.rot @ m.rot_off)

    # 2. mechanism measurements from the plug's real pose
    R = _rot(m.quat)
    axis, up = R[:, 0], R[:, 2]
    keyed = bool(axis[0] > 0.96 and up[2] > 0.90)   # tip first (+x), latch tab up (+z)
    tip = m.pos + R @ np.array([PLUG_TIP, 0.0, 0.0])
    depth = float(tip[0] - PORT_X)
    dy_t, dy_d = float(tip[1] - TARGET_Y), float(tip[1] - DECOY_Y)
    dz = float(tip[2] - (TOP + PORT_Z))
    in_target = abs(dy_t) < 0.006 and abs(dz) < 0.005
    in_decoy = abs(dy_d) < 0.006 and abs(dz) < 0.005
    m.depth, m.keyed = depth, keyed

    # 3. progress milestones and physical faults
    if keyed and in_target and 0.0 <= depth <= DEPTH_ALIGN + 0.008:
        m.aligned = True
    if in_decoy and depth > 0.010:
        m.wrong_port = True
    if depth > 0.014 and not keyed:
        m.forced = True
    if depth >= DEPTH_LATCH - 0.002:
        if keyed and in_target:
            m.latched = True
        elif in_decoy:
            m.wrong_port = True
        elif not keyed:
            m.latch_miss = True
    if m.latched and in_target and depth >= DEPTH_SEAT - 0.001 and abs(dy_t) < 0.003 and abs(dz) < 0.003 and keyed:
        m.boot_seated = True

    # once the boot is seated the latch pins the plug: a pull cannot back it out of the cage
    if m.boot_seated:
        if m.seat_pos is None:
            m.seat_pos = m.pos.copy()
            m.seat_quat = m.quat.copy()
        m.pos = m.seat_pos.copy()
        m.quat = m.seat_quat.copy()

    # 4. the tug test: grip the boot flange and draw it back TUG_PULL; the latch must keep the plug seated.
    #    The gripper must have let go of the plug (m.held is None) first, so merely carrying the plug past the
    #    boot's x on the way into the cage cannot be mistaken for a tug.
    tug_pt = np.array([TUG_X, TARGET_Y, TOP + PORT_Z + GRASP_UP])
    pulling = False
    if m.held is None:
        for side in ("left", "right"):
            g = grips[f"{side}_gripper"]
            if g.opening <= GRIP_CLOSE and float(np.linalg.norm(g.tcp - tug_pt)) <= 0.006:
                pulling = True
    if pulling:
        if not m.boot_seated:
            m.boot_missing = True
        else:
            m.tug_held = True
    if m.tug_held and depth < DEPTH_LATCH - 0.008:
        m.tug_failed = True

    _write_plug(env, m.pos, m.quat, env_ids)
    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={m.t:.2f}s depth {depth * 1000:5.1f}mm keyed {int(keyed)} "
              f"steps {int(m.aligned)}{int(m.latched)}{int(m.boot_seated)}{int(m.tug_held)} "
              f"at {np.round(m.pos, 4)}"
              f"{'  WRONG-PORT' if m.wrong_port else ''}{'  FORCED' if m.forced else ''}"
              f"{'  LATCH-MISS' if m.latch_miss else ''}{'  BOOT-MISSING' if m.boot_missing else ''}"
              f"{'  TUG-FAIL' if m.tug_failed else ''}", flush=True)
    m.t += float(base.step_dt)


def extra_states(env, out, origin):
    m = _MECH
    out["plug"] = ObjectState(pos=m.pos.copy(), quat_xyzw=m.quat.copy(), lin_vel=np.zeros(3),
                              ang_vel=np.zeros(3), low=m.pos.copy(), high=m.pos.copy())
    for k in ("aligned", "latched", "boot_seated", "tug_held", "wrong_port", "forced", "latch_miss",
              "boot_missing", "tug_failed", "depth", "keyed", "held"):
        out[k] = getattr(m, k)


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("aligned at target port", "latched / click", "boot seated in comb", "tug holds")
POINTS = 0.25
CAP_KEYS = ("wrong_port", "forced", "latch_miss", "boot_missing", "tug_failed")


def steps_done(objects) -> tuple[bool, ...]:
    s0 = bool(objects.get("aligned"))
    s1 = s0 and bool(objects.get("latched"))
    s2 = s1 and bool(objects.get("boot_seated"))
    s3 = s2 and bool(objects.get("tug_held"))
    return (s0, s1, s2, s3)


def _caps(objects) -> bool:
    return any(bool(objects.get(k)) for k in CAP_KEYS)


def _pad_points(g) -> list:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _clear(objects) -> bool:
    axis = np.array([SEAT_X - 0.010, TARGET_Y, TOP + PORT_Z])
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.opening < GRIP_RELEASE or g.contact >= GRIP_FORCE:
            return False
        for pt in _pad_points(g):
            if float(np.linalg.norm(pt - axis)) < GRIP_CLEAR + 0.05:
                return False
    return True


def _home(objects, side) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


def _full(objects) -> bool:
    return bool(all(steps_done(objects)) and not _caps(objects) and _clear(objects)
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
_DOWN = yam_demo.grip_rotation(0.0)   # the plug keeps its world pose (tip +x) under the declared carry
_OPEN = FINGER_OPEN


def _w(dur, cx, cy, cz, finger, label, rot=_DOWN):
    """Waypoint from the plug-centre target (x,y,local z); the TCP sits GRASP_UP above it."""
    return Waypoint(dur, (float(cx), float(cy), TOP + float(cz) + GRASP_UP), finger, label, rot=rot)


def _hover(cx, cy, cz, finger, label, rot=_DOWN):
    return _w(1.4, cx, cy, cz, finger, label, rot)


def demo():
    """The left arm picks the plug, aligns it at the near cage, pushes until the latch clicks, seats the boot in
    the comb, then draws the boot back to tug-test the latch.  The right arm stays home.  All approaches are from
    above; the plug's world pose is preserved by the carry, so it goes in tip-first and latch-up."""
    hold = -0.010
    y = TARGET_Y
    left = []
    # A. pick the plug from its staging cradle
    left += [_hover(STAGE_XY[0], STAGE_XY[1], STAGE_CZ + 0.040, _OPEN, "over the staged plug"),
             _w(1.2, STAGE_XY[0], STAGE_XY[1], STAGE_CZ, _OPEN, "down to the plug"),
             _w(0.5, STAGE_XY[0], STAGE_XY[1], STAGE_CZ, _OPEN, "settle at the plug"),
             _w(0.7, STAGE_XY[0], STAGE_XY[1], STAGE_CZ, hold, "grip the plug"),
             _w(1.4, STAGE_XY[0], STAGE_XY[1], STAGE_CZ + 0.040, hold, "lift the plug")]
    # B. carry to the target port and hold it aligned with the keyed cage
    left += [_hover(ALIGN_X, y, PORT_Z, hold, "carry plug to the port"),
             _w(1.2, ALIGN_X, y, PORT_Z, hold, "align plug at the port"),
             _w(0.6, ALIGN_X, y, PORT_Z, hold, "hold alignment")]
    # C. push in until the latch clicks into the cage notch
    left += [_w(1.4, LATCH_X, y, PORT_Z, hold, "push plug in / latch clicks"),
             _w(0.6, LATCH_X, y, PORT_Z, hold, "hold the latch")]
    # D. final straight push: the boot flange seats between the comb teeth
    left += [_w(1.2, SEAT_X, y, PORT_Z, hold, "seat boot in the comb"),
             _w(0.6, SEAT_X, y, PORT_Z, hold, "hold the seated boot"),
             _w(0.6, SEAT_X, y, PORT_Z, _OPEN, "release the plug"),
             _w(0.8, SEAT_X, y, PORT_Z + 0.040, _OPEN, "lift away from the plug")]
    # E. tug the boot back 4 mm: the latch must keep the plug in the cage
    left += [_hover(BOOT_GRASP_X, y, PORT_Z, _OPEN, "over the boot flange"),
             _w(1.0, BOOT_GRASP_X, y, PORT_Z, _OPEN, "down to the boot"),
             _w(0.6, BOOT_GRASP_X, y, PORT_Z, hold, "grip the boot"),
             _w(1.0, TUG_X, y, PORT_Z, hold, "tug the plug"),
             _w(0.8, TUG_X, y, PORT_Z, hold, "hold the tug"),
             _w(0.6, TUG_X, y, PORT_Z, _OPEN, "release the boot"),
             _w(0.8, TUG_X, y, PORT_Z + 0.040, _OPEN, "lift away from the boot")]
    left += [Waypoint(2.4, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]
    right = [Waypoint(2.4, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"])]
    return {"left": left, "right": right}


def trace(objects) -> str:
    return (f"depth {objects.get('depth', 0.0) * 1000:5.1f}mm keyed {int(bool(objects.get('keyed')))} "
            f"steps {''.join('1' if x else '0' for x in steps_done(objects))}, "
            f"caps {'/'.join(k for k in CAP_KEYS if objects.get(k)) or '-'}")
