"""On the station bench: a Dyson V6-class cordless stick vacuum's motor/body module with its 0.4 L clear dust bin
seated on the front. The red bin-release button on the top front is sprung (about 6 N over 10 mm). The robot presses
it, pulls the clear bin out of its cradle, carries it over the waste bin, empties the dust through the bottom dump
flap, then refits the bin and lets the button click back. The vac must stay put; the dust must all land in the waste
bin; the bin must be latched and both hands clear for 1 s.

The vac is one articulation (Body free on four feet, the Eject button a prismatic link). The bin is a second free
articulation (Can + a hinged bottom dump flap). Six pieces of debris and the waste bin are free/fixed bodies.

Run it:  inspect-robots run --embodiment yam_arena -E task=stick_vac_bin_eject.py --scorer task_score ...
"""
import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from types import SimpleNamespace
from pathlib import Path

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

TAG = "stickvac"
SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself times the 1 s full-score hold
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001

# -- real product and mechanism (metres; mirror build_assets.py) ------------------------------------------------------
VAC_XY = (0.36, 0.0)
VAC_POS = (*VAC_XY, TOP)
VAC_MASS = 2.04                       # Dyson V6 Cord-free published weight
VAC_TILT_MAX, VAC_MOVE_MAX = math.radians(8.0), 0.025
BIN_XY = (0.290, 0.0)                 # seated bin centre (x,y)
BIN_BOTTOM = TOP + 0.016              # cradle shelf top; bin root = bottom face centre
BIN_START = (*BIN_XY, BIN_BOTTOM)
BIN_L, BIN_W, BIN_H = 0.070, 0.070, 0.085
BIN_MASS = 0.14                       # empty bin (the 0.06 kg dust is separate debris)
EJECT_KNOB = np.array([0.292, 0.0, TOP + 0.215])
EJECT_PRESS = 0.010                    # 10 mm stop
EJECT_TRIG = 0.005                     # pressed past 5 mm = bin released
EJECT_SHUT = 0.001                     # back within 1 mm = clicked shut
FLAP_OPEN = math.radians(-100.0)
FLAP_SHUT = math.radians(-2.0)
WASTE_XY = (0.30, 0.22)
WASTE_W, WASTE_H = 0.240, 0.035
WASTE_R_IN = WASTE_W / 2 - 0.010      # inner half-width for the debris test

# -- scoring geometry ------------------------------------------------------------------------------------------------
BIN_OUT = 0.050                       # bin root this far from its seat = pulled out
SEAT_OFF, SEAT_TILT = 0.012, math.radians(8.0)
CLEAR = 0.045                          # both grippers this far from vac/bin at the scored state
STILL_T, STILL_MOVE, STILL_TURN = 1.0, 0.003, math.radians(2.0)
STABLE_TILT, STABLE_MOVE = math.radians(2.0), 0.010
DEBRIS_TRAY_MARGIN = 0.002

# -- gripper ----------------------------------------------------------------------------------------------------------
PAD_BACK = 0.025
GRIP_FORCE = 0.5                      # N of commanded grip that counts as a hold
GRIP_OPEN = (0.006, 0.096)            # button knob 14 mm / bin ribs 88 mm both count
EJECT_FINGER = -0.007                 # 14 mm opening on the 14 mm knob
BIN_FINGER = -0.034                   # 68 mm opening: pads press the bin side walls at its centre
OPEN_FINGER = FINGER_OPEN
BIN_REACH = 0.020                     # TCP within this of the bin/root or knob counts as handling

JOINTS = ("EjectPress",)
BIN_JOINTS = ("FlapHinge",)


@dataclass(frozen=True)
class UsdPart(Asset):
    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


VAC = UsdPart("vac", asset="vac", xy=VAC_XY, extent=0.50, mass=VAC_MASS,
              usd_path=str(ASSETS / "vac.usda"), pos=VAC_POS, articulation=True)
BIN = UsdPart("bin", asset="bin", xy=BIN_XY, extent=0.10, mass=BIN_MASS,
              usd_path=str(ASSETS / "bin.usda"), pos=BIN_START, articulation=True)
WASTE = UsdPart("waste", asset="waste", xy=WASTE_XY, extent=0.20, mass=0.30,
                usd_path=str(ASSETS / "waste_bin.usda"), pos=(WASTE_XY[0], WASTE_XY[1], TOP), kinematic=True)

_DEBRIS_KINDS = (("debris_pebble", 0.010), ("debris_chip", 0.004), ("debris_twig", 0.003))
_GRID = [(-0.020, -0.018), (0.020, -0.018), (-0.020, 0.018), (0.020, 0.018), (0.0, 0.0), (0.0, -0.018)]
DEBRIS = []
for _i, ((_kind, _mass), (_dx, _dy)) in enumerate(zip(_DEBRIS_KINDS * 2, _GRID, strict=True)):
    DEBRIS.append(UsdPart(f"{_kind}_{_i:02d}", asset=_kind, xy=(BIN_XY[0] + _dx, BIN_XY[1] + _dy),
                          extent=0.03, mass=_mass, usd_path=str(ASSETS / f"{_kind}.usda"),
                          pos=(BIN_XY[0] + _dx, BIN_XY[1] + _dy, BIN_BOTTOM + 0.007 + 0.004 * (_i % 2))))
OBJECTS = [VAC, BIN, WASTE, *DEBRIS]
MOVERS = ("vac", "bin")
SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2


def _spawn(scene, spec):
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False)
    return scene.Object(
        name=spec.name, usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
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
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)
        out[f"{side}_gripper"] = GripperState(
            tcp=tcp, back=tcp - PAD_BACK * axis_dir(rot), tips=tcp + PAD_TIP_PAST_TCP * axis_dir(rot),
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
            rot=np.asarray(rot, float))
    return out


def axis_dir(rot):
    return np.asarray(rot)[:, 2]


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
        extra_states(base, out)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.stickvac_reset = EventTermCfg(func=_reset, mode="reset",
                                                         params={"asset_cfg": SceneEntityCfg("vac")})
            env_cfg.events.stickvac_step = EventTermCfg(func=_step, mode="interval",
                                                        interval_range_s=(0.0, 0.0), is_global_time=False,
                                                        params={"asset_cfg": SceneEntityCfg("vac")})
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


# -- joint / pose helpers --------------------------------------------------------------------------------------------
def _joint_ids(art, names):
    return art.find_joints(list(names), preserve_order=True)[0]


def _set_targets(art, names, values) -> None:
    import torch

    view = art.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp
        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    dof = list(view.shared_metatype.dof_names)
    for name, v in zip(names, values, strict=True):
        t[:, dof.index(name)] = float(v)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, art._ALL_INDICES)


def _pose(asset, body: str | None = None):
    if body is None:
        p = _arr(asset.data.root_link_pose_w)
    else:
        idx = asset.find_bodies(body)[0][0]
        p = _arr(asset.data.body_link_pose_w)[idx]
    return p[:3], p[3:]


def _rot(quat) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat, float))


def _knob_world(vac) -> np.ndarray:
    p, q = _pose(vac, "Eject")
    return p + _rot(q) @ np.array([-0.010, 0.0, 0.0])


class _Mechanism:
    def __init__(self):
        self.t = 0.0
        self.vac0 = None
        self.opened_once = False
        self.removed = False
        self.emptied = False
        self.clicked = False
        self.forced = False
        self.dropped = False
        self.tipped = False
        self.debris_left = False
        self.dump_open = False
        self.prev_eject = 0.0
        self.traced = -1.0
        self.reported = [False] * 4

    def reset(self):
        self.__init__()


_MECH = _Mechanism()


def _reset(env, env_ids, asset_cfg=None):
    import torch

    if env_ids is None:
        return
    vac = env.scene["vac"]
    eids = _joint_ids(vac, JOINTS)
    q = torch.zeros((len(env_ids), len(eids)), device=vac.device, dtype=torch.float32)
    vac.write_joint_position_to_sim_index(position=q, joint_ids=eids, env_ids=env_ids)
    vac.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=eids, env_ids=env_ids)
    vp = torch.tensor([[VAC_POS[0], VAC_POS[1], VAC_POS[2] + 0.001, 0.0, 0.0, 0.0, 1.0]],
                      device=vac.device, dtype=torch.float32)
    vac.write_root_pose_to_sim_index(root_pose=vp, env_ids=env_ids)
    vac.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=vac.device), env_ids=env_ids)
    _set_targets(vac, JOINTS, [0.0])

    bin_ = env.scene["bin"]
    bids = _joint_ids(bin_, BIN_JOINTS)
    bq = torch.zeros((len(env_ids), len(bids)), device=bin_.device, dtype=torch.float32)
    bin_.write_joint_position_to_sim_index(position=bq, joint_ids=bids, env_ids=env_ids)
    bin_.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(bq), joint_ids=bids, env_ids=env_ids)
    bp = torch.tensor([[BIN_START[0], BIN_START[1], BIN_START[2] + 0.001, 0.0, 0.0, 0.0, 1.0]],
                      device=bin_.device, dtype=torch.float32)
    bin_.write_root_pose_to_sim_index(root_pose=bp, env_ids=env_ids)
    bin_.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=bin_.device), env_ids=env_ids)
    _set_targets(bin_, BIN_JOINTS, [0.0])

    for spec in DEBRIS:
        tool = env.scene[spec.name]
        p = spec.pos
        tp = torch.tensor([[p[0], p[1], p[2] + 0.001, 0.0, 0.0, 0.0, 1.0]],
                          device=tool.device, dtype=torch.float32)
        tool.write_root_pose_to_sim_index(root_pose=tp, env_ids=env_ids)
        tool.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=tool.device), env_ids=env_ids)
    _MECH.reset()
    p, q0 = _pose(vac)
    _MECH.vac0 = (p.copy(), q0.copy())


def _pad_points(g) -> list[np.ndarray]:
    if g is None:
        return []
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _grips(g) -> bool:
    return bool(g is not None and g.contact >= GRIP_FORCE and GRIP_OPEN[0] <= g.opening <= GRIP_OPEN[1])


def _grips_near(g, point, radius=BIN_REACH) -> bool:
    return _grips(g) and float(np.linalg.norm(g.tcp - np.asarray(point, float))) <= radius


def _in_tray(p) -> bool:
    x, y, z = np.asarray(p, float)
    return bool(abs(x - WASTE_XY[0]) <= WASTE_R_IN - DEBRIS_TRAY_MARGIN
                and abs(y - WASTE_XY[1]) <= WASTE_R_IN - DEBRIS_TRAY_MARGIN
                and TOP + 0.010 <= z <= TOP + WASTE_H + 0.020)


def debris_in_waste(objects) -> bool:
    ds = [objects[s.name] for s in DEBRIS]
    return bool(ds and all(_in_tray(d.pos) for d in ds))


def _debris_still_in_bin(objects) -> bool:
    bp, bq = objects["bin"].pos, objects["bin"].quat_xyzw
    R = _rot(bq).T
    for spec in DEBRIS:
        q = R @ (np.asarray(objects[spec.name].pos, float) - np.asarray(bp, float))
        if abs(q[0]) <= BIN_L / 2 + 0.006 and abs(q[1]) <= BIN_W / 2 + 0.006 and -0.010 <= q[2] <= BIN_H + 0.010:
            return True
    return False


def _step(env, env_ids, asset_cfg=None):
    base = env.unwrapped
    vac = env.scene["vac"]
    bin_ = env.scene["bin"]
    grips = _gripper_states(env)
    ej = float(vac.data.joint_pos.torch[0, _joint_ids(vac, JOINTS)[0]])
    bp, bq = _pose(bin_)
    objects = _states_from_env(env)

    # the declared bottom dump: the flap opens when the held bin is over the waste bin (the real bin's red bottom tab)
    waste_dist = float(np.linalg.norm(bp[:2] - np.asarray(WASTE_XY, float)))
    over_waste = bool(waste_dist <= 0.100 and bp[2] >= TOP + 0.10)
    carry_bar = bp + _rot(bq) @ np.array([0.0, 0.0, BIN_H / 2])
    held = any(_grips_near(grips[f"{s}_gripper"], carry_bar) for s in ("left", "right"))
    # The flap is the declared bottom dump: open only while dust is still inside the held bin over the waste, and
    # shut again the moment the bin is empty, BEFORE lifting it out (an open flap catches the waste-bin rim).
    dust_out = debris_in_waste(objects)
    if over_waste and held and not dust_out:
        _MECH.dump_open = True
    if dust_out or _MECH.emptied:
        _MECH.dump_open = False
    flap_target = FLAP_OPEN if _MECH.dump_open else 0.0
    _set_targets(bin_, BIN_JOINTS, [flap_target])

    # progress latches
    any_grip = [grips[f"{s}_gripper"] for s in ("left", "right")]
    eject_hand = any(_grips_near(g, _knob_world(vac)) for g in any_grip)
    if ej >= EJECT_TRIG and eject_hand:
        _MECH.opened_once = True
    dist = float(np.linalg.norm(bp[:2] - np.asarray(BIN_XY, float)))
    if _MECH.opened_once and (dist >= BIN_OUT or bp[2] >= BIN_BOTTOM + BIN_OUT):
        _MECH.removed = True
    if _MECH.removed and debris_in_waste(objects):
        _MECH.emptied = True
    # click: the button spring returns after having been pressed, with the bin back in its seat
    seated = bool(dist <= SEAT_OFF and abs(bp[2] - BIN_BOTTOM) <= SEAT_OFF)
    if _MECH.removed and seated and ej <= EJECT_SHUT:
        _MECH.clicked = True                     # bin back in its cradle with the button released = clicked latched
    # caps
    if not _MECH.opened_once and (dist >= BIN_OUT or bp[2] >= BIN_BOTTOM + BIN_OUT):
        _MECH.forced = True                      # bin pulled with the latch still shut
    if _MECH.removed and seated and ej > EJECT_TRIG:
        _MECH.forced = True                      # refitted while the button is still held down (no click)
    if bp[2] < TOP - 0.020 or (dist > 0.22 and bp[2] < TOP + 0.05):
        _MECH.dropped = True
    if _MECH.removed and seated and _debris_still_in_bin(objects):
        _MECH.debris_left = True
    if _MECH.vac0 is not None:
        vp, vq = _pose(vac)
        tilt = _tilt(vq)
        move = float(np.linalg.norm(vp - _MECH.vac0[0]))
        if tilt > VAC_TILT_MAX or move > VAC_MOVE_MAX or vp[2] < TOP - 0.02:
            _MECH.tipped = True
    _MECH.prev_eject = ej
    _MECH.t += float(base.step_dt)

    if int(base.episode_length_buf[0]) % 40 == 0:
        print(f"[{TAG}] t={_MECH.t:.2f}s eject {ej*1000:.1f} mm, bin d {dist*1000:.0f} mm z {bp[2]:.3f}, "
              f"flap {math.degrees(float(bin_.data.joint_pos.torch[0, _joint_ids(bin_, BIN_JOINTS)[0]])):.0f} deg, "
              f"dust {'in' if _debris_still_in_bin(objects) else 'OUT'}, "
              f"{'FORCED ' if _MECH.forced else ''}{'DROP ' if _MECH.dropped else ''}"
              f"{'TIP' if _MECH.tipped else ''}", flush=True)


def _states_from_env(env):
    """Rebuild a small objects dict for the debris helpers inside the step event."""
    out = {"bin": SimpleNamespace(pos=_pose(env.scene["bin"])[0], quat_xyzw=_pose(env.scene["bin"])[1])}
    for spec in DEBRIS:
        p, q = _pose(env.scene[spec.name])
        out[spec.name] = SimpleNamespace(pos=p, quat_xyzw=q)
    return out


def _tilt(quat) -> float:
    R = _rot(quat)
    return math.acos(min(1.0, max(-1.0, float(R[2, 2]))))


def extra_states(env, out, *args):
    vac = env.scene["vac"]
    bin_ = env.scene["bin"]
    out["eject"] = float(vac.data.joint_pos.torch[0, _joint_ids(vac, JOINTS)[0]])
    out["flap"] = float(bin_.data.joint_pos.torch[0, _joint_ids(bin_, BIN_JOINTS)[0]])
    bp, bq = _pose(bin_)
    out["bin_pos"] = bp
    out["bin_quat"] = bq
    vp, vq = _pose(vac)
    out["vac_pos"] = vp
    out["vac_quat"] = vq
    out["knob"] = _knob_world(vac)
    out["vac0"] = _MECH.vac0
    out["opened_once"] = _MECH.opened_once
    out["removed"] = _MECH.removed
    out["emptied"] = _MECH.emptied
    out["clicked"] = _MECH.clicked
    out["forced"] = _MECH.forced
    out["dropped"] = _MECH.dropped
    out["tipped"] = _MECH.tipped
    out["debris_left"] = _MECH.debris_left


# -- scoring ---------------------------------------------------------------------------------------------------------
STEPS = ("button pressed, bin unlatched", "bin pulled out", "dust emptied into the waste bin",
         "bin refitted and latched, vac stable, hands clear")
POINTS = 0.25


def _bin_dist(objects) -> float:
    return float(np.linalg.norm(np.asarray(objects["bin"].pos, float)[:2] - np.asarray(BIN_XY, float)))


def _bin_tilt(objects) -> float:
    return _tilt(objects["bin"].quat_xyzw)


def steps_done(objects) -> tuple[bool, ...]:
    p = _MECH
    s0 = bool(p.opened_once)
    s1 = s0 and bool(p.removed)
    s2 = s1 and bool(p.emptied)
    s3 = bool(s2 and p.clicked and _bin_dist(objects) <= SEAT_OFF
              and abs(np.asarray(objects["bin"].pos, float)[2] - BIN_BOTTOM) <= SEAT_OFF
              and _bin_tilt(objects) <= SEAT_TILT and _stable(objects) and _clear(objects)
              and not p.dropped and not p.forced and not p.tipped and not p.debris_left)
    return (s0, s1, s2, s3)


def _tilt_deg(objects) -> float:
    return math.degrees(_tilt(objects["vac"].quat_xyzw))


def _moved(objects) -> float:
    b0 = objects.get("vac0")
    if b0 is None:
        return 0.0
    return float(np.linalg.norm(np.asarray(objects["vac"].pos, float) - np.asarray(b0[0], float)))


def _stable(objects) -> bool:
    return bool(_tilt_deg(objects) <= math.degrees(STABLE_TILT) and _moved(objects) <= STABLE_MOVE)


def _clear(objects) -> bool:
    p = np.asarray(objects["vac"].pos, float)
    R = _rot(objects["vac"].quat_xyzw)
    bp = np.asarray(objects["bin"].pos, float)
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        if g.contact >= 0.5:
            return False
        for pt in _pad_points(g):
            q = R.T @ (pt - p)
            if (-0.090 - CLEAR <= q[0] <= 0.100 + CLEAR and -0.100 - CLEAR <= q[1] <= 0.100 + CLEAR
                    and -0.02 - CLEAR <= q[2] <= 0.22 + CLEAR):
                return False
            if float(np.linalg.norm(pt - bp)) <= CLEAR + BIN_L / 2:
                return False
    return True


_HOLD = {"t": None}


def _reset_hold():
    _HOLD["t"] = None


def _full(objects) -> bool:
    return bool(steps_done(objects)[3])


def success(objects) -> bool:
    if not _full(objects):
        _reset_hold()
        return False
    if _HOLD["t"] is None:
        _HOLD["t"] = 0.0
        return False
    _HOLD["t"] += 0.05
    return _HOLD["t"] >= STILL_T - 1e-9


def _caps(objects) -> float:
    p = _MECH
    cap = 1.0
    if p.dropped:
        cap = min(cap, 0.25)
    if p.debris_left or not p.emptied and p.clicked:
        cap = min(cap, 0.75)
    if p.forced:
        cap = min(cap, 0.50)
    return cap


def score(objects) -> float:
    if objects.get("tipped") or _MECH.tipped:
        _reset_hold()
        return 0.0
    done = steps_done(objects)
    pts = POINTS * sum(done[:3])
    pts = min(pts, _caps(objects))
    if _full(objects) and _HOLD["t"] is not None and _HOLD["t"] >= STILL_T - 1e-9:
        return 1.0
    return round(pts, 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
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


def _tool_frame() -> np.ndarray:
    """Fingers close along world -y (the gripper's local x); the approach axis (local z) points forward (+x) and
    17 degrees down. This wrist orientation reaches every button/bin/carry pose with margin on the right arm."""
    z = np.array([1.0, 0.0, -0.30])
    z = z / np.linalg.norm(z)
    x = np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


TOOL = _tool_frame()
BIN_GRIP_C = np.array([BIN_XY[0], BIN_XY[1], BIN_BOTTOM + BIN_H / 2])  # the bin centre (its COM)
EJECT_PRE = EJECT_KNOB + np.array([-0.045, 0.0, 0.010])
EJECT_IN = EJECT_KNOB + np.array([EJECT_PRESS + 0.010, 0.0, 0.0])
BIN_PRE = BIN_GRIP_C + np.array([-0.050, 0.0, 0.010])
BIN_OUT_P = BIN_GRIP_C + np.array([-0.075, 0.0, 0.0])
BIN_LIFT = BIN_GRIP_C + np.array([-0.075, 0.0, 0.100])
OVER_WASTE = np.array([WASTE_XY[0], WASTE_XY[1], TOP + 0.210])
BIN_BACK = BIN_GRIP_C + np.array([-0.075, 0.0, 0.100])


def right_plan():
    return [
        Waypoint(1.6, tuple(EJECT_PRE), OPEN_FINGER, "over the red button", rot=TOOL),
        Waypoint(0.9, tuple(EJECT_KNOB), OPEN_FINGER, "fingers on the button", rot=TOOL),
        Waypoint(0.7, tuple(EJECT_KNOB), EJECT_FINGER, "grip the button", rot=TOOL),
        Waypoint(1.2, tuple(EJECT_IN), EJECT_FINGER, "press the button", rot=TOOL),
        Waypoint(0.5, tuple(EJECT_IN), EJECT_FINGER, "hold the button", rot=TOOL),
        Waypoint(0.5, tuple(EJECT_KNOB), OPEN_FINGER, "let the button go", rot=TOOL),
        Waypoint(0.8, tuple(EJECT_PRE), OPEN_FINGER, "lift off button", rot=TOOL),
        Waypoint(1.5, tuple(BIN_PRE), OPEN_FINGER, "over the bin grip ribs", rot=TOOL),
        Waypoint(0.9, tuple(BIN_GRIP_C), OPEN_FINGER, "around the bin", rot=TOOL),
        Waypoint(0.7, tuple(BIN_GRIP_C), BIN_FINGER, "grip the bin", rot=TOOL),
        Waypoint(1.4, tuple(BIN_OUT_P), BIN_FINGER, "pull the bin out", rot=TOOL),
        Waypoint(1.2, tuple(BIN_LIFT), BIN_FINGER, "lift the bin clear", rot=TOOL),
        Waypoint(2.0, tuple(OVER_WASTE), BIN_FINGER, "carry over the waste bin", rot=TOOL),
        Waypoint(1.2, tuple(OVER_WASTE), BIN_FINGER, "open the flap, empty dust", rot=TOOL),
        Waypoint(1.0, tuple(OVER_WASTE), BIN_FINGER, "hold over waste", rot=TOOL),
        Waypoint(3.0, tuple(BIN_LIFT), BIN_FINGER, "carry the empty bin back", rot=TOOL),
        Waypoint(0.8, tuple(BIN_LIFT), BIN_FINGER, "steady over the cradle", rot=TOOL),
        Waypoint(1.6, tuple(BIN_GRIP_C), BIN_FINGER, "lower bin to the cradle", rot=TOOL),
        Waypoint(0.8, tuple(BIN_GRIP_C), OPEN_FINGER, "let go of the bin", rot=TOOL),
        Waypoint(0.8, tuple(BIN_GRIP_C + np.array([0.0, 0.0, 0.080])), OPEN_FINGER,
                 "lift straight clear of the bin", rot=TOOL),
        Waypoint(0.8, tuple(BIN_PRE), OPEN_FINGER, "lift off the bin", rot=TOOL),
        Waypoint(2.0, tuple(_START_TCP["right"]), OPEN_FINGER, "home", rot=_START_ROT["right"]),
    ]


def left_plan():
    return [Waypoint(1.0, tuple(_START_TCP["left"]), OPEN_FINGER, "stay home", rot=_START_ROT["left"])]


def demo():
    return {"left": left_plan(), "right": right_plan()}


def trace(objects) -> str:
    return (f"eject {objects['eject']*1000:.1f} mm, bin d {_bin_dist(objects)*1000:.0f} mm, "
            f"dust {'left' if _debris_still_in_bin(objects) else 'out'}, "
            f"tilt {_tilt_deg(objects):.2f} deg")
