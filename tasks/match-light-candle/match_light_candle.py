"""On the station table: a Swan Vestas class safety-match box (52 x 36 x 16 mm) lying flat on the left, a single safety match (47 mm, 2.1 mm splint) lying on
the right, a low brass chamberstick with a 250 mm dinner candle standing in it (the holder is free on the table), and a round ceramic ashtray.

The task is to light the candle with the match: strike the match on the box's striker strip, bring its flame to the wick (the wick lights after 2 s in the flame), and put the
used match in the ashtray before the flame reaches the fingers, leaving the box down and the hands clear. Ignition, the match flame and the wick flame are declared effects gated on
the robot's strike: the match head must be pressed on the striker strip (any real contact) and slide over at least 15 mm of it at 0.3 m/s or more; no force reading is scored. A
real trial scores the visible flame instead, and fits the gripper pads with clip-on stainless/ceramic heat shields (see the README). The match burns 2 mm/s down the stick from the
head: a flame front that reaches a held pinch, or a pad within 4 mm of it, caps the score at 0.5. Nothing else is scripted.

Run it:  inspect-robots run --embodiment yam_arena -E task=match_light_candle.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "match"
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
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: a free rigid body, an articulation
    (``articulation``: its root link is free) or a fixture (``kinematic``). ``Asset``'s fields only satisfy the
    task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False  # an articulation whose root link is fixed to the world
    no_gravity: bool = False  # a free part with gravity off (parked out of the way)

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=True if spec.fixed else None)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): the span of its finger pads along its pointing axis (``back``
    to the pad ``tips``), how far apart the pads are, and the smaller of the two fingers' grip forces (N): each
    finger's drive stiffness times how far something holds it back from its commanded position."""

    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float


PAD_BACK = 0.025  # m: the pads reach this far behind the TCP


def _gripper_states(env) -> dict:
    """Each gripper's ``GripperState`` and the episode's time (s), keyed ``left_gripper``, ``right_gripper`` and
    ``episode_time``."""
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        axis = runtime.link_poses(robot)["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)  # fingers open to -q each; a closing finger is held back
        out[f"{side}_gripper"] = GripperState(
            back=tcp - PAD_BACK * axis,
            tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
        )
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
        """Each part's root pose and velocity, the grippers and the episode time, plus what ``extra_states`` adds."""
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
        """yam_arena's environment settings, plus the task's reset events (``reset_events``)."""
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            env_cfg = configure(env_cfg)
            for name, func, params in reset_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="reset", params=params))
            for name, func, params in step_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="interval", interval_range_s=(0.0, 0.0),
                                                           is_global_time=False, params=params))
            return env_cfg

        return with_events

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


class _AfterSceneImport(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` right after it is first imported (it needs Isaac Sim, which starts later)."""

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
# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
BX, BY, BZ = 0.052, 0.036, 0.016  # the match box
PANEL_T, PANEL_X, PANEL_Z, PANEL_K = 0.0005, 0.048, 0.012, 4000.0  # the striker panel (0.8 mm), its strip, and its spring (N/m): a contact sensor
M_L, M_S, M_HR = 0.047, 0.0021, 0.0016  # the match: length, square splint, head radius
HZ = M_L - M_HR  # the head's centre along the stick from its base end
C_H, WICK_UP = 0.250, 0.008  # the candle and its wick above the top
GRIP_FB = 0.012  # the pinch is this far from the match's base end
SOCKET_DEPTH, RIM = 0.025, 0.030  # the chamberstick: socket depth and its rim above the table
FLOOR_Z = RIM - SOCKET_DEPTH
A_R, A_H, A_IN, A_FLOOR = 0.052, 0.024, 0.042, 0.006  # the ashtray: outer radius, height, bowl radius

BOX_START = (0.27, 0.17, TOP + BZ / 2 + 0.0003)
MATCH_C = (0.20, -0.20)  # the match's middle, lying along y with its head toward -y
MATCH_START = (MATCH_C[0], MATCH_C[1] + M_L / 2, TOP + 0.0017)  # the base end (the head rests on the table, the stick a little up)
HOLDER_XY = (0.44, -0.075)
ASH_XY = (0.33, -0.11)
CANDLE_START = (HOLDER_XY[0], HOLDER_XY[1], TOP + FLOOR_Z + 0.0003)
PARK = (0.0, -0.10, TOP - 0.15)  # where a flame waits, out of sight under the table
_QX90N = (0.70710678, 0.0, 0.0, 0.70710678)  # z along -y

BOX = UsdPart("box", asset="box", xy=BOX_START[:2], extent=0.03, mass=0.012, usd_path=str(ASSETS / "matchbox.usda"), pos=BOX_START, articulation=True)
MATCH = UsdPart("match", asset="match", xy=MATCH_START[:2], extent=0.03, mass=0.002, usd_path=str(ASSETS / "match.usda"), pos=MATCH_START, rot_xyzw=_QX90N)
HOLDER = UsdPart("holder", asset="holder", xy=HOLDER_XY, extent=0.05, mass=0.35, usd_path=str(ASSETS / "holder.usda"), pos=(HOLDER_XY[0], HOLDER_XY[1], TOP))
CANDLE = UsdPart("candle", asset="candle", xy=HOLDER_XY, extent=0.03, mass=0.08, usd_path=str(ASSETS / "candle.usda"), pos=CANDLE_START)
ASHTRAY = UsdPart("ashtray", asset="ashtray", xy=ASH_XY, extent=0.06, mass=0.20, usd_path=str(ASSETS / "ashtray.usda"), pos=(ASH_XY[0], ASH_XY[1], TOP))
FLAME_M = UsdPart("flame_m", asset="flame_m", xy=PARK[:2], extent=0.01, mass=1e-4, usd_path=str(ASSETS / "flame_match.usda"), pos=PARK, kinematic=True)
FLAME_C = UsdPart("flame_c", asset="flame_c", xy=PARK[:2], extent=0.01, mass=1e-4, usd_path=str(ASSETS / "flame_candle.usda"), pos=PARK, kinematic=True)
OBJECTS = [BOX, MATCH, HOLDER, CANDLE, ASHTRAY, FLAME_M, FLAME_C]
MOVERS = ("box", "match", "candle", "holder", "ashtray")
PANEL_JOINT = "PanelJoint"

# -- the declared effects (strike, flames, burn-down), run every control step ----------------------------------------------
STRIKE_PUSH, STRIKE_V, STRIKE_LEN = 0.0001, 0.30, 0.015  # m, m/s, m: the head must press the strip (0.1 mm contact) and slide this fast over this much of it
V_BURN = 0.0020  # m/s: the flame front's speed down the stick from the head (estimate; the whole stick burns in about 23 s and the front reaches the pad edge of a grip 8 mm from the base end in 12 s)
PAD_HALF = 0.0137  # m: the flame body's upward reach plus a margin: the flame is treated as reaching the fingers when the front is this far from their near edge
WICK_NEAR, WICK_S = 0.005, 2.0  # the flame within 5 mm of the wick tip for 2 s lights it
FLAME_LO, FLAME_HI = -0.003, 0.012  # m: the flame's body runs from 3 mm below the burning point to 12 mm above it (it rises)
CANDLE_R = 0.0106  # m: the candle's collider radius near its foot (0.0103 above 25 mm); the visible taper is 11 to 9.9 mm
TOUCH_CLEAR = 0.0015  # m: the match or a pad this close to the candle body counts as touching the hot wax
PAD_FLAME_CLEAR = 0.004  # m: a pad this close to the match flame would touch the real-trial heat shield
FX: dict = {}


def _fx_init() -> None:
    FX.clear()
    FX.update(t=-1.0, lit=False, lit_ever=False, spent=False, t_ign=0.0, front=0.0, stroke=0.0, prev=None, prev_push=0.0, near_s=0.0, near_ever=False, wick=False,
              best_v=0.0, q=(0.0, 0.0, 0.0, 0.0), events=[], hidden=(True, True), push=0.0, speed=0.0, dist=9.9)


_fx_init()


def _root(env, name):
    d = env.scene[name].data
    origin = env.scene.env_origins[0].detach().cpu().numpy()
    pose = _arr(d.root_link_pose_w)
    return pose[:3] - origin, pose[3:]


def _write_pose(env, env_ids, name, pos, quat=(0.0, 0.0, 0.0, 1.0)) -> None:
    import torch

    origin = env.scene.env_origins[0].detach().cpu().numpy()
    part = env.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *quat]], device=part.device, dtype=torch.float32)
    part.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)


def _state_of(pos, quat):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _panel_push(env) -> float:
    art = env.scene["box"]
    idx = art.find_joints([PANEL_JOINT], preserve_order=True)[0]
    return float(art.data.joint_pos.torch[0, idx].detach().cpu().numpy()[0])


def match_geo(pos, quat):
    """The match's head centre and its axis (base end to head)."""
    R = quat_xyzw_to_matrix(np.asarray(quat, float))
    a = R[:, 2]
    return np.asarray(pos, float) + a * HZ, a, R


def burn_point(pos, quat, front):
    head, a, _ = match_geo(pos, quat)
    return head - a * front


def candle_wick(pos, quat):
    R = quat_xyzw_to_matrix(np.asarray(quat, float))
    return np.asarray(pos, float) + R[:, 2] * (C_H + WICK_UP)


def _seg_dist(p, a, b) -> float:
    """Distance from point ``p`` to the segment ``a``-``b``."""
    ab = b - a
    s = max(0.0, min(1.0, float(np.dot(p - a, ab) / np.dot(ab, ab))))
    return float(np.linalg.norm(p - (a + s * ab)))


def _touches_candle(pm, qm, pc, qc, g) -> bool:
    """True if the match's stick or either gripper's pads touch the candle's wax body below its top. The candle body
    is the hot part a real trial must keep the match, the pads and their heat shields clear of; only the match flame
    may come near the wick."""
    base = np.asarray(pm, float)
    head, a, _ = match_geo(pm, qm)
    tip = head + a * M_HR
    Rc = quat_xyzw_to_matrix(np.asarray(qc, float))
    pc = np.asarray(pc, float)
    for s in np.linspace(0.0, 1.0, 25):
        q = Rc.T @ (base + s * (tip - base) - pc)
        if 0.002 <= q[2] <= C_H - 0.002 and math.hypot(q[0], q[1]) <= CANDLE_R + M_S / 2 + TOUCH_CLEAR:
            return True
    for side in ("left", "right"):
        for p in _pads(g[f"{side}_gripper"]):
            q = Rc.T @ (p - pc)
            if 0.0 <= q[2] <= C_H - 0.002 and math.hypot(q[0], q[1]) <= CANDLE_R + TOUCH_CLEAR:
                return True
    return False


def _fx_step(env, env_ids, asset_cfg=None) -> None:
    """Every control step: the strike (contact, speed, length), the match flame and its burn-down, the wick, the
    flames' poses and the rule events (a burn, a scorch, a touch of the candle body, a fire)."""
    t = float(env.episode_length_buf[0]) * float(env.step_dt)
    if t < FX["t"]:
        _fx_init()
    if t == FX["t"]:
        return
    dt = t - FX["t"] if FX["t"] >= 0 else float(env.step_dt)
    FX["t"] = t
    pb, qb = _root(env, "box")
    pm, qm = _root(env, "match")
    pc, qc = _root(env, "candle")
    ph, qh = _root(env, "holder")
    pa, qa = _root(env, "ashtray")
    head, a, Rm = match_geo(pm, qm)
    g = _gripper_states(env)
    # ---- the strike ----
    if not FX["lit"] and not FX["spent"]:
        d = _panel_push(env)
        Rb = quat_xyzw_to_matrix(qb)
        q = Rb.T @ (head - pb)
        face = -(BY / 2 + PANEL_T) + d
        touching = bool(-0.001 <= (q[1] + M_HR) - face <= 0.003 and abs(q[0]) <= PANEL_X / 2 + 0.001 and abs(q[2]) <= PANEL_Z / 2 + 0.0015 and q[1] < -BY / 2)
        contact = touching and d >= STRIKE_PUSH
        prev = FX["prev"]
        FX["prev"] = (q[0], q[2]) if touching else None
        prev_push = FX["prev_push"]
        FX["prev_push"] = d if touching else 0.0
        speed = 0.0
        if contact and prev is not None:
            step_len = math.hypot(q[0] - prev[0], q[2] - prev[1])
            speed = step_len / max(dt, 1e-6)
            if prev_push >= STRIKE_PUSH and speed >= STRIKE_V:
                FX["stroke"] += step_len
            else:
                FX["stroke"] = 0.0
        elif not contact:
            FX["stroke"] = 0.0
        FX["push"], FX["speed"] = d, speed
        FX["q"] = (float(q[0]), float(q[1]), float(q[2]), float(d))
        FX["best_v"] = max(FX["best_v"], speed)
        if FX["stroke"] >= STRIKE_LEN:
            FX.update(lit=True, lit_ever=True, t_ign=t, front=0.0)
            print(f"[{TAG}] t={t:.2f}s  STRUCK: strip push {d * 1000:.2f} mm, speed {speed:.2f} m/s, {FX['stroke'] * 1000:.0f} mm of the strip: the match is lit", flush=True)
    # ---- the burning match ----
    wick_tip = candle_wick(pc, qc)
    if FX["lit"]:
        FX["front"] = V_BURN * (t - FX["t_ign"])
        if FX["front"] >= M_L - 0.002:
            FX.update(lit=False, spent=True)
            print(f"[{TAG}] t={t:.2f}s  the match has burnt out", flush=True)
    if FX["lit"]:
        bp = burn_point(pm, qm, FX["front"])
        dist = _seg_dist(wick_tip, bp + np.array([0.0, 0.0, FLAME_LO]), bp + np.array([0.0, 0.0, FLAME_HI]))
        FX["dist"] = dist
        if dist <= WICK_NEAR:
            FX["near_s"] += dt
            if FX["near_s"] >= 0.5:
                FX["near_ever"] = True
            if FX["near_s"] >= WICK_S and not FX["wick"]:
                FX["wick"] = True
                print(f"[{TAG}] t={t:.2f}s  the wick has caught after {FX['near_s']:.1f} s in the flame", flush=True)
        else:
            FX["near_s"] = 0.0
        # events: the flame reaching a held pinch, a pad close to the flame, a lit match on the table outside the ashtray
        objs = {"left_gripper": g["left_gripper"], "right_gripper": g["right_gripper"], "match": _state_of(pm, qm)}
        lo, hi = np.array([-M_S / 2, -M_S / 2, 0.0]), np.array([M_S / 2, M_S / 2, M_L])
        for side in ("left", "right"):
            gs = g[f"{side}_gripper"]
            P = (gs.back + np.linspace(0.0, 1.0, 11)[:, None] * (gs.tips - gs.back) - pm) @ Rm
            near = np.all(np.abs(P[:, :2]) <= M_S / 2 + 0.004, axis=1) & (P[:, 2] >= -0.004) & (P[:, 2] <= M_L + 0.004)
            if np.any(near) and gs.opening <= M_S + 0.012:
                dgrip = HZ - (float(P[near, 2].max()) + PAD_HALF)
                if FX["front"] >= max(dgrip, 0.0) - 0.001:
                    _event(t, "burn", "the match's flame reached the fingers holding it")
            flame_lo, flame_hi = bp + np.array([0.0, 0.0, FLAME_LO]), bp + np.array([0.0, 0.0, FLAME_HI])
            if any(_seg_dist(p, flame_lo, flame_hi) <= PAD_FLAME_CLEAR for p in _pads(gs)):
                _event(t, "scorch", "the match flame came within 4 mm of a gripper pad")
        far_ash = float(np.hypot(pm[0] - pa[0], pm[1] - pa[1])) > A_R + 0.002 or float(np.hypot(head[0] - pa[0], head[1] - pa[1])) > A_R + 0.002
        if far_ash and min(pm[2], head[2]) <= TOP + 0.004:
            _event(t, "fire", "a lit match fell on the table outside the ashtray")
    # ---- the candle ----
    Rc = quat_xyzw_to_matrix(qc)
    ctilt = math.acos(max(-1.0, min(1.0, float(Rc[2, 2]))))
    htilt = math.acos(max(-1.0, min(1.0, float(quat_xyzw_to_matrix(qh)[2, 2]))))
    if ctilt > math.radians(15.0) or htilt > math.radians(15.0) or pc[2] < TOP - 0.03:
        _event(t, "fire", "the candle was knocked over")
    if _touches_candle(pm, qm, pc, qc, g):
        _event(t, "touch", "the match, a gripper pad or a heat shield touched the candle body")
    # ---- the flames ----
    if FX["lit"]:
        bp = burn_point(pm, qm, FX["front"])
        _write_pose(env, env_ids, "flame_m", bp + np.array([0.0, 0.0, -0.003]))
        FX["hidden"] = (False, FX["hidden"][1])
    elif not FX["hidden"][0]:
        _write_pose(env, env_ids, "flame_m", PARK)
        FX["hidden"] = (True, FX["hidden"][1])
    if FX["wick"]:
        _write_pose(env, env_ids, "flame_c", wick_tip + np.array([0.0, 0.0, -0.0045]))
        FX["hidden"] = (FX["hidden"][0], False)


def _event(t, kind, what) -> None:
    if (kind, what) not in [tuple(e) for e in FX["events"]]:
        FX["events"].append((kind, what))
        print(f"[{TAG}] t={t:.2f}s  RULE BROKEN: {what} (score {'is 0' if kind == 'fire' else 'capped at 0.5'} from here on)", flush=True)


def _fx_reset(env, env_ids, asset_cfg=None) -> None:
    _fx_init()
    for name in ("flame_m", "flame_c"):
        _write_pose(env, env_ids, name, PARK)


def extra_states(base, out, origin) -> None:
    """The declared effects' state (a copy), and the panel's push (for the trace)."""
    out["fx"] = {k: (list(v) if isinstance(v, (list, tuple)) else v) for k, v in FX.items()}


def reset_events() -> list:
    return [("fx_reset", _fx_reset, {})]


def step_events() -> list:
    return [("fx_step", _fx_step, {})]


@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None  # a full gripper (TCP) rotation; replaces ``yaw`` when set


def _target(self, t: float):
    """``ArmPlan.target`` that also follows a waypoint's ``rot``."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (
                prev_pos + s * (goal - prev_pos),
                ik.slerp_matrix(prev_rot, rot, s),
                prev_finger + s * (w.finger - prev_finger),
                w.label,
            )
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target

# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
# -- the rule and what is scored ----------------------------------------------------------------------------------------
CLEAR = 0.03
ASH_MATCH_R = A_IN + 0.001  # both ends of the match inside the bowl (its wall is at 42 mm)
STEPS = ("the match struck alight on the box's striker", "the flame held at the wick", "the candle flame established",
         "the used match in the ashtray, the candle upright, the box down, hands clear, no contact")
MATCH_LO = np.array([-M_S / 2, -M_S / 2, 0.0])
MATCH_HI = np.array([M_S / 2, M_S / 2, M_L])
BOX_LO = np.array([-BX / 2, -BY / 2, -BZ / 2])
BOX_HI = np.array([BX / 2, BY / 2, BZ / 2])


def _tilt_of(s):
    return math.acos(max(-1.0, min(1.0, float(_rot(s)[2, 2]))))


def match_in_ashtray(objects) -> bool:
    s = objects["match"]
    ash = np.asarray(objects["ashtray"].pos, float)  # the ashtray is free: its live pose
    head, a, _ = match_geo(s.pos, s.quat_xyzw)
    base = np.asarray(s.pos, float)
    ok = True
    for p in (base, head):
        ok = ok and float(np.hypot(p[0] - ash[0], p[1] - ash[1])) <= ASH_MATCH_R and p[2] <= ash[2] + A_H - 0.004 and p[2] >= ash[2] + A_FLOOR - 0.002
    return bool(ok)


def candle_lit_upright(objects) -> bool:
    """The wick is lit and the candle still stands seated in its chamberstick (within 4 mm of the socket axis, on the socket floor, 6 degrees of upright)."""
    c, h = objects["candle"], objects["holder"]
    p, q = np.asarray(c.pos, float), np.asarray(h.pos, float)
    off = float(np.hypot(p[0] - q[0], p[1] - q[1]))
    return bool(objects["fx"]["wick"] and off <= 0.004 and p[2] <= q[2] + FLOOR_Z + 0.003 and _tilt_of(c) <= math.radians(6.0) and _tilt_of(h) <= math.radians(6.0))


def box_down(objects) -> bool:
    b = objects["box"]
    return bool(b.pos[2] <= TOP + 0.012 and _tilt_of(b) <= math.radians(15.0))


def held_any(objects) -> bool:
    return any(holds(objects, s, n, lo, hi) for s in ("left", "right") for n, lo, hi in (("match", MATCH_LO, MATCH_HI), ("box", BOX_LO, BOX_HI)))


def clear_of_things(objects) -> bool:
    return not (pads_near(objects, "match", MATCH_LO, MATCH_HI, CLEAR) or pads_near(objects, "box", BOX_LO, BOX_HI, CLEAR))


def _steps(objects) -> list[bool]:
    fx = objects["fx"]
    last = bool(match_in_ashtray(objects) and candle_lit_upright(objects) and box_down(objects) and not held_any(objects) and clear_of_things(objects)
                and not fx["events"])
    return [bool(fx["lit_ever"]), bool(fx["near_ever"]), bool(fx["wick"]), last]


def events_now(objects, p) -> list:
    return list(objects["fx"]["events"])


def trace_line(objects) -> str:
    fx = objects["fx"]
    m = objects["match"]
    return (f"match {'LIT front %.0f mm' % (fx['front'] * 1000) if fx['lit'] else ('spent' if fx['spent'] else 'unlit')}, strip push {fx['push'] * 1000:.2f} mm, slide {fx['speed']:.2f} m/s, "
            f"stroke {fx['stroke'] * 1000:.0f} mm, flame to wick {fx['dist'] * 1000:.1f} mm for {fx['near_s']:.1f} s, wick {'LIT' if fx['wick'] else 'out'}, "
            f"match at {(m.pos[0]) * 1000:.0f},{m.pos[1] * 1000:.0f},{(m.pos[2] - TOP) * 1000:.0f} mm")


# -- scoring (shared machinery) ---------------------------------------------------------------------------------------
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
POINTS = 0.25  # per step (3 of them); only success scores 1.0
GRIP_SLACK = 0.004  # held: a pad's point within this of the part's bounds ...
GRIP_FORCE = 0.5  # ... both fingers pressing with at least this (N) ...
GRIP_OPENING = 0.001  # ... and the pads apart by at least this (not closed on nothing)


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def holds(objects, side: str, name: str, lo, hi) -> bool:
    """The ``side`` gripper holds the part: a point along its pads is within GRIP_SLACK of the part's bounds (``lo``,
    ``hi``, in the part's frame), both fingers press with GRIP_FORCE and the pads are apart by GRIP_OPENING."""
    g = objects.get(f"{side}_gripper")
    if g is None or g.contact < GRIP_FORCE or g.opening < GRIP_OPENING:
        return False
    s = objects[name]
    q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
    return bool(np.any(np.all((q >= np.asarray(lo) - GRIP_SLACK) & (q <= np.asarray(hi) + GRIP_SLACK), axis=1)))


def pads_near(objects, name: str, lo, hi, margin: float) -> bool:
    """Either gripper's pads are within ``margin`` of the part's bounds."""
    s = objects[name]
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
        if np.any(np.all((q >= np.asarray(lo) - margin) & (q <= np.asarray(hi) + margin), axis=1)):
            return True
    return False


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in MOVERS)


def still(objects, anchor) -> bool:
    """Every part within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses, not from the
    simulator's instantaneous speeds."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: the rule events (the declared effects'
    list, which last for the whole episode) and since when every full-success condition has held without a break
    (``met_since``, ``None`` while one fails) with the poses when they began to hold (``anchor``). Starts over when the
    episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.mem: dict = {}
        self.events: list = []
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.events = [tuple(e) for e in events_now(objects, self)]
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not self.events)
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand (see ``_steps``)."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = f"({trace_line(objects)}; rule events {len(p.events)})"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _cap(events) -> float:
    """1.0 with no rule event; 0.5 after a burn, a scorch or a touch of the candle body; 0 for a fire (a lit match on
    the table outside the ashtray or a knocked-over candle)."""
    cap = 1.0
    for kind, _ in events:
        cap = min(cap, 0.0 if kind == "fire" else 0.5)
    return cap


def _violation(objects) -> bool:
    """Scores 0: a state that is not finite, or a fire (a lit match on the table, a knocked-over candle). Reads the
    episode's progress without updating it."""
    values = [v for n in MOVERS for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return _cap(_PROGRESS.events) <= 0.0


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all steps done, no rule event so far (a burn, scorch or touch is one), and the parts within STILL_MOVE and STILL_TURN of where they were when
    that second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise POINTS per step done (the first three,
    at most 0.75, also while the second is being timed), capped at 0.5 after a burn, scorch or touch; 0 after a fire."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(min(_cap(_PROGRESS.events), POINTS * sum(done[:3])), 6)
# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left arm picks the box by its ends, lifts it and turns its wrist to hold it standing on its edge with the striker strip up. The right arm picks the match lying on
# the table by its base end, turns its wrist to hold it upright head down, and runs the head along the strip pressing it (a quick pressed stroke). It carries the lit match to
# the wick, tilts it 45 degrees so the burnt end hangs clear of the candle, holds the flame at the wick, moves over the ashtray and lets go. The left arm puts the box
# back on the table meanwhile. Both go home.
_grip = yam_demo.grip_rotation
ROTX = _grip(math.pi / 2)  # pads close along x (the right arm picks the match with it)
ROTL = _grip(-math.pi / 2)  # the same for the left wrist, turned the other way (joint 6's limit)
D_G = 0.012  # the stick's middle is this far from the TCP along the pointing axis (1 mm inside the pad tips)
D_B = 0.013  # a box held by its ends has its middle this far from the TCP along the pointing axis
OPEN_BOX, HOLD_BOX = -(BX + 0.016) / 2, -(BX - 0.008) / 2
OPEN_M, HOLD_M = -(M_S + 0.020) / 2, 0.0
A_LOCAL = ROTX.T @ np.array([0.0, -1.0, 0.0])  # the stick's direction (base to head) in the gripper frame: the match lies head toward -y
CB = (0.30, -0.01, TOP + 0.17)  # the held box's middle during the strike
STRIP_TOP = CB[2] + BY / 2 + PANEL_T  # the striker panel's outer face (up), world z
HC_Z = STRIP_TOP + M_HR  # the head's middle when it just touches it
CANDLE_TOP = TOP + FLOOR_Z + C_H
WICK = np.array([HOLDER_XY[0], HOLDER_XY[1], CANDLE_TOP + WICK_UP])
GRASP_G = np.array([MATCH_C[0], MATCH_C[1] + M_L / 2 - GRIP_FB, TOP + 0.0014])
ASH_TOP = TOP + A_H


def _Rx(a):
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), a)


def _Ry(a):
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), a)


R_BOX = _Rx(-math.pi / 2) @ ROTL  # the left wrist pointing -y, the box standing on its edge, striker up
R_HEAD_DOWN = _Rx(math.pi / 2) @ ROTX  # the right wrist pointing +y, the match upright, head down


def tcp_G(G, R):
    return np.asarray(G, float) - D_G * R[:, 2]


def a_of(R):
    return R @ A_LOCAL


def tcp_H(H, R):
    """The TCP when the head's middle is at ``H`` with the gripper at rotation ``R``."""
    G = np.asarray(H, float) - a_of(R) * (HZ - GRIP_FB)
    return tcp_G(G, R)


def left_plan() -> list[Waypoint]:
    bx, by, bz = BOX_START[0], BOX_START[1], TOP + BZ / 2
    tg = lambda x, y, zc: (x, y, zc + D_B)  # noqa: E731  (TCP for a held box lying flat, its middle at height zc)
    r = ROTL
    held_at = lambda x: np.array([x, CB[1], CB[2]]) - D_B * R_BOX[:, 2]  # the TCP that holds the box with its middle at x (and at CB's y and z)
    held, held_end = held_at(XH - SWEEP / 2), held_at(XH + SWEEP / 2)
    return [
        Waypoint(2.0, tg(bx, by, bz + 0.05), OPEN_BOX, "over the box", rot=r),
        Waypoint(1.2, tg(bx, by, bz), OPEN_BOX, "pads astride the box", rot=r),
        Waypoint(0.7, tg(bx, by, bz), HOLD_BOX, "grip the box", rot=r),
        Waypoint(0.4, tg(bx, by, bz), HOLD_BOX, "hold", rot=r),
        Waypoint(1.2, tg(bx, by, bz + 0.08), HOLD_BOX, "lift the box", rot=r),
        Waypoint(2.4, tuple(held), HOLD_BOX, "turn the box on its edge, striker up", rot=R_BOX),
        Waypoint(T_L_WAIT, tuple(held), HOLD_BOX, "hold the box while the match is pressed on the strip", rot=R_BOX),
        Waypoint(T_SWEEP, tuple(held_end), HOLD_BOX, "draw the strip under the match head", rot=R_BOX),
        Waypoint(T_L_AFTER, tuple(held_end), HOLD_BOX, "hold", rot=R_BOX),
        Waypoint(2.4, tg(bx, by, bz + 0.05), HOLD_BOX, "bring the box back over its place", rot=r),
        Waypoint(1.2, tg(bx, by, bz + 0.003), HOLD_BOX, "put the box down", rot=r),
        Waypoint(0.6, tg(bx, by, bz + 0.003), OPEN_BOX, "let go", rot=r),
        Waypoint(1.0, tg(bx, by, bz + 0.07), OPEN_BOX, "lift away", rot=r),
        Waypoint(2.0, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


XH = 0.30  # the match head's x (it stays put: the right arm presses it on the strip, the left arm draws the box under it)
SWEEP = 0.044  # m: the box's travel under the head (the head runs from 22 mm on one side of the box's middle to 22 mm on the other)
T_SWEEP = 0.075  # s: the draw (one control step: the arm trails its command and reaches about 0.4 m/s)
T_PRESS = 2.6  # s: the right arm's press builds as it lowers its command below the strip
T_L_AFTER = 3.2  # the left arm keeps the box where it is until the right arm has carried the match clear
T_L_WAIT = 4.6
PRESS_DEPTH = 0.027  # commanded depth beyond the touching position (the arm is soft in a vertical press; only the panel's contact is checked)
GAP = 0.002
LEAN = math.radians(45.0)  # at the wick the match is turned 45 degrees about the pointing axis: the burnt end hangs clear of the candle
BP_LATERAL = 0.002  # the burning point beside the wick (m)
STRIKE_LEAN = math.radians(0.0)  # the match leans 15 degrees, head toward -x, at the strike
DROP_LEAN = math.radians(45.0)
ASH_DX, ASH_REL = 0.022, 0.040  # over the ashtray the head is this far toward the robot of the bowl axis and this high above the table
BP_ABOVE = 0.010  # and this far above the candle top


def right_plan() -> list[Waypoint]:
    g = GRASP_G
    hv = lambda z: (g[0], g[1], z)  # noqa: E731
    r0 = ROTX
    out = [
        Waypoint(2.5, tcp_G(hv(TOP + 0.06), r0), OPEN_M, "over the match", rot=r0),
        Waypoint(1.4, tcp_G(hv(g[2]), r0), OPEN_M, "pads astride the match", rot=r0),
        Waypoint(0.7, tcp_G(hv(g[2]), r0), HOLD_M, "pinch the match", rot=r0),
        Waypoint(0.4, tcp_G(hv(g[2]), r0), HOLD_M, "hold", rot=r0),
        Waypoint(1.2, tcp_G(hv(TOP + 0.12), r0), HOLD_M, "lift the match", rot=r0),
    ]
    R = R_HEAD_DOWN
    RS = _Ry(STRIKE_LEAN) @ R  # at the strike the match leans with its head trailing (as a hand strikes): the drag on the head then runs along the stick
    H = lambda dx, dz, dy=-0.004: (XH + dx, CB[1] + dy, HC_Z + dz)  # noqa: E731  (head's middle relative to the strip's face and the strike point)
    out += [
        Waypoint(2.6, tcp_H(H(0.0, 0.030), RS), HOLD_M, "turn the match head down, over the box's striker", rot=RS),
        Waypoint(1.0, tcp_H(H(0.0, GAP), RS), HOLD_M, "head down to the strip", rot=RS),
        Waypoint(T_PRESS, tcp_H(H(0.0, -PRESS_DEPTH), RS), HOLD_M, "press the head on the strip", rot=RS),
        Waypoint(T_SWEEP + 0.2, tcp_H(H(0.0, -PRESS_DEPTH), RS), HOLD_M, "keep pressing while the strip is drawn under it", rot=RS),
        Waypoint(0.8, tcp_H(H(0.0, 0.04), RS), HOLD_M, "lift off the box", rot=RS),
    ]
    # to the wick: the head hangs down-and-away (toward -x) from the burning point, which sits beside the wick
    Rw = _Ry(LEAN) @ R
    bp = np.array([WICK[0] - BP_LATERAL, WICK[1], CANDLE_TOP + BP_ABOVE])
    aw = a_of(Rw)
    Rd = _Ry(DROP_LEAN) @ R  # over the ashtray the match leans like at the wick, so it falls flat along x in the bowl
    front = 0.008  # about the burn so far when the match reaches the wick (the burning point is this far up the stick from the head)
    hw = bp + aw * front
    out += [
        Waypoint(1.3, tcp_H(hw + np.array([0.0, -0.03, 0.03]), Rw), HOLD_M, "carry the flame to the candle", rot=Rw),
        Waypoint(0.7, tcp_H(hw, Rw), HOLD_M, "bring the flame to the wick", rot=Rw),
        Waypoint(T_WICK / 2, tcp_H(hw, Rw), HOLD_M, "hold the flame at the wick", rot=Rw),
        Waypoint(T_WICK / 2, tcp_H(hw + aw * V_BURN * T_WICK / 2, Rw), HOLD_M, "keep the flame at the wick as the match burns down", rot=Rw),
        Waypoint(0.7, tcp_H(hw + np.array([-0.13, -0.05, 0.03]), R), HOLD_M, "draw the match back from the candle and turn it upright", rot=R),
        Waypoint(1.1, tcp_H(np.array([ASH_XY[0] - ASH_DX, ASH_XY[1], ASH_TOP + 0.035]), Rd), HOLD_M, "over the ashtray, the match leaning head toward the robot", rot=Rd),
        Waypoint(0.5, tcp_H(np.array([ASH_XY[0] - ASH_DX, ASH_XY[1], TOP + ASH_REL]), Rd), HOLD_M, "lower it into the bowl", rot=Rd),
        Waypoint(0.3, tcp_H(np.array([ASH_XY[0] - ASH_DX, ASH_XY[1], TOP + ASH_REL]), Rd), OPEN_M, "let the match drop", rot=Rd),
        Waypoint(1.0, tcp_H(np.array([ASH_XY[0] - 0.05, ASH_XY[1] + 0.02, TOP + 0.12]), R), OPEN_M, "draw the hand away", rot=R),
        Waypoint(1.4, tcp_H(np.array([0.22, ASH_XY[1] + 0.05, TOP + 0.14]), R), OPEN_M, "draw back toward the robot, clear of the candle", rot=R),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return out


T_WICK = 2.35


def demo():
    """The left arm holds the box and puts it down; the right arm strikes the match, lights the wick and drops the match in the ashtray."""
    return {"left": left_plan(), "right": right_plan()}
