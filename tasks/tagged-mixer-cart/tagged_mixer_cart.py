"""On the station table, a small equipment bench: three mini vortex mixers stand in a row (11 x 8 x 6.5 cm body, 0.45 kg), each
with an equipment tag on its front face, and a low repair cart with a flat tray stands to the left. Two tags are green
(in service), one is red (out of service, awaiting repair). The robot takes the unit tagged out of service to the repair
cart; the working units stay on the bench.

Left arm: straight down on the red-tagged mixer's sides, lifts it clear of its neighbour, carries it over the cart, lowers it
onto the tray floor, lets go and lifts away.

Each mixer and the cart is one rigid body from ``assets/*.usda``.

Run it:  inspect-robots run --embodiment yam_arena -E task=tagged_mixer_cart.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
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

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "tagged_mixer_cart"

# -- the mixers (m): body 110 (x) x 80 (y) x 65 high, origin in the middle of the base; the rubber cup on top 45 mm across, 15 tall
MIX_L, MIX_W, MIX_H, CUP_H = 0.110, 0.080, 0.065, 0.015
MIX_MASS = 0.45
REST_GAP = 0.0003
# -- the cart: tray floor at DECK, 160 (x) x 200 (y) inside, 12 mm rim 12 mm high
DECK = 0.030
TRAY_X, TRAY_Y = 0.160, 0.200

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
CART_XY = (0.31, 0.27)
MIXER_XY = {"mixer_a": (0.32, -0.03), "mixer_b": (0.32, 0.09), "mixer_c": (0.32, -0.15)}   # mixer_b has the red tag, nearest the cart
RED = "mixer_b"
GREENS = ("mixer_a", "mixer_c")

_X, _Y, _Z = np.eye(3)


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self) -> bool:
        return not self.fixed


CART = UsdPart("cart", asset="cart", xy=CART_XY, extent=0.24, mass=8.0, usd_path=str(ASSETS / "repair_cart.usda"),
               pos=(*CART_XY, TOP), fixed=True)
MIXERS = {k: UsdPart(k, asset="mixer", xy=xy, extent=MIX_L, mass=MIX_MASS, usd_path=str(ASSETS / f"{k}.usda"),
                     pos=(*xy, TOP + REST_GAP)) for k, xy in MIXER_XY.items()}
OBJECTS = [CART, *MIXERS.values()]
NAMES = tuple(MIXERS)

SOLVER_ITERATIONS = 64
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=None)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                  rotation_xyzw=tuple(float(v) for v in spec.rot_xyzw)),
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


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """The panel's and the jack's roots, the grippers, the episode time and the jack's two tab angles."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}

        def arr(t):
            return t.torch[0].detach().cpu().numpy().astype(np.float64)

        for name in names:
            data = base.scene[name].data
            pose = arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=arr(data.root_lin_vel_w),
                                    ang_vel=arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        return out

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states


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


# -- scoring ----------------------------------------------------------------------------------------------------------
LIFTED = 0.02  # m: step 1, the red unit's base this far above where it stood, in a gripper
IN_TRAY_MARGIN = 0.010  # m: a unit is on the tray when its centre is within the tray floor less this
UPRIGHT = math.radians(10.0)  # a unit within this of upright
FALLEN = math.radians(45.0)  # tilted this far, a unit has fallen (scores 0)
STAY_BAND = 0.02  # m: a working unit within this of where it stood is at its place
CLEAR = 0.03  # m: both grippers at least this far from the units
GRIP_FORCE = 0.5  # N a finger
GRIP_OPENING = 0.02  # m: the pads are at least this far apart (something between them)
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
STEPS = ("red-tagged unit lifted in a gripper", "red-tagged unit standing on the repair cart's tray",
         "and both working units still at their places on the bench", "let go of everything")
POINTS = 0.2


def _rot(q) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(q, float))


def _pose(objects, name):
    o = objects[name]
    return np.asarray(o.pos, float), _rot(o.quat_xyzw)


def tilt(objects, name) -> float:
    _, r = _pose(objects, name)
    return math.acos(max(-1.0, min(1.0, float(r[2, 2]))))


def on_tray(objects, name) -> bool:
    """The unit's base centre is over the tray floor (less IN_TRAY_MARGIN), on it (within 3 mm of the floor), upright."""
    pos, _ = _pose(objects, name)
    dx, dy = pos[0] - CART_XY[0], pos[1] - CART_XY[1]
    return bool(abs(dx) <= TRAY_X / 2 - IN_TRAY_MARGIN and abs(dy) <= TRAY_Y / 2 - IN_TRAY_MARGIN
                and abs(pos[2] - (TOP + DECK)) <= 0.004 and tilt(objects, name) <= UPRIGHT)


def over_tray(objects, name) -> bool:
    """Any part of the unit is over the tray (for a working unit put there by mistake)."""
    pos, _ = _pose(objects, name)
    return bool(abs(pos[0] - CART_XY[0]) <= TRAY_X / 2 + MIX_L / 2 and abs(pos[1] - CART_XY[1]) <= TRAY_Y / 2 + MIX_W / 2
                and pos[2] > TOP + DECK - 0.01)


def at_place(objects, name) -> bool:
    pos, _ = _pose(objects, name)
    return bool(np.linalg.norm(pos[:2] - np.array(MIXER_XY[name])) <= STAY_BAND and tilt(objects, name) <= UPRIGHT
                and abs(pos[2] - (TOP + REST_GAP)) <= 0.004)


def _pads(g) -> list[np.ndarray]:
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def in_hand(objects, name) -> bool:
    """A gripper presses (GRIP_FORCE a finger, pads at least GRIP_OPENING apart) with a pad point beside the unit's body."""
    pos, r = _pose(objects, name)
    for g in ("left_gripper", "right_gripper"):
        gr = objects.get(g)
        if gr is None or gr.contact < GRIP_FORCE or gr.opening < GRIP_OPENING:
            continue
        for p in _pads(gr):
            k = r.T @ (p - pos)
            if abs(k[0]) <= MIX_L / 2 + 0.01 and abs(k[1]) <= MIX_W / 2 + 0.01 and -0.01 <= k[2] <= MIX_H + CUP_H + 0.01:
                return True
    return False


def let_go(objects) -> bool:
    for g in ("left_gripper", "right_gripper"):
        gr = objects.get(g)
        if gr is None:
            continue
        for p in _pads(gr):
            for n in NAMES:
                pos, r = _pose(objects, n)
                k = r.T @ (p - pos)
                if abs(k[0]) <= MIX_L / 2 + CLEAR and abs(k[1]) <= MIX_W / 2 + CLEAR and -CLEAR <= k[2] <= MIX_H + CUP_H + CLEAR:
                    return False
    return True


def _poses(objects) -> tuple:
    out = []
    for n in NAMES:
        o = objects[n]
        out += [np.array(o.pos, float), np.array(o.quat_xyzw, float)]
    return tuple(out)


def _turn(q0, q1) -> float:
    return 2 * math.acos(min(1.0, abs(float(np.dot(q0, q1)))))


def still(objects, anchor) -> bool:
    now = _poses(objects)
    return all(np.linalg.norm(now[i] - anchor[i]) <= STILL_MOVE and _turn(anchor[i + 1], now[i + 1]) <= STILL_TURN
               for i in range(0, len(now), 2))


class _Progress:
    """The episode so far, updated once per control step: whether the red unit has been lifted in a gripper, whether a unit
    fell, and since when every full-success condition has held (``met_since``, ``anchor``)."""

    def __init__(self):
        self.time = -1.0
        self.lifted = False
        self.fell = False
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        pos, _ = _pose(objects, RED)
        if pos[2] >= TOP + LIFTED and in_hand(objects, RED):
            self.lifted = True
        self.fell |= any(tilt(objects, n) > FALLEN for n in NAMES)
        if t is None:
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): timing restarts",
                  flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    lifted = bool(_PROGRESS.lifted)
    tray = bool(lifted and on_tray(objects, RED))
    stays = bool(tray and all(at_place(objects, g) for g in GREENS))
    return [lifted, tray, stays, bool(stays and let_go(objects))]


def steps_done(objects) -> list[bool]:
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        rp, _ = _pose(objects, RED)
        line = (f"(red unit at x {rp[0]:.3f} y {rp[1]:.3f} z {rp[2] - TOP:.3f}; greens at place: "
                f"{[at_place(objects, g) for g in GREENS]}; tilts {[round(math.degrees(tilt(objects, n)), 1) for n in NAMES]})")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: a unit fallen (now or earlier), a working unit on the repair cart, anything off the table, non-finite state."""
    values = [v for n in NAMES for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    low = any(objects[n].pos[2] < TOP - 0.03 for n in NAMES)
    wrong = any(over_tray(objects, g) for g in GREENS)
    return bool(_PROGRESS.fell or low or wrong)


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every condition held for STILL_S); otherwise 0.2 per step done (at most 0.8); 0 for a violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
SQUEEZE = 0.008  # the fingers close this far inside the body's width (about 20 N a finger)
HOVER = 0.06
GRIP_Z = 0.057  # the TCP this far above the base: the pads cover the body's upper 3 cm
CARRY = 0.075  # the base passes this far above the table
SET_GAP = 0.002  # lowered to this above the tray floor, then let go
ROT = yam_demo.grip_rotation(0.0)  # straight down, fingers closing along y

_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


def arm_plan(side="left"):
    open_, hold = -(MIX_W + 0.022) / 2, -(MIX_W - SQUEEZE) / 2
    sx, sy = MIXER_XY[RED]
    cx, cy = CART_XY
    w = [
        Waypoint(2.5, [sx, sy, TOP + GRIP_Z + HOVER], open_, "over the red-tagged unit", rot=ROT),
        Waypoint(1.5, [sx, sy, TOP + GRIP_Z], open_, "down on its sides", rot=ROT),
        Waypoint(0.8, [sx, sy, TOP + GRIP_Z], hold, "grip", rot=ROT),
        Waypoint(1.5, [sx, sy, TOP + CARRY + GRIP_Z], hold, "lift it clear", rot=ROT),
        Waypoint(3.0, [cx, cy, TOP + CARRY + GRIP_Z], hold, "carry it over the cart", rot=ROT),
        Waypoint(2.0, [cx, cy, TOP + DECK + SET_GAP + GRIP_Z], hold, "lower it onto the tray", rot=ROT),
        Waypoint(0.6, [cx, cy, TOP + DECK + SET_GAP + GRIP_Z], open_, "let go", rot=ROT),
        Waypoint(1.2, [cx, cy, TOP + CARRY + GRIP_Z], open_, "lift away", rot=ROT),
        Waypoint(2.5, _START_TCP[side], FINGER_OPEN, "home", rot=_START_ROT[side]),
    ]
    return w


def demo():
    """The left arm takes the red-tagged unit to the repair cart's tray; the right arm stays home."""
    return {"left": arm_plan("left")}
