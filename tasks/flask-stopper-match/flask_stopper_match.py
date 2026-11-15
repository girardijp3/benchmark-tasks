"""On the station table, a chemistry bench: two Erlenmeyer flasks stand open, a 125 mL flask with a 24/29 standard-taper
neck and a 50 mL flask with a 14/23 neck, and in front of them stand two solid ground-glass stoppers, a 24/29 and a 14/23
(each size printed on the flask and on its stopper's head). The robot closes each flask with the stopper that belongs
to it: the cone goes into the neck's tapered socket until the head rests on the rim. The wrong stopper does not fit: the
24/29 cone is wider than the 14/23 neck, and the 14/23 stopper slides straight down the 24/29 bore and is lost inside.

Left arm: takes the 24/29 stopper by its head and seats it in the 125 mL flask. Right arm: the same for the 14/23 stopper and
the 50 mL flask. Each lets go and lifts away.

Each flask is one rigid body (24 tapered wedge colliders form the socket, 0.1 mm radial play, 1.5 mm lead-in), each stopper a
rigid body (cone and head hulls); from ``assets/flask_*.usda`` and ``assets/stopper_*.usda``.

Run it:  inspect-robots run --embodiment yam_arena -E task=flask_stopper_match.py --scorer task_score ...

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
TAG = "flask_stopper_match"

# -- the glassware (mm; ISO 383 / DIN 12242 standard-taper joints, 1:10 on the diameter) -------------------------------
# A flask's frame: origin in the middle of its base on the table, z up; the neck's rim is at z = RIM. A stopper's frame:
# origin in the middle of the underside of its head, z up (the cone points down).
BIG = dict(name="big", label="24/29", rim=125.0 * MM, cone_l=29.0 * MM, head_d=40.0 * MM, head_t=8.0 * MM,
           neck_od=34.0 * MM, base_d=67.0 * MM, shoulder=85.0 * MM, big_end=24.0 * MM, mass=0.045)
SMALL = dict(name="small", label="14/23", rim=95.0 * MM, cone_l=23.0 * MM, head_d=24.0 * MM, head_t=6.0 * MM,
             neck_od=22.0 * MM, base_d=50.0 * MM, shoulder=62.0 * MM, big_end=14.5 * MM, mass=0.015)
SIZES = (BIG, SMALL)

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
FLASK_XY = {"big": (0.32, 0.12), "small": (0.38, -0.13)}
STOPPER_XY = {"big": (0.27, 0.22), "small": (0.27, -0.21)}
REST_GAP = 0.0003  # the stoppers' start: this far above the table

_X, _Y, _Z = np.eye(3)


# -- objects ----------------------------------------------------------------------------------------------------------


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


def _flask(size):
    return UsdPart(f"flask_{size['name']}", asset="flask", xy=FLASK_XY[size["name"]], extent=size["base_d"], mass=0.085 if size is BIG else 0.045,
                   usd_path=str(ASSETS / f"flask_{size['label'].replace('/', '_')}.usda"),
                   pos=(*FLASK_XY[size["name"]], TOP))


def _stopper(size):
    return UsdPart(f"stopper_{size['name']}", asset="stopper", xy=STOPPER_XY[size["name"]], extent=size["head_d"], mass=size["mass"],
                   usd_path=str(ASSETS / f"stopper_{size['label'].replace('/', '_')}.usda"),
                   pos=(*STOPPER_XY[size["name"]], TOP + size["cone_l"] + REST_GAP))


FLASKS = {s["name"]: _flask(s) for s in SIZES}
STOPPERS = {s["name"]: _stopper(s) for s in SIZES}
OBJECTS = [*FLASKS.values(), *STOPPERS.values()]

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
        """The four parts' roots, the grippers and the episode time."""
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
SEAT_LATERAL = 1.0 * MM  # the stopper's head centre within this of the neck's axis
SEAT_TILT = math.radians(2.0)  # and its axis within this of the neck's
SEAT_LOW, SEAT_HIGH = -0.5 * MM, 1.5 * MM  # the head's underside within this of the rim (negative: below it)
SUNK = 3.0 * MM  # a stopper whose head is this far below a rim and inside its bore has gone down the neck
BORE_R = 0.016  # m: inside a neck's bore, near enough (the widest bore, with its lead-in, is 14 mm in radius)
UPRIGHT = math.radians(10.0)  # a flask within this of upright
FALLEN = math.radians(45.0)  # tilted this far, a flask has fallen (scores 0)
PLACE_BAND = 0.02  # m: a flask within this of where it stood is at its place
CLEAR = 0.03  # m: both grippers at least this far from every flask and stopper
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
STEPS = ("24/29 stopper seated in the 125 mL flask", "14/23 stopper seated in the 50 mL flask",
         "both flasks closed with their own stoppers, upright", "let go of everything, both stoppers seated")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, float))


def _pose(objects, name):
    o = objects[name]
    return np.asarray(o.pos, float), _rot(o.quat_xyzw)


def flask_tilt(objects, key) -> float:
    """A flask's axis from vertical (rad)."""
    _, r = _pose(objects, f"flask_{key}")
    return math.acos(max(-1.0, min(1.0, float(r[2, 2]))))


def flask_at_place(objects, key) -> bool:
    pos, _ = _pose(objects, f"flask_{key}")
    return bool(flask_tilt(objects, key) <= UPRIGHT and np.linalg.norm(pos[:2] - np.array(FLASK_XY[key])) <= PLACE_BAND)


def stopper_in_flask(objects, skey, fkey):
    """The stopper's head-underside centre in the flask's frame (its height above the rim, lateral offset) and the angle
    between the two axes."""
    fp, fr = _pose(objects, f"flask_{fkey}")
    sp, sr = _pose(objects, f"stopper_{skey}")
    rim = (BIG if fkey == "big" else SMALL)["rim"]
    q = fr.T @ (sp - fp)
    axis = fr.T @ sr[:, 2]
    off = math.acos(max(-1.0, min(1.0, float(axis[2]))))
    return float(q[2] - rim), float(math.hypot(q[0], q[1])), off


def seated(objects, key) -> bool:
    """The stopper of size ``key`` sits in the flask of that size: its head on the rim (within SEAT_LOW..SEAT_HIGH of it),
    centred (SEAT_LATERAL) and square (SEAT_TILT), the flask upright at its place."""
    h, lat, off = stopper_in_flask(objects, key, key)
    return bool(SEAT_LOW <= h <= SEAT_HIGH and lat <= SEAT_LATERAL and off <= SEAT_TILT and flask_at_place(objects, key))


def sunk(objects) -> bool:
    """A stopper has gone down inside a flask's neck (the wrong, smaller one into the big flask)."""
    for s in ("big", "small"):
        for f in ("big", "small"):
            h, lat, _ = stopper_in_flask(objects, s, f)
            if h <= -SUNK and lat <= BORE_R and h >= -0.06:
                return True
    return False


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def let_go(objects) -> bool:
    """No pad point within CLEAR of any flask (a cylinder round its body and neck) or stopper (its cone and head)."""
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None:
            continue
        for p in _pads(gripper):
            for s in SIZES:
                fp, fr = _pose(objects, f"flask_{s['name']}")
                q = fr.T @ (p - fp)
                if math.hypot(q[0], q[1]) <= s["base_d"] / 2 + CLEAR and -CLEAR <= q[2] <= s["rim"] + CLEAR:
                    return False
                sp, sr = _pose(objects, f"stopper_{s['name']}")
                k = sr.T @ (p - sp)
                if math.hypot(k[0], k[1]) <= s["head_d"] / 2 + CLEAR and -s["cone_l"] - CLEAR <= k[2] <= s["head_t"] + CLEAR:
                    return False
    return True


def _poses(objects) -> tuple:
    out = []
    for n in ("flask_big", "flask_small", "stopper_big", "stopper_small"):
        o = objects[n]
        out += [np.array(o.pos, float), np.array(o.quat_xyzw, float)]
    return tuple(out)


def _turn(q0, q1) -> float:
    return 2 * math.acos(min(1.0, abs(float(np.dot(q0, q1)))))


def still(objects, anchor) -> bool:
    """Every part within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses."""
    now = _poses(objects)
    return all(np.linalg.norm(now[i] - anchor[i]) <= STILL_MOVE and _turn(anchor[i + 1], now[i + 1]) <= STILL_TURN
               for i in range(0, len(now), 2))


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a flask fell, and since when every
    full-success condition has held without a break (``met_since``) with the poses when they began to hold (``anchor``).
    Any lapse restarts the count. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.fell = False
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
        self.fell |= any(flask_tilt(objects, k) > FALLEN for k in ("big", "small"))
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    a, b = seated(objects, "big"), seated(objects, "small")
    both = bool(a and b)
    return [a, b, both, bool(both and let_go(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        hb, lb, ob = stopper_in_flask(objects, "big", "big")
        hs, ls, os_ = stopper_in_flask(objects, "small", "small")
        line = (f"(24/29 stopper {hb * 1000:+.1f} mm above its rim, off-axis {lb * 1000:.2f} mm, {math.degrees(ob):.1f} deg; "
                f"14/23 stopper {hs * 1000:+.1f} mm, {ls * 1000:.2f} mm, {math.degrees(os_):.1f} deg)")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: a flask fallen (now or earlier this episode), a stopper gone down a neck, anything off the table, or a
    state that is not finite. Reads the episode's progress without updating it."""
    values = [v for n in ("flask_big", "flask_small", "stopper_big", "stopper_small") for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    low = any(objects[n].pos[2] < TOP - 0.03 for n in ("flask_big", "flask_small", "stopper_big", "stopper_small"))
    return bool(_PROGRESS.fell or low or sunk(objects) or any(flask_tilt(objects, k) > FALLEN for k in ("big", "small")))


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all four steps done (the last one: both grippers clear), no violation, and every part within STILL_MOVE and
    STILL_TURN of where it was when that second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: straight down on the 24/29 stopper's head (fingers closing along y), lifts it clear of the big flask's rim, moves
# over the neck, lowers the cone into the socket until the head rests on the rim (commanded a little past, so the arm
# presses a few newtons), lets go and lifts away. Right arm: the same with the 14/23 stopper and the small flask.
SQUEEZE = 0.008  # the fingers close this far inside the head's width (about 20 N a finger)
HOVER = 0.07  # the TCPs hover this far above their grips
HEAD_GRIP_UP = 0.5  # the TCP this fraction of the head's thickness above its underside
SEAT_PRESS = 0.003  # the commanded descent goes this far past the head on the rim: the arm presses a few N
CLEAR_ABOVE = 0.006  # the cone's tip passes this far above the neck's rim on the way over
ROT = yam_demo.grip_rotation(0.0)  # straight down, fingers closing along y

_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


def arm_plan(size, side, wait_s=0.0):
    key = size["name"]
    hd, ht, cl, rim = size["head_d"], size["head_t"], size["cone_l"], size["rim"]
    open_, hold = -(hd + 0.022) / 2, -(hd - SQUEEZE) / 2
    sx, sy = STOPPER_XY[key]
    fx, fy = FLASK_XY[key]
    grip_z = TOP + cl + REST_GAP + HEAD_GRIP_UP * ht   # the TCP's height with the stopper standing on its tip
    carry_z = TOP + rim + CLEAR_ABOVE + cl + HEAD_GRIP_UP * ht   # the cone's tip clears the rim
    seat_z = TOP + rim + HEAD_GRIP_UP * ht   # the head's underside on the rim
    pre_z = seat_z + 0.014
    w = [Waypoint(wait_s, _START_TCP[side], FINGER_OPEN, "wait", rot=_START_ROT[side])] if wait_s else []
    w += [
        Waypoint(2.5, [sx, sy, grip_z + HOVER], open_, f"over the {size['label']} stopper", rot=ROT),
        Waypoint(1.5, [sx, sy, grip_z], open_, "down on the head", rot=ROT),
        Waypoint(0.8, [sx, sy, grip_z], hold, "grip the head", rot=ROT),
        Waypoint(1.5, [sx, sy, carry_z], hold, "lift the stopper", rot=ROT),
        Waypoint(2.0, [fx, fy, carry_z], hold, "over the neck", rot=ROT),
        Waypoint(1.2, [fx, fy, pre_z], hold, "down to the mouth", rot=ROT),
        Waypoint(3.0, [fx, fy, seat_z - SEAT_PRESS], hold, "lower the cone into the socket", rot=ROT),
        Waypoint(0.6, [fx, fy, seat_z - SEAT_PRESS], hold, "hold", rot=ROT),
        Waypoint(0.6, [fx, fy, seat_z - SEAT_PRESS], open_, "let go of the stopper", rot=ROT),
        Waypoint(1.2, [fx, fy, seat_z + 0.03], open_, "lift away", rot=ROT),
        Waypoint(2.5, _START_TCP[side], FINGER_OPEN, "home", rot=_START_ROT[side]),
    ]
    return w


def demo():
    """Each arm closes its own flask; both go home."""
    return {"left": arm_plan(BIG, "left"), "right": arm_plan(SMALL, "right")}
