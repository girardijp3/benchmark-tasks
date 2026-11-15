"""On the station table: a 3-way corner cube connector for 20 x 20 mm aluminium extrusion, fixed, with two rail stubs in
its side sockets (one along +x, one along -y) as the corner of a frame, and its third socket open upward; and a post
rack holding a 100 mm length of 2020 extrusion standing upright (43 g).

The up socket is 22 x 22 mm and 22 mm deep, with 4 mm lead-in chamfers (1 mm a side around the 20 mm profile). The post
has to be lowered into it square and plumb, centred within about a millimetre, until its end sits on the socket's floor:
a post resting on the rim, leaning on a wall or left standing in the socket's mouth is not seated.

The left arm lifts the post out of the rack by its upper end, carries it over the cube, lowers it into the socket until
it is seated, lets go and backs off.

All three objects are rigid bodies from their own USD files in ``assets/``: the cube (with its stubs) and the rack are
fixtures, the post is free. Colliders are boxes and small convex prisms (the socket walls with their chamfer).

Run it:  inspect-robots run --embodiment yam_arena -E task=extrusion_corner_cube.py --scorer task_score ...

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
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body or a fixture
    (``kinematic``). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic




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


def _grip(yaw: float) -> np.ndarray:
    return yam_demo.grip_rotation(yaw)


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}


_X, _Y, _Z = np.eye(3)
GRIP_SLACK = 0.005  # a gripper's pads this close to a part's box count as reaching it
GRIP_FORCE = 0.5  # N each finger: both fingers press on something
GRIP_OPENING = 0.002  # the fingers are not closed on nothing


def _rz(a: float) -> np.ndarray:
    return axis_angle_matrix(_Z, a)


def _xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(r))


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _in_box(point, origin, rot, lo, hi, slack: float) -> bool:
    q = rot.T @ (np.asarray(point, float) - origin)
    return bool(np.all(q >= lo - slack) and np.all(q <= hi + slack))


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def _grips(gripper) -> bool:
    return bool(gripper is not None and gripper.contact >= GRIP_FORCE and gripper.opening >= GRIP_OPENING)


def _grippers(objects):
    return [g for g in (objects.get("left_gripper"), objects.get("right_gripper")) if g is not None]


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
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon={"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)},
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
        """Each object's root pose and speeds (Arena's geometry queries take stock objects only, so bounds are left
        at the root), the grippers and the episode time."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            data = base.scene[name].data

            def arr(t):
                return t.torch[0].detach().cpu().numpy().astype(np.float64)

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




TAG = "extrusion_corner_cube"

# -- parts ------------------------------------------------------------------------------------------------------------
PROFILE = 20.0 * MM
POST_L = 100.0 * MM
POST_MASS = 0.043
POST_LO = np.array([-PROFILE / 2, -PROFILE / 2, 0.0])  # the post's frame: origin at the middle of its lower end
POST_HI = np.array([PROFILE / 2, PROFILE / 2, POST_L])
CUBE = 34.0 * MM
SOCK = 22.0 * MM
FLOOR = 12.0 * MM  # the up socket's floor above the table
CUBE_TOP = CUBE  # the cube's top face above the table
RACK_H, RACK_DEPTH = 30.0 * MM, 18.0 * MM

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ------------------------------
CUBE_XY = (0.30, -0.02)
CUBE_POS = np.array([*CUBE_XY, TOP])
RACK_XY = (0.30, 0.22)
RACK_POS = np.array([*RACK_XY, TOP])
POST_START = np.array([RACK_XY[0], RACK_XY[1], TOP + RACK_H - RACK_DEPTH + 0.0003])

CUBE_PART = UsdPart("cube", asset="cube", xy=CUBE_XY, extent=0.2, mass=None, usd_path=str(ASSETS / "corner_cube.usda"),
                    pos=tuple(float(v) for v in CUBE_POS), kinematic=True)
RACK = UsdPart("rack", asset="rack", xy=RACK_XY, extent=0.06, mass=None, usd_path=str(ASSETS / "post_rack.usda"),
               pos=tuple(float(v) for v in RACK_POS), kinematic=True)
POST = UsdPart("post", asset="post", xy=RACK_XY, extent=0.1, mass=POST_MASS, usd_path=str(ASSETS / "post.usda"),
               pos=tuple(float(v) for v in POST_START))
OBJECTS = [CUBE_PART, RACK, POST]


# -- scoring ----------------------------------------------------------------------------------------------------------
LIFTED = 0.02  # m: the post at least this far above where it stood, held, counts as picked up
ENGAGE = 0.008  # m: its lower end at least this far below the cube's top, inside the socket's straight walls
IN_OFF = 0.002  # m: its axis within this of the socket's axis (the socket has 1 mm a side)
IN_TILT = math.radians(8.0)
SEAT_Z = 0.0015  # m: its lower end at most this far above the socket's floor
SEAT_OFF = 0.0015
SEAT_TILT = math.radians(3.0)
CLEAR = 0.03  # m: both grippers' pads at least this far from the post
FALLEN = math.radians(45.0)
FALLEN_Z = -0.02
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)
TRACE_S = 1.0
STEPS = ("post lifted out of the rack", "post's lower end inside the socket", "post seated on the socket's floor",
         "post let go")
POINTS = 0.2  # per step; the four steps give 0.8: only success scores 1.0


def post_pose(objects) -> dict:
    """The post against the socket: ``off`` its lower end's distance from the socket's axis (m), ``low`` its lower end
    above the table (m), ``tilt`` its axis from vertical (rad)."""
    p = objects["post"]
    pos, r = np.array(p.pos, float), _rot(p)
    if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(r))):
        return {"off": math.nan, "low": math.nan, "tilt": math.nan}
    return {"off": float(np.linalg.norm((pos - CUBE_POS)[:2])), "low": float(pos[2] - TOP),
            "tilt": float(math.acos(min(1.0, max(-1.0, r[2, 2]))))}


def post_held(objects) -> bool:
    """A gripper holds the post: its pads reach the post's box and both fingers press."""
    p = objects["post"]
    r = _rot(p)
    return any(_grips(g) and any(_in_box(q, p.pos, r, POST_LO, POST_HI, GRIP_SLACK) for q in _pads(g))
               for g in _grippers(objects))


def in_socket(objects) -> bool:
    """The post's lower end is inside the socket's straight walls (at least ENGAGE below the cube's top), its axis
    within IN_OFF of the socket's axis and within IN_TILT of vertical."""
    s = post_pose(objects)
    return bool(np.isfinite(s["low"]) and s["low"] <= CUBE_TOP - ENGAGE and s["off"] <= IN_OFF and s["tilt"] <= IN_TILT)


def seated(objects) -> bool:
    """In the socket and on its floor: lower end within SEAT_Z of the floor, centred within SEAT_OFF, within SEAT_TILT
    of vertical."""
    s = post_pose(objects)
    return bool(in_socket(objects) and s["low"] <= FLOOR + SEAT_Z and s["off"] <= SEAT_OFF and s["tilt"] <= SEAT_TILT)


def let_go(objects) -> bool:
    """No gripper holds the post or has its pads within CLEAR of it."""
    p = objects["post"]
    r = _rot(p)
    return not (post_held(objects) or any(_in_box(q, p.pos, r, POST_LO, POST_HI, CLEAR)
                                          for g in _grippers(objects) for q in _pads(g)))


def _poses(objects):
    p = objects["post"]
    return np.array(p.pos, float), np.array(p.quat_xyzw, float)


def still(objects, anchor) -> bool:
    pos, quat = _poses(objects)
    pos0, quat0 = anchor
    turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
    return bool(np.linalg.norm(pos - pos0) <= STILL_MOVE and turn <= STILL_TURN)


def _violation(objects) -> bool:
    """Scores 0: the post off the table or fallen over (more than 45 degrees), the cube moved from where it is fixed, or
    a state that is not finite."""
    p, c = objects["post"], objects["cube"]
    values = [*p.pos, *p.quat_xyzw, *c.pos]
    if not np.all(np.isfinite(values)):
        return True
    s = post_pose(objects)
    return bool(p.pos[2] < TOP + FALLEN_Z or s["tilt"] > FALLEN or np.linalg.norm(c.pos - CUBE_POS) > 0.005)


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper has lifted the post out of
    the rack, and since when every full-success condition has held without a break (``met_since``) with the post's pose
    then (``anchor``). Any lapse (a step undone, a violation, or the post moving more than STILL_MOVE or turning more than
    STILL_TURN from the anchor) restarts the count. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.picked = False
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
        self.picked |= bool(objects["post"].pos[2] > POST_START[2] + LIFTED and post_held(objects))
        if t is None:
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'post moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    p = _PROGRESS
    inn = bool(p.picked and in_socket(objects))
    on = bool(inn and seated(objects))
    free = bool(on and let_go(objects))
    return [bool(p.picked), inn, on, free]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached (a gripper has lifted the post 2 cm out of the rack), steps 2 to 4 as
    things stand, each only with the one before."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        s = post_pose(objects)
        line = (f"(post: off {s['off'] * 1000:.1f} mm, lower end {s['low'] * 1000:.1f} mm up (floor {FLOOR * 1000:.0f}), "
                f"tilt {math.degrees(s['tilt']):.1f} deg, held {post_held(objects)})")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all four steps done (the last one: both grippers clear), no violation, and the post within STILL_MOVE and
    STILL_TURN of where it was when that second began. The arms' return is not scored."""
    p = _PROGRESS
    steps_done(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from above, fingers closing across the post (along y), pads on its upper 35 mm; lifts it out of the rack,
# carries it over the cube with its lower end 8 mm over the cube's top, lowers it into the socket until it sits on the
# floor, lets go, lifts off and goes home.
GRIP_YAW = 0.0  # fingers close along y
WIDTH = PROFILE
HOLD = -(WIDTH - 0.008) / 2  # about 20 N a finger
OPEN = -(WIDTH + 0.030) / 2
TCP_BELOW_TOP = 18.0 * MM  # the TCP this far below the post's top end
CARRY_CLEAR = 0.008  # m: the post's lower end this far over the cube's top while it is carried
AIM = np.array([0.0, 0.0, 0.0]) * MM
SET_DOWN = 0.0008  # m: the post's lower end this far over the socket's floor when it is let go
BACK_OFF = 0.05


def _tcp_for(origin) -> np.ndarray:
    return np.asarray(origin, float) + np.array([0.0, 0.0, POST_L - TCP_BELOW_TOP])


def left_plan() -> list[Waypoint]:
    rot = _grip(GRIP_YAW)
    pick = _tcp_for(POST_START)
    lift = (0.0, 0.0, CUBE_TOP + CARRY_CLEAR - (RACK_H - RACK_DEPTH))
    over = np.array([CUBE_POS[0], CUBE_POS[1], POST_START[2]]) + lift
    goal = np.array([CUBE_POS[0], CUBE_POS[1], TOP + FLOOR + SET_DOWN])
    return [
        Waypoint(2.5, pick + (0, 0, 0.06), FINGER_OPEN, "over the post", rot=rot),
        Waypoint(1.5, pick, OPEN, "pads astride the post", rot=rot),
        Waypoint(0.8, pick, HOLD, "grip the post", rot=rot),
        Waypoint(2.0, pick + lift, HOLD, "lift it out of the rack", rot=rot),
        Waypoint(3.0, _tcp_for(over) + AIM, HOLD, "over the cube", rot=rot),
        Waypoint(3.0, _tcp_for(goal) + AIM, HOLD, "lower it into the socket", rot=rot),
        Waypoint(0.5, _tcp_for(goal) + AIM, HOLD, "hold", rot=rot),
        Waypoint(0.6, _tcp_for(goal) + AIM, OPEN, "let go", rot=rot),
        Waypoint(1.0, _tcp_for(goal) + AIM + (0, 0, 0.03), OPEN, "lift off", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


def demo():
    """The left arm lifts the post out of the rack, carries it over the cube, lowers it into the socket, lets go and goes
    home."""
    return {"left": left_plan()}
