"""On the station table: a hand clipper lying loose on its flat back with a number 2 guide comb clipped onto its
blade, and to the right of it a tray holding five more numbered guide combs (1, 3, 4, 5, 6) with one slot empty. The
left arm holds the clipper still; the right arm pulls the number 2 guide off, puts it back in the tray's empty slot,
picks the number 4 guide (by the number printed on its roof), slides it onto the blade and pushes it on until its
rear clip catches. Then both arms let go.

The clipper is a free rigid body (280 g, 160 mm). Each guide is one articulation (``assets/guard_N.usda``): the comb
is the root and its rear clip, a sprung arm with a hook, is on a hinge (joint ``Clip``, 0 with the hook down,
positive lifts it). The clip is real contact: the hook rides up the front slope of a catch ridge on the clipper's blade
collar and drops behind it, so pushing a guide on takes about 3 N and pulling it off about 5 N; the loose clipper
(friction about 1.4 N) would slide away from either. The tray is a fixture.

Run it:  inspect-robots run --embodiment yam_arena -E task=clipper_guard_swap.py --scorer task_score ...

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
TAG = "clipper_guard"

# -- the clipper and the guides (frames as in assets/build: clipper frame = origin on the table at the blade collar's
# front face, +x toward the blade tip; guide frame = origin on the table at the comb's rear edge, +x toward its teeth) --
BLADE_LEN = 45 * MM  # the blade head's length from the collar's face
CLIPPER_BOX = (np.array([-115 * MM, -28 * MM, 0.0]), np.array([BLADE_LEN, 28 * MM, 38 * MM]))
CLIPPER_MASS = 0.280
LENGTHS = {1: 3.2, 2: 6.4, 3: 9.5, 4: 12.7, 5: 15.9, 6: 19.1}  # cutting length (mm): number / 8 inch
G_LEN, G_HALF_W = 53 * MM, 27 * MM
HOOK_X = (-10.5 * MM, -8.5 * MM)  # the clip's hook, in the guide frame, behind the collar's ridge when seated


def roof_top(n: int) -> float:
    """The guide's roof height above the table (m): the blade's top face (11.5 mm) + the cutting length + 1.5 mm."""
    return (11.5 + LENGTHS[n] + 1.5) * MM


def guard_box(n: int):
    return (np.array([-11 * MM, -G_HALF_W, 0.0]), np.array([G_LEN, G_HALF_W, roof_top(n) + 0.6 * MM]))


GUARD_MASS = {1: 0.0137, 2: 0.0144, 3: 0.0150, 4: 0.0156, 5: 0.0163, 6: 0.0170}  # kg, plastic comb + clip arm
TARGET, REMOVE = 4, 2  # the number to fit and the number that is on the clipper at the start
GUARDS = tuple(sorted(LENGTHS))

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The clipper lies along y with its blade tip toward -y (the right arm's side) and its rear end toward +y; the guides
# are turned the same way (teeth toward -y), so one is pushed onto the blade by moving +y.
CLIP_YAW = -math.pi / 2
CLIPPER_XY = (0.42, -0.010)  # the blade collar's front face on the centre line
TRAY_XY = (0.42, -0.215)
TRAY_T = 2 * MM
TRAY_COLS = (-0.070, 0.0, 0.070)  # slot centres across (world x) from the tray's centre
TRAY_ROWS = (0.033, -0.033)  # ... along the guides (world y); the first row is nearer the clipper
# (column, row) of each guide's slot; guide 2 starts on the clipper, its slot is the empty one
SLOTS = {1: (0, 1), 3: (2, 0), 4: (0, 0), 2: (1, 0), 5: (1, 1), 6: (2, 1)}


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


CLIP_R = _rz(CLIP_YAW)
CLIPPER_POS = np.array([*CLIPPER_XY, TOP])
TRAY_POS = np.array([*TRAY_XY, TOP])


def slot_origin(n: int) -> np.ndarray:
    """A guide's origin (its rear edge on its centre line, on the tray's face) in its tray slot."""
    c, r = SLOTS[n]
    return np.array([TRAY_XY[0] + TRAY_COLS[c], TRAY_XY[1] + TRAY_ROWS[r] + G_LEN / 2, TOP + TRAY_T])


def start_pos(n: int) -> np.ndarray:
    return CLIPPER_POS.copy() if n == REMOVE else slot_origin(n)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: a rigid body, an articulation with a free
    root, or a fixture (``kinematic``). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


def _tuple(v) -> tuple[float, float, float]:
    return tuple(float(x) for x in v)


CLIPPER = UsdPart("clipper", asset="clipper", xy=CLIPPER_XY, extent=0.16, mass=CLIPPER_MASS,
                  usd_path=str(ASSETS / "clipper.usda"), pos=_tuple(CLIPPER_POS), rot_xyzw=_yaw_xyzw(CLIP_YAW))
TRAY = UsdPart("tray", asset="tray", xy=TRAY_XY, extent=0.24, mass=0.25, usd_path=str(ASSETS / "tray.usda"),
               pos=_tuple(TRAY_POS), kinematic=True)
GUARD_PARTS = {n: UsdPart(f"guard{n}", asset=f"guard{n}", xy=tuple(start_pos(n)[:2]), extent=0.055, mass=GUARD_MASS[n],
                          usd_path=str(ASSETS / f"guard_{n}.usda"), pos=_tuple(start_pos(n)),
                          rot_xyzw=_yaw_xyzw(CLIP_YAW), articulation=True) for n in GUARDS}
OBJECTS = [TRAY, CLIPPER, *[GUARD_PARTS[n] for n in GUARDS]]
NAMES = {n: GUARD_PARTS[n].name for n in GUARDS}

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
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=32, solver_velocity_iteration_count=1)
    return scene.Object(
        name=spec.name, usd_path=spec.usd_path,
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


def _clip_angle(art) -> float:
    ids = art.find_joints(["Clip"], preserve_order=True)[0]
    return float(art.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)[0])


def _reset_guards(env, env_ids, asset_cfg) -> None:
    """Reset event: a guide's clip arm down (the simulator keeps an articulation's joints where they were)."""
    import torch

    if env_ids is None:
        return
    art = env.scene[asset_cfg.name]
    ids = art.find_joints(["Clip"], preserve_order=True)[0]
    zero = torch.zeros((len(env_ids), len(ids)), device=art.device)
    art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root pose and velocity (Arena's geometry queries take rigid objects only; an articulation's
        bounds are left at its root), the grippers, the episode time and each guide's clip angle (``<name>_clip``,
        rad, positive lifts the hook)."""
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
        for n in GUARDS:
            out[f"{NAMES[n]}_clip"] = _clip_angle(base.scene[NAMES[n]])
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event per guide (its clip arm down)."""
        configure = stock_cfg_callback(control_hz)

        def with_guards(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            for n in GUARDS:
                setattr(env_cfg.events, f"guard{n}_reset",
                        EventTermCfg(func=_reset_guards, mode="reset", params={"asset_cfg": SceneEntityCfg(NAMES[n])}))
            return env_cfg

        return with_guards

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


# -- scoring ----------------------------------------------------------------------------------------------------------
# A guide's pose is read in the clipper's frame: ``gap`` is how far its rear edge is from the collar's face along the
# blade (0 when seated), ``side`` and ``up`` its offsets across and in height, ``yaw`` and ``tilt`` its angles.
SEATED_GAP = (-0.0008, 0.0025)  # m: the rear edge within this of the collar's face (the clip's play is ~1.7 mm)
SEATED_SIDE, SEATED_UP = 0.003, 0.0025
SEATED_YAW, SEATED_TILT = math.radians(3.0), math.radians(4.0)
CLIP_DOWN = math.radians(1.5)  # the clip's hook is down (behind the ridge) at most this far lifted
ON_GAP, ON_SIDE, ON_UP = 0.030, 0.004, 0.003  # started on: the blade at least 15 mm inside, aligned
ON_YAW, ON_TILT = math.radians(6.0), math.radians(6.0)
OFF_GAP = 0.050  # a guide is off the blade once its rear edge is this far past the collar (the tip is at 45 mm)
LIFTED = 0.012  # m: a guide held this far above where it started counts as lifted
TOUCH = 0.005  # m: a gripper's pad span within this of a guide's box has a hand on it
GRIP_FORCE, GRIP_OPENING = 0.5, 0.003
CLEAR = 0.03  # m: released = no pinch and the pad span at least this far from the clipper and every guide
UNDISTURBED = 0.015  # m: the guides left in the tray stay this close to where they were ...
UNDISTURBED_TURN = math.radians(15.0)  # ... and within this angle
FALLEN_Z = -0.02  # m above the table: lower than this has fallen off
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)  # settled: within 2 mm and 2 degrees over 1 s
TRACE_S = 1.0
STEPS = (f"number {REMOVE} guide off the clipper", f"number {TARGET} guide lifted from the tray",
         f"number {TARGET} guide on the blade", f"number {TARGET} guide clipped on")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def guard_frame(objects, n: int):
    """Guide ``n``'s pose in the clipper's frame: (position, rotation matrix)."""
    c, g = objects[CLIPPER.name], objects[NAMES[n]]
    rc = _rot(c)
    return rc.T @ (np.asarray(g.pos, float) - c.pos), rc.T @ _rot(g)


def _angles(r: np.ndarray) -> tuple[float, float]:
    """(yaw about the vertical, tilt from upright) of a rotation matrix, rad."""
    return float(math.atan2(r[1, 0], r[0, 0])), float(math.acos(min(1.0, max(-1.0, r[2, 2]))))


def pose_of(objects, n: int) -> dict:
    p, r = guard_frame(objects, n)
    yaw, tilt = _angles(r)
    return {"gap": float(p[0]), "side": float(p[1]), "up": float(p[2]), "yaw": yaw, "tilt": tilt}


def seated(objects, n: int) -> bool:
    """Guide ``n`` is pushed fully onto the blade: its rear edge against the collar's face, square and level."""
    q = pose_of(objects, n)
    return bool(SEATED_GAP[0] <= q["gap"] <= SEATED_GAP[1] and abs(q["side"]) < SEATED_SIDE and abs(q["up"]) < SEATED_UP
                and abs(q["yaw"]) < SEATED_YAW and q["tilt"] < SEATED_TILT)


def clipped(objects, n: int) -> bool:
    """Guide ``n`` is seated and its clip's hook has dropped behind the catch ridge (the arm is down)."""
    return bool(seated(objects, n) and abs(objects[f"{NAMES[n]}_clip"]) < CLIP_DOWN)


def on_blade(objects, n: int) -> bool:
    """Guide ``n`` is on the blade by at least 15 mm, aligned with it."""
    q = pose_of(objects, n)
    return bool(-0.001 <= q["gap"] <= ON_GAP and abs(q["side"]) < ON_SIDE and abs(q["up"]) < ON_UP
                and abs(q["yaw"]) < ON_YAW and q["tilt"] < ON_TILT)


def off_clipper(objects, n: int) -> bool:
    """Guide ``n`` is clear of the blade: not within reach of it on the blade's side of the clipper."""
    p, _ = guard_frame(objects, n)
    over = bool(-0.06 < p[0] < OFF_GAP and abs(p[1]) < 0.04 and -0.01 < p[2] < 0.035)
    return not over


def _pad_points(g) -> list[np.ndarray]:
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _dist_to_box(point_local: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    d = np.maximum(np.maximum(lo - point_local, point_local - hi), 0.0)
    return float(np.linalg.norm(d))


def _gap_to_guard(objects, g, n: int) -> float:
    """The pad span's smallest distance to guide ``n``'s box (m)."""
    s = objects[NAMES[n]]
    r = _rot(s)
    lo, hi = guard_box(n)
    return min(_dist_to_box(r.T @ (p - s.pos), lo, hi) for p in _pad_points(g))


def _gap_to_clipper(objects, g) -> float:
    c = objects[CLIPPER.name]
    r = _rot(c)
    return min(_dist_to_box(r.T @ (p - c.pos), *CLIPPER_BOX) for p in _pad_points(g))


def _grippers(objects):
    return [g for g in (objects.get("left_gripper"), objects.get("right_gripper")) if g is not None]


def guard_gripped(objects, n: int) -> bool:
    """A gripper holds guide ``n``: its pads reach it, both fingers press, and the pads are apart (not closed on
    nothing)."""
    return any(g.contact >= GRIP_FORCE and g.opening >= GRIP_OPENING and _gap_to_guard(objects, g, n) <= TOUCH
               for g in _grippers(objects))


def released(objects) -> bool:
    """No gripper pinches, and each gripper's pad span is at least CLEAR from the clipper and every guide."""
    for g in _grippers(objects):
        if g.contact >= GRIP_FORCE:
            return False
        if _gap_to_clipper(objects, g) < CLEAR or any(_gap_to_guard(objects, g, n) < CLEAR for n in GUARDS):
            return False
    return True


def lifted(objects, n: int) -> bool:
    """Guide ``n`` is held and at least LIFTED above where it started."""
    s = objects[NAMES[n]]
    return bool(guard_gripped(objects, n) and s.pos[2] > start_pos(n)[2] + LIFTED)


def tray_undisturbed(objects) -> bool:
    """The guides that were not touched by the task (all but REMOVE and TARGET) are where they started."""
    for n in GUARDS:
        if n in (REMOVE, TARGET):
            continue
        s = objects[NAMES[n]]
        turn = math.acos(min(1.0, max(-1.0, (float(np.trace(CLIP_R.T @ _rot(s))) - 1.0) / 2.0)))
        if np.linalg.norm(np.asarray(s.pos, float) - start_pos(n)) > UNDISTURBED or turn > UNDISTURBED_TURN:
            return False
    return True


def others_clear(objects) -> bool:
    """No guide but the target is on the blade or the clipper's collar (the removed one is off it)."""
    return bool(all(off_clipper(objects, n) for n in GUARDS if n != TARGET))


def _poses(objects) -> dict:
    out = {}
    for name in (CLIPPER.name, *NAMES.values()):
        s = objects[name]
        out[name] = (np.array(s.pos, float), np.array(s.quat_xyzw, float))
    return out


def still(objects, anchor) -> bool:
    """Every part within STILL_MOVE and STILL_TURN of its pose in ``anchor``."""
    now = _poses(objects)
    for name, (pos0, q0) in anchor.items():
        pos, q = now[name]
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(q0, q)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


def _violation(objects) -> bool:
    """Scores 0: the clipper or a guide off the table, or a state that is not finite."""
    values = []
    for name in (CLIPPER.name, *NAMES.values()):
        values += list(objects[name].pos)
    if not np.all(np.isfinite(values)):
        return True
    return bool(any(objects[name].pos[2] < TOP + FALLEN_Z for name in (CLIPPER.name, *NAMES.values())))


class _Progress:
    """What has happened so far this episode, updated once per control step: whether the removed guide has come off
    the blade, the target guide has been lifted, and has been on the blade with a hand on it, and since when every
    full-success condition has held without a break (``met_since``, ``None`` while one fails) with the poses when
    they began to hold (``anchor``). Any lapse restarts the count. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.removed = False
        self.lifted = False
        self.fitted = False
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
        self.removed |= bool(off_clipper(objects, REMOVE))
        self.lifted |= bool(lifted(objects, TARGET))
        self.fitted |= bool(on_blade(objects, TARGET) and guard_gripped(objects, TARGET))
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not _violation(objects) and tray_undisturbed(objects)
                   and released(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a condition undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` from the episode's progress so far (see ``steps_done``)."""
    p = _PROGRESS
    removed = bool(p.removed)
    return [removed, bool(p.lifted), bool(p.fitted),
            bool(removed and p.lifted and clipped(objects, TARGET) and others_clear(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``. Step 1 counts once the removed guide has come off the blade; step 2 once the target guide has
    been held and lifted; step 3 once it has been on the blade, aligned, with a hand on it; step 4 as things stand:
    the target guide seated with its clip's hook dropped, no other guide on the blade, after steps 1 and 2."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        q2, q4 = pose_of(objects, REMOVE), pose_of(objects, TARGET)
        line = (f"(no.{REMOVE} gap {q2['gap'] * 1000:.0f} mm; no.{TARGET} gap {q4['gap'] * 1000:.1f} side "
                f"{q4['side'] * 1000:+.1f} up {q4['up'] * 1000:+.1f} mm yaw {math.degrees(q4['yaw']):+.1f} tilt "
                f"{math.degrees(q4['tilt']):.1f} deg, clip {math.degrees(objects[NAMES[TARGET] + '_clip']):.1f} deg, "
                f"gripped {guard_gripped(objects, TARGET)}; released {released(objects)}; tray "
                f"{'undisturbed' if tray_undisturbed(objects) else 'DISTURBED'})")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S: all four steps done, no violation, the
    guides left in the tray where they were, both grippers released, and the clipper and every guide within STILL_MOVE
    and STILL_TURN of where they were when that second began. The arms' return is not scored."""
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
# Left arm: from above, fingers closing across the clipper's housing (42 mm) behind the blade collar; it holds the
# clipper still on the table until the right arm has let go of the guide it fitted, then lets go and goes home.
# Right arm: from above, fingers closing across the number 2 guide's side walls (54 mm); it pulls it off along the
# blade, carries it to the tray's empty slot and lets go; then grips the number 4 guide in its slot, carries it
# to the blade's tip with its mouth toward the tip, slides it on and pushes it on past the clip, lets go and goes home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (about 20 N a finger)
CLIPPER_W = 0.042
HOLD_C = -(CLIPPER_W - SQUEEZE) / 2
AROUND_C = -(CLIPPER_W + 0.012) / 2
G_W = 2 * G_HALF_W
HOLD_G = -(G_W - SQUEEZE) / 2
AROUND_G = -(G_W + 0.012) / 2
GRIP_YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along world x (across the clipper and guides)
GRIP_XC = 0.034  # m from a guide's rear edge: where the right hand grips its walls
TIPS_G = 0.005  # m: the pad tips this far above a guide's floor
HOLD_AT = -0.100  # m from the collar's face along the clipper: where the left hand grips the housing
TIPS_C = 0.006
CARRY = 0.035  # m: guides are carried this high
SEAT_AIM = 0.008  # m: the push is commanded this far past the seat (the collar's face stops the guide)
PULL = 0.085  # m: the removed guide is pulled to its rear edge this far past the collar's face
STAGE_GAP = 0.020  # m: before the push, the guide's rear edge is this far short of the blade's tip
TABLE_PRESS = -0.0005  # m: the guide's floor is commanded this far below the table (the arm's sag lifts it)
LEFT_IDLE = False  # (tests only) the left arm stays home


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

_Z = np.array([0.0, 0.0, 1.0])
LEAN = {"left": math.radians(-25.0), "right": math.radians(25.0)}  # each gripper leans about world x, its wrist away from
# the other arm (the two wrists would otherwise be too close over the clipper)
GRIP_ROT = {side: axis_angle_matrix(np.array([1.0, 0.0, 0.0]), LEAN[side]) @ yam_demo.grip_rotation(yaw)
            for side, yaw in GRIP_YAW.items()}


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


def _gw(origin, p) -> np.ndarray:
    """A point given in a guide's (or the clipper's) frame, the frame's origin at ``origin``, in the world."""
    return np.asarray(origin, float) + CLIP_R @ np.asarray(p, float)


def _guard_tcp(origin, lift: float = 0.0, press: float = 0.0) -> np.ndarray:
    """The right TCP gripping a guide whose origin is at ``origin``, ``lift`` higher."""
    return _tcp_with_tips(_gw(origin, [GRIP_XC, 0.0, TIPS_G + press]), 0.0, GRIP_ROT["right"]) + (0.0, 0.0, lift)


def _clipper_tcp() -> np.ndarray:
    return _tcp_with_tips(_gw(CLIPPER_POS, [HOLD_AT, 0.0, TIPS_C]), 0.0, GRIP_ROT["left"])


def left_plan(t_release: float) -> tuple[list[Waypoint], float]:
    """Grip the clipper's housing and hold it until ``t_release`` (s); let go, go home. Also returns when it holds."""
    RL = GRIP_ROT["left"]
    grip = _clipper_tcp()
    if LEFT_IDLE:
        return [Waypoint(max(0.1, t_release), _START_TCP["left"], FINGER_OPEN, "stay home", rot=_START_ROT["left"])], 0.0
    wps = [
        Waypoint(2.5, grip + (0, 0, 0.04), AROUND_C, "above the clipper", rot=RL),
        Waypoint(1.5, grip, AROUND_C, "around the housing", rot=RL),
        Waypoint(0.8, grip, HOLD_C, "grip the housing", rot=RL),
    ]
    holding = sum(w.duration for w in wps)
    wps += [
        Waypoint(max(0.1, t_release - holding), grip, HOLD_C, "hold the clipper still", rot=RL),
        Waypoint(0.6, grip, AROUND_C, "let go", rot=RL),
        Waypoint(1.0, grip + (0, 0, 0.04), AROUND_C, "up", rot=RL),
        Waypoint(1.5, grip + (0, 0, 0.04) + CLIP_R @ np.array([-0.07, 0, 0]), AROUND_C, "away", rot=RL),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps, holding


def right_plan(t_hand_on: float) -> tuple[list[Waypoint], float]:
    """From ``t_hand_on`` (s, the left hand holding the clipper): pull the number 2 guide off, put it in the empty slot,
    fit the number 4 guide; let go, go home. Also returns when it lets go of the number 4 guide."""
    R = GRIP_ROT["right"]
    seat = CLIPPER_POS.copy()
    pulled = seat + CLIP_R @ np.array([PULL, 0.0, 0.0])
    slot2 = slot_origin(REMOVE)
    slot4 = slot_origin(TARGET)
    stage = seat + CLIP_R @ np.array([BLADE_LEN - STAGE_GAP, 0.0, 0.0])
    # (the staging origin is one whole guide length short of the tip along the blade: the guide's mouth faces the tip)
    stage = seat + CLIP_R @ np.array([BLADE_LEN + 0.012, 0.0, 0.0])
    over = lambda o, z=CARRY: _guard_tcp(o, z)
    wps = [
        Waypoint(max(0.1, t_hand_on - 2.5), _START_TCP["right"], FINGER_OPEN, "wait", rot=_START_ROT["right"]),
        Waypoint(2.5, over(seat), AROUND_G, f"above the number {REMOVE} guide", rot=R),
        Waypoint(1.5, _guard_tcp(seat), AROUND_G, "around its walls", rot=R),
        Waypoint(0.8, _guard_tcp(seat), HOLD_G, "grip it", rot=R),
        Waypoint(3.0, _guard_tcp(pulled), HOLD_G, "pull it off the blade", rot=R),
        Waypoint(1.2, over(pulled), HOLD_G, "lift it", rot=R),
        Waypoint(2.5, over(slot2), HOLD_G, "over the empty slot", rot=R),
        Waypoint(1.5, _guard_tcp(slot2 + (0, 0, 0.0005)), HOLD_G, "lower it into the slot", rot=R),
        Waypoint(0.6, _guard_tcp(slot2 + (0, 0, 0.0005)), AROUND_G, "let go", rot=R),
        Waypoint(1.0, over(slot2), AROUND_G, "up", rot=R),
        Waypoint(2.5, over(slot4), AROUND_G, f"above the number {TARGET} guide", rot=R),
        Waypoint(1.5, _guard_tcp(slot4), AROUND_G, "around its walls", rot=R),
        Waypoint(0.8, _guard_tcp(slot4), HOLD_G, "grip it", rot=R),
        Waypoint(1.2, over(slot4, 0.025), HOLD_G, "lift it out of the tray", rot=R),
    ]
    # carry it toward the clipper first (its mouth stays toward the tip), then across to the blade's line
    mid = np.array([slot4[0], stage[1], 0.0])
    on_line = np.array([stage[0], stage[1], 0.0])
    wps += [
        Waypoint(2.5, _guard_tcp(np.array([*mid[:2], slot4[2]]), 0.025), HOLD_G, "carry it toward the clipper", rot=R),
        Waypoint(2.0, _guard_tcp(np.array([*on_line[:2], slot4[2]]), 0.025), HOLD_G, "line it up with the blade", rot=R),
        Waypoint(1.2, _guard_tcp(np.array([*on_line[:2], TOP]), 0.0, TABLE_PRESS), HOLD_G, "set it on the table", rot=R),
    ]
    first = seat + CLIP_R @ np.array([BLADE_LEN + 0.002, 0.0, 0.0])
    push_to = seat + CLIP_R @ np.array([-SEAT_AIM, 0.0, 0.0])
    wps += [
        Waypoint(1.2, _guard_tcp(np.array([*first[:2], TOP]), 0.0, TABLE_PRESS), HOLD_G, "bring its mouth to the tip", rot=R),
        Waypoint(4.0, _guard_tcp(np.array([*push_to[:2], TOP]), 0.0, TABLE_PRESS), HOLD_G, "slide it on and push", rot=R),
        Waypoint(0.8, _guard_tcp(np.array([*push_to[:2], TOP]), 0.0, TABLE_PRESS), HOLD_G, "hold", rot=R),
    ]
    t_on = sum(w.duration for w in wps)
    end = np.array([*push_to[:2], TOP])
    wps += [
        Waypoint(0.6, _guard_tcp(end, 0.0, TABLE_PRESS), AROUND_G, "let go", rot=R),
        Waypoint(1.0, _guard_tcp(end, 0.035), AROUND_G, "up", rot=R),
        Waypoint(1.5, _guard_tcp(np.array([*(seat + CLIP_R @ np.array([0.105, 0, 0]))[:2], TOP]), 0.035), AROUND_G, "away", rot=R),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return wps, t_on


def demo():
    """The left hand holds the clipper still; the right pulls the number 2 guide off, puts it in the tray, fits the
    number 4 guide and lets go; the left lets go and both go home."""
    _, holding = left_plan(0.0)
    _, t_on = right_plan(holding + 0.3)
    left, _ = left_plan(t_on + 1.0)
    right, _ = right_plan(holding + 0.3)
    return {"left": left, "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
