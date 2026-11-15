"""On the station table: a small frame of 20 x 20 mm aluminium extrusion for a sliding panel: a bottom and a top rail, 220 mm
long, their T-slots facing each other, and an end post at each end (the one at the robot's left is the end stop). A 100 x 80 x
3 mm clear acrylic panel (37 g) with a pull knob on its front face stands in the two slots, 60 mm from the stop.

The slots are the standard 2020 slot (a 6.2 mm opening between 1.8 mm lips over an 11 mm wide, 6 mm deep cavity); the panel
has 1.6 mm a side in the opening and its edges are 4.5 mm into the top slot and 7.8 mm into the bottom one, so it is trapped
between the rails and can only slide along them; pulled unevenly (the knob is 20 mm in front of the panel) it can cock in
the slots and bind.

The left arm takes the knob between its fingers, slides the panel along the slots until its end meets the stop, lets go and
backs off.

Both objects are rigid bodies from their own USD files in ``assets/``: the frame is a fixture, the panel is free. Colliders are
boxes (the lips, walls and floors of both slots).

Run it:  inspect-robots run --embodiment yam_arena -E task=extrusion_sliding_panel.py --scorer task_score ...

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




TAG = "extrusion_sliding_panel"

# -- the frame (its asset frame: origin on the table at the middle of the frame, in the slots' plane) -------------------
P = 20.0 * MM
OPEN_W = 6.2 * MM
LIP = 1.8 * MM
CAV_D = 6.0 * MM
RAIL_HALF = 110.0 * MM
FLOOR_Z = P - LIP - CAV_D  # the bottom slot's cavity floor above the table: the panel's lower edge sits on it

# -- the panel (its frame: origin at the middle of the lower edge; thickness along x, width along y, up z) ---------------
PANEL_W, PANEL_H, PANEL_T = 100.0 * MM, 80.0 * MM, 3.0 * MM
PANEL_MASS = 0.037
PANEL_LO, PANEL_HI = np.array([-PANEL_T / 2, -PANEL_W / 2, 0.0]), np.array([PANEL_T / 2, PANEL_W / 2, PANEL_H])
KNOB_LO, KNOB_HI = np.array([-27.5 * MM, -8.0 * MM, 47.0 * MM]), np.array([-PANEL_T / 2, 8.0 * MM, 63.0 * MM])
PANEL_PARTS = ((PANEL_LO, PANEL_HI), (KNOB_LO, KNOB_HI))
KNOB_HEAD = np.array([-20.5 * MM, 0.0, 55.0 * MM])  # the knob head's middle (16 mm across in y)
STOP_Y = RAIL_HALF  # the stop post's face
TRAVEL = 0.060  # m: the panel's middle starts this far from where it ends

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ------------------------------
FRAME_XY = (0.34, 0.0)
FRAME_POS = np.array([*FRAME_XY, TOP])
START_Y = STOP_Y - PANEL_W / 2 - TRAVEL  # the panel's middle at the start (frame frame)
PANEL_START = np.array([FRAME_XY[0], FRAME_XY[1] + START_Y, TOP + FLOOR_Z + 0.0003])

FRAME = UsdPart("frame", asset="frame", xy=FRAME_XY, extent=0.25, mass=None, usd_path=str(ASSETS / "slide_frame.usda"),
                pos=tuple(float(v) for v in FRAME_POS), kinematic=True)
PANEL = UsdPart("panel", asset="panel", xy=FRAME_XY, extent=0.12, mass=PANEL_MASS, usd_path=str(ASSETS / "sliding_panel.usda"),
                pos=tuple(float(v) for v in PANEL_START))
OBJECTS = [FRAME, PANEL]


# -- scoring ----------------------------------------------------------------------------------------------------------
MOVED = 0.010  # m: the panel pulled this far along the slots, held, counts as started
HALF_WAY = 0.030  # m: step 2: pulled at least this far
AT_STOP = 0.002  # m: step 3: the panel's end within this of the stop
IN_X = OPEN_W / 2 - PANEL_T / 2 + 0.0003  # m: the panel's plane within this of the slots' axis
SEAT_Z = 0.0015  # m: its lower edge within this of the cavity floor
LEAN = math.radians(3.0)
POPPED = 0.006  # m: its lower edge this far above the floor: the panel has come out of the bottom slot
CLEAR = 0.03  # m: both grippers' pads at least this far from the panel and its knob
FALLEN = math.radians(30.0)
FALLEN_Z = -0.02
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)
TRACE_S = 1.0
STEPS = ("panel started along the slots by a gripper", "panel pulled over half way", "panel at the end stop, in the slots",
         "panel let go")
POINTS = 0.2


def _panel(objects):
    p = objects["panel"]
    return np.array(p.pos, float), _rot(p)


def panel_pose(objects) -> dict:
    """The panel against the frame: ``x`` its plane's offset from the slots' axis (m), ``y`` its middle along the slots
    (m, frame frame), ``low`` its lower edge above the cavity floor (m), ``lean`` its tilt from upright, ``yaw`` its turn
    about the vertical from square to the rails (rad), ``end`` how far its leading end is from the stop (m)."""
    pos, r = _panel(objects)
    if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(r))):
        return {k: math.nan for k in ("x", "y", "low", "lean", "yaw", "end")}
    q = pos - FRAME_POS
    return {"x": float(q[0]), "y": float(q[1]), "low": float(q[2] - FLOOR_Z),
            "lean": float(math.acos(min(1.0, max(-1.0, r[2, 2])))), "yaw": float(math.atan2(r[0, 1], r[1, 1])),
            "end": float(STOP_Y - (q[1] + PANEL_W / 2))}


def panel_held(objects) -> bool:
    """A gripper holds the panel or its knob: its pads reach one of them and both fingers press."""
    pos, r = _panel(objects)
    return any(_grips(g) and any(_in_box(q, pos, r, lo, hi, GRIP_SLACK) for q in _pads(g) for lo, hi in PANEL_PARTS)
               for g in _grippers(objects))


def in_slots(objects) -> bool:
    """The panel is in both slots: its plane within IN_X of the axis, its lower edge within POPPED of the floor, upright
    and square within LEAN."""
    s = panel_pose(objects)
    return bool(np.isfinite(s["x"]) and abs(s["x"]) <= IN_X and s["low"] <= POPPED and s["lean"] <= LEAN
                and abs(s["yaw"]) <= LEAN)


def at_stop(objects) -> bool:
    s = panel_pose(objects)
    return bool(in_slots(objects) and -0.001 <= s["end"] <= AT_STOP)


def let_go(objects) -> bool:
    pos, r = _panel(objects)
    return not (panel_held(objects) or any(_in_box(q, pos, r, lo, hi, CLEAR) for g in _grippers(objects)
                                           for q in _pads(g) for lo, hi in PANEL_PARTS))


def _poses(objects):
    p = objects["panel"]
    return np.array(p.pos, float), np.array(p.quat_xyzw, float)


def still(objects, anchor) -> bool:
    pos, quat = _poses(objects)
    pos0, quat0 = anchor
    turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
    return bool(np.linalg.norm(pos - pos0) <= STILL_MOVE and turn <= STILL_TURN)


def _violation(objects) -> bool:
    """Scores 0: the panel off the table or tipped over (more than 30 degrees), the frame moved from where it is fixed, or a
    state that is not finite."""
    p, f = objects["panel"], objects["frame"]
    if not np.all(np.isfinite([*p.pos, *p.quat_xyzw, *f.pos])):
        return True
    s = panel_pose(objects)
    return bool(p.pos[2] < TOP + FALLEN_Z or s["lean"] > FALLEN or np.linalg.norm(f.pos - FRAME_POS) > 0.005)


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper holding the panel or its
    knob has pulled it along the slots, and since when every full-success condition has held without a break
    (``met_since``) with the panel's pose then (``anchor``). A lapse restarts the count. Starts over when the episode
    time goes back."""

    def __init__(self):
        self.time = -1.0
        self.moved = False
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
        s = panel_pose(objects)
        self.moved |= bool(np.isfinite(s["y"]) and s["y"] >= START_Y + MOVED and panel_held(objects))
        if t is None:
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'panel moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    p = _PROGRESS
    s = panel_pose(objects)
    half = bool(p.moved and in_slots(objects) and s["y"] >= START_Y + HALF_WAY)
    stop = bool(half and at_stop(objects))
    free = bool(stop and let_go(objects))
    return [bool(p.moved), half, stop, free]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached (a gripper holding the panel or its knob has pulled it 1 cm along the
    slots), steps 2 to 4 as things stand, each only with the one before."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        s = panel_pose(objects)
        line = (f"(panel: travelled {(s['y'] - START_Y) * 1000:.1f} mm, end {s['end'] * 1000:.1f} mm from the stop, x "
                f"{s['x'] * 1000:.1f} mm, edge {s['low'] * 1000:.1f} mm up, lean {math.degrees(s['lean']):.1f} yaw "
                f"{math.degrees(s['yaw']):.1f} deg, held {panel_held(objects)})")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S: all four steps done (the last one: both grippers
    clear), no violation, and the panel within STILL_MOVE and STILL_TURN of where it was when that second began. The arms'
    return is not scored."""
    p = _PROGRESS
    steps_done(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success; otherwise 0.2 per step done (``STEPS``, at most 0.8, also while the second is being timed); 0 for
    any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: pointing ahead and 25 degrees down, fingers closing across the knob head (along y), pads on the head; grips it, slides the panel 60 mm
# along the slots to the stop (a slow pull, the arm level with the knob), lets go, lifts off and goes home.
# the gripper points ahead and 25 degrees down, fingers closing along y: the hand stays in front of the frame (pointing
# straight down, its body would meet the top rail)
GRIP_TILT = math.radians(25.0)  # leaned down from horizontal (the arm reaches a knob this low only that way)
GRIP_ROT = axis_angle_matrix(_Y, -math.pi / 2 + GRIP_TILT) @ _grip(0.0)
HEAD_W = 16.0 * MM
HOLD = -(HEAD_W - 0.008) / 2
OPEN = -(HEAD_W + 0.030) / 2
TCP_ABOVE_HEAD = 0.0 * MM  # the TCP at the knob head's middle
OVERSHOOT = 0.0005  # m: pulled this far past where the panel meets the stop (the arm is soft)


def _knob_tcp(panel_origin) -> np.ndarray:
    return np.asarray(panel_origin, float) + KNOB_HEAD + np.array([0.0, 0.0, TCP_ABOVE_HEAD])


def left_plan() -> list[Waypoint]:
    rot = GRIP_ROT
    pick = _knob_tcp(PANEL_START)
    end = pick + (0.0, TRAVEL + OVERSHOOT, 0.0)
    return [
        Waypoint(2.5, pick - 0.06 * rot[:, 2], FINGER_OPEN, "in front of the knob", rot=rot),
        Waypoint(1.5, pick, OPEN, "pads astride the knob head", rot=rot),
        Waypoint(0.8, pick, HOLD, "grip the knob", rot=rot),
        Waypoint(3.5, end, HOLD, "slide the panel to the stop", rot=rot),
        Waypoint(0.5, end, HOLD, "hold", rot=rot),
        Waypoint(0.6, end, OPEN, "let go", rot=rot),
        Waypoint(1.0, end - 0.06 * rot[:, 2], OPEN, "back off", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


def demo():
    """The left arm takes the knob, slides the panel along the slots to the stop, lets go and goes home."""
    return {"left": left_plan()}
