"""On the station table, from the arms outward: an 18650 flashlight (110 mm long, black anodised aluminium) standing
head-down on its bezel midway between the arms, its screw-on tail cap up and its battery inside; then, in front of
it, an open cardboard box (20 x 17 cm, 10 cm tall), fixed in place. The left arm holds the flashlight body; the right
arm unscrews the tail cap (one turn counter-clockwise, seen from above, frees it) and sets it on the table; then the
flashlight is turned tail-down over the box so the battery slides out into it, and stood back on the table.

Scoring: 0.2 for each of ``STEPS`` (body held, cap off, cap set on the table, battery in the box, flashlight set back
on the table); the grippers' state comes from the simulator (``GripperState``), so a part that falls or is dropped
is not "set" down.

The flashlight is three rigid bodies (body, tail cap, battery) from ``assets/``; the cap's 4-start thread collides as
modelled. The box is NVIDIA's SimReady open cardboard box (CC-BY 4.0), downloaded once from Hugging Face.

Run it:  inspect-robots run --embodiment yam_arena -E task=flashlight_battery.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

import hashlib
import importlib.abc
import importlib.util
import json
import math
import os
import sys
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.task_file import TABLE_TOP, Asset
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
ASSETS = Path(__file__).resolve().parent / "assets"

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
FLASH_XY = (0.30, -0.02)  # the flashlight's axis
BOX_XY = (0.50, -0.02)  # the box's centre, in front of the flashlight

# -- the flashlight (its own frame: tail at z = 0, head at z = 0.110, axis z) -----------------------------------------
LENGTH = 0.110
LEAD = 0.010  # cap travel per turn: 4-start thread, 2.5 mm pitch
CAP_HEIGHT = 0.0208
CAP_DIAMETER = 0.026
TUBE_DIAMETER = 0.022
HEAD_DIAMETER = 0.034
# Head down: the flashlight's frame turned 180 deg about x, so a point at height h in it is at world z = Z0 - h.
Z0 = TOP + LENGTH + 0.0005
HEAD_DOWN_XYZW = (1.0, 0.0, 0.0, 0.0)

# -- the box: SimReady open cardboard box A06 (61 x 50.5 x 30.2 cm) at a third of its size, fixed ----------------------
BOX_SCALE = 1 / 3
BOX_HALF = (0.611 / 2 * BOX_SCALE, 0.505 / 2 * BOX_SCALE)  # half length (x) and half width (y): 10.2, 8.4 cm
BOX_HEIGHT = 0.302 * BOX_SCALE  # its tall walls: 10.1 cm
SIMREADY_REPO = "nvidia/simready-assets"  # Hugging Face dataset
SIMREADY_REVISION = "2464ca32165bfff81ba34bc703ddb932822cfd27"
BOX_PACKAGE, BOX_USD = "box_cardboard_open_a06", "sm_box_cardboard_open_a06_01.usd"


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned as one rigid body at a full pose, with the physics authored in it. ``Asset``'s fields only
    satisfy the task-file checks. ``usd_path`` is a local file, or empty for the SimReady box (fetched at spawn)."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    scale: float = 1.0
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


def _flashlight_part(name: str, usd: str, mass: float) -> UsdPart:
    return UsdPart(
        name, asset=name, xy=FLASH_XY, extent=0.034, mass=mass,
        usd_path=str(ASSETS / usd), pos=(*FLASH_XY, Z0), rot_xyzw=HEAD_DOWN_XYZW,
    )


BODY = _flashlight_part("flashlight_body", "flashlight_body.usda", 0.070)
CAP = _flashlight_part("flashlight_cap", "flashlight_cap.usda", 0.018)
BATTERY = _flashlight_part("flashlight_battery", "flashlight_battery.usda", 0.047)
BOX = UsdPart(
    "box", asset="box", xy=BOX_XY, extent=2 * BOX_HALF[0],
    pos=(*BOX_XY, TOP + 0.001), scale=BOX_SCALE, kinematic=True,
)
OBJECTS = [BODY, CAP, BATTERY, BOX]


def fetch_box() -> Path:
    """The SimReady box's root USD, downloaded once (pinned revision) into ``~/.cache/simready`` (``SIMREADY_CACHE``
    overrides). Only its USD layers, textures and licence are fetched, about 50 MB."""
    root = Path(os.environ.get("SIMREADY_CACHE", "~/.cache/simready")).expanduser() / SIMREADY_REPO / SIMREADY_REVISION
    target, done = root / BOX_PACKAGE / "simready_usd" / BOX_USD, root / BOX_PACKAGE / ".complete"
    if done.is_file() and target.is_file():
        return target

    def get(url: str) -> bytes:
        headers = {"User-Agent": "robocurve-benchmark-task"}
        if os.environ.get("HF_TOKEN"):
            headers["Authorization"] = f"Bearer {os.environ['HF_TOKEN']}"
        with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=120) as response:
            return response.read()

    hf = "https://huggingface.co"
    listing = json.loads(get(f"{hf}/api/datasets/{SIMREADY_REPO}/tree/{SIMREADY_REVISION}/{BOX_PACKAGE}?recursive=true"))
    for entry in listing:
        path = entry["path"]
        wanted = path == f"{BOX_PACKAGE}/.metadata/LICENSE.txt" or (
            path.startswith(f"{BOX_PACKAGE}/simready_usd/") and "/.thumbs/" not in path
            and not (path.startswith(f"{BOX_PACKAGE}/simready_usd/source_assets/") and path.endswith(".png"))
        )
        dest = root / path
        if entry.get("type") != "file" or not wanted or (dest.is_file() and dest.stat().st_size == entry.get("size")):
            continue
        data = get(f"{hf}/datasets/{SIMREADY_REPO}/resolve/{SIMREADY_REVISION}/{path}")
        sha = (entry.get("lfs") or {}).get("oid")
        if sha and hashlib.sha256(data).hexdigest() != sha:
            raise OSError(f"checksum mismatch for {path}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.with_suffix(dest.suffix + ".part").write_bytes(data)
        dest.with_suffix(dest.suffix + ".part").replace(dest)
    if not target.is_file():
        raise FileNotFoundError(f"{SIMREADY_REPO}@{SIMREADY_REVISION[:8]}: no {BOX_PACKAGE}/simready_usd/{BOX_USD}")
    done.write_text(SIMREADY_REVISION)
    return target


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``: its physics as authored in the USD, kinematic (fixed) if asked."""
    addon = {}
    if spec.kinematic:
        addon.update(func=scene._spawn_fixture, rigid_props=scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True))
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path or str(fetch_box()),
        object_type=scene.ObjectType.RIGID,
        scale=(spec.scale,) * 3,
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


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """The stock object states, plus the grippers and the episode time for this file's scoring."""
        return {**stock_states(env, names), **_gripper_states(env)}

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


# -- scoring ----------------------------------------------------------------------------------------------------------
# Centres in metres; heights above the table top. Assembled, the cap's centre is 4.7 cm from the body's and the
# battery's 1.1 cm; the battery is fully out of the tube once its centre is more than 8.5 cm from the body's.
CAP_OFF = 0.08  # m from the body's centre: the cap is off the flashlight
BATTERY_OUT = 0.09  # m from the body's centre: the battery is out of the flashlight
ON_TABLE = 0.01  # m: a part's lowest point within this of the table top: it rests on the table
SET_DOWN = 0.02  # m: a gripper let go of a part with its lowest point within this of the table: it was set down
INSIDE_MARGIN = 0.01  # m: the battery's centre at least this far inside the box's outer walls
IN_BOX_Z = 0.06  # m above the table: the battery's centre is lower, so it lies in the box (not held over it)
FALLEN_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something
# with at least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
HOLD_S = 1.0  # step 1: the body held this long
# Step 2 needs the cap unscrewed: turned at least this far relative to the body before it comes off (it frees after
# about one turn). A cap forced off the thread without that is a simulation artefact (real threads don't pull
# apart), and the trial scores 0.
UNSCREW_TURN = math.radians(300)
STEPS = ("body held", "cap off", "cap set on the table", "battery in the box", "flashlight set back on the table")
POINTS = 0.2  # per step


def _over_box(pos, margin: float = 0.0) -> bool:
    return bool(abs(pos[0] - BOX_XY[0]) < BOX_HALF[0] - margin and abs(pos[1] - BOX_XY[1]) < BOX_HALF[1] - margin)


def _dist(objects, a: str, b: str) -> float:
    return float(np.linalg.norm(objects[a].pos - objects[b].pos))


def _violation(objects) -> bool:
    """Scores 0: a non-finite state, an object off the table, or the flashlight body dropped into the box."""
    states = [objects[o.name] for o in OBJECTS]
    if not all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.lin_vel)) for s in states):
        return True
    if any(objects[o.name].pos[2] < TOP + FALLEN_Z for o in OBJECTS if o.free):
        return True
    body = objects["flashlight_body"].pos
    return _over_box(body) and body[2] < TOP + BOX_HEIGHT


def _cap_off(objects) -> bool:
    return _dist(objects, "flashlight_cap", "flashlight_body") > CAP_OFF


def _cap_on_table(objects) -> bool:
    cap = objects["flashlight_cap"]
    return _cap_off(objects) and cap.low[2] < TOP + ON_TABLE and not _over_box(cap.pos)


def _battery_out(objects) -> bool:
    return _dist(objects, "flashlight_battery", "flashlight_body") > BATTERY_OUT


def _battery_in_box(objects) -> bool:
    battery = objects["flashlight_battery"].pos
    return _battery_out(objects) and _over_box(battery, INSIDE_MARGIN) and battery[2] < TOP + IN_BOX_Z


def _inside(p, part) -> bool:
    return bool(np.all(p >= part.low - GRIP_SLACK) and np.all(p <= part.high + GRIP_SLACK))


def _on_body(p, objects) -> bool:
    """``p`` is on the flashlight body itself: within the head's radius of its axis and, with the cap on, past the
    cap's edge. The body's threaded end sits inside the cap, so a grip on the cap is not one on the body."""
    body, cap = objects["flashlight_body"], objects["flashlight_cap"]
    if not _inside(p, body):
        return False
    axis = quat_xyzw_to_matrix(body.quat_xyzw)[:, 2]
    d = p - body.pos
    if np.linalg.norm(d - (d @ axis) * axis) > HEAD_DIAMETER / 2 + GRIP_SLACK:
        return False
    if _cap_off(objects):
        return True
    toward_body = np.sign((body.pos - cap.pos) @ axis)
    return bool((p - cap.pos) @ axis * toward_body > CAP_HEIGHT / 2 + GRIP_SLACK)


def _held(on, gripper: GripperState) -> bool:
    """``gripper`` holds a part: its pads reach it (``on`` of a point along them) and both fingers press."""
    if gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    return any(on(gripper.back + s * (gripper.tips - gripper.back)) for s in np.linspace(0.0, 1.0, 11))


def _held_by_any(objects, name: str) -> bool:
    grippers = [g for g in (objects.get("left_gripper"), objects.get("right_gripper")) if g is not None]
    if name == "flashlight_body":
        return any(_held(lambda p: _on_body(p, objects), g) for g in grippers)
    return any(_held(lambda p: _inside(p, objects[name]), g) for g in grippers)


def _resting_on_table(objects, name: str) -> bool:
    part = objects[name]
    return bool(part.low[2] < TOP + ON_TABLE and not _over_box(part.pos))


class _Progress:
    """What has happened so far this episode; ``score`` and ``success`` read the steps that depend on history (a
    part held, then let go). Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.body_held_since = None
        self.body_held = False  # step 1
        self.cap_off = False  # step 2
        self.twist = 0.0  # rad the cap has turned relative to the body while on it
        self.last_angle = None
        self.forced_off = False  # the cap came off without being unscrewed
        self.battery_was_in_box = False
        self.holding = {"flashlight_cap": False, "flashlight_body": False}
        self.set_down = {"flashlight_cap": False, "flashlight_body": False}  # the last let-go was a set-down
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return  # success() and score() both read the same step
            if t < self.time:
                self.__init__()
            self.time = t
        held = {name: _held_by_any(objects, name) for name in self.holding}
        if held["flashlight_body"]:
            now = 0.0 if t is None else t
            self.body_held_since = now if self.body_held_since is None else self.body_held_since
            self.body_held |= now - self.body_held_since >= HOLD_S or t is None
        else:
            self.body_held_since = None
        if not self.cap_off:
            body, cap = objects["flashlight_body"], objects["flashlight_cap"]
            rel = quat_xyzw_to_matrix(body.quat_xyzw).T @ quat_xyzw_to_matrix(cap.quat_xyzw)
            angle = math.atan2(rel[1, 0], rel[0, 0])  # the cap's turn about the flashlight's axis
            if self.last_angle is not None:
                self.twist += (angle - self.last_angle + math.pi) % (2 * math.pi) - math.pi
            self.last_angle = angle
            if _cap_off(objects):
                self.cap_off = True
                self.forced_off = abs(self.twist) < UNSCREW_TURN
                if "episode_time" in objects:
                    print(f"[flashlight_battery] t={objects['episode_time']:.2f}s  cap off after "
                          f"{math.degrees(abs(self.twist)):.0f} deg of unscrewing"
                          + (" -- forced off the thread: the trial scores 0" if self.forced_off else ""), flush=True)
        self.battery_was_in_box |= _battery_in_box(objects)
        for name in self.holding:
            if self.holding[name] and not held[name]:  # a gripper let go: low enough, beside the box, in order?
                part = objects[name]
                in_order = name == "flashlight_cap" or self.battery_was_in_box
                self.set_down[name] = bool(part.low[2] <= TOP + SET_DOWN and not _over_box(part.pos) and in_order)
            elif held[name]:
                self.set_down[name] = False
            self.holding[name] = held[name]


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: done (steps 1 and 2 once reached; 3 to 5 as things stand now)."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(p, objects)
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, d in zip(STEPS, done, strict=True) if d) or "none"
        print(f"[flashlight_battery] t={objects['episode_time']:.2f}s  steps done: {got}", flush=True)
    return done


def _steps(p, objects) -> list[bool]:
    return [
        bool(done)
        for done in (
            p.body_held,
            p.cap_off,
            p.set_down["flashlight_cap"] and _cap_on_table(objects) and not p.holding["flashlight_cap"],
            _battery_in_box(objects),
            p.set_down["flashlight_body"] and _resting_on_table(objects, "flashlight_body")
            and not p.holding["flashlight_body"],
        )
    ]


def success(objects) -> bool:
    """All five steps done, the three parts still, and no violation."""
    parts = ("flashlight_body", "flashlight_cap", "flashlight_battery")
    return bool(
        all(steps_done(objects)) and not _PROGRESS.forced_off and not _violation(objects)
        and all(objects[n].resting() for n in parts)
    )


def score(objects) -> float:
    """0.2 per step done (``STEPS``); 0 for any violation or a cap forced off its thread. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects) or _PROGRESS.forced_off:
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: a sideways grip on the body tube. Right arm: unscrew the cap with straight-down grips, rising with the
# thread, then set it on the table. Left arm: lift, carry over the box, roll the flashlight tail-down to pour, then
# roll it back upright and stand it on its mark, head down.
LEFT_APPROACH = -math.pi / 2  # the left gripper points along -y, from its own side
LEFT_ROLL = 0.0  # fingers close horizontally across the upright body
# The pour rolls the gripper about its pointing axis; joint6 stops at +-120 deg, so 115 deg (not 180): the tail ends
# 25 deg below horizontal, well past where the battery slides. Negative swings the tail toward +x, the box.
POUR_ROLL = -math.radians(115)
POUR_XY = (0.45, -0.02)  # the flashlight's axis while pouring: its tail swings ~5 cm on, over the box
LIFT_Z = TOP + 0.20  # gripper height while carrying and pouring
GRIP_ON_BODY = 0.050  # left grip along the tube, clear of the cap and the collar at 21-23 mm
BODY_GRIP_REACH = 0.035  # the left pad tips 3.5 cm past the body's axis: the whole pads, not the tips
CAP_GRIP_DEPTH = 0.018  # the right pad tips 1.8 cm down the cap's side
# Two straight-down grips of 200 deg each (400 deg, a little over the one turn that frees the cap). Over the cap the
# right arm's joint6 is ~44 deg - yaw, so its yaw stays within -76..+164 deg. The re-grip comes while the cap is still
# on the thread; the last twist keeps hold and lifts the cap straight off.
TWIST = math.radians(200)
TWIST_START = -math.radians(70)
TWISTS = 2
TWIST_STEPS = 2  # waypoints per twist, each under 180 deg so the rotation is interpolated the right way round
CAP_LIFT = 0.025  # higher, a straight-down grip runs into joint4's limit
CAP_SET_XY = (0.28, -0.20)  # where the cap is set down, beside the right arm
CARRY_YAW = math.radians(60)  # unwind the wrist while carrying the cap
# Grips close the fingers to SQUEEZE inside the part's width, not fully: they stop on it pressing ~20 N each (5000 N/m
# finger stiffness x 4 mm) instead of driving at the 50 N limit, which squeezes a round part out. A finger target of
# -w/2 opens the pads to w (FINGER_OPEN, -0.047, is 94 mm).
SQUEEZE = 0.008
CAP_HOLD = -(CAP_DIAMETER - SQUEEZE) / 2
BODY_HOLD = -(TUBE_DIAMETER - SQUEEZE) / 2
BODY_APPROACH = -0.050 / 2  # the left fingers open 5 cm coming in, their tips clear of the box
# Setting it back down: the head pressed 2 mm onto the table so it stands flat, the fingers opened wide, slowly, and
# the gripper lifted straight up off the tube (sliding the tapered fingertips off sideways tips it over).
PUT_DOWN_PRESS = 0.002


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

_DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
_X, _Y, _Z = np.eye(3)


def side_grip(approach: float, roll: float) -> np.ndarray:
    """The gripper pointing horizontally along ``approach`` (rad from +x), fingers closing horizontally at roll 0,
    turned ``roll`` about its pointing axis."""
    forward = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN  # pointing +x, fingers closing along y
    return axis_angle_matrix(_Z, approach) @ axis_angle_matrix(_X, roll) @ forward


def _tcp_for_pads(target_xy, approach: float, reach: float):
    """The TCP's x, y that put the pad tips ``reach`` past ``target_xy`` along the pointing direction."""
    d = np.array([math.cos(approach), math.sin(approach)])
    return tuple(np.asarray(target_xy) + d * (reach - PAD_TIP_PAST_TCP))


LEFT_GRIP_S = 2.5 + 2.0 + 0.8  # the left arm's approach and grip (see left_plan)


def right_plan() -> list[Waypoint]:
    """Wait for the left grip, unscrew the cap in two grips rising with the thread, set it on the table."""
    x, y = FLASH_XY
    grip_z = Z0 + PAD_TIP_PAST_TCP - CAP_GRIP_DEPTH
    rise = LEAD * TWIST / (2 * math.pi)
    a, b = TWIST_START, TWIST_START + TWIST
    plan = [  # wind the wrist to the start yaw in two steps, turning the side that stays inside joint6's range
        Waypoint(LEFT_GRIP_S / 2, (x, y, grip_z + 0.05), FINGER_OPEN, "wait for the left grip", a / 2),
        Waypoint(LEFT_GRIP_S / 2, (x, y, grip_z + 0.05), FINGER_OPEN, "wind the wrist", a),
        Waypoint(1.5, (x, y, grip_z), FINGER_OPEN, "down to the cap", a),
    ]
    z = grip_z
    for k in range(TWISTS):
        plan.append(Waypoint(0.6, (x, y, z), CAP_HOLD, f"grip {k + 1}", a))
        for i in range(1, TWIST_STEPS + 1):
            f = i / TWIST_STEPS
            plan.append(Waypoint(2.0, (x, y, z + rise * f), CAP_HOLD, f"twist {k + 1}", a + TWIST * f))
        z += rise
        if k < TWISTS - 1:  # let go, turn back up clear of the cap, come down again
            plan += [
                Waypoint(0.5, (x, y, z), FINGER_OPEN, f"let go {k + 1}", b),
                Waypoint(0.6, (x, y, z + 0.03), FINGER_OPEN, f"rise {k + 1}", b),
                Waypoint(0.6, (x, y, z + 0.03), FINGER_OPEN, f"turn back {k + 1}", (a + b) / 2),
                Waypoint(0.6, (x, y, z + 0.03), FINGER_OPEN, f"turn back {k + 1}", a),
                Waypoint(0.6, (x, y, z), FINGER_OPEN, f"down {k + 1}", a),
            ]
    set_z = TOP + CAP_HEIGHT + 0.002 + PAD_TIP_PAST_TCP - CAP_GRIP_DEPTH
    plan += [
        Waypoint(1.5, (x, y, z + CAP_LIFT), CAP_HOLD, "lift the cap off", b),
        Waypoint(2.0, (*CAP_SET_XY, z + CAP_LIFT), CAP_HOLD, "carry the cap", CARRY_YAW),
        Waypoint(1.5, (*CAP_SET_XY, set_z), CAP_HOLD, "set the cap down", CARRY_YAW),
        Waypoint(0.6, (*CAP_SET_XY, set_z), FINGER_OPEN, "release the cap", CARRY_YAW),
        Waypoint(1.5, (*CAP_SET_XY, set_z + 0.10), FINGER_OPEN, "clear away", CARRY_YAW),
    ]
    return plan


def left_plan(wait_s: float) -> list[Waypoint]:
    """Grip the body from the side, hold it while the cap comes off, carry it over the box and tip it tail-down,
    then stand it back on its mark, head down, and let go."""
    grip_z = Z0 - GRIP_ON_BODY
    rot = side_grip(LEFT_APPROACH, LEFT_ROLL)
    half = side_grip(LEFT_APPROACH, (LEFT_ROLL + POUR_ROLL) / 2)
    over = side_grip(LEFT_APPROACH, POUR_ROLL)
    at = _tcp_for_pads(FLASH_XY, LEFT_APPROACH, BODY_GRIP_REACH)
    back = _tcp_for_pads(FLASH_XY, LEFT_APPROACH, -0.04)
    pour = _tcp_for_pads(POUR_XY, LEFT_APPROACH, BODY_GRIP_REACH)
    return [
        Waypoint(2.5, (*back, grip_z), BODY_APPROACH, "beside the flashlight", rot=rot),
        Waypoint(2.0, (*at, grip_z), BODY_APPROACH, "around the body", rot=rot),
        Waypoint(0.8, (*at, grip_z), BODY_HOLD, "grip the body", rot=rot),
        Waypoint(wait_s, (*at, grip_z), BODY_HOLD, "hold while the cap comes off", rot=rot),
        Waypoint(2.0, (*at, LIFT_Z), BODY_HOLD, "lift", rot=rot),
        Waypoint(2.5, (*pour, LIFT_Z), BODY_HOLD, "over the box", rot=rot),
        Waypoint(1.5, (*pour, LIFT_Z), BODY_HOLD, "tip sideways", rot=half),
        Waypoint(1.5, (*pour, LIFT_Z), BODY_HOLD, "tail down: pour", rot=over),
        Waypoint(3.0, (*pour, LIFT_Z), BODY_HOLD, "let the battery fall", rot=over),
        Waypoint(1.5, (*pour, LIFT_Z), BODY_HOLD, "tip back", rot=half),
        Waypoint(1.5, (*pour, LIFT_Z), BODY_HOLD, "upright", rot=rot),
        Waypoint(2.5, (*at, LIFT_Z), BODY_HOLD, "back to its mark", rot=rot),
        Waypoint(2.0, (*at, grip_z + 0.02), BODY_HOLD, "down near the table", rot=rot),
        Waypoint(1.5, (*at, grip_z - PUT_DOWN_PRESS), BODY_HOLD, "stand it on the table", rot=rot),
        Waypoint(0.5, (*at, grip_z - PUT_DOWN_PRESS), BODY_HOLD, "settle", rot=rot),
        Waypoint(1.0, (*at, grip_z - PUT_DOWN_PRESS), FINGER_OPEN, "let go", rot=rot),
        Waypoint(1.5, (*at, LIFT_Z), FINGER_OPEN, "lift off the top", rot=rot),
    ]


def demo():
    """Both arms: the right arm's plan, and the left arm's, which holds until the cap is off."""
    right = right_plan()
    return {"left": left_plan(wait_s=sum(w.duration for w in right) - LEFT_GRIP_S + 0.5), "right": right}
