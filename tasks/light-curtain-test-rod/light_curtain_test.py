"""On the station table: two bench machines, each guarded by a safety light curtain (a 30 mm-resolution Type 4 pair of
posts, protected height 150 mm, 400 mm between the lenses) and each with its own 30 mm test rod resting on two saddles
near the robot. A red 'DO NOT USE' tag stands in a rack between the machines, and each machine has an empty tag
holder. Each receiver has a lamp on its top cap: green while its field is clear, red while a beam is interrupted. One
receiver has a dead beam channel (drawn anew every episode): the 30 mm rod only trips its lamp where it covers a working beam.

The rod is pushed through each field, one pass at the bottom, one at the middle (centred on the dead beam's
height, the only place the rod covers that beam alone) and one at the top, while the lamp is watched; the tag goes in
the holder of the machine whose curtain missed the rod, after that curtain's passes (the exposing one included).

The curtains, holders, saddles and rack are fixtures; the rods and the tag are rigid bodies. The lamp is a declared
code effect: a red and a green disc (no collider, no gravity) that the task swaps on the receiver's cap, red while a
working beam is covered by a rod (a rod covers a beam when its cross-section holds the whole 10 mm lens), green
otherwise. It never moves, holds or places anything and does not decide success.

Run it:  inspect-robots run --embodiment yam_arena -E task=light_curtain_test.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

import importlib.abc
import importlib.util
import math
import os
import random
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

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "light_curtain"

# -- the light curtain (frame: origin on the table at the field's footprint, x normal to the field, y along the beams)
POST_Y = 0.22  # the posts' centres, either side: 400 mm between the lens faces
FIELD_HALF = 0.20  # the beams run this far either side of the footprint's middle
BEAMS, PITCH, FIRST = 8, 0.020, 0.057  # beam heights above the table: 57 mm + 20 mm k (field 52 to 202 mm)
FIELD_LO, FIELD_HI = 0.052, 0.202
LENS_R = 0.005
HOUSING_TOP = 0.242
ROD_R, ROD_L = 0.015, 0.30  # the test rod: 30 mm across, 300 mm long
ROD_AXIS_REST = 0.027  # its axis above the table on the saddles
DEAD_BEAM = 3  # the failed channel: 57 + 60 = 117 mm, in the field's middle third
FAULTY = "right"  # the machine whose receiver has the dead channel: drawn anew for every run and every episode (below)

CUR_X = 0.44
CURTAIN_Y = {"left": 0.38, "right": -0.38}
HOLDER_POS = {"left": (0.29, 0.49), "right": (0.29, -0.27)}
RACK_POS = (0.26, 0.0)
TIP_BEFORE, TIP_PAST = 0.05, 0.08  # a pass starts with the tip this far before the plane and ends this far past it
LANE_X0 = CUR_X - TIP_BEFORE - ROD_L / 2  # the rod rests on its saddles here, in front of its field
LANE_X1 = CUR_X + TIP_PAST - ROD_L / 2
ROD_XY = {s: (LANE_X0, y) for s, y in CURTAIN_Y.items()}
SIDES = ("left", "right")
FLOOR = 0.006  # the holder's slot floor above the table
TAG_W, TAG_T, TAG_H = 0.076, 0.005, 0.127

_Z = np.array([0.0, 0.0, 1.0])


def draw_faulty() -> str:
    """Draws which machine's receiver has the dead channel, the way the real station's person flips a coin. Every run
    and every episode draws again. ``LIGHT_CURTAIN_FAULTY=left|right`` in the environment fixes the draw (used to film
    and check each case); it changes nothing else."""
    global FAULTY
    forced = os.environ.get("LIGHT_CURTAIN_FAULTY", "")
    FAULTY = forced if forced in ("left", "right") else random.SystemRandom().choice(("left", "right"))
    print(f"[{TAG}] faulty machine this episode: {FAULTY}", flush=True)
    return FAULTY


draw_faulty()


def _p(v):
    return tuple(float(a) for a in v)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body, a fixture (``kinematic``) or a
    body with no gravity. ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False
    no_gravity: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


def _part(name, usd, xy, z, extent, mass, **kw):
    return UsdPart(name, asset=name, xy=tuple(xy), extent=extent, mass=mass, usd_path=str(ASSETS / usd),
                   pos=(float(xy[0]), float(xy[1]), float(z)), **kw)


LAMP_ON_Z, LAMP_OFF_Z = TOP + HOUSING_TOP + 0.0001, TOP + HOUSING_TOP - 0.020  # on the cap / hidden inside the housing


def _lamp_xy(side):
    return (CUR_X, CURTAIN_Y[side] + POST_Y)


OBJECTS = []
for _s in SIDES:
    OBJECTS += [
        _part(f"curtain_{_s}", "curtain.usda", (CUR_X, CURTAIN_Y[_s]), TOP, 0.44, 6.0, kinematic=True),
        _part(f"holder_{_s}", "tag_holder.usda", HOLDER_POS[_s], TOP, 0.1, 0.5, kinematic=True),
        _part(f"saddles_{_s}", "rod_saddles.usda", ROD_XY[_s], TOP, 0.28, 0.4, kinematic=True),
        _part(f"rod_{_s}", "test_rod.usda", ROD_XY[_s], TOP + ROD_AXIS_REST + 0.0003, ROD_L, 0.15),
    ]
OBJECTS.append(_part("rack", "tag_holder.usda", RACK_POS, TOP, 0.1, 0.5, kinematic=True))
OBJECTS.append(_part("tag", "tag.usda", RACK_POS, TOP + FLOOR + 0.0003, TAG_W, 0.046))
for _s in SIDES:
    OBJECTS.append(_part(f"lamp_green_{_s}", "lamp_green.usda", _lamp_xy(_s), LAMP_ON_Z, 0.028, 0.001, no_gravity=True))
    OBJECTS.append(_part(f"lamp_red_{_s}", "lamp_red.usda", _lamp_xy(_s), LAMP_OFF_Z, 0.028, 0.001, no_gravity=True))


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {}
    if spec.no_gravity:
        addon["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(disable_gravity=True)
    return scene.Object(
        name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw), spawn_cfg_addon=addon)


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


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


# -- the trip lamp (a declared code effect) ----------------------------------------------------------------------------
_YS = np.linspace(-FIELD_HALF, FIELD_HALF, 81)
_BEAM_Z = TOP + FIRST + PITCH * np.arange(BEAMS)


def blocked_beams(objects, side: str) -> list[int]:
    """The beams of ``side``'s curtain a rod covers: a rod covers a beam when, at some point along it, the beam's whole
    10 mm lens lies inside the rod's cross-section (the beam's centre within 15 - 5 = 10 mm of the rod's axis), within
    the rod's length. Either rod can cover either curtain's beams."""
    cy = CURTAIN_Y[side]
    pts = np.array([[CUR_X, cy + y, z] for z in _BEAM_Z for y in _YS])
    out = set()
    for name in ("rod_left", "rod_right"):
        st = objects[name]
        a = _rot(st)[:, 0]
        d = pts - np.asarray(st.pos, float)
        t = d @ a
        r = np.linalg.norm(d - t[:, None] * a, axis=1)
        hit = (np.abs(t) <= ROD_L / 2) & (r <= ROD_R - LENS_R)
        out |= {int(k) for k in np.nonzero(hit.reshape(BEAMS, -1).any(axis=1))[0]}
    return sorted(out)


def dead_beam(side: str):
    return DEAD_BEAM if side == FAULTY else None


def lamp_red(objects, side: str) -> bool:
    """The receiver's lamp is red: a working beam is covered."""
    return any(k != dead_beam(side) for k in blocked_beams(objects, side))


def lamp_pose(side: str, color: str, red: bool):
    shown = red if color == "red" else not red
    x, y = _lamp_xy(side)
    return np.array([x, y, LAMP_ON_Z if shown else LAMP_OFF_Z])


def _show_lamps(base, origin, objects) -> None:
    """Puts each receiver's red or green lamp on its cap and the other inside the housing."""
    import torch

    for side in SIDES:
        red = lamp_red(objects, side)
        for color in ("green", "red"):
            name = f"lamp_{color}_{side}"
            want = lamp_pose(side, color, red)
            if np.linalg.norm(objects[name].pos - want) < 1e-5:
                continue
            asset = base.scene[name]
            pose = asset.data.root_link_pose_w.torch.clone()
            pose[:, :3] = torch.as_tensor(want + origin, dtype=pose.dtype, device=pose.device)
            asset.write_root_pose_to_sim_index(root_pose=pose)
            asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(asset.data.default_root_vel.torch))


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root pose, the grippers and the episode time; also swaps the receivers' lamps."""
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
        if out["episode_time"] < _PROGRESS.time:
            _PROGRESS.restart()
        _show_lamps(base, origin, out)
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


# -- scoring ----------------------------------------------------------------------------------------------------------
PASS_TILT = math.radians(15.0)  # a pass: the rod's axis within this of the field's normal
PASS_BEYOND = 0.04  # ... its tip this far beyond the plane
PASS_Y = 0.17  # ... crossing the plane within this of the field's middle
THIRDS = ("bottom", "middle", "top")
# The middle pass only counts where the rod covers the dead beam's lens and no working beam's: the rod's axis within
# 10 mm of the dead channel's height (107 to 127 mm; the beams next to it, 20 mm either side, are covered from 10 mm),
# taken 1 mm inside so that a pass at the edge cannot cover a neighbour. Anywhere else in the middle third a working
# neighbour trips the lamp and hides the fault.
EXPOSE_LO = FIRST + PITCH * DEAD_BEAM - (ROD_R - LENS_R) + 0.001
EXPOSE_HI = FIRST + PITCH * DEAD_BEAM + (ROD_R - LENS_R) - 0.001
SEAT_X, SEAT_Y, SEAT_DROP, SEAT_RISE = 0.012, 0.004, 0.002, 0.004  # the tag's lower edge's middle off the slot's centre
SEAT_TILT = math.radians(6.0)
CLEAR = 0.03  # m: every gripper at least this far from the tag and the rods
FIELD_X = 0.045  # a rod within this of the plane (and between the posts, below the housing top) is in the field
FALLEN_Z = -0.03
GRIP_PAD_POINTS = 11
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
STEPS = ("left curtain tested at the bottom, at the dead beam's height and at the top",
         "right curtain tested at the bottom, at the dead beam's height and at the top",
         "tag in the holder of the machine that missed the rod (after its test), none in the other",
         "let go: both rods out of the fields, grippers clear")
POINTS = 0.2


def pass_height(objects, side: str, rod: str):
    """The height of ``rod``'s axis above the table where it crosses ``side``'s field plane while it is pushed through
    (axis within 15 degrees of the plane's normal, tip 4 cm beyond it, rod's middle still in front of it, crossing
    between the posts, inside the field's height), or None."""
    st = objects[rod]
    pos = np.asarray(st.pos, float)
    a = _rot(st)[:, 0]
    if a[0] < 0:
        a = -a
    if math.acos(max(-1.0, min(1.0, float(a[0])))) > PASS_TILT:
        return None
    tip = pos + a * ROD_L / 2
    if tip[0] < CUR_X + PASS_BEYOND or pos[0] > CUR_X - 0.03:
        return None
    s = (CUR_X - pos[0]) / a[0]
    at = pos + s * a
    if abs(at[1] - CURTAIN_Y[side]) > PASS_Y:
        return None
    h = float(at[2] - TOP)
    return h if FIELD_LO <= h <= FIELD_HI else None


def rod_pass(objects, side: str, rod: str):
    """Which pass (0 bottom third, 1 middle, 2 top third) ``rod`` is making through ``side``'s field right now, or
    None. The middle pass is only the one with the axis in the band where the rod covers the dead beam alone."""
    h = pass_height(objects, side, rod)
    if h is None:
        return None
    third = min(2, int((h - FIELD_LO) / ((FIELD_HI - FIELD_LO) / 3)))
    if third == 1 and not EXPOSE_LO <= h <= EXPOSE_HI:
        return None
    return third


def tag_in(objects, holder_xy) -> bool:
    """The tag stands in the slot of the holder at ``holder_xy``: its lower edge's middle in the slot, upright."""
    st = objects["tag"]
    d = np.asarray(st.pos, float) - np.array([holder_xy[0], holder_xy[1], TOP])
    r = _rot(st)
    tilt = math.acos(max(-1.0, min(1.0, float(r[2, 2]))))
    return bool(abs(d[0]) <= SEAT_X and abs(d[1]) <= SEAT_Y and FLOOR - SEAT_DROP <= d[2] <= FLOOR + SEAT_RISE
                and tilt <= SEAT_TILT)


def rod_in_field(objects, side: str) -> bool:
    cy = CURTAIN_Y[side]
    for name in ("rod_left", "rod_right"):
        st = objects[name]
        a = _rot(st)[:, 0]
        for f in np.linspace(-0.5, 0.5, 41):
            p = np.asarray(st.pos, float) + f * ROD_L * a
            if abs(p[0] - CUR_X) <= FIELD_X and abs(p[1] - cy) <= FIELD_HALF + 0.02 and p[2] - TOP <= HOUSING_TOP:
                return True
    return False


def _pads(g) -> list:
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, GRIP_PAD_POINTS)]


def let_go(objects) -> bool:
    """No gripper pad is within CLEAR of the tag or of either rod."""
    tag = objects["tag"]
    r = _rot(tag)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - np.asarray(tag.pos, float))
            if (abs(q[0]) <= TAG_W / 2 + CLEAR and abs(q[1]) <= TAG_T / 2 + CLEAR
                    and -CLEAR <= q[2] <= TAG_H + CLEAR):
                return False
            for name in ("rod_left", "rod_right"):
                st = objects[name]
                a = _rot(st)[:, 0]
                d = p - np.asarray(st.pos, float)
                t = float(np.clip(d @ a, -ROD_L / 2, ROD_L / 2))
                if np.linalg.norm(d - t * a) <= ROD_R + CLEAR:
                    return False
    return True


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float))
                 for n in ("rod_left", "rod_right", "tag"))


def still(objects, anchor) -> bool:
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: which thirds of each field a rod has been
    pushed through, when each curtain's test was complete, since when the tag has stood in each holder, and since when
    every full-success condition has held without a break (``met_since``) with the poses then (``anchor``). Starts
    over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.passed = {s: set() for s in SIDES}
        self.tested_at = {s: None for s in SIDES}
        self.seated_since = {s: None for s in SIDES}
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def restart(self) -> None:
        """A new episode: forget everything and draw the faulty machine again."""
        self.__init__()
        draw_faulty()

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.restart()
            self.time = t
        now = t if t is not None else 0.0
        for s in SIDES:
            for rod in ("rod_left", "rod_right"):
                third = rod_pass(objects, s, rod)
                if third is not None and third not in self.passed[s]:
                    self.passed[s].add(third)
                    print(f"[{TAG}] t={now:.2f}s  {s} curtain: {THIRDS[third]} pass counted, rod axis "
                          f"{pass_height(objects, s, rod) * 1000:.1f} mm", flush=True)
            if len(self.passed[s]) == 3 and self.tested_at[s] is None:
                self.tested_at[s] = now
            if tag_in(objects, HOLDER_POS[s]):
                if self.seated_since[s] is None:
                    self.seated_since[s] = now
            else:
                self.seated_since[s] = None
        if t is None:
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
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def other(side: str) -> str:
    return "left" if side == "right" else "right"


def tag_decision(objects) -> bool:
    """The tag stands in the holder of the machine whose curtain missed the rod, not before that curtain's test was
    complete, and not in the other holder."""
    p = _PROGRESS
    bad, good = FAULTY, other(FAULTY)
    return bool(p.seated_since[bad] is not None and p.tested_at[bad] is not None
                and p.seated_since[bad] >= p.tested_at[bad] and p.seated_since[good] is None)


def _steps(objects) -> list[bool]:
    p = _PROGRESS
    one, two = (len(p.passed[s]) == 3 for s in SIDES)
    three = bool(tag_decision(objects))
    four = bool(one and two and three and not rod_in_field(objects, "left") and not rod_in_field(objects, "right")
                and let_go(objects))
    return [one, two, three, four]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: 1 and 2 once a rod has been pushed through that field at the bottom third, at the top third
    and in the middle with its axis within 10 mm of the dead beam's height (a rod's axis within 15 degrees of the
    field's normal, its tip 4 cm beyond the plane, crossing between the posts); 3 as things stand
    (the tag in the faulty machine's holder, put there after that curtain's test, and none in the other); 4 with
    steps 1 to 3, both rods out of the fields and every gripper at least 3 cm from the tag and the rods."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        red = {s: lamp_red(objects, s) for s in SIDES}
        line = (f"(passes left {sorted(p.passed['left'])} right {sorted(p.passed['right'])}; lamps "
                f"left {'red' if red['left'] else 'green'} right {'red' if red['right'] else 'green'}; tag in "
                f"{[s for s in SIDES if tag_in(objects, HOLDER_POS[s])] or 'neither holder'})")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: a rod or the tag off the table, or a state that is not finite."""
    for name in ("rod_left", "rod_right", "tag"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S: all four steps, no violation, and the rods
    and the tag within 2 mm and 2 degrees of where they were when that second began."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success; otherwise 0.2 per step done (``STEPS``, at most 0.8, also while the second is being timed); 0 for a
    violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Each arm, straight down with its fingers closing across the rod (along y), takes its rod off the saddles by its middle
# (the rod stays level: its centre of mass is in the grip), carries it to its machine's field and pushes it through, tip
# first, at the bottom, middle and top of the field (axis 77, 117 and 165 mm above the table), pausing in the field and
# drawing it back each time; puts the rod back on its saddles and lets go. Then the arm on the machine that missed the
# rod takes the tag from the rack by its upper part and stands it in that machine's holder.
SQUEEZE = 0.010
ROD_OPEN = -(2 * ROD_R + 0.030) / 2
ROD_HOLD = -(2 * ROD_R - SQUEEZE) / 2
TAG_OPEN = -(TAG_T + 0.024) / 2
TAG_HOLD = -(TAG_T - 0.004) / 2
TRAVEL = 0.15  # the rod's axis height when carried
GRIP_ABOVE_AXIS = 0.012  # the TCP this far above the rod's axis (the pad tips end 1 mm below it)
HEIGHTS = (0.077, 0.117, 0.165)  # the rod's axis above the table for the bottom, middle and top passes
AIM = {"left": np.zeros(3), "right": np.zeros(3)}  # measured droop offsets added to the commands (m)


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
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target

DOWN = yam_demo.grip_rotation(0.0)  # pointing down, the fingers closing along y


def _rod_tcp(x, y, axis_h, side):
    return np.array([x, y, TOP + axis_h + GRIP_ABOVE_AXIS]) + AIM[side]


def rod_plan(side: str, t_wait: float = 0.0) -> list[Waypoint]:
    """Take the rod off its saddles, push it through the field at the bottom, middle and top, put it back."""
    cy = CURTAIN_Y[side]
    at = _rod_tcp(LANE_X0, cy, ROD_AXIS_REST, side)
    wps = []
    if t_wait > 0:
        wps.append(Waypoint(t_wait, _START_TCP[side], FINGER_OPEN, "wait", rot=_START_ROT[side]))
    wps += [
        Waypoint(2.5, at + (0, 0, 0.08), ROD_OPEN, "above the rod", rot=DOWN),
        Waypoint(1.2, at, ROD_OPEN, "around the rod", rot=DOWN),
        Waypoint(0.8, at, ROD_HOLD, "grip the rod", rot=DOWN),
    ]
    for h, name in zip(HEIGHTS, THIRDS, strict=True):
        wps += [
            Waypoint(1.2, _rod_tcp(LANE_X0, cy, h, side), ROD_HOLD, f"to the {name} of the field", rot=DOWN),
            Waypoint(2.0, _rod_tcp(LANE_X1, cy, h, side), ROD_HOLD, f"push it through at the {name}", rot=DOWN),
            Waypoint(0.8, _rod_tcp(LANE_X1, cy, h, side), ROD_HOLD, "hold it in the field", rot=DOWN),
            Waypoint(1.5, _rod_tcp(LANE_X0, cy, h, side), ROD_HOLD, "draw it back", rot=DOWN),
        ]
    wps += [
        Waypoint(1.2, at + (0, 0, 0.004), ROD_HOLD, "lower it onto the saddles", rot=DOWN),
        Waypoint(0.6, at + (0, 0, 0.004), ROD_OPEN, "let go of the rod", rot=DOWN),
        Waypoint(1.0, at + (0, 0, 0.08), ROD_OPEN, "lift away", rot=DOWN),
    ]
    return wps


TAG_GRIP_ABOVE_EDGE = 0.075  # the pad tips reach down to this far above the tag's lower edge (21 mm above the slot's rim)
AIM_TAG = np.zeros(3)


def _tag_tcp(xy, z_bottom):
    """The TCP holding the tag (its lower edge's middle at height ``z_bottom`` above the table) by its upper part."""
    return np.array([xy[0], xy[1], TOP + z_bottom + TAG_GRIP_ABOVE_EDGE + PAD_TIP_PAST_TCP]) + AIM_TAG


def tag_plan(side: str) -> list[Waypoint]:
    """Take the tag from the rack by its upper part, carry it over ``side``'s holder, stand it in the slot, let go."""
    hx, hy = HOLDER_POS[side]
    start = _tag_tcp(RACK_POS, FLOOR)
    seat = _tag_tcp((hx, hy), FLOOR + 0.0005)
    return [
        Waypoint(2.0, start + (0, 0, 0.07), TAG_OPEN, "above the tag", rot=DOWN),
        Waypoint(0.8, start + (0, 0, 0.07), TAG_OPEN, "steady above the tag", rot=DOWN),
        Waypoint(2.0, start, TAG_OPEN, "round the tag", rot=DOWN),
        Waypoint(0.6, start, TAG_OPEN, "steady", rot=DOWN),
        Waypoint(0.8, start, TAG_HOLD, "grip the tag", rot=DOWN),
        Waypoint(1.0, start + (0, 0, 0.04), TAG_HOLD, "lift it out of the rack", rot=DOWN),
        Waypoint(2.5, seat + (0, 0, 0.04), TAG_HOLD, "carry it over the holder", rot=DOWN),
        Waypoint(0.8, seat + (0, 0, 0.04), TAG_HOLD, "steady over the slot", rot=DOWN),
        Waypoint(2.0, seat, TAG_HOLD, "stand it in the holder", rot=DOWN),
        Waypoint(0.5, seat, TAG_HOLD, "hold", rot=DOWN),
        Waypoint(0.6, seat, TAG_OPEN, "let go of the tag", rot=DOWN),
        Waypoint(1.2, seat + (0, 0, 0.08), TAG_OPEN, "lift away", rot=DOWN),
    ]


def demo():
    """Both arms test their curtain with their rod at the same time; the arm of the machine that missed the rod then
    tags it. A fixed open-loop script (it reads nothing from the simulator); the other arm goes home."""
    plans = {s: rod_plan(s) for s in SIDES}
    for s in SIDES:
        if s == FAULTY:
            plans[s] += tag_plan(s) + [Waypoint(2.5, _START_TCP[s], FINGER_OPEN, "home", rot=_START_ROT[s])]
        else:
            plans[s] += [Waypoint(2.5, _START_TCP[s], FINGER_OPEN, "home", rot=_START_ROT[s])]
    return plans


# Where each arm's TCP starts (its ready pose), to come back to.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
