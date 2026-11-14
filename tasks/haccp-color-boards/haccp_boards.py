"""On the station table: three colour-coded HDPE cutting boards in a row (9 x 12 in, 1/2 in thick: yellow on the robot's
left, red in the middle, blue on its right), a stainless gastronorm pan nearer the robot holding three raw cuts (a
boneless chicken thigh, a pork loin chop and a cod fillet, each a rigid shape of true size and mass), and a sanitiser
dip tub at the right.

The cuts are rigid stand-ins (the robot cannot handle soft meat), but they are irregular shapes with rounded edges, not
blocks. The colour code is the US food-service one: red raw meat, yellow raw poultry, blue raw fish and seafood.

The task is to set each cut on its own board. Cross-contamination is a penalty that lasts the whole run, judged at every
control step: a raw cut touching any board but its own (even for an instant), a gripper that has held one kind of raw
food gripping a different one before its pads were sanitised (dipped, fingers open, 2 s, in the tub), or unsanitised pads
that have held one kind of raw food touching another kind's board. Any one scores 0 for the episode.

Run it:  inspect-robots run --embodiment yam_arena -E task=haccp_boards.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts).
"""

import importlib.abc
import importlib.util
import math
import re
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
TAG = "haccp_boards"

# -- the parts (as built in assets/; each frame's origin and axes are given in its USD file's doc string) --------------
BOARD_X, BOARD_Y, BOARD_T = 229 * MM, 305 * MM, 12.7 * MM  # 9 x 12 x 1/2 in; origin at the underside's centre
PAN_X, PAN_Y, PAN_Z, PAN_FLOOR = 265 * MM, 325 * MM, 20 * MM, 1.5 * MM  # gastronorm 1/2 x 20 mm
TUB_X, TUB_Y, TUB_Z, TUB_WALL, TUB_FLOOR = 150 * MM, 200 * MM, 70 * MM, 3 * MM, 3 * MM
LIQUID = 40 * MM  # the sanitiser's surface above the tub's underside

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
BOARD_XY = {"yellow": (0.40, 0.32), "red": (0.40, 0.0), "blue": (0.40, -0.32)}
PAN_XY = (0.145, 0.0)
TUB_XY = (0.16, -0.38)
CUT_XY = {"pork": (0.145, -0.105), "thigh": (0.145, 0.105), "cod": (0.145, 0.0)}
OWN_BOARD = {"pork": "red", "thigh": "yellow", "cod": "blue"}  # US colour code: meat red, poultry yellow, fish blue
CUTS = tuple(CUT_XY)
BOARD_TOP = TOP + BOARD_T
PAN_TOP_FLOOR = TOP + PAN_FLOOR
LIQUID_TOP = TOP + LIQUID

_FLOAT = r"[-+0-9.eE]+"


def _collider(name: str) -> tuple[np.ndarray, np.ndarray]:
    """A cut's collider (its convex hull) from its USD file: the vertices (m, in the cut's frame) and the triangles."""
    text = (ASSETS / f"raw_{name}.usda").read_text()
    block = text[text.index('def Mesh "Collider"'):]
    pts = re.search(r"point3f\[\] points = \[(.*?)\]\n", block, re.S).group(1)
    idx = re.search(r"int\[\] faceVertexIndices = \[(.*?)\]", block, re.S).group(1)
    pts = np.array([[float(v) for v in m] for m in re.findall(rf"\(({_FLOAT}), ({_FLOAT}), ({_FLOAT})\)", pts)])
    return pts, np.array([int(v) for v in idx.split(",")]).reshape(-1, 3)


def _surface(pts: np.ndarray, tris: np.ndarray, n: int = 2500) -> np.ndarray:
    """The hull's vertices plus ``n`` points spread over its surface (fixed seed), for contact tests."""
    a, b, c = pts[tris[:, 0]], pts[tris[:, 1]], pts[tris[:, 2]]
    area = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    rng = np.random.default_rng(1)
    pick = rng.choice(len(tris), size=n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return np.vstack([pts, a[pick] + u[:, None] * (b[pick] - a[pick]) + v[:, None] * (c[pick] - a[pick])])


_HULL = {name: _collider(name) for name in CUTS}
SURFACE = {name: _surface(*_HULL[name]) for name in CUTS}
CUT_BOX = {name: (_HULL[name][0].min(0), _HULL[name][0].max(0)) for name in CUTS}  # each cut's bounds, its own frame
CUT_MASS = {"pork": 0.131, "thigh": 0.114, "cod": 0.193}  # kg (raw meat 1.06 g/cm3 x the hull's volume)
# Where a gripper pinches each cut (its frame, m): at its centre of mass, across its width (y), and that width.
PINCH = {"pork": (0.0021, 0.0686), "thigh": (0.0019, 0.0666), "cod": (0.0049, 0.0580)}


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body or a fixture (``kinematic``).
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


def _fixture(name, usd, xy, mass, extent):
    return UsdPart(name, asset=name, xy=xy, extent=extent, mass=mass, usd_path=str(ASSETS / usd),
                   pos=(float(xy[0]), float(xy[1]), TOP), kinematic=True)


BOARDS = {c: _fixture(f"board_{c}", f"board_{c}.usda", BOARD_XY[c], 0.9, BOARD_Y) for c in BOARD_XY}
PAN = _fixture("prep_pan", "prep_pan.usda", PAN_XY, 0.6, PAN_Y)
TUB = _fixture("sanitiser_tub", "sanitiser_tub.usda", TUB_XY, 0.25, TUB_Y)
REST_GAP = 0.0008  # the cuts spawn this far above the pan's floor and settle onto it
ITEMS = {c: UsdPart(c, asset=c, xy=CUT_XY[c], extent=0.16, mass=CUT_MASS[c], usd_path=str(ASSETS / f"raw_{c}.usda"),
                    pos=(*CUT_XY[c], PAN_TOP_FLOOR + REST_GAP)) for c in CUTS}
OBJECTS = [*BOARDS.values(), PAN, TUB, *ITEMS.values()]
NAMES = tuple(o.name for o in OBJECTS)


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    pose = scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw)
    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID, initial_pose=pose,
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    return scene.Object(name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
                        initial_pose=pose, spawn_cfg_addon={})


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
        """Each object's root pose and velocities, plus the grippers and the episode time."""
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


# -- scoring ----------------------------------------------------------------------------------------------------------
TOUCH = 1.0 * MM  # a point of a cut within this of a board's top face, over that board, touches it
PAD_TOUCH = 1.5 * MM  # ... and a pad tip within this
ON_BOARD = 1.5 * MM  # a cut rests on its board: its lowest point within this of the top face, all of its underside over it
LEVEL = 3.0 * MM  # the points of a cut this close to its lowest point make its underside
TILT = math.radians(10.0)  # a cut lying on its board is tilted less than this from flat
FALLEN_Z = -0.03  # m below the table top: a cut that low has fallen off the table
CLEAR = 0.03  # m: a gripper's pads at least this far from every cut is a clear release
GRIP_SLACK = 0.004  # held: a pad's point within this of the cut's bounds ...
GRIP_FORCE = 0.5  # ... both fingers pressing with at least this (N) ...
GRIP_OPENING = 0.003  # ... and the pads apart by at least this (not closed on nothing)
DIP_OPENING = 0.020  # sanitising: pads at least this far apart (open, so the solution wets both faces) ...
DIP_DEPTH = 0.010  # ... the pad tips at least this far under the solution's surface ...
DIP_S = 2.0  # ... for this long without a break, holding nothing
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
STEPS = ("pork set on the red board", "thigh set on the yellow board", "cod set on the blue board",
         "every gripper clear of the food")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def cut_points(objects, name: str) -> np.ndarray:
    """The cut's surface points in the world frame."""
    s = objects[name]
    return SURFACE[name] @ _rot(s).T + np.asarray(s.pos, float)


def _inside(pts: np.ndarray, colour: str, margin: float = 0.0) -> np.ndarray:
    cx, cy = BOARD_XY[colour]
    return (np.abs(pts[:, 0] - cx) <= BOARD_X / 2 + margin) & (np.abs(pts[:, 1] - cy) <= BOARD_Y / 2 + margin)


def touches_board(objects, name: str, colour: str) -> bool:
    """Any point of the cut is on the ``colour`` board's top face (or below it, against its side)."""
    pts = cut_points(objects, name)
    return bool(np.any(_inside(pts, colour) & (pts[:, 2] <= BOARD_TOP + TOUCH)))


def wrong_board(objects, name: str) -> str | None:
    """The first board other than the cut's own that it touches now, if any."""
    for colour in BOARD_XY:
        if colour != OWN_BOARD[name] and touches_board(objects, name, colour):
            return colour
    return None


def tilt(objects, name: str) -> float:
    """The cut's flat underside from horizontal (rad)."""
    return math.acos(max(-1.0, min(1.0, float(_rot(objects[name])[2, 2]))))


def on_own_board(objects, name: str) -> bool:
    """The cut lies on its own board: flat, resting on the top face, its whole underside over the board."""
    pts = cut_points(objects, name)
    low = pts[:, 2] <= pts[:, 2].min() + LEVEL
    colour = OWN_BOARD[name]
    return bool(pts[:, 2].min() <= BOARD_TOP + ON_BOARD and np.all(_inside(pts[low], colour))
                and tilt(objects, name) <= TILT)


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def holds(objects, side: str, name: str) -> bool:
    """The ``side`` gripper holds the cut: a point along its pads is within GRIP_SLACK of the cut's bounds, both
    fingers press with GRIP_FORCE and the pads are apart by GRIP_OPENING."""
    g = objects.get(f"{side}_gripper")
    if g is None or g.contact < GRIP_FORCE or g.opening < GRIP_OPENING:
        return False
    s = objects[name]
    q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
    lo, hi = CUT_BOX[name]
    return bool(np.any(np.all((q >= lo - GRIP_SLACK) & (q <= hi + GRIP_SLACK), axis=1)))


def pad_touches_board(gripper, colour: str) -> bool:
    """A pad tip (the closing pads reach about ``opening / 2`` to each side of it) is on the ``colour`` board."""
    tip = gripper.tips
    r = gripper.opening / 2 + 0.006
    cx, cy = BOARD_XY[colour]
    dx = max(abs(tip[0] - cx) - BOARD_X / 2, 0.0)
    dy = max(abs(tip[1] - cy) - BOARD_Y / 2, 0.0)
    return bool(math.hypot(dx, dy) <= r and tip[2] <= BOARD_TOP + PAD_TOUCH)


def pads_in_solution(gripper) -> bool:
    """The pad tips are in the tub, under the sanitiser's surface by DIP_DEPTH, and the pads are open."""
    tip = gripper.tips
    return bool(gripper.opening >= DIP_OPENING and abs(tip[0] - TUB_XY[0]) <= TUB_X / 2 - TUB_WALL - 0.005
                and abs(tip[1] - TUB_XY[1]) <= TUB_Y / 2 - TUB_WALL - 0.005
                and TOP + TUB_FLOOR + 0.002 <= tip[2] <= LIQUID_TOP - DIP_DEPTH)


def clear_of_food(objects) -> bool:
    """No gripper's pads are within CLEAR of any cut."""
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None:
            continue
        for name in CUTS:
            s = objects[name]
            q = (_pads(gripper) - np.asarray(s.pos, float)) @ _rot(s)
            lo, hi = CUT_BOX[name]
            if np.any(np.all((q >= lo - CLEAR) & (q <= hi + CLEAR), axis=1)):
                return False
    return True


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in CUTS)


def still(objects, anchor) -> bool:
    """Every cut within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses, not from the
    simulator's instantaneous speeds."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: every cross-contamination event (they
    last for the whole episode), which kind of raw food each gripper last held without being sanitised since, and
    since when every full-success condition has held without a break (``met_since``, ``None`` while one fails) with
    the poses when they began to hold (``anchor``). Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.events: list[str] = []
        self.dirty = {"left": None, "right": None}
        self.dipping = {"left": 0.0, "right": 0.0}
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def contaminate(self, t, what: str) -> None:
        if what not in self.events:
            self.events.append(what)
            print(f"[{TAG}] t={t:.2f}s  CROSS-CONTAMINATION: {what} (scores 0 from here on)", flush=True)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            dt = t - self.time if self.time >= 0 else 0.0
            self.time = t
        else:
            dt = 0.0
        stamp = -1.0 if t is None else t
        for name in CUTS:  # a cut touching a board that is not its own, at any moment
            colour = wrong_board(objects, name)
            if colour:
                self.contaminate(stamp, f"the {name} touched the {colour} board")
            if objects[name].pos[2] < TOP + FALLEN_Z:
                self.contaminate(stamp, f"the {name} fell off the table")
        for side in ("left", "right"):
            g = objects.get(f"{side}_gripper")
            if g is None:
                continue
            held = [n for n in CUTS if holds(objects, side, n)]
            for name in held:  # pads that held one kind of raw food, not sanitised since, gripping another kind
                if self.dirty[side] not in (None, name):
                    self.contaminate(stamp, f"the {side} gripper, unsanitised after the {self.dirty[side]}, "
                                            f"gripped the {name}")
                self.dirty[side] = name
            if self.dirty[side] is not None:  # ... or touching a board that is not that food's
                for colour in BOARD_XY:
                    if colour != OWN_BOARD[self.dirty[side]] and pad_touches_board(g, colour):
                        self.contaminate(stamp, f"the {side} gripper, unsanitised after the {self.dirty[side]}, "
                                                f"touched the {colour} board")
            if not held and pads_in_solution(g):  # sanitising: pads open in the solution, holding nothing, DIP_S
                self.dipping[side] += dt
                if self.dipping[side] >= DIP_S - 1e-6 and self.dirty[side] is not None:
                    print(f"[{TAG}] t={stamp:.2f}s  {side} gripper sanitised (was {self.dirty[side]})", flush=True)
                    self.dirty[side] = None
            else:
                self.dipping[side] = 0.0
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not self.events)
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'food moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand."""
    on = [on_own_board(objects, n) for n in ("pork", "thigh", "cod")]
    return [*on, bool(all(on) and clear_of_food(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand: every cut lying on its own board; the last one only with all three, and
    with every gripper clear of the food."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        line = ("(" + "; ".join(f"{n}: {'on its board' if on_own_board(objects, n) else 'off'}"
                                f"{'*' if wrong_board(objects, n) else ''}" for n in ("pork", "thigh", "cod"))
                + f"; dirty pads: left {p.dirty['left'] or 'clean'}, right {p.dirty['right'] or 'clean'}"
                + f"; contamination events {len(p.events)})")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: any cross-contamination event this episode (kept for the whole run), a cut off the table, or a state
    that is not finite. Reads the episode's progress without updating it."""
    values = [v for n in CUTS for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.events)


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: the three cuts each lying on their own board, every gripper clear, no cross-contamination event so far, and
    the cuts within STILL_MOVE and STILL_TURN of where they were when that second began. The arms' return is not
    scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 after any cross-contamination."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Every grip is straight down with the fingers closing across the cut's width (along y). The right arm carries the pork
# to the red (middle) board, dips its open pads in the sanitiser, then carries the cod to the blue board; the left arm
# carries the thigh to the yellow board.
SQUEEZE = 0.008  # grips close the fingers this far inside the cut's width (about 20 N a finger)
OPEN_EXTRA = 0.016  # the pads open to the cut's width plus this
TIPS_ABOVE = 0.004  # at the pinch the pad tips are this far above what the cut rests on
CARRY_Z = 0.065  # the TCP's height above the table top in transit (cuts clear the pan's rim and the boards)
HOVER = 0.04  # the TCP's height above the pinch height when coming down to a cut or leaving a board
SET_GAP = 0.0015  # a cut is set down this far above its board before the fingers open
RED_NEAR = 0.03  # the pork is set this much nearer the robot than the middle board's centre (the arm's reach)
DIP_Z = 0.020  # the pad tips this far above the tub's underside (under the surface by LIQUID - DIP_Z = 20 mm)


def _tcp_at(xy, support: float, lift: float = 0.0) -> tuple[float, float, float]:
    """The TCP pinching a cut whose underside rests at height ``support``, over ``xy`` (the cut's centre of mass)."""
    return (float(xy[0]), float(xy[1]), support + TIPS_ABOVE + PAD_TIP_PAST_TCP + lift)


def _open(name: str) -> float:
    return -(PINCH[name][1] + OPEN_EXTRA) / 2


def _hold(name: str) -> float:
    return -(PINCH[name][1] - SQUEEZE) / 2


def _carry(name: str, src_xy, dst_xy, dst_support: float, dst_yaw: float = 0.0):
    """Waypoints that pick the cut from ``src_xy`` (its origin, resting on the pan) and set it on a board with its
    origin over ``dst_xy``."""
    cx = PINCH[name][0]
    s = (src_xy[0] + cx, src_xy[1])
    d = (dst_xy[0] + cx * math.cos(dst_yaw), dst_xy[1] + cx * math.sin(dst_yaw))
    op, hd = _open(name), _hold(name)
    zc = TOP + CARRY_Z
    return [
        Waypoint(2.0, (*s, zc), op, f"over the {name}"),
        Waypoint(1.5, _tcp_at(s, PAN_TOP_FLOOR, 0.02), op, f"down beside the {name}"),
        Waypoint(1.0, _tcp_at(s, PAN_TOP_FLOOR), op, f"pads round the {name}"),
        Waypoint(0.8, _tcp_at(s, PAN_TOP_FLOOR), hd, f"grip the {name}"),
        Waypoint(0.4, _tcp_at(s, PAN_TOP_FLOOR), hd, f"hold the {name}"),
        Waypoint(1.2, (*s, zc), hd, f"lift the {name}"),
        Waypoint(2.8, (*d, zc), hd, f"carry the {name} to its board", yaw=dst_yaw),
        Waypoint(1.5, _tcp_at(d, dst_support, SET_GAP), hd, f"lower the {name} onto its board", yaw=dst_yaw),
        Waypoint(0.6, _tcp_at(d, dst_support, SET_GAP), op, f"let go of the {name}", yaw=dst_yaw),
        Waypoint(1.0, _tcp_at(d, dst_support, HOVER), op, "lift away", yaw=dst_yaw),
    ]


def _dip():
    x, y = TUB_XY
    wide = -0.045
    return [
        Waypoint(2.2, (x, y, TOP + CARRY_Z), wide, "over the sanitiser tub"),
        Waypoint(1.6, (x, y, TOP + DIP_Z + PAD_TIP_PAST_TCP), wide, "pads into the sanitiser"),
        Waypoint(DIP_S + 0.8, (x, y, TOP + DIP_Z + PAD_TIP_PAST_TCP), wide, "hold the pads in the sanitiser"),
        Waypoint(1.4, (x, y, TOP + CARRY_Z), wide, "out of the sanitiser"),
    ]


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


def left_plan() -> list[Waypoint]:
    wps = _carry("thigh", CUT_XY["thigh"], BOARD_XY["yellow"], BOARD_TOP)
    wps.append(Waypoint(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]))
    return wps


def right_plan() -> list[Waypoint]:
    wps = _carry("pork", CUT_XY["pork"], (BOARD_XY["red"][0] - RED_NEAR, BOARD_XY["red"][1]), BOARD_TOP)
    wps += _dip()
    wps += _carry("cod", CUT_XY["cod"], BOARD_XY["blue"], BOARD_TOP)
    wps.append(Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]))
    return wps


def demo():
    """The right arm sets the pork on the red board, sanitises its pads, then sets the cod on the blue board; the left
    arm sets the thigh on the yellow board. Both go home."""
    return {"left": left_plan(), "right": right_plan()}
