"""On the station table: a mini 18650 spot welder (124 x 24 x 22 mm housing, black ABS) lying on its feet, its
lithium-ion cell inside under a latched lid, and an open cardboard box (20 x 17 cm, 10 cm tall) fixed in front of the
robot. The left arm holds the welder down on the table; the right arm pushes the latch's knob so the lid springs open,
and goes back; the left arm lifts the welder, turns it over above the box so the cell falls in, sets it back on the
table and goes back.

The welder is one articulation from ``assets/`` (its housing, the hinged lid with its opening spring, the latch slider
that holds it shut by contact, the sprung + contact and the USB-C fingers); the cell is its own body in the bay. The
box is NVIDIA's SimReady open cardboard box (CC-BY 4.0) at a third of its size, downloaded once from Hugging Face.

Run it:  inspect-robots run --embodiment yam_arena -E task=welder_battery.py --scorer task_score ...

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
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the welder (its own frame: origin at the rear face's bottom centre, +x to the probes, z up) ----------------------
WELDER_WIDTH, WELDER_HEIGHT = 0.024, 0.022
FEET = 0.0006  # the rubber feet reach this far below the housing's bottom
BODY_SHIFT = np.array([0.004, 0.0, 0.0])  # the housing body's own origin (its articulation root) is 4 mm in
LID_TOP = 0.022  # the lid's top face; it spans x 7.2-80.2 mm and is hinged at x 7.95 mm, z 21.25 mm
KNOB_REAR, KNOB_TOP = 0.0818, 0.0234  # the latch knob's rear face (x) and top; it stands 1.4 mm above the lid
CELL_IN_WELDER = np.array([0.0421, 0.0, 0.011])  # the 18650 cell in the bay, under the closed lid
CELL_IN_WELDER_XYZW = (-0.5, 0.5, -0.5, 0.5)
CENTRE_IN_WELDER = np.array([0.062, 0.0, 0.011])  # the housing's middle: the welder is turned about its long axis here
WELDER_GRIP_AT = 0.105  # along the welder from its rear face: its front end, clear of the lid's swing

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
WELDER_XY, WELDER_YAW = (0.27, -0.02), math.pi / 2  # rear face at y = -0.02, probes toward +y
BOX_XY = (0.50, -0.02)  # the box's centre, front middle

# -- the box: SimReady open cardboard box A06 (61 x 50.5 x 30.2 cm) at a third of its size, fixed ----------------------
BOX_SCALE = 1 / 3
BOX_HALF = (0.611 / 2 * BOX_SCALE, 0.505 / 2 * BOX_SCALE)  # half length (x) and half width (y): 10.2, 8.4 cm
BOX_HEIGHT = 0.302 * BOX_SCALE  # its tall walls: 10.1 cm
SIMREADY_REPO = "nvidia/simready-assets"  # Hugging Face dataset
SIMREADY_REVISION = "2464ca32165bfff81ba34bc703ddb932822cfd27"
BOX_PACKAGE, BOX_USD = "box_cardboard_open_a06", "sm_box_cardboard_open_a06_01.usd"


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _qmul(a, b) -> tuple[float, float, float, float]:
    """Quaternion product a * b (xyzw)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


WELDER_POS = (*WELDER_XY, TOP + FEET)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks. ``usd_path`` is a local file, or empty for the SimReady box
    (fetched at spawn)."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    scale: float = 1.0
    kinematic: bool = False  # a body that never moves (the box)

    @property
    def free(self) -> bool:
        return not self.kinematic


# Arena places an articulation by its root link: the welder's is its housing body, 4 mm in from the rear face.
WELDER = UsdPart("welder", asset="welder", xy=WELDER_XY, extent=0.14, mass=0.1267,
                 usd_path=str(ASSETS / "welder_task.usda"),
                 pos=tuple(float(v) for v in np.array(WELDER_POS) + _rz(WELDER_YAW) @ BODY_SHIFT),
                 rot_xyzw=_yaw_xyzw(WELDER_YAW), articulation=True)
CELL = UsdPart("cell", asset="cell", xy=WELDER_XY, extent=0.065, mass=0.047,
               usd_path=str(ASSETS / "battery_18650.usda"),
               pos=tuple(float(v) for v in np.array(WELDER_POS) + _rz(WELDER_YAW) @ CELL_IN_WELDER),
               rot_xyzw=tuple(float(v) for v in _qmul(_yaw_xyzw(WELDER_YAW), CELL_IN_WELDER_XYZW)))
BOX = UsdPart("box", asset="box", xy=BOX_XY, extent=2 * BOX_HALF[0],
              pos=(*BOX_XY, TOP + 0.001), scale=BOX_SCALE, kinematic=True)
OBJECTS = [WELDER, CELL, BOX]


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


SOLVER_ITERATIONS = 32  # position iterations per step for the welder's articulation (light links: lid, latch, fingers)
MAX_DEPENETRATION = 0.2  # m/s: tight contacts with 20 N grippers stay stable


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD; the box kinematic (fixed)."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path or str(fetch_box()), object_type=scene.ObjectType.RIGID,
            scale=(spec.scale,) * 3, initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )


@dataclass(frozen=True)
class PartPose:
    """One link of an articulation after a control step (world frame, m)."""

    pos: np.ndarray
    quat_xyzw: np.ndarray


def _link_pose(base, origin, asset: str, link: str) -> PartPose:
    art = base.scene[asset]
    i = art.find_bodies([link])[0][0]
    pose = art.data.body_link_pose_w.torch[0, i].detach().cpu().numpy().astype(np.float64)
    return PartPose(pos=pose[:3] - origin, quat_xyzw=pose[3:])


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
        """Each object's root (Arena's geometry queries take rigid objects only; an articulation's bounds are left at
        its root), plus the grippers, the episode time, the welder's housing and its lid and latch."""
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
        out["housing"] = _link_pose(base, origin, "welder", "Body")
        welder = base.scene["welder"]
        ids = welder.find_joints(["LidHinge", "LatchSlide"], preserve_order=True)[0]
        lid, latch = welder.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
        out["lid_angle"], out["latch_travel"] = float(lid), float(latch)  # rad (negative opens), m
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that closes the lid and the latch and winds their
        springs: Isaac Lab sets every joint drive's target to 0, which would leave the lid's spring holding it shut."""
        configure = stock_cfg_callback(control_hz)

        def with_springs(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.welder_springs = EventTermCfg(func=_wind_springs, mode="reset",
                                                         params={"asset_cfg": SceneEntityCfg("welder")})
            return env_cfg

        return with_springs

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


# The springs' targets, as in the welder's USD: the lid's torsion spring winds toward fully open, the latch's return
# spring presses the slider back with a small preload.
SPRING_TARGETS = {"LidHinge": math.radians(-105), "LatchSlide": -0.001}


def _wind_springs(env, env_ids, asset_cfg) -> None:
    """Reset event: the lid shut and the latch home (the simulator keeps an articulation's joints where they were),
    and the springs' drive targets as the welder's USD has them."""
    import torch

    if env_ids is None:
        return
    welder = env.scene[asset_cfg.name]
    ids = welder.find_joints(list(SPRING_TARGETS), preserve_order=True)[0]
    zero = torch.zeros((len(env_ids), len(ids)), device=welder.device)
    welder.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    welder.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
    target = torch.tensor([list(SPRING_TARGETS.values())], device=welder.device).repeat(len(env_ids), 1)
    welder.set_joint_position_target_index(target=target, joint_ids=ids, env_ids=env_ids)


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
LID_OPEN = math.radians(-60)  # the lid's hinge past this counts as open (its spring takes it to -105 deg)
LATCH_PUSHED = 0.001  # m: the latch slider pushed at least this far (it lets go of the lid at 1.3-1.8 mm) ...
KNOB_REACH = 0.015  # ... while a gripper's pads were within this of the knob's centre: the latch was pushed by hand
KNOB_CENTRE = np.array([0.084, 0.0, 0.0227])  # the latch knob's centre in the welder's frame (closed)
INSIDE_MARGIN = 0.01  # m: the cell's centre at least this far inside the box's outer walls, and
IN_BOX_Z = 0.03  # ... lower than this above the table: it lies on the box floor (the cell rests at 12 mm)
ON_FEET = 0.003  # m: the welder's bottom within this of resting on its feet on the table, and
UPRIGHT = math.radians(10)  # ... upright within this
LIFTED = 0.02  # m: the welder's lowest point at least this far above the table, held, counts as lifted
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
# The welder's housing box in its own (origin) frame, m: from the rear face to the probes' tips.
WELDER_BOX = (np.array([0.0, -WELDER_WIDTH / 2, 0.0]), np.array([0.1387, WELDER_WIDTH / 2, WELDER_HEIGHT]))
STEPS = ("lid opened", "cell in the box", "welder put back")
POINTS = 1 / 3  # per step


def welder_origin(objects) -> np.ndarray:
    h = objects["housing"]
    return h.pos - quat_xyzw_to_matrix(h.quat_xyzw) @ BODY_SHIFT


def _welder_pose(objects) -> PartPose:
    return PartPose(pos=welder_origin(objects), quat_xyzw=objects["housing"].quat_xyzw)


def _over_box(pos, margin: float = 0.0) -> bool:
    return bool(abs(pos[0] - BOX_XY[0]) < BOX_HALF[0] - margin and abs(pos[1] - BOX_XY[1]) < BOX_HALF[1] - margin)


def cell_in_box(objects) -> bool:
    """The cell's centre lies inside the box's walls (by INSIDE_MARGIN) and low in it (below IN_BOX_Z)."""
    cell = objects["cell"].pos
    return bool(_over_box(cell, INSIDE_MARGIN) and cell[2] < TOP + IN_BOX_Z)


def welder_on_table(objects) -> bool:
    """The welder stands on its feet on the table (not in the box): its bottom within ON_FEET of there and upright
    within UPRIGHT."""
    origin = welder_origin(objects)
    up = float(quat_xyzw_to_matrix(objects["housing"].quat_xyzw)[2, 2])
    return bool(abs(origin[2] - (TOP + FEET)) < ON_FEET and up > math.cos(UPRIGHT) and not _over_box(origin))


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose``."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ quat_xyzw_to_matrix(pose.quat_xyzw).T + pose.pos
    return w.min(0), w.max(0)


def _held(bounds, gripper) -> bool:
    """``gripper`` holds the part with world ``bounds``: its pads reach them and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = bounds[0] - GRIP_SLACK, bounds[1] + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def _held_by_any(objects, bounds) -> bool:
    return any(_held(bounds, objects.get(g)) for g in ("left_gripper", "right_gripper"))


def _near_knob(objects) -> bool:
    """A gripper's pads (from their back to their tips) pass within KNOB_REACH of the latch knob's centre."""
    knob = welder_origin(objects) + quat_xyzw_to_matrix(objects["housing"].quat_xyzw) @ KNOB_CENTRE
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        seg = g.tips - g.back
        f = float(np.clip((knob - g.back) @ seg / (seg @ seg), 0.0, 1.0))
        if np.linalg.norm(g.back + f * seg - knob) < KNOB_REACH:
            return True
    return False


def welder_put_back(objects) -> bool:
    """The welder stands on the table and no gripper holds it."""
    return bool(welder_on_table(objects) and not _held_by_any(objects, _box(_welder_pose(objects), WELDER_BOX)))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether a gripper pushed the latch and whether the welder has been lifted.
    Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.latch_pushed = self.welder_lifted = False
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.latch_pushed |= bool(objects["latch_travel"] > LATCH_PUSHED and _near_knob(objects))
        bounds = _box(_welder_pose(objects), WELDER_BOX)
        self.welder_lifted |= bool(bounds[0][2] > TOP + LIFTED and _held_by_any(objects, bounds))


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand: the lid open (once a gripper has pushed the latch), the cell in the box, the
    welder standing on the table not held (once it has been lifted: at the start it stands there untouched)."""
    p = _PROGRESS
    p.update(objects)
    done = [bool(p.latch_pushed and objects["lid_angle"] < LID_OPEN), cell_in_box(objects),
            bool(p.welder_lifted and welder_put_back(objects))]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        print(f"[welder_battery] t={objects['episode_time']:.2f}s  steps done: {got}  (lid "
              f"{math.degrees(objects['lid_angle']):.0f} deg, cell {np.round(objects['cell'].pos * 1000, 0)} mm, welder "
              f"{np.round(welder_origin(objects) * 1000, 0)} mm)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the welder or its cell off the table, or a state that is not finite."""
    for pos in (objects["housing"].pos, objects["cell"].pos):
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def success(objects) -> bool:
    """All three steps done, both arms back where they started with nothing in hand, the welder and the cell still, and
    no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["welder"].resting() and objects["cell"].resting())


def score(objects) -> float:
    """A third per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from above, across the welder's front end; hold it down on the table while the right arm pushes the latch;
# lift it, carry it over the box, turn it about its long axis until the open bay faces down so the cell falls in, turn
# it back, carry it high over the box's wall, set it down where it started, let go and go home.
# Right arm: fingers closed and upright, lowered onto the lid just behind the latch knob; slide them toward the probes so
# the latch lets the lid go, lift away as the lid springs open, and go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width
WELDER_HOLD = -(WELDER_WIDTH - 1.5 * SQUEEZE) / 2  # ~30 N a finger: the welder hangs from its front end
WELDER_TIP_PAST_MID = 0.003  # the left pad tips this far past the housing's mid-height
# The left gripper leans toward +y (the probes) about its finger-closing axis, its wrist away from the right one's.
LEAN = math.radians(50)
LIFT = 0.08  # m: how high the left arm lifts the welder off the table, once the lid is open
CARRY_HIGH = 0.17  # m: the welder's bottom on the way back from the box, clear of its 10 cm walls
# The right fingertips come down upright PUSH_GAP behind the knob and slide forward so their front face pushes the
# knob's 1.4 mm rear face PUSH_TRAVEL toward the probes (the latch lets go at 1.3-1.8 mm of its 3 mm travel; the arm
# lags its target by a millimetre or two). The pads are 15.5 mm wide along the welder, but rounded at their ends: in
# simulation the knob met them 5.7 mm ahead of their middle, and with the pads resting on the lid their rounded edge rode
# up over the knob after 1 mm. So they are pressed TIP_INTO_LID into the lid (the arm gives). (Leaned, their sloped end
# face rode up over the knob too.)
TIP_HALF = 0.0057
TIP_INTO_LID = 0.0015
PUSH_GAP = 0.003
PUSH_TRAVEL = 0.006
# Pouring: the welder carried over the box turned end for end (probes toward the robot) and about its long axis as far
# as the left wrist goes (scanned offline: 110 deg), so the open bay faces 20 deg below level and the cell rolls out of
# it. It is carried straight into that pose: from level over the box the arm's posture can't follow the turn (it can
# turn back to level from it).
POUR_CENTRE = np.array([0.47, -0.02, TOP + 0.20])  # the welder's middle over the box
POUR_YAW = math.pi
POUR_ROLL = math.radians(110)


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

_X = np.array([1.0, 0.0, 0.0])
LEFT_GRIP = axis_angle_matrix(_X, -LEAN) @ yam_demo.grip_rotation(WELDER_YAW - math.pi)  # fingers across the housing
WELDER_AXIS = _rz(WELDER_YAW) @ _X  # +y: the welder's long axis, rear to probes
RIGHT_PUSH = yam_demo.grip_rotation(math.pi / 2)  # upright, fingers closing across the welder, their width along it


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ np.array([0.0, 0.0, 1.0]))


def _welder_turn(yaw: float, roll: float) -> np.ndarray:
    """The welder's orientation: heading ``yaw``, then turned ``roll`` about its own long axis."""
    return axis_angle_matrix(_rz(yaw) @ _X, roll) @ _rz(yaw)


def _welder_grip(origin, yaw: float = WELDER_YAW, roll: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP and rotation gripping the welder whose origin is at ``origin``, turned as ``_welder_turn``."""
    turn = _welder_turn(yaw, roll)
    grip = np.asarray(origin, float) + turn @ np.array([WELDER_GRIP_AT, 0.0, WELDER_HEIGHT / 2])
    rot = turn @ _rz(-WELDER_YAW) @ LEFT_GRIP
    return _tcp_with_tips(grip, WELDER_TIP_PAST_MID, rot), rot


def _origin_for_centre(centre, yaw: float, roll: float) -> np.ndarray:
    """The welder's origin that puts its middle at ``centre``, turned as ``_welder_turn``."""
    return np.asarray(centre, float) - _welder_turn(yaw, roll) @ CENTRE_IN_WELDER


LEFT_HELD_S = 2.5 + 1.5 + 0.8 + 0.5  # the left arm over, down, closed and settled


def right_plan() -> list[Waypoint]:
    """Once the left hand holds the welder down on the table, slide a fingertip along the lid into the latch knob,
    lift, go home."""
    rot = RIGHT_PUSH
    knob = np.asarray(WELDER_POS) + _rz(WELDER_YAW) @ np.array([KNOB_REAR, 0.0, LID_TOP - TIP_INTO_LID])

    def tcp(front_edge):
        return _tcp_with_tips(front_edge - TIP_HALF * WELDER_AXIS, 0.0, rot)

    behind = tcp(knob - PUSH_GAP * WELDER_AXIS)
    pushed = tcp(knob + PUSH_TRAVEL * WELDER_AXIS)
    return [
        Waypoint(2.5, behind + (0, 0, 0.05), 0.0, "over the latch", rot=rot),
        Waypoint(max(LEFT_HELD_S - 2.5, 1.0), behind + (0, 0, 0.05), 0.0, "wait for the left hand", rot=rot),
        Waypoint(1.5, behind, 0.0, "down onto the lid", rot=rot),
        Waypoint(2.0, pushed, 0.0, "push the knob", rot=rot),
        Waypoint(0.6, pushed + (0, 0, 0.05), 0.0, "lift away", rot=rot),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan(hold_s: float) -> list[Waypoint]:
    """Grip the welder by its front end and hold it down on the table for the latch; then lift it, pour the cell into
    the box, set the welder back down where it started, let go and go home."""
    at, rot = _welder_grip(WELDER_POS)
    held = at + (0, 0, LIFT)
    over, over_rot = _welder_grip(_origin_for_centre(POUR_CENTRE, POUR_YAW, 0.0), POUR_YAW)
    turned, turned_rot = _welder_grip(_origin_for_centre(POUR_CENTRE, POUR_YAW, POUR_ROLL), POUR_YAW, POUR_ROLL)
    down = at + (0, 0, 0.002)
    return [
        Waypoint(2.5, at + (0, 0, 0.06), FINGER_OPEN, "over the welder", rot=rot),
        Waypoint(1.5, at, FINGER_OPEN, "down to the welder", rot=rot),
        Waypoint(0.8, at, WELDER_HOLD, "grip the welder", rot=rot),
        Waypoint(hold_s, at, WELDER_HOLD, "hold it for the latch", rot=rot),
        Waypoint(2.0, held, WELDER_HOLD, "lift it", rot=rot),
        Waypoint(4.0, turned, WELDER_HOLD, "over the box, turned over", rot=turned_rot),
        Waypoint(1.5, turned, WELDER_HOLD, "pour", rot=turned_rot),
        Waypoint(2.0, over, WELDER_HOLD, "turn it back", rot=over_rot),
        Waypoint(3.0, down + (0, 0, CARRY_HIGH), WELDER_HOLD, "back over its place", rot=rot),
        Waypoint(1.5, down + (0, 0, 0.04), WELDER_HOLD, "down near the table", rot=rot),
        Waypoint(1.5, down, WELDER_HOLD, "set it down", rot=rot),
        Waypoint(1.0, down, FINGER_OPEN, "let go", rot=rot),
        Waypoint(0.5, down, FINGER_OPEN, "fingers open", rot=rot),
        Waypoint(1.0, down + (0, 0, 0.06), FINGER_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(3.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


def demo():
    """Both arms: the left holds the welder down on the table while the right opens the latch and goes home; then the
    left lifts it, pours the cell into the box, sets the welder down and goes home."""
    right = right_plan()
    return {"left": left_plan(hold_s=sum(w.duration for w in right) - (LEFT_HELD_S - 0.5) + 0.5), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
