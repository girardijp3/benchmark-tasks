"""On the station table: a hard plastic screwdriver case (216 x 80 x 47 mm, blue PP) with its lid standing open at
100 degrees, an EVA foam insert whose cutout follows a PH2 screwdriver, and the screwdriver itself lying on the bench
beside it. The right arm picks the screwdriver up by its handle, turns it and lays it in the cutout, handle in the
handle pocket, and goes back; the left arm pushes the lid over, presses it shut until the clasp catches, and goes back.

The case is one articulation from ``assets/`` (its base with the foam insert, the lid on a damped hinge, the clasp
slider that holds the lid shut by contact). The screwdriver is NVIDIA's SimReady Phillips PH2 screwdriver (CC-BY 4.0),
downloaded once from Hugging Face at a pinned revision, with this task's own colliders (``assets/screwdriver_task.usda``,
copied next to the download).

Run it:  inspect-robots run --embodiment yam_arena -E task=screwdriver_case.py --scorer task_score ...

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
import shutil
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
MM = 0.001
TAG = "screwdriver_case"

# -- the screwdriver (SimReady's frame: +x along the axis from the butt to the tip, its axis SD_AXIS_Z above z = 0) -----
SIMREADY_REPO = "nvidia/simready-assets"  # Hugging Face dataset
SIMREADY_REVISION = "7879e0e50423290300ac20ecd6e727325f680b60"
SD_PACKAGE = "handTool_screwdriver_phillips_ph2_a01"
SD_WRAPPER = "screwdriver_task.usda"  # this task's colliders over the SimReady model
SD_AXIS_Z = 16.366 * MM
SD_BUTT_X, SD_TIP_X = -50.42 * MM, 137.53 * MM
SD_HALF_WIDTH = 17.12 * MM  # the colliders' largest half-width (the handle's grip panels)
GRIP_SD_X = -13.0 * MM  # the middle of the handle's straight grip zone (x -24 to -2 mm): where a gripper holds it
GRIP_WIDTH = 34.04 * MM  # across the grip zone's two side faces
SHAFT_SD_X = 120.0 * MM  # a point on the shaft, for checking where the screwdriver lies
# At rest on a table it lies on its handle and the end of its shaft, pitched REST_PITCH tip down (its colliders' hull
# face holding its centre of mass); its origin is then REST_Z above the table.
REST_PITCH = math.radians(6.017)
REST_Z = -0.380 * MM

# -- the case (its frame: origin at the underside centre, +x along it toward the handle pocket, +y toward the hinge) ---
CASE_L, CASE_W, CASE_H = 216 * MM, 80 * MM, 47 * MM
HINGE_YZ = np.array([42.5, 31.0]) * MM  # the lid's hinge axis (along x)
LID_TOP_ABOVE_HINGE = 16.0 * MM  # the shut lid's top face above the hinge axis
LID_OPEN_ANGLE = math.radians(100)  # its open stop; it rests there (its centre of mass behind the hinge)
SEAT_T = np.array([43.555, 0.0, 9.634]) * MM  # the seated screwdriver's origin (tip toward -x, half a turn about z)
SEAT_R = np.diag([-1.0, -1.0, 1.0])
NOTCH_FLOOR = 16.0 * MM  # the finger notch across the handle pocket: its floor above the case's underside
NOTCH_HALF = 33.0 * MM  # ... and how far it reaches across from the cutout's middle

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The case lies across the table in front of the robot, clasp toward it, hinge at the back, the handle pocket on the
# right; the screwdriver lies on the bench on the right, tip pointing away from the robot. (Turned in place to lie
# along the case, its tip swings past the case's right end, clear of the open lid.)
CASE_XY, CASE_YAW = (0.34, 0.05), -math.pi / 2  # case +x points to the robot's right (-y), +y away from it (+x)
SD_GRIP_XY, SD_YAW = (0.30, -0.22), 0.0  # where the handle's grip zone lies; the tip points along SD_YAW


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _ry(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), a)


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _matrix_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    q = ik.matrix_to_quat_xyzw(np.asarray(r, float))
    return tuple(float(v) for v in q)


CASE_POS = np.array([*CASE_XY, TOP])
CASE_R = _rz(CASE_YAW)
GRIP_LOCAL = np.array([GRIP_SD_X, 0.0, SD_AXIS_Z])  # the grip zone's middle, on the axis (screwdriver frame)
SHAFT_LOCAL = np.array([SHAFT_SD_X, 0.0, SD_AXIS_Z])
SD_REST_R = _rz(SD_YAW) @ _ry(REST_PITCH)
_g = SD_REST_R @ GRIP_LOCAL
SD_REST_POS = np.array([SD_GRIP_XY[0] - _g[0], SD_GRIP_XY[1] - _g[1], TOP + REST_Z + 0.0003])
SEAT_POS_W = CASE_POS + CASE_R @ SEAT_T  # the seated screwdriver's origin (world)
SEAT_R_W = CASE_R @ SEAT_R


def case_point(p) -> np.ndarray:
    """A point given in the case's frame (m), in the world, the case where it starts."""
    return CASE_POS + CASE_R @ np.asarray(p, float)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks. ``usd_path`` is a local file, or empty for the SimReady
    screwdriver (fetched at spawn)."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


# Arena places an articulation by its root link: the case's is its base, whose origin is the case's.
CASE = UsdPart("case", asset="case", xy=CASE_XY, extent=CASE_L, mass=0.158,
               usd_path=str(ASSETS / "screwdriver_case.usda"), pos=(*CASE_XY, TOP + 0.0003),
               rot_xyzw=_yaw_xyzw(CASE_YAW), articulation=True)
SCREWDRIVER = UsdPart("screwdriver", asset="screwdriver", xy=SD_GRIP_XY, extent=SD_TIP_X - SD_BUTT_X, mass=0.080,
                      pos=tuple(float(v) for v in SD_REST_POS), rot_xyzw=_matrix_xyzw(SD_REST_R))
OBJECTS = [CASE, SCREWDRIVER]


def fetch_screwdriver() -> Path:
    """The screwdriver's USD: the SimReady package downloaded once (pinned revision) into ``~/.cache/simready``
    (``SIMREADY_CACHE`` overrides), with this task's collider wrapper copied next to its ``simready_usd/`` folder. Only
    its USD layers, textures and licence are fetched, about 45 MB."""
    root = Path(os.environ.get("SIMREADY_CACHE", "~/.cache/simready")).expanduser() / SIMREADY_REPO / SIMREADY_REVISION
    pkg = root / SD_PACKAGE
    target, done = pkg / "simready_usd" / f"sm_{SD_PACKAGE}_01.usd", pkg / ".complete"
    if not (done.is_file() and target.is_file()):
        def get(url: str) -> bytes:
            headers = {"User-Agent": "robocurve-benchmark-task"}
            if os.environ.get("HF_TOKEN"):
                headers["Authorization"] = f"Bearer {os.environ['HF_TOKEN']}"
            with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=120) as response:
                return response.read()

        hf = "https://huggingface.co"
        listing = json.loads(get(f"{hf}/api/datasets/{SIMREADY_REPO}/tree/{SIMREADY_REVISION}/{SD_PACKAGE}"
                                 "?recursive=true"))
        for entry in listing:
            path = entry["path"]
            wanted = path == f"{SD_PACKAGE}/.metadata/LICENSE.txt" or (
                path.startswith(f"{SD_PACKAGE}/simready_usd/") and "/.thumbs/" not in path
                and not (path.startswith(f"{SD_PACKAGE}/simready_usd/source_assets/") and path.endswith(".png"))
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
            raise FileNotFoundError(f"{SIMREADY_REPO}@{SIMREADY_REVISION[:8]}: no {target.relative_to(root)}")
        done.write_text(SIMREADY_REVISION)
    wrapper = pkg / SD_WRAPPER
    source = ASSETS / SD_WRAPPER
    if not wrapper.is_file() or wrapper.read_bytes() != source.read_bytes():
        shutil.copyfile(source, wrapper)
    return wrapper


SOLVER_ITERATIONS = 32  # position iterations per step for the case's articulation (the light clasp slider)
MAX_DEPENETRATION = 0.2  # m/s: tight contacts with 20 N grippers stay stable


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
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path or str(fetch_screwdriver()),
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


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root (Arena's geometry queries take rigid objects only; an articulation's bounds are left at
        its root), plus the grippers, the episode time and the case's lid and clasp."""
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
        case = base.scene["case"]
        ids = case.find_joints(["LidHinge", "LatchSlide"], preserve_order=True)[0]
        lid, latch = case.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
        out["lid_angle"], out["latch_travel"] = float(lid), float(latch)  # rad (negative opens), m (negative: out)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that opens the lid, puts the clasp slider home and
        sets its spring: Isaac Lab starts an articulation's joints at 0 (lid shut) and every drive's target at 0."""
        configure = stock_cfg_callback(control_hz)

        def with_lid(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.case_lid = EventTermCfg(func=_open_lid, mode="reset",
                                                   params={"asset_cfg": SceneEntityCfg("case")})
            return env_cfg

        return with_lid

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


# The drives' targets, as in the case's USD (the hinge has no spring, only damping; the clasp's return spring presses
# the slider home with a 0.2 N preload), and the joints at the start: the lid open on its stop, the slider home.
DRIVE_TARGETS = {"LidHinge": 0.0, "LatchSlide": 0.001}
START_JOINTS = {"LidHinge": -LID_OPEN_ANGLE, "LatchSlide": 0.0}


def _open_lid(env, env_ids, asset_cfg) -> None:
    """Reset event: the lid open on its stop and the clasp slider home, and the clasp spring's target as in the USD."""
    import torch

    if env_ids is None:
        return
    case = env.scene[asset_cfg.name]
    ids = case.find_joints(list(DRIVE_TARGETS), preserve_order=True)[0]
    start = torch.tensor([[START_JOINTS[k] for k in DRIVE_TARGETS]], device=case.device).repeat(len(env_ids), 1)
    case.write_joint_position_to_sim_index(position=start, joint_ids=ids, env_ids=env_ids)
    case.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(start), joint_ids=ids, env_ids=env_ids)
    target = torch.tensor([list(DRIVE_TARGETS.values())], device=case.device).repeat(len(env_ids), 1)
    case.set_joint_position_target_index(target=target, joint_ids=ids, env_ids=env_ids)


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
LIFTED = 0.02  # m: the screwdriver's lowest point at least this far above the table, held, counts as picked up
SEATED_OFF = 0.003  # m: the grip zone's and the shaft's axis points each within this of their places in the cutout
LID_CLOSED = math.radians(-5)  # the lid's hinge past this counts as closed (resting on the clasp it stops at -3 deg)
LID_SHUT = math.radians(-1)  # past this counts as shut, and
LATCH_HOME = 0.0003  # ... the clasp slider back within this: the clasp has caught the lid
CASE_MOVED = 0.03  # m: the case within this of where it started, and
CASE_TURNED = math.radians(10)  # ... turned less than this, for success
UPRIGHT = math.radians(20)  # the case tipped further than this is broken (scores 0)
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
SD_BOX = (np.array([SD_BUTT_X, -SD_HALF_WIDTH, SD_AXIS_Z - SD_HALF_WIDTH]),
          np.array([SD_TIP_X, SD_HALF_WIDTH, SD_AXIS_Z + SD_HALF_WIDTH]))
TRACE_S = 1.0  # s between trace lines in a run
STEPS = ("screwdriver picked up", "screwdriver in its cutout", "lid closed", "lid latched")
POINTS = 0.25  # per step


@dataclass(frozen=True)
class PartPose:
    pos: np.ndarray
    quat_xyzw: np.ndarray


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose``."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ _rot(pose).T + pose.pos
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


def screwdriver_held(objects) -> bool:
    return _held_by_any(objects, _box(objects["screwdriver"], SD_BOX))


def screwdriver_lowest(objects) -> float:
    """The screwdriver's lowest point (its colliders' bounding box, m, world z)."""
    return float(_box(objects["screwdriver"], SD_BOX)[0][2])


def in_case(objects, point) -> np.ndarray:
    """A world point in the case's frame (as the case is now)."""
    c = objects["case"]
    return _rot(c).T @ (np.asarray(point, float) - c.pos)


def seat_offsets(objects) -> tuple[float, float]:
    """How far the screwdriver's grip-zone and shaft axis points are from where they lie when it is seated in its
    cutout, the right way round (m, in the case's frame)."""
    sd = objects["screwdriver"]
    r = _rot(sd)
    off = []
    for local in (GRIP_LOCAL, SHAFT_LOCAL):
        now = in_case(objects, sd.pos + r @ local)
        seat = SEAT_R @ local + SEAT_T
        off.append(float(np.linalg.norm(now - seat)))
    return off[0], off[1]


def screwdriver_seated(objects) -> bool:
    """The screwdriver lies in its cutout, handle in the handle pocket (both axis points within SEATED_OFF), and no
    gripper holds it."""
    a, b = seat_offsets(objects)
    return bool(a < SEATED_OFF and b < SEATED_OFF and not screwdriver_held(objects))


def lid_closed(objects) -> bool:
    return bool(objects["lid_angle"] > LID_CLOSED)


def lid_latched(objects) -> bool:
    return bool(objects["lid_angle"] > LID_SHUT and objects["latch_travel"] > -LATCH_HOME)


def case_upright(objects) -> bool:
    return bool(_rot(objects["case"])[2, 2] > math.cos(UPRIGHT))


def case_moved(objects) -> tuple[float, float]:
    """How far the case has moved (m, on the table) and turned (rad) from where it started."""
    c = objects["case"]
    x = _rot(c)[:, 0]
    turn = abs((math.atan2(x[1], x[0]) - CASE_YAW + math.pi) % (2 * math.pi) - math.pi)
    return float(np.linalg.norm(c.pos[:2] - CASE_POS[:2])), turn


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether a gripper has picked the screwdriver up. Starts over when the
    episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.picked = False
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
        self.picked |= bool(screwdriver_lowest(objects) > TOP + LIFTED and screwdriver_held(objects))


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 1 once reached; 2 to 4 as things stand. Step 2 counts only once a gripper has picked the
    screwdriver up; steps 3 and 4 only with the screwdriver in its cutout."""
    p = _PROGRESS
    p.update(objects)
    seated = bool(p.picked and screwdriver_seated(objects))
    done = [bool(p.picked), seated, bool(seated and lid_closed(objects)), bool(seated and lid_latched(objects))]
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        a, b = seat_offsets(objects)
        sd, case = objects["screwdriver"], objects["case"]
        moved, turned = case_moved(objects)
        rel = in_case(objects, sd.pos + _rot(sd) @ GRIP_LOCAL) - (SEAT_R @ GRIP_LOCAL + SEAT_T)
        axis = _rot(sd)[:, 0]
        line = (f"(seat offsets {a * 1000:.1f}/{b * 1000:.1f} mm, grip point off by "
                f"{np.array2string(rel * 1000, precision=1, separator=',')} mm in the case, axis pitched "
                f"{math.degrees(math.asin(np.clip(axis[2], -1, 1))):.1f} deg; lid {math.degrees(objects['lid_angle']):.1f} deg, "
                f"clasp {objects['latch_travel'] * 1000:.2f} mm; screwdriver {np.linalg.norm(sd.lin_vel) * 1000:.1f} "
                f"mm/s, lowest {(screwdriver_lowest(objects) - TOP) * 1000:.1f} mm; case moved {moved * 1000:.1f} mm, "
                f"{math.degrees(turned):.1f} deg, {np.linalg.norm(case.lin_vel) * 1000:.1f} mm/s)")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: the case or the screwdriver off the table, the case tipped over, or a state that is not finite."""
    for name in ("case", "screwdriver"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return not case_upright(objects)


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the case where it started (within
    3 cm and 10 deg), the case and the screwdriver still, and no violation."""
    moved, turned = case_moved(objects)
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and moved < CASE_MOVED and turned < CASE_TURNED
                and objects["case"].resting() and objects["screwdriver"].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Right arm: from above, the fingers closing across the handle's straight grip zone (its two flat side faces, 34 mm
# apart); lift it, turn it in place to lie along the case and level (it lies pitched 6 deg on the bench; its tip swings
# past the case's right end, clear of the open lid), carry it over the cutout, lower it into the
# cutout with the fingers in the finger notch, let go 1.5 mm above its seat (opening the fingers only a little: the
# notch is 66 mm across), go home.
# Left arm: from behind the open lid, the closed fingertips push its outer face over (past 75 deg it settles shut under
# its own weight, damped), follow it down, press it shut near its front edge so the hook rides over the clasp's ramp and
# the slider snaps over it, go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width
SD_HOLD = -(GRIP_WIDTH - SQUEEZE) / 2  # ~20 N a finger
SD_RELEASE = SD_HOLD - 0.005  # opens each finger 5 mm: inside the finger notch
TIPS_BELOW_AXIS = 0.008  # the pad tips this far below the handle's axis when holding it
PICK_FLIP = 0  # 0 or 1: which way round the right gripper is about the vertical (its wrist's range)
PUSH_FLIP = 0  # ... and the left gripper (0: its closing axis toward the case's front)
PICK_LIFT = 0.09
OVER_SEAT = 0.06  # m: the screwdriver is lowered straight from this far above its seat
RELEASE_ABOVE = 0.0015  # m: let go this far above its seat
# Closing the lid. A gripper pointing straight down can't reach behind the open lid (its wrist bends to its limit), so
# the left gripper points PUSH_TILT forward of straight down, its closing axis along the case's depth: the outer face of
# its closed front finger (FINGER_FACE from its axis) meets the lid's top edge FACE_BACK back from the fingertips,
# PUSH_GAP off it, and tips the lid over toward the robot to TIPPED (past 75 deg its weight takes it the rest of the way,
# damped, onto the clasp's ramp). Then, pointing straight down, the closed fingertips press the lid's top LID_PRESS
# below shut, PRESS_IN from its front face over the clasp, so the hook rides over the ramp and the slider snaps over it.
PUSH_TILT = math.radians(30)
FINGER_FACE = 0.00395
FACE_BACK = 0.005
PUSH_GAP = 0.0015
LID_EDGE = np.array([82.5, 16.0]) * MM  # the lid's top edge from the hinge: along the lid, and out along its normal
TIPPED = math.radians(55)
PRESS_IN = 0.007
LID_PRESS = 0.0025
LID_PUSH_X = 0.0  # along the case (its frame), where the fingertips push: the middle, over the clasp


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


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


PICK_ROT = yam_demo.grip_rotation(SD_YAW + PICK_FLIP * math.pi)  # pointing down, pads along the screwdriver
GRIP_W = SD_REST_POS + SD_REST_R @ GRIP_LOCAL  # the grip zone's middle (world), the screwdriver at rest
PICK_TCP = _tcp_with_tips(GRIP_W, TIPS_BELOW_AXIS, PICK_ROT)
# The screwdriver in the right hand (gripper frame), fixed from the pick on.
SD_IN_HAND_R = PICK_ROT.T @ SD_REST_R
SD_IN_HAND_P = PICK_ROT.T @ (SD_REST_POS - PICK_TCP)
PLACE_ROT = SEAT_R_W @ SD_IN_HAND_R.T  # the gripper holding the screwdriver seated (level, along the case)


def _tcp_holding_sd_at(origin) -> np.ndarray:
    return np.asarray(origin, float) - PLACE_ROT @ SD_IN_HAND_P


# The left gripper tipping the lid: its closing axis (gripper x) along the case's depth toward the front, pointing
# forward and down; and pressing: the same, straight down.
PRESS_ROT = yam_demo.grip_rotation(CASE_YAW + PUSH_FLIP * math.pi)
PUSH_ROT = axis_angle_matrix(CASE_R @ np.array([1.0, 0.0, 0.0]), PUSH_TILT) @ PRESS_ROT
assert PUSH_ROT[0, 2] > 0.4 and np.allclose(PRESS_ROT[:, 0], -CASE_R @ np.array([0.0, 1.0, 0.0])), PUSH_ROT


def _lid_edge(angle: float) -> np.ndarray:
    """The world point of the lid's top edge (its outer face's free edge), over the clasp, the lid opened ``angle``
    (rad, 0 shut)."""
    d = np.array([0.0, -math.cos(angle), math.sin(angle)])  # along the lid, hinge to free edge (case frame)
    n = np.array([0.0, math.sin(angle), math.cos(angle)])  # its outer face's normal
    return case_point(np.array([LID_PUSH_X, *HINGE_YZ]) + LID_EDGE[0] * d + LID_EDGE[1] * n)


def _tip_tcp(edge, gap: float = PUSH_GAP) -> np.ndarray:
    """The left TCP (PUSH_ROT) whose front finger's outer face is ``gap`` behind the lid's edge point ``edge``."""
    on_face = np.array([FINGER_FACE, 0.0, PAD_TIP_PAST_TCP - FACE_BACK])  # gripper frame
    return np.asarray(edge, float) - gap * PUSH_ROT[:, 0] - PUSH_ROT @ on_face


def _press_tcp(press: float) -> np.ndarray:
    """The left TCP (PRESS_ROT) with the closed fingertips ``press`` below the shut lid's top, over the clasp."""
    point = case_point(np.array([LID_PUSH_X, -CASE_W / 2 + PRESS_IN, CASE_H - press]))
    return _tcp_with_tips(point, 0.0, PRESS_ROT)


def right_plan() -> list[Waypoint]:
    """Pick the screwdriver up by its handle, lay it in its cutout, go home."""
    seat_tcp = _tcp_holding_sd_at(SEAT_POS_W)
    release = seat_tcp + (0, 0, RELEASE_ABOVE)
    return [
        Waypoint(2.5, PICK_TCP + (0, 0, 0.10), FINGER_OPEN, "above the handle", rot=PICK_ROT),
        Waypoint(1.5, PICK_TCP, FINGER_OPEN, "around the handle", rot=PICK_ROT),
        Waypoint(0.8, PICK_TCP, SD_HOLD, "grip the handle", rot=PICK_ROT),
        Waypoint(1.5, PICK_TCP + (0, 0, PICK_LIFT), SD_HOLD, "lift it", rot=PICK_ROT),
        Waypoint(1.5, PICK_TCP + (0, 0, PICK_LIFT), SD_HOLD, "turn it along the case", rot=PLACE_ROT),
        Waypoint(2.5, seat_tcp + (0, 0, OVER_SEAT), SD_HOLD, "over the cutout", rot=PLACE_ROT),
        Waypoint(1.5, seat_tcp + (0, 0, 0.012), SD_HOLD, "down into the case", rot=PLACE_ROT),
        Waypoint(1.0, release, SD_HOLD, "down to its seat", rot=PLACE_ROT),
        Waypoint(0.8, release, SD_RELEASE, "let go", rot=PLACE_ROT),
        Waypoint(1.0, release + (0, 0, 0.06), SD_RELEASE, "up", rot=PLACE_ROT),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan(wait_s: float) -> list[Waypoint]:
    """Wait for the screwdriver to be in, tip the lid over from behind by its top edge, press it shut, go home."""
    behind = _tip_tcp(_lid_edge(LID_OPEN_ANGLE), gap=0.008)
    sweep = [_tip_tcp(_lid_edge(math.radians(a))) for a in (100, 92, 84, 76, 68, 61)] + [_tip_tcp(_lid_edge(TIPPED))]
    over_front = _press_tcp(-0.05)
    wps = [
        Waypoint(wait_s, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
        Waypoint(3.0, behind + (0, 0, 0.04), 0.0, "over the lid's back", rot=PUSH_ROT),
        Waypoint(1.0, behind, 0.0, "behind the lid's edge", rot=PUSH_ROT),
    ]
    wps += [Waypoint(0.8, p, 0.0, "tip the lid over", rot=PUSH_ROT) for p in sweep]
    wps += [
        Waypoint(0.6, sweep[-1] + (0, 0, 0.03), 0.0, "let it fall", rot=PUSH_ROT),
        Waypoint(2.0, over_front, 0.0, "over the lid's front", rot=PRESS_ROT),
        Waypoint(0.8, over_front, 0.0, "wait for the lid to settle", rot=PRESS_ROT),  # it falls from 55 deg in ~1.8 s
        Waypoint(1.0, _press_tcp(-0.002), 0.0, "down to the lid", rot=PRESS_ROT),
        Waypoint(0.8, _press_tcp(LID_PRESS), 0.0, "press it shut", rot=PRESS_ROT),
        Waypoint(0.6, _press_tcp(LID_PRESS), 0.0, "hold", rot=PRESS_ROT),
        Waypoint(0.8, over_front, 0.0, "lift away", rot=PRESS_ROT),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps


def demo():
    """The right arm lays the screwdriver in the case; once it is lifting away, the left closes and latches the lid.
    Both go home."""
    right = right_plan()
    let_go = sum(w.duration for w in right[:9])
    return {"left": left_plan(wait_s=let_go + 0.5), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
