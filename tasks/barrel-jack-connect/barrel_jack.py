"""On the station table, against a painted back panel with a wall socket: a 12 V wall adapter plugged into the socket,
its cord running down onto the bench and ending in a female 5.5 x 2.1 mm in-line barrel jack; on the right, an LED
strip dimmer stuck to the bench, its input cord ending in a male 5.5 x 2.1 mm barrel plug. The left arm picks the jack
up and holds it; the right arm picks the plug up by its overmold, turns its hand, pushes the plug fully into the jack,
lets go and goes back; then the left arm lays the joined connectors on the bench, lets go and goes back.

The adapter, its cord and the jack are one articulation (28 cord links on bending joints, the adapter its fixed root)
from ``assets/``; the dimmer, its cord and the plug are another (23 links). The fit between plug and jack is made of
exact PhysX primitives (a 5.9 mm bore of twelve boxes around the 5.5 mm barrel, three springy contacts that centre
and hold it); README.md
says why. The wall socket is a SimReady model, downloaded at spawn (pinned revision, checksummed).

Run it:  inspect-robots run --embodiment yam_arena -E task=barrel_jack.py --scorer task_score ...

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

# -- the parts (their own frames) -------------------------------------------------------------------------------------
LINK, CORD_R = 0.0125, 0.00175  # cord links: 12.5 mm, 3.5 mm thick
N_ADAPTER_LINKS, N_DIMMER_LINKS = 28, 23
# Jack: origin at its mouth (front face centre), +x out of the mouth; 10.5 mm across its flats, 42 mm with its strain
# relief; the cord joins at its back end.
JACK_R, JACK_BACK = 0.00525, 0.042
# Plug: origin at the overmold's front face centre, +x toward the barrel's tip (9.5 mm); the overmold is 9 mm across
# its flats and 20 mm long; the cord joins 33 mm behind the face.
OVERMOLD_R, PLUG_BACK, BARREL_LEN = 0.0045, 0.033, 0.0095
# Adapter: origin at its back face (on the socket's plate), at the socket's centre height, +x out of the wall; its cord
# leaves the strain relief at its bottom, pointing down.
AD_EXIT = np.array([0.018, 0.0, -0.055])
# Dimmer: origin at its bottom centre, +x toward its input cord, which leaves its end 9 mm up, along +x.
DIM_EXIT = np.array([0.038, 0.0, 0.009])
# The wall socket (SimReady equipment_powerOutlet_wallMount_b01): a 60 x 60 mm Schuko socket with a flip lid, its
# frame on the bottom of its box, +x out of the socket face. Its plate's back is 11.5 mm out from its frame (it sits on
# the wall panel's face), its face 18.4 mm out (the adapter's back sits there) and its centre 30 mm up.
OUTLET_PLATE_BACK, OUTLET_FACE, OUTLET_CENTRE_Z = 0.0115, 0.0184, 0.030
OUTLET_LID_OPEN = math.radians(-87.5)  # the lid flipped up, resting on the adapter's top
SIMREADY_REPO = "nvidia/simready-assets"
SIMREADY_REVISION = "7879e0e50423290300ac20ecd6e727325f680b60"
OUTLET_PACKAGE, OUTLET_USD = "equipment_powerOutlet_wallMount_b01", "sm_equipment_powerOutlet_wallMount_b01_01.usd"


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def _ry(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), a)


def _matrix_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    x, y, z, w = ik.matrix_to_quat_xyzw(np.asarray(r, float))
    return float(x), float(y), float(z), float(w)


# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The back panel stands on the bench's back edge, its face at WALL_X; the socket is in it, its centre OUTLET_Y to the
# left and high enough that the adapter's cord, hanging from the adapter's bottom, bends onto the bench.
WALL_X, OUTLET_Y = 0.56, 0.14
DIMMER_XY, DIMMER_YAW = (0.42, -0.24), math.pi  # on the right, toward the back, its input cord toward the robot

# Each cord's starting shape: per joint (``J_kk``: 0 at the root, the last one at the connector), a bend (rad) about
# the joint's y (``J_kk:1``: positive tips the cord down when it lies flat) or about its z (``J_kk:2``: positive turns
# it left). Arena resets objects' poses but not their joint angles, so the links are laid out in this shape when the
# cords spawn (``_lay_cable``) and a reset event writes the same joint angles; the cords' bending springs rest in this
# shape (a cord that has taken a set), so they lie still.
_RISE_JACK = math.asin((JACK_R - CORD_R) / LINK)  # the last link rises to the jack's axis
_RISE_PLUG = math.asin((OVERMOLD_R - CORD_R) / LINK)
_DROP_DIMMER = math.asin((DIM_EXIT[2] - CORD_R) / (2 * LINK))  # the dimmer's cord drops 7.25 mm over two links
ADAPTER_BENDS = {
    **{k: (math.radians(-22.5), 0.0) for k in range(2, 6)},  # down from the adapter, then out along the bench
    **{k: (0.0, math.radians(15.0)) for k in range(18, 24)},  # a 90 deg turn to the robot's right, toward the jack
    N_ADAPTER_LINKS - 1: (-_RISE_JACK, 0.0),
    N_ADAPTER_LINKS: (_RISE_JACK, 0.0),
}
DIMMER_BENDS = {
    0: (_DROP_DIMMER, 0.0),
    2: (-_DROP_DIMMER, 0.0),
    **{k: (0.0, math.radians(-15.0)) for k in range(12, 18)},  # a 90 deg turn to the robot's left, toward the jack
    N_DIMMER_LINKS - 1: (-_RISE_PLUG, 0.0),
    N_DIMMER_LINKS: (_RISE_PLUG, 0.0),
}
# The adapter's cord leaves it pointing down: its first link's frame (x down, z out of the wall).
_CORD_DOWN = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])


def _chain(root_pos, root_rot, exit_pos, exit_rot, bends, n_links, back):
    """World poses (position, rotation) of a cord's links and of its end connector's frame."""
    rot = np.asarray(root_rot, float) @ exit_rot
    p = np.asarray(root_pos, float) + np.asarray(root_rot, float) @ exit_pos
    links = []
    for k in range(n_links + 1):
        pitch, yaw = bends.get(k, (0.0, 0.0))
        rot = rot @ _ry(pitch) @ _rz(yaw)
        if k == n_links:
            return links, (p + rot @ np.array([back, 0.0, 0.0]), rot)
        links.append((p.copy(), rot.copy()))
        p = p + rot @ np.array([LINK, 0.0, 0.0])


_ADAPTER_DROP = 2 * LINK + LINK * sum(math.cos(math.radians(a)) for a in (22.5, 45.0, 67.5))
OUTLET_Z = float(TOP + CORD_R + _ADAPTER_DROP - AD_EXIT[2])  # the socket's centre: 10.7 cm above the bench
ADAPTER_YAW = math.pi  # facing the robot
ADAPTER_POS = (float(WALL_X - (OUTLET_FACE - OUTLET_PLATE_BACK)), float(OUTLET_Y), float(OUTLET_Z))
DIMMER_POS = (float(DIMMER_XY[0]), float(DIMMER_XY[1]), float(TOP))
_AD_LINKS, (JACK_START, JACK_START_ROT) = _chain(ADAPTER_POS, _rz(ADAPTER_YAW), AD_EXIT, _CORD_DOWN, ADAPTER_BENDS,
                                                 N_ADAPTER_LINKS, JACK_BACK)
_DIM_LINKS, (PLUG_START, PLUG_START_ROT) = _chain(DIMMER_POS, _rz(DIMMER_YAW), DIM_EXIT, np.eye(3), DIMMER_BENDS,
                                                  N_DIMMER_LINKS, PLUG_BACK)
# The jack lies with its mouth toward the robot's right (-y), the plug with its tip toward the robot's left (+y).
JACK_AXIS = np.array([0.0, -1.0, 0.0])  # out of the jack's mouth
PLUG_AXIS = np.array([0.0, 1.0, 0.0])  # the plug's insertion direction
# Where the plug goes in: the jack held right above where it lies, its mouth MATE_UP above the bench.
MATE_UP = 0.06  # the turned right gripper's body stays 3 cm above the bench
MATE = np.array([JACK_START[0], JACK_START[1], TOP + MATE_UP])


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation
    (``fixed_base`` pins its root link where it spawns). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed_base: bool = False
    kinematic: bool = False  # a rigid body that never moves (the wall)
    joint_pos: tuple = ()  # (joint name, rad) pairs: an articulation's starting joint angles

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed_base)


def _cord_start(bends) -> tuple:
    out = []
    for k, (pitch, yaw) in bends.items():
        if pitch:
            out.append((f"J_{k:02d}:1", float(pitch)))
        if yaw:
            out.append((f"J_{k:02d}:2", float(yaw)))
    return tuple(out)


WALL = UsdPart("wall", asset="wall", xy=(WALL_X, 0.0), extent=0.8, mass=4.0, usd_path=str(ASSETS / "wall_panel.usda"),
               pos=(WALL_X, 0.0, TOP), rot_xyzw=_yaw_xyzw(math.pi), kinematic=True)
ADAPTER = UsdPart("adapter", asset="adapter", xy=ADAPTER_POS[:2], extent=0.07, mass=0.11,
                  usd_path=str(ASSETS / "adapter_jack_cable.usda"), pos=ADAPTER_POS, rot_xyzw=_yaw_xyzw(ADAPTER_YAW),
                  articulation=True, fixed_base=True, joint_pos=_cord_start(ADAPTER_BENDS))
DIMMER = UsdPart("dimmer", asset="dimmer", xy=DIMMER_XY, extent=0.07, mass=0.035,
                 usd_path=str(ASSETS / "controller_plug_cable.usda"), pos=DIMMER_POS, rot_xyzw=_yaw_xyzw(DIMMER_YAW),
                 articulation=True, fixed_base=True, joint_pos=_cord_start(DIMMER_BENDS))
OBJECTS = [WALL, ADAPTER, DIMMER]


def fetch_outlet() -> Path:
    """The SimReady wall socket's root USD, downloaded once (pinned revision) into ``~/.cache/simready``
    (``SIMREADY_CACHE`` overrides). Only its USD layers, textures and licence are fetched, about 35 MB."""
    root = Path(os.environ.get("SIMREADY_CACHE", "~/.cache/simready")).expanduser() / SIMREADY_REPO / SIMREADY_REVISION
    target, done = root / OUTLET_PACKAGE / "simready_usd" / OUTLET_USD, root / OUTLET_PACKAGE / ".complete"
    if done.is_file() and target.is_file():
        return target

    def get(url: str) -> bytes:
        headers = {"User-Agent": "robocurve-benchmark-task"}
        if os.environ.get("HF_TOKEN"):
            headers["Authorization"] = f"Bearer {os.environ['HF_TOKEN']}"
        with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=120) as response:
            return response.read()

    hf = "https://huggingface.co"
    listing = json.loads(get(f"{hf}/api/datasets/{SIMREADY_REPO}/tree/{SIMREADY_REVISION}/{OUTLET_PACKAGE}"
                             "?recursive=true"))
    for entry in listing:
        path = entry["path"]
        wanted = path == f"{OUTLET_PACKAGE}/.metadata/LICENSE.txt" or (
            path.startswith(f"{OUTLET_PACKAGE}/simready_usd/") and "/.thumbs/" not in path
            and not (path.startswith(f"{OUTLET_PACKAGE}/simready_usd/source_assets/") and path.endswith(".png"))
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
        raise FileNotFoundError(f"{SIMREADY_REPO}@{SIMREADY_REVISION[:8]}: no {OUTLET_PACKAGE}/simready_usd/{OUTLET_USD}")
    done.write_text(SIMREADY_REVISION)
    return target


def _add_outlet(stage, root: str) -> None:
    """The wall socket in the panel at ``root``, part of its fixed body: its own rigid bodies and lid hinge switched
    off, no collisions (only the adapter, which is fixed, touches it), the lid flipped up onto the adapter."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    outlet = stage.DefinePrim(f"{root}/Outlet", "Xform")
    outlet.GetReferences().AddReference(str(fetch_outlet()))
    xf = UsdGeom.Xformable(outlet)
    xf.ClearXformOpOrder()
    # The panel's frame: origin at the bottom of its face, +x out of it (toward the robot), so the socket keeps its own
    # orientation; the socket's centre sits OUTLET_Y to the robot's left, which is the panel's -y.
    xf.AddTranslateOp().Set(Gf.Vec3d(-OUTLET_PLATE_BACK, -OUTLET_Y, OUTLET_Z - TOP - OUTLET_CENTRE_Z))
    for name in ("cap_01_obj_00", "socket_01_obj_00"):
        part = stage.GetPrimAtPath(f"{root}/Outlet/Geometry/{name}")
        part.SetInstanceable(False)
        part.RemoveAPI(UsdPhysics.RigidBodyAPI)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{root}/Outlet")):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    joints = stage.GetPrimAtPath(f"{root}/Outlet/Joints")
    if joints:
        joints.SetActive(False)
    lid = UsdGeom.Xformable(stage.GetPrimAtPath(f"{root}/Outlet/Geometry/cap_01_obj_00"))
    for op in lid.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            half = OUTLET_LID_OPEN / 2
            op.Set(Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, math.sin(half), 0.0)))


def _lay_cable(stage, root: str, links, end, end_name: str, bends, asset_pos, asset_rot) -> None:
    """Place a cord's links and its end connector as laid out (world poses -> the asset's frame), and make that shape
    the cord's rest shape: the bending springs' targets are the joint angles (USD drives are in degrees)."""
    from pxr import Gf, UsdGeom

    inv = np.asarray(asset_rot, float).T
    origin = np.asarray(asset_pos, float)

    def place(path, pos, rot):
        ops = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
        p = inv @ (pos - origin)
        x, y, z, w = _matrix_xyzw(inv @ rot)
        ops[0].Set(Gf.Vec3d(*p))
        ops[1].Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))

    for k, (pos, rot) in enumerate(links):
        place(f"{root}/Cable/Link_{k:02d}", pos, rot)
    place(f"{root}/{end_name}", *end)
    for k, (pitch, yaw) in bends.items():
        joint = stage.GetPrimAtPath(f"{root}/Joints/J_{k:02d}")
        joint.GetAttribute("drive:rotY:physics:targetPosition").Set(math.degrees(pitch))
        joint.GetAttribute("drive:rotZ:physics:targetPosition").Set(math.degrees(yaw))


def _spawner(spec: "UsdPart", base):
    """A spawn function: ``base`` (Isaac Lab's, or Arena's fixture spawner), then the cord laid out or the socket
    added, in every environment."""

    def spawn(prim_path: str, cfg, *args, **kwargs):
        import isaaclab.sim as sim_utils

        prim = base(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        for root in sim_utils.find_matching_prim_paths(prim_path):
            if spec.name == "wall":
                _add_outlet(stage, root)
            elif spec.name == "adapter":
                _lay_cable(stage, root, _AD_LINKS, (JACK_START, JACK_START_ROT), "Jack", ADAPTER_BENDS, spec.pos,
                           _rz(ADAPTER_YAW))
            elif spec.name == "dimmer":
                _lay_cable(stage, root, _DIM_LINKS, (PLUG_START, PLUG_START_ROT), "Plug", DIMMER_BENDS, spec.pos,
                           _rz(DIMMER_YAW))
        return prim

    return spawn


SOLVER_ITERATIONS = 32  # position iterations per step for the cords: light links (0.3 g) and light connectors (4-7 g)
MAX_DEPENETRATION = 0.2  # m/s: ... in tight contacts with 10 N grips stay stable


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.from_files import spawn_from_usd

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": _spawner(spec, scene._spawn_fixture),
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION),
             "func": _spawner(spec, spawn_from_usd)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=spec.fixed_base or None, solver_position_iteration_count=SOLVER_ITERATIONS,
            solver_velocity_iteration_count=1)
    obj = scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )
    if spec.joint_pos:  # Arena keeps the Isaac Lab config: its starting joint angles are set (and reset to) here
        obj.object_cfg.init_state.joint_pos = dict(spec.joint_pos)
    return obj


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


def _cords_start(env, env_ids, asset_cfg, start) -> None:
    """Reset event: a cord's joints in its laid-out shape, still (the simulator keeps an articulation's joints where
    they were and starts them at 0). The bending springs' targets need no writing: Arena gives task objects no
    actuators, so Isaac Lab never sends joint targets, and the drives keep the targets ``_lay_cable`` wrote into the
    USD at spawn."""
    import torch

    if env_ids is None:
        return
    art = env.scene[asset_cfg.name]
    names = list(art.joint_names)
    q = torch.zeros((len(env_ids), len(names)), device=art.device)
    for name, value in start:
        q[:, names.index(name)] = value
    ids = list(range(len(names)))
    art.write_joint_position_to_sim_index(position=q, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids, env_ids=env_ids)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root (an articulation's bounds are left at its root), plus the grippers, the episode time and
        the two connectors."""
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
        out["jack"] = _link_pose(base, origin, "adapter", "Jack")
        out["plug"] = _link_pose(base, origin, "dimmer", "Plug")
        speeds = []
        for name, asset, link in (("jack_vel", "adapter", "Jack"), ("plug_vel", "dimmer", "Plug")):
            art = base.scene[asset]
            i = art.find_bodies([link])[0][0]
            vel = art.data.body_link_vel_w.torch[0].detach().cpu().numpy().astype(np.float64)
            out[name] = vel[i]
            speeds.append(float(np.linalg.norm(vel[:, :3], axis=1).max()))
        out["cords_speed"] = max(speeds)  # the fastest cord link or connector (m/s)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus reset events that put both cords in their laid-out shape."""
        configure = stock_cfg_callback(control_hz)

        def with_cords(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            for spec in (ADAPTER, DIMMER):
                setattr(env_cfg.events, f"{spec.name}_cord", EventTermCfg(
                    func=_cords_start, mode="reset",
                    params={"asset_cfg": SceneEntityCfg(spec.name), "start": spec.joint_pos}))
            return env_cfg

        return with_cords

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
PLUGGED_DEPTH = 0.001  # m: the plug's overmold face within this of the jack's mouth (the barrel 9.5 mm in), and
PLUGGED_OFF = 0.0005  # ... the plug within this of the bore's centre line, and
PLUGGED_ANGLE = math.radians(5)  # ... its axis within this of the bore's
LIFTED = 0.02  # m: a part's lowest point at least this far above the bench counts as lifted
ON_BENCH = 0.003  # m: a part whose lowest point is within this of the bench lies on it
FALLEN_Z = -0.02  # m above the bench: a part whose centre is lower has fallen off the bench
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
STILL = 0.01  # m/s: the connectors this slow are still
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
# The connectors' boxes in their own frames (m): the jack (mouth at 0, back to its strain relief) and the plug (face at
# 0, back to its strain relief, forward over the barrel).
JACK_BOX = (np.array([-JACK_BACK, -JACK_R, -JACK_R]), np.array([0.0, JACK_R, JACK_R]))
PLUG_BOX = (np.array([-PLUG_BACK, -OVERMOLD_R, -OVERMOLD_R]), np.array([BARREL_LEN, OVERMOLD_R, OVERMOLD_R]))
STEPS = ("jack lifted", "plug lifted", "plugged in by hand", "joined connectors laid down, both let go")
POINTS = 0.25  # per step
TRACE_EVERY_S = float(os.environ.get("BARREL_JACK_TRACE_S", 0.5))  # s between trace lines in a run


def plug_in_jack(objects):
    """The plug's overmold face in the jack's frame (m; x out of the mouth, 0 at the mouth: negative is in) and the
    angle between the plug's axis and the jack's bore (rad)."""
    j, p = objects["jack"], objects["plug"]
    rj = quat_xyzw_to_matrix(j.quat_xyzw)
    d = rj.T @ (p.pos - j.pos)
    angle = math.acos(float(np.clip(quat_xyzw_to_matrix(p.quat_xyzw)[:, 0] @ -rj[:, 0], -1.0, 1.0)))
    return d, angle


def plugged_in(objects) -> bool:
    """The plug is fully in the jack: its face within PLUGGED_DEPTH of the mouth, on the bore's line within PLUGGED_OFF
    and its axis within PLUGGED_ANGLE of the bore's."""
    d, angle = plug_in_jack(objects)
    return bool(d[0] < PLUGGED_DEPTH and np.hypot(d[1], d[2]) < PLUGGED_OFF and angle < PLUGGED_ANGLE)


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


def _lifted_and_held(objects, pose, box) -> bool:
    bounds = _box(pose, box)
    return bool(bounds[0][2] > TOP + LIFTED and _held_by_any(objects, bounds))


def laid_down(objects) -> bool:
    """Both connectors lie on the bench (their lowest points within ON_BENCH of it) and no gripper holds either."""
    jb, pb = _box(objects["jack"], JACK_BOX), _box(objects["plug"], PLUG_BOX)
    return bool(jb[0][2] < TOP + ON_BENCH and pb[0][2] < TOP + ON_BENCH
                and not _held_by_any(objects, jb) and not _held_by_any(objects, pb))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether the jack and the plug have been lifted, and how the plug went in:
    held by a gripper (pushed in by hand) or not. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.jack_lifted = self.plug_lifted = False
        self.plugged, self.plug_held = False, False
        self.by_hand = self.not_by_hand = False
        self.reported = [False] * len(STEPS)
        self.traced = -1e9

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.jack_lifted |= _lifted_and_held(objects, objects["jack"], JACK_BOX)
        self.plug_lifted |= _lifted_and_held(objects, objects["plug"], PLUG_BOX)
        plugged = plugged_in(objects)
        held = _held_by_any(objects, _box(objects["plug"], PLUG_BOX))
        if plugged and not self.plugged:  # it just went fully in: held then (or a step before), or not
            if held or self.plug_held:
                self.by_hand = True
            else:
                self.not_by_hand = True
        self.plugged, self.plug_held = plugged, held


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: steps 1 and 2 once reached; 3 and 4 as things stand. Step 3 counts only if a gripper held the
    plug when it went fully in; a plug that ever went in without a gripper on it never earns steps 3 and 4."""
    p = _PROGRESS
    p.update(objects)
    plugged = bool(plugged_in(objects) and p.by_hand and not p.not_by_hand)
    done = [bool(p.jack_lifted), bool(p.plug_lifted), plugged, bool(plugged and laid_down(objects))]
    t = objects.get("episode_time")
    if t is not None and t >= p.traced + TRACE_EVERY_S - 1e-6:  # in a run, a trace line every TRACE_EVERY_S
        p.traced = t
        d, angle = plug_in_jack(objects)
        ja = quat_xyzw_to_matrix(objects["jack"].quat_xyzw)[:, 0]
        pa = -quat_xyzw_to_matrix(objects["plug"].quat_xyzw)[:, 0]  # the plug's axis, turned to compare with the jack's
        axes = " ".join(f"{n} yaw {math.degrees(math.atan2(a[1], a[0])):+.1f} pitch {math.degrees(math.asin(np.clip(a[2], -1, 1))):+.1f}"
                        for n, a in (("jack", ja), ("plug", pa)))
        print(f"[barrel_jack] t={t:.2f}s  trace: {axes}; jack {np.round(objects['jack'].pos, 4).tolist()}, plug "
              f"{np.round(objects['plug'].pos, 4).tolist()}, plug face {d[0] * 1000:+.1f} mm out of the mouth, "
              f"{np.hypot(d[1], d[2]) * 1000:.2f} mm off, {math.degrees(angle):.1f} deg; fastest cord part "
              f"{objects.get('cords_speed', 0.0) * 1000:.1f} mm/s", flush=True)
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        d, angle = plug_in_jack(objects)
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        print(f"[barrel_jack] t={objects['episode_time']:.2f}s  steps done: {got}  (plug {-d[0] * 1000:+.2f} mm from "
              f"fully in, {np.hypot(d[1], d[2]) * 1000:.2f} mm off the bore's line, tilted {math.degrees(angle):.1f} "
              "deg)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: a connector off the bench, or a state that is not finite."""
    for name in ("jack", "plug"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return False


def _still(objects) -> bool:
    return all(float(np.linalg.norm(objects[k][:3])) < STILL for k in ("jack_vel", "plug_vel") if k in objects)


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the connectors still, and no
    violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and _still(objects))


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: straight down over the jack's middle, fingers across it; lift it straight up so its mouth is MATE_UP above
# the bench, and hold it there while the right arm plugs in and goes home; then lay it back down, let go, go home.
# Right arm: straight down over the plug's overmold, fingers across it; lift it, and turn the hand a quarter turn about
# the plug's own axis so the gripper points forward (+x) with its fingers above and below the plug: its wrist then
# stays in front of the plug, well clear of the left wrist above the jack. Line the plug up a few cm short of the
# jack's mouth, push it in along +y until the overmold meets the jack, let go, back off, go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width, pressing ~20 N each
JACK_HOLD = -(2 * JACK_R - SQUEEZE / 2) / 2  # ~10 N a finger on the 7 g jack
PLUG_HOLD = -(2 * OVERMOLD_R - SQUEEZE / 2) / 2  # ~10 N a finger on the 4 g plug
JACK_GRIP_AT = 0.020  # m behind the jack's mouth: its body, clear of the mouth and the strain relief
PLUG_GRIP_AT = 0.016  # m behind the plug's face: the 27 mm wide pads then end 2.3 mm behind it, clear of the jack
TIPS_UP = 0.0015  # m: the pad tips this far above the bench at the picks
TCP_UP = TIPS_UP + PAD_TIP_PAST_TCP  # the TCP's height above the bench at the picks
PLUG_IN_HAND = TCP_UP - OVERMOLD_R  # the plug's axis this far from the TCP along the pointing axis, once gripped
JACK_IN_HAND = TCP_UP - JACK_R
STANDOFF = 0.03  # m: the plug's face this far short of the jack's mouth before lining up
ENTRY = 0.012  # ... then this far (the barrel's tip 2.5 mm short of the mouth), to settle before pushing
# The arms are soft position servos (a few hundred N/m at the hand): the push target goes this far past seated so the
# arm's spring builds up the force the jack's spring contacts need (the jack's face stops the overmold); then the hand
# eases back to the seated place while it opens, so it stops pressing.
PUSH_PAST = 0.006
PLUG_ALIGN_FIX = np.array([0.0, 0.0, 0.0])  # measured in simulation: where the plug hand aims, from where the mouth is
# Measured in simulation: held in the turned hand, the plug points 1.8 deg nose-down and 1.1 deg toward the robot from
# the hand's aim (it sits in the pads' friction), so the hand is turned back by that much for the push.
PLUG_PITCH_FIX, PLUG_YAW_FIX = math.radians(1.8), math.radians(-1.1)
RELEASE = -0.012  # m a finger: the right hand opens to 24 mm to let go, so its fingers stay clear of the left hand's
LAY_DOWN = 0.001  # m: the jack set down this far above where it first lay


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

LEFT_GRIP = yam_demo.grip_rotation(-math.pi / 2)  # pointing down, fingers across the jack (closing along x)
RIGHT_PICK = yam_demo.grip_rotation(math.pi / 2)  # pointing down, fingers across the plug
# Once the plug is up, the right hand turns about the plug's own axis (y) until it points forward and down at ROLL
# from the vertical: the plug stays level, and the wrist moves out in front of it, away from the left wrist above the
# jack.
ROLL = math.radians(70)
_DOWN = np.array([0.0, 0.0, -1.0])


def _right_turned() -> np.ndarray:
    return _ry(-ROLL) @ RIGHT_PICK
JACK_GRIP = JACK_START - JACK_GRIP_AT * JACK_AXIS  # the jack's grip point (on its axis) where it lies
PLUG_GRIP = PLUG_START - PLUG_GRIP_AT * PLUG_AXIS


def _plug_fix() -> np.ndarray:
    return _rz(PLUG_YAW_FIX) @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), PLUG_PITCH_FIX)


def _right_tcp(face) -> np.ndarray:
    """The right TCP holding the plug (the hand turned, and corrected by ``_plug_fix``) whose face is at ``face``."""
    offset = PLUG_GRIP_AT * PLUG_AXIS + PLUG_IN_HAND * _right_turned()[:, 2]
    return np.asarray(face, float) - _plug_fix() @ offset




def right_plan() -> list[Waypoint]:
    """Pick the plug up across its overmold, turn the hand, line it up short of the jack, push it in, let go, go."""
    pick = np.array([PLUG_GRIP[0], PLUG_GRIP[1], TOP + TCP_UP])
    aim = MATE + PLUG_ALIGN_FIX
    out, near, seated = aim - STANDOFF * PLUG_AXIS, aim - ENTRY * PLUG_AXIS, aim + PUSH_PAST * PLUG_AXIS
    turned = _plug_fix() @ _right_turned()
    back = -turned[:, 2]  # straight back out of the grip
    return [
        Waypoint(2.5, pick + (0, 0, 0.05), FINGER_OPEN, "over the plug", rot=RIGHT_PICK),
        Waypoint(1.5, pick, FINGER_OPEN, "down to the plug", rot=RIGHT_PICK),
        Waypoint(0.8, pick, PLUG_HOLD, "grip the plug", rot=RIGHT_PICK),
        Waypoint(1.5, pick + (0, 0, 0.05), PLUG_HOLD, "lift it", rot=RIGHT_PICK),
        Waypoint(2.5, _right_tcp(out), PLUG_HOLD, "turn the hand, in front of the jack", rot=turned),
        Waypoint(1.0, _right_tcp(out), PLUG_HOLD, "settle", rot=turned),
        Waypoint(1.5, _right_tcp(near), PLUG_HOLD, "up to the mouth", rot=turned),
        Waypoint(1.0, _right_tcp(near), PLUG_HOLD, "settle at the mouth", rot=turned),
        Waypoint(2.5, _right_tcp(seated), PLUG_HOLD, "push it in", rot=turned),
        Waypoint(1.0, _right_tcp(aim), RELEASE, "let go", rot=turned),
        Waypoint(0.5, _right_tcp(aim), RELEASE, "fingers open", rot=turned),
        Waypoint(1.0, _right_tcp(aim) + 0.03 * back, RELEASE, "back off", rot=turned),
        Waypoint(1.0, _right_tcp(aim) + 0.05 * back - 0.06 * PLUG_AXIS, FINGER_OPEN, "aside, away from the jack",
                 rot=turned),
        Waypoint(1.0, _right_tcp(aim) + 0.05 * back - 0.06 * PLUG_AXIS + (0, 0, 0.05), FINGER_OPEN, "up", rot=turned),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan(hold_s: float) -> list[Waypoint]:
    """Pick the jack up across its middle, lift it straight up, hold it while the plug goes in and the right arm goes
    home, then lay the joined connectors down where the jack lay, let go and go home."""
    rot = LEFT_GRIP
    at = np.array([JACK_GRIP[0], JACK_GRIP[1], TOP + TCP_UP])
    held = MATE - JACK_GRIP_AT * JACK_AXIS + JACK_IN_HAND * -_DOWN
    down = at + (0, 0, LAY_DOWN)
    return [
        Waypoint(2.5, at + (0, 0, 0.05), FINGER_OPEN, "over the jack", rot=rot),
        Waypoint(1.5, at, FINGER_OPEN, "down to the jack", rot=rot),
        Waypoint(0.8, at, JACK_HOLD, "grip the jack", rot=rot),
        Waypoint(2.0, held, JACK_HOLD, "lift it", rot=rot),
        Waypoint(hold_s, held, JACK_HOLD, "hold it still", rot=rot),
        Waypoint(2.5, down, JACK_HOLD, "lay it down", rot=rot),
        Waypoint(1.0, down, FINGER_OPEN, "let go", rot=rot),
        Waypoint(0.5, down, FINGER_OPEN, "fingers open", rot=rot),
        Waypoint(1.0, down + (0, 0, 0.05), FINGER_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(3.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


LEFT_HELD_S = 2.5 + 1.5 + 0.8 + 2.0  # the left arm over, down, closed on the jack and lifted


def demo():
    """Both arms: the left lifts the jack and holds it, the right plugs the plug in and goes home, then the left lays
    the joined connectors down and goes home."""
    right = right_plan()
    return {"left": left_plan(hold_s=sum(w.duration for w in right) - LEFT_HELD_S + 0.5), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
