"""On the station table: a mini 18650 spot welder (124 x 24 x 22 mm housing, black ABS) lying on its feet in front of
the robot, with the USB-C cable of a power bank (92 x 46 x 14 mm, fixed to the table on the right) plugged into its
rear port; on the left, an empty 3D-printed holder for the welder, screwed to the table. The left arm holds the welder
down while the right arm pulls the plug out by its plastic body, lays it on the table and goes back; then the left
arm puts the welder into its holder and goes back.

The power bank, cable and plug are one articulation (24 cable links on bending joints) from ``assets/``, spawned with
the plug fully in the port; the welder is another (its housing, lid, latch and the two USB-C spring fingers that press
on the plug). The port has real USB-C clearances (0.10-0.23 mm a side); README.md says how its colliders hold that fit.

Run it:  inspect-robots run --embodiment yam_arena -E task=welder_unplug.py --scorer task_score ...

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
SUCCESS_HOLD_S = 1.0
ASSETS = Path(__file__).resolve().parent / "assets"

# -- the parts (their own frames) -------------------------------------------------------------------------------------
# Power bank asset: origin at the bank's bottom centre at its cable end; the cable leaves along +x at z = 7 mm and the
# plug's frame sits on its mating face (the overmold's front), 338 mm out, +x the insertion direction.
PLUG_IN_BANK = np.array([0.338, 0.0, 0.007])
OVERMOLD = (0.022, 0.012, 0.0062)  # length behind the mating face, width, thickness
# The right grip's centre this far behind the mating face: the YAM finger pads are ~28 mm wide, so their front edge ends
# ~0.8 mm behind the face and they never reach the welder's rear face, while gripping the whole 22 mm overmold.
PLUG_GRIP_BEHIND = 0.015
# Welder asset: origin at the rear end, bottom of the housing, centre; +x to the probes; the port is in the rear face.
PORT_IN_WELDER = np.array([0.0, 0.0, 0.011])
PORT_IN_BODY = PORT_IN_WELDER - (0.004, 0.0, 0.0)  # the housing body's own origin sits 4 mm in from the rear face
WELDER_WIDTH, WELDER_HEIGHT = 0.024, 0.022
WELDER_GRIP_AT = 0.062  # along the welder from its rear face: its middle, so it hangs level in the grip
FEET = 0.0006  # the rubber feet reach this far below the housing's bottom
# The 18650 cell in the welder's bay, under the closed lid (its place in the welder asset).
CELL_IN_WELDER = np.array([0.0421, 0.0, 0.011])
CELL_IN_WELDER_XYZW = (-0.5, 0.5, -0.5, 0.5)

# The holder asset: origin at its underside centre (its rubber feet reach 0.5 mm below), +x toward the welder's probes.
# The welder sits in it with its origin here (on its feet on the pocket floor, centred); its walls are 14 mm tall.
SEAT_IN_HOLDER = np.array([-0.062, 0.0, 0.0035])
HOLDER_WALL_TOP = 0.014
HOLDER_FEET = 0.0005

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
WELDER_XY, WELDER_YAW = (0.27, -0.07), math.pi / 2  # its rear face at y = -0.07, port facing -y, probes toward +y
CABLE_YAW = math.pi / 2  # the cable runs along +y, from the power bank to the port
HOLDER_XY, HOLDER_YAW = (0.30, 0.26), math.pi  # on the left, lengthwise front to back: probes toward the robot

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
PORT_START = np.array(WELDER_POS) + _rz(WELDER_YAW) @ PORT_IN_WELDER  # the port's mouth, where the plug's face starts
HOLDER_POS = (*HOLDER_XY, TOP + HOLDER_FEET)
SEAT = np.array(HOLDER_POS) + _rz(HOLDER_YAW) @ SEAT_IN_HOLDER  # the welder's origin seated in the holder


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
    kinematic: bool = False  # a rigid body that never moves (screwed to the table)
    joint_pos: tuple = ()  # (joint-name regex, rad) pairs: an articulation's starting joint angles


# The cable starts lying on the table from the power bank to the welder and rises over its last links into the port:
# it drops from the bank's outlet onto the table over its first links, runs along the table, and its last joint
# levels the plug at the port's height. Each cable joint is spherical (twist, then two bends; ":1" is up-down,
# positive bending the cable down). Arena resets objects' poses but not their joint angles, so the links are laid out
# in this pose when the power bank spawns (``_lay_cable``), and the same angles are its joints' defaults.
CABLE_LINK = 0.0125  # m: each of the 24 links
CABLE_RADIUS = 0.00175
CABLE_OUTLET = np.array([0.008, 0.0, 0.007])  # the first joint, on the bank (asset frame)
PLUG_BEHIND_JOINT = 0.030  # the plug's face this far past the last joint
BANK_UP = 0.0005  # the bank's origin above the table (its feet)
DROP_LINKS, RISE_LINKS = 2, 4
_DROP = math.asin((CABLE_OUTLET[2] - (CABLE_RADIUS - BANK_UP)) / (DROP_LINKS * CABLE_LINK))
_RISE = math.asin(((FEET + PORT_IN_WELDER[2]) - CABLE_RADIUS) / (RISE_LINKS * CABLE_LINK))
CABLE_BENDS = {0: _DROP, DROP_LINKS: -_DROP, 24 - RISE_LINKS: -_RISE, 24: _RISE}  # joint -> rad, positive down


def _cable_layout():
    """Each link's origin and pitch (rad, up positive) in the power bank's frame, then the plug's face, from the bends."""
    p, pitch, links = CABLE_OUTLET.copy(), 0.0, []
    for k in range(24):
        pitch -= CABLE_BENDS.get(k, 0.0)
        links.append((p.copy(), pitch))
        p = p + CABLE_LINK * np.array([math.cos(pitch), 0.0, math.sin(pitch)])
    pitch -= CABLE_BENDS.get(24, 0.0)
    return links, p + PLUG_BEHIND_JOINT * np.array([math.cos(pitch), 0.0, math.sin(pitch)]), pitch


_CABLE_LINKS, _PLUG_FACE_IN_BANK, _ = _cable_layout()
CABLE_START = tuple((f"J_{k:02d}:1", v) for k, v in CABLE_BENDS.items())
# The bank sits so the laid-out cable ends with the plug's face in the port.
BANK_XY = (WELDER_XY[0], float(PORT_START[1] - _PLUG_FACE_IN_BANK[0]))
BANK_POS = (*BANK_XY, TOP + BANK_UP)
POWERBANK = UsdPart("powerbank", asset="powerbank", xy=BANK_XY, extent=0.1, mass=0.1356,
                    usd_path=str(ASSETS / "powerbank_task.usda"), pos=BANK_POS, rot_xyzw=_yaw_xyzw(CABLE_YAW),
                    articulation=True, fixed_base=True, joint_pos=CABLE_START)
# Arena places an articulation by its root link: the welder's is its housing body, 4 mm in from the rear face.
WELDER = UsdPart("welder", asset="welder", xy=WELDER_XY, extent=0.14, mass=0.1267,
                 usd_path=str(ASSETS / "welder_task.usda"),
                 pos=tuple(float(v) for v in np.array(WELDER_POS) + _rz(WELDER_YAW) @ (PORT_IN_WELDER - PORT_IN_BODY)),
                 rot_xyzw=_yaw_xyzw(WELDER_YAW), articulation=True)
CELL = UsdPart("cell", asset="cell", xy=WELDER_XY, extent=0.065, mass=0.047,
               usd_path=str(ASSETS / "battery_18650.usda"),
               pos=tuple(float(v) for v in np.array(WELDER_POS) + _rz(WELDER_YAW) @ CELL_IN_WELDER),
               rot_xyzw=tuple(float(v) for v in _qmul(_yaw_xyzw(WELDER_YAW), CELL_IN_WELDER_XYZW)))
HOLDER = UsdPart("holder", asset="holder", xy=HOLDER_XY, extent=0.15, mass=0.0207,
                 usd_path=str(ASSETS / "spot_welder_holder.usda"), pos=HOLDER_POS, rot_xyzw=_yaw_xyzw(HOLDER_YAW),
                 kinematic=True)
OBJECTS = [POWERBANK, WELDER, CELL, HOLDER]


# The welder housing's collider is a signed distance field of its whole shell; inside the USB-C opening it isn't
# exact to the 0.1 mm the fit needs, so the plug's metal shell meets only the port's own steel pieces (as in a real
# receptacle). The overmold still meets the housing, where it seats on the rear face.
PLUG_SHELL_COLLIDERS = "Plug/Colliders/Shell_Cap.*"
WELDER_HOUSING_COLLIDER = "Body/Body_Shell"


def _lay_cable(stage, root: str) -> None:
    """Place the cable's links and the plug as ``_cable_layout`` has them (the starting pose of ``CABLE_START``), and
    make that pose the cable's rest shape, so its bending springs don't straighten it and push the plug in."""
    from pxr import Gf, UsdGeom

    for k, (origin, pitch) in enumerate(_CABLE_LINKS):
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(f"{root}/Cable/Link_{k:02d}"))
        xf.GetOrderedXformOps()[0].Set(Gf.Vec3d(*origin))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Quatf(math.cos(-pitch / 2), Gf.Vec3f(0.0, math.sin(-pitch / 2), 0.0)))  # about y: +x tips up
    UsdGeom.Xformable(stage.GetPrimAtPath(f"{root}/Plug")).GetOrderedXformOps()[0].Set(Gf.Vec3d(*_PLUG_FACE_IN_BANK))
    for k, bend in CABLE_BENDS.items():  # the cable's bending springs rest in this shape (a cable that has taken a set)
        stage.GetPrimAtPath(f"{root}/Joints/J_{k:02d}").GetAttribute("drive:rotY:physics:targetPosition").Set(
            math.degrees(bend))


def _spawn_powerbank(prim_path: str, cfg, *args, **kwargs):
    """Spawn the power bank as Isaac Lab does, with its cable laid out plugged into the welder, then keep its plug's
    shell clear of the welder housing's collider."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import UsdPhysics

    prim = spawn_from_usd(prim_path, cfg, *args, **kwargs)
    stage = prim.GetStage()
    for root in sim_utils.find_matching_prim_paths(prim_path):
        _lay_cable(stage, root)
    for path in sim_utils.find_matching_prim_paths(f"{prim_path}/{PLUG_SHELL_COLLIDERS}"):
        env = path.split("/powerbank/")[0]
        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path))
        pairs.CreateFilteredPairsRel().AddTarget(f"{env}/welder/{WELDER_HOUSING_COLLIDER}")
    return prim


SOLVER_ITERATIONS = 32  # position iterations per step for the articulations: light links (0.3 g cable, 4 g plug) in
MAX_DEPENETRATION = 0.2  # m/s: ... tight contacts with 20 N grippers stay stable


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             kinematic_enabled=spec.kinematic or None)}
    if spec.name == "powerbank":
        addon["func"] = _spawn_powerbank
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




def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each articulation's root (Arena's geometry queries take rigid objects only; its bounds are left at its
        root), plus the grippers, the episode time, the plug and the welder's housing."""
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
        out["plug"] = _link_pose(base, origin, "powerbank", "Plug")
        out["housing"] = _link_pose(base, origin, "welder", "Body")
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that puts the cable in its starting pose: Arena
        resets objects' poses but not their joint angles, and the simulator starts an articulation's joints at 0."""
        configure = stock_cfg_callback(control_hz)

        def with_cable_start(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg
            from isaaclab_arena.terms.events import reset_articulation_joints

            env_cfg = configure(env_cfg)
            env_cfg.events.cable_start = EventTermCfg(func=reset_articulation_joints, mode="reset",
                                                      params={"asset_cfg": SceneEntityCfg("powerbank")})
            return env_cfg

        return with_cable_start

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
UNPLUGGED = 0.005  # m: the plug's front face at least this far out of the port counts as out of it, or
UNPLUGGED_OFF = 0.002  # ... the plug this far off the port's line, or
UNPLUGGED_ANGLE = math.radians(30)  # ... turned this far from the port's direction
ON_TABLE = 0.003  # m: a part whose lowest point is within this of the table lies on it
LIFTED = 0.02  # m: a part's lowest point at least this far above the table counts as lifted
SEATED_OFF = 0.005  # m: the welder's origin within this of its seat in the holder (sideways and along it), and
SEATED_UP = 0.002  # ... within this of the pocket floor's height, and
UPRIGHT = math.radians(10)  # ... upright within this
CELL_SHIFT = 0.01  # m: the cell farther than this from its place in the welder's bay has come out (lid opened)
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
BODY_SHIFT = np.array([0.004, 0.0, 0.0])  # the housing's Body frame starts 4 mm in from the rear face
# The parts' boxes in their own frames (m): the welder housing (its Body frame) and the plug (its frame on its body's
# front face: back to the cable boot, forward over the metal shell).
WELDER_BOX = (np.array([-0.004, -WELDER_WIDTH / 2, 0.0]), np.array([0.120, WELDER_WIDTH / 2, WELDER_HEIGHT]))
PLUG_BOX = (np.array([-0.030, -OVERMOLD[1] / 2, -OVERMOLD[2] / 2]), np.array([0.0066, OVERMOLD[1] / 2, OVERMOLD[2] / 2]))
CELL_IN_BODY = CELL_IN_WELDER - BODY_SHIFT
STEPS = ("unplugged by hand", "plug put down", "welder lifted", "welder in its holder")
POINTS = 0.25  # per step


def plug_in_port(objects):
    """The plug's front face in the port's frame (m; x along the insertion direction, 0 at the welder's rear face) and
    the angle between the plug's and the port's directions (rad)."""
    h, p = objects["housing"], objects["plug"]
    rh = quat_xyzw_to_matrix(h.quat_xyzw)
    mouth = h.pos + rh @ PORT_IN_BODY
    d = rh.T @ (p.pos - mouth)
    angle = math.acos(float(np.clip(quat_xyzw_to_matrix(p.quat_xyzw)[:, 0] @ rh[:, 0], -1.0, 1.0)))
    return d, angle


def unplugged(objects) -> bool:
    """The plug is out of the port: its face more than UNPLUGGED out, off the port's line, or turned away from it."""
    d, angle = plug_in_port(objects)
    return bool(d[0] < -UNPLUGGED or np.hypot(d[1], d[2]) > UNPLUGGED_OFF or angle > UNPLUGGED_ANGLE)


def welder_in_holder(objects):
    """The welder's origin relative to its seat in the holder (m, in the holder's frame) and whether it is seated: on the
    pocket floor, centred within SEATED_OFF and upright within UPRIGHT."""
    h = objects["housing"]
    rh = quat_xyzw_to_matrix(h.quat_xyzw)
    origin = h.pos - rh @ BODY_SHIFT
    holder = objects["holder"]
    rr = quat_xyzw_to_matrix(holder.quat_xyzw)
    d = rr.T @ (origin - (holder.pos + rr @ SEAT_IN_HOLDER))
    seated = np.hypot(d[0], d[1]) < SEATED_OFF and abs(d[2]) < SEATED_UP and rh[2, 2] > math.cos(UPRIGHT)
    return d, bool(seated)


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


def plug_put_down(objects) -> bool:
    """The plug is out of the port, lying on the table (its lowest point within ON_TABLE of it), and not held."""
    bounds = _box(objects["plug"], PLUG_BOX)
    return bool(unplugged(objects) and bounds[0][2] < TOP + ON_TABLE and not _held_by_any(objects, bounds))


def welder_stored(objects) -> bool:
    """The welder sits in its holder and no gripper holds it."""
    return bool(welder_in_holder(objects)[1] and not _held_by_any(objects, _box(objects["housing"], WELDER_BOX)))


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether the welder has been lifted, and how the plug came out of the
    port: held by a gripper (pulled out by hand) or not (yanked out by the cable, the welder pulled off it). Starts over
    when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.welder_lifted = False
        self.plugged, self.plug_held = True, False
        self.by_hand = self.yanked = False
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        bounds = _box(objects["housing"], WELDER_BOX)
        self.welder_lifted |= bool(bounds[0][2] > TOP + LIFTED and _held_by_any(objects, bounds))
        plugged = not unplugged(objects)
        held = _held_by_any(objects, _box(objects["plug"], PLUG_BOX))
        if self.plugged and not plugged:  # it just came out: held then (or a step before), or yanked by the cable
            if held or self.plug_held:
                self.by_hand = True
            else:
                self.yanked = True
        self.plugged, self.plug_held = plugged, held


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: step 3 once reached; 1, 2 and 4 as things stand. Steps 1 and 2 count only if a gripper held
    the plug as it came out of the port: a plug yanked out by the cable (the welder pulled away from it) never earns
    them."""
    p = _PROGRESS
    p.update(objects)
    by_hand = bool(p.by_hand and not p.yanked)
    out = bool(unplugged(objects) and by_hand)
    done = [out, bool(out and plug_put_down(objects)), bool(p.welder_lifted), welder_stored(objects)]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        d, angle = plug_in_port(objects)
        w, _ = welder_in_holder(objects)
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        print(f"[welder_unplug] t={objects['episode_time']:.2f}s  steps done: {got}  (plug {d[0] * 1000:+.1f} mm from "
              f"fully in; welder {np.hypot(w[0], w[1]) * 1000:.1f} mm from its seat, {w[2] * 1000:+.1f} mm up)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the welder, its cell or the plug off the table, the cell out of its bay (the lid opened), or a state
    that is not finite."""
    h = objects["housing"]
    for pos in (h.pos, objects["cell"].pos, objects["plug"].pos):
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    cell_in_body = quat_xyzw_to_matrix(h.quat_xyzw).T @ (objects["cell"].pos - h.pos)
    return bool(np.linalg.norm(cell_in_body - CELL_IN_BODY) > CELL_SHIFT)


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the welder and its cell still, and
    no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["welder"].resting() and objects["cell"].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from above, across the welder's middle; hold it pressed on the table while the right arm unplugs it; then
# lift it, carry it over the holder, let it drop into the pocket from just above the walls, and go home.
# Right arm: from above, across the plug's plastic body right behind the welder; pull it straight out along -y, lay it
# on the table a few cm back, let go and go home first.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width, pressing ~20 N each
WELDER_HOLD = -(WELDER_WIDTH - 1.5 * SQUEEZE) / 2  # ~30 N a finger
PLUG_HOLD = -(OVERMOLD[1] - SQUEEZE / 2) / 2  # ~10 N a finger: firmer pinches with the tapered tips and rolls the plug
WELDER_TIP_PAST_MID = 0.003  # the left pad tips this far past the housing's mid-height
PLUG_TIP_PAST_AXIS = 0.0016  # the right pad tips this far past the plug's axis
# The left gripper leans about its own finger axis, its wrist away from the plug hand (toward +y), so the two wrist
# motors (joint 5) don't meet over the welder's rear end.
LEAN = math.radians(50)
PULL = 0.03  # m: the plug pulled straight out this far, then
LAY_BACK = 0.07  # ... laid on the table with its face this far behind the port
DROP = 0.0015  # m: the welder's feet this far above the holder's walls when it is let go

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
RIGHT_GRIP = yam_demo.grip_rotation(CABLE_YAW)  # fingers across the plug's plastic body
PLUG_AXIS = _rz(CABLE_YAW) @ _X  # +y: the plug's insertion direction


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ np.array([0.0, 0.0, 1.0]))


def _welder_grip_tcp(welder_origin, yaw: float = WELDER_YAW) -> np.ndarray:
    """The left TCP gripping the welder whose frame origin (rear, bottom centre) is at ``welder_origin``, turned ``yaw``."""
    grip = np.asarray(welder_origin, float) + _rz(yaw) @ np.array([WELDER_GRIP_AT, 0.0, WELDER_HEIGHT / 2])
    return _tcp_with_tips(grip, WELDER_TIP_PAST_MID, _rz(yaw - WELDER_YAW) @ LEFT_GRIP)


def _plug_grip_tcp(face) -> np.ndarray:
    """The right TCP holding the plug whose front face is at ``face`` (pointing +y, flat)."""
    return _tcp_with_tips(np.asarray(face, float) - PLUG_GRIP_BEHIND * PLUG_AXIS, PLUG_TIP_PAST_AXIS, RIGHT_GRIP)


LEFT_GRIPPED_S = 2.5 + 1.5 + 0.8  # the left arm over, down and closed on the welder


def right_plan() -> list[Waypoint]:
    """Grip the plug behind the welder once the left hand holds it, pull it straight out, lay it down, go home."""
    rot = RIGHT_GRIP
    at = _plug_grip_tcp(PORT_START)
    out = _plug_grip_tcp(PORT_START - PULL * PLUG_AXIS)
    laid = np.array([*(PORT_START - LAY_BACK * PLUG_AXIS)[:2], TOP + OVERMOLD[2] / 2 + 0.0005])
    down = _plug_grip_tcp(laid)
    return [
        Waypoint(2.5, at + (0, 0, 0.05), FINGER_OPEN, "over the plug", rot=rot),
        Waypoint(max(LEFT_GRIPPED_S - 2.5, 1.5), at + (0, 0, 0.05), FINGER_OPEN, "wait for the left hand", rot=rot),
        Waypoint(1.5, at, FINGER_OPEN, "down to the plug", rot=rot),
        Waypoint(0.8, at, PLUG_HOLD, "grip the plug", rot=rot),
        Waypoint(1.5, out, PLUG_HOLD, "pull it out", rot=rot),
        Waypoint(1.5, down + (0, 0, 0.01), PLUG_HOLD, "back and down", rot=rot),
        Waypoint(0.8, down, PLUG_HOLD, "on the table", rot=rot),
        Waypoint(1.0, down, FINGER_OPEN, "let go", rot=rot),
        Waypoint(0.5, down, FINGER_OPEN, "fingers open", rot=rot),
        Waypoint(1.0, down + (0, 0, 0.05), FINGER_OPEN, "up", rot=rot),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan(hold_s: float) -> list[Waypoint]:
    """Grip the welder and hold it on the table while the plug comes out and the right arm goes home; then carry it over
    the holder, let it drop into the pocket from just above the walls, and go home."""
    rot = LEFT_GRIP
    at = _welder_grip_tcp(WELDER_POS)
    seat_rot = _rz(HOLDER_YAW - WELDER_YAW) @ LEFT_GRIP  # the welder turned as it is carried, to lie along the holder
    over_seat = _welder_grip_tcp(SEAT + (0.0, 0.0, HOLDER_WALL_TOP - SEAT_IN_HOLDER[2] + FEET + DROP), HOLDER_YAW)
    return [
        Waypoint(2.5, at + (0, 0, 0.06), FINGER_OPEN, "over the welder", rot=rot),
        Waypoint(1.5, at, FINGER_OPEN, "down to the welder", rot=rot),
        Waypoint(0.8, at, WELDER_HOLD, "grip the welder", rot=rot),
        Waypoint(hold_s, at, WELDER_HOLD, "hold it down", rot=rot),
        Waypoint(1.5, at + (0, 0, 0.06), WELDER_HOLD, "lift it", rot=rot),
        Waypoint(3.0, over_seat + (0, 0, 0.04), WELDER_HOLD, "over the holder, turned", rot=seat_rot),
        Waypoint(1.5, over_seat, WELDER_HOLD, "down to the walls", rot=seat_rot),
        Waypoint(1.0, over_seat, FINGER_OPEN, "let go", rot=seat_rot),
        Waypoint(0.5, over_seat, FINGER_OPEN, "fingers open", rot=seat_rot),
        Waypoint(1.0, over_seat + (0, 0, 0.06), FINGER_OPEN, "up", rot=seat_rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
        Waypoint(3.0, _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]),
    ]


def demo():
    """Both arms: the left holds the welder while the right unplugs it and goes home; then the left puts the welder in
    its holder and goes home."""
    right = right_plan()
    return {"left": left_plan(hold_s=sum(w.duration for w in right) - LEFT_GRIPPED_S + 0.5), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
