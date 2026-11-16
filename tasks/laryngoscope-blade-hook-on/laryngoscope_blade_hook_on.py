"""On a crash-cart tray: a laryngoscope handle (a medium C-cell fibre-optic handle, ISO 7376 green standard, 28 mm x 150 mm, 0.25 kg)
lies on its side with its hinge bar (4.5 mm, vertical) at the head end, and a Macintosh 3 style fibre-optic blade (130 mm, 0.08 kg)
lies in line with it, 45 mm short of the bar, its hook slot facing the bar.

The robot readies the scope for the check the way a nurse does: it steadies the round handle with one hand, slides the blade's
flared hook slot onto the hinge bar with the other, swings the blade out through a quarter turn until it clicks into the
locked, lit position (a blade at a right angle to the handle), and leaves the lit scope on the tray.

The hinge and the detent are declared effects. Every control step the task reads where the bar is in the blade's slot. Once the bar
is inside the slot (3 mm or more in, within 1 mm of its centre line, the blade flat and in line with the handle) the blade is hooked:
from then on it can only turn about the bar (its pose is written each step), and it comes off again only by being pulled straight
back off while it is still in line. A swing of 87 degrees or more clicks into the detent: the blade is held at 90 degrees, the
lamp comes on, and it comes out only if the hand pulls it back out of the detent (4 degrees in one step).

Run it:  inspect-robots run --embodiment yam_arena -E task=laryngoscope_blade_hook_on.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "scope"
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


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: a free rigid body, an articulation
    (``articulation``: its root link is free) or a fixture (``kinematic``). ``Asset``'s fields only satisfy the
    task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False
    fixed: bool = False  # an articulation whose root link is fixed to the world
    no_gravity: bool = False  # a free part with gravity off (parked out of the way)

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed)


SOLVER_ITERATIONS = 64
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
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=False, fix_root_link=True if spec.fixed else None)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
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


def _arr(t):
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each part's root pose and velocity, the grippers and the episode time, plus what ``extra_states`` adds."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            data = base.scene[name].data
            pose = _arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=_arr(data.root_lin_vel_w),
                                    ang_vel=_arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        extra_states(base, out, origin)
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus the task's reset events (``reset_events``)."""
        configure = stock_cfg_callback(control_hz)

        def with_events(env_cfg):
            env_cfg = configure(env_cfg)
            for name, func, params in reset_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="reset", params=params))
            for name, func, params in step_events():
                from isaaclab.managers import EventTermCfg

                setattr(env_cfg.events, name, EventTermCfg(func=func, mode="interval", interval_range_s=(0.0, 0.0),
                                                           is_global_time=False, params=params))
            return env_cfg

        return with_events

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
# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
ZT = 3.0 * MM  # the tray plate's thickness
TRAY_XY, TRAY_SIZE = (0.33, 0.03), (0.26, 0.40)
H_R, H_L, H_MASS = 14.0 * MM, 150.0 * MM, 0.25
HANDLE_XY = (0.28, 0.125)  # the middle of the grip; +y is the base end
BAR_LOC = np.array([30.0 * MM, -(H_L / 2 + 12.0 * MM), -3.2 * MM])  # the hinge bar in the handle frame
BLADE_L, BLADE_H, BLADE_MASS = 130.0 * MM, 20.0 * MM, 0.08
PIVOT = 6.5 * MM - 2.2525 * MM  # the bar centre depth in the slot when home: the slot floor (6.5 mm) minus the bar radius (4.505 mm bar, ISO 7376)
BLADE_DZ = -0.5 * MM  # the blade's mid-height relative to the bar's middle
SLOT_DEPTH_MIN, SLOT_LATERAL, SLOT_Z = 3.0 * MM, 1.0 * MM, 4.0 * MM
IN_LINE = math.radians(10.0)  # hooked on only with the blade this close to in line with the handle
FLAT = math.radians(5.0)
BAR_W = TOP + ZT + 0.0105  # the bar's middle height when the handle lies on the tray
GAP = 45.0 * MM  # the bar's distance outside the blade's base face at the start

HANDLE_START = np.array([HANDLE_XY[0], HANDLE_XY[1], TOP + ZT + H_R + 0.0002])
BAR_START = HANDLE_START + BAR_LOC
BLADE_START = np.array([BAR_START[0], BAR_START[1] - GAP, TOP + ZT + BLADE_H / 2 + 0.0002])
BLADE_QUAT = (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5))  # pointing along -y
TRAY = UsdPart("tray", asset="tray", xy=TRAY_XY, extent=0.2, mass=1.0, usd_path=str(ASSETS / "tray.usda"), pos=(TRAY_XY[0], TRAY_XY[1], TOP), kinematic=True)
HANDLE = UsdPart("handle", asset="handle", xy=HANDLE_XY, extent=0.15, mass=H_MASS, usd_path=str(ASSETS / "handle.usda"), pos=tuple(float(v) for v in HANDLE_START))
BLADE = UsdPart("blade", asset="blade", xy=tuple(BLADE_START[:2]), extent=0.13, mass=BLADE_MASS, usd_path=str(ASSETS / "blade.usda"),
                pos=tuple(float(v) for v in BLADE_START), rot_xyzw=BLADE_QUAT, articulation=True)
OBJECTS = [TRAY, HANDLE, BLADE]
MOVERS = ("handle", "blade")
HANDLE_LO = np.array([-H_R, -H_L / 2, -H_R])
HANDLE_HI = np.array([H_R, H_L / 2, H_R])
BLADE_LO = np.array([0.0, -0.0125, -BLADE_H / 2])
BLADE_HI = np.array([BLADE_L, 0.0125, BLADE_H / 2])

# -- the declared hinge and detent --------------------------------------------------------------------------------------
LOCK_AT = math.radians(87.0)  # a swing this far clicks into the detent, held at 90 degrees
LIT_BAND = math.radians(3.0)
UNLOCK_PULL = math.radians(4.0)  # a pull back this far out of the detent (in one control step) frees it
UNHOOK_PULL = 3.0 * MM  # pulling the blade straight back off the bar (each step more than 1 mm out of the slot, summed), while in line, takes it off
LAMP_UP = 0.0045


def _mat_quat(r):
    """xyzw quaternion of a rotation matrix."""
    t = float(r[0, 0] + r[1, 1] + r[2, 2])
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def bar_world(objects):
    h = objects["handle"]
    return np.asarray(h.pos, float) + _rot(h) @ BAR_LOC


def swing_of(objects) -> float:
    """The blade's angle from in line with the handle (rad, counter-clockwise seen from above is positive)."""
    v = _rot(objects["handle"]).T @ _rot(objects["blade"])[:, 0]
    return math.atan2(float(v[0]), -float(v[1]))


def bar_in_blade(objects):
    """The hinge bar's centre in the blade's frame: (depth into the slot, lateral offset, height offset)."""
    b = objects["blade"]
    q = _rot(b).T @ (bar_world(objects) - np.asarray(b.pos, float) - np.array([0.0, 0.0, BLADE_DZ]))
    return float(q[0]), float(q[1]), float(q[2])


class _Hinge:
    """The declared hinge and detent. Updated once per control step from the poses (the episode time); starts over when the time goes back.
    ``target`` is the blade pose to write (position, xyzw) while hooked on, else None."""

    def __init__(self):
        self.time = -1.0
        self.hooked = False
        self.locked = False
        self.phi = 0.0
        self.pull = 0.0
        self.target = None

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        self.target = None
        phi_raw = swing_of(objects)
        if not self.hooked:
            depth, lat, dz = bar_in_blade(objects)
            tilt = math.acos(max(-1.0, min(1.0, float(_rot(objects["blade"])[2, 2]))))
            if (SLOT_DEPTH_MIN <= depth <= SLOT_DEPTH_MIN + 4.0 * MM and abs(lat) <= SLOT_LATERAL and abs(dz) <= SLOT_Z
                    and abs(phi_raw) <= IN_LINE and tilt <= FLAT):
                self.hooked, self.locked, self.phi, self.pull = True, False, max(phi_raw, 0.0), 0.0
            else:
                return
        else:
            depth, _, _ = bar_in_blade(objects)
            if self.locked:
                if phi_raw <= math.pi / 2 - UNLOCK_PULL:  # pulled back out of the detent by the hand
                    self.locked, self.phi = False, max(0.0, phi_raw)
            else:
                self.phi = max(0.0, min(math.pi / 2, phi_raw))
                if self.phi >= LOCK_AT:
                    self.locked, self.phi = True, math.pi / 2
            if self.phi <= math.radians(5.0) and not self.locked:
                self.pull = max(0.0, self.pull + (PIVOT - depth)) if depth < PIVOT - 1.0 * MM else 0.0
                if self.pull >= UNHOOK_PULL:
                    self.hooked, self.pull = False, 0.0
                    return
            else:
                self.pull = 0.0
        rh = _rot(objects["handle"])
        rb = rh @ _rz(self.phi - math.pi / 2)
        pos = bar_world(objects) + np.array([0.0, 0.0, BLADE_DZ]) - rb @ np.array([PIVOT, 0.0, 0.0])
        self.target = (pos, _mat_quat(rb))

    def lit(self) -> bool:
        return bool(self.hooked and self.locked)


_HINGE = _Hinge()


def _pose_of(base, name, origin):
    p = base.scene[name].data.root_link_pose_w.torch[0].detach().cpu().numpy().astype(np.float64)
    return p[:3] - origin, p[3:]


def _write(base, name, origin, pos, quat) -> None:
    import torch

    part = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *np.asarray(quat, float)]], device=part.device, dtype=torch.float32)
    part.write_root_pose_to_sim_index(root_pose=pose)
    part.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=part.device))


def _write_targets(art, names, values) -> None:
    """Writes joint drive targets (``names`` order) to PhysX directly: Arena gives task objects no actuators."""
    import torch

    view = art.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    dofs = list(view.shared_metatype.dof_names)
    cols = [dofs.index(n) for n in names]
    t[:, cols] = torch.as_tensor(np.asarray(values, np.float32), device=t.device)
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, art._ALL_INDICES)


_SHOWN = [None]


def _objs(base, origin) -> dict:
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for n in MOVERS:
        pos, quat = _pose_of(base, n, origin)
        out[n] = ObjectState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())
    return out


def hinge_step(base) -> None:
    """Once per control step: updates the hinge from the poses, holds a hooked blade on the bar (writes its pose) and lights the lamp."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    objs = _objs(base, origin)
    t = objs["episode_time"]
    if t < _HINGE.time:
        _SHOWN[0] = None
    if t == _HINGE.time:
        return
    _HINGE.update(objs)
    if _HINGE.target is not None:
        _write(base, "blade", origin, *_HINGE.target)
    lamp = LAMP_UP if _HINGE.lit() else 0.0
    if _SHOWN[0] != lamp:
        _write_targets(base.scene["blade"], ["Lamp"], [lamp])
        _SHOWN[0] = lamp


def _hinge_step(env, env_ids, asset_cfg=None) -> None:
    hinge_step(env.unwrapped if hasattr(env, "unwrapped") else env)


def _hinge_reset(env, env_ids, asset_cfg=None) -> None:
    _HINGE.__init__()
    _SHOWN[0] = None


def extra_states(base, out, origin) -> None:
    pass


def reset_events() -> list:
    return [("hinge_reset", _hinge_reset, {})]


def step_events() -> list:
    return [("hinge_step", _hinge_step, {})]


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
# -- the rule and what is scored ----------------------------------------------------------------------------------------
SWUNG = math.radians(45.0)
CLEAR = 0.03
TRAY_HALF = (TRAY_SIZE[0] / 2, TRAY_SIZE[1] / 2)
STEPS = ("the blade hooked on the handle's hinge bar", "blade swung 45 degrees or more", "blade clicked into the locked position, lamp lit",
         "lit scope lying on the tray, let go, grippers clear")


def on_tray(objects) -> bool:
    """Both parts' middles over the tray, the blade's tip too."""
    h, b = np.asarray(objects["handle"].pos, float), np.asarray(objects["blade"].pos, float)
    tip = b + _rot(objects["blade"])[:, 0] * BLADE_L
    for p in (h, b, tip):
        if abs(p[0] - TRAY_XY[0]) > TRAY_HALF[0] or abs(p[1] - TRAY_XY[1]) > TRAY_HALF[1]:
            return False
    return bool(h[2] < TOP + ZT + 0.03 and b[2] < TOP + ZT + 0.03)


def lit_at_detent(objects) -> bool:
    """Hooked, locked, and the blade really at the detent within the lit band (the lamp's contact closes only there)."""
    _HINGE.update(objects)
    return bool(_HINGE.lit() and abs(swing_of(objects) - math.pi / 2) <= LIT_BAND)


def parts_free(objects) -> bool:
    if any(holds(objects, s, "handle", HANDLE_LO, HANDLE_HI) or holds(objects, s, "blade", BLADE_LO, BLADE_HI) for s in ("left", "right")):
        return False
    return not (pads_near(objects, "handle", HANDLE_LO, HANDLE_HI, CLEAR) or pads_near(objects, "blade", BLADE_LO, BLADE_HI, CLEAR))


def _steps(objects) -> list[bool]:
    _HINGE.update(objects)
    s1 = bool(_HINGE.hooked)
    s2 = bool(s1 and swing_of(objects) >= SWUNG)
    s3 = bool(s2 and lit_at_detent(objects))
    s4 = bool(s3 and on_tray(objects) and parts_free(objects))
    return [s1, s2, s3, s4]


def events_now(objects, p) -> list[str]:
    _HINGE.update(objects)
    out = []
    for n in MOVERS:
        if objects[n].pos[2] < TOP - 0.03:
            out.append(f"the {n} fell off the table")
    return out


def trace_line(objects) -> str:
    depth, lat, dz = bar_in_blade(objects)
    return (f"bar {depth * 1000:.1f} mm into the slot (off {lat * 1000:.1f}, dz {dz * 1000:.1f}), swing {math.degrees(swing_of(objects)):.0f} deg, "
            f"hooked {_HINGE.hooked}, locked {_HINGE.locked}, on tray {on_tray(objects)}")


# -- scoring (shared machinery) ---------------------------------------------------------------------------------------
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
POINTS = 0.2  # per step; only success scores 1.0
GRIP_SLACK = 0.004  # held: a pad's point within this of the part's bounds ...
GRIP_FORCE = 0.5  # ... both fingers pressing with at least this (N) ...
GRIP_OPENING = 0.003  # ... and the pads apart by at least this (not closed on nothing)


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def holds(objects, side: str, name: str, lo, hi) -> bool:
    """The ``side`` gripper holds the part: a point along its pads is within GRIP_SLACK of the part's bounds (``lo``,
    ``hi``, in the part's frame), both fingers press with GRIP_FORCE and the pads are apart by GRIP_OPENING."""
    g = objects.get(f"{side}_gripper")
    if g is None or g.contact < GRIP_FORCE or g.opening < GRIP_OPENING:
        return False
    s = objects[name]
    q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
    return bool(np.any(np.all((q >= np.asarray(lo) - GRIP_SLACK) & (q <= np.asarray(hi) + GRIP_SLACK), axis=1)))


def pads_near(objects, name: str, lo, hi, margin: float) -> bool:
    """Either gripper's pads are within ``margin`` of the part's bounds."""
    s = objects[name]
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        q = (_pads(g) - np.asarray(s.pos, float)) @ _rot(s)
        if np.any(np.all((q >= np.asarray(lo) - margin) & (q <= np.asarray(hi) + margin), axis=1)):
            return True
    return False


def _poses(objects) -> tuple:
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in MOVERS)


def still(objects, anchor) -> bool:
    """Every part within STILL_MOVE and STILL_TURN of its pose in ``anchor``. Judged from the poses, not from the
    simulator's instantaneous speeds."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


class _Progress:
    """What has happened so far this episode, updated once per control step: the task's own memory (``mem``), every
    rule violation (``events``, they last for the whole episode), and since when every full-success condition has held
    without a break (``met_since``, ``None`` while one fails) with the poses when they began to hold (``anchor``).
    Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.mem: dict = {}
        self.events: list[str] = []
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def flag(self, t, what: str) -> None:
        if what not in self.events:
            self.events.append(what)
            print(f"[{TAG}] t={t:.2f}s  RULE BROKEN: {what} (scores 0 from here on)", flush=True)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        stamp = -1.0 if t is None else t
        for what in events_now(objects, self):
            self.flag(stamp, what)
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not self.events)
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


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand (see ``_steps``)."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):
        line = f"({trace_line(objects)}; rule events {len(p.events)})"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: any rule event this episode (kept for the whole run), or a state that is not finite. Reads the
    episode's progress without updating it."""
    values = [v for n in MOVERS for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.events)


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all steps done, no rule event so far, and the parts within STILL_MOVE and STILL_TURN of where they were when
    that second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise POINTS per step done (``STEPS``, at
    most 0.8, also while the second is being timed); 0 after any rule event."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)
# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left arm takes the handle from above by its base end (pads closing along x, across the grip) and holds it down. The right arm
# takes the blade by its middle, slides its hook slot onto the hinge bar, swings it out through two 45 degree moves (wrist yaw follows
# the swing, the hand moves on an arc about the bar) until it clicks into the lit position, lets go and lifts away. Then the left arm
# lets go and both go home.
_DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
YAW0 = math.pi / 2  # pads close along x at yaw0 (gripper's x axis is the closing axis)


def _grip(yaw):
    return yam_demo.grip_rotation(yaw + YAW0)


OPEN = FINGER_OPEN
HANDLE_HOLD = -(2 * H_R - 0.005) / 2
BLADE_HOLD = -(0.020 - 0.005) / 2
LEFT_Y = 0.17  # where the left hand takes the handle
GRIP_X_B = 0.060  # the right hand takes the blade this far from its base face
RADIUS = GRIP_X_B - PIVOT  # that point's distance from the bar when hooked
HAND_TIP_Z = ZT + 0.0125  # pad tips above the table at the blade (blade top at ZT + 0.020)
LEFT_TIP_Z = ZT + 0.0125
LEFT_AIM = (0.0, 0.0, 0.0)
RIGHT_AIM = (0.0, 0.0, 0.0)  # trim of the right hand's aim (m), on top of PAD_SHIFT
PAD_SHIFT = 0.0  # the pads' middle sits this far from the TCP along the gripper's closing axis at yaw -90 (turns with the wrist)
OVER = 0.07
SLIDE_MORE = 0.004
HOOK_YAW = 0.0  # the wrist's yaw trim while taking and sliding the blade (it drifts about 10 degrees ccw while sliding)


def _tcp(x, y, tip_z, aim):
    return np.array([x + aim[0], y + aim[1], TOP + tip_z + PAD_TIP_PAST_TCP + aim[2]])


def _blade_pt(phi, aim=RIGHT_AIM, extra=0.0):
    """The right TCP when the blade (hooked) is swung ``phi`` rad: on an arc of radius RADIUS about the bar."""
    return _tcp(BAR_START[0] + (RADIUS + extra) * math.sin(phi) + PAD_SHIFT * math.cos(phi),
                BAR_START[1] - (RADIUS + extra) * math.cos(phi) + PAD_SHIFT * math.sin(phi), HAND_TIP_Z, aim)


def right_plan():
    start = BLADE_START[1] - GRIP_X_B  # y of the grip point at the start (blade pointing -y)
    p0 = _tcp(BAR_START[0] + PAD_SHIFT, start, HAND_TIP_Z, RIGHT_AIM)
    r0 = _grip(0.0)
    hooked = _blade_pt(0.0, extra=0.0) + np.array([0.0, SLIDE_MORE, 0.0])
    wps = [
        Waypoint(2.5, p0 + (0, 0, OVER), OPEN, "over the blade", rot=r0),
        Waypoint(1.5, p0, OPEN, "pads astride the blade", rot=r0),
        Waypoint(0.8, p0, BLADE_HOLD, "grip the blade", rot=r0),
        Waypoint(4.0, hooked, BLADE_HOLD, "slide the hook slot onto the bar", rot=r0),
        Waypoint(1.0, hooked, BLADE_HOLD, "hooked", rot=r0),
    ]
    n = 8
    half = math.pi / 4
    for i in range(1, n + 1):  # the first 45 degrees: yaw follows the swing from +90 degrees
        phi = half * i / n
        wps.append(Waypoint(3.0 / n, _blade_pt(phi), BLADE_HOLD, "swing the blade out (first 45 degrees)", rot=_grip(phi)))
    mid = _blade_pt(half)
    # the wrist cannot turn on past about +165 degrees: let go (the hinge holds the blade), turn the wrist the other way round and grip again
    wps += [
        Waypoint(0.6, mid, OPEN, "let go", rot=_grip(half)),
        Waypoint(1.0, mid + (0, 0, OVER), OPEN, "up", rot=_grip(half)),
        Waypoint(3.0, mid + (0, 0, OVER), OPEN, "turn the wrist round", rot=_grip(half - math.pi)),
        Waypoint(1.2, mid, OPEN, "pads astride the blade again", rot=_grip(half - math.pi)),
        Waypoint(0.8, mid, BLADE_HOLD, "grip again", rot=_grip(half - math.pi)),
    ]
    for i in range(1, n + 1):
        phi = half + half * i / n
        wps.append(Waypoint(3.0 / n, _blade_pt(phi), BLADE_HOLD, "swing the blade out (second 45 degrees)", rot=_grip(phi - math.pi)))
    end = _blade_pt(math.pi / 2)
    wps += [
        Waypoint(1.5, end, BLADE_HOLD, "click", rot=_grip(math.pi / 2 - math.pi)),
        Waypoint(0.8, end, OPEN, "let go", rot=_grip(math.pi / 2 - math.pi)),
        Waypoint(1.4, end + (0, 0, OVER), OPEN, "lift away", rot=_START_ROT["right"]),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return wps


def left_plan(wait_until: float):
    g = _tcp(HANDLE_XY[0], LEFT_Y, LEFT_TIP_Z, LEFT_AIM)
    r0 = _grip(0.0)
    wps = [
        Waypoint(2.5, g + (0, 0, OVER), OPEN, "over the handle", rot=r0),
        Waypoint(1.5, g, OPEN, "pads astride the handle", rot=r0),
        Waypoint(0.8, g, HANDLE_HOLD, "grip the handle", rot=r0),
    ]
    held = sum(w.duration for w in wps)
    wps.append(Waypoint(max(wait_until - held, 0.1), g, HANDLE_HOLD, "hold the handle", rot=r0))
    wps += [
        Waypoint(0.8, g, OPEN, "let go", rot=r0),
        Waypoint(1.4, g + (0, 0, OVER), OPEN, "lift away", rot=r0),
        Waypoint(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps


def demo():
    right = right_plan()
    # the right hand lets go at: grip + slide + hook + swing + click
    t_release = sum(w.duration for w in right[:-4])
    return {"left": left_plan(wait_until=t_release + 2.0), "right": right}
