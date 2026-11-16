"""On the station table: a bench rack with a freshly cut piece of 22 x 0.9 mm copper tube (100 mm, 0.053 kg) standing in it, open
end up; and, to the right, a hand deburring reamer (an inner-outer reamer with a conical blade, 0.090 kg) standing cone-down in
its rest. The cut left a 0.4 mm burr round the inside of the tube's end.

The robot has to ream the burr off the way a plumber does: lift the tube out of the rack and hold it upright in one hand,
lower the reamer's cone into the bore and turn it clockwise in strokes (the wrist turns about 120 degrees a stroke, so the hand
lets go, turns back and takes a fresh grip between strokes), stop once the burr is gone all round (further turning chamfers the
thin wall), lift the reamer out and stand it back in its rest, and stand the tube back in the rack.

The burr and the chamfer are a declared effect, not simulated material. Every control step the task reads where the reamer's
cone is in the tube's bore and how far the reamer has turned. While the cone is seated (its tip well inside the bore, the axis
within a few degrees of the tube's, a hand holding the tube against the torque) and the reamer turns clockwise, its two blades
pass the 12 sectors of the bore; each pass takes 0.2 mm off the burr of the sector it crosses (two passes clear it), and a pass
over a sector that is already clear chamfers its rim by 0.2 mm. Turning back, or turning while the cone is not seated, cuts
nothing. The burr layers and the rim chamfer are drawn as marker links on the tube (an articulation) that slide away.

Run it:  inspect-robots run --embodiment yam_arena -E task=copper_tube_deburr_reamer.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "reamer"
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
VEL_ITERATIONS = 8
MAX_DEPENETRATION = 0.2  # m/s
SETTLE_LIN, SETTLE_ANG = 1.0, 5.0  # damping of the two loose parts (1/s): without it they rock on against the rim of their oversize holders in the simulator


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
                                                             solver_position_iteration_count=SOLVER_ITERATIONS,
                                                             solver_velocity_iteration_count=VEL_ITERATIONS,
                                                             linear_damping=SETTLE_LIN,
                                                             angular_damping=SETTLE_ANG,
                                                             disable_gravity=True if spec.no_gravity else None)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=VEL_ITERATIONS,
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
TUBE_OD, TUBE_WALL, TUBE_L = 22.0 * MM, 0.9 * MM, 100.0 * MM  # EN 1057 22 x 0.9 mm, a 100 mm cut piece
TUBE_RI = TUBE_OD / 2 - TUBE_WALL  # 10.1 mm
TUBE_MASS = 0.053
TUBE_XY = (0.27, -0.05)  # the rack's bore axis
RACK_PLATE, RACK_TOP = 4.0 * MM, 20.0 * MM
TUBE_Z0 = RACK_PLATE + 0.3 * MM  # the tube's lower end in the rack, above the table
REST_XY = (0.22, -0.20)
REST_TOP, REST_BORE_R = 18.0 * MM, 9.0 * MM
CONE_TIP_R, CONE_BASE_R, CONE_H = 4.0 * MM, 13.0 * MM, 14.0 * MM
SLOPE = (CONE_BASE_R - CONE_TIP_R) / CONE_H
TOOL_L, TOOL_MASS = 72.0 * MM, 0.090
HANDLE_R, HANDLE_BOT = 15.0 * MM, 22.0 * MM  # the handle's radius and where it starts above the cone's tip
CUP_R = 22.5 * MM
TOOL_Z0 = REST_TOP - (REST_BORE_R - CONE_TIP_R) / SLOPE + 0.5 * MM  # the cone's tip in its rest (resting on the bore's rim)
SEAT_DEPTH = (TUBE_RI - CONE_TIP_R) / SLOPE  # the cone's tip this far below the tube's rim when it rests on the bore's edge (9.5 mm)
TUBE_LIFT = 0.025  # the held tube is raised this far out of the rack

TUBE_START = np.array([TUBE_XY[0], TUBE_XY[1], TOP + TUBE_Z0])
TOOL_START = np.array([REST_XY[0], REST_XY[1], TOP + TOOL_Z0])
RACK = UsdPart("rack", asset="rack", xy=TUBE_XY, extent=0.09, mass=0.6, usd_path=str(ASSETS / "tube_rack.usda"),
               pos=(TUBE_XY[0], TUBE_XY[1], TOP), kinematic=True)
REST = UsdPart("rest", asset="rest", xy=REST_XY, extent=0.09, mass=0.4, usd_path=str(ASSETS / "reamer_rest.usda"),
               pos=(REST_XY[0], REST_XY[1], TOP), kinematic=True)
TUBE = UsdPart("tube", asset="tube", xy=TUBE_XY, extent=0.10, mass=TUBE_MASS, usd_path=str(ASSETS / "copper_tube.usda"),
               pos=tuple(float(v) for v in TUBE_START), articulation=True)
TOOL = UsdPart("tool", asset="tool", xy=REST_XY, extent=0.072, mass=TOOL_MASS, usd_path=str(ASSETS / "reamer.usda"),
               pos=tuple(float(v) for v in TOOL_START))
OBJECTS = [RACK, REST, TUBE, TOOL]
MOVERS = ("tube", "tool")
TUBE_LO = np.array([-TUBE_OD / 2, -TUBE_OD / 2, 0.0])
TUBE_HI = np.array([TUBE_OD / 2, TUBE_OD / 2, TUBE_L])
TOOL_LO = np.array([-HANDLE_R, -HANDLE_R, HANDLE_BOT])
TOOL_HI = np.array([HANDLE_R, HANDLE_R, TOOL_L])

# -- the declared cutting effect ---------------------------------------------------------------------------------------
N_SECT = 12
TURN = 2 * math.pi
SECT_C = np.radians(-165.0 + 30.0 * np.arange(N_SECT))  # sector middles in the tube's frame
BURR = 0.4 * MM  # the inner burr (a lip of about 0.4 mm: typical of a wheel cutter)
BITE = 0.2 * MM  # what one blade pass takes off a sector's burr (two passes clear it)
CHAMFER_PASS = 0.2 * MM  # what a blade pass over a clear sector takes off its rim
THIN = 0.5 * MM  # a rim chamfered past this has thinned the 0.9 mm wall too far
SEAT_MIN = 6.0 * MM  # seated: the cone's tip at least this far below the rim (rest: 9.5 mm) ...
SEAT_OFF = 3.0 * MM  # ... its axis within this of the tube's at the tip ...
SEAT_TILT = math.radians(8.0)  # ... and within this angle of the tube axis
COMMIT = math.radians(2.0)  # turning less than this is not counted (sensor jitter, a hand easing back)
JOINT_NAMES = tuple(f"{k}{i}" for i in range(N_SECT) for k in ("BurrA", "BurrB", "Bevel"))


def _tool_in_tube(objects):
    """The reamer's cone tip in the tube's frame: (depth below the rim, offset from the axis, axis tilt, turn angle about it)."""
    t, r = objects["tube"], objects["tool"]
    rt, rr = _rot(t), _rot(r)
    p = rt.T @ (np.asarray(r.pos, float) - np.asarray(t.pos, float))
    rel = rt.T @ rr
    return (TUBE_L - float(p[2]), float(math.hypot(p[0], p[1])), math.acos(max(-1.0, min(1.0, float(rel[2, 2])))),
            math.atan2(float(rel[1, 0]), float(rel[0, 0])))


def seated(objects) -> bool:
    depth, off, tilt, _ = _tool_in_tube(objects)
    return bool(depth >= SEAT_MIN and off <= SEAT_OFF and tilt <= SEAT_TILT)


def tube_held(objects) -> bool:
    return any(holds(objects, s, "tube", TUBE_LO, TUBE_HI) for s in ("left", "right"))


class _Cut:
    """The declared cutting effect and what has happened so far: blade passes per sector, the reamer's unwrapped turn angle and
    the angle up to which turning has been counted. Updates once per control step (the episode time); starts over when the time
    goes back."""

    def __init__(self):
        self.time = -1.0
        self.passes = np.zeros(N_SECT, int)
        self.last = None
        self.u = 0.0
        self.commit = None
        self.cutting = False
        self.shown = None
        self.last_rate = 0.0  # the reamer's turn rate about the tube's axis (rad/s, clockwise negative)
        self.drag = False  # the cone is seated and turning clockwise: the blade's torque acts on the tube

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        dt = None
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            elif self.time >= 0.0:
                dt = t - self.time
            self.time = t
        _, _, _, phi = _tool_in_tube(objects)
        self.last_rate = 0.0
        if self.last is not None:
            step = (phi - self.last + math.pi) % TURN - math.pi
            self.u += step
            if dt:
                self.last_rate = step / dt
        else:
            self.u = phi
        self.last = phi
        ok = seated(objects) and tube_held(objects)
        self.drag = bool(seated(objects) and self.last_rate < -DRAG_RATE)
        if not ok or self.commit is None or not self.cutting:
            self.commit = self.u
        else:
            d = self.u - self.commit
            if d <= -COMMIT:
                self._sweep(self.commit, self.u)
                self.commit = self.u
            elif d >= COMMIT:
                self.commit = self.u
        self.cutting = ok

    def _sweep(self, hi: float, lo: float) -> None:
        """Both blades (half a turn apart) sweep clockwise from angle ``hi`` down to ``lo``: each sector middle they cross is passed."""
        for b in (0.0, math.pi):
            for i, c in enumerate(SECT_C):
                v = c + math.floor((hi + b - c) / TURN) * TURN
                if v > lo + b:
                    self.passes[i] += 1

    def burr(self) -> np.ndarray:
        return np.clip(BURR - BITE * self.passes, 0.0, None)

    def chamfer(self) -> np.ndarray:
        return CHAMFER_PASS * np.clip(self.passes - round(BURR / BITE), 0, None)

    def targets(self) -> list[float]:
        """Marker slide targets (JOINT_NAMES order, m): burr layer A is gone after one pass, B after two, a rim mark shows once
        the sector's rim is chamfered."""
        out = []
        for n in self.passes:
            out += [0.0005 if n >= 1 else 0.0, 0.0005 if n >= 2 else 0.0, 0.0004 if n >= 3 else 0.0]
        return out


_CUT = _Cut()


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


_DRAG = {"tube": None, "tool": None}


def _apply_drag(art, on: bool, key: str = "tube") -> None:
    """The cut's resisting torque: a steady torque about the world's vertical on the tube while the cone is seated and the reamer
    turns clockwise (the blade drags the tube round). The hand holding the tube has to react it; an unheld tube turns."""
    if _DRAG[key] is on or not hasattr(art, "permanent_wrench_composer"):  # (the MuJoCo stand-in has no torque)
        return
    import torch

    tq = torch.zeros((1, 1, 3), dtype=torch.float32, device=art.device)
    if on:
        tq[0, 0, 2] = -DRAG_TORQUE if key == "tube" else DRAG_TORQUE  # equal and opposite: the tube is dragged clockwise, the reamer resisted
    art.permanent_wrench_composer.set_forces_and_torques_index(torques=tq, body_ids=[0], env_ids=torch.tensor([0], device=art.device),
                                                               is_global=True)
    _DRAG[key] = on


def extra_states(base, out, origin) -> None:
    """Advances the cutting effect and moves the burr and chamfer markers on the tube (once every control step)."""
    _CUT.update(out)
    _apply_drag(base.scene["tube"], _CUT.drag, "tube")
    _apply_drag(base.scene["tool"], _CUT.drag, "tool")
    key = tuple(_CUT.targets())
    if _CUT.shown != key:
        _write_targets(base.scene["tube"], list(JOINT_NAMES), list(key))
        _CUT.shown = key


def _reset_markers(env, env_ids, asset_cfg) -> None:
    """Reset event: every marker back to its start (burr in place, no chamfer mark), the cutting effect started over."""
    import torch

    if env_ids is None:
        return
    art = env.scene[asset_cfg.name]
    ids = art.find_joints(list(JOINT_NAMES), preserve_order=True)[0]
    zero = torch.zeros((len(env_ids), len(ids)), device=art.device)
    art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
    _write_targets(art, list(JOINT_NAMES), [0.0] * len(JOINT_NAMES))
    _DRAG["tube"] = _DRAG["tool"] = None
    _apply_drag(art, False, "tube")
    _apply_drag(env.scene["tool"], False, "tool")
    _CUT.__init__()
    _CUT.shown = tuple([0.0] * len(JOINT_NAMES))


def reset_events() -> list:
    from isaaclab.managers import SceneEntityCfg

    return [("tube_markers_reset", _reset_markers, {"asset_cfg": SceneEntityCfg("tube")})]


def step_events() -> list:
    return []


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
RACK_OFF, RACK_TILT, RACK_Z = 0.004, math.radians(20.0), 0.004  # tube back in the rack: lower end within 4 mm of the bore axis and 4 mm of its seat, leaning under 20 degrees (it stands loose in a 27 mm bore)
KNOCKED = math.radians(60.0)  # a tube tipped more than this has been dropped or knocked over
CLEAR = 0.03
STEPS = ("the reamer's cone seated in the bore of the held tube", "burr at least half removed all round",
         "burr removed all round, rim not chamfered past 0.5 mm",
         "tube back upright in its rack, reamer out of the bore and let go, grippers clear")


def _tilt_of(objects, name):
    return math.acos(max(-1.0, min(1.0, float(_rot(objects[name])[2, 2]))))


def tube_in_rack(objects) -> bool:
    p = np.asarray(objects["tube"].pos, float)
    return bool(math.hypot(p[0] - TUBE_XY[0], p[1] - TUBE_XY[1]) <= RACK_OFF and p[2] <= TOP + TUBE_Z0 + RACK_Z
                and _tilt_of(objects, "tube") <= RACK_TILT)


def tool_out_of_bore(objects) -> bool:
    depth, off, _, _ = _tool_in_tube(objects)
    return not (off < 0.02 and depth > -0.02)


def parts_free(objects) -> bool:
    """Neither part is held and no pad is within CLEAR of either."""
    if tube_held(objects) or any(holds(objects, s, "tool", TOOL_LO, TOOL_HI) for s in ("left", "right")):
        return False
    return not (pads_near(objects, "tube", TUBE_LO, TUBE_HI, CLEAR) or pads_near(objects, "tool", TOOL_LO, TOOL_HI, CLEAR))


def thin(objects) -> bool:
    _CUT.update(objects)
    return bool(_CUT.chamfer().max() > THIN + 1e-9)


def _steps(objects) -> list[bool]:
    _CUT.update(objects)
    s1 = bool(_PROGRESS.mem.get("seated_up"))
    s2 = bool(s1 and np.all(_CUT.passes >= 1))
    s3 = bool(s2 and np.all(_CUT.passes >= 2) and not thin(objects))
    s4 = bool(s3 and tube_in_rack(objects) and tool_out_of_bore(objects) and parts_free(objects))
    return [s1, s2, s3, s4]


def events_now(objects, p) -> list[str]:
    """Rule events as things stand. Also remembers (``p.mem``) that the cone was seated in the lifted tube."""
    _CUT.update(objects)
    out = []
    if seated(objects) and tube_held(objects):
        p.mem["seated_up"] = True
    for n in MOVERS:
        if objects[n].pos[2] < TOP - 0.03:
            out.append(f"the {n} fell off the table")
    if _tilt_of(objects, "tube") > KNOCKED:
        out.append("the tube was dropped or knocked over")
    return out


def _yaw_deg(objects, name) -> float:
    x, y, z, w = (float(v) for v in objects[name].quat_xyzw)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def trace_line(objects) -> str:
    depth, off, tilt, phi = _tool_in_tube(objects)
    tp = np.asarray(objects["tube"].pos, float)
    return (f"tube {(tp[2] - TOP - TUBE_Z0) * 1000:.0f} mm up{'*' if tube_held(objects) else ''} (lean {math.degrees(_tilt_of(objects, 'tube')):.1f} deg, "
            f"{(tp[0] - TUBE_XY[0]) * 1000:+.1f}, {(tp[1] - TUBE_XY[1]) * 1000:+.1f} mm x, y off the rack axis, yaw {_yaw_deg(objects, 'tube'):.0f} deg), cone {depth * 1000:.1f} mm below the rim "
            f"off {off * 1000:.1f} mm tilt {math.degrees(tilt):.1f} deg turn {math.degrees(_CUT.u):.0f} deg, passes {int(_CUT.passes.min())}-{int(_CUT.passes.max())}, "
            f"burr {_CUT.burr().max() * 1000:.2f} mm, chamfer {_CUT.chamfer().max() * 1000:.2f} mm, seated {seated(objects)}, in rack {tube_in_rack(objects)}")


# -- scoring (shared machinery) ---------------------------------------------------------------------------------------
DRAG_TORQUE = 0.08  # N m: the resisting torque of the cut: +0.08 on the reamer (against its turning) and -0.08 on the tube (dragged round).
# An estimate from the burr's shear, not a measurement: shear strength of annealed to half-hard copper 150-230 MPa (taken as 200 MPa),
# chip section per blade = 0.2 mm bite x 0.1 mm of ragged lip engaged at a time = 2e-8 m2, so 4 N per blade at the 10 mm bore radius;
# two blades cut at once: 2 x 4 N x 0.010 m = 0.08 N m. It is about half of what the demo's two-pad side grip can hold
# (2 pads x 0.5 friction x 15 N x 11 mm = 0.17 N m) and several times what the tube's own friction on the rack plate can hold
# (0.017 N m), so an unheld tube turns with the reamer.
DRAG_RATE = 0.2  # rad/s: slower turning than this is not cutting (no drag)
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
# The left arm side-grips the tube (pointing along -y, pads closing along x) and lifts it out of the rack. The right arm takes
# the reamer from its rest, lowers its cone into the bore and turns it clockwise in four strokes of 120 degrees (lets go, turns
# its wrist back and grips again between strokes), lifts it out, stands it back in its rest and goes home; then the left arm
# stands the tube back in the rack.
_X, _Y, _Z = np.eye(3)
_DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
_grip = yam_demo.grip_rotation


def side_grip(approach: float, roll: float = 0.0) -> np.ndarray:
    """The gripper pointing horizontally along ``approach`` (rad from +x), fingers closing horizontally at roll 0."""
    forward = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN  # pointing +x, fingers closing along y
    return axis_angle_matrix(_Z, approach) @ axis_angle_matrix(_X, roll) @ forward


LEFT_ROT = side_grip(-math.pi / 2)  # pointing along -y, closing along x
SQUEEZE_TUBE = 0.006
TUBE_HOLD = -(TUBE_OD - SQUEEZE_TUBE) / 2  # about 15 N a finger
TOOL_HOLD = -(2 * HANDLE_R - 0.008) / 2  # an 8 mm squeeze: the reamer turns against the cut torque, the grip must not slip
OPEN = FINGER_OPEN
TUBE_GRIP_H = 0.084  # the pads middle above the tube lower end (a level gripper cannot reach much below 0.085 m)
TOOL_GRIP_H = 0.030  # the pad tips above the cone's tip (the handle starts at 22 mm)
AIM_X = 0.0  # arm sag trims (m)
AIM_Y = 0.0
AIM_Z = 0.0
YAW0 = 1.8  # the right wrist's yaw at the start of each stroke (rad); strokes turn it clockwise (falling yaw)
STROKE = math.radians(120.0)
STROKES = 4
STROKE_S = 4.0
STROKE_WP = 8
PUSH = 0.0008  # the right hand aims this far below the cone's rest height
LEFT_AIM = (0.0, 0.0, 0.0)
LIFT_CMD = TUBE_LIFT + 0.009  # the left arm sags about 9 mm more when raised: command this much more lift


LOWER_AIM = (0.0023, 0.0025)  # the held tube hangs about 4 mm toward -x of the grip command: lower it this far over


def _tube_tcp(lift: float = 0.0, back: float = 0.0, aim=(0.0, 0.0)):
    """The left TCP gripping the tube at its rack position raised by ``lift``; ``back`` further out along +y."""
    return np.array([TUBE_XY[0] + LEFT_AIM[0] + aim[0], TUBE_XY[1] + PAD_TIP_PAST_TCP - 0.002 + back + LEFT_AIM[1] + aim[1],
                     TOP + TUBE_Z0 + TUBE_GRIP_H + lift + LEFT_AIM[2]])


def _tool_tcp(x, y, tip_z):
    return np.array([x + AIM_X, y + AIM_Y, tip_z + TOOL_GRIP_H + PAD_TIP_PAST_TCP + AIM_Z])


TOOL_REST_TCP = _tool_tcp(REST_XY[0], REST_XY[1], TOP + TOOL_Z0)
SEAT_TIP_Z = TOP + TUBE_Z0 + TUBE_LIFT + TUBE_L - SEAT_DEPTH  # the cone's tip when it rests on the bore's edge


def right_plan():
    """Take the reamer, ream in strokes, put it back, go home. Also returns when the reamer is clear of the tube and the
    time the right hand has left the tube's area."""
    over_rest = TOOL_REST_TCP + (0, 0, 0.06)
    hov = _tool_tcp(TUBE_XY[0], TUBE_XY[1], SEAT_TIP_Z + 0.012)
    seat = _tool_tcp(TUBE_XY[0], TUBE_XY[1], SEAT_TIP_Z - PUSH)
    r0 = _grip(YAW0)
    wps = [
        Waypoint(2.5, over_rest, OPEN, "over the reamer", rot=r0),
        Waypoint(1.5, TOOL_REST_TCP, OPEN, "pads astride the handle", rot=r0),
        Waypoint(0.8, TOOL_REST_TCP, TOOL_HOLD, "grip the reamer", rot=r0),
        Waypoint(1.2, TOOL_REST_TCP + (0, 0, 0.05), TOOL_HOLD, "lift it out of its rest", rot=r0),
        Waypoint(3.0, hov + (0, 0, 0.010), TOOL_HOLD, "over the tube", rot=r0),
        Waypoint(1.2, hov, TOOL_HOLD, "cone over the bore", rot=r0),
        Waypoint(1.2, seat, TOOL_HOLD, "cone into the bore", rot=r0),
        Waypoint(2.0, seat, TOOL_HOLD, "settle", rot=r0),
    ]
    for k in range(STROKES):
        for i in range(1, STROKE_WP + 1):
            wps.append(Waypoint(STROKE_S / STROKE_WP, seat, TOOL_HOLD, f"stroke {k + 1}", rot=_grip(YAW0 - STROKE * i / STROKE_WP)))
        if k == STROKES - 1:
            break
        end = _grip(YAW0 - STROKE)
        wps += [
            Waypoint(0.6, seat, OPEN, "let go", rot=end),
            Waypoint(1.6, seat, OPEN, "turn the wrist back", rot=r0),
            Waypoint(0.6, seat, TOOL_HOLD, "grip again", rot=r0),
        ]
    wps += [
        Waypoint(0.5, seat, TOOL_HOLD, "hold", rot=_grip(YAW0 - STROKE)),
        Waypoint(1.2, hov + (0, 0, 0.010), TOOL_HOLD, "lift the reamer out", rot=_grip(YAW0 - STROKE)),
    ]
    out_t = sum(w.duration for w in wps)
    wps += [
        Waypoint(3.0, over_rest, TOOL_HOLD, "over its rest", rot=r0),
        Waypoint(1.5, TOOL_REST_TCP + (0, 0, 0.004), TOOL_HOLD, "into the rest", rot=r0),
        Waypoint(0.8, TOOL_REST_TCP + (0, 0, 0.004), OPEN, "let go", rot=r0),
        Waypoint(1.0, over_rest, OPEN, "up", rot=r0),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return wps, out_t


def left_plan(wait_until: float):
    t0 = _tube_tcp()
    wps = [
        Waypoint(2.5, _tube_tcp(0.0, 0.07), OPEN, "facing the tube", rot=LEFT_ROT),
        Waypoint(1.5, t0, OPEN, "pads astride the tube", rot=LEFT_ROT),
        Waypoint(0.8, t0, TUBE_HOLD, "grip the tube", rot=LEFT_ROT),
        Waypoint(1.5, _tube_tcp(LIFT_CMD), TUBE_HOLD, "lift it out of the rack", rot=LEFT_ROT),
    ]
    held = sum(w.duration for w in wps)
    wps.append(Waypoint(max(wait_until - held, 0.1), _tube_tcp(LIFT_CMD), TUBE_HOLD, "hold it upright", rot=LEFT_ROT))
    wps += [
        Waypoint(1.8, _tube_tcp(LIFT_CMD - 0.002, aim=LOWER_AIM), TUBE_HOLD, "over the rack", rot=LEFT_ROT),
        Waypoint(1.5, _tube_tcp(0.004, aim=LOWER_AIM), TUBE_HOLD, "lower it into the rack", rot=LEFT_ROT),
        Waypoint(1.0, _tube_tcp(-0.0005, aim=LOWER_AIM), TUBE_HOLD, "down onto the plate", rot=LEFT_ROT),
        Waypoint(0.8, _tube_tcp(-0.0005, aim=LOWER_AIM), OPEN, "let go", rot=LEFT_ROT),
        Waypoint(1.2, _tube_tcp(-0.0005, 0.07, aim=LOWER_AIM), OPEN, "back off", rot=LEFT_ROT),
        Waypoint(2.5, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps


def demo():
    """The left arm lifts the tube out of the rack and holds it; the right arm takes the reamer, reams in four strokes and puts
    it back in its rest; then the left arm stands the tube back in the rack."""
    right, out_t = right_plan()
    return {"left": left_plan(wait_until=out_t + 4.5), "right": right}
