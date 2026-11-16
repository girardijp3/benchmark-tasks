"""Bump-test a four-gas monitor before a confined-space entry.

On the bench: an MSA ALTAIR 4X four-gas monitor lies face up with its calibration cap sitting unsealed on its sensor
plate; the cap's hose runs to a 0.25 L/min fixed-flow regulator on a 34 L quad-gas cylinder (1.45 % CH4, 15 % O2,
60 ppm CO, 20 ppm H2S in nitrogen) in a bench cradle. The robot has to seat and snap the cap until its gasket seals,
open the regulator, let the four readings come up to the cylinder's values and stay there, then close the gas, pull
the cap off and leave the monitor back on fresh air.

The gas path is a declared physical model, not a pass stamp: the cap's internal volume (25 mL), the regulator's
0.25 L/min flow, the cap's seal or leak and the sensors' first-order response (T90 10 s) are integrated every control
step. A cap that is not snapped leaks far more than the regulator supplies and the readings never come up; a valve
left open after the cap is pulled vents the gas; a reading that is still climbing cannot count as stable.

Run it:  inspect-robots run --embodiment yam_arena -E task=gas_monitor_bump_test.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

TAG = "gasmon"
import importlib.abc
import importlib.util
import math
import sys
from collections import deque
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
        """yam_arena's environment settings, plus the task's reset/step events."""
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
MON_POS = (0.400, 0.000, TOP)          # the monitor's underside centre
MON_H = 0.033                          # ALTAIR 4X thickness, face up
SENSOR_X = -0.026                      # the sensor plate's centre in the monitor frame
CAP_SEAT = (MON_POS[0] + SENSOR_X, 0.0, TOP + MON_H)   # the cap's sealing plane when seated
CAP_REST = (0.290, -0.100, TOP + 0.060)
CAP_START = CAP_REST
REG_POS = (0.300, -0.220, TOP + 0.038)  # the regulator inlet face at the cylinder valve
CYL_POS = (0.300, -0.220, TOP)          # the cylinder valve face; the cylinder runs along +x
RIDGE_Z = TOP + 0.0185                  # the monitor's snap-ridge height

MONITOR = UsdPart("monitor", asset="monitor", xy=MON_POS[:2], extent=0.07, mass=0.224,
                  usd_path=str(ASSETS / "monitor.usda"), pos=MON_POS, articulation=True)
CAP = UsdPart("cap", asset="cap", xy=CAP_START[:2], extent=0.06, mass=0.058,
              usd_path=str(ASSETS / "cap.usda"), pos=CAP_START, articulation=True)
CYLINDER = UsdPart("cylinder", asset="cylinder", xy=CYL_POS[:2], extent=0.30, mass=2.0,
                   usd_path=str(ASSETS / "cylinder.usda"), pos=CYL_POS, kinematic=True)
REGULATOR = UsdPart("regulator", asset="regulator", xy=REG_POS[:2], extent=0.09, mass=0.239,
                    usd_path=str(ASSETS / "regulator.usda"), pos=REG_POS, articulation=True, fixed=True)
CAP_REST_PART = UsdPart("cap_rest", asset="cap_rest", xy=CAP_REST[:2], extent=0.08, mass=0.30,
                        usd_path=str(ASSETS / "cap_rest.usda"), pos=(CAP_REST[0], CAP_REST[1], TOP), kinematic=True)
OBJECTS = [MONITOR, CAP, CYLINDER, REGULATOR, CAP_REST_PART]
MOVERS = ("cap", "monitor")


# -- the monitor's declared display: four bar pointers on prismatic joints ---------------------------------------------
BAR_JOINTS = ("BarH2S", "BarCO", "BarLEL", "BarO2")
BAR_TRAVEL = 0.024
BAR_CHANNELS = ("H2S", "CO", "LEL", "O2")


def _joint_ids(art, names) -> tuple:
    return tuple(int(v) for v in art.find_joints(list(names), preserve_order=True)[0])


def _set_targets(art, values: dict) -> None:
    """Set a few joints' drive targets in the simulator (Arena objects have no actuators)."""
    import torch

    view = art.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    names = list(view.shared_metatype.dof_names)
    for name, value in values.items():
        t[:, names.index(name)] = value
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, art._ALL_INDICES)


# -- the gas path: a declared first-order physical model ---------------------------------------------------------------
QMAX = 0.25 / 60000.0        # 0.25 L/min (MSA ALTAIR 4X manual), m3/s
V_CAP = 25.0e-6             # cap cavity + hose dead volume, m3 (estimated from the modelled 25 mL)
T90 = 10.0                  # s, the ALTAIR 4X response is "less than 15 s": 10 s used
TAU = T90 / math.log(10.0)  # first-order sensor time constant, s
LEAK = 40.0 / 60000.0       # an unsealed cap leaks about 40 L/min, m3/s (a deliberately large, stated leak)
VALVE_FULL = math.radians(75.0)   # the knob is "open" from 75 degrees
VALVE_CLOSED = math.radians(3.0)
GAS = {"H2S": 20.0, "CO": 60.0, "LEL": 29.0, "O2": 15.0}   # MSA 34 L quad mix: 20 ppm, 60 ppm, 1.45 % CH4, 15 % O2
AMBIENT = {"H2S": 0.0, "CO": 0.0, "LEL": 0.0, "O2": 20.9}
BAND = {"H2S": 0.10, "CO": 0.10, "LEL": 0.10, "O2": 0.034}  # +-10 % of the cylinder's value; O2 +-0.5 % vol
WINDOW = (5.0, 45.0)        # s after the regulator opens: the real bump-test window
STABLE_S = 3.0              # the reading must hold inside the band for this long


class _Gas:
    """The cap's gas fraction and the four sensor readings, integrated at the control rate."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.f = 0.0
        self.s = 0.0
        self.t = 0.0
        self.flow = 0.0
        self.valve = 0.0
        self.seal = False
        self.on = False
        self.open_t = None
        self.history = deque(maxlen=int(round(STABLE_S * 20.0)))
        self.cap_seated = False
        self.opened = False
        self.stable = False
        self.order_error = False
        self.leak_event = False
        self.vented = False
        self.safe = True
        self.readings = dict(AMBIENT)

    def update(self, dt: float, seal: bool, on: bool, valve: float) -> None:
        self.t += dt
        self.valve = float(valve)
        self.seal = bool(seal)
        self.on = bool(on)
        frac = float(np.clip((self.valve - 0.02) / (math.radians(88.0) - 0.02), 0.0, 1.0))
        self.flow = QMAX * frac
        if self.seal and self.on:
            self.cap_seated = True
        if self.flow > 0.02 * QMAX and not self.cap_seated:
            self.order_error = True                       # gas opened before the cap sealed
        if self.flow > 0.02 * QMAX and not self.seal:
            self.leak_event = True                        # gas flowing through an unsealed/absent cap
        if self.cap_seated and self.flow > 0.02 * QMAX and not self.seal:
            self.vented = True                            # the cap was pulled while the gas was still on
        if self.flow >= 0.80 * QMAX and self.seal and self.on:
            self.open_t = 0.0 if self.open_t is None else self.open_t + dt
        elif self.flow < 0.05 * QMAX:
            self.open_t = None
        if self.flow >= 0.80 * QMAX and self.seal and self.on:
            self.opened = True
        # the well-mixed cap: dC/dt = (Q(1-C) - leak*C) / V
        leak = 0.0 if self.seal else LEAK
        if self.on:
            self.f += dt * (self.flow * (1.0 - self.f) - leak * self.f) / V_CAP
            self.f = float(np.clip(self.f, 0.0, 1.0))
        else:
            self.f = 0.0
        self.s += dt * (self.f - self.s) / TAU
        self.s = float(np.clip(self.s, 0.0, 1.0))
        readings = {k: AMBIENT[k] + (GAS[k] - AMBIENT[k]) * self.s for k in GAS}
        self.history.append(readings)
        in_band = all(abs(readings[k] - GAS[k]) <= BAND[k] * abs(GAS[k]) for k in GAS)
        in_window = self.open_t is not None and WINDOW[0] <= self.open_t <= WINDOW[1]
        steady = False
        if len(self.history) == self.history.maxlen and in_band and in_window:
            steady = all(max(h[k] for h in self.history) - min(h[k] for h in self.history)
                         <= 0.02 * abs(GAS[k]) for k in GAS)
        if steady:
            self.stable = True
        self.safe = all(abs(readings[k] - AMBIENT[k]) <= (1.0 if k == "H2S" else 2.0 if k == "CO" else 1.0 if k == "LEL" else 0.5)
                        for k in GAS)
        self.readings = readings

    def snapshot(self) -> dict:
        out = dict(self.readings)
        out.update(seal=self.seal, on=self.on, flow=self.flow, valve=math.degrees(self.valve),
                   cap_seated=self.cap_seated, opened=self.opened, stable=self.stable,
                   order_error=self.order_error, leak_event=self.leak_event, vented=self.vented,
                   safe=self.safe, open_t=-1.0 if self.open_t is None else self.open_t, f=self.f, s=self.s)
        return out

    def bar_fractions(self) -> tuple:
        r = self.readings
        return (float(np.clip(r["H2S"] / GAS["H2S"], 0.0, 1.0)),
                float(np.clip(r["CO"] / GAS["CO"], 0.0, 1.0)),
                float(np.clip(r["LEL"] / GAS["LEL"], 0.0, 1.0)),
                float(np.clip((AMBIENT["O2"] - r["O2"]) / (AMBIENT["O2"] - GAS["O2"]), 0.0, 1.0)))


_GAS = _Gas()
_RIGHT_OPEN = {"value": False}


def _cap_frame(base):
    """The cap's pose in the monitor's frame and the tab angles."""
    mon = base.scene["monitor"]
    cap = base.scene["cap"]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    mp = _arr(mon.data.root_link_pose_w)[:3] - origin
    mq = _arr(mon.data.root_link_pose_w)[3:]
    cp = _arr(cap.data.root_link_pose_w)[:3] - origin
    cq = _arr(cap.data.root_link_pose_w)[3:]
    Rm = quat_xyzw_to_matrix(mq)
    Rc = quat_xyzw_to_matrix(cq)
    local = Rm.T @ (cp - mp)
    tilt = math.acos(max(-1.0, min(1.0, float(Rm[:, 2] @ Rc[:, 2]))))
    xr = Rm.T @ Rc[:, 0]
    yaw = abs(math.atan2(float(xr[1]), float(xr[0])))
    ids = _joint_ids(cap, ("TabL", "TabR"))
    tabs = _arr(cap.data.joint_pos)[list(ids)]
    return local, tilt, yaw, tabs


def _seal_ok(local, tilt, yaw, tabs) -> bool:
    """The gasket/seal rule: cap on the sensor plate, level, both tabs closed. Kept pure so the tests can probe it."""
    return bool(abs(local[0] - SENSOR_X) < 0.0030 and abs(local[1]) < 0.0030
                and abs(local[2] - MON_H) < 0.0018 and tilt < math.radians(3.0) and yaw < math.radians(3.0)
                and max(abs(float(tabs[0])), abs(float(tabs[1]))) < math.radians(3.0))


def _cap_state(base):
    local, tilt, yaw, tabs = _cap_frame(base)
    on = bool(abs(local[0] - SENSOR_X) < 0.045 and abs(local[1]) < 0.060
              and MON_H - 0.010 < local[2] < MON_H + 0.060)
    return bool(on and _seal_ok(local, tilt, yaw, tabs)), on


def _valve_deg(base) -> float:
    reg = base.scene["regulator"]
    ids = _joint_ids(reg, ("Knob",))
    return math.degrees(float(_arr(reg.data.joint_pos)[list(ids)][0]))


def _gas_step(env, env_ids, asset_cfg=None) -> None:
    """Every control step: read the cap and valve, integrate the gas path, and move the display pointers."""
    base = env.unwrapped
    seal, on = _cap_state(base)
    valve = _valve_deg(base)
    dt = float(env.step_dt) if hasattr(env, "step_dt") else 0.05
    _GAS.update(dt, seal, on, math.radians(valve))
    mon = base.scene["monitor"]
    _set_targets(mon, {name: BAR_TRAVEL * f for name, f in zip(BAR_JOINTS, _GAS.bar_fractions())})
    _RIGHT_OPEN["value"] = bool(_GAS.opened)


def _reset_episode(env, env_ids, asset_cfg=None) -> None:
    """Reset event: the cap tabs, the regulator knob and the display pointers back to their starts, gas state zeroed."""
    import torch

    if env_ids is None:
        return
    base = env.unwrapped
    for name, joints in (("cap", ("TabL", "TabR")), ("regulator", ("Knob",)), ("monitor", BAR_JOINTS)):
        art = base.scene[name]
        ids = torch.tensor(_joint_ids(art, joints), dtype=torch.long, device=art.device)
        zero = torch.zeros((len(env_ids), len(ids)), device=art.device)
        art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
        art.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
        _set_targets(art, {n: 0.0 for n in joints})
    _GAS.reset()
    _RIGHT_OPEN["value"] = False


def extra_states(base, out, origin) -> None:
    """The cap tabs, the regulator knob, the display bars and the gas readings."""
    cap = base.scene["cap"]
    reg = base.scene["regulator"]
    mon = base.scene["monitor"]
    local, tilt, yaw, tabs = _cap_frame(base)
    out["cap_tabs"] = tuple(float(v) for v in tabs)
    out["cap_local"] = tuple(float(v) for v in local)
    out["cap_tilt"] = float(tilt)
    out["cap_yaw"] = float(yaw)
    out["valve_deg"] = _valve_deg(base)
    out["bars"] = tuple(float(v) for v in _arr(mon.data.joint_pos)[list(_joint_ids(mon, BAR_JOINTS))])
    out["gas"] = _GAS.snapshot()


def reset_events() -> list:
    return [("gas_reset", _reset_episode, {})]


def step_events() -> list:
    return [("gas_step", _gas_step, {})]


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
STEPS = ("the calibration cap seated and its gasket sealed",
         "the regulator opened after the cap sealed",
         "the four readings stable inside the cylinder's values",
         "the gas shut off, the cap off, the monitor back on fresh air")
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002
TRACE_S = 1.0
POINTS = 0.25
GRIP_SLACK = 0.004
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
CLEAR = 0.03
CAP_LO = np.array([-0.031, -0.042, -0.021])
CAP_HI = np.array([0.031, 0.042, 0.012])
MON_LO = np.array([-0.056, -0.038, 0.0])
MON_HI = np.array([0.056, 0.038, MON_H])


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _pads(gripper) -> np.ndarray:
    return gripper.back + np.linspace(0.0, 1.0, 11)[:, None] * (gripper.tips - gripper.back)


def holds(objects, side: str, name: str, lo, hi) -> bool:
    """The ``side`` gripper holds the part: a point along its pads is within GRIP_SLACK of the part's bounds, both
    fingers press with GRIP_FORCE and the pads are apart by GRIP_OPENING."""
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
    """Every mover within STILL_MOVE and STILL_TURN of its pose in ``anchor``."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


def _final(objects) -> bool:
    """Gas off, cap off and clear, monitor safe and on the table, with the bump test having happened."""
    gas = objects["gas"]
    cap = objects["cap"]
    mon = objects["monitor"]
    loc = objects.get("cap_local", (0.0, 0.0, 0.0))
    off = abs(loc[0] - SENSOR_X) > 0.030 or abs(loc[1]) > 0.030   # the cap is away from the sensor plate
    return bool(not gas["seal"] and gas["valve"] < 3.0 and gas["safe"] and off
                and cap.pos[2] < TOP + 0.09 and mon.pos[2] > TOP - 0.02
                and not pads_near(objects, "cap", CAP_LO, CAP_HI, CLEAR))


def _steps(objects) -> list[bool]:
    gas = objects["gas"]
    base = [bool(gas["cap_seated"]), bool(gas["opened"]), bool(gas["stable"])]
    return base + [bool(all(base) and _final(objects))]


def events_now(objects, p) -> list[str]:
    """Rule events (score 0): the cap or the monitor off the table. The gas-path mistakes are caps, not zeros."""
    out = []
    if objects["cap"].pos[2] < TOP - 0.03:
        out.append("the calibration cap fell off the table")
    if objects["monitor"].pos[2] < TOP - 0.03:
        out.append("the monitor fell off the table")
    return out


def trace_line(objects) -> str:
    g = objects["gas"]
    cap = objects["cap"]
    loc = objects.get("cap_local", (0.0, 0.0, 0.0))
    return (f"H2S {g['H2S']:5.1f} ppm, CO {g['CO']:5.1f} ppm, LEL {g['LEL']:5.1f} %, O2 {g['O2']:4.1f} %; "
            f"valve {objects['valve_deg']:5.1f} deg, flow {g['flow'] * 60000:4.2f} L/min, seal {g['seal']}, "
            f"cap local ({loc[0]:.3f},{loc[1]:.3f},{loc[2]:.3f}) tilt {math.degrees(objects.get('cap_tilt', 0.0)):.1f} "
            f"yaw {math.degrees(objects.get('cap_yaw', 0.0)):.1f}, tabs "
            f"{math.degrees(max(abs(v) for v in objects['cap_tabs'])):.1f} deg, "
            f"open {g['open_t']:4.1f} s, stable {g['stable']}, safe {g['safe']}")


class _Progress:
    """What has happened so far this episode: rule events and the 1 s full-success timer."""

    def __init__(self):
        self.time = -1.0
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
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        stamp = -1.0 if t is None else t
        for what in events_now(objects, self):
            self.flag(stamp, what)
        if t is None:
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
    values = [v for n in MOVERS for v in objects[n].pos]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.events)


def violation(objects) -> bool:
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all four steps, no rule event, and the cap and monitor within STILL_MOVE and STILL_TURN of where they were
    when that second began."""
    p = _PROGRESS
    p.update(objects)
    gas = objects["gas"]
    if gas["order_error"] or gas["leak_event"] or gas["vented"] or gas["valve"] >= 3.0:
        return False
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success; otherwise 0.25 per step (at most 0.75), with the gas-path caps: an unsealed cap or gas opened
    before the cap seals caps at 0.25, a reading that never stabilises at 0.5, gas left on at 0.75; 0 for a part off
    the table."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    gas = objects["gas"]
    if success(objects):
        return 1.0
    # 0.25, 0.50, 0.75 for the three reached steps; the fourth state is only worth 1.0 once it has held 1 s
    s = POINTS * sum(done[:3])
    if gas["order_error"] or gas["leak_event"] or gas["vented"]:
        s = min(s, 0.25)
    if gas["valve"] >= 3.0:
        s = min(s, 0.75)
    if not done[2]:
        s = min(s, 0.5)
    return round(s, 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
_grip = yam_demo.grip_rotation
CAP_OPEN = -0.036   # half-opening: the pads 72 mm apart
CAP_HOLD = -0.028   # the pads squeeze the 62 mm cap ends from outside
KNOB_OPEN = -0.020  # the pads 40 mm apart
KNOB_HOLD = -0.0135  # the pads squeeze the 32 mm knob
R_CAP = _grip(math.pi / 2)
R_KNOB = _grip(0.0)


def _turn(angle: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), angle) @ R_KNOB


def right_plan() -> list[Waypoint]:
    cap_x, cap_y = CAP_START[0], CAP_START[1]
    seat_x, seat_y = CAP_SEAT[0], CAP_SEAT[1]
    knob = (REG_POS[0] - 0.026, REG_POS[1], REG_POS[2] + 0.020)
    wait = (0.300, -0.300, TOP + 0.110)
    return [
        Waypoint(2.0, (cap_x, cap_y, CAP_START[2] + 0.035), CAP_OPEN, "over the cap on its rest", rot=R_CAP),
        Waypoint(1.2, (cap_x, cap_y, CAP_START[2] + 0.003), CAP_OPEN, "pads astride the cap ends", rot=R_CAP),
        Waypoint(0.8, (cap_x, cap_y, CAP_START[2] + 0.003), CAP_HOLD, "grip the cap", rot=R_CAP),
        Waypoint(1.4, (cap_x, cap_y, CAP_START[2] + 0.045), CAP_HOLD, "lift the cap", rot=R_CAP),
        Waypoint(2.2, (seat_x, seat_y, CAP_SEAT[2] + 0.055), CAP_HOLD, "carry it over the sensor plate", rot=R_CAP),
        Waypoint(1.2, (seat_x, seat_y, CAP_SEAT[2] + 0.026), CAP_HOLD, "lower onto the plate", rot=R_CAP),
        Waypoint(0.5, (seat_x, seat_y, CAP_SEAT[2] + 0.026), CAP_OPEN, "set it on the plate", rot=R_CAP),
        Waypoint(0.8, (seat_x, seat_y, CAP_SEAT[2] + 0.055), CAP_OPEN, "lift clear and open the pads", rot=R_CAP),
        Waypoint(0.8, (seat_x, seat_y, CAP_SEAT[2] + 0.055), 0.004, "close the pads to press", rot=R_CAP),
        Waypoint(1.4, (seat_x, seat_y, CAP_SEAT[2] + 0.025), 0.004, "press the cap top: the tabs snap", rot=R_CAP),
        Waypoint(0.6, (seat_x, seat_y + 0.024, CAP_SEAT[2] + 0.028), 0.004, "press the +y side", rot=R_CAP),
        Waypoint(0.6, (seat_x, seat_y + 0.024, CAP_SEAT[2] + 0.022), 0.004, "seat the +y tab", rot=R_CAP),
        Waypoint(0.6, (seat_x, seat_y - 0.024, CAP_SEAT[2] + 0.028), 0.004, "press the -y side", rot=R_CAP),
        Waypoint(0.6, (seat_x, seat_y - 0.024, CAP_SEAT[2] + 0.022), 0.004, "seat the -y tab", rot=R_CAP),
        Waypoint(0.6, (seat_x, seat_y, CAP_SEAT[2] + 0.025), 0.004, "settle the gasket", rot=R_CAP),
        Waypoint(0.5, (seat_x, seat_y, CAP_SEAT[2] + 0.025), CAP_OPEN, "let go: the tabs hold it", rot=R_CAP),
        Waypoint(0.8, (seat_x, seat_y, CAP_SEAT[2] + 0.055), CAP_OPEN, "lift clear of the cap", rot=R_CAP),
        Waypoint(2.2, (knob[0], knob[1], TOP + 0.085), KNOB_OPEN, "over the regulator knob", rot=R_KNOB),
        Waypoint(1.0, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "pads astride the knob", rot=R_KNOB),
        Waypoint(0.7, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "grip the knob", rot=R_KNOB),
        Waypoint(3.0, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "open the regulator 45 deg", rot=_turn(math.radians(45.0))),
        Waypoint(0.4, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "release", rot=_turn(math.radians(45.0))),
        Waypoint(0.8, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "wrist back", rot=R_KNOB),
        Waypoint(0.7, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "grip again", rot=R_KNOB),
        Waypoint(3.0, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "open the regulator to 90 deg", rot=_turn(math.radians(90.0))),
        Waypoint(0.4, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "release the open knob", rot=_turn(math.radians(90.0))),
        Waypoint(1.0, (knob[0], knob[1], TOP + 0.090), KNOB_OPEN, "lift clear", rot=R_KNOB),
        Waypoint(2.5, wait, KNOB_OPEN, "wait for the readings", rot=R_KNOB),
        Waypoint(30.0, wait, KNOB_OPEN, "the four readings stabilise", rot=R_KNOB),
        Waypoint(2.2, (knob[0], knob[1], TOP + 0.085), KNOB_OPEN, "back to the regulator", rot=R_KNOB),
        Waypoint(1.0, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "pads astride the knob", rot=R_KNOB),
        Waypoint(0.7, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "grip to close", rot=_turn(math.radians(90.0))),
        Waypoint(3.0, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "close 45 deg", rot=_turn(math.radians(45.0))),
        Waypoint(0.4, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "release", rot=_turn(math.radians(45.0))),
        Waypoint(0.8, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "wrist back", rot=_turn(math.radians(90.0))),
        Waypoint(0.7, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "grip again", rot=_turn(math.radians(90.0))),
        Waypoint(5.0, (knob[0], knob[1], TOP + 0.077), KNOB_HOLD, "close past 0 to the stop", rot=_turn(math.radians(-20.0))),
        Waypoint(0.4, (knob[0], knob[1], TOP + 0.077), KNOB_OPEN, "release the closed knob", rot=_turn(math.radians(0.0))),
        Waypoint(1.0, (knob[0], knob[1], TOP + 0.090), KNOB_OPEN, "lift clear", rot=R_KNOB),
        Waypoint(2.2, (seat_x + 0.006, seat_y, CAP_SEAT[2] + 0.055), CAP_OPEN, "back over the cap", rot=R_CAP),
        Waypoint(1.0, (seat_x + 0.006, seat_y, CAP_SEAT[2] + 0.011), CAP_OPEN, "pads astride the cap", rot=R_CAP),
        Waypoint(0.7, (seat_x + 0.006, seat_y, CAP_SEAT[2] + 0.011), CAP_HOLD, "grip the cap", rot=R_CAP),
        Waypoint(1.6, (seat_x + 0.006, seat_y, CAP_SEAT[2] + 0.055), CAP_HOLD, "pull the tabs out and lift", rot=R_CAP),
        Waypoint(2.2, (CAP_REST[0], CAP_REST[1], CAP_REST[2] + 0.050), CAP_HOLD, "carry the cap to its rest", rot=R_CAP),
        Waypoint(1.2, (CAP_REST[0], CAP_REST[1], CAP_REST[2] + 0.030), CAP_HOLD, "set it on the rest", rot=R_CAP),
        Waypoint(0.5, (CAP_REST[0], CAP_REST[1], CAP_REST[2] + 0.030), CAP_OPEN, "let go", rot=R_CAP),
        Waypoint(0.8, (CAP_REST[0], CAP_REST[1], CAP_REST[2] + 0.050), CAP_OPEN, "lift clear", rot=R_CAP),
        Waypoint(2.5, wait, KNOB_OPEN, "let the monitor clear", rot=R_KNOB),
        Waypoint(25.0, wait, KNOB_OPEN, "the monitor back on fresh air", rot=R_KNOB),
        Waypoint(2.5, tuple(_START_TCP["right"]), FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def left_plan() -> list[Waypoint]:
    return [Waypoint(1.0, tuple(_START_TCP["left"]), FINGER_OPEN, "home", rot=_START_ROT["left"])]


def demo():
    """The left hand presses the monitor down; the right takes the cap from its rest, seats it, opens the regulator,
    waits for the readings, closes it, pulls the cap off and sets it back on its rest."""
    return {"left": left_plan(), "right": right_plan()}
