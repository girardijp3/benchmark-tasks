"""On the station table, in front of the robot: a bench DC power supply (0-30 V / 0-5 A, 110 x 158 x 262 mm, 2.5 kg) on
its rubber feet, front panel toward the robot, switched off, its voltage knob at 12 V. The left arm presses the power
button (a latching push button) and goes back; the right arm grips the coarse voltage knob, turns it from 12 V down to
9 V (27 deg counter-clockwise, seen from the front), lets go and goes back. The voltmeter on the panel shows the knob's
voltage while the supply is on.

The supply is one articulation from ``assets/`` (its case, the push button on a sprung slide, four damped knobs). The
button's latch (push on, push off) and the two 7-segment displays are run by this file: a step event reads the
button's travel and moves its spring's target between "out" and "held in", and shows the voltage on the display by
switching the display's segment prims on and off in the USD stage.

Run it:  inspect-robots run --embodiment yam_arena -E task=psu_voltage.py --scorer task_score ...

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
TAG = "psu_voltage"

# -- the supply (its own frame: origin at the footprint's middle on the table, front panel facing -y, z up) ----------
Y_PANEL = -0.128  # the front panel's face
BUTTON_AT = np.array([-0.036, Y_PANEL, 0.028])  # the power button's middle on the panel face
BUTTON_OUT = 0.008  # its cap stands this far out of the panel when released
BUTTON_HALF = 0.007  # half its width (a 14 mm square cap)
KNOB_AT = {  # each knob's axis on the panel face; grip radius, depth (from the panel to its face), skirt depth
    "Knob_V_Coarse": np.array([-0.026, Y_PANEL, 0.101]),
    "Knob_V_Fine": np.array([-0.026, Y_PANEL, 0.064]),
    "Knob_I_Coarse": np.array([0.026, Y_PANEL, 0.101]),
    "Knob_I_Fine": np.array([0.026, Y_PANEL, 0.064]),
}
KNOB_R, KNOB_DEPTH, SKIRT_FRONT = 0.013, 0.018, 0.0026  # the coarse knobs: radius, face depth, skirt's front face
JOINTS = ("PowerButton", "Knob_V_Coarse", "Knob_V_Fine", "Knob_I_Coarse", "Knob_I_Fine")
OTHER_KNOBS = ("Knob_V_Fine", "Knob_I_Coarse", "Knob_I_Fine")

# The voltage: the coarse knob's joint is 0 at 12 V and turns 270 deg for 0-30 V, clockwise (seen from the front) up;
# the fine knob adds -0.5..+0.5 V over its 270 deg, 0 at its middle.
VOLTS_START = 12.0
VOLTS_PER_RAD = 30.0 / (1.5 * math.pi)
FINE_VOLTS_PER_RAD = 1.0 / (1.5 * math.pi)
VOLTS_TARGET = 9.0

# The push button's latch: a push past CLICK (of its 4.5 mm travel) switches the supply on and the spring's target moves
# to HELD_IN (the latch holds the button in there); the next push past CLICK switches it off and the target goes back
# to OUT (2 mm past "out": a 0.8 N preload). A push counts again only after the button has come back under REARM.
CLICK, REARM, HELD_IN, OUT = 0.0035, 0.0030, 0.0025, -0.002

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
PSU_XY, PSU_YAW = (0.50, -0.03), -math.pi / 2  # its front panel toward the robot (the supply's -y is the world's -x)


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


PSU_POS = (*PSU_XY, TOP)
PSU_ROT = _rz(PSU_YAW)


def psu_point(p, pos=PSU_POS, rot=PSU_ROT) -> np.ndarray:
    """A point given in the supply's frame, in the world (at the supply's start pose unless given)."""
    return np.asarray(pos, float) + np.asarray(rot) @ np.asarray(p, float)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not self.kinematic


# Arena places an articulation by its root link: the supply's is its Body, at the asset's origin.
PSU = UsdPart("psu", asset="psu", xy=PSU_XY, extent=0.27, mass=2.5, usd_path=str(ASSETS / "bench_psu.usda"),
              pos=PSU_POS, rot_xyzw=_yaw_xyzw(PSU_YAW), articulation=True)
OBJECTS = [PSU]

SOLVER_ITERATIONS = 32  # position iterations per step for the supply's articulation (light links: button, knobs)
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

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


# -- the supply's switch and displays (run by a step event while the simulator runs) ----------------------------------
class _Switch:
    """The power button's push-on/push-off latch."""

    def __init__(self):
        self.on = False
        self.armed = True  # the button has come back since the last switch

    def update(self, travel: float) -> bool:
        """Follows the button's travel (m); True when the supply switches on or off."""
        if travel > CLICK and self.armed:
            self.on, self.armed = not self.on, False
            return True
        if travel < REARM:
            self.armed = True
        return False

    @property
    def target(self) -> float:
        return HELD_IN if self.on else OUT


_SWITCH = _Switch()

SEGMENTS = {" ": "", "0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg", "6": "acdefg",
            "7": "abc", "8": "abcdefg", "9": "abcdfg"}
DP_AFTER = {"V": 2, "A": 1}  # the decimal point lit on each display: after its 2nd digit (12.00) and 1st (0.000)


def volts_from(coarse: float, fine: float) -> float:
    """The output voltage set by the coarse and fine knobs' joint angles (rad)."""
    return float(np.clip(VOLTS_START + coarse * VOLTS_PER_RAD + fine * FINE_VOLTS_PER_RAD, 0.0, 30.0))


def display_text(on: bool, volts: float) -> dict:
    """What the two displays show: four digits each (a space is dark); the current reads 0.000 with nothing on the
    terminals. Off, both are dark."""
    if not on:
        return {"V": "    ", "A": "    "}
    v = f"{min(volts, 30.0):5.2f}".replace(".", "")
    return {"V": v if v[0] != "0" else " " + v[1:], "A": "0000"}


class _Display:
    """Shows the displays' digits by making their segment prims (authored invisible) visible, in the running
    simulator only."""

    def __init__(self):
        self.shown = None
        self.root = None

    def _body(self, stage):
        if self.root is None:
            prim = stage.GetPrimAtPath("/World/envs/env_0/psu/Body")
            if not prim.IsValid():  # wherever Arena put it
                prim = next((p for p in stage.Traverse() if p.GetName() == "Display_V"), None)
                prim = prim.GetParent() if prim is not None else None
            self.root = prim.GetPath() if prim is not None and prim.IsValid() else ""
        return self.root

    def show(self, on: bool, volts: float) -> None:
        text = display_text(on, volts)
        key = (text["V"], text["A"], on)
        if key == self.shown:
            return
        try:
            import omni.usd
            from pxr import UsdGeom
        except ImportError:
            return
        stage = omni.usd.get_context().get_stage()
        body = self._body(stage) if stage is not None else ""
        if not body:
            return
        for name, digits in text.items():
            for d, ch in enumerate(digits, start=1):
                lit = SEGMENTS[ch] + ("p" if on and d == DP_AFTER[name] else "")
                for seg in "abcdefgp":
                    prim = stage.GetPrimAtPath(f"{body}/Display_{name}/D{d}_{'dp' if seg == 'p' else seg}")
                    if prim.IsValid():
                        im = UsdGeom.Imageable(prim)
                        im.MakeVisible() if seg in lit else im.MakeInvisible()
        led = stage.GetPrimAtPath(f"{body}/Leds/CV_Lit")
        if led.IsValid():
            UsdGeom.Imageable(led).MakeVisible() if on else UsdGeom.Imageable(led).MakeInvisible()
        if self.shown is None or self.shown[2] != on:
            print(f"[{TAG}] display {'on' if on else 'off'}", flush=True)
        self.shown = key


_DISPLAY = _Display()


TRACE_EVERY = 20  # control steps between trace lines (0: none)


def _joint_ids(psu):
    return psu.find_joints(list(JOINTS), preserve_order=True)[0]


def _set_button_target(psu, value: float) -> None:
    """Moves the button spring's drive target (m) in PhysX directly: Arena gives objects no actuators, so Isaac Lab's
    joint targets never reach the simulator and the drives keep the targets authored in the USD."""
    import torch

    view = psu.root_view
    cur = view.get_dof_position_targets()
    try:
        import warp as wp

        warp_data = isinstance(cur, wp.array)
    except ImportError:
        warp_data = False
    t = (wp.to_torch(cur) if warp_data else torch.as_tensor(cur)).clone()
    t[:, list(view.shared_metatype.dof_names).index("PowerButton")] = value
    view.set_dof_position_targets(wp.from_torch(t) if warp_data else t, psu._ALL_INDICES)


def _reset_psu(env, env_ids, asset_cfg) -> None:
    """Reset event: the button out and every knob at its start (the simulator keeps an articulation's joints where
    they were), the button's spring target back to "out", the supply off."""
    import torch

    if env_ids is None:
        return
    psu = env.scene[asset_cfg.name]
    ids = _joint_ids(psu)
    zero = torch.zeros((len(env_ids), len(ids)), device=psu.device)
    psu.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    psu.write_joint_velocity_to_sim_index(velocity=zero, joint_ids=ids, env_ids=env_ids)
    _set_button_target(psu, OUT)
    _SWITCH.__init__()
    _DISPLAY.shown = None
    _DISPLAY.show(False, VOLTS_START)


def _psu_step(env, env_ids, asset_cfg) -> None:
    """Every control step: the button's latch (its spring's target) and the displays."""
    psu = env.scene[asset_cfg.name]
    ids = _joint_ids(psu)
    q = psu.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
    step = int(env.episode_length_buf[0])
    t = step * float(env.step_dt)
    if _SWITCH.update(float(q[0])):
        _set_button_target(psu, _SWITCH.target)
        print(f"[{TAG}] t={t:.2f}s  power {'on' if _SWITCH.on else 'off'} (button in {q[0] * 1000:.1f} mm)", flush=True)
    _DISPLAY.show(_SWITCH.on, volts_from(q[1], q[2]))
    if TRACE_EVERY and step % TRACE_EVERY == 0:  # the supply's motion and joints, for tuning
        lin = float(psu.data.root_lin_vel_w.torch[0].norm())
        ang = float(psu.data.root_ang_vel_w.torch[0].norm())
        pos = psu.data.root_link_pose_w.torch[0, :3].detach().cpu().numpy() - env.scene.env_origins[0].cpu().numpy()
        print(f"[{TAG}] t={t:.2f}s  trace: supply {lin * 1000:.2f} mm/s {math.degrees(ang):.2f} deg/s, at "
              f"({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}); button {q[0] * 1000:.2f} mm; knobs (deg) "
              + " ".join(f"{math.degrees(v):.2f}" for v in q[1:]) + f"; {volts_from(q[1], q[2]):.2f} V", flush=True)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root (an articulation's bounds are left at its root), the grippers, the episode time, the
        button's travel, the knobs' angles and whether the supply is on."""
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
        psu = base.scene["psu"]
        q = psu.data.joint_pos.torch[0, _joint_ids(psu)].detach().cpu().numpy().astype(np.float64)
        out["button_travel"] = float(q[0])
        out.update({name: float(v) for name, v in zip(JOINTS[1:], q[1:], strict=True)})
        out["power_on"] = _SWITCH.on
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that puts the button and knobs at their start, and a
        step event (every control step) for the button's latch and the displays."""
        configure = stock_cfg_callback(control_hz)

        def with_supply(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.psu_reset = EventTermCfg(func=_reset_psu, mode="reset",
                                                    params={"asset_cfg": SceneEntityCfg("psu")})
            env_cfg.events.psu_step = EventTermCfg(func=_psu_step, mode="interval", interval_range_s=(0.0, 0.0),
                                                   is_global_time=False, params={"asset_cfg": SceneEntityCfg("psu")})
            return env_cfg

        return with_supply

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
VOLTS_TOL = 0.2  # V: the knob set within this of 9 V
KNOB_UNTOUCHED = math.radians(3)  # the other knobs within this of their start (0.1 A on the current knob)
HELD_IN_SLACK = 0.0008  # m: the button within this of its held-in travel (or pressed further): latched
BUTTON_REACH = 0.025  # m: a gripper's pad tips this close to the button's face when it switched on: pressed by a gripper
PSU_SHIFT = 0.01  # m: the supply moved no more than this on the table, and
PSU_TURN = math.radians(5)  # ... turned no more than this
TIPPED = math.radians(30)  # tilted past this: knocked over
FALLEN_Z = -0.02  # m above the table: lower, it has fallen off
HOME_SLACK = 0.03  # m: a gripper this close to where its arm started, holding nothing, is home
GRIP_SLACK, GRIP_FORCE, GRIP_OPENING = 0.005, 0.5, 0.003  # see _held
STEPS = ("power on", "voltage knob gripped", "set to 9 V", "nothing else changed")
POINTS = 0.25  # per step


def _rot(objects) -> np.ndarray:
    return quat_xyzw_to_matrix(objects["psu"].quat_xyzw)


def volts(objects) -> float:
    return volts_from(objects["Knob_V_Coarse"], objects["Knob_V_Fine"])


def button_face(objects) -> np.ndarray:
    """The middle of the button's face (world), where it is now."""
    p = BUTTON_AT + np.array([0.0, objects["button_travel"] - BUTTON_OUT, 0.0])
    return psu_point(p, objects["psu"].pos, _rot(objects))


def latched_on(objects) -> bool:
    """The supply is on and the latch holds the button in."""
    return bool(objects["power_on"] and objects["button_travel"] > HELD_IN - HELD_IN_SLACK)


def knob_bounds(objects, name: str = "Knob_V_Coarse"):
    """World bounds of a coarse knob's grip (a cylinder from its skirt to its face)."""
    lo = KNOB_AT[name] + np.array([-KNOB_R, -KNOB_DEPTH, -KNOB_R])
    hi = KNOB_AT[name] + np.array([KNOB_R, -SKIRT_FRONT, KNOB_R])
    return _box((objects["psu"].pos, _rot(objects)), (lo, hi))


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose`` (position, rotation)."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ np.asarray(pose[1]).T + pose[0]
    return w.min(0), w.max(0)


def _held(bounds, gripper) -> bool:
    """``gripper`` holds the part with world ``bounds``: its pads reach them and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = bounds[0] - GRIP_SLACK, bounds[1] + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def knob_held(objects) -> bool:
    return any(_held(knob_bounds(objects), objects.get(g)) for g in ("left_gripper", "right_gripper"))


def _gripper_at_button(objects) -> bool:
    face = button_face(objects)
    return any(g is not None and np.linalg.norm(g.tips - face) < BUTTON_REACH
               for g in (objects.get("left_gripper"), objects.get("right_gripper")))


def others_untouched(objects) -> bool:
    return all(abs(objects[k]) < KNOB_UNTOUCHED for k in OTHER_KNOBS)


def psu_in_place(objects) -> bool:
    """The supply within PSU_SHIFT of its start on the table and turned no more than PSU_TURN."""
    p = objects["psu"]
    x = _rot(objects)[:, 0]
    turn = abs((math.atan2(x[1], x[0]) - PSU_YAW + math.pi) % (2 * math.pi) - math.pi)
    return bool(np.linalg.norm(p.pos[:2] - np.array(PSU_XY)) < PSU_SHIFT and turn < PSU_TURN)


def _home(objects, side: str) -> bool:
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    tcp = g.back + PAD_BACK * (g.tips - g.back) / np.linalg.norm(g.tips - g.back)
    return bool(np.linalg.norm(tcp - _START_TCP[side]) < HOME_SLACK and g.contact < GRIP_FORCE)


class _Progress:
    """What has happened so far this episode: whether the supply was last switched on by a gripper pressing the button,
    and whether a gripper has held the voltage knob. Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.was_on = False
        self.on_by_gripper = False
        self.knob_gripped = False
        self.reported = [False] * len(STEPS)

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
            self.time = t
        on = bool(objects["power_on"])
        if on and not self.was_on:
            self.on_by_gripper = _gripper_at_button(objects)
        self.was_on = on
        self.knob_gripped |= knob_held(objects)


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: 2 once reached; 1, 3 and 4 as things stand. Step 4 counts only with steps 1 and 3."""
    p = _PROGRESS
    p.update(objects)
    on = bool(latched_on(objects) and p.on_by_gripper)
    set_9 = bool(abs(volts(objects) - VOLTS_TARGET) <= VOLTS_TOL and not knob_held(objects))
    done = [on, bool(p.knob_gripped), set_9, bool(on and set_9 and others_untouched(objects) and psu_in_place(objects))]
    if "episode_time" in objects and done != p.reported:  # in a run, log each change: the run's step timeline
        p.reported = done
        got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
        shift = np.linalg.norm(objects["psu"].pos[:2] - np.array(PSU_XY))
        print(f"[{TAG}] t={objects['episode_time']:.2f}s  steps done: {got}  (knob at {volts(objects):.2f} V; power "
              f"{'on' if objects['power_on'] else 'off'}, button in {objects['button_travel'] * 1000:.1f} mm; supply "
              f"moved {shift * 1000:.1f} mm)", flush=True)
    return done


def _violation(objects) -> bool:
    """Scores 0: the supply off the table or knocked over, or a state that is not finite."""
    p = objects["psu"]
    if not np.all(np.isfinite(p.pos)) or p.pos[2] < TOP + FALLEN_Z:
        return True
    return bool(_rot(objects)[2, 2] < math.cos(TIPPED))


def success(objects) -> bool:
    """All four steps done, both arms back where they started with nothing in hand, the supply still, no violation."""
    return bool(all(steps_done(objects)) and not _violation(objects) and _home(objects, "left")
                and _home(objects, "right") and objects["psu"].resting())


def score(objects) -> float:
    """0.25 per step done (``STEPS``); 0 for any violation. Success scores 1.0."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: fingers closed, pointing at the panel, push the power button in past its click and let it come back to
# where the latch holds it; go home.
# Right arm: pointing at the panel, fingers open around the coarse voltage knob (closing across it, level), the pad
# tips just short of its skirt; grip it, roll the wrist 27 deg counter-clockwise (seen from the front: 12 V -> 9 V),
# let go, back off and go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (~20 N a finger)
KNOB_HOLD = -(2 * KNOB_R - SQUEEZE) / 2
KNOB_OPEN = -0.020  # the fingers 40 mm apart around the 26 mm knob
TIPS_SHORT = 0.0015  # m: the pad tips stop this far in front of the knob's skirt
TURN = -(VOLTS_START - VOLTS_TARGET) / VOLTS_PER_RAD  # rad about the knob's axis (the world's +x): -27 deg
TURN_EXTRA = math.radians(-0.8)  # more wrist roll than the knob should turn: the knob runs on ~0.8 deg
PRESS = 0.0078  # m: the fingertips sent this far past the button's face: it clicks at 3.5 mm and stops at 4.5; the arm
# gives about 2 mm against the button's spring
# The closed fingertips are 15.5 mm wide (along the pad-width axis) with rounded ends; tilted, their upper edge leads.
# It touches the button: PRESS_EDGE from the tips' middle along the pad width, toward the panel.
PRESS_EDGE = 0.0055
PRESS_ROLL = 0.0  # rad: the left gripper's roll about its pointing axis while pressing (its body clear of the table)
PRESS_TILT = math.radians(25.0)  # rad: the left gripper points this far below level (its body clear of the table)
KNOB_ROLL = 0.0  # rad: the right gripper's roll before turning (fingers closing level at 0)
STANDOFF = 0.06  # m: approach and retreat this far in front, along the gripper's pointing


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

_X, _Y, _Z = np.eye(3)
_DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
_FORWARD = axis_angle_matrix(_Y, -math.pi / 2) @ _DOWN  # pointing +x (at the panel), fingers closing along y


def facing_panel(roll: float = 0.0, tilt: float = 0.0) -> np.ndarray:
    """The gripper pointing at the panel (+x), turned ``roll`` about its pointing axis (+: clockwise seen from behind,
    like the knobs) and then ``tilt`` down."""
    return axis_angle_matrix(_Y, tilt) @ axis_angle_matrix(_X, roll) @ _FORWARD


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


def left_plan() -> list[Waypoint]:
    """Press the power button in past its click, let it back to the latch, go home."""
    rot = facing_panel(PRESS_ROLL, PRESS_TILT)
    face = psu_point(BUTTON_AT - np.array([0.0, BUTTON_OUT, 0.0]))  # the button's face, released
    edge = rot @ _Y if (rot @ _Y)[0] > 0 else -(rot @ _Y)  # along the pad width, the end toward the panel
    touch = _tcp_with_tips(face - PRESS_EDGE * edge, 0.0, rot)  # the leading edge on the face's middle
    point = rot @ _Z
    push = touch + PRESS * _X  # the fingertips move straight in (+x), whatever their tilt
    return [
        Waypoint(2.5, touch - STANDOFF * point, 0.0, "in front of the button", rot=rot),
        Waypoint(1.2, touch - 0.004 * _X, 0.0, "up to the button", rot=rot),
        Waypoint(1.0, push, 0.0, "press it", rot=rot),
        Waypoint(0.5, push, 0.0, "hold it in", rot=rot),
        Waypoint(0.8, touch - 0.04 * _X, 0.0, "let it come back", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


LEFT_CLEAR_S = 2.5 + 1.2 + 1.0 + 0.5 + 0.8  # the left fingers are off the button


def right_plan(wait_s: float = LEFT_CLEAR_S) -> list[Waypoint]:
    """Grip the coarse voltage knob, turn it from 12 V to 9 V, let go, go home."""
    rot = facing_panel(KNOB_ROLL)
    turned = facing_panel(KNOB_ROLL + TURN - TURN_EXTRA)
    skirt = psu_point(KNOB_AT["Knob_V_Coarse"] - np.array([0.0, SKIRT_FRONT + TIPS_SHORT, 0.0]))
    grip = _tcp_with_tips(skirt, 0.0, rot)
    away = grip - STANDOFF * _X
    return [
        Waypoint(wait_s, _START_TCP["right"], FINGER_OPEN, "wait for the power", rot=_START_ROT["right"]),
        Waypoint(2.5, away, KNOB_OPEN, "in front of the knob", rot=rot),
        Waypoint(1.5, grip, KNOB_OPEN, "around the knob", rot=rot),
        Waypoint(0.8, grip, KNOB_HOLD, "grip the knob", rot=rot),
        Waypoint(2.5, grip, KNOB_HOLD, "turn it to 9 V", rot=turned),
        Waypoint(0.4, grip, KNOB_HOLD, "hold", rot=turned),
        Waypoint(0.8, grip, KNOB_OPEN, "let go", rot=turned),
        Waypoint(1.0, away, KNOB_OPEN, "back off", rot=turned),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]


def demo():
    """The left arm switches the supply on; then the right arm sets it to 9 V. Both go home."""
    right = right_plan()
    left = left_plan()
    left.append(Waypoint(max(sum(w.duration for w in right) - sum(w.duration for w in left), 0.5) + 1.0,
                         _START_TCP["left"], FINGER_OPEN, "wait", rot=_START_ROT["left"]))
    return {"left": left, "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
