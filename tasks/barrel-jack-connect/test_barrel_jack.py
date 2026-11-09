"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_barrel_jack.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "barrel_jack.py")
T = TASK.module

DOWN = np.array([0.0, 0.0, -1.0])
JACK_ROT = np.asarray(T.JACK_START_ROT, float)
PLUG_ROT = np.asarray(T.PLUG_START_ROT, float)
JACK_LYING = np.asarray(T.JACK_START, float)
JACK_HELD = np.asarray(T.MATE, float)


def _quat(r):
    return np.array(T._matrix_xyzw(r))


def _jack(pos):
    return T.PartPose(pos=np.asarray(pos, float), quat_xyzw=_quat(JACK_ROT))


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.009), contact=10.0, opening=0.008)


def _home(side):
    return _gripper(T._START_TCP[side])


def _plug_in(mouth, depth=0.0, off=0.0, tilt_deg=0.0):
    """The plug's pose with its overmold face ``depth`` short of fully in the jack whose mouth is at ``mouth``, ``off``
    beside the bore's line, tilted ``tilt_deg`` about the jack's own z axis."""
    face = np.asarray(mouth, float) + JACK_ROT @ np.array([depth, off, 0.0])
    rot = JACK_ROT @ T._rz(math.pi) @ T._rz(math.radians(tilt_deg))
    return T.PartPose(pos=face, quat_xyzw=_quat(rot))


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.jack = _jack(JACK_LYING)
        self.plug = T.PartPose(pos=np.asarray(T.PLUG_START, float), quat_xyzw=_quat(PLUG_ROT))
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        return {
            "wall": _state(T.WALL.pos, T.WALL.rot_xyzw),
            "adapter": _state(T.ADAPTER.pos, T.ADAPTER.rot_xyzw),
            "dimmer": _state(T.DIMMER.pos, T.DIMMER.rot_xyzw),
            "jack": self.jack,
            "plug": self.plug,
            "jack_vel": np.array([self.speed, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "plug_vel": np.array([self.speed, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "left_gripper": self.left,
            "right_gripper": self.right,
            "episode_time": self.t,
        }

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            setattr(self, k, v)
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def done(self):
        return T.steps_done(self.objects())

    def lift_jack(self):
        self.step(jack=_jack(JACK_HELD), left=_holding(JACK_HELD - T.JACK_GRIP_AT * T.JACK_AXIS))

    def lift_plug(self):
        up = T.PartPose(pos=np.asarray(T.PLUG_START, float) + (0, 0, 0.05), quat_xyzw=_quat(PLUG_ROT))
        self.step(plug=up, right=_holding(up.pos - T.PLUG_GRIP_AT * T.PLUG_AXIS))

    def push_in(self, **plug):
        plugged = _plug_in(JACK_HELD, **plug)
        self.step(plug=plugged, right=_holding(plugged.pos - T.PLUG_GRIP_AT * T.PLUG_AXIS))
        self.step(right=_home("right"))

    def lay_down(self, **plug):
        self.step(jack=_jack(JACK_LYING), plug=_plug_in(JACK_LYING, **plug), left=_home("left"))

    def full_course(self, **plug):
        self.lift_jack()
        self.lift_plug()
        self.push_in(**plug)
        self.lay_down(**plug)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert {o.name for o in T.OBJECTS} == {"wall", "adapter", "dimmer"}


def test_layout_is_on_the_bench_and_apart():
    assert JACK_LYING[2] == pytest.approx(T.TOP + T.JACK_R, abs=1e-4)  # lying on a flat
    assert T.PLUG_START[2] == pytest.approx(T.TOP + T.OVERMOLD_R, abs=1e-4)
    assert JACK_ROT[:, 0] @ T.JACK_AXIS == pytest.approx(1.0) and PLUG_ROT[:, 0] @ T.PLUG_AXIS == pytest.approx(1.0)
    jack_grip, plug_grip = T.JACK_GRIP, T.PLUG_GRIP
    assert np.linalg.norm((jack_grip - plug_grip)[:2]) > 0.15  # the two picks are far enough apart for both wrists
    for pos, _ in (*T._AD_LINKS[5:], *T._DIM_LINKS[2:]):  # the cords lie on the bench
        assert T.TOP + 0.001 < pos[2] < T.TOP + 0.006


def test_start_scores_zero():
    assert Episode().step() == (0.0, False)


def test_jack_lifted_is_a_quarter():
    e = Episode()
    e.lift_jack()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_plug_lifted_is_a_quarter():
    e = Episode()
    e.lift_plug()
    assert e.done() == [False, True, False, False] and e.step()[0] == 0.25


def test_plugged_in_by_hand_is_three_quarters():
    e = Episode()
    e.lift_jack()
    e.lift_plug()
    e.push_in()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.step(1.0) == (1.0, True)


def test_a_slightly_loose_plug_still_counts():
    e = Episode()
    e.full_course(depth=0.0008, off=0.0003, tilt_deg=3.0)
    assert e.step(1.0) == (1.0, True)


@pytest.mark.parametrize("plug", [{"depth": 0.0015}, {"off": 0.0008}, {"tilt_deg": 7.0}])
def test_plug_not_in_far_enough_or_straight_enough(plug):
    e = Episode()
    e.full_course(**plug)
    assert e.done()[2:] == [False, False]
    assert e.step() == (0.5, False)


def test_plug_that_went_in_without_a_hand_never_counts():
    e = Episode()
    e.lift_jack()
    e.step(plug=_plug_in(JACK_HELD))  # in, with no gripper on the plug (pulled in by its cord, or the jack shoved on)
    e.lay_down()
    assert e.done() == [True, False, False, False]


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(left=_gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_connectors_are_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_jack_still_held_misses_the_last_step():
    e = Episode()
    e.full_course()
    e.step(left=_holding(JACK_LYING - T.JACK_GRIP_AT * T.JACK_AXIS))
    assert e.done() == [True, True, True, False]


def test_connectors_not_laid_down_miss_the_last_step():
    e = Episode()
    e.lift_jack()
    e.lift_plug()
    e.push_in()
    e.step(left=_home("left"))  # let go in the air (as if they hung there)
    assert e.done()[3] is False


def test_connector_off_the_bench_scores_zero():
    e = Episode()
    e.lift_jack()
    assert e.step(jack=_jack(np.array([0.3, 0.4, T.TOP - 0.3])))[0] == 0.0


def test_state_not_finite_scores_zero():
    e = Episode()
    e.full_course()
    assert e.step(plug=T.PartPose(pos=np.array([np.nan, 0.0, 0.0]), quat_xyzw=_quat(PLUG_ROT)))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
