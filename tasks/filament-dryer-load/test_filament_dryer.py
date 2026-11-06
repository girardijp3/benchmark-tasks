"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_filament_dryer.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "filament_dryer.py")
T = TASK.module

UPRIGHT = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])
FORWARD = np.array([1.0, 0.0, 0.0])
SPOOL_LIFTED = np.array([*T.SPOOL_START[:2], T.SPOOL_START[2] + 0.13])


def _quat_about(axis, angle):
    s = math.sin(angle / 2)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)])


def _gripper(tcp, contact=0.0, opening=0.09, axis=DOWN):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _holding_spool(centre):
    """The right gripper closed on the spool's flanges at its top (pointing forward and down)."""
    return _gripper(T._spool_tcp(centre), contact=20.0, opening=0.057, axis=T.RIGHT_POINT)


def _home(side):
    return _gripper(T._START_TCP[side])


def _pushing_lid(angle):
    """The left gripper, fingers closed, against the hood's outer top face with the hood at ``angle``."""
    return _gripper(T._lid_tcp(angle), axis=FORWARD, opening=0.0)


def _state(pos, quat=UPRIGHT, speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.dryer, self.dryer_quat = T.DRYER_POS.copy(), UPRIGHT.copy()
        self.spool, self.spool_quat = T.SPOOL_START.copy(), UPRIGHT.copy()
        self.lid, self.latch = T.LID_OPEN, 0.0
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        return {
            "dryer": _state(self.dryer, self.dryer_quat),
            "stand": _state(T.STAND_POS),
            "spool": _state(self.spool, self.spool_quat, self.speed),
            "lid_angle": self.lid,
            "latch_travel": self.latch,
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

    def take_spool(self):
        self.step(right=_holding_spool(T.SPOOL_START))
        self.step(spool=SPOOL_LIFTED.copy(), right=_holding_spool(SPOOL_LIFTED))

    def seat_spool(self, at=None, quat=UPRIGHT):
        at = T.SEAT.copy() if at is None else np.asarray(at, float)
        self.step(spool=at + (0, 0, 0.05), right=_holding_spool(at + (0, 0, 0.05)))
        self.step(spool=at, spool_quat=np.asarray(quat, float), right=_home("right"))

    def swing_lid(self):
        for a in (-80, -60, -40):
            self.step(lid=math.radians(a), left=_pushing_lid(math.radians(a)))

    def shut_lid(self, latch=0.0):
        self.step(lid=0.0, latch=latch, left=_pushing_lid(0.0))
        self.step(left=_home("left"))

    def full_course(self):
        self.take_spool()
        self.seat_spool()
        self.swing_lid()
        self.shut_lid()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"dryer", "stand", "spool"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_spool_taken_off_is_a_quarter():
    e = Episode()
    e.take_spool()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_spool_gripped_on_the_stand_is_not_taken_off():
    e = Episode()
    e.step(right=_holding_spool(T.SPOOL_START))
    assert e.done()[0] is False


def test_spool_knocked_up_without_a_gripper_is_not_taken_off():
    e = Episode()
    e.step(spool=SPOOL_LIFTED.copy())
    assert e.done()[0] is False


def test_spool_on_the_rollers_is_a_step():
    e = Episode()
    e.take_spool()
    e.seat_spool()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_spool_still_held_is_not_on_the_rollers():
    e = Episode()
    e.take_spool()
    e.step(spool=T.SEAT.copy(), right=_holding_spool(T.SEAT))
    assert e.done()[1] is False


def test_spool_on_one_roller_is_not_on_the_rollers():
    e = Episode()
    e.take_spool()
    e.seat_spool(at=T.SEAT + (0.02, 0.0, 0.01))
    assert e.done()[1] is False


def test_spool_turned_across_the_dryer_is_not_on_the_rollers():
    e = Episode()
    e.take_spool()
    e.seat_spool(quat=_quat_about((0, 0, 1), math.radians(30)))
    assert e.done()[1] is False


def test_spool_in_the_stand_after_all_is_not_on_the_rollers():
    e = Episode()
    e.take_spool()
    e.seat_spool(at=T.SPOOL_START)
    assert e.done()[:2] == [True, False]


def test_lid_swung_over_by_a_gripper_is_a_step():
    e = Episode()
    e.take_spool()
    e.seat_spool()
    e.swing_lid()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_lid_moved_without_a_gripper_is_not_swung_over():
    e = Episode()
    e.take_spool()
    e.seat_spool()
    e.step(lid=math.radians(-30))
    assert e.done()[2] is False


def test_lid_shut_and_caught_is_a_step():
    e = Episode()
    e.take_spool()
    e.seat_spool()
    e.swing_lid()
    e.shut_lid()
    assert e.done() == [True, True, True, True]


def test_lid_shut_but_not_caught_is_not_caught():
    e = Episode()
    e.take_spool()
    e.seat_spool()
    e.swing_lid()
    e.shut_lid(latch=0.0012)
    assert e.done() == [True, True, True, False]


def test_lid_shut_over_an_empty_dryer_earns_nothing():
    e = Episode()
    e.swing_lid()
    e.shut_lid()
    assert e.done() == [False, False, False, False] and e.step()[0] == 0.0


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.0) == (1.0, True)


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(left=_gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_dryer_pushed_away_is_not_success():
    e = Episode()
    e.full_course()
    moved = T.DRYER_POS + (0.05, 0.0, 0.0)
    assert e.step(dryer=moved, spool=T.SEAT + (0.05, 0.0, 0.0)) == (1.0, False)


def test_moving_spool_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_dryer_off_the_table_scores_zero():
    e = Episode()
    e.take_spool()
    assert e.step(dryer=np.array([0.3, 0.4, T.TOP - 0.3]))[0] == 0.0


def test_spool_off_the_table_scores_zero():
    e = Episode()
    e.take_spool()
    assert e.step(spool=np.array([0.3, -0.5, T.TOP - 0.3]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
