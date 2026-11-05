"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_welder_insert.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "welder_insert.py")
T = TASK.module

START = np.array(T.WELDER_START)
START_QUAT = np.array(T._yaw_xyzw(T.START_YAW))
PLACED = np.array(T.WELDER_POS)
PLACED_QUAT = np.array(T._yaw_xyzw(T.WELDER_YAW))
CELL_UP = np.array([0.0, 0.0, 0.0, 1.0])
SEATED_QUAT = np.array(T._qmul(tuple(PLACED_QUAT), T.CELL_IN_WELDER_XYZW))
BACKWARDS_QUAT = np.array(T._qmul(tuple(np.array(T._yaw_xyzw(T.WELDER_YAW + math.pi))), T.CELL_IN_WELDER_XYZW))
CELL_LIFTED = np.array(T.CELL_POS[0]) + (0.0, 0.0, 0.06)
DOWN = np.array([0.0, 0.0, -1.0])


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.01), contact=10.0, opening=0.01)


def _home(side):
    return _gripper(T._START_TCP[side])


def _grip_point(origin, yaw):
    return np.asarray(origin, float) + T._rz(yaw) @ np.array([T.WELDER_GRIP_AT, 0.0, T.WELDER_HEIGHT / 2])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.origin, self.quat = START.copy(), START_QUAT.copy()
        self.cell, self.cell_quat = np.array(T.CELL_POS[0]), CELL_UP.copy()
        self.lid, self.latch = -T.LID_OPEN_ANGLE, 0.0
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        housing = self.origin + T.quat_xyzw_to_matrix(self.quat) @ T.BODY_SHIFT
        out = {
            "welder": _state(housing, self.quat, self.speed),
            "holder": _state(T.HOLDER_POS),
            "housing": T.PartPose(pos=housing, quat_xyzw=self.quat),
            "lid_angle": self.lid,
            "latch_travel": self.latch,
            "left_gripper": self.left,
            "right_gripper": self.right,
            "episode_time": self.t,
            "cell_1": _state(self.cell, self.cell_quat),
        }
        for k in range(1, 4):
            out[f"cell_{k + 1}"] = _state(T.CELL_POS[k])
        return out

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

    def place_welder(self, origin=PLACED, quat=PLACED_QUAT, yaw=T.WELDER_YAW):
        self.step(left=_holding(_grip_point(START, T.START_YAW)))
        lifted = START + (0.0, 0.0, 0.03)
        self.step(origin=lifted, left=_holding(_grip_point(lifted, T.START_YAW)))
        self.step(origin=np.asarray(origin, float).copy(), quat=np.asarray(quat, float).copy(),
                  left=_holding(_grip_point(origin, yaw)))

    def take_cell(self):
        self.step(right=_holding(np.array(T.CELL_POS[0]) + (0.0, 0.0, 0.02)))
        self.step(cell=CELL_LIFTED.copy(), right=_holding(CELL_LIFTED + (0.0, 0.0, 0.02)))

    def insert_cell(self, quat=SEATED_QUAT):
        self.step(cell=T.SEAT.copy(), cell_quat=np.asarray(quat, float).copy(), right=_home("right"))

    def close_lid(self):
        self.step(lid=0.0, latch=0.0)

    def let_go(self):
        self.step(left=_home("left"))

    def full_course(self):
        self.place_welder()
        self.take_cell()
        self.insert_cell()
        self.close_lid()
        self.let_go()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"welder", "holder", "cell_1", "cell_2", "cell_3", "cell_4"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_battery_taken_out_is_a_quarter():
    e = Episode()
    e.take_cell()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_battery_gripped_in_its_cup_is_not_taken_out():
    e = Episode()
    e.step(right=_holding(np.array(T.CELL_POS[0]) + (0.0, 0.0, 0.02)))
    assert e.done()[0] is False


def test_battery_knocked_up_without_a_gripper_is_not_taken_out():
    e = Episode()
    e.step(cell=CELL_LIFTED.copy())
    assert e.done()[0] is False


def test_battery_in_the_bay_is_a_step():
    e = Episode()
    e.place_welder()
    e.take_cell()
    e.insert_cell()
    assert e.done()[:3] == [True, True, False] and e.step()[0] == 0.5


def test_battery_in_backwards_is_not_in_the_bay():
    e = Episode()
    e.place_welder()
    e.take_cell()
    e.insert_cell(quat=BACKWARDS_QUAT)
    assert e.done()[1] is False


def test_battery_resting_on_the_rim_is_not_in_the_bay():
    e = Episode()
    e.place_welder()
    e.take_cell()
    e.step(cell=T.SEAT + (0.0, 0.0, 0.01), cell_quat=SEATED_QUAT.copy())
    assert e.done()[1] is False


def test_lid_latched_with_the_battery_in_is_a_step():
    e = Episode()
    e.place_welder()
    e.take_cell()
    e.insert_cell()
    e.close_lid()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_lid_shut_over_an_empty_bay_earns_nothing():
    e = Episode()
    e.place_welder()
    e.close_lid()
    assert e.done()[2] is False


def test_lid_shut_but_not_caught_by_the_latch_is_not_latched():
    e = Episode()
    e.place_welder()
    e.take_cell()
    e.insert_cell()
    e.step(lid=0.0, latch=0.0012)
    assert e.done()[2] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.0) == (1.0, True)


def test_welder_still_held_is_not_put_down():
    e = Episode()
    e.place_welder()
    assert e.done()[3] is False


def test_welder_put_back_unturned_is_not_put_down():
    e = Episode()
    e.place_welder(origin=START, quat=START_QUAT, yaw=T.START_YAW)
    e.let_go()
    assert e.done()[3] is False


def test_welder_on_its_side_is_not_put_down():
    e = Episode()
    e.place_welder()
    on_side = np.array(T._qmul(tuple(PLACED_QUAT), (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))))
    e.step(quat=on_side, left=_home("left"))
    assert e.done()[3] is False


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_welder_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_welder_off_the_table_scores_zero():
    e = Episode()
    e.take_cell()
    assert e.step(origin=np.array([0.3, 0.4, T.TOP - 0.3]))[0] == 0.0


def test_cell_off_the_table_scores_zero():
    e = Episode()
    e.take_cell()
    assert e.step(cell=np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
