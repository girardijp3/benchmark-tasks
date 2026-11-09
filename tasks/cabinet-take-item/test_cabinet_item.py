"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_cabinet_item.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "cabinet_item.py")
T = TASK.module

UPRIGHT = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])
METER_START = np.array(T.METER_POS)
METER_PLACED = np.array([*T.PLACE_XY, T.TOP])
METER_OUT = np.array([T.CABINET_XY[0] - 0.12, T.CABINET_XY[1], T.METER_POS[2] + 0.008])  # drawn out, in hand


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.01), contact=10.0, opening=0.01)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _bar(angle):
    return T._door_point(T.HANDLE_IN_DOOR, angle)


class Episode:
    def __init__(self):
        self.t = 0.0
        self.cabinet = np.array(T.CABINET_POS)
        self.cabinet_quat = UPRIGHT.copy()
        self.angle = 0.0
        self.meter, self.meter_quat = METER_START.copy(), UPRIGHT.copy()
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        h = T.door_hinge_world() + (self.cabinet[:2] - T.CABINET_XY)
        return {
            "cabinet": _state(self.cabinet, self.cabinet_quat, self.speed),
            "multimeter": _state(self.meter, self.meter_quat),
            "door": T.PartPose(pos=np.array([h[0], h[1], self.cabinet[2]]), quat_xyzw=np.array(T._yaw_xyzw(-self.angle))),
            "door_angle": self.angle,
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

    def open_door(self, angle=T.DOOR_OPEN):
        self.step(left=_holding(_bar(0.0)))
        self.step(angle=angle, left=_holding(_bar(angle)))

    def grip_meter(self):
        self.step(right=_holding(self.meter + (0, 0, T.METER_GRIP_Z)))

    def take_meter(self):
        self.grip_meter()
        self.step(meter=METER_OUT.copy(), right=_holding(METER_OUT + (0, 0, T.METER_GRIP_Z)))

    def place_meter(self, pos=METER_PLACED, quat=UPRIGHT):
        self.step(meter=np.asarray(pos, float).copy(), meter_quat=np.asarray(quat, float).copy(), right=_home("right"))

    def close_door(self):
        self.step(angle=0.0, left=_holding(_bar(0.0)))
        self.step(left=_home("left"))

    def full_course(self):
        self.open_door()
        self.take_meter()
        self.place_meter()
        self.close_door()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"cabinet", "multimeter"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_door_opened_by_its_handle_is_a_quarter():
    e = Episode()
    e.open_door()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_door_opened_a_little_is_not_opened():
    e = Episode()
    e.open_door(angle=math.radians(30))
    assert e.done()[0] is False


def test_door_swung_open_without_the_handle_is_not_opened():
    e = Episode()
    e.step(angle=T.DOOR_OPEN)
    assert e.done()[0] is False


def test_multimeter_taken_out_is_a_step():
    e = Episode()
    e.open_door()
    e.take_meter()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_multimeter_gripped_inside_is_not_taken_out():
    e = Episode()
    e.open_door()
    e.grip_meter()
    e.step(meter=METER_START + (-0.03, 0, 0.008), right=_holding(METER_START + (-0.03, 0, 0.008 + T.METER_GRIP_Z)))
    assert e.done()[1] is False


def test_multimeter_knocked_out_without_a_gripper_is_not_taken_out():
    e = Episode()
    e.open_door()
    e.step(meter=METER_PLACED.copy())
    assert e.done()[1] is False


def test_multimeter_on_the_bench_is_a_step():
    e = Episode()
    e.open_door()
    e.take_meter()
    e.place_meter()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_multimeter_lying_on_the_bench_counts():
    e = Episode()
    e.open_door()
    e.take_meter()
    on_back = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))  # lying on its display side
    e.place_meter(pos=METER_PLACED + (0, T.MM_T / 2, T.MM_T / 2), quat=on_back)
    assert e.done()[2] is True


def test_multimeter_still_held_is_not_on_the_bench():
    e = Episode()
    e.open_door()
    e.take_meter()
    e.step(meter=METER_PLACED.copy(), right=_holding(METER_PLACED + (0, 0, T.METER_GRIP_Z)))
    assert e.done()[2] is False


def test_multimeter_on_top_of_the_cabinet_is_not_on_the_bench():
    e = Episode()
    e.open_door()
    e.take_meter()
    top = np.array([T.CABINET_XY[0] + 0.12, T.CABINET_XY[1], T.TOP + T.CAB_H])
    e.place_meter(pos=top)
    assert e.done()[2] is False


def test_multimeter_put_back_inside_is_not_on_the_bench():
    e = Episode()
    e.open_door()
    e.take_meter()
    e.place_meter(pos=METER_START)
    assert e.done()[1:3] == [True, False]


def test_door_shut_and_caught_is_a_step():
    e = Episode()
    e.open_door()
    e.take_meter()
    e.close_door()
    assert e.done() == [True, True, False, True] and e.step()[0] == 0.75


def test_door_shut_but_still_held_is_not_caught():
    e = Episode()
    e.full_course()
    e.step(left=_holding(_bar(0.0)))
    assert e.done()[3] is False


def test_door_resting_against_the_catch_is_not_caught():
    e = Episode()
    e.full_course()
    e.step(angle=math.radians(2.5))
    assert e.done()[3] is False


def test_door_shut_with_the_multimeter_never_taken_out_earns_nothing():
    e = Episode()
    e.open_door()
    e.close_door()
    assert e.done() == [True, False, False, False]


def test_door_shut_on_a_shoved_cabinet_is_not_a_step():
    e = Episode()
    e.full_course()
    e.step(cabinet=np.array(T.CABINET_POS) + (0.05, 0, 0))
    assert e.done()[3] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.0) == (1.0, True)


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_cabinet_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_multimeter_off_the_table_scores_zero():
    e = Episode()
    e.open_door()
    e.take_meter()
    assert e.step(meter=np.array([0.3, -0.5, T.TOP - 0.3]))[0] == 0.0


def test_cabinet_tipped_over_scores_zero():
    e = Episode()
    e.full_course()
    tipped = (0.0, math.sin(math.radians(45)), 0.0, math.cos(math.radians(45)))
    assert e.step(cabinet_quat=np.array(tipped))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
