"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_screwdriver_case.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "screwdriver_case.py")
T = TASK.module

CASE_QUAT = np.array(T._yaw_xyzw(T.CASE_YAW))
REST_QUAT = np.array(T._matrix_xyzw(T.SD_REST_R))
SEATED_QUAT = np.array(T._matrix_xyzw(T.SEAT_R_W))
BACKWARDS_R = T.CASE_R  # tip toward the handle pocket: the screwdriver turned half a turn from seated
DOWN = np.array([0.0, 0.0, -1.0])


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.005), contact=10.0, opening=0.026)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _grip_point(pos, quat):
    return np.asarray(pos, float) + T.quat_xyzw_to_matrix(np.asarray(quat, float)) @ T.GRIP_LOCAL


class Episode:
    def __init__(self):
        self.t = 0.0
        self.sd, self.sd_quat = T.SD_REST_POS.copy(), REST_QUAT.copy()
        self.case, self.case_quat = np.array([*T.CASE_XY, T.TOP]), CASE_QUAT.copy()
        self.lid, self.latch = -T.LID_OPEN_ANGLE, 0.0
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        return {
            "case": _state(self.case, self.case_quat, self.speed),
            "screwdriver": _state(self.sd, self.sd_quat),
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

    def pick(self, lift=0.07):
        self.step(right=_holding(_grip_point(self.sd, self.sd_quat)))
        self.sd = self.sd + (0.0, 0.0, lift)
        self.step(right=_holding(_grip_point(self.sd, self.sd_quat)))

    def lay_in(self, pos=None, quat=SEATED_QUAT):
        pos = T.SEAT_POS_W.copy() if pos is None else np.asarray(pos, float)
        self.step(sd=pos, sd_quat=np.asarray(quat, float), right=_holding(_grip_point(pos, quat)))
        self.step(right=_home("right"))

    def close_lid(self, angle=0.0, latch=0.0):
        self.step(lid=angle, latch=latch, left=_home("left"))

    def full_course(self):
        self.pick()
        self.lay_in()
        self.close_lid()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"case", "screwdriver"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_picked_up_is_a_quarter():
    e = Episode()
    e.pick()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_gripped_on_the_bench_is_not_picked_up():
    e = Episode()
    e.step(right=_holding(_grip_point(e.sd, e.sd_quat)))
    assert e.done()[0] is False


def test_knocked_up_without_a_gripper_is_not_picked_up():
    e = Episode()
    e.step(sd=e.sd + (0.0, 0.0, 0.07))
    assert e.done()[0] is False


def test_in_its_cutout_is_a_step():
    e = Episode()
    e.pick()
    e.lay_in()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_seated_within_play_counts():
    e = Episode()
    e.pick()
    e.lay_in(pos=T.SEAT_POS_W + T.CASE_R @ np.array([0.0, 0.0015, 0.0008]))
    assert e.done()[1] is True


def test_still_held_in_its_cutout_is_not_in():
    e = Episode()
    e.pick()
    e.step(sd=T.SEAT_POS_W.copy(), sd_quat=SEATED_QUAT.copy(), right=_holding(_grip_point(T.SEAT_POS_W, SEATED_QUAT)))
    assert e.done()[1] is False


def test_backwards_is_not_in_its_cutout():
    e = Episode()
    e.pick()
    # turned half a turn about the vertical through the cutout's middle: handle where the tip goes
    mid = T.case_point(np.array([0.0, 0.0, 0.0]))
    pos = mid + T._rz(math.pi) @ (T.SEAT_POS_W - mid)
    e.lay_in(pos=pos, quat=np.array(T._matrix_xyzw(BACKWARDS_R)))
    assert e.done()[1] is False


def test_resting_on_the_foam_is_not_in_its_cutout():
    e = Episode()
    e.pick()
    e.lay_in(pos=T.SEAT_POS_W + (0.0, 0.0, 0.012))
    assert e.done()[1] is False


def test_lying_in_the_case_without_a_pick_does_not_count():
    e = Episode()
    e.step(sd=T.SEAT_POS_W.copy(), sd_quat=SEATED_QUAT.copy())
    assert e.done()[1] is False


def test_lid_closed_over_it_is_a_step():
    e = Episode()
    e.pick()
    e.lay_in()
    e.close_lid(angle=math.radians(-3), latch=-0.0017)  # resting on the clasp's ramp, not caught
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_lid_latched_is_the_last_step():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]


def test_lid_shut_over_an_empty_case_earns_nothing():
    e = Episode()
    e.close_lid()
    assert e.done()[2:] == [False, False]


def test_lid_shut_with_the_slider_out_is_not_latched():
    e = Episode()
    e.pick()
    e.lay_in()
    e.close_lid(angle=0.0, latch=-0.0012)
    assert e.done()[3] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.step(1.0) == (1.0, True)


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(left=_gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_case_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_case_pushed_away_is_not_success():
    e = Episode()
    e.full_course()
    d = np.array([0.05, 0.0, 0.0])
    assert e.step(case=e.case + d, sd=e.sd + d) == (1.0, False)


def test_screwdriver_off_the_table_scores_zero():
    e = Episode()
    e.pick()
    assert e.step(sd=np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


def test_case_tipped_over_scores_zero():
    e = Episode()
    e.pick()
    on_end = np.array(T.ik.matrix_to_quat_xyzw(T.CASE_R @ T.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.pi / 2)))
    assert e.step(case_quat=on_end)[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
