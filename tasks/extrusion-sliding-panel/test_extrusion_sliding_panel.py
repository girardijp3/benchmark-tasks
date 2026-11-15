"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_extrusion_sliding_panel.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "extrusion_sliding_panel.py")
T = TASK.module
UPRIGHT = np.eye(3)


def _rx(a):
    return T.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), a)


def _state(pos, rot):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos.copy(), quat_xyzw=np.asarray(T.ik.matrix_to_quat_xyzw(rot), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _gripper(tcp, axis, contact=0.0, opening=0.09):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side="left"):
    return _gripper(T._START_TCP[side], T._START_ROT[side][:, 2])


def _on_knob(pos, contact=20.0):
    return _gripper(T._knob_tcp(pos), T.GRIP_ROT[:, 2], contact=contact, opening=0.016)


def at(y_travel, z=0.0, x=0.0):
    """The panel's origin after it has travelled ``y_travel`` along the slots."""
    return T.PANEL_START + np.array([x, y_travel, z])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos, self.rot = T.PANEL_START.copy(), UPRIGHT.copy()
        self.frame_pos = T.FRAME_POS.copy()
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        return {"frame": _state(self.frame_pos, np.eye(3)), "panel": _state(self.pos, self.rot),
                "left_gripper": self.left, "right_gripper": self.right, "episode_time": self.t}

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

    def pull_to(self, y, rot=None, z=0.0, x=0.0):
        for k in np.linspace(self.pos[1] - T.PANEL_START[1], y, 5):
            p = at(float(k), z, x)
            self.step(pos=p, rot=UPRIGHT if rot is None else rot, left=_on_knob(p))

    def let_go(self):
        self.step(left=_home("left"))

    def full_course(self):
        self.pull_to(T.TRAVEL)
        self.let_go()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert TASK.names == ("frame", "panel") and TASK.free_objects == ("panel",)
    assert T.SUCCESS_HOLD_S == 0.0


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_slid_without_a_grip_scores_nothing():
    e = Episode()
    e.step(pos=at(T.TRAVEL))
    assert e.done() == [False, False, False, False]


def test_started_is_a_step():
    e = Episode()
    e.pull_to(0.015)
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.2


def test_over_half_way_is_a_step():
    e = Episode()
    e.pull_to(0.035)
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.4


def test_at_the_stop_with_the_hand_on_the_knob_is_not_let_go():
    e = Episode()
    e.pull_to(T.TRAVEL)
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.6


def test_short_of_the_stop_is_not_at_the_stop():
    e = Episode()
    e.pull_to(T.TRAVEL - 0.006)
    e.let_go()
    assert e.done() == [True, True, False, False]


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.1) == (1.0, True)


def test_a_gripper_beside_the_knob_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(left=_gripper(T._knob_tcp(e.pos) + (0.0, 0.0, 0.015), T.GRIP_ROT[:, 2]))
    assert e.done()[3] is False


def test_panel_cocked_in_the_slots_is_not_in_them():
    e = Episode()
    e.pull_to(T.TRAVEL, rot=T._rz(math.radians(4.0)))
    e.let_go()
    assert e.done()[2] is False


def test_panel_leaning_is_not_in_the_slots():
    e = Episode()
    e.pull_to(T.TRAVEL, rot=_rx(math.radians(4.0)))
    e.let_go()
    assert e.done()[2] is False


def test_panel_out_of_the_slots_sideways_is_not_in_them():
    e = Episode()
    e.pull_to(T.TRAVEL, x=0.004)
    e.let_go()
    assert e.done()[2] is False


def test_success_is_timed_in_the_task_file():
    e = Episode()
    e.pull_to(T.TRAVEL)
    e.let_go()
    assert e.step(0.8)[1] is False and T.score(e.objects()) <= 0.8
    assert e.step(0.3)[1] is True and T.score(e.objects()) == 1.0


def test_no_full_score_while_a_condition_is_missing():
    e = Episode()
    e.pull_to(T.TRAVEL)
    assert e.step(2.0)[1] is False  # still holding the knob
    e2 = Episode()
    e2.pull_to(T.TRAVEL - 0.006)
    e2.let_go()
    assert e2.step(2.0)[1] is False


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    e.step(0.6)
    e.step(pos=at(T.TRAVEL - 0.01))
    e.step(0.1)
    e.step(pos=at(T.TRAVEL))
    assert e.step(0.6)[1] is False
    assert e.step(0.5)[1] is True


def test_panel_creeping_back_restarts_the_second():
    e = Episode()
    e.full_course()
    for k in range(40):
        e.step(pos=at(T.TRAVEL - 0.0002 * k))
    assert e.step()[1] is False


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    away = _gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1), np.array([0.0, 0.0, -1.0]))
    assert e.step(1.2, left=away) == (1.0, True)


def test_panel_pushed_back_loses_the_steps():
    e = Episode()
    e.full_course()
    e.step(pos=at(0.02))
    assert e.done() == [True, False, False, False]


def test_panel_tipped_over_scores_zero():
    e = Episode()
    e.pull_to(0.02)
    assert e.step(pos=at(0.02, 0.03), rot=_rx(math.radians(60.0)))[0] == 0.0


def test_panel_off_the_table_scores_zero():
    e = Episode()
    e.pull_to(0.02)
    assert e.step(pos=np.array([0.3, 0.6, T.TOP - 0.3]))[0] == 0.0


def test_frame_torn_loose_scores_zero():
    e = Episode()
    e.full_course()
    assert e.step(frame_pos=T.FRAME_POS + (0.02, 0.0, 0.0))[0] == 0.0


def test_not_finite_scores_zero():
    e = Episode()
    e.pull_to(0.02)
    assert e.step(pos=np.array([math.nan, 0.0, 1.0]))[0] == 0.0


def test_demo_uses_the_left_arm_only_and_is_reachable():
    assert set(T.demo()) == {"left"}
    plan = ArmPlan(T.demo()["left"], BASES["left"])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    assert sum(w.duration for w in T.demo()["left"]) < 30.0
