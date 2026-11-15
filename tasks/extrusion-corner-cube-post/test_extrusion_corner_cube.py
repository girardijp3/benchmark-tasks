"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_extrusion_corner_cube.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "extrusion_corner_cube.py")
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


def _on_post(pos, contact=20.0):
    """The left gripper closed on the upper end of a post at ``pos`` (upright)."""
    return _gripper(T._tcp_for(pos), np.array([0.0, 0.0, -1.0]), contact=contact, opening=0.02)


def seat(dx=0.0, dy=0.0, low=T.FLOOR):
    """The post's origin (lower end) on the socket's axis, ``low`` above the table."""
    return np.array([T.CUBE_POS[0] + dx, T.CUBE_POS[1] + dy, T.TOP + low])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos, self.rot = T.POST_START.copy(), UPRIGHT.copy()
        self.cube_pos = T.CUBE_POS.copy()
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        return {"cube": _state(self.cube_pos, np.eye(3)), "rack": _state(T.RACK_POS, np.eye(3)),
                "post": _state(self.pos, self.rot), "left_gripper": self.left, "right_gripper": self.right,
                "episode_time": self.t}

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

    def carry(self, pos, rot=None):
        self.step(pos=np.asarray(pos, float), rot=UPRIGHT if rot is None else rot, left=_on_post(pos))

    def pick(self):
        self.step(left=_on_post(self.pos))
        self.carry(self.pos + (0, 0, 0.04))

    def insert(self, dx=0.0, dy=0.0, rot=None):
        self.carry(seat(dx, dy, T.CUBE_TOP + 0.01), rot)
        self.carry(seat(dx, dy, 0.02), rot)

    def lower(self, dx=0.0, dy=0.0, rot=None, low=T.FLOOR + 0.0006):
        self.carry(seat(dx, dy, low), rot)

    def let_go(self):
        self.step(left=_home("left"), pos=seat())

    def full_course(self):
        self.pick()
        self.insert()
        self.lower()
        self.let_go()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert TASK.names == ("cube", "rack", "post") and TASK.free_objects == ("post",)
    assert T.SUCCESS_HOLD_S == 0.0


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_picked_is_a_step():
    e = Episode()
    e.pick()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.2


def test_moved_without_a_grip_is_not_picked():
    e = Episode()
    e.step(pos=e.pos + (0, 0, 0.05))
    assert e.done() == [False, False, False, False]


def test_in_the_socket_is_a_step():
    e = Episode()
    e.pick()
    e.insert()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.4


def test_resting_on_the_rim_is_not_in_the_socket():
    e = Episode()
    e.pick()
    e.carry(seat(0.0, 0.0, T.CUBE_TOP + 0.001))
    assert e.done() == [True, False, False, False]


def test_post_8_mm_off_the_axis_is_not_in_the_socket():
    e = Episode()
    e.pick()
    e.insert(dx=0.008)
    assert e.done() == [True, False, False, False]


def test_post_leaning_in_the_socket_is_not_in_it():
    e = Episode()
    e.pick()
    e.carry(seat(0, 0, 0.02), _rx(math.radians(10.0)))
    assert e.done() == [True, False, False, False]


def test_seated_is_a_step():
    e = Episode()
    e.pick()
    e.insert()
    e.lower()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.6


def test_in_the_socket_but_held_up_is_not_seated():
    e = Episode()
    e.pick()
    e.insert()
    e.carry(seat(0, 0, T.FLOOR + 0.006))
    assert e.done() == [True, True, False, False]


def test_leaning_post_is_not_seated():
    e = Episode()
    e.pick()
    e.insert()
    e.carry(seat(0, 0, T.FLOOR + 0.0006), _rx(math.radians(5.0)))
    assert e.done()[2] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.1) == (1.0, True)


def test_still_holding_the_post_is_not_let_go():
    e = Episode()
    e.pick()
    e.insert()
    e.lower()
    e.step(2.0)
    assert e.done() == [True, True, True, False]
    assert e.step(1.5) == (0.6, False)


def test_a_gripper_beside_the_post_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(left=_gripper(e.pos + (0.0, 0.0, 0.08), np.array([0.0, 0.0, -1.0])))
    assert e.done()[3] is False


def test_success_is_timed_in_the_task_file():
    e = Episode()
    e.pick()
    e.insert()
    e.lower()
    e.step(left=_home("left"), pos=seat())
    assert e.step(0.9)[1] is False and T.score(e.objects()) <= 0.8
    assert e.step(0.15)[1] is True and T.score(e.objects()) == 1.0


def test_no_full_score_without_the_pick():
    e = Episode()
    e.step(pos=seat(), rot=UPRIGHT)  # simply set in the socket without a gripper
    assert e.step(3.0) == (0.0, False)


@pytest.mark.parametrize("what", ["held", "up", "leaning", "off"])
def test_no_full_score_while_a_condition_is_missing(what):
    e = Episode()
    e.pick()
    e.insert()
    if what == "held":
        e.lower()
        e.step(2.5)
    elif what == "up":
        e.carry(seat(0, 0, T.FLOOR + 0.006))
        e.step(left=_home("left"))
    elif what == "leaning":
        e.carry(seat(0, 0, T.FLOOR + 0.0006), _rx(math.radians(5.0)))
        e.step(left=_home("left"))
    else:
        e.carry(seat(0.004, 0, T.FLOOR + 0.0006))
        e.step(left=_home("left"))
    assert e.step(2.0)[1] is False and e.step(0.1)[0] < 1.0


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    e.step(0.6)
    e.step(pos=seat(0, 0, T.FLOOR + 0.005))
    e.step(0.1)
    e.step(pos=seat())
    assert e.step(0.6)[1] is False
    assert e.step(0.5)[1] is True


def test_post_creeping_beyond_the_tolerance_restarts_the_second():
    e = Episode()
    e.full_course()
    for k in range(40):
        e.step(pos=seat() + (0.0002 * k * 0.0, 0.0002 * k * 0.1, 0.0))  # drift inside the socket stays tiny
    assert e.step()[1] is True
    e2 = Episode()
    e2.full_course()
    for k in range(40):
        e2.step(pos=seat() + (0.0, 0.0, 0.0003 * k))  # 0.3 mm every 0.05 s: lifted out
    assert e2.step()[1] is False


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    away = _gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1), np.array([0.0, 0.0, -1.0]))
    assert e.step(1.2, left=away) == (1.0, True)


def test_post_pulled_out_loses_the_steps():
    e = Episode()
    e.full_course()
    e.step(pos=seat(0, 0, T.CUBE_TOP + 0.03))
    assert e.done() == [True, False, False, False]


def test_post_knocked_over_scores_zero():
    e = Episode()
    e.pick()
    e.insert()
    assert e.step(pos=T.RACK_POS + (0.0, 0.0, 0.0), rot=_rx(math.radians(90.0)))[0] == 0.0


def test_post_off_the_table_scores_zero():
    e = Episode()
    e.pick()
    assert e.step(pos=np.array([0.3, 0.6, T.TOP - 0.3]))[0] == 0.0


def test_cube_torn_loose_scores_zero():
    e = Episode()
    e.full_course()
    assert e.step(cube_pos=T.CUBE_POS + (0.02, 0.0, 0.0))[0] == 0.0


def test_not_finite_scores_zero():
    e = Episode()
    e.pick()
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
