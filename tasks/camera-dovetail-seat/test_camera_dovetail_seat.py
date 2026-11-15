"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_camera_dovetail_seat.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "camera_dovetail_seat.py")
T = TASK.module
START = np.array([*T.RACK_XY, T.RACK_TOP + T.REST])
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def at(y, x=0.0, z=0.0):
    """The module's pose on the plate: y along the rail from the plate's middle."""
    return np.array([T.MOUNT_XY[0] + x, T.MOUNT_XY[1] + y, T.PLATE_TOP + z])


ENTRY, HOME = at(T.ENTRY_Y), at(T.HOME_Y)


def _quat_z(deg):
    a = math.radians(deg)
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def _gripper(tcp, contact=0.0, opening=0.09, axis=(0.0, 0.0, -1.0)):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _on(pos, contact=20.0):
    return _gripper(np.asarray(pos) + np.array([0.0, 0.0, T.GRIP_UP]), contact=contact, opening=0.028)


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos, self.quat = START.copy(), IDENT.copy()
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        m = ObjectState(pos=self.pos.copy(), quat_xyzw=self.quat.copy(), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                        low=self.pos.copy(), high=self.pos.copy())
        return {"module": m, "left_gripper": self.left, "right_gripper": self.right, "episode_time": self.t}

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

    def scores(self, seconds, **changes):
        for k, v in changes.items():
            setattr(self, k, v)
        out, end = [], self.t + seconds
        while self.t < end - 1e-9:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            out.append((T.score(objects), T.success(objects)))
        return out

    def done(self):
        return T.steps_done(self.objects())

    def pick(self):
        self.step(right=_on(self.pos))
        up = self.pos + np.array([0.0, 0.0, 0.05])
        self.step(pos=up, right=_on(up))

    def push_to(self, pos):
        self.step(pos=pos, right=_on(pos))

    def let_go(self):
        self.step(right=_home("right"))

    def full_course(self):
        self.pick()
        self.push_to(ENTRY)
        self.push_to(at(0.0))
        self.push_to(HOME)
        self.let_go()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert [o.name for o in TASK.objects] == ["mount", "rack", "module"]


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False] * 4


def test_picked_up_is_a_step():
    e = Episode()
    e.pick()
    assert e.done() == [True, False, False, False]
    assert e.step() == (0.2, False)


def test_held_on_the_rack_is_not_picked_up():
    e = Episode()
    e.step(right=_on(e.pos))
    assert e.done() == [False] * 4


def test_a_gripper_beside_or_closed_does_not_pick_it_up():
    e = Episode()
    e.step(right=_on(e.pos + np.array([0.1, 0.0, 0.0])))
    e.step(pos=e.pos + np.array([0.0, 0.0, 0.03]), right=_on(e.pos + np.array([0.1, 0.0, 0.0])))
    assert e.done()[0] is False
    T._PROGRESS.__init__()
    e = Episode()
    closed = _gripper(e.pos + np.array([0.0, 0.0, T.GRIP_UP]), contact=20.0, opening=0.0)
    e.step(right=closed)
    e.step(pos=e.pos + np.array([0.0, 0.0, 0.03]), right=closed)
    assert e.done()[0] is False


def test_set_down_in_front_of_the_rail_is_not_on_it():
    e = Episode()
    e.pick()
    e.push_to(ENTRY)
    assert e.done() == [True, False, False, False]


def test_slid_onto_the_rail_is_a_step_and_home_a_second():
    e = Episode()
    e.pick()
    e.push_to(ENTRY)
    e.push_to(at(0.0))
    assert e.done() == [True, True, False, False]
    e.push_to(HOME)
    assert e.done() == [True, True, True, False]
    assert e.step() == (0.6, False)


def test_module_dropped_on_top_of_the_rail_is_not_on_it():
    e = Episode()
    e.pick()
    e.push_to(at(0.0, z=0.0065))  # resting on the rail's top, the slot not over its undercut
    assert e.done() == [True, False, False, False]


def test_module_off_the_rails_axis_is_not_on_it():
    e = Episode()
    e.pick()
    e.push_to(at(0.0, x=0.004))
    assert e.done()[1] is False


def test_module_crooked_or_tilted_is_not_on_the_rail():
    e = Episode()
    e.pick()
    e.step(pos=at(0.0), quat=_quat_z(6.0), right=_on(at(0.0)))
    assert e.done()[1] is False
    a = math.radians(5.0)
    e.step(quat=np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)]))
    assert e.done()[1] is False


def test_module_a_few_mm_short_of_the_stop_is_not_home():
    e = Episode()
    e.pick()
    e.push_to(at(T.HOME_Y - 0.004))
    assert e.done() == [True, True, False, False]
    e.push_to(at(T.HOME_Y - 0.0015))
    assert e.done()[2] is True


def test_module_never_held_scores_nothing_on_the_rail():
    e = Episode()
    e.step(pos=HOME)
    assert e.done() == [False] * 4
    assert max(s for s, _ in e.scores(3.0)) == 0.0


def test_still_holding_the_module_is_not_let_go():
    e = Episode()
    e.pick()
    e.push_to(HOME)
    assert e.done() == [True, True, True, False]


def test_a_gripper_near_the_module_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(right=_gripper(HOME + np.array([0.0, 0.0, 0.05])))
    assert e.done()[3] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.1) == (1.0, True)


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step() == (1.0, True)


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0


MISSING = {
    "still held": lambda e: (e.pick(), e.push_to(HOME)),
    "short of the stop": lambda e: (e.pick(), e.push_to(at(T.HOME_Y - 0.01)), e.let_go()),
    "in front of the rail": lambda e: (e.pick(), e.push_to(ENTRY), e.let_go()),
    "on top of the rail": lambda e: (e.pick(), e.push_to(at(0.0, z=0.0065)), e.let_go()),
    "off the axis": lambda e: (e.pick(), e.push_to(at(T.HOME_Y, x=0.004)), e.let_go()),
    "gripper within 3 cm": lambda e: (e.full_course(), e.step(right=_gripper(HOME + np.array([0.0, 0.0, 0.05])))),
    "crooked": lambda e: (e.full_course(), e.step(quat=_quat_z(6.0))),
}


@pytest.mark.parametrize("missing", sorted(MISSING))
def test_no_full_score_while_any_success_condition_is_missing(missing):
    e = Episode()
    MISSING[missing](e)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history), missing
    assert max(s for s, _ in history) <= 0.8


def test_no_full_score_while_the_module_keeps_moving():
    e = Episode()
    e.full_course()
    history = []
    for k in range(60):
        e.quat = _quat_z(3.5 * (k % 2))  # within the 4 degree alignment, more than 2 degrees from where it began
        history.append(e.step())
    assert all(s == 0.8 and not ok for s, ok in history)


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    e.step(0.5)
    e.step(pos=at(T.HOME_Y - 0.01))  # slides back a little for a moment
    e.step(pos=HOME)
    assert e.step(0.9) == (0.8, False)
    assert e.step(0.15) == (1.0, True)


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    e.step(right=_gripper(T._START_TCP["right"] + np.array([0.0, 0.0, 0.2])))
    assert e.step(1.1) == (1.0, True)


def test_module_pulled_off_loses_the_steps():
    e = Episode()
    e.full_course()
    e.step(pos=at(-0.06))
    assert e.done() == [True, False, False, False]


def test_module_off_the_table_scores_zero():
    e = Episode()
    e.full_course()
    e.step(pos=np.array([0.0, 0.0, T.TOP - 0.2]))
    assert e.step() == (0.0, False)


def test_not_finite_scores_zero():
    e = Episode()
    e.full_course()
    e.step(pos=np.array([float("nan"), 0.0, 0.0]))
    assert e.step() == (0.0, False)


def test_demo_is_reachable():
    plan = ArmPlan(T.demo()["right"], BASES["right"])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    assert max(sum(w.duration for w in d[s]) for s in ("left", "right")) < 40.0
