"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_robot_cover_swap.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "robot_cover_swap.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
LOCKED = np.array([*T.DECK_XY, T.DECK_TOP + T.REST])
OLD0 = LOCKED.copy()
NEW0 = np.array([*T.RACK_XY, T.RACK_TOP + T.REST])
IN_TRAY = np.array([*T.TRAY_XY, T.TRAY_FLOOR + 0.001])
UP = np.array([0.0, 0.0, 0.05])


def _quat_z(deg):
    a = math.radians(deg)
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def _quat_x(deg):
    a = math.radians(deg)
    return np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)])


def _gripper(tcp, contact=0.0, opening=0.09, axis=(0.0, 0.0, -1.0)):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _on(pos, contact=20.0):
    """The right gripper closed on a cover's rib."""
    return _gripper(np.asarray(pos) + np.array([0.0, 0.0, T.GRIP_UP]), contact=contact, opening=0.012)


class Episode:
    def __init__(self):
        self.t = 0.0
        self.old, self.old_q = OLD0.copy(), IDENT.copy()
        self.new, self.new_q = NEW0.copy(), IDENT.copy()
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        def st(p, q):
            return ObjectState(pos=p.copy(), quat_xyzw=q.copy(), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                               low=p.copy(), high=p.copy())

        return {"old": st(self.old, self.old_q), "new": st(self.new, self.new_q), "left_gripper": self.left,
                "right_gripper": self.right, "episode_time": self.t}

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

    def take_old_off(self):
        """Hold the cracked cover, push it off the slots' ends, lift it, carry it into the tray, let go."""
        self.step(right=_on(self.old))
        self.step(old=self.old + np.array([T.SLIDE, 0.0, 0.0]), right=_on(self.old + np.array([T.SLIDE, 0.0, 0.0])))
        self.step(old=self.old + np.array([0.0, 0.0, 0.05]), right=_on(self.old + np.array([0.0, 0.0, 0.05])))
        self.step(old=IN_TRAY.copy(), right=_on(IN_TRAY))
        self.step(right=_home("right"))

    def fit_new(self):
        """Hold the new cover, carry it onto the posts, slide it to the lock, let go."""
        self.step(right=_on(self.new))
        self.step(new=self.new + UP, right=_on(self.new + UP))
        over = LOCKED + np.array([T.SLIDE, 0.0, 0.0])
        self.step(new=over, right=_on(over))
        self.step(new=LOCKED.copy(), right=_on(LOCKED))
        self.step(right=_home("right"))

    def full_course(self):
        self.take_old_off()
        self.fit_new()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert [o.name for o in TASK.objects] == ["deck", "rack", "tray", "old", "new"]


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False] * 4


def test_the_old_cover_starts_locked():
    e = Episode()
    assert T.locked(e.objects(), "old")


def test_old_cover_lifted_clear_is_step_one():
    e = Episode()
    e.step(right=_on(e.old))
    e.step(old=e.old + np.array([T.SLIDE, 0.05, 0.05]), right=_on(e.old + np.array([T.SLIDE, 0.05, 0.05])))
    assert e.done() == [False] * 4  # only 0.05 m away: still over the bay
    e.step(old=e.old + np.array([-0.20, 0.0, 0.0]), right=_on(e.old + np.array([-0.20, 0.0, 0.0])))
    assert e.done() == [True, False, False, False]


def test_old_cover_in_the_tray_is_step_two():
    e = Episode()
    e.take_old_off()
    assert e.done() == [True, True, False, False]
    assert e.step() == (0.4, False)


def test_old_cover_pushed_off_the_bay_without_being_held_scores_nothing():
    e = Episode()
    e.step(old=IN_TRAY.copy())
    assert e.done() == [False] * 4


def test_old_cover_on_the_tray_wall_is_not_in_the_tray():
    e = Episode()
    e.take_old_off()
    e.step(old=IN_TRAY + np.array([0.02, 0.0, 0.0]))
    assert e.done() == [True, False, False, False]
    e.step(old=IN_TRAY + np.array([0.0, 0.0, 0.03]))
    assert e.done()[1] is False
    e.step(old=IN_TRAY.copy(), old_q=_quat_x(20.0))
    assert e.done()[1] is False


def test_old_cover_on_the_rack_or_table_is_not_in_the_tray():
    e = Episode()
    e.take_old_off()
    e.step(old=np.array([0.1, -0.3, T.TOP + 0.001]))
    assert e.done() == [True, False, False, False]


def test_new_cover_locked_is_step_three():
    e = Episode()
    e.take_old_off()
    e.fit_new()
    assert e.done() == [True, True, True, True]


def test_new_cover_still_unlocked_is_not_locked():
    e = Episode()
    e.take_old_off()
    e.step(right=_on(e.new))
    over = LOCKED + np.array([T.SLIDE, 0.0, 0.0])
    e.step(new=over, right=_on(over))
    assert e.done() == [True, True, False, False]


def test_new_cover_resting_on_the_post_heads_is_not_locked():
    e = Episode()
    e.take_old_off()
    e.step(right=_on(e.new))
    e.step(new=LOCKED + np.array([0.0, 0.0, 0.0065]), right=_on(LOCKED))
    assert e.done()[2] is False


def test_new_cover_locked_crooked_is_not_locked():
    e = Episode()
    e.take_old_off()
    e.step(right=_on(e.new))
    e.step(new=LOCKED.copy(), new_q=_quat_z(7.0), right=_on(LOCKED))
    assert e.done()[2] is False


def test_new_cover_locked_but_off_by_3mm_is_not_locked():
    e = Episode()
    e.take_old_off()
    e.step(right=_on(e.new))
    e.step(new=LOCKED + np.array([0.0, 0.003, 0.0]), right=_on(LOCKED))
    assert e.done()[2] is False


def test_new_cover_never_held_is_not_locked():
    e = Episode()
    e.take_old_off()
    e.step(new=LOCKED.copy())
    assert e.done() == [True, True, False, False]


def test_a_gripper_beside_a_cover_does_not_hold_it():
    e = Episode()
    far = OLD0 + np.array([0.0, 0.2, 0.0])
    e.step(right=_on(far))
    e.step(old=IN_TRAY.copy() + UP, right=_on(far))
    assert e.done() == [False] * 4


def test_closed_fingers_do_not_hold_a_cover():
    e = Episode()
    closed = _gripper(OLD0 + np.array([0.0, 0.0, T.GRIP_UP]), contact=20.0, opening=0.0)
    e.step(right=closed)
    e.step(old=IN_TRAY.copy() + UP, right=closed)
    assert e.done() == [False] * 4


def test_still_holding_the_new_cover_is_not_let_go():
    e = Episode()
    e.take_old_off()
    e.step(right=_on(e.new))
    e.step(new=LOCKED.copy(), right=_on(LOCKED))
    assert e.done() == [True, True, True, False]


def test_a_gripper_near_a_cover_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(right=_gripper(LOCKED + np.array([0.0, 0.0, 0.07])))
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
    "old cover not taken off": lambda e: (e.step(right=_on(e.new)), e.step(new=LOCKED + np.array([0.0, 0.0, 0.05]))),
    "old cover left on the table": lambda e: (e.take_old_off(), e.step(old=np.array([0.1, -0.3, T.TOP + 0.001])),
                                              e.fit_new()),
    "new cover left on the rack": lambda e: e.take_old_off(),
    "new cover unlocked": lambda e: (e.take_old_off(), e.fit_new(), e.step(new=LOCKED + np.array([0.01, 0.0, 0.0]))),
    "new cover on the heads": lambda e: (e.take_old_off(), e.fit_new(), e.step(new=LOCKED + np.array([0.0, 0.0, 0.007]))),
    "gripper within 3 cm": lambda e: (e.full_course(), e.step(right=_gripper(LOCKED + np.array([0.0, 0.0, 0.07])))),
    "old cover out of the tray": lambda e: (e.full_course(), e.step(old=IN_TRAY + np.array([0.03, 0.0, 0.0]))),
    "new cover crooked": lambda e: (e.full_course(), e.step(new_q=_quat_z(7.0))),
}


@pytest.mark.parametrize("missing", sorted(MISSING))
def test_no_full_score_while_any_success_condition_is_missing(missing):
    e = Episode()
    MISSING[missing](e)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history), missing
    assert max(s for s, _ in history) <= 0.8


@pytest.mark.parametrize("part", ["old", "new"])
def test_no_full_score_while_a_cover_keeps_moving(part):
    e = Episode()
    e.full_course()
    history = []
    for k in range(60):
        sign = 1.0 if k % 2 else -1.0
        if part == "old":
            e.old = IN_TRAY + np.array([0.0025 * sign, 0.0, 0.0])
        else:
            e.new = LOCKED + np.array([0.0, 0.0012 * sign * 2.0, 0.0])
        history.append(e.step())
    assert all(s == 0.8 and not ok for s, ok in history), part


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    e.step(0.5)
    e.step(new=LOCKED + np.array([0.01, 0.0, 0.0]))  # the cover slides off its slots' ends for a moment
    e.step(new=LOCKED.copy())
    assert e.step(0.9) == (0.8, False)
    assert e.step(0.15) == (1.0, True)


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    e.step(right=_gripper(T._START_TCP["right"] + np.array([0.0, 0.0, 0.2])))
    assert e.step(1.1) == (1.0, True)


def test_a_cover_off_the_table_scores_zero():
    e = Episode()
    e.full_course()
    e.step(old=np.array([0.0, 0.0, T.TOP - 0.2]))
    assert e.step() == (0.0, False)
    T._PROGRESS.__init__()
    e = Episode()
    e.full_course()
    e.step(new=np.array([0.0, 0.0, T.TOP - 0.2]))
    assert e.step() == (0.0, False)


def test_not_finite_scores_zero():
    e = Episode()
    e.full_course()
    e.step(new=np.array([float("nan"), 0.0, 0.0]))
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
