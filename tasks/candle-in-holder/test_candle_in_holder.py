"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome (the candle picked up, its base end down
the socket, seated on the floor upright, grippers clear; a candle off the table scores 0 for the rest of the run), the 1 s
full-success timing and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_candle_in_holder.py"""

import math
import os
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "candle_in_holder.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _tilted(deg):
    a = math.radians(deg) / 2
    return np.array([math.sin(a), 0.0, 0.0, math.cos(a)])


STICK0 = (np.array([T.STICK_XY[0], T.STICK_XY[1], T.TOP]), IDENT)
START = {"candle": (np.array(T.CANDLE_START), IDENT), "stick": STICK0}


def _in(dx=0.0, dy=0.0, dz=0.0, tilt=0.0):
    return (np.array([T.STICK_XY[0] + dx, T.STICK_XY[1] + dy, T.TOP + T.FLOOR_Z + dz]), _tilted(tilt))


SOLVED = {"candle": _in(), "stick": STICK0}
NEAR_TCP = np.array([T.STICK_XY[0], T.STICK_XY[1], T.TOP + T.FLOOR_Z + 0.05])
RULE_CASES = [lambda e: {"candle": (np.array([T.STICK_XY[0], T.STICK_XY[1], T.TOP - 0.06]), IDENT)},
              lambda e: {"stick": (STICK0[0], _tilted(11.0))},  # the candlestick tipped over 10 degrees
              lambda e: {"stick": (STICK0[0] + np.array([0.011, 0.0, 0.0]), IDENT)},  # slid more than 10 mm
              lambda e: {"stick": (np.array([T.STICK_XY[0], T.STICK_XY[1], T.TOP - 0.06]), IDENT)}]


# -- shared checks ----------------------------------------------------------------------------------------------------
DOWN = np.array([0.0, 0.0, -1.0])
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()
    yield


def _gripper(tcp, contact=0.0, opening=0.09, axis=DOWN):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


class Episode:
    """A made-up episode: the parts at ``START`` (or ``SOLVED``), both grippers at home; ``step`` advances 0.05 s ticks."""

    def __init__(self, poses=None):
        self.t = 0.0
        self.poses = {n: (np.array(p, float), np.array(q, float)) for n, (p, q) in (poses or START).items()}
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {n: _state(p, q) for n, (p, q) in self.poses.items()}
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            else:
                self.poses[k] = (np.array(v[0], float), np.array(v[1], float))
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def done(self):
        return T.steps_done(self.objects())


def test_loads():
    assert TASK.module is T
    assert hasattr(T, "demo") and hasattr(T, "score") and hasattr(T, "success")


def test_start_scores_zero():
    assert Episode().step(1.0) == (0.0, False)


def test_solved_succeeds_after_one_second():
    e = Episode(SOLVED)
    score, ok = e.step(0.5)
    assert score == 0.8 and not ok  # all four steps done, the second not yet up: below 1.0
    score, ok = e.step(1.0)
    assert score == 1.0 and ok


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode(SOLVED)
    for _ in range(19):
        score, ok = e.step()
        assert score < 1.0 and not ok


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S >= 1.0 and T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def test_a_lapse_restarts_the_second():
    e = Episode(SOLVED)
    e.step(0.8)
    name = T.MOVERS[0]
    p, q = e.poses[name]
    e.step(0.1, **{name: (p + np.array([0.004, 0.0, 0.0]), q)})  # moved 4 mm: the count starts over
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok


def test_a_gripper_still_near_a_part_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.3, right=_gripper(NEAR_TCP))
    assert e.done()[-1] is False
    assert e.step(3.0)[0] < 1.0


def test_not_finite_scores_zero():
    e = Episode(SOLVED)
    p, q = e.poses[T.MOVERS[0]]
    assert e.step(0.1, **{T.MOVERS[0]: (np.array([np.nan, 0, 0]), q)})[0] == 0.0


@pytest.mark.parametrize("case", range(len(RULE_CASES)))
def test_a_rule_event_scores_zero_for_the_rest_of_the_run(case):
    e = Episode(SOLVED)
    e.step(0.3)
    e.step(0.1, **RULE_CASES[case](e))
    assert e.step(0.1)[0] == 0.0
    e.step(0.1, **SOLVED)  # put right again (the stick back where it was too)
    score, ok = e.step(3.0)
    assert score == 0.0 and not ok  # the event stays


@pytest.mark.parametrize("side", ["left", "right"])
def test_demo_is_reachable(side):
    plan = ArmPlan(T.demo()[side], BASES[side])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    assert max(sum(w.duration for w in d[s]) for s in d) < 150.0


# -- this task's episode: the candle's earlier lifting is remembered -----------------------------------------------------------------
class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None):
        super().__init__(poses)
        if poses is SOLVED:
            T._PROGRESS.mem["lifted"] = True


def _solved_with(**changes):
    e = Episode(SOLVED)
    e.step(0.1, **changes)
    return e.done()


def test_lifting_the_candle_is_step_one_and_remembered():
    e = Episode()
    p, q = START["candle"]
    e.step(0.2, candle=(p + np.array([0.0, 0.0, 0.03]), q))
    assert e.done() == [True, False, False, False]
    e.step(0.2, candle=START["candle"])
    assert e.done()[0] is True


def test_the_seated_candle_completes_all_steps():
    assert _solved_with() == [True, True, True, True]


def test_a_candle_above_the_floor_has_entered_but_is_not_seated():
    d = _solved_with(candle=_in(dz=0.004))
    assert d[1] is True and d[2] is False


def test_a_candle_resting_on_the_rim_has_not_entered():
    assert _solved_with(candle=_in(dz=0.020))[1] is False  # 2 mm down only: not wedged


def test_a_candle_leaning_in_the_socket_is_not_seated():
    assert _solved_with(candle=_in(tilt=3.0))[2] is False  # 3 degrees: not upright within 2


@pytest.mark.parametrize("dx,dy", [(0.006, 0.0), (0.0, -0.006), (0.004, 0.004)])
def test_a_candle_off_the_socket_axis_has_not_entered(dx, dy):
    assert _solved_with(candle=_in(dx=dx, dy=dy))[1] is False


def test_a_candle_1p6_mm_off_the_axis_is_not_seated():
    assert _solved_with(candle=_in(dx=0.0016))[2] is False
    assert _solved_with(candle=_in(dx=0.0014))[2] is True


def test_a_candle_not_down_to_20p5_mm_is_not_seated():
    assert _solved_with(candle=_in(dz=0.0016))[2] is False
    assert _solved_with(candle=_in(dz=0.0014))[2] is True


def test_the_left_hand_pinching_the_stem_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.2, left=_gripper(np.array([T.STICK_XY[0], T.STICK_XY[1], T.TOP + 0.052]), contact=15.0))
    assert e.done()[3] is False


def test_a_candle_left_on_the_table_is_nothing():
    assert _solved_with(candle=START["candle"])[1:] == [False, False, False]


def test_the_candle_still_held_in_the_gripper_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.2, right=_gripper(NEAR_TCP, contact=15.0))
    assert e.done()[3] is False
    assert e.step(3.0)[0] < 1.0


def test_a_lapse_restarts_the_second():  # noqa: F811  (the whole seat shifts 2.5 mm: still seated, but the parts moved: the second restarts)
    e = Episode(SOLVED)
    e.step(0.8)
    e.step(0.1, candle=_in(dx=0.0025), stick=(STICK0[0] + np.array([0.0025, 0.0, 0.0]), IDENT))
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok


def test_the_walls_seating_push_is_measured_in_the_simulator():
    """The press-in force of the sprung cup (the real value to compare with a kitchen scale) is measured in MuJoCo by tests in the
    task's notes: here only its design value is checked: 12 walls, 7700 N/m each, 0.6 mm travel."""
    from build_assets import N_WALL, WALL_K, WALL_TRAVEL

    assert (N_WALL, WALL_K, WALL_TRAVEL) == (12, 7700.0, 0.0006)
