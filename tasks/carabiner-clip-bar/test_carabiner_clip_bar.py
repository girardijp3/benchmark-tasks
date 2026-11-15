"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome (the carabiner lifted, the staple's near
leg through its loop, clipped = gate closed and lying on the board, grippers clear; a carabiner off the table scores 0 for the
rest of the run), the 1 s full-success timing, the threading geometry and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_carabiner_clip_bar.py"""

import math
import os
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "carabiner_clip_bar.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _q(rot):
    return np.array(T._quat(rot))


def _lying(x, y, dz=0.0, rot=None):
    return (np.array([x, y, T.TOP + T.BOARD_T + T.WIRE_R + dz]), _q(T._R_FLAT if rot is None else rot))


START = {"carabiner": (np.array(T.CARAB_START), _q(T._R_FLAT))}
CLIP_X, CLIP_Y = T.LEG_A[0] - 0.022, T.LEG_A[1] + 0.007
SOLVED = {"carabiner": _lying(CLIP_X, CLIP_Y)}
NEAR_TCP = np.array([CLIP_X, CLIP_Y, T.TOP + T.BOARD_T + T.WIRE_R])


def _off(name):
    p, q = SOLVED[name]
    return (np.array([p[0], p[1], T.TOP - 0.06]), q)


RULE_CASES = [lambda e: {"carabiner": _off("carabiner")}]


def _rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


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
    e.step(0.1, **SOLVED)  # put right again
    score, ok = e.step(3.0)
    assert score == 0.0 and not ok  # the event stays


@pytest.mark.parametrize("side", ["right"])
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


# -- this task's episode: the gate angle and the carabiner's earlier lifting are remembered ----------------------------------------
class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None, gate=None):
        super().__init__(poses)
        self.gate = 0.0 if gate is None else gate
        if poses is SOLVED:
            T._PROGRESS.mem["lifted"] = True

    def objects(self):
        out = super().objects()
        out.update(gate=self.gate)
        return out

    def step(self, seconds=0.05, **changes):
        if "gate" in changes:
            self.gate = changes.pop("gate")
        return super().step(seconds, **changes)


def _solved_with(**changes):
    e = Episode(SOLVED)
    e.step(0.1, **changes)
    return e.done()


def test_lifting_the_carabiner_is_step_one_and_remembered():
    e = Episode()
    p, q = START["carabiner"]
    e.step(0.2, carabiner=(p + np.array([0.0, 0.0, 0.015]), q))
    assert e.done()[0] is True and not any(e.done()[1:])
    e.step(0.2, carabiner=START["carabiner"])
    assert e.done()[0] is True


def test_clipped_carabiner_completes_all_steps():
    assert _solved_with() == [True, True, True, True]


def test_leg_inside_the_loop_with_the_gate_open_is_threaded_not_clipped():
    d = Episode(SOLVED, gate=math.radians(-20.0))
    d.step(0.1)
    assert d.done()[1] is True and d.done()[2] is False


def test_carabiner_held_up_off_the_board_is_not_clipped():
    d = _solved_with(carabiner=_lying(CLIP_X, CLIP_Y, dz=0.012))
    assert d[1] is True and d[2] is False


@pytest.mark.parametrize("dx,dy", [(-0.03, 0.0), (0.0, 0.035), (0.0, -0.035), (0.03, 0.0)])
def test_leg_outside_the_loop_is_not_threaded(dx, dy):
    assert _solved_with(carabiner=_lying(CLIP_X + dx, CLIP_Y + dy))[1] is False


def test_carabiner_standing_on_edge_is_not_threaded():
    assert _solved_with(carabiner=(np.array([CLIP_X, CLIP_Y, T.TOP + 0.03]), IDENT))[1] is False


def test_carabiner_at_the_start_is_nothing():
    assert _solved_with(carabiner=START["carabiner"])[1:3] == [False, False]


def test_carabiner_still_held_in_the_gripper_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.2, right=_gripper(NEAR_TCP, contact=15.0))
    assert e.done()[3] is False
    assert e.step(3.0)[0] < 1.0


def test_a_lapse_restarts_the_second():  # noqa: F811  (a 3 mm slide keeps the clip: only stillness lapses)
    e = Episode(SOLVED)
    e.step(0.8)
    e.step(0.1, carabiner=_lying(CLIP_X + 0.003, CLIP_Y))
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok
