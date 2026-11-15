"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome (the roll picked up, seated with its round stub in the
bearing slot and its square stub in the drive socket, the retainer latched closed over it, grippers clear; a roll off the table, or the retainer
latched with the roll not seated, scores 0 for the rest of the run), the 1 s full-success timing, the seating geometry and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_robot_vacuum_brushroll.py"""

import math
import os
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "robot_vacuum_brushroll.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _q_about(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float)
    return np.concatenate([v / np.linalg.norm(v) * math.sin(a), [math.cos(a)]])


def _seat(dx=0.0, dy=0.0, dz=0.0, yaw=0.0, twist=0.0, flip=False):
    q = _q_about([0, 0, 1], yaw + (180.0 if flip else 0.0)) if (yaw or flip) else IDENT
    if twist:
        # roll about its own (y) axis: compose
        a = math.radians(twist) / 2
        qt = np.array([0.0, math.sin(a), 0.0, math.cos(a)])
        x1, y1, z1, w1 = q
        x2, y2, z2, w2 = qt
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                      w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    return (np.array([T.BAY_XY[0] + dx, T.BAY_XY[1] + dy, T.TOP + T.AXIS_Z + dz]), q)


START = {"roll": (np.array(T.ROLL_START), IDENT)}
SOLVED = {"roll": _seat()}
NEAR_TCP = np.array([T.BAY_XY[0], T.BAY_XY[1], T.TOP + T.AXIS_Z + 0.02])
RULE_CASES = [lambda e: {"roll": (np.array([T.BAY_XY[0], T.BAY_XY[1], T.TOP - 0.06]), IDENT)},
              lambda e: {"roll": START["roll"], "retainer": 0.0, "latched": True}]  # the retainer latched with the roll not seated


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


# -- this task's episode: the retainer, the latch and the roll's earlier lifting ------------------------------------------------------
class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None, retainer=None, latched=None):
        super().__init__(poses)
        solved = poses is SOLVED
        self.retainer = (0.0 if solved else math.radians(75.0)) if retainer is None else retainer
        self.latched = solved if latched is None else latched
        if solved:
            T._PROGRESS.mem["lifted"] = True

    def objects(self):
        out = super().objects()
        out.update(retainer=self.retainer, latched=self.latched)
        return out

    def step(self, seconds=0.05, **changes):
        for k in ("retainer", "latched"):
            if k in changes:
                setattr(self, k, changes.pop(k))
        return super().step(seconds, **changes)


def test_lifting_the_roll_is_step_one_and_remembered():
    e = Episode()
    p, q = START["roll"]
    e.step(0.2, roll=(p + np.array([0.0, 0.0, 0.03]), q))
    assert e.done() == [True, False, False, False]
    e.step(0.2, roll=START["roll"])
    assert e.done()[0] is True


def test_the_seated_roll_with_the_retainer_open_is_steps_one_and_two():
    e = Episode()
    e.step(0.2, roll=_seat())
    e.step(0.1, roll=_seat())
    T._PROGRESS.mem["lifted"] = True
    assert e.done() == [True, True, False, False]


@pytest.mark.parametrize("kw", [dict(dz=0.004), dict(dx=0.005), dict(dy=0.006), dict(yaw=6.0), dict(twist=15.0), dict(flip=True)])
def test_a_roll_not_seated_is_not_step_two(kw):
    e = Episode()
    e.step(0.2, roll=_seat(**kw))
    assert e.done()[1] is False


@pytest.mark.parametrize("kw", [dict(dx=0.002), dict(dy=0.003), dict(dz=0.0015), dict(yaw=2.0), dict(twist=10.0)])
def test_a_roll_within_the_slots_play_is_seated(kw):
    e = Episode()
    e.step(0.2, roll=_seat(**kw))
    assert e.done()[1] is True


def test_the_latched_retainer_over_the_seated_roll_completes_all_steps():
    e = Episode(SOLVED)
    e.step(0.1)
    assert e.done() == [True, True, True, True]


def test_the_retainer_down_without_the_latch_is_not_closed():
    e = Episode(SOLVED, latched=False)
    e.step(0.1)
    assert e.done()[2] is False


def test_the_latched_retainer_not_down_is_not_closed():
    e = Episode(SOLVED, retainer=math.radians(20.0))
    e.step(0.1)
    assert e.done()[2] is False


def test_the_roll_still_held_in_the_gripper_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.2, right=_gripper(NEAR_TCP, contact=15.0))
    assert e.done()[3] is False
    assert e.step(3.0)[0] < 1.0


def test_a_lapse_restarts_the_second():  # noqa: F811  (a 1.5 mm slide along the slots keeps the seat: only stillness lapses)
    e = Episode(SOLVED)
    e.step(0.8)
    p, q = e.poses["roll"]
    e.step(0.1, roll=(p + np.array([0.0, 0.0025, 0.0]), q))
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok
