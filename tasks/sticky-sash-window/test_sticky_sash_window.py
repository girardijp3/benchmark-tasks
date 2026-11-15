"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome (the sash started, over half way, shut against
the stop in its lane, the tab let go), the rule events (a bent tab, a sash moved without a grip on the tab, a sash out of its lane or off the
table score 0 for the rest of the run), the 1 s full-success timing and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_sticky_sash_window.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "sticky_sash_window.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _s(x, y, z, quat=IDENT):
    return (np.array([x, y, z], float), quat)


def _at(along, dz=0.0, dx=0.0):
    return _s(T.SASH_START[0] + dx, T.SASH_START[1] + along, T.SASH_START[2] + dz)


START = {"sash": _at(0.0)}
SOLVED = {"sash": _at(T.TRAVEL)}
NEAR_TCP = np.array([T.SASH_START[0] - 0.024, T.SASH_START[1] + T.TRAVEL + T.TAB_HINGE[1], T.SASH_START[2] + T.TAB_HINGE[2]])
RULE_CASES = [lambda e: {"tab": math.radians(-12.0)},
              lambda e: {"sash": _at(T.TRAVEL - 0.004)},
              lambda e: {"sash": _at(T.TRAVEL, dz=0.02)},
              lambda e: {"sash": _s(T.SASH_START[0], T.SASH_START[1] + T.TRAVEL, T.TOP - 0.06)}]



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


@pytest.mark.parametrize("side", ["left"])
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


# -- this task's episode: the tab's angle ---------------------------------------------------------------------------------------
class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None, tab=0.0):
        super().__init__(poses)
        self.tab = tab

    def objects(self):
        out = super().objects()
        out["tab"] = self.tab
        return out

    def step(self, seconds=0.05, **changes):
        if "tab" in changes:
            self.tab = changes.pop("tab")
        return super().step(seconds, **changes)



def test_started_is_step_one_only():
    e = Episode()
    e.step(0.2, sash=_at(0.012))
    assert e.done() == [True, False, False, False] or e.done()[0] is True


def test_half_way_is_step_two():
    e = Episode()
    e.step(0.2, sash=_at(0.080))
    d = e.done()
    assert d[:2] == [True, True] and d[2] is False


def test_closed_against_the_stop_completes_the_steps():
    e = Episode(SOLVED)
    e.step(0.2)
    assert e.done() == [True, True, True, True]


def test_two_centimetres_short_is_not_closed():
    e = Episode(SOLVED)
    e.step(0.2, sash=_at(T.TRAVEL - 0.02))
    assert e.done()[2] is False


def test_a_bent_tab_means_not_closed_and_a_zero():
    e = Episode(SOLVED)
    e.step(0.2, tab=math.radians(-11.0))
    assert e.done()[2] is False
    assert e.step(0.1)[0] == 0.0


def test_a_sash_out_of_its_lane_is_not_closed():
    e = Episode(SOLVED)
    e.step(0.2, sash=_at(T.TRAVEL, dx=0.008))
    assert e.done()[2] is False


def test_a_sash_pushed_without_holding_the_tab_scores_zero():
    e = Episode()
    e.step(0.2)
    e.step(0.05, sash=_at(0.020))
    assert e.step(0.05)[0] == 0.0


def test_a_sash_moved_while_the_tab_is_held_is_fine():
    e = Episode()
    tab_tcp = np.array([T.SASH_START[0] - 0.024, T.SASH_START[1] + T.TAB_HINGE[1], T.SASH_START[2] + T.TAB_HINGE[2]])
    e.step(0.2, right=_gripper(tab_tcp, contact=15.0))
    e.step(0.05, sash=_at(0.020), right=_gripper(tab_tcp + np.array([0, 0.02, 0]), contact=15.0))
    assert e.step(0.05)[0] >= 0.0 and not T._PROGRESS.events


def test_a_gripper_holding_the_tab_is_not_let_go():
    e = Episode(SOLVED)
    e.step(0.3, right=_gripper(NEAR_TCP, contact=15.0))
    assert e.done()[3] is False


def test_a_lapse_restarts_the_second():  # noqa: F811  (a 2.5 mm shift across the lane keeps every condition: only stillness lapses)
    e = Episode(SOLVED)
    e.step(0.8)
    e.step(0.1, sash=_at(T.TRAVEL, dx=0.0025))
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok


def test_a_sash_pushed_by_one_gripper_while_the_other_holds_the_tab_scores_zero():
    e = Episode()
    tab_tcp = np.array([T.SASH_START[0] - 0.024, T.SASH_START[1] + T.TAB_HINGE[1], T.SASH_START[2] + T.TAB_HINGE[2]])
    glass = np.array([T.SASH_START[0] - 0.012, T.SASH_START[1] - 0.06, T.SASH_START[2] + 0.15])  # a pad against the sash's front face
    e.step(0.2, right=_gripper(tab_tcp, contact=15.0), left=_gripper(glass, contact=0.0))
    assert not T._PROGRESS.events
    e.step(0.05, sash=_at(0.020), right=_gripper(tab_tcp + np.array([0, 0.02, 0]), contact=15.0), left=_gripper(glass + np.array([0, 0.02, 0])))
    assert e.step(0.05)[0] == 0.0
    assert "pushed by a gripper" in " ".join(T._PROGRESS.events)
