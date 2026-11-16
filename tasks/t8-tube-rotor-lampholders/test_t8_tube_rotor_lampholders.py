"""Offline checks (no GPU, no network): the task file loads, the engaged test (tube level on the holders' axis, pins in both
holders), the declared click detent (45 degrees) and lamp (both rotors at 90 degrees +-5, tube engaged), the step ladder, the 1 s
full-success timing, the dropped-tube latch and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_t8_tube_rotor_lampholders.py"""

import math
import os
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "t8_tube_rotor_lampholders.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _roll(deg):
    """Quaternion of a turn of ``deg`` about +y (the tube's axis)."""
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


START = {"tube": (np.array(T.TUBE_XYZ, float), IDENT)}
SOLVED = {"tube": (np.array(T.TUBE_FITTED, float) + np.array([0.0, 0.0, -0.0025]), _roll(-90.0))}  # fitted, turned, hanging 2.5 mm in the pockets
FITTED = {"tube": (np.array(T.TUBE_FITTED, float), IDENT)}
HOLD_TCP = np.array(T.TUBE_FITTED, float) + np.array([0.0, 0.2, 0.0])  # pads across the tube at the left grip point
NEAR_TCP = np.array(T.TUBE_FITTED, float) + np.array([0.0, 0.2, 0.04])  # pad tips 14 mm above the tube
OFF_TABLE = (np.array([0.3, 0.0, T.TOP - 0.06]), IDENT)
RULE_CASES = [
    lambda e: {"tube": OFF_TABLE},  # the tube falls off the table
    lambda e: {"tube": (np.array(T.TUBE_XYZ, float) + np.array([0.0, 0.0, -0.05]), IDENT)},  # dropped 50 mm: it breaks
]


@pytest.fixture(autouse=True)
def fresh_lamp():
    T._LAMP.__init__()
    yield




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


# -- this task's episode: the rotors' angles ride along as extra states --------------------------------------------------------------
class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None, rot=(0.0, 0.0)):
        super().__init__(poses)
        self.rot = list(rot)
        if poses is SOLVED:
            self.rot = [-math.pi / 2, -math.pi / 2]

    def objects(self):
        out = super().objects()
        out["rotL"], out["rotR"] = self.rot
        return out


def test_the_tube_in_its_cradles_is_not_engaged():
    assert not T.engaged(Episode(START).objects())


def test_fitted_but_not_turned_scores_one_step():
    e = Episode(FITTED)
    e.step(0.1)
    assert e.done() == [True, False, False, False]
    assert T.score(e.objects()) == pytest.approx(0.2)


def test_the_click_pulls_a_rotor_on_past_45_degrees_and_back_short_of_it():
    e = Episode(FITTED, rot=(math.radians(-50), math.radians(-30)))
    e.step(0.1)
    assert T._LAMP.tgt == [-T.STOP, 0.0]


def test_both_rotors_turned_45_degrees_gives_step_two():
    e = Episode(FITTED, rot=(math.radians(-60), math.radians(-60)))
    e.step(0.1)
    assert e.done() == [True, True, False, False]


def test_one_rotor_turned_is_not_enough():
    e = Episode(FITTED, rot=(math.radians(-90), math.radians(-20)))
    e.step(0.1)
    assert e.done()[1] is False


def test_both_rotors_at_the_stops_light_the_tube_and_give_step_three():
    e = Episode(SOLVED)
    e.rot = [math.radians(-88), math.radians(-92)]
    e.step(0.1)
    assert T._LAMP.lit and e.done()[:3] == [True, True, True]


def test_a_rotor_short_of_its_stop_leaves_the_tube_dark():
    e = Episode(SOLVED)
    e.rot = [math.radians(-90), math.radians(-80)]
    e.step(0.1)
    assert not T._LAMP.lit and e.done()[2] is False


def test_rotors_at_the_stops_without_the_tube_fitted_do_not_light_it():
    e = Episode(START, rot=(-math.pi / 2, -math.pi / 2))
    e.step(0.1)
    assert not T._LAMP.lit and e.done() == [False, False, False, False]


def test_a_tube_off_the_axis_is_not_engaged():
    for dx, dz in ((0.005, 0.0), (0.0, 0.005), (-0.005, 0.0)):
        p = np.array(T.TUBE_FITTED, float) + np.array([dx, 0.0, dz])
        assert not T.engaged(Episode({"tube": (p, IDENT)}).objects())


def test_a_tube_too_far_along_its_axis_is_not_engaged():
    p = np.array(T.TUBE_FITTED, float) + np.array([0.0, 0.004, 0.0])
    assert not T.engaged(Episode({"tube": (p, IDENT)}).objects())


def test_a_tube_slewed_in_the_holders_is_not_engaged():
    r = np.array([0.0, 0.0, math.sin(math.radians(2.0) / 2 * 2.0), math.cos(math.radians(2.0))])  # 4 degrees about z: ends 20 mm off
    assert not T.engaged(Episode({"tube": (np.array(T.TUBE_FITTED, float), r / np.linalg.norm(r))}).objects())


def test_a_gripper_holding_the_lit_tube_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.3, left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
    assert e.done() == [True, True, True, False]
    assert e.step(3.0)[0] < 1.0


def test_full_score_needs_the_second_after_the_grippers_let_go():
    e = Episode(SOLVED)
    e.step(0.3, left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
    e.step(0.1, left=_home("left"))
    assert e.step(0.5)[0] == pytest.approx(0.8)
    assert e.step(1.0) == (1.0, True)


def test_the_tube_creeping_out_of_the_pockets_restarts_the_second():
    e = Episode(SOLVED)
    e.step(0.8)
    p, q = e.poses["tube"]
    e.step(0.1, tube=(p + np.array([0.0, 0.003, 0.0]), q))  # 3 mm along its axis
    assert e.step(0.9)[1] is False


def test_the_demo_has_both_arms_and_a_quarter_turn():
    d = T.demo()
    assert set(d) == {"left", "right"}
    assert any(w.label == "turn the tube" for w in d["left"]) and any(w.label == "turn the tube" for w in d["right"])


def test_a_lapse_restarts_the_second():  # replaces the shared one: a tube 4 mm out of its holders is no longer fitted at all
    e = Episode(SOLVED)
    e.step(0.8)
    p, q = e.poses["tube"]
    e.step(0.1, tube=(p + np.array([0.004, 0.0, 0.0]), q))
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    e.step(0.1, tube=(p, q))  # put back: the second counts again from here
    score, ok = e.step(0.5)
    assert not ok
    assert e.step(1.0)[1]
