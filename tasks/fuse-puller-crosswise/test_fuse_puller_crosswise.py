"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome (the jaws closed on the fuse, the fuse out of its pocket by the
puller, the fuse in the tray, the jaws open, the puller set down and let go, the grippers and the puller clear), jaws closed on nothing or beside the fuse are not clamped, a fuse taken out without the
puller clamped on it limits the score for the rest of the run (even if the puller grips it later), the fuse or puller off the table scores 0, the 1 s
full-success timing and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_fuse_puller_crosswise.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "fuse_puller_crosswise.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
CLOSED = math.radians(1.8)
EMPTY = math.radians(4.4)
JAW_MID = 0.5 * (T.JAW_X0 + T.JAW_X1)

START = {"fuse": (np.array(T.FUSE_START), IDENT), "puller": (np.array(T.PULLER_START), IDENT),
         **{n: (np.array(p), IDENT) for n, p in T.NEIGHBOUR_START.items()}}


def _puller_on_fuse(dy=0.0, dz=0.0):
    """The puller's pose with its jaws straddling the fuse in its pocket: the fuse's middle JAW_MID along the puller's x, 3 mm under the jaws' mid height."""
    c = np.array([T.FUSE_START[0], T.FUSE_START[1] + dy, T.FUSE_START[2] + T.FUSE_H / 2])
    return (c - np.array([JAW_MID, 0.0, -0.003 + dz]), IDENT)


def _fuse_in_tray():
    return (np.array([T.TRAY_XY[0], T.TRAY_XY[1], T.TOP + T.TRAY_T + 0.0003]), IDENT)


def _puller_away():
    """The puller laid on the table where it started, far from the fuse in the tray."""
    return (np.array(T.PULLER_START), IDENT)


SOLVED = {"fuse": _fuse_in_tray(), "puller": _puller_away(), **{n: (np.array(p), IDENT) for n, p in T.NEIGHBOUR_START.items()}}
NEAR_TCP = np.array([T.TRAY_XY[0], T.TRAY_XY[1], T.TOP + 0.02])
RULE_CASES = [lambda e: {"fuse": (np.array([T.TRAY_XY[0], T.TRAY_XY[1], T.TOP - 0.06]), IDENT)},
              lambda e: {"puller": (np.array([0.2, 0.0, T.TOP - 0.06]), IDENT)}]


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
# -- this task's episode: the puller's arm angles, and what the puller has done ------------------------------------------------------------
class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None, arms=(0.0, 0.0)):
        super().__init__(poses)
        self.arms = arms
        if poses is SOLVED:
            T._PROGRESS.mem["clamped"] = True
            T._PROGRESS.mem["out_seen"] = True  # the fuse was drawn out of its pocket by the clamped puller

    def objects(self):
        out = super().objects()
        out["arm_p"], out["arm_m"] = -self.arms[0], self.arms[1]
        return out

    def step(self, seconds=0.05, **changes):
        if "arms" in changes:
            self.arms = changes.pop("arms")
        return super().step(seconds, **changes)


def _solved_with(**changes):
    e = Episode(SOLVED)
    e.step(0.1, **changes)
    return e.done()


def test_the_jaws_closed_on_the_fuse_are_step_one_and_remembered():
    e = Episode()
    e.step(0.2, puller=_puller_on_fuse(), arms=(CLOSED, CLOSED))
    assert e.done() == [True, False, False, False]
    e.step(0.2, puller=START["puller"], arms=(0.0, 0.0))
    assert e.done()[0] is True


def test_jaws_closed_on_nothing_are_not_clamped():
    e = Episode()
    e.step(0.2, puller=_puller_on_fuse(), arms=(EMPTY, EMPTY))
    assert e.done()[0] is False


def test_jaws_beside_the_fuse_are_not_clamped():
    e = Episode()
    e.step(0.2, puller=_puller_on_fuse(dy=0.007), arms=(CLOSED, CLOSED))
    assert e.done()[0] is False


def test_jaws_above_the_fuse_are_not_clamped():
    e = Episode()
    e.step(0.2, puller=_puller_on_fuse(dz=0.040), arms=(CLOSED, CLOSED))
    assert e.done()[0] is False


def test_the_fuse_in_the_tray_after_the_puller_completes_all_steps():
    assert _solved_with() == [True, True, True, True]


def test_the_fuse_in_the_tray_with_the_jaws_still_closed_is_not_clear():
    d = _solved_with(arms=(CLOSED, CLOSED))
    assert d[2] is True and d[3] is False


def test_the_fuse_on_the_table_beside_the_block_is_not_in_the_tray():
    d = _solved_with(fuse=(np.array([T.BLOCK_XY[0] - 0.10, T.BLOCK_XY[1] - 0.06, T.TOP + 0.0003]), IDENT))
    assert d[1] is True and d[2] is False and d[3] is False


def test_the_fuse_back_in_its_pocket_is_not_out():
    d = _solved_with(fuse=(np.array(T.FUSE_START), IDENT))
    assert d[1] is False and d[2] is False


def test_a_fuse_lifted_bare_limits_the_score_even_if_the_puller_grips_it_later():
    e = Episode()
    p, q = START["fuse"]
    e.step(0.2, fuse=(p + np.array([0.0, 0.0, 0.012]), q))  # lifted with nothing clamped on it
    assert T._PROGRESS.mem.get("bare") is True
    e.step(0.2, fuse=START["fuse"])  # put back
    e.step(0.2, puller=_puller_on_fuse(), arms=(CLOSED, CLOSED))  # now clamped, too late
    e.step(0.2, puller=_puller_away(), arms=(0.0, 0.0), fuse=_fuse_in_tray())
    assert e.done() == [True, False, False, False]
    assert e.step(3.0)[0] < 1.0


def test_a_fuse_pulled_with_the_puller_is_not_bare():
    e = Episode()
    e.step(0.2, puller=_puller_on_fuse(), arms=(CLOSED, CLOSED))
    p, q = START["fuse"]
    pp, pq = _puller_on_fuse()
    e.step(0.2, puller=(pp + np.array([0.0, 0.0, 0.012]), pq), fuse=(p + np.array([0.0, 0.0, 0.012]), q), arms=(CLOSED, CLOSED))
    assert not T._PROGRESS.mem.get("bare")


def test_the_fuse_still_pinched_in_a_gripper_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.2, right=_gripper(NEAR_TCP, contact=15.0))
    assert e.done()[3] is False
    assert e.step(3.0)[0] < 1.0


def test_a_fuse_pinched_out_by_a_gripper_after_the_jaws_once_closed_on_it_is_still_bare():
    e = Episode()
    e.step(0.2, puller=_puller_on_fuse(), arms=(CLOSED, CLOSED))
    assert e.done()[0] is True
    e.step(1.0, puller=_puller_away(), arms=(0.0, 0.0))  # the puller let go and left
    p, q = START["fuse"]
    e.step(0.2, fuse=(p + np.array([0.0, 0.0, 0.040]), q))  # the fuse leaves its pocket with no jaws on it
    assert T._PROGRESS.mem.get("bare") is True
    e.step(0.2, fuse=_fuse_in_tray())
    assert e.done()[1:] == [False, False, False]
    assert e.step(3.0)[0] < 1.0


# -- step 4: the puller must be truly clear of the fuse and set down -------------------------------------------------------
def _puller_around_fuse_in_tray(dz=0.0):
    """The puller's jaws straddling the fuse where it lies in the tray (flat), as when the demo used to let go of it there."""
    c = np.array([T.TRAY_XY[0], T.TRAY_XY[1], T.TOP + T.TRAY_T + T.FUSE_H / 2])
    return (c - np.array([JAW_MID, 0.0, -0.003 + dz]), IDENT)


def test_the_puller_left_around_the_fuse_in_the_tray_is_not_clear():
    d = _solved_with(puller=_puller_around_fuse_in_tray(), arms=(0.0, 0.0))
    assert d[2] is True and d[3] is False
    e = Episode(SOLVED)
    e.step(0.1, puller=_puller_around_fuse_in_tray())
    assert e.step(3.0)[0] < 1.0


def test_the_puller_lifted_clear_but_still_over_the_fuse_is_not_clear():
    e = Episode(SOLVED)
    p, q = _puller_around_fuse_in_tray(dz=0.020)
    e.step(0.1, puller=(p + np.array([0.0, 0.0, 0.020]), q))  # 2 cm above the fuse: inside the 3 cm margin
    assert e.done()[3] is False


@pytest.mark.parametrize("dx", [0.05, 0.08, 0.15])
def test_the_puller_set_down_beyond_the_margin_is_clear(dx):
    far = np.array([T.TRAY_XY[0] - dx - 0.12, T.TRAY_XY[1], T.TOP + 0.0003])  # the head end is the nearest part
    assert _solved_with(puller=(far, IDENT))[3] is True


def test_the_puller_just_inside_the_margin_is_not_clear():
    # the jaw tips (x = 0.102) 25 mm short of the fuse's end face along x: inside 30 mm
    x = T.TRAY_XY[0] - T.FUSE_W / 2 - 0.025 - 0.102
    assert _solved_with(puller=(np.array([x, T.TRAY_XY[1], T.TOP + 0.0003]), IDENT))[3] is False


def test_the_puller_just_outside_the_margin_is_clear():
    x = T.TRAY_XY[0] - T.FUSE_W / 2 - 0.040 - 0.102
    assert _solved_with(puller=(np.array([x, T.TRAY_XY[1], T.TOP + 0.0003]), IDENT))[3] is True


def test_the_puller_still_held_in_a_gripper_is_not_set_down():
    e = Episode(SOLVED)
    pu = _puller_away()[0]
    e.step(0.2, right=_gripper(pu + np.array([0.082, 0.0, 0.008]), contact=15.0))
    assert e.done()[3] is False


def test_the_puller_in_the_air_is_not_set_down():
    p, q = _puller_away()
    assert _solved_with(puller=(p + np.array([0.0, 0.0, 0.05]), q))[3] is False


def test_the_puller_standing_on_its_head_is_not_set_down():
    p, q = _puller_away()
    turned = np.array([0.0, math.sin(math.radians(45)), 0.0, math.cos(math.radians(45))])  # 90 degrees about y
    assert _solved_with(puller=(p + np.array([0.0, 0.0, 0.06]), turned))[3] is False


def test_the_demo_ends_with_the_puller_down_beside_the_block():
    wps = T.demo()["right"]
    assert wps[-3].label == "let go of the puller" and wps[-4].label == "lay it on the table"


@pytest.mark.parametrize("name", ["fuse_a", "fuse_b"])
def test_moving_a_neighbouring_fuse_scores_zero_for_the_rest_of_the_run(name):
    e = Episode(SOLVED)
    e.step(0.3)
    p, q = SOLVED[name]
    e.step(0.1, **{name: (p + np.array([0.0, 0.0, 0.010]), q)})  # lifted 10 mm
    assert e.step(0.1)[0] == 0.0
    e.step(0.1, **{name: SOLVED[name]})  # put back
    score, ok = e.step(3.0)
    assert score == 0.0 and not ok


def test_a_neighbouring_fuse_shifted_2_mm_is_still_in_place():
    e = Episode(SOLVED)
    p, q = SOLVED["fuse_a"]
    e.step(0.1, fuse_a=(p + np.array([0.0, 0.002, 0.0]), q))
    assert e.step(1.5)[0] == 1.0
