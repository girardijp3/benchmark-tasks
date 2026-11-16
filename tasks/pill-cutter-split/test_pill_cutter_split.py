"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome (the cut only counts with the
tablet seated, a half off the table scores 0 for the rest of the run, both halves in one cup is not a step), the 1 s
full-success timing, the cradle test, the lid fin's geometry and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_pill_cutter_split.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "pill_cutter_split.py")
T = TASK.module

SEAT = np.array([*T.SP_XY, T.TOP + T.SEAT_Z + 0.0004])
START = {
    "tablet": (np.array([*T.TB_XY, T.TABLET_Z]), np.array([0.0, 0.0, 0.0, 1.0])),
    "half_a": (np.array(T.PARK), np.array([0.0, 0.0, 0.0, 1.0])),
    "half_b": (np.array([T.PARK[0] + 0.05, T.PARK[1], T.PARK[2]]), np.array([0.0, 0.0, 0.0, 1.0])),
}


def _in_cup(cup, dx=0.0):
    cx, cy = T.CUPS[cup]
    return (np.array([cx + dx, cy, T.TOP + T.TRAY_T + 0.002 + T.TAB_T / 2]), np.array([0.0, 0.0, 0.0, 1.0]))


SOLVED = {"tablet": (np.array([T.PARK[0] - 0.05, T.PARK[1], 0.004]), np.array([0.0, 0.0, 0.0, 1.0])),
          "half_a": _in_cup("orange"), "half_b": _in_cup("blue")}
NEAR_TCP = np.array([*T.CUPS["orange"], T.TOP + T.TRAY_T + 0.03])
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _off(name):
    p, q = SOLVED[name]
    return (np.array([p[0], p[1], T.TOP - 0.06]), q)


RULE_CASES = [
    lambda e: {"half_a": _off("half_a")},  # a half falls off the table
    lambda e: {"half_b": _off("half_b")},
]



# -- shared checks ----------------------------------------------------------------------------------------------------
DOWN = np.array([0.0, 0.0, -1.0])
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()
    yield


def _gripper(tcp, contact=0.0, opening=0.09, axis=DOWN, closing=None):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact, closing=closing)


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


# -- this task's episode: also the lid's opening and whether the tablet has been cut -------------------------------------------
class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None, lid=95.0, split=None):
        super().__init__(poses)
        self.lid = lid
        self.split = (poses is SOLVED) if split is None else split
        if self.split:
            T._PROGRESS.mem["seated"] = True  # the tablet lay in the cradle earlier in this episode

    def objects(self):
        out = super().objects()
        out.update(lid_deg=self.lid, split=self.split)
        return out

    def step(self, seconds=0.05, **changes):
        for k in ("lid", "split"):
            if k in changes:
                setattr(self, k, changes.pop(k))
        return super().step(seconds, **changes)


def _seat(offset=(0.0, 0.0, 0.0), turn_deg=0.0, tilt_deg=0.0):
    a = math.radians(turn_deg) / 2
    quat = np.array([0.0, 0.0, math.sin(a), math.cos(a)])
    if tilt_deg:
        b = math.radians(tilt_deg) / 2
        quat = np.array([math.sin(b), 0.0, 0.0, math.cos(b)])
    return (SEAT + np.array(offset, float), quat)


def test_tablet_in_the_cradle_is_step_one_and_remembered():
    e = Episode()
    e.step(0.2, tablet=_seat())
    assert e.done()[0] is True and not any(e.done()[1:])
    e.step(0.2, tablet=START["tablet"])  # taken out again: the step stays reached
    assert e.done()[0] is True


@pytest.mark.parametrize("offset,turn,tilt", [((0.0, 0.004, 0.0), 0, 0), ((0.004, 0.0, 0.0), 0, 0), ((0.0, 0.0, 0.004), 0, 0),
                                              ((0.0, 0.0, 0.0), 12, 0), ((0.0, 0.0, 0.0), 0, 12)])
def test_a_tablet_off_the_blade_or_crooked_is_not_in_the_cradle(offset, turn, tilt):
    e = Episode()
    e.step(0.2, tablet=_seat(offset, turn, tilt))
    assert e.done()[0] is False


def test_a_tablet_within_tolerance_is_in_the_cradle():
    e = Episode()
    e.step(0.2, tablet=_seat((0.002, -0.002, 0.001), 4.0))
    assert e.done()[0] is True


def test_the_cut_is_step_two_and_the_lid_must_open_for_step_three():
    e = Episode()
    e.step(0.2, tablet=_seat())
    e.step(0.2, split=True, lid=1.0, tablet=SOLVED["tablet"], half_a=_seat((0, -0.0075, 0)), half_b=_seat((0, 0.0075, 0)))
    assert e.done()[:2] == [True, True] and e.done()[2] is False
    e.step(0.2, lid=60.0)
    assert e.done()[2] is False
    e.step(0.2, lid=90.0)
    assert e.done()[2] is True


def test_both_halves_in_one_cup_is_not_a_step():
    e = Episode(SOLVED)
    e.step(0.3, half_b=_in_cup("orange", dx=0.006))
    assert e.done()[3] is False and e.step(2.0)[0] < 1.0


def test_one_half_only_in_a_cup_is_not_a_step():
    e = Episode(SOLVED)
    e.step(0.3, half_b=(np.array([T.SP_XY[0], T.SP_XY[1], T.TOP + 0.014]), IDENT))
    assert e.done()[3] is False


def test_halves_beside_the_cup_are_not_in_it():
    e = Episode(SOLVED)
    cx, cy = T.CUPS["blue"]
    e.step(0.3, half_b=(np.array([cx + 0.021, cy, T.TOP + T.TRAY_T + 0.004]), IDENT))
    assert e.done()[3] is False


def test_the_lid_standing_shut_after_the_cut_is_not_opened():
    e = Episode(SOLVED)
    e.step(0.3, lid=5.0)
    assert e.done()[2] is False and e.step(3.0)[0] < 1.0


def test_a_gripper_at_the_lid_fin_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.3, right=_gripper(T.fin_point(95.0) + np.array([0.0, 0.0, 0.01])))
    assert e.done()[3] is False


def test_the_tablet_off_the_table_before_the_cut_scores_zero():
    e = Episode()
    e.step(0.2, tablet=_seat())
    e.step(0.2, tablet=(np.array([0.3, 0.3, T.TOP - 0.1]), IDENT))
    assert e.step(0.2)[0] == 0.0


def test_the_fin_is_where_the_hinge_geometry_puts_it():
    shut = T.fin_point(0.0)
    assert np.allclose(shut, T.SP_POS + T.HINGE_REL + T.FIN_REL)
    open_ = T.fin_point(95.0)
    # opened 95 degrees about the hinge the fin hangs to the robot's side of the hinge, 65 mm up
    assert open_[0] < (T.SP_POS + T.HINGE_REL)[0] and 0.05 < open_[2] - (T.SP_POS + T.HINGE_REL)[2] < 0.075


def test_the_cut_needs_the_lid_shut_and_the_tablet_in_the_cradle():
    assert T.CUT_DEG <= 3.0 and T.CUT_CENTRE <= 0.0025
    c = T.cradle_frame(SEAT)
    assert abs(c[0]) < 1e-9 and abs(c[1]) < 1e-9


def test_the_halves_are_half_a_tablet_and_apart_when_cut():
    assert abs(T.HALF_L * 2 - T.TAB_L) < 1e-9 and T.HALF_GAP >= 0.004


# -- the grip retune, the outcome coverage and the pass-through collider ------------------------------------------------
def test_the_caplet_pinch_is_gentle_enough_not_to_eject_it():
    """2.5 N a finger on the 9 mm caplet: above the 0.5 N hold threshold, far below the 20 N that ejected it on PhysX."""
    from yam_arena.constants import GRIPPER

    target = T._hold(T.TAB_W, T.TAB_SQUEEZE)
    held_at = -T.TAB_W / 2  # the caplet stops each finger at its 9 mm width
    force = GRIPPER["stiffness"] * (target - held_at)
    assert abs(-2 * target - (T.TAB_W - T.TAB_SQUEEZE)) < 1e-12  # the commanded opening
    assert T.GRIP_FORCE < force < 5.0, force


def test_the_caplet_in_the_dish_is_not_in_the_cradle():
    e = Episode()
    e.step(0.2, tablet=START["tablet"])
    assert e.done() == [False, False, False, False]
    assert e.step(1.0)[0] == 0.0


def test_a_shut_lid_without_the_cut_is_not_step_two():
    e = Episode()
    e.step(0.2, tablet=_seat())
    e.step(0.2, lid=1.0, split=False)  # shut, but no cut event
    assert e.done()[:2] == [True, False]
    assert e.step(1.0)[0] <= 0.2


def test_a_caplet_in_the_cradle_but_never_pinched_scores_only_step_one():
    e = Episode()
    e.step(0.2, tablet=_seat())
    e.step(0.2, lid=95.0, split=False)  # no lid pull, no cut
    assert e.done()[:2] == [True, False]
    assert e.step(1.0)[0] == 0.2


def test_the_blade_has_a_collider_only_above_the_caplet_in_the_cradle():
    """Pass-through: the visible blade is a solid; its collider starts at lid z 5 mm (table 16 mm with the lid shut),
    above the caplet's 15.5 mm top in the 11 mm cradle, so it cannot push the caplet or the halves at the cut."""
    text = (HERE / "assets" / "pill_cutter.usda").read_text()
    assert 'def Cube "BladeCollider' in text
    assert "float3 xformOp:scale = (0.016, 0.0008, 0.007)" in text
    assert "double3 xformOp:translate = (0.04, 0, 0.0085)" in text


def test_the_cradle_and_caplet_are_not_resized():
    assert T.TAB_W == 0.009 and T.TAB_L == 0.020 and T.TAB_T == 0.0075
    assert T.V_DEPTH == 0.0055 and T.V_SLOPE == 0.55
    assert T.HALF_GAP <= 0.005 and T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def _grip_at_caplet(opening):
    """The left gripper's pads at the caplet's mid-height, closing along x, at a commanded opening."""
    return _gripper(SEAT + np.array([0.0, 0.0, T.PAD_TIP_PAST_TCP]), contact=2.5, opening=opening,
                    closing=np.array([1.0, 0.0, 0.0]))


def test_a_normal_caplet_grip_is_not_a_penetration():
    e = Episode()
    e.step(0.2, tablet=_seat())
    e.step(0.2, left=_grip_at_caplet(T.TAB_W - T.TAB_SQUEEZE))  # the 8 mm commanded opening: 0.5 mm inside each face
    assert e.done()[0] is True
    assert e.step(0.2)[0] >= 0.2  # no latch: the pad face is within 0.5 mm of the caplet side


def test_a_pad_inside_the_caplet_is_a_rule_event():
    e = Episode()
    e.step(0.2, tablet=_seat())
    e.step(0.1, left=_grip_at_caplet(0.001))  # a 4 mm-deep pad penetration
    assert e.step(0.1)[0] == 0.0
