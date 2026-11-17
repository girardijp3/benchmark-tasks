"""Offline checks (no GPU, no network): the task file loads, the declared cutting effect (two blades, 12 sectors, clockwise only,
needs the blade engaged in the burr, the tube lifted and held), the step ladder, the 1 s full-success timing, the latches (an
over-reamed rim, a dropped tube, a part off the table) and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_copper_tube_deburr_reamer.py"""

import math
import os
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "copper_tube_deburr_reamer.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _yaw(a):
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def _tilted(deg, axis=0):
    a = math.radians(deg) / 2
    q = np.zeros(4)
    q[axis] = math.sin(a)
    q[3] = math.cos(a)
    return q


START = {"tube": (np.array(T.TUBE_START, float), IDENT), "tool": (np.array(T.TOOL_START, float), IDENT)}
SOLVED = dict(START)  # the tube standing in its rack, the reamer in its rest
UP = np.array([0.0, 0.0, 0.03])  # the tube held 30 mm above its seat
TUBE_UP = np.array(T.TUBE_START, float) + UP


def _tool_in_bore(phi=0.0, depth=None, off=0.0, tilt=0.0, tube=TUBE_UP):
    """The reamer's pose with its cone's tip ``depth`` below the held tube's rim, turned ``phi`` about the tube's axis."""
    d = T.SEAT_DEPTH if depth is None else depth
    q = _yaw(phi) if not tilt else (_yaw(phi) + 0 * _tilted(tilt))
    if tilt:
        h = math.radians(tilt) / 2
        qy = np.array([math.sin(h), 0.0, 0.0, math.cos(h)])
        zq = _yaw(phi)
        q = np.array([zq[3] * qy[0], zq[3] * qy[1] + zq[2] * qy[0], zq[2] * qy[3], zq[3] * qy[3]])
        q /= np.linalg.norm(q)
    return (np.array([tube[0] + off, tube[1], tube[2] + T.TUBE_L - d]), q)


HOLD_TCP = np.array(T.TUBE_START, float) + np.array([0.0, 0.0, 0.05]) + UP  # pads across the held tube
NEAR_TCP = np.array(T.TUBE_START, float) + np.array([0.0, 0.0, 0.05])
OFF_TABLE = (np.array([0.3, 0.0, T.TOP - 0.06]), IDENT)
RULE_CASES = [
    lambda e: {"tube": OFF_TABLE},  # the tube falls off the table
    lambda e: {"tool": OFF_TABLE},  # the reamer falls off the table
    lambda e: {"tube": (np.array(T.TUBE_START, float) + np.array([0.0, 0.0, 0.01]), _tilted(90.0))},  # the tube knocked over
]




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


# -- this task's episode: the cutting effect and the earlier lift are remembered ---------------------------------------------------
@pytest.fixture(autouse=True)
def fresh_cut():
    T._CUT.__init__()
    yield


class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None):
        super().__init__(poses)
        if poses is SOLVED:  # the burr is gone, the blade was engaged in the lifted tube earlier
            T._CUT.passes[:] = 2
            T._PROGRESS.mem["engaged_up"] = True


def _held_session(e):
    """The tube held 30 mm up by the left gripper with the blade engaged: returns the episode ready to turn."""
    e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0), left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
    return e


def _turn(e, start_deg, end_deg, seconds=2.0, **kw):
    """The reamer turned from ``start_deg`` to ``end_deg`` (clockwise = falling) in 0.05 s steps."""
    n = max(2, int(seconds / 0.05))
    for i in range(1, n + 1):
        phi = math.radians(start_deg + (end_deg - start_deg) * i / n)
        e.step(0.05, tool=_tool_in_bore(phi, **kw))


def test_a_clockwise_half_turn_while_engaged_and_held_gives_every_sector_one_pass():
    e = _held_session(Episode())
    _turn(e, 0.0, -190.0)
    assert int(T._CUT.passes.min()) == 1 and int(T._CUT.passes.max()) == 1
    assert e.done()[:3] == [True, True, False]
    assert float(T._CUT.burr().max()) == pytest.approx(0.2 * T.MM)


def test_a_full_clockwise_turn_clears_the_burr_all_round():
    e = _held_session(Episode())
    _turn(e, 0.0, -370.0, seconds=4.0)
    assert int(T._CUT.passes.min()) == 2 and float(T._CUT.burr().max()) == 0.0
    assert e.done()[:3] == [True, True, True]


def test_turning_counter_clockwise_cuts_nothing():
    e = _held_session(Episode())
    _turn(e, 0.0, 400.0, seconds=4.0)
    assert int(T._CUT.passes.max()) == 0 and e.done()[1] is False


def test_turning_back_and_forth_over_the_same_arc_is_counted_each_time_it_is_cut_but_jitter_is_not():
    e = _held_session(Episode())
    for _ in range(10):  # +-1.5 degrees of jitter is under the 2 degree commit
        _turn(e, 0.0, -1.5, seconds=0.2)
        _turn(e, -1.5, 0.0, seconds=0.2)
    assert int(T._CUT.passes.max()) == 0


def test_a_hand_must_hold_the_tube():
    e = Episode()
    e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0))  # nobody holds the tube
    _turn(e, 0.0, -370.0, seconds=4.0)
    assert int(T._CUT.passes.max()) == 0


def test_the_tube_need_not_be_lifted_out_of_its_rack_but_a_hand_must_hold_it():
    e = Episode()
    low = np.array(T.TUBE_START, float)
    e.step(0.05, tube=(low, IDENT), tool=_tool_in_bore(0.0, tube=low), left=_gripper(NEAR_TCP, contact=15.0, opening=0.02))
    for i in range(1, 80):
        e.step(0.05, tool=_tool_in_bore(math.radians(-5.0 * i), tube=low))
    assert int(T._CUT.passes.max()) >= 1


def test_the_blade_torque_acts_only_while_engaged_and_turning_clockwise():
    e = Episode()
    e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0), left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
    for i in range(1, 6):
        e.step(0.05, tool=_tool_in_bore(math.radians(-5.0 * i)))
    assert T._CUT.drag
    for i in range(1, 6):
        e.step(0.05, tool=_tool_in_bore(math.radians(-25.0 + 5.0 * i)))  # turning back
    assert not T._CUT.drag
    for depth in (0.003, 0.006):  # inside the bore but not touching the burr
        e.step(0.05, tool=_tool_in_bore(math.radians(-10.0), depth=depth))
        assert not T._CUT.drag


def test_a_centered_cone_at_six_mm_has_no_blade_contact_and_cuts_nothing():
    """The reviewer's case: a 6 mm seat leaves the cone's 7.86 mm edge 1.84 mm clear of the 9.70 mm burr, so turning earns nothing."""
    e = Episode()
    e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0, depth=0.006), left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
    edge_r, bite, force = T.blade_contact(e.objects())
    assert edge_r == pytest.approx(0.007857, abs=1e-6)
    assert bite == pytest.approx(-0.001843, abs=1e-6)
    assert force == 0.0
    assert T.engaged(e.objects()) is False
    _turn(e, 0.0, -370.0, seconds=4.0, depth=0.006)
    assert int(T._CUT.passes.max()) == 0
    assert float(T._CUT.burr().max()) == pytest.approx(0.4 * T.MM)
    assert e.done()[0] is False and e.done()[1] is False
    assert not T._CUT.drag


def test_the_blade_is_engaged_at_the_natural_seat():
    e = _held_session(Episode())
    edge_r, bite, force = T.blade_contact(e.objects())
    assert edge_r == pytest.approx(T.TUBE_RI, abs=1e-6)  # 10.100 mm: the cone's edge circle exactly on the bore wall
    assert bite == pytest.approx(0.400 * T.MM, abs=1e-6)
    assert force >= T.CUT_FORCE
    assert T.engaged(e.objects())


def test_a_jammed_cone_past_the_wall_does_not_count():
    e = Episode()
    e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0, depth=0.0105), left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
    edge_r, bite, force = T.blade_contact(e.objects())
    assert bite > T.ENGAGE_MAX
    assert T.engaged(e.objects()) is False
    _turn(e, 0.0, -370.0, seconds=4.0, depth=0.0105)
    assert int(T._CUT.passes.max()) == 0
    assert e.done()[0] is False


def test_the_engagement_band_ends_at_the_bore_wall():
    for depth, want in ((T.SEAT_MAX - 1e-6, True), (T.SEAT_MAX + 0.001, False)):
        T._CUT.__init__()
        T._PROGRESS.__init__()
        e = Episode()
        e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0, depth=depth),
               left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
        assert T.engaged(e.objects()) is want, depth


def test_the_blade_must_be_engaged():
    for kw in ({"depth": 0.003}, {"depth": 0.006}, {"depth": 0.008}, {"off": 0.006}, {"tilt": 12.0}):
        T._CUT.__init__()
        T._PROGRESS.__init__()
        e = Episode()
        e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0, **kw), left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
        _turn(e, 0.0, -370.0, seconds=4.0, **kw)
        assert int(T._CUT.passes.max()) == 0, kw
        assert e.done()[0] is False, kw


def test_a_seat_a_little_off_axis_still_cuts():
    e = Episode()
    e.step(0.05, tube=(TUBE_UP, IDENT), tool=_tool_in_bore(0.0, off=0.002, tilt=4.0), left=_gripper(HOLD_TCP, contact=15.0, opening=0.02))
    _turn(e, 0.0, -190.0, off=0.002, tilt=4.0)
    assert int(T._CUT.passes.min()) == 1


def test_turning_on_far_past_the_burr_chamfers_the_wall_thin_and_caps_the_score():
    e = _held_session(Episode())
    _turn(e, 0.0, -370.0, seconds=4.0)
    assert e.done()[2] is True
    _turn(e, -370.0, -1100.0, seconds=8.0)  # about two more turns
    assert float(T._CUT.chamfer().max()) > T.THIN
    assert e.done()[2] is False and e.done()[3] is False
    e.step(0.1, tube=SOLVED["tube"], tool=SOLVED["tool"], left=_home("left"))
    score, ok = e.step(3.0)
    assert score == pytest.approx(2 * T.POINTS) and not ok  # steps 1 and 2 only


def test_a_little_chamfer_is_fine():
    e = _held_session(Episode())
    _turn(e, 0.0, -620.0, seconds=6.0)  # 1.7 turns: the third pass in some sectors chamfers 0.2 mm
    assert 0.0 < float(T._CUT.chamfer().max()) <= T.THIN and e.done()[2] is True


def test_the_reamer_left_in_the_bore_is_not_done():
    e = Episode(SOLVED)
    e.step(0.1, tube=(np.array(T.TUBE_START, float), IDENT), tool=_tool_in_bore(0.0, tube=np.array(T.TUBE_START, float)))
    assert e.done()[3] is False


def test_the_tube_left_standing_loose_in_the_rack_counts():
    p = np.array(T.TUBE_START, float) + np.array([0.0015, 0.0, 0.0])
    e = Episode(SOLVED)
    e.step(0.1, tube=(p, _tilted(8.0)))
    assert e.done()[3] is True


def test_the_tube_not_back_in_the_rack_is_not_done():
    for p, q in ((np.array(T.TUBE_START, float) + np.array([0.03, 0.0, 0.0]), IDENT), (np.array(T.TUBE_START, float) + UP, IDENT),
                 (np.array(T.TUBE_START, float), _tilted(25.0))):
        e = Episode(SOLVED)
        e.step(0.1, tube=(p, q))
        assert e.done()[3] is False


def test_a_hand_still_holding_the_reamer_is_not_done():
    e = Episode(SOLVED)
    p = np.array(T.TOOL_START, float)
    e.step(0.2, right=_gripper(p + np.array([0.0, 0.0, 0.045]), contact=15.0, opening=0.03))
    assert e.done()[3] is False


def test_a_lapse_restarts_the_second():  # noqa: F811  (the reamer in its rest shifted 4 mm keeps every condition: only stillness lapses)
    e = Episode(SOLVED)
    e.step(0.8)
    p, q = e.poses["tool"]
    e.step(0.1, tool=(p + np.array([0.004, 0.0, 0.0]), q))
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok
