"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome (including that a
cross-contamination event scores 0 for the rest of the run), the 1 s full-success timing, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_haccp_boards.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "haccp_boards.py")
T = TASK.module

DOWN = np.array([0.0, 0.0, -1.0])
UP = np.array([0.0, 0.0, 0.0, 1.0])
NAMES = ("pork", "thigh", "cod")


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


def _state(pos, quat=UP):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _on(name, colour=None, dx=0.0, dy=0.0, lift=0.0):
    """A cut lying flat on the ``colour`` board (its own by default), its origin ``dx``, ``dy`` off the board's centre
    and ``lift`` above the top face."""
    cx, cy = T.BOARD_XY[colour or T.OWN_BOARD[name]]
    return np.array([cx + dx, cy + dy, T.BOARD_TOP + 0.0003 + lift])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos = {n: np.array([*T.CUT_XY[n], T.PAN_TOP_FLOOR + 0.0003]) for n in NAMES}
        self.quat = {n: UP.copy() for n in NAMES}
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {f"board_{c}": _state([*T.BOARD_XY[c], T.TOP]) for c in T.BOARD_XY}
        out["prep_pan"], out["sanitiser_tub"] = _state([*T.PAN_XY, T.TOP]), _state([*T.TUB_XY, T.TOP])
        out.update({n: _state(self.pos[n], self.quat[n]) for n in NAMES})
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            else:
                getattr(self, k).update(v)
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def done(self):
        return T.steps_done(self.objects())

    def set_on_boards(self, seconds=0.05, **offsets):
        return self.step(seconds, pos={n: _on(n, **offsets.get(n, {})) for n in NAMES})

    def hold(self, side, name, tcp=None, contact=20.0, opening=None):
        """The ``side`` gripper pinching ``name`` where it is (or with its TCP at ``tcp``)."""
        s = self.pos[name]
        tcp = (s + np.array([T.PINCH[name][0], 0.0, 0.017])) if tcp is None else tcp
        return self.step(**{side: _gripper(tcp, contact=contact,
                                           opening=T.PINCH[name][1] - 0.008 if opening is None else opening)})


def test_loads_and_counts_objects():
    assert len(TASK.objects) == 8
    assert {o.name for o in TASK.objects} == {"board_red", "board_yellow", "board_blue", "prep_pan", "sanitiser_tub",
                                              "pork", "thigh", "cod"}


def test_nothing_done_scores_zero_and_untouched_cuts_are_clean():
    e = Episode()
    for _ in range(10):
        assert e.step() == (0.0, False)
    assert e.done() == [False] * 4
    assert T._PROGRESS.events == []


def test_each_cut_on_its_own_board_scores_a_step():
    e = Episode()
    e.step(pos={"pork": _on("pork")})
    assert e.done() == [True, False, False, False]
    e.step(pos={"thigh": _on("thigh")})
    assert e.done() == [True, True, False, False]
    assert e.step(pos={"cod": _on("cod")})[0] == pytest.approx(0.8)  # the grippers are clear: the fourth step
    assert e.done() == [True, True, True, True]


def test_colour_code_is_meat_red_poultry_yellow_fish_blue():
    assert T.OWN_BOARD == {"pork": "red", "thigh": "yellow", "cod": "blue"}


def test_steps_alone_stay_below_one():
    assert T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def test_all_conditions_for_one_second_is_success_and_not_before():
    e = Episode()
    e.set_on_boards()
    seen = []
    for _ in range(40):
        seen.append(e.step())
        if seen[-1][1]:
            break
    t_met = T._PROGRESS.met_since
    assert seen[-1] == (1.0, True)
    assert e.t - t_met == pytest.approx(T.STILL_S, abs=0.06)
    assert all(score <= 0.8 + 1e-9 for score, ok in seen[:-1] if not ok)
    # 0.95 s in: not yet
    e = Episode()
    e.step(pos={n: _on(n) for n in NAMES})
    score, ok = e.step(0.9)  # the timing began at the first step above: 0.95 s so far
    assert ok is False and score == pytest.approx(0.8)


@pytest.mark.parametrize("name", NAMES)
def test_a_cut_on_a_wrong_board_scores_zero(name):
    wrong = next(c for c in T.BOARD_XY if c != T.OWN_BOARD[name])
    e = Episode()
    assert e.step(pos={name: _on(name, wrong)})[0] == 0.0
    assert any(f"{name} touched" in ev for ev in T._PROGRESS.events)


def test_a_brief_touch_of_a_wrong_board_costs_the_whole_run():
    """Every cut then ends on its own board, still, for as long as you like: never 1.0, never a step's credit."""
    e = Episode()
    e.step(pos={"pork": _on("pork", "yellow")})  # touches the yellow board for one control step
    e.set_on_boards(seconds=0.5)
    for _ in range(80):
        score, ok = e.step()
        assert score == 0.0 and ok is False
    assert e.done()[:3] == [True, True, True]  # the steps themselves are as they stand; the run scores 0


def test_a_cut_hovering_over_a_wrong_board_does_not_touch_it():
    e = Episode()
    assert e.step(pos={"pork": _on("pork", "yellow", lift=0.02)})[0] == 0.0
    assert T._PROGRESS.events == []
    assert T.wrong_board(e.objects(), "pork") is None


def test_a_cut_against_a_wrong_boards_side_touches_it():
    e = Episode()
    x = T.BOARD_XY["yellow"][0]
    # lying on the table (below the board's top) with its middle inside the yellow board's footprint
    e.step(pos={"pork": np.array([x, T.BOARD_XY["yellow"][1], T.TOP + 0.0003])})
    assert T._PROGRESS.events


def test_a_cut_half_on_its_board_is_not_on_it():
    e = Episode()
    e.step(pos={"pork": _on("pork", dx=0.0, dy=0.14)})  # hangs over the red board's edge toward the yellow one
    assert e.done()[0] is False


def test_a_cut_on_the_table_is_not_on_its_board():
    e = Episode()
    e.step(pos={"pork": np.array([0.40, -0.52, T.TOP + 0.0003])})
    assert e.done()[0] is False


def test_a_cut_tipped_on_its_board_is_not_set_flat():
    e = Episode()
    q = np.array([math.sin(math.radians(15)), 0.0, 0.0, math.cos(math.radians(15))])
    e.step(pos={"pork": _on("pork", lift=0.004)}, quat={"pork": q})
    assert e.done()[0] is False


def test_a_cut_off_the_table_scores_zero():
    e = Episode()
    assert e.step(pos={"cod": np.array([0.3, 0.9, T.TOP - 0.2])})[0] == 0.0


def test_holding_one_kind_then_gripping_another_unsanitised_scores_zero():
    e = Episode()
    e.hold("left", "thigh")
    assert T._PROGRESS.dirty["left"] == "thigh"
    e.step(left=_home("left"))
    e.hold("left", "cod")
    assert any("unsanitised after the thigh" in ev for ev in T._PROGRESS.events)
    assert e.step()[0] == 0.0


def test_gripping_the_same_kind_again_is_fine_and_other_hand_is_independent():
    e = Episode()
    e.hold("left", "thigh")
    e.step(left=_home("left"))
    e.hold("left", "thigh")
    e.hold("right", "pork")
    assert T._PROGRESS.events == []


def test_open_pads_touching_a_cut_are_not_holding_it():
    e = Episode()
    e.hold("left", "thigh", contact=0.0, opening=0.08)
    assert T._PROGRESS.dirty["left"] is None


def _dip(e, seconds, opening=0.09, depth=0.02, side="left", contact=0.0):
    x, y = T.TUB_XY
    tcp = np.array([x, y, T.TOP + depth + T.PAD_TIP_PAST_TCP])
    return e.step(seconds, **{side: _gripper(tcp, contact=contact, opening=opening)})


def test_dipping_open_pads_in_the_sanitiser_for_two_seconds_cleans_them():
    e = Episode()
    e.hold("left", "thigh")
    e.step(left=_home("left"))
    _dip(e, T.DIP_S - 0.2)
    assert T._PROGRESS.dirty["left"] == "thigh"
    _dip(e, 0.4)
    assert T._PROGRESS.dirty["left"] is None
    e.hold("left", "cod")
    assert T._PROGRESS.events == []


@pytest.mark.parametrize("what", ["shallow", "closed", "outside", "holding", "interrupted"])
def test_a_dip_that_is_too_shallow_closed_outside_holding_or_interrupted_does_not_clean(what):
    e = Episode()
    e.hold("left", "thigh")
    e.step(left=_home("left"))
    if what == "shallow":
        _dip(e, 3.0, depth=0.035)  # the tips are only 5 mm under the surface
    elif what == "closed":
        _dip(e, 3.0, opening=0.01)
    elif what == "outside":
        e.step(3.0, left=_gripper(np.array([T.TUB_XY[0] + 0.2, T.TUB_XY[1], T.TOP + 0.03])))
    elif what == "holding":
        e.pos["thigh"] = np.array([T.TUB_XY[0] - T.PINCH["thigh"][0], T.TUB_XY[1], T.TOP + 0.003])
        _dip(e, 3.0, contact=20.0, opening=0.02)
    else:
        _dip(e, 1.5)
        e.step(0.1, left=_home("left"))
        _dip(e, 1.5)
    assert T._PROGRESS.dirty["left"] == "thigh"


def test_unsanitised_pads_touching_another_kinds_board_score_zero():
    e = Episode()
    e.hold("left", "thigh")
    # a pad tip on the blue board (cod) with thigh on the pads
    cx, cy = T.BOARD_XY["blue"]
    e.step(left=_gripper(np.array([cx, cy, T.BOARD_TOP + 0.001 + T.PAD_TIP_PAST_TCP]), opening=0.08))
    assert any("touched the blue board" in ev for ev in T._PROGRESS.events)
    assert e.step()[0] == 0.0


def test_pads_over_a_board_without_touching_it_are_fine():
    e = Episode()
    e.hold("left", "thigh")
    cx, cy = T.BOARD_XY["blue"]
    e.step(left=_gripper(np.array([cx, cy, T.BOARD_TOP + 0.006 + T.PAD_TIP_PAST_TCP]), opening=0.08))
    assert T._PROGRESS.events == []


def test_pads_of_the_right_kind_touching_their_own_board_are_fine():
    e = Episode()
    e.hold("right", "pork")
    cx, cy = T.BOARD_XY["red"]
    e.step(right=_gripper(np.array([cx, cy, T.BOARD_TOP + 0.001 + T.PAD_TIP_PAST_TCP]), opening=0.08))
    assert T._PROGRESS.events == []


def test_success_needs_every_gripper_clear():
    e = Episode()
    e.set_on_boards()
    near = _gripper(np.array([*_on("cod")[:2], T.BOARD_TOP + 0.017]), contact=0.0, opening=0.07)
    for _ in range(60):
        score, ok = e.step(left=near)
        assert ok is False and score <= 0.6 + 1e-9
    assert e.done() == [True, True, True, False]


def test_success_needs_the_cuts_still_and_a_move_restarts_the_second():
    e = Episode()
    e.set_on_boards()
    e.step(0.5)
    e.step(pos={"cod": _on("cod", dx=0.004)})  # slides 4 mm
    for _ in range(18):  # not yet a second since the slide
        assert e.step()[1] is False
    assert e.step(0.3)[1] is True


def test_success_needs_all_three_cuts():
    e = Episode()
    e.step(pos={"pork": _on("pork"), "thigh": _on("thigh")})
    for _ in range(60):
        assert e.step()[1] is False


def test_no_grip_free_cheat_other_boards_unused_does_not_matter():
    """Putting the cuts on their boards without ever touching them is a state the scene cannot reach (they start on the
    pan); the scorer itself only looks at where they are, so the demo's grippers must have moved them."""
    e = Episode()
    assert e.step()[0] == 0.0


def test_score_is_zero_for_a_nonfinite_state():
    e = Episode()
    e.set_on_boards()
    assert e.step(pos={"cod": np.array([np.nan, 0.0, 0.8])})[0] == 0.0


def test_a_new_episode_forgets_the_last_ones_contamination():
    e = Episode()
    e.step(0.5, pos={"pork": _on("pork", "yellow")})
    assert T._PROGRESS.events
    e = Episode()
    e.step()
    assert T._PROGRESS.events == []


def test_masses_and_sizes_are_real():
    assert T.CUT_MASS == {"pork": 0.131, "thigh": 0.114, "cod": 0.193}
    for n, (lo_hi, size) in {"pork": ((112, 69, 22), 0.112), "thigh": ((100, 67, 24), 0.100),
                             "cod": ((170, 58, 28), 0.170)}.items():
        lo, hi = T.CUT_BOX[n]
        d = (hi - lo) * 1000
        assert d == pytest.approx(lo_hi, abs=1.5)
    assert (T.BOARD_X, T.BOARD_Y, T.BOARD_T) == pytest.approx((0.229, 0.305, 0.0127))
    for n in NAMES:
        pts, tris = T._HULL[n]
        assert len(pts) <= 64


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_the_demo_pinches_every_cut_within_the_pads_reach():
    for n in NAMES:
        assert T.PINCH[n][1] + T.OPEN_EXTRA <= 0.094 - 0.004
