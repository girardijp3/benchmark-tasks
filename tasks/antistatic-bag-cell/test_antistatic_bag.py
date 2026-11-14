"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_antistatic_bag.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "antistatic_bag.py")
T = TASK.module

UP = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])
CELL_1 = np.array(T.CELL_POS[0])
CELL_LIFTED = CELL_1 + (0.0, 0.0, T.LIFT)
SEATED = T.cup_seat()
UPSIDE_DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # turned half a turn about x


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding_cell(centre):
    """A gripper closed on a cell standing with its centre at ``centre`` (pressing, pads apart)."""
    return _gripper(T.cell_tcp(centre), contact=10.0, opening=0.01)


def _holding_slider(travel):
    return _gripper(T.slider_tcp(travel), contact=10.0, opening=0.008)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0, spin=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.array([0.0, 0.0, spin]), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.travel = 0.0
        self.cell, self.cell_quat = CELL_1.copy(), UP.copy()
        self.left, self.right = _home("left"), _home("right")
        self.holder = np.array(T.HOLDER_POS)
        self.speed = 0.0
        self.spin = 0.0
        self.cell_2_away = False
        self.cell_2_quat = UP.copy()

    def objects(self):
        return {
            "bag": _state(T.BAG_POS, T._yaw_xyzw(T.BAG_YAW)),
            "holder": _state(self.holder, speed=self.speed),
            "cell_1": _state(self.cell, self.cell_quat, self.speed, self.spin),
            "cell_2": _state(np.array(T.CELL_POS[1]) + ((0.0, -0.2, 0.0) if self.cell_2_away else 0.0),
                             self.cell_2_quat),
            "left_gripper": self.left,
            "right_gripper": self.right,
            "episode_time": self.t,
            "slider_travel": self.travel,
            "panel_angles": T.panel_targets(self.travel),
        }

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

    def done(self):
        return T.steps_done(self.objects())

    def unzip(self):
        self.step(left=_holding_slider(0.0))
        self.step(travel=T.ZIP_TRAVEL, left=_holding_slider(T.ZIP_TRAVEL))
        self.step(left=_home("left"))

    def take_cell(self):
        self.step(right=_holding_cell(CELL_1))
        self.step(cell=CELL_LIFTED.copy(), right=_holding_cell(CELL_LIFTED))

    def insert_cell(self, quat=UP, at=SEATED):
        self.step(cell=np.asarray(at, float).copy(), cell_quat=np.asarray(quat, float).copy(), right=_home("right"))

    def full_course(self):
        self.unzip()
        self.take_cell()
        self.insert_cell()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"cell_1", "cell_2"}  # the bag is glued down, the holder taped


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_cells_start_in_the_bag():
    objects = Episode().objects()
    assert T.cell_in_bag(objects, "cell_1") and T.cell_in_bag(objects, "cell_2")


def test_panels_follow_the_slider():
    assert np.allclose(T.panel_targets(0.0)[:2], 0.0, atol=0.01)
    half = T.panel_targets(T.ZIP_TRAVEL / 2)
    assert np.allclose(half[:3], np.radians(T.PANEL_OPEN_DEG[:3])) and np.allclose(half[5:8], 0.0)
    full = T.panel_targets(T.ZIP_TRAVEL)
    assert np.allclose(full[:7], np.radians(T.PANEL_OPEN_DEG[:7])) and np.allclose(full[8:], -full[:8])


def test_unzipping_is_a_quarter():
    e = Episode()
    e.unzip()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.2


def test_half_unzipped_is_not_unzipped():
    e = Episode()
    e.step(travel=T.ZIP_TRAVEL / 2)
    assert e.done()[0] is False


def test_unzipped_counts_once_reached():
    e = Episode()
    e.unzip()
    e.step(travel=0.01)
    assert e.done()[0] is True


def test_cell_taken_out_is_a_step():
    e = Episode()
    e.unzip()
    e.take_cell()
    assert e.done() == [True, True, False, True] and e.step()[0] == 0.6


def test_cell_gripped_in_the_bag_is_not_taken_out():
    e = Episode()
    e.unzip()
    e.step(right=_holding_cell(CELL_1 + (0.0, 0.0, 0.03)), cell=CELL_1 + (0.0, 0.0, 0.03))
    assert e.done()[1] is False


def test_cell_knocked_up_without_a_gripper_is_not_taken_out():
    e = Episode()
    e.step(cell=CELL_LIFTED.copy())
    assert e.done()[1] is False


def test_cell_out_and_the_other_left_in_is_three_quarters():
    e = Episode()
    e.unzip()
    e.take_cell()
    e.step(cell=SEATED + (0.05, 0.0, 0.05))
    assert e.done() == [True, True, False, True] and e.step()[0] == 0.6


def test_cell_upside_down_in_the_holder_is_not_in():
    e = Episode()
    e.unzip()
    e.take_cell()
    e.insert_cell(quat=UPSIDE_DOWN)
    assert e.done()[2] is False


def test_cell_resting_on_the_holder_is_not_in():
    e = Episode()
    e.unzip()
    e.take_cell()
    e.insert_cell(at=SEATED + (0.0, 0.0, 0.009))
    assert e.done()[2] is False


def test_cell_still_held_in_the_cup_is_not_in():
    e = Episode()
    e.unzip()
    e.take_cell()
    e.step(cell=SEATED.copy(), right=_holding_cell(SEATED))
    assert e.done()[2] is False


def test_cell_in_any_cup_counts():
    e = Episode()
    e.unzip()
    e.take_cell()
    e.insert_cell(at=T.cup_seat(0))
    assert e.done()[2] is True


def test_full_course_is_all_four_steps():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]


def test_other_cell_counts_only_after_one_is_taken_out():
    e = Episode()
    e.unzip()
    assert e.done()[3] is False


def test_other_cell_knocked_over_is_not_left_in():
    e = Episode()
    e.unzip()
    e.take_cell()
    e.insert_cell()
    e.step(cell_2_quat=np.array([math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)]))
    assert e.done()[3] is False


def test_both_cells_taken_out_is_not_one_left_in():
    e = Episode()
    e.unzip()
    e.take_cell()
    e.insert_cell()
    e.step(cell_2_away=True)
    assert e.done()[3] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.step(1.0) == (1.0, True)


def test_no_full_score_before_one_second_of_stillness():
    """The success conditions first hold on the step the cell is let go in its cup (the last of ``full_course``):
    0.8 until they have held for 1 s, then 1.0."""
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # held 0.95 s
    assert e.step(0.05) == (1.0, True)  # held 1.0 s


def test_cell_moving_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.6) == (0.8, False)
    assert e.step(speed=0.02) == (0.8, False)  # 2 cm/s: not still, the count stops
    assert e.step(speed=0.0) == (0.8, False)  # still again: the second starts over on this step
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # 1.65 s since the conditions first held, but 0.95 s unbroken
    assert e.step(0.05) == (1.0, True)


def test_cell_spinning_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.6) == (0.8, False)
    assert e.step(spin=1.0) == (0.8, False)  # 1 rad/s: not still
    assert e.step(spin=0.0) == (0.8, False)
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step(0.05) == (1.0, True)


def test_arm_leaving_home_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.6) == (0.8, False)
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (0.8, False)
    assert e.step(right=_home("right")) == (0.8, False)
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step(0.05) == (1.0, True)


def test_full_score_drops_when_the_cell_moves_after_success():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S) == (1.0, True)
    assert e.step(speed=0.05) == (0.8, False)


def _course_missing(missing):
    """The full course with one full-success condition left out, from the start or after the course."""
    e = Episode()
    if missing != "bag never unzipped":
        e.unzip()
    if missing == "cell never taken out by a gripper":
        e.insert_cell()
        return e
    e.take_cell()
    if missing == "cell not in a cup":
        e.insert_cell(at=SEATED + (0.05, 0.0, 0.05))
    elif missing == "cell upside down in the cup":
        e.insert_cell(quat=UPSIDE_DOWN)
    else:
        e.insert_cell()
    changes = {
        "other cell taken out too": {"cell_2_away": True},
        "left arm not home": {"left": _gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))},
        "right arm not home": {"right": _gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))},
        "right gripper still holding the seated cell": {"right": _holding_cell(SEATED)},
        "cell and holder moving": {"speed": 0.02},
        "cell turning": {"spin": 1.0},
    }.get(missing, {})
    for k, v in changes.items():
        setattr(e, k, v)
    return e


@pytest.mark.parametrize("missing", ["bag never unzipped", "cell never taken out by a gripper", "cell not in a cup",
                                     "cell upside down in the cup", "other cell taken out too", "left arm not home",
                                     "right arm not home", "right gripper still holding the seated cell",
                                     "cell and holder moving", "cell turning"])
def test_no_full_score_while_any_success_condition_is_missing(missing):
    """With each full-success condition missing in turn, 3 s go by and score() never reaches 1.0."""
    e = _course_missing(missing)
    for _ in range(60):
        got, ok = e.step()
        assert got < 1.0 and not ok, (missing, e.t, got)


def test_run_ends_when_success_first_holds():
    """success() already needs the conditions held for 1 s, so the run is not held a second more."""
    assert TASK.success_hold_s == 0.0


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (0.8, False)  # 4 x 0.2, not success


def test_moving_cell_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (0.8, False)  # 4 x 0.2, not success


def test_cell_off_the_table_scores_zero():
    e = Episode()
    e.unzip()
    e.take_cell()
    assert e.step(cell=np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


def test_holder_off_the_table_scores_zero():
    e = Episode()
    e.unzip()
    assert e.step(holder=np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_arms_end_together():
    d = T.demo()
    assert math.isclose(sum(w.duration for w in d["left"]), sum(w.duration for w in d["right"]), abs_tol=1e-6)
