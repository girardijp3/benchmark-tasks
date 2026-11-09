"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_pack_assembly.py"""

import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "pack_assembly.py")
T = TASK.module

UP = np.array([0.0, 0.0, 0.0, 1.0])
FLIPPED = np.array(T.FLIPPED)
DOWN = np.array([0.0, 0.0, -1.0])
IN_JIG = np.array([*T.JIG_XY, T.FLOOR_Z])
LID_ON = IN_JIG + (0.0, 0.0, T.TOP_ON_CELLS)


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.01), contact=10.0, opening=0.02)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=UP, speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.parts = {"tray": (np.array(T.TRAY_POS), UP), "bottom_holder": (np.array(T.BOTTOM_POS), UP),
                      "top_holder": (np.array(T.LID_POS), FLIPPED)}
        for name, cup in zip(T.CELL_NAMES, T.ORDER, strict=True):
            self.parts[name] = (T.tray_cell(cup), UP)
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        out = {n: _state(p, q, self.speed) for n, (p, q) in self.parts.items()}
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in self.parts:
                self.parts[k] = (np.asarray(v[0], float), np.asarray(v[1], float))
            else:
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

    def place_bottom(self, pos=IN_JIG, quat=UP, holder="bottom_holder"):
        self.step(left=_holding(self.parts[holder][0]))
        self.step(left=_holding(pos), **{holder: (pos, quat)})
        self.step(left=_home("left"))

    def insert(self, k, quat=UP, offset=(0.0, 0.0, 0.0)):
        name, cup = T.CELL_NAMES[k], T.ORDER[k]
        self.step(right=_holding(self.parts[name][0]))
        seat = T.cup_seat(cup) + np.asarray(offset, float)
        self.step(right=_holding(seat), **{name: (seat, quat)})
        self.step(right=_home("right"))

    def put_lid_on(self, pos=LID_ON, quat=FLIPPED, holder="top_holder"):
        self.step(left=_holding(self.parts[holder][0]))
        self.step(left=_holding(pos), **{holder: (pos, quat)})
        self.step(left=_home("left"))

    def full_course(self):
        self.place_bottom()
        for k in range(4):
            self.insert(k)
        self.put_lid_on()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"tray", "bottom_holder", "top_holder", "cell_1", "cell_2", "cell_3", "cell_4"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_bottom_holder_in_the_jig_is_a_quarter():
    e = Episode()
    e.place_bottom()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_holder_on_the_jigs_rim_is_not_in_the_jig():
    e = Episode()
    e.place_bottom(pos=IN_JIG + (0.004, 0.0, 0.003))
    assert e.done()[0] is False


def test_holder_upside_down_in_the_jig_is_not_in_the_jig():
    e = Episode()
    e.place_bottom(pos=IN_JIG + (0.0, 0.0, T.HOLDER_H), quat=FLIPPED)
    assert e.done()[0] is False


def test_either_half_can_be_the_bottom():
    e = Episode()
    e.place_bottom(holder="top_holder")
    for k in range(4):
        e.insert(k)
    e.put_lid_on(holder="bottom_holder")
    assert e.done() == [True, True, True, True]


def test_two_cells_seated_is_a_step():
    e = Episode()
    e.place_bottom()
    e.insert(0)
    assert e.done() == [True, False, False, False]
    e.insert(1)
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_four_cells_seated_is_a_step():
    e = Episode()
    e.place_bottom()
    for k in range(4):
        e.insert(k)
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_cell_upside_down_is_not_seated():
    e = Episode()
    e.place_bottom()
    e.insert(0)
    e.insert(1, quat=np.array(T.FLIPPED))
    assert e.done()[1] is False


def test_cell_leaning_in_its_cup_is_seated():
    e = Episode()
    e.place_bottom()
    e.insert(0)
    lean = np.array([np.sin(np.radians(1.5)), 0.0, 0.0, np.cos(np.radians(1.5))])  # 3 degrees about x
    ledge = T.cup_seat(T.ORDER[1]) - (0.0, 0.0, T.CELL_HALF)
    centre = ledge + T.quat_xyzw_to_matrix(lean) @ np.array([0.0, 0.0, T.CELL_HALF])
    e.step(**{T.CELL_NAMES[1]: (centre, lean)})
    assert e.done()[1] is True


def test_cell_resting_on_the_cups_mouth_is_not_seated():
    e = Episode()
    e.place_bottom()
    e.insert(0)
    e.insert(1, offset=(0.0, 0.0, 0.008))
    assert e.done()[1] is False


def test_cells_in_a_holder_outside_the_jig_earn_nothing():
    e = Episode()
    on_table = np.array([0.30, 0.10, T.TOP])
    e.step(bottom_holder=(on_table, UP))
    for k, cup in enumerate(T.ORDER):
        e.step(**{T.CELL_NAMES[k]: (T.cup_seat(cup, holder_xy=on_table[:2], floor_z=T.TOP), UP)})
    assert e.done() == [False, False, False, False]


def test_top_holder_on_the_cells_is_a_step():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]


def test_top_holder_cups_up_is_not_on():
    e = Episode()
    e.place_bottom()
    for k in range(4):
        e.insert(k)
    e.put_lid_on(pos=LID_ON - (0.0, 0.0, T.HOLDER_H), quat=UP)
    assert e.done()[3] is False


def test_top_holder_resting_on_the_cells_nubs_is_not_on():
    e = Episode()
    e.place_bottom()
    for k in range(4):
        e.insert(k)
    e.put_lid_on(pos=LID_ON + (0.003, 0.0, 0.008))
    assert e.done()[3] is False


def test_top_holder_on_three_cells_is_not_on():
    e = Episode()
    e.place_bottom()
    for k in range(3):
        e.insert(k)
    e.put_lid_on()
    assert e.done() == [True, True, False, False]


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.step(1.0) == (1.0, True)


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_still_holding_the_top_holder_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(left=_holding(LID_ON)) == (1.0, False)


def test_moving_pack_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_cell_off_the_table_scores_zero():
    e = Episode()
    e.place_bottom()
    assert e.step(cell_1=(np.array([0.3, -0.4, T.TOP - 0.3]), UP))[0] == 0.0


def test_holder_off_the_table_scores_zero():
    e = Episode()
    assert e.step(top_holder=(np.array([0.3, 0.4, T.TOP - 0.3]), UP))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
