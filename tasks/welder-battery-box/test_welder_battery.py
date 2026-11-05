"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_welder_battery.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "welder_battery.py")
T = TASK.module

WELDER_QUAT = np.array(T._yaw_xyzw(T.WELDER_YAW))
R_WELDER = T._rz(T.WELDER_YAW)
ORIGIN_START = np.array(T.WELDER_POS)
ORIGIN_LIFTED = ORIGIN_START + (0.0, 0.0, 0.06)
KNOB = ORIGIN_START + R_WELDER @ T.KNOB_CENTRE
CELL_IN_BAY = ORIGIN_START + R_WELDER @ T.CELL_IN_WELDER
CELL_IN_BOX = np.array([*T.BOX_XY, T.TOP + 0.02])
LID_SHUT, LID_OPENED = 0.0, math.radians(-105)
DOWN = np.array([0.0, 0.0, -1.0])


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.01), contact=10.0, opening=0.01)


def _pushing(point):
    """A gripper, fingers closed, its tips pressed on ``point``."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.01), contact=0.0, opening=0.0)


def _grip_point(origin):
    return np.asarray(origin, float) + R_WELDER @ np.array([T.WELDER_GRIP_AT, 0.0, T.WELDER_HEIGHT / 2])


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.origin, self.quat = ORIGIN_START.copy(), WELDER_QUAT.copy()
        self.cell = CELL_IN_BAY.copy()
        self.lid, self.latch = LID_SHUT, 0.0
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        housing = self.origin + T.quat_xyzw_to_matrix(self.quat) @ T.BODY_SHIFT
        return {
            "welder": _state(housing, self.quat, self.speed),
            "cell": _state(self.cell, self.quat, self.speed),
            "box": _state((*T.BOX_XY, T.TOP)),
            "housing": T.PartPose(pos=housing, quat_xyzw=self.quat),
            "lid_angle": self.lid,
            "latch_travel": self.latch,
            "left_gripper": self.left,
            "right_gripper": self.right,
            "episode_time": self.t,
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

    def hold_down(self):
        self.step(left=_holding(_grip_point(ORIGIN_START)))

    def open_lid(self):
        self.hold_down()
        self.step(right=_pushing(KNOB), latch=0.0015)
        self.step(lid=LID_OPENED, latch=0.0, right=_home("right"))

    def lift_welder(self):
        self.step(origin=ORIGIN_LIFTED, left=_holding(_grip_point(ORIGIN_LIFTED)))

    def pour(self):
        self.step(cell=CELL_IN_BOX.copy())

    def put_back(self, origin=ORIGIN_START, release=True):
        origin = np.asarray(origin, float)
        self.step(origin=origin, left=_home("left") if release else _holding(_grip_point(origin)))

    def full_course(self):
        self.open_lid()
        self.lift_welder()
        self.pour()
        self.put_back()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"welder", "cell"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False]


def test_holding_the_welder_down_earns_nothing():
    e = Episode()
    e.hold_down()
    assert e.step()[0] == 0.0


def test_lid_opened_is_a_third():
    e = Episode()
    e.open_lid()
    assert e.done() == [True, False, False] and e.step()[0] == pytest.approx(1 / 3, abs=1e-6)


def test_latch_pushed_but_lid_still_shut_earns_nothing():
    e = Episode()
    e.hold_down()
    e.step(right=_pushing(KNOB), latch=0.0015)
    assert e.done()[0] is False


def test_lid_open_without_a_gripper_at_the_latch_is_not_opened():
    e = Episode()
    e.step(latch=0.0015)  # the latch moved, but no gripper was at its knob
    e.step(lid=LID_OPENED, latch=0.0)
    assert e.done()[0] is False


def test_cell_in_the_box_is_a_step():
    e = Episode()
    e.open_lid()
    e.lift_welder()
    e.pour()
    assert e.done() == [True, True, False] and e.step()[0] == pytest.approx(2 / 3, abs=1e-6)


@pytest.mark.parametrize("cell", [CELL_IN_BOX + (0.0, 0.0, 0.10), CELL_IN_BOX + (0.15, 0.0, 0.0)])
def test_cell_above_the_box_or_beside_it_is_not_in_it(cell):
    e = Episode()
    e.open_lid()
    e.step(cell=cell)
    assert e.done()[1] is False


def test_welder_never_lifted_is_not_put_back():
    e = Episode()
    e.open_lid()
    e.step(left=_home("left"))
    assert e.done()[2] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True]
    assert e.step(1.0) == (1.0, True)


def test_welder_put_down_elsewhere_on_the_table_still_counts():
    e = Episode()
    e.open_lid()
    e.lift_welder()
    e.pour()
    e.put_back(origin=ORIGIN_START + (0.03, -0.05, 0.0))
    assert e.step(1.0) == (1.0, True)


def test_welder_still_held_is_not_put_back():
    e = Episode()
    e.open_lid()
    e.lift_welder()
    e.pour()
    e.put_back(release=False)
    assert e.done()[2] is False


def test_welder_dropped_into_the_box_is_not_put_back():
    e = Episode()
    e.open_lid()
    e.lift_welder()
    e.pour()
    e.put_back(origin=(*T.BOX_XY, T.TOP + T.FEET))
    assert e.done()[2] is False


def test_welder_on_its_side_is_not_put_back():
    e = Episode()
    e.open_lid()
    e.lift_welder()
    e.pour()
    e.put_back()
    on_side = T._qmul(WELDER_QUAT, (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)))
    assert not e.step(quat=np.array(on_side))[1] and e.done()[2] is False


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_welder_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_welder_off_the_table_scores_zero():
    e = Episode()
    e.open_lid()
    assert e.step(origin=np.array([0.3, 0.4, T.TOP - 0.3]))[0] == 0.0


def test_cell_off_the_table_scores_zero():
    e = Episode()
    e.open_lid()
    assert e.step(cell=np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
