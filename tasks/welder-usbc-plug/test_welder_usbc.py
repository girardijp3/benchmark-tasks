"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_welder_usbc.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "welder_usbc.py")
T = TASK.module

YAW = np.array(T._yaw_xyzw(T.WELDER_YAW))
BODY_SHIFT = np.array([0.004, 0.0, 0.0])  # the housing's Body frame starts 4 mm in from the rear face
HOUSING_ON_TABLE = np.array(T.WELDER_POS) + T._rz(T.WELDER_YAW) @ BODY_SHIFT
HOUSING_HELD = HOUSING_ON_TABLE + (0.0, -0.14, 0.08)
PLUG_LYING = np.array([*T.PLUG_START[:2], T.TOP + T.OVERMOLD[2] / 2])
PLUG_QUAT = np.array(T._yaw_xyzw(T.CABLE_YAW))
DOWN = np.array([0.0, 0.0, -1.0])


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.01), contact=10.0, opening=0.01)


def _home(side):
    return _gripper(T._START_TCP[side])


def _plug_in(housing_pos, depth=0.0, off=0.0, tilt_deg=0.0):
    """The plug's pose with its front face ``depth`` short of fully in, ``off`` beside the port's line, tilted."""
    rh = T.quat_xyzw_to_matrix(YAW)
    face = housing_pos + rh @ T.PORT_IN_BODY + rh @ np.array([-depth, off, 0.0])
    a = math.radians(tilt_deg) / 2
    tilt = np.array([0.0, math.sin(a), 0.0, math.cos(a)])  # nose up or down, about the port's own y axis
    return T.PartPose(pos=face, quat_xyzw=np.array(T._qmul(YAW, tilt)))


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.housing = T.PartPose(pos=HOUSING_ON_TABLE.copy(), quat_xyzw=YAW.copy())
        self.plug = T.PartPose(pos=PLUG_LYING.copy(), quat_xyzw=PLUG_QUAT.copy())
        self.left, self.right = _home("left"), _home("right")
        self.cell_shift = np.zeros(3)
        self.speed = 0.0

    def objects(self):
        rh = T.quat_xyzw_to_matrix(self.housing.quat_xyzw)
        cell = self.housing.pos + rh @ T.CELL_IN_BODY + self.cell_shift
        welder_root = self.housing.pos - rh @ BODY_SHIFT
        return {
            "powerbank": _state(T.BANK_POS, T._yaw_xyzw(T.CABLE_YAW)),
            "welder": _state(welder_root, self.housing.quat_xyzw, self.speed),
            "cell": _state(cell, self.housing.quat_xyzw, self.speed),
            "housing": self.housing,
            "plug": self.plug,
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

    def lift_welder(self):
        held = T.PartPose(pos=HOUSING_HELD, quat_xyzw=YAW.copy())
        self.step(housing=held, left=_holding(held.pos + (0.0, 0.05, 0.011)))

    def lift_plug(self):
        up = T.PartPose(pos=PLUG_LYING + (0, 0, 0.05), quat_xyzw=PLUG_QUAT.copy())
        self.step(plug=up, right=_holding(up.pos + (0.0, -0.015, 0.0)))

    def full_course(self, **plug):
        self.lift_welder()
        self.lift_plug()
        self.step(plug=_plug_in(HOUSING_HELD, **plug), right=_home("right"))
        down = T.PartPose(pos=HOUSING_ON_TABLE.copy(), quat_xyzw=YAW.copy())
        self.step(housing=down, plug=_plug_in(HOUSING_ON_TABLE, **plug), left=_home("left"))


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"powerbank", "welder", "cell"}


def test_start_scores_zero():
    assert Episode().step() == (0.0, False)


def test_welder_lifted_is_a_quarter():
    e = Episode()
    e.lift_welder()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_plug_lifted_is_a_quarter():
    e = Episode()
    e.lift_plug()
    assert e.done() == [False, True, False, False] and e.step()[0] == 0.25


def test_plugged_in_while_held_is_three_quarters():
    e = Episode()
    e.lift_welder()
    e.lift_plug()
    e.step(plug=_plug_in(HOUSING_HELD), right=_home("right"))
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.step(1.0) == (1.0, True)


def test_a_sagging_plug_still_counts():
    e = Episode()
    e.full_course(depth=0.0012, off=0.0003, tilt_deg=6.0)
    assert e.step(1.0) == (1.0, True)


@pytest.mark.parametrize("plug", [{"depth": 0.002}, {"off": 0.0008}, {"tilt_deg": 10.0}])
def test_plug_not_in_far_enough_or_straight_enough(plug):
    e = Episode()
    e.full_course(**plug)
    assert e.done()[2:] == [False, False]
    assert e.step() == (0.5, False)


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(left=_gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_welder_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_welder_still_held_misses_the_last_step():
    e = Episode()
    e.full_course()
    e.step(left=_holding(HOUSING_ON_TABLE + (0.0, 0.05, 0.011)))
    assert e.done() == [True, True, True, False]


def test_welder_not_set_down_misses_the_last_step():
    e = Episode()
    e.lift_welder()
    e.lift_plug()
    e.step(plug=_plug_in(HOUSING_HELD), right=_home("right"), left=_home("left"))
    assert e.done()[3] is False


def test_welder_off_the_table_scores_zero():
    e = Episode()
    e.lift_welder()
    fallen = T.PartPose(pos=np.array([0.3, 0.4, T.TOP - 0.3]), quat_xyzw=YAW.copy())
    assert e.step(housing=fallen)[0] == 0.0


def test_cell_out_of_its_bay_scores_zero():
    e = Episode()
    e.full_course()
    assert e.step(cell_shift=np.array([0.0, 0.0, 0.03]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
