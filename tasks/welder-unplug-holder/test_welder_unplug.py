"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_welder_unplug.py"""

import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "welder_unplug.py")
T = TASK.module

WELDER_QUAT = np.array(T._yaw_xyzw(T.WELDER_YAW))
HOLDER_QUAT = np.array(T._yaw_xyzw(T.HOLDER_YAW))
HOUSING_START = np.array(T.WELDER_POS) + T._rz(T.WELDER_YAW) @ T.BODY_SHIFT
HOUSING_LIFTED = HOUSING_START + (0.0, 0.0, 0.06)
HOUSING_SEATED = T.SEAT + T._rz(T.HOLDER_YAW) @ T.BODY_SHIFT
PLUG_IN = T.PORT_START.copy()
PLUG_OUT = PLUG_IN - (0.0, 0.03, 0.0)
PLUG_LAID = np.array([PLUG_IN[0], PLUG_IN[1] - 0.07, T.TOP + T.OVERMOLD[2] / 2])
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


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _plug(pos, quat=WELDER_QUAT):
    return T.PartPose(pos=np.asarray(pos, float), quat_xyzw=np.array(quat, float))


class Episode:
    def __init__(self):
        self.t = 0.0
        self.housing = T.PartPose(pos=HOUSING_START.copy(), quat_xyzw=WELDER_QUAT.copy())
        self.plug = _plug(PLUG_IN)
        self.left, self.right = _home("left"), _home("right")
        self.cell_shift = np.zeros(3)
        self.speed = 0.0

    def objects(self):
        rh = T.quat_xyzw_to_matrix(self.housing.quat_xyzw)
        return {
            "powerbank": _state(T.BANK_POS, T._yaw_xyzw(T.CABLE_YAW)),
            "welder": _state(self.housing.pos, self.housing.quat_xyzw, self.speed),
            "cell": _state(self.housing.pos + rh @ T.CELL_IN_BODY + self.cell_shift, self.housing.quat_xyzw, self.speed),
            "holder": _state(T.HOLDER_POS, HOLDER_QUAT),
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

    def unplug(self):
        self.step(left=_holding(HOUSING_START + (0.0, 0.058, 0.011)))
        self.step(plug=_plug(PLUG_OUT), right=_holding(PLUG_OUT - (0.0, 0.015, 0.0)))

    def put_plug_down(self):
        self.step(plug=_plug(PLUG_LAID), right=_home("right"))

    def lift_welder(self):
        self.step(housing=T.PartPose(pos=HOUSING_LIFTED, quat_xyzw=WELDER_QUAT.copy()),
                  left=_holding(HOUSING_LIFTED + (0.0, 0.058, 0.011)))

    def store_welder(self, offset=(0.0, 0.0, 0.0), release=True):
        pos = HOUSING_SEATED + np.asarray(offset, float)
        self.step(housing=T.PartPose(pos=pos, quat_xyzw=HOLDER_QUAT.copy()),
                  left=_home("left") if release else _holding(pos + (0.0, 0.0, 0.011)))

    def full_course(self):
        self.unplug()
        self.put_plug_down()
        self.lift_welder()
        self.store_welder()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"powerbank", "welder", "cell", "holder"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_holding_the_welder_down_earns_nothing():
    e = Episode()
    assert e.step(left=_holding(HOUSING_START + (0.0, 0.058, 0.011)))[0] == 0.0


def test_unplugged_is_a_quarter():
    e = Episode()
    e.unplug()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_plug_yanked_out_by_lifting_the_welder_earns_no_plug_steps():
    e = Episode()
    e.lift_welder()  # the welder pulled up and away while nothing holds the plug: the cable yanks it out
    e.step(plug=_plug(PLUG_LAID))
    assert e.done()[:3] == [False, False, True] and e.step()[0] == 0.25


def test_plug_yanked_out_then_picked_up_still_earns_no_plug_steps():
    e = Episode()
    e.lift_welder()
    e.step(plug=_plug(PLUG_LAID))
    e.step(right=_holding(PLUG_LAID - (0.0, 0.015, 0.0)))
    e.step(right=_home("right"))
    assert e.done()[:2] == [False, False]


def test_plug_put_down_is_a_half():
    e = Episode()
    e.unplug()
    e.put_plug_down()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_plug_on_the_table_but_still_held_is_not_put_down():
    e = Episode()
    e.unplug()
    e.step(plug=_plug(PLUG_LAID), right=_holding(PLUG_LAID - (0.0, 0.015, 0.0)))
    assert e.done()[1] is False


def test_plug_held_in_the_air_is_not_put_down():
    e = Episode()
    e.unplug()
    assert e.done()[1] is False


def test_welder_lifted_is_three_quarters():
    e = Episode()
    e.unplug()
    e.put_plug_down()
    e.lift_welder()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.step(1.0) == (1.0, True)


def test_welder_a_little_off_its_seat_still_counts():
    e = Episode()
    e.unplug()
    e.put_plug_down()
    e.lift_welder()
    e.store_welder(offset=(0.003, 0.002, 0.001))
    assert e.step(1.0) == (1.0, True)


@pytest.mark.parametrize("offset", [(0.008, 0.0, 0.0), (0.0, 0.0, 0.004)])
def test_welder_off_its_seat_or_resting_on_a_wall_misses_the_last_step(offset):
    e = Episode()
    e.unplug()
    e.put_plug_down()
    e.lift_welder()
    e.store_welder(offset=offset)
    assert e.done()[3] is False and e.step()[0] == 0.75


def test_welder_still_held_in_its_holder_misses_the_last_step():
    e = Episode()
    e.unplug()
    e.put_plug_down()
    e.lift_welder()
    e.store_welder(release=False)
    assert e.done()[3] is False


def test_welder_in_the_holder_still_plugged_in_misses_the_plug_steps():
    e = Episode()
    e.lift_welder()
    e.store_welder()
    e.step(plug=_plug(HOUSING_SEATED + T._rz(T.HOLDER_YAW) @ T.PORT_IN_BODY, HOLDER_QUAT), right=_home("right"))
    assert e.done()[:2] == [False, False]


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
    e.unplug()
    fallen = T.PartPose(pos=np.array([0.3, 0.4, T.TOP - 0.3]), quat_xyzw=WELDER_QUAT.copy())
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
