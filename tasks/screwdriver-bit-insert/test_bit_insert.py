"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_bit_insert.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "bit_insert.py")
T = TASK.module

DOWN = np.array([0.0, 0.0, -1.0])
SD_QUAT = np.array(T._yaw_xyzw(T.DOCK_YAW))
BLOCK_QUAT = np.array(T._yaw_xyzw(T.BLOCK_YAW))
SEATED_POS = T.MOUTH - np.array([0.0, 0.0, T.SOCKET_DEPTH])
SEATED_QUAT = np.array(T._yaw_xyzw(T.DOCK_YAW + math.pi / 3))  # its hexagon lined up with the socket's
LIFTED = np.array(T.BIT_POS[T.TARGET]) + (0.0, 0.0, 0.04)


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.005), contact=7.0, opening=0.006)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _tilted(quat, axis, deg):
    a = math.radians(deg) / 2
    return np.array(T._qmul(tuple(quat), (*(np.asarray(axis) * math.sin(a)), math.cos(a))))


class Episode:
    def __init__(self):
        self.t = 0.0
        self.sd, self.sd_quat = np.array(T.SD_POS), SD_QUAT.copy()
        self.bits = {n: [np.array(T.BIT_POS[n]), BLOCK_QUAT.copy()] for n in T.BIT_NAMES}
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        out = {
            "dock": _state(T.DOCK_POS, SD_QUAT),
            "screwdriver": _state(self.sd, self.sd_quat, self.speed),
            "bit_block": _state(T.BLOCK_POS, BLOCK_QUAT),
            "left_gripper": self.left,
            "right_gripper": self.right,
            "episode_time": self.t,
        }
        for n, (p, q) in self.bits.items():
            out[n] = _state(p, q, self.speed if n == T.TARGET else 0.0)
        return out

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

    def move_bit(self, name, pos, quat=None, **changes):
        self.bits[name][0] = np.asarray(pos, float).copy()
        if quat is not None:
            self.bits[name][1] = np.asarray(quat, float).copy()
        return self.step(**changes)

    def done(self):
        return T.steps_done(self.objects())

    def take_out(self, name=T.TARGET):
        start = np.array(T.BIT_POS[name])
        self.step(right=_holding(start + (0.0, 0.0, 0.02)))
        self.move_bit(name, start + (0.0, 0.0, 0.04), right=_holding(start + (0.0, 0.0, 0.06)))

    def insert(self, name=T.TARGET, depth=T.SOCKET_DEPTH, quat=SEATED_QUAT, off=(0.0, 0.0)):
        pos = T.MOUTH + np.array([off[0], off[1], -depth])
        self.move_bit(name, pos, quat, right=_holding(pos + (0.0, 0.0, 0.02)))

    def full_course(self):
        self.step(left=_holding(np.array(T.SD_POS) + (0.0, 0.0, 0.08)))
        self.take_out()
        self.insert()
        self.step(right=_home("right"))
        self.step(left=_home("left"))


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"dock", "screwdriver", "bit_block", "bit_ph1", "bit_ph2", "bit_sl5", "bit_t20"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_parts_start_where_the_scorer_expects_them():
    o = Episode().objects()
    assert T.docked(o) and all(T.in_its_hole(o, n) for n in T.BIT_NAMES)
    assert not T.started(o) and T.bit_in_socket(o, T.TARGET).off > 0.1


def test_ph2_taken_out_is_a_quarter():
    e = Episode()
    e.take_out()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_ph2_gripped_in_its_hole_is_not_taken_out():
    e = Episode()
    e.step(right=_holding(np.array(T.BIT_POS[T.TARGET]) + (0.0, 0.0, 0.02)))
    assert e.done()[0] is False


def test_ph2_knocked_up_without_a_gripper_is_not_taken_out():
    e = Episode()
    e.move_bit(T.TARGET, LIFTED)
    assert e.done()[0] is False


def test_another_bit_taken_out_earns_nothing():
    e = Episode()
    e.take_out("bit_ph1")
    assert e.done()[0] is False and e.step()[0] == 0.0


def test_ph2_started_in_the_holder_is_a_step():
    e = Episode()
    e.take_out()
    e.insert(depth=0.004)
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_ph2_resting_on_the_mouth_is_not_started():
    e = Episode()
    e.take_out()
    e.insert(depth=0.0005)
    assert e.done()[1] is False


def test_ph2_seated_while_held_is_three_quarters():
    e = Episode()
    e.take_out()
    e.insert()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_ph2_part_way_in_is_not_seated():
    e = Episode()
    e.take_out()
    e.insert(depth=0.008)
    assert e.done()[1:3] == [True, False]


def test_ph2_tilted_in_the_socket_is_not_seated():
    e = Episode()
    e.take_out()
    e.insert(quat=_tilted(SEATED_QUAT, (1.0, 0.0, 0.0), 3.0))
    assert e.done()[2] is False


def test_ph2_off_the_axis_is_not_seated():
    e = Episode()
    e.take_out()
    e.insert(off=(0.0008, 0.0))
    assert e.done()[2] is False


def test_hexagon_out_of_line_is_not_seated():
    e = Episode()
    e.take_out()
    e.insert(quat=np.array(T._yaw_xyzw(T.DOCK_YAW + math.radians(10))))
    assert e.done()[2] is False


def test_hexagon_lined_up_on_any_flat_is_seated():
    e = Episode()
    e.take_out()
    e.insert(quat=np.array(T._yaw_xyzw(T.DOCK_YAW - math.radians(120))))
    assert e.done()[2] is True


def test_wrong_bit_seated_earns_nothing_for_it():
    e = Episode()
    e.take_out()
    e.move_bit(T.TARGET, np.array(T.BIT_POS[T.TARGET]) + (0.0, 0.06, -T.HOLE_FLOOR), right=_home("right"))
    e.take_out("bit_ph1")
    e.insert("bit_ph1")
    e.step(right=_home("right"))
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.0) == (1.0, True)


def test_still_holding_the_bit_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(right=_holding(SEATED_POS + (0.0, 0.0, 0.02)))
    assert e.done()[3] is False


def test_still_holding_the_screwdriver_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(left=_holding(np.array(T.SD_POS) + (0.0, 0.0, 0.08)))
    assert e.done()[3] is False


def test_screwdriver_lifted_out_of_its_dock_is_not_docked():
    e = Episode()
    e.full_course()
    lift = np.array([0.0, 0.0, 0.03])
    e.step(sd=np.array(T.SD_POS) + lift)
    e.move_bit(T.TARGET, SEATED_POS + lift)
    assert e.done() == [True, True, True, False]


def test_another_bit_knocked_out_of_the_block_fails_the_last_step():
    e = Episode()
    e.full_course()
    e.move_bit("bit_t20", np.array(T.BIT_POS["bit_t20"]) + (0.03, 0.0, -T.HOLE_FLOOR), _tilted(BLOCK_QUAT, (1, 0, 0), 90))
    assert e.done() == [True, True, True, False]


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_screwdriver_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_screwdriver_off_the_table_scores_zero():
    e = Episode()
    e.take_out()
    assert e.step(sd=np.array([0.3, 0.4, T.TOP - 0.3]))[0] == 0.0


def test_bit_off_the_table_scores_zero():
    e = Episode()
    e.take_out()
    assert e.move_bit("bit_sl5", np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
