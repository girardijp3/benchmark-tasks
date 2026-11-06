"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_servo_mount.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "servo_mount.py")
T = TASK.module

DOWN = np.array([0.0, 0.0, -1.0])
BRACKET_QUAT = np.array(T._yaw_xyzw(T.BRACKET_YAW))
SERVO_QUAT = np.array(T._matrix_xyzw(T.SERVO_ROT0))
SEATED_QUAT = np.array(T._matrix_xyzw(T.SEAT_ROT))
BACKWARDS_QUAT = np.array(T._matrix_xyzw(T._rz(math.pi) @ T.SEAT_ROT))  # horn toward the robot's right
SERVO_LIFTED = np.array(T.SERVO_POS0) + (0.0, 0.0, 0.05)


def _gripper(tcp, contact=0.0, opening=0.09, closing=(1.0, 0.0, 0.0)):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact, closing=np.asarray(closing, float))


def _holding(middle, width, closing):
    """A gripper pointing down whose pads' middle is at ``middle``, closed on a part ``width`` across along
    ``closing`` (pressing)."""
    tcp = np.asarray(middle, float) + (T.PAD_BACK - T.PAD_TIP_PAST_TCP) / 2 * DOWN
    return _gripper(tcp, contact=10.0, opening=width, closing=closing)


def _on_servo(centre, rot):
    """The right hand on the servo at ``centre``/``rot``: across its sides, pads' middle near its top as in the demo."""
    r = np.asarray(rot)
    return _holding(np.asarray(centre, float) + r[:, 0] * 0.0216, T.SERVO_WIDTH, r[:, 1])


def _on_ring(bracket_pos):
    """The left hand across the bracket's motor-holder ring."""
    middle = np.asarray(bracket_pos, float) + T.BRACKET_ROT @ np.array([0.006, sum(T.RING_Y) / 2, sum(T.RING_Z) / 2])
    return _holding(middle, T.RING_WIDTH, T.BRACKET_ROT[:, 1])


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.bracket, self.bracket_quat = T.BRACKET_POS0.copy(), BRACKET_QUAT.copy()
        self.servo, self.quat = np.array(T.SERVO_POS0), SERVO_QUAT.copy()
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0

    def objects(self):
        return {
            "bracket": _state(self.bracket, self.bracket_quat, self.speed),
            "servo": _state(self.servo, self.quat, self.speed),
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

    def seat(self, height=0.0):
        """The seated servo's centre ``height`` above its seat, wherever the bracket is now."""
        r = T.quat_xyzw_to_matrix(self.bracket_quat)
        return self.bracket + r @ T.SEAT_POS + (0.0, 0.0, height)

    def lift_bracket(self):
        self.step(left=_on_ring(T.BRACKET_POS0))
        self.step(bracket=T.HOLD_POS.copy(), left=_on_ring(T.HOLD_POS))

    def pick_servo(self):
        self.step(right=_on_servo(T.SERVO_POS0, T.SERVO_ROT0))
        self.step(servo=SERVO_LIFTED.copy(), right=_on_servo(SERVO_LIFTED, T.SERVO_ROT0))

    def insert(self, height=0.0, quat=SEATED_QUAT, side=(0.0, 0.0)):
        pos = self.seat(height) + (side[0], side[1], 0.0)
        self.step(servo=pos, quat=np.asarray(quat, float), right=_on_servo(pos, T.quat_xyzw_to_matrix(quat)))

    def right_lets_go(self):
        self.step(right=_home("right"))

    def set_down(self):
        held_servo = self.servo - self.bracket
        self.bracket = T.BRACKET_POS0.copy()
        self.step(servo=self.bracket + held_servo, left=_on_ring(self.bracket))
        self.step(left=_home("left"))

    def full_course(self):
        self.lift_bracket()
        self.pick_servo()
        self.insert(0.010)
        self.insert(0.0)
        self.right_lets_go()
        self.set_down()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"bracket", "servo"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_bracket_picked_up_is_a_quarter():
    e = Episode()
    e.lift_bracket()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_bracket_gripped_on_the_table_is_not_picked_up():
    e = Episode()
    e.step(left=_on_ring(T.BRACKET_POS0))
    assert e.done()[0] is False


def test_bracket_knocked_up_without_a_gripper_is_not_picked_up():
    e = Episode()
    e.step(bracket=T.HOLD_POS.copy())
    assert e.done()[0] is False


def test_servo_partly_in_the_held_bracket_is_a_step():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(0.010)
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_servo_above_the_rim_is_not_in_the_pocket():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(T.POCKET_DEPTH + 0.002)
    assert e.done()[1] is False


def test_servo_just_inside_the_rim_is_not_yet_in_the_pocket():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(T.POCKET_DEPTH - 0.003)
    assert e.done()[1] is False


def test_pressed_home_is_three_steps():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(0.0)
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.0) == (1.0, True)


def test_still_holding_the_bracket_is_not_set_down():
    e = Episode()
    e.full_course()
    e.step(left=_on_ring(e.bracket))
    assert e.done()[3] is False


def test_bracket_left_in_the_air_is_not_set_down():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(0.0)
    e.right_lets_go()
    e.step(left=_home("left"))
    assert e.done()[3] is False


def test_bracket_on_its_side_is_not_set_down():
    e = Episode()
    e.full_course()
    on_side = np.array(T._matrix_xyzw(T.BRACKET_ROT @ T.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.pi / 2)))
    e.step(bracket=T.BRACKET_POS0 + (0.0, 0.0, 0.025), bracket_quat=on_side)
    assert e.done()[3] is False


def test_servo_mounted_with_the_bracket_left_on_the_table_misses_only_step_1():
    e = Episode()
    e.pick_servo()
    e.insert(0.0)
    e.right_lets_go()
    assert e.done() == [False, True, True, True] and e.step(1.0) == (0.75, False)


def test_horn_the_wrong_way_is_not_in_the_pocket():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(0.0, quat=BACKWARDS_QUAT)
    assert e.done() == [True, False, False, False]


def test_upside_down_is_not_in_the_pocket():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    flipped = np.array(T._matrix_xyzw(T.SEAT_ROT @ T.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.pi)))
    e.insert(0.0, quat=flipped)
    assert e.done()[1] is False


def test_tilted_is_not_seated():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    tilted = np.array(T._matrix_xyzw(T.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(3)) @ T.SEAT_ROT))
    e.insert(0.0, quat=tilted)
    assert e.done()[1:3] == [True, False]


def test_off_the_pocket_line_is_not_in_the_pocket():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(0.0, side=(0.003, 0.0))
    assert e.done()[1] is False


def test_not_pressed_fully_home_is_not_seated():
    e = Episode()
    e.lift_bracket()
    e.pick_servo()
    e.insert(0.002)
    assert e.done()[1:3] == [True, False]


def test_servo_in_without_a_gripper_doing_it_earns_nothing():
    e = Episode()
    e.lift_bracket()
    e.step(servo=e.seat(0.0), quat=SEATED_QUAT.copy())
    assert e.done() == [True, False, False, False]


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_left_arm_away_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(left=_gripper(T._START_TCP["left"] + (0.1, 0.0, 0.0))) == (1.0, False)


def test_moving_parts_are_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_servo_off_the_table_scores_zero():
    e = Episode()
    e.pick_servo()
    assert e.step(servo=np.array([0.3, -0.5, T.TOP - 0.3]))[0] == 0.0


def test_bracket_off_the_table_scores_zero():
    e = Episode()
    e.lift_bracket()
    assert e.step(bracket=np.array([0.3, 0.5, T.TOP - 0.3]))[0] == 0.0


def test_demo_grips_are_where_the_parts_are():
    tcp, rot = T._right_on_servo((*T.SERVO_XY, T.TOP + T.SERVO_HALF), T.SERVO_ROT0)
    assert np.allclose(rot[:, 0], T.SERVO_ROT0[:, 1])  # the fingers close across the servo's sides
    tips = tcp + T.PAD_TIP_PAST_TCP * rot[:, 2]
    low = tips[2] - T.PAD_HALF_WIDTH * abs(rot[2, 1])
    assert abs(low - (T.TOP + T.GRIP_ABOVE_REAR)) < 1e-4  # the pads' lowest corner, 5 mm over the seated rim
    tcp, rot = T._left_on_bracket(T.BRACKET_POS0)
    assert abs(abs(rot[:, 0] @ (T.BRACKET_ROT @ np.array([0.0, 1.0, 0.0]))) - 1.0) < 1e-9  # across the ring


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
