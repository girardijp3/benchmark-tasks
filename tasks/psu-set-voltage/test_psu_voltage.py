"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, the switch latches and the
displays read right, and the demo's poses are reachable. Run in the yam_arena environment:
python -m pytest -q test_psu_voltage.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "psu_voltage.py")
T = TASK.module

FORWARD = np.array([1.0, 0.0, 0.0])  # a gripper pointing at the panel
START_QUAT = np.array(T._yaw_xyzw(T.PSU_YAW))
PER_VOLT = 1.0 / T.VOLTS_PER_RAD  # rad of the coarse knob per volt


def _gripper(tcp, contact=0.0, opening=0.09, axis=FORWARD):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side], axis=np.array([0.0, 0.0, -1.0]))


def _on_knob():
    """A gripper closed around the coarse voltage knob."""
    skirt = T.psu_point(T.KNOB_AT["Knob_V_Coarse"] - np.array([0.0, T.SKIRT_FRONT + T.TIPS_SHORT, 0.0]))
    return _gripper(skirt - T.PAD_TIP_PAST_TCP * FORWARD, contact=20.0, opening=0.018)


def _on_button(travel):
    """Closed fingertips on the button's face, pushed in ``travel``."""
    face = T.psu_point(T.BUTTON_AT + np.array([0.0, travel - T.BUTTON_OUT, 0.0]))
    return _gripper(face - T.PAD_TIP_PAST_TCP * FORWARD, contact=0.0, opening=0.0)


def _state(pos, quat, speed=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos, self.quat, self.speed = np.array(T.PSU_POS), START_QUAT.copy(), 0.0
        self.travel, self.power = 0.0, False
        self.knobs = dict.fromkeys(T.JOINTS[1:], 0.0)
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {"psu": _state(self.pos, self.quat, self.speed), "button_travel": self.travel, "power_on": self.power,
               "left_gripper": self.left, "right_gripper": self.right, "episode_time": self.t}
        out.update(self.knobs)
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

    def done(self):
        return T.steps_done(self.objects())

    def press_power(self):
        self.step(left=_on_button(0.0))
        self.step(left=_on_button(0.004), travel=0.004, power=True)
        self.step(left=_home("left"), travel=T.HELD_IN)

    def set_volts(self, v, fine=0.0):
        self.step(right=_on_knob())
        self.knobs["Knob_V_Coarse"] = (v - T.VOLTS_START) * PER_VOLT
        self.knobs["Knob_V_Fine"] = fine
        self.step()
        self.step(right=_home("right"))

    def full_course(self):
        self.press_power()
        self.set_volts(T.VOLTS_TARGET)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"psu"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_start_reads_12_volts():
    assert T.volts(Episode().objects()) == pytest.approx(12.0)


def test_turn_for_9_volts_is_27_degrees_counter_clockwise():
    assert math.degrees(T.TURN) == pytest.approx(-27.0)


def test_power_on_by_a_gripper_is_a_quarter():
    e = Episode()
    e.press_power()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.25


def test_power_on_with_no_gripper_at_the_button_does_not_count():
    e = Episode()
    e.step(travel=0.004, power=True)
    e.step(travel=T.HELD_IN)
    assert e.done()[0] is False


def test_power_switched_off_again_does_not_count():
    e = Episode()
    e.press_power()
    e.step(left=_on_button(0.004), travel=0.004, power=False)
    e.step(left=_home("left"), travel=0.0)
    assert e.done()[0] is False


def test_button_not_held_in_does_not_count():
    e = Episode()
    e.press_power()
    e.step(travel=0.0)
    assert e.done()[0] is False


def test_knob_gripped_is_a_step_once_reached():
    e = Episode()
    e.step(right=_on_knob())
    assert e.done()[1] is True
    e.step(right=_home("right"))
    assert e.done()[1] is True


def test_open_fingers_around_the_knob_are_not_a_grip():
    e = Episode()
    e.step(right=_gripper(_on_knob().back + T.PAD_BACK * FORWARD, contact=0.0, opening=0.04))
    assert e.done()[1] is False


@pytest.mark.parametrize("v, ok", [(9.0, True), (8.85, True), (9.15, True), (8.7, False), (9.3, False), (12.0, False)])
def test_set_to_9_volts(v, ok):
    e = Episode()
    e.set_volts(v)
    assert e.done()[2] is ok


def test_knob_at_9_volts_still_held_is_not_set():
    e = Episode()
    e.step(right=_on_knob())
    e.knobs["Knob_V_Coarse"] = T.TURN
    assert e.done()[2] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.0) == (1.0, True)


def test_9_volts_with_power_off_is_three_quarters_at_most():
    e = Episode()
    e.set_volts(9.0)
    assert e.done() == [False, True, True, False] and e.step()[0] == 0.5


@pytest.mark.parametrize("knob", T.OTHER_KNOBS)
def test_another_knob_turned_spoils_the_last_step(knob):
    e = Episode()
    e.full_course()
    e.knobs[knob] = math.radians(10)
    assert e.done()[3] is False


def test_fine_knob_counts_in_the_voltage():
    e = Episode()
    e.press_power()
    e.step(right=_on_knob())
    e.knobs["Knob_V_Coarse"] = (9.4 - T.VOLTS_START) * PER_VOLT
    e.knobs["Knob_V_Fine"] = -0.4 / T.FINE_VOLTS_PER_RAD
    e.step(right=_home("right"))
    assert T.volts(e.objects()) == pytest.approx(9.0)
    assert e.done() == [True, True, True, False]


def test_supply_pushed_away_spoils_the_last_step():
    e = Episode()
    e.full_course()
    e.step(pos=np.array(T.PSU_POS) + (0.02, 0.0, 0.0))
    assert e.done() == [True, True, True, False]


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (1.0, False)


def test_moving_supply_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (1.0, False)


def test_supply_knocked_over_scores_zero():
    e = Episode()
    e.full_course()
    on_side = np.array([math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)])
    assert e.step(quat=on_side)[0] == 0.0


def test_supply_off_the_table_scores_zero():
    e = Episode()
    e.press_power()
    assert e.step(pos=np.array([0.5, 0.0, T.TOP - 0.3]))[0] == 0.0


def test_switch_is_push_on_push_off():
    s = T._Switch()
    assert not s.on and s.target == T.OUT
    assert s.update(0.004) and s.on and s.target == T.HELD_IN
    assert not s.update(0.0042)  # held down: no second switch
    assert not s.update(T.HELD_IN) and s.on  # comes back to the latch
    assert s.update(0.004) and not s.on and s.target == T.OUT
    assert not s.update(0.002) and not s.on
    assert not s.update(0.0034)  # not past the click


@pytest.mark.parametrize("on, v, shown", [(True, 9.0, " 900"), (True, 12.0, "1200"), (True, 0.5, " 050"),
                                          (True, 8.996, " 900"), (False, 9.0, "    ")])
def test_voltmeter(on, v, shown):
    assert T.display_text(on, v)["V"] == shown
    assert T.display_text(on, v)["A"] == ("0000" if on else "    ")


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
