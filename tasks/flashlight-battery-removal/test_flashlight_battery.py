"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses
are reachable. Run in the yam_arena environment:  python -m pytest -q test_flashlight_battery.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "flashlight_battery.py")
T = TASK.module

# Centres as the scene starts (from a run): the flashlight head-down at its mark, the box fixed in front of it.
BODY0 = (*T.FLASH_XY, T.TOP + 0.0495)
CAP0 = (*T.FLASH_XY, T.TOP + 0.0965)
BATTERY0 = (*T.FLASH_XY, T.TOP + 0.0605)
HALF = {"flashlight_body": 0.055, "flashlight_cap": 0.0105, "flashlight_battery": 0.033}
HELD_OVER_BOX = (0.44, -0.02, T.TOP + 0.20)  # the body in the left hand, pouring
CAP_LIFTED = (*T.FLASH_XY, T.TOP + 0.14)  # the cap in the right hand, just off the thread
CAP_SET = (*T.CAP_SET_XY, T.TOP + 0.0105)  # the cap set down on its rim beside the right arm
BATTERY_IN_BOX = (T.BOX_XY[0] + 0.07, T.BOX_XY[1], T.TOP + 0.01)  # lying on the box floor
OPEN = T.GripperState(back=np.array([0.2, 0.5, 1.2]), tips=np.array([0.2, 0.5, 1.15]), opening=0.09, contact=0.0)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def _state(pos, vel=(0.0, 0.0, 0.0), half=0.01, turn=0.0):
    """A part at ``pos``, turned ``turn`` degrees about the vertical."""
    pos, a = np.asarray(pos, float), math.radians(turn) / 2
    return ObjectState(pos=pos, quat_xyzw=np.array([0.0, 0.0, math.sin(a), math.cos(a)]),
                       lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3), low=pos - half, high=pos + half)


def _gripping(pos):
    """A gripper closed across ``pos`` from the side, both fingers pressing."""
    pos = np.asarray(pos, float)
    return T.GripperState(back=pos - (0.0, 0.03, 0.0), tips=pos + (0.0, 0.01, 0.0), opening=0.018, contact=10.0)


class Episode:
    """Steps of one episode at 20 Hz: each ``step`` moves parts and sets what each gripper holds."""

    def __init__(self):
        self.t = 0.0
        self.pos = {"flashlight_body": BODY0, "flashlight_cap": CAP0, "flashlight_battery": BATTERY0}
        self.vel = {}
        self.turn = {}  # degrees each part has turned about the vertical

    def objects(self, left=None, right=None):
        out = {n: _state(p, self.vel.get(n, (0.0, 0.0, 0.0)), HALF[n], self.turn.get(n, 0.0)) for n, p in self.pos.items()}
        out["box"] = _state((*T.BOX_XY, T.TOP + 0.05), half=0.05)
        out["left_gripper"] = _gripping(self.pos[left]) if left else OPEN
        out["right_gripper"] = _gripping(self.pos[right]) if right else OPEN
        out["episode_time"] = self.t
        return out

    def step(self, seconds=0.05, left=None, right=None, **moved):
        """Hold for ``seconds``, then report the score; ``moved`` parts move to new centres first."""
        self.pos.update(moved)
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects(left, right)
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def unscrew(self, degrees=400.0):
        """The right gripper turns the cap counter-clockwise on the thread while the left holds the body."""
        for _ in range(int(degrees // 20)):
            self.turn["flashlight_cap"] = self.turn.get("flashlight_cap", 0.0) + 20.0
            self.step(left="flashlight_body", right="flashlight_cap")

    def done(self):
        return T.steps_done(self.objects())


def _demo_until(stage: int) -> Episode:
    """The demo's course up to and including step ``stage`` (1-5), the parts let go at the end."""
    e = Episode()
    e.step(1.5, left="flashlight_body")
    if stage >= 2:
        e.unscrew()
        e.step(left="flashlight_body", right="flashlight_cap", flashlight_cap=CAP_LIFTED)
    if stage >= 3:
        e.step(left="flashlight_body", right="flashlight_cap", flashlight_cap=CAP_SET)
        e.step(left="flashlight_body")
    if stage >= 4:
        e.step(left="flashlight_body", flashlight_body=HELD_OVER_BOX, flashlight_battery=BATTERY_IN_BOX)
    if stage >= 5:
        e.step(left="flashlight_body", flashlight_body=BODY0)
    e.step()
    return e


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"flashlight_body", "flashlight_cap", "flashlight_battery"}


def test_start_scores_zero():
    assert Episode().step() == (0.0, False)


@pytest.mark.parametrize("stage", [1, 2, 3, 4, 5])
def test_each_step_adds_a_fifth(stage):
    e = _demo_until(stage)
    assert e.done() == [True] * stage + [False] * (5 - stage)
    assert e.step() == (pytest.approx(0.2 * stage), stage == 5)


def test_full_course_ends_in_success():
    e = _demo_until(5)
    assert e.step(1.0) == (1.0, True)


def test_body_held_under_a_second_is_not_step_one():
    e = Episode()
    e.step(0.8, left="flashlight_body")
    e.step()
    assert e.done()[0] is False


def test_cap_pulled_off_the_thread_without_unscrewing_scores_zero():
    e = Episode()
    e.step(1.5, left="flashlight_body")
    e.unscrew(100)  # loosened a little, then pulled apart
    assert e.step(left="flashlight_body", right="flashlight_cap", flashlight_cap=CAP_LIFTED)[0] == 0.0
    assert e.step(1.0, left="flashlight_body", flashlight_cap=CAP_SET)[0] == 0.0


def test_gripping_the_cap_of_the_closed_flashlight_is_not_holding_the_body():
    e = Episode()
    e.step(1.5, left="flashlight_cap")  # the body's threaded end is inside the cap, within the body's bounding box
    e.step(1.5, left="flashlight_cap", flashlight_body=(*T.FLASH_XY, T.TOP + 0.10),
           flashlight_cap=(*T.FLASH_XY, T.TOP + 0.147))  # lifted by the cap
    assert e.done()[0] is False


def test_cap_falling_from_the_gripper_is_not_set_down():
    e = _demo_until(2)
    e.step(left="flashlight_body", right="flashlight_cap", flashlight_cap=(*T.CAP_SET_XY, T.TOP + 0.06))
    e.step(left="flashlight_body")  # let go 5 cm up
    e.step(left="flashlight_body", flashlight_cap=CAP_SET)  # it lands on the table
    assert e.done()[2] is False


def test_cap_set_down_then_held_again_does_not_count():
    e = _demo_until(3)
    e.step(left="flashlight_body", right="flashlight_cap")
    assert e.done()[2] is False


def test_cap_set_in_the_box_does_not_count():
    e = _demo_until(2)
    in_box = (T.BOX_XY[0] - 0.05, T.BOX_XY[1], T.TOP + 0.0105)
    e.step(left="flashlight_body", right="flashlight_cap", flashlight_cap=in_box)
    e.step(left="flashlight_body")
    assert e.done()[2] is False


def test_cap_knocked_off_the_table_top_after_set_down_does_not_count():
    e = _demo_until(3)
    e.step(left="flashlight_body", flashlight_cap=(*T.CAP_SET_XY, T.TOP + 0.05))  # on the other arm, say
    assert e.done()[2] is False


def test_battery_held_over_box_is_not_in_it():
    e = _demo_until(3)
    e.step(left="flashlight_body", flashlight_body=HELD_OVER_BOX,
           flashlight_battery=(T.BOX_XY[0], T.BOX_XY[1], T.TOP + 0.12))
    assert e.done()[3] is False


def test_battery_on_the_box_rim_is_not_in_it():
    e = _demo_until(3)
    on_rim = (T.BOX_XY[0], T.BOX_XY[1] + T.BOX_HALF[1] - 0.004, T.TOP + 0.03)
    e.step(left="flashlight_body", flashlight_body=HELD_OVER_BOX, flashlight_battery=on_rim)
    assert e.done()[3] is False


def test_flashlight_dropped_after_pouring_is_not_set_back():
    e = _demo_until(4)
    e.step(flashlight_body=(*T.FLASH_XY, T.TOP + 0.15))  # let go 10 cm up, beside the box
    e.step(flashlight_body=BODY0)
    assert e.done()[4] is False


def test_flashlight_set_down_before_the_battery_is_out_is_not_step_five():
    e = _demo_until(3)
    e.step()  # left lets go of the body standing on its mark
    e.step(flashlight_battery=BATTERY_IN_BOX)
    assert e.done()[3:] == [True, False]


def test_flashlight_lying_on_its_side_counts():
    e = _demo_until(4)
    lying = (0.30, 0.10, T.TOP + 0.017)
    e.step(left="flashlight_body", flashlight_body=lying)
    assert e.step(1.0) == (1.0, True)


def test_a_new_episode_starts_over():
    e = _demo_until(5)
    assert Episode().step() == (0.0, False)


def test_parts_still_moving_is_not_success():
    e = _demo_until(5)
    e.vel["flashlight_battery"] = (0.05, 0.0, 0.0)
    assert e.step(1.0) == (1.0, False)


def test_cap_loosened_but_on_the_flashlight_is_not_off():
    e = Episode()
    e.step(right="flashlight_cap", flashlight_cap=(*T.FLASH_XY, T.TOP + 0.108))
    assert e.done()[1] is False


def test_flashlight_dropped_in_box_scores_zero():
    e = _demo_until(4)
    assert e.step(flashlight_body=(T.BOX_XY[0], T.BOX_XY[1], T.TOP + 0.02)) == (0.0, False)


def test_object_off_table_scores_zero():
    for name in ("flashlight_body", "flashlight_cap", "flashlight_battery"):
        e = _demo_until(3)
        assert e.step(**{name: (0.4, 0.8, T.TOP - 0.3)})[0] == 0.0


def test_non_finite_scores_zero():
    e = _demo_until(3)
    assert e.step(flashlight_battery=(np.nan, 0.0, T.TOP))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    # the left arm's free-air approach is up to ~5 mm off; its grip and pour are within 0.5 mm
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
