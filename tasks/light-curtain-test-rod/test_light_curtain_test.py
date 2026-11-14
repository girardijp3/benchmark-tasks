"""Offline checks (no GPU, no network): the task file loads, the lamp rule and the scorer cover each outcome, and the
demo's poses are reachable. Run in the yam_arena environment:  python -m pytest -q test_light_curtain_test.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "light_curtain_test.py")
T = TASK.module

UP = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])
GOOD = T.other(T.FAULTY)


def _state(pos, quat=UP):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _gripper(tcp, contact=0.0, opening=0.094):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _q(axis, angle):
    s = math.sin(angle / 2)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.rods = {s: (np.array([*T.ROD_XY[s], T.TOP + T.ROD_AXIS_REST]), UP.copy()) for s in T.SIDES}
        self.tag, self.tag_quat = np.array([*T.RACK_POS, T.TOP + T.FLOOR]), UP.copy()
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {f"rod_{s}": _state(*self.rods[s]) for s in T.SIDES}
        out["tag"] = _state(self.tag, self.tag_quat)
        out["left_gripper"], out["right_gripper"], out["episode_time"] = self.left, self.right, self.t
        for s in T.SIDES:
            for c in ("green", "red"):
                out[f"lamp_{c}_{s}"] = _state(T.lamp_pose(s, c, False))
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

    def rod_at(self, side, x, y, h, quat=UP):
        self.rods[side] = (np.array([x, y, T.TOP + h]), np.asarray(quat, float))

    def push(self, rod_side, curtain, h, seconds=0.2, past=T.TIP_PAST):
        """Rod ``rod_side`` pushed through ``curtain``'s field with its axis ``h`` above the table, then drawn back."""
        y = T.CURTAIN_Y[curtain]
        self.rod_at(rod_side, T.CUR_X + past - T.ROD_L / 2, y, h)
        self.step(seconds)
        self.rod_at(rod_side, T.LANE_X0, y, h)
        self.step(seconds)

    def test(self, side, rod_side=None, heights=T.HEIGHTS):
        for h in heights:
            self.push(rod_side or side, side, h)
        self.rod_at(rod_side or side, *T.ROD_XY[rod_side or side], T.ROD_AXIS_REST)
        self.step()

    def holder(self, side):
        return T.HOLDER_POS[side]

    def tag_in(self, side, **changes):
        h = self.holder(side)
        self.tag, self.tag_quat = np.array([h[0], h[1], T.TOP + T.FLOOR + 0.0005]), UP.copy()
        return self.step(**changes)

    def full_course(self):
        self.test("left")
        self.test("right")
        self.tag_in(T.FAULTY)


def objects_with_rod(side, x, y, h, quat=UP, other_far=True):
    e = Episode()
    e.rod_at(side, x, y, h, quat)
    return e.objects()


def test_loads():
    assert TASK.scene == "station"
    assert {"rod_left", "rod_right", "tag"} <= set(TASK.free_objects)
    assert "physxRigidBody:disableGravity = 1" in (HERE / "assets" / "lamp_red.usda").read_text()
    assert "PhysicsCollisionAPI" not in (HERE / "assets" / "lamp_red.usda").read_text()


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


# -- the lamp (a declared code effect) ---------------------------------------------------------------------------------
def _lamp(side, h, rod="left"):
    o = objects_with_rod(rod, T.CUR_X + 0.08 - T.ROD_L / 2, T.CURTAIN_Y[side], h)
    return T.lamp_red(o, side)


@pytest.mark.parametrize("h", T.HEIGHTS)
def test_healthy_curtain_trips_at_every_height(h):
    assert _lamp(GOOD, h)


def test_clear_field_is_green():
    e = Episode()
    assert not any(T.lamp_red(e.objects(), s) for s in T.SIDES)


def test_rod_in_front_of_the_plane_does_not_trip():
    o = objects_with_rod("left", T.CUR_X - 0.2 - T.ROD_L / 2, T.CURTAIN_Y["left"], 0.117)
    assert not T.lamp_red(o, "left")


def test_dead_beam_is_missed_at_its_height_and_only_there():
    dead_h = T.FIRST + T.PITCH * T.DEAD_BEAM
    assert abs(dead_h - 0.117) < 1e-9
    assert not _lamp(T.FAULTY, dead_h)  # the rod sits on the dead beam only
    assert not _lamp(T.FAULTY, dead_h + 0.008)
    assert not _lamp(T.FAULTY, dead_h - 0.008)
    assert _lamp(T.FAULTY, dead_h + 0.012)  # now it also covers a working beam's whole lens
    assert _lamp(T.FAULTY, dead_h - 0.012)
    assert _lamp(T.FAULTY, T.HEIGHTS[0]) and _lamp(T.FAULTY, T.HEIGHTS[2])  # bottom and top still trip it


def test_the_other_curtain_trips_at_the_dead_beams_height():
    assert _lamp(GOOD, 0.117)


def test_rod_tilted_off_the_beams_trips_by_its_cross_section():
    o = objects_with_rod("left", T.CUR_X + 0.08 - T.ROD_L / 2, T.CURTAIN_Y[GOOD], 0.117, _q((0, 0, 1), 0.2))
    assert T.lamp_red(o, GOOD)


def test_lamp_discs_swap_on_the_receiver_cap():
    for s in T.SIDES:
        assert T.lamp_pose(s, "green", False)[2] == T.LAMP_ON_Z and T.lamp_pose(s, "red", False)[2] == T.LAMP_OFF_Z
        assert T.lamp_pose(s, "red", True)[2] == T.LAMP_ON_Z and T.lamp_pose(s, "green", True)[2] == T.LAMP_OFF_Z


# -- the steps ---------------------------------------------------------------------------------------------------------
def test_each_third_counts_once_and_three_are_needed():
    e = Episode()
    e.push("left", "left", 0.077)
    e.push("left", "left", 0.077)
    assert e.done()[0] is False
    e.push("left", "left", 0.117)
    assert e.done()[0] is False
    e.push("left", "left", 0.165)
    assert e.done() == [True, False, False, False]


def test_either_rod_can_test_either_curtain():
    e = Episode()
    e.test("left", rod_side="right")
    assert e.done()[0] is True


@pytest.mark.parametrize("what", ["short", "tilted", "beside", "low", "high", "behind"])
def test_not_a_pass(what):
    e = Episode()
    y, x = T.CURTAIN_Y["left"], T.CUR_X + T.TIP_PAST - T.ROD_L / 2
    if what == "short":  # the tip stops less than 4 cm beyond the plane
        e.rod_at("left", T.CUR_X + 0.02 - T.ROD_L / 2, y, 0.117)
    elif what == "tilted":
        e.rod_at("left", x, y, 0.117, _q((0, 0, 1), math.radians(25)))
    elif what == "beside":  # past the receiver post
        e.rod_at("left", x, y + 0.19, 0.117)
    elif what == "low":
        e.rod_at("left", x, y, 0.04)
    elif what == "high":
        e.rod_at("left", x, y, 0.22)
    else:  # the rod's middle past the plane (the hand would be in the field)
        e.rod_at("left", T.CUR_X + 0.05, y, 0.117)
    e.step(0.3)
    assert e.done()[0] is False


def test_tag_in_faulty_holder_after_both_tests_is_step_three():
    e = Episode()
    e.test("left")
    e.test("right")
    e.tag_in(T.FAULTY)
    assert e.done() == [True, True, True, True]


def test_tag_in_the_wrong_holder_is_not_step_three():
    e = Episode()
    e.test("left")
    e.test("right")
    e.tag_in(GOOD)
    assert e.done()[2] is False
    assert e.done()[3] is False


def test_tag_in_both_is_not_step_three():
    e = Episode()
    e.test("left")
    e.test("right")
    e.tag_in(T.FAULTY)
    _PROGRESS_SEATED_GOOD = T._PROGRESS.seated_since
    # (one tag cannot be in two holders: put it in the other and the faulty one is empty)
    e.tag_in(GOOD)
    assert e.done()[2] is False
    assert _PROGRESS_SEATED_GOOD is not None


def test_tag_before_the_test_does_not_count_until_it_is_set_again():
    e = Episode()
    e.tag_in(T.FAULTY)
    e.step(0.5)
    e.test("left")
    e.test("right")
    assert e.done()[2] is False
    e.tag = np.array([*T.RACK_POS, T.TOP + T.FLOOR])
    e.step(0.2)
    e.tag_in(T.FAULTY)
    assert e.done()[2] is True


def test_tag_without_testing_the_faulty_curtain_is_not_step_three():
    e = Episode()
    e.test(GOOD)
    e.tag_in(T.FAULTY)
    assert e.done()[2] is False and e.done()[3] is False


def test_tag_resting_tilted_or_off_centre_in_the_holder_is_not_seated():
    h = T.HOLDER_POS[T.FAULTY]
    for pos, quat in [((h[0], h[1], T.TOP + T.FLOOR + 0.02), UP), ((h[0], h[1] + 0.008, T.TOP + T.FLOOR), UP),
                      ((h[0] + 0.03, h[1], T.TOP + T.FLOOR), UP), ((h[0], h[1], T.TOP + T.FLOOR), _q((1, 0, 0), 0.3))]:
        e = Episode()
        e.test("left")
        e.test("right")
        e.tag, e.tag_quat = np.array(pos), quat
        e.step()
        assert e.done()[2] is False


def test_step_four_needs_the_rods_out_of_the_fields():
    e = Episode()
    e.full_course()
    assert e.done()[3] is True
    e.rod_at("left", T.CUR_X + T.TIP_PAST - T.ROD_L / 2, T.CURTAIN_Y["left"], 0.117)
    assert e.done()[3] is False and e.done()[2] is True


def test_step_four_needs_the_grippers_clear():
    e = Episode()
    e.full_course()
    h = T.HOLDER_POS[T.FAULTY]
    near_tag = _gripper([h[0], h[1], T.TOP + 0.05], contact=10.0)
    e.step(right=near_tag)
    assert e.done()[3] is False
    e.step(right=_home("right"), left=_gripper([*T.ROD_XY["left"], T.TOP + 0.03]))
    assert e.done()[3] is False


# -- full score ---------------------------------------------------------------------------------------------------------
def test_full_course_succeeds_after_one_second_still():
    e = Episode()
    e.full_course()
    score, ok = e.step(0.5)
    assert ok is False and score <= 0.8
    score, ok = e.step(0.45)  # 0.95 s from the first still moment: not yet
    assert ok is False and score <= 0.8
    score, ok = e.step(0.2)
    assert (score, ok) == (1.0, True)


def test_step_points_stop_short_of_one():
    assert T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


@pytest.mark.parametrize("missing", ["left test", "right test", "tag", "release"])
def test_no_full_score_while_a_condition_is_missing(missing):
    e = Episode()
    if missing != "left test":
        e.test("left")
    if missing != "right test":
        e.test("right")
    if missing != "tag":
        e.tag_in(T.FAULTY)
    if missing == "release":
        h = T.HOLDER_POS[T.FAULTY]
        e.right = _gripper([h[0], h[1], T.TOP + 0.05], contact=10.0)
    for _ in range(6):
        score, ok = e.step(0.5)
        assert ok is False and score <= 0.8


def test_moving_parts_restart_the_second():
    e = Episode()
    e.full_course()
    e.step(0.6)
    e.tag = e.tag + (0.005, 0, 0)
    e.step(0.1)
    score, ok = e.step(0.6)
    assert ok is False
    score, ok = e.step(0.6)
    assert (score, ok) == (1.0, True)


def test_a_rod_bumped_after_the_tag_restarts_the_second():
    e = Episode()
    e.full_course()
    e.step(0.6)
    e.rods["left"] = (e.rods["left"][0] + (0, 0.004, 0), e.rods["left"][1])
    score, ok = e.step(0.5)
    assert ok is False


@pytest.mark.parametrize("part", ["rod_left", "rod_right", "tag"])
def test_part_off_the_table_scores_zero(part):
    e = Episode()
    e.full_course()
    o = e.objects()
    o[part] = _state(o[part].pos - (0, 0, 0.8))
    assert T.score(o) == 0.0


def test_a_swapped_fault_flips_the_right_answer(monkeypatch):
    monkeypatch.setattr(T, "FAULTY", T.other(T.FAULTY))
    e = Episode()
    e.test("left")
    e.test("right")
    e.tag_in(T.other(T.FAULTY))  # the tag in what was the right answer
    assert e.done()[2] is False


def test_no_file_is_too_big_for_the_review_archive():
    for f in HERE.rglob("*"):
        if f.is_file():
            assert f.stat().st_size <= 1_870_000, f


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
