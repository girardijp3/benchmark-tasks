"""Offline checks (no GPU, no network): the task file loads, the worn in-service rotor installed on its hub/knuckle with the caliper
bracket, its eight inspection points and marked thinnest spot, the on-rotor test (faces square on the band at the dot), the declared
thread, ratchet, the step ladder, the 1 s full-success timing, the drop latch and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_brake_rotor_micrometer.py"""

import json
import math
import os
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "brake_rotor_micrometer.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
ON = np.array([T.XM, T.YA - 0.0015, T.DOT_Z])  # anvil face on the rotor's face at the marked spot
START = {"mic": (np.array(T.MIC_START, float), IDENT)}
SOLVED = {"mic": (np.array(T.MIC_START, float), IDENT)}  # reading kept, back on its rest
CLAMPED = {"mic": (ON, IDENT)}
OFF_TABLE = (np.array([0.3, 0.0, T.TOP - 0.06]), IDENT)
RULE_CASES = [lambda e: {"mic": OFF_TABLE}]
HOLD_TCP = np.array(T.MIC_START, float) + np.array([-0.043, -0.008, 0.0])
NEAR_TCP = HOLD_TCP + np.array([0.0, 0.0, 0.04])




# -- shared checks ----------------------------------------------------------------------------------------------------
DOWN = np.array([0.0, 0.0, -1.0])
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()
    yield


def _gripper(tcp, contact=0.0, opening=0.09, axis=DOWN):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


class Episode:
    """A made-up episode: the parts at ``START`` (or ``SOLVED``), both grippers at home; ``step`` advances 0.05 s ticks."""

    def __init__(self, poses=None):
        self.t = 0.0
        self.poses = {n: (np.array(p, float), np.array(q, float)) for n, (p, q) in (poses or START).items()}
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {n: _state(p, q) for n, (p, q) in self.poses.items()}
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            else:
                self.poses[k] = (np.array(v[0], float), np.array(v[1], float))
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def done(self):
        return T.steps_done(self.objects())


def test_loads():
    assert TASK.module is T
    assert hasattr(T, "demo") and hasattr(T, "score") and hasattr(T, "success")


def test_start_scores_zero():
    assert Episode().step(1.0) == (0.0, False)


def test_solved_succeeds_after_one_second():
    e = Episode(SOLVED)
    score, ok = e.step(0.5)
    assert score == 0.8 and not ok  # all four steps done, the second not yet up: below 1.0
    score, ok = e.step(1.0)
    assert score == 1.0 and ok


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode(SOLVED)
    for _ in range(19):
        score, ok = e.step()
        assert score < 1.0 and not ok


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S >= 1.0 and T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def test_a_lapse_restarts_the_second():
    e = Episode(SOLVED)
    e.step(0.8)
    name = T.MOVERS[0]
    p, q = e.poses[name]
    e.step(0.1, **{name: (p + np.array([0.004, 0.0, 0.0]), q)})  # moved 4 mm: the count starts over
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok


def test_a_gripper_still_near_a_part_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.3, right=_gripper(NEAR_TCP))
    assert e.done()[-1] is False
    assert e.step(3.0)[0] < 1.0


def test_not_finite_scores_zero():
    e = Episode(SOLVED)
    p, q = e.poses[T.MOVERS[0]]
    assert e.step(0.1, **{T.MOVERS[0]: (np.array([np.nan, 0, 0]), q)})[0] == 0.0


@pytest.mark.parametrize("case", range(len(RULE_CASES)))
def test_a_rule_event_scores_zero_for_the_rest_of_the_run(case):
    e = Episode(SOLVED)
    e.step(0.3)
    e.step(0.1, **RULE_CASES[case](e))
    assert e.step(0.1)[0] == 0.0
    e.step(0.1, **SOLVED)  # put right again
    score, ok = e.step(3.0)
    assert score == 0.0 and not ok  # the event stays


@pytest.mark.parametrize("side", ["left", "right"])
def test_demo_is_reachable(side):
    plan = ArmPlan(T.demo()[side], BASES[side])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    assert max(sum(w.duration for w in d[s]) for s in d) < 150.0
@pytest.fixture(autouse=True)
def fresh_mic():
    T._MIC.__init__()
    yield


class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None, q=0.0, th=0.0, phi=None, **kw):
        super().__init__(poses)
        self.q, self.th, self.phi = q, th, (T.LEV_UNLOCK if phi is None else phi)
        if poses is SOLVED:
            self.q = T.G_OPEN - T.THICK
            T._MIC.ever_gauged, T._MIC.clicks, T._MIC.q_hold = True, 3, self.q

    def objects(self):
        out = super().objects()
        out["q_s"], out["th_t"], out["th_r"], out["phi_l"] = self.q, 0.0, self.th, self.phi
        s = out["mic"]
        out["ratchet"] = ObjectState(pos=s.pos + np.array([0.0, -0.094, 0.0]), quat_xyzw=s.quat_xyzw, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=s.pos, high=s.pos)
        return out


Q_ON = T.G_OPEN - T.THICK  # spindle travel at the rotor's thickness


def _stall():
    T._MIC.t_cmd = T.TURN * (Q_ON + 0.9 * T.LAG) / T.PITCH
    T._MIC.held = True


def test_micrometer_on_its_rest_is_not_gauged():
    e = Episode(START)
    e.step(0.1)
    assert e.done() == [False] * 4 and T.score(e.objects()) == 0.0


def test_faces_on_the_rotor_at_its_thickness_is_step_one():
    e = Episode(CLAMPED, q=Q_ON)
    e.step(0.1)
    assert e.done() == [True, False, False, False]


def test_open_spindle_on_the_rotor_is_not_gauged():
    e = Episode(CLAMPED, q=0.001)
    e.step(0.1)
    assert e.done()[0] is False


def test_micrometer_off_the_band_is_not_gauged():
    p = ON + np.array([0.04, 0.0, 0.0])  # 40 mm further in: inside the friction band
    e = Episode({"mic": (p, IDENT)}, q=Q_ON)
    e.step(0.1)
    assert e.done()[0] is False


def test_micrometer_tilted_off_square_is_not_gauged():
    a = math.radians(8.0) / 2
    q = np.array([0.0, 0.0, math.sin(a), math.cos(a)])
    e = Episode({"mic": (ON, q)}, q=Q_ON)
    e.step(0.1)
    assert e.done()[0] is False


def test_ratchet_slip_counts_clicks_only_while_gauged_and_held():
    e = Episode(CLAMPED, q=Q_ON)
    e.step(0.1)
    _stall()
    for i in range(1, 5):
        e.th = i * T.TOOTH * 1.05
        e.step(0.1)
    assert T._MIC.clicks >= 3


def test_clicks_do_not_count_off_the_rotor():
    e = Episode(START, q=0.0)
    e.step(0.1)
    _stall()
    for i in range(1, 6):
        e.th = i * T.TOOTH * 1.05
        e.step(0.1)
    assert T._MIC.clicks == 0 and e.done()[1] is False


def test_two_clicks_give_step_two():
    e = Episode(CLAMPED, q=Q_ON)
    e.step(0.1)
    _stall()
    for i in range(1, 4):
        e.th = i * T.TOOTH * 1.05
        e.step(0.1)
    assert e.done()[:2] == [True, True]


def test_free_thimble_follows_the_knob_both_ways():
    e = Episode(START)
    e.step(0.1)
    e.th, e.q = 1.0, T.PITCH * 1.0 / T.TURN
    e.step(0.1)
    assert T._MIC.t_cmd == pytest.approx(1.0)
    e.th, e.q = 0.4, T.PITCH * 0.4 / T.TURN
    e.step(0.1)
    assert T._MIC.t_cmd == pytest.approx(0.4)


def _clicked(e):
    e.step(0.1)
    _stall()
    for i in range(1, 4):
        e.th = i * T.TOOTH * 1.05
        e.step(0.1)


OFF_ROTOR = (np.array([T.XM - 0.045, T.YA + 0.035, T.TOP + T.ZA]), IDENT)


def _lifted_off(e, q=Q_ON):
    e.step(0.1)
    e.poses["mic"] = OFF_ROTOR
    e.q = q
    e.step(0.1)


def test_lifted_off_at_the_reading_gives_step_three():
    e = Episode(CLAMPED, q=Q_ON)
    _clicked(e)
    _lifted_off(e)
    assert e.done()[:3] == [True, True, True]


def test_lifted_off_with_the_wrong_reading_does_not_give_step_three():
    e = Episode(CLAMPED, q=Q_ON)
    _clicked(e)
    _lifted_off(e, q=Q_ON - 0.00005)  # 0.05 mm more open than the rotor: outside +-0.02 mm
    assert e.done()[2] is False


def test_the_band_is_two_hundredths_of_a_millimetre():
    assert T.READ_LO == pytest.approx(-0.00002) and T.READ_HI == pytest.approx(0.00002)
    assert T.REFERENCE_TOL_MM == pytest.approx(0.02)


def test_the_rotor_is_a_worn_in_service_part():
    assert T.NEW_MM == pytest.approx(22.00)  # the real part's new thickness
    assert T.DISCARD_MM == pytest.approx(19.00)  # the stamp on the part
    assert T.REFERENCE_MM == pytest.approx(21.50)  # the worn example in the simulator
    assert T.THICK == pytest.approx(0.0215)
    assert T.NEW_MM > T.REFERENCE_MM > T.DISCARD_MM  # worn but still serviceable
    assert T.ROTOR.mass == pytest.approx(5.74)  # estimated worn mass, not the 5.86 kg new part's
    assert T.REFERENCE_REPEAT_MM == pytest.approx(0.005)


def test_the_rotor_is_installed_in_a_wheel_well():
    names = [o.name for o in T.OBJECTS]
    assert {"rotor", "hub_knuckle", "caliper_bracket", "rest", "mic"} <= set(names)
    hub = next(o for o in T.OBJECTS if o.name == "hub_knuckle")
    bracket = next(o for o in T.OBJECTS if o.name == "caliper_bracket")
    assert hub.kinematic and bracket.kinematic and T.ROTOR.kinematic  # fixed installed hardware
    assert "hub_knuckle.usda" in hub.usd_path and "caliper_bracket.usda" in bracket.usd_path
    assert np.allclose(hub.pos, T.ROTOR.pos) and np.allclose(bracket.pos, T.ROTOR.pos)  # same hub-centre origin
    assert bracket.pos[2] == T.TOP + T.ZA > T.TOP + 0.10  # the wheel-well assembly is off the bench top


def test_eight_inspection_points_at_the_stated_radius():
    assert T.INSPECTION_N == 8
    assert T.INSPECTION_R == pytest.approx(0.135)  # 5 mm inboard of the 140 mm friction rim
    radius = math.hypot(T.DOT_X - T.XC, T.DOT_Z - (T.TOP + T.ZA))
    assert radius == pytest.approx(T.INSPECTION_R)  # the marked thinnest point is one of the eight
    assert T.BAND_LO <= T.INSPECTION_R <= T.BAND_HI


def test_a_new_rotor_reading_is_out_of_band():
    new = T.G_OPEN - 22.00 * T.MM
    assert not (T.READ_LO <= new - T.THICK <= T.READ_HI)


def test_the_dot_is_inside_the_scored_band():
    radius = math.hypot(T.DOT_X - T.XC, T.DOT_Z - (T.TOP + T.ZA))
    assert T.BAND_LO <= radius <= T.BAND_HI
    assert T.DOT_TOL == pytest.approx(0.005)


def test_micrometer_off_the_marked_spot_is_not_gauged():
    p = ON + np.array([0.010, 0.0, 0.0])  # 10 mm from the dot, still inside the 100-136 mm band
    assert T.BAND_LO <= math.hypot(p[0] - T.XC, p[2] - (T.TOP + T.ZA)) <= T.BAND_HI
    e = Episode({"mic": (p, IDENT)}, q=Q_ON)
    e.step(0.1)
    assert e.done()[0] is False


def test_the_worn_reference_and_installed_setting_are_documented():
    readme = (HERE / "README.md").read_text()
    task = json.loads((HERE / "task.json").read_text())
    text = readme + "\n" + " ".join(str(v) for v in task.values())
    assert "Brembo 09.7012.11" in text and "280" in text and "22.0" in text and "19" in text
    assert "6061-T6" in text and "EPDM" in text  # the rest's materials
    assert "21.50 mm" in text
    for word in ("knuckle", "caliper bracket", "wheel well", "eight", "median", "five", "0.005"):
        assert word.lower() in text.lower(), word
    assert "TBD" not in readme and "TBD" not in task["scores"]


def test_off_the_rotor_the_thimble_holds_its_reading():
    e = Episode(CLAMPED, q=Q_ON)
    _stall()
    e.step(0.1)
    _lifted_off(e)
    e.th += 3.0
    e.step(0.1)
    assert T._MIC.tgt[0] == pytest.approx(Q_ON, abs=1e-5)


def test_reading_kept_back_on_the_rest_lets_go_gives_one_after_the_second():
    e = Episode(CLAMPED, q=Q_ON)
    _clicked(e)
    _lifted_off(e)
    e.poses["mic"] = (np.array(T.MIC_START, float), IDENT)
    e.step(0.3)
    assert e.done() == [True, True, True, True]
    assert e.step(0.5)[0] == pytest.approx(0.8)
    assert e.step(1.0) == (1.0, True)


def test_a_gripper_holding_it_on_the_rest_is_not_clear():
    e = Episode(CLAMPED, q=Q_ON)
    _clicked(e)
    _lifted_off(e)
    e.poses["mic"] = (np.array(T.MIC_START, float), IDENT)
    e.step(0.3, left=_gripper(HOLD_TCP, contact=15.0, opening=0.012, axis=np.array([1.0, 0.0, 0.0])))
    assert e.done()[3] is False


def test_a_dropped_micrometer_scores_zero_for_the_run():
    e = Episode(CLAMPED, q=Q_ON)
    e.step(0.1)
    s = T.ObjectState if hasattr(T, "ObjectState") else ObjectState
    p = np.array(T.MIC_START, float)
    o = e.objects()
    o["mic"] = ObjectState(pos=p, quat_xyzw=IDENT, lin_vel=np.array([0.0, 0.0, -1.0]), ang_vel=np.zeros(3), low=p, high=p)
    o["episode_time"] = 0.3
    assert T.score(o) == 0.0


def test_the_demo_has_both_arms_and_strokes():
    d = T.demo()
    assert set(d) == {"left", "right"}
    assert any(w.label.startswith("stroke") for w in d["right"])




def test_the_demo_rocks_the_frame_within_its_limits_and_ends_square():
    left = T.demo()["left"]
    rock = [w for w in left if w.label == "rock the frame across the rotor"]
    assert len(rock) > 100
    squared = T.LEFT_ROT
    tilt = [math.degrees(math.acos(min(1.0, (np.trace(w.rot @ squared.T) - 1.0) / 2.0))) for w in rock]
    assert max(tilt) <= 1.5 * T.ROCK_DEG * 1.45 and max(tilt) > 0.5 * T.ROCK_DEG  # the demo rock (1.5 deg: the worn 21.50 mm rotor leaves 1.25 mm frame clearance; the real technique is ~2.5 deg)
    assert tilt[-1] < 0.05  # the rocking dies away to the squared attitude before the frame is eased off
    first = left.index(rock[0])
    assert all(w.label != "rock the frame across the rotor" for w in left[:first]) and left[first - 1].label == "hold it square"


def test_clicks_count_on_the_rotor_while_a_rocking_frame_reads_high():
    e = Episode(CLAMPED, q=Q_ON - 0.0001)  # 0.1 mm open from tilt: outside the band, still against the rotor
    e.step(0.1)
    _stall()
    for i in range(1, 5):
        e.th = i * T.TOOTH * 1.05
        e.step(0.1)
    assert T._MIC.clicks >= 3 and e.done()[0] is False


def test_the_ratchet_keeps_pushing_so_the_reading_settles_at_the_minimum():
    e = Episode(CLAMPED, q=Q_ON - 0.0001)
    e.step(0.1)
    _stall()
    e.th = 0.01
    e.step(0.1)
    assert T._MIC.touched
    q_open = Q_ON - 0.0001
    for q in (q_open + 0.00005, q_open + 0.0001):  # the frame rocks toward square: the faces close
        e.q = q
        e.step(0.05)
    target = T._MIC.tgt[0]
    assert target >= Q_ON - 1e-9 + 0.9 * T.LAG - 1e-4 and e.done()[0] is True
    e.q = Q_ON - 0.0002  # rocked off square again: the spindle is pushed open but the thimble does not back off
    e.step(0.05)
    assert T._MIC.tgt[0] >= target - 1e-12
    assert T._MIC.q_hold == pytest.approx(Q_ON, abs=T.READ_HI)


def test_the_kept_reading_is_the_best_one_when_lifted_off_after_rocking():
    e = Episode(CLAMPED, q=Q_ON)
    _clicked(e)
    e.q = Q_ON - 0.0002  # ends rocked, reading high
    e.step(0.05)
    _lifted_off(e, q=Q_ON)
    assert e.done()[:3] == [True, True, True]
