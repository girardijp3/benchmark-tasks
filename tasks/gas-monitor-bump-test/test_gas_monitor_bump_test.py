"""Offline checks (no GPU, no network): the task file loads, the gas path's finite model (seal, flow, leak, sensor lag),
every scoring outcome and cap, the 1 s full-success timing, and the scripted demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_gas_monitor_bump_test.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "gas_monitor_bump_test.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


START = {"cap": (np.array(T.CAP_REST, float), IDENT), "monitor": (np.array(T.MON_POS, float), IDENT)}
SEATED = {"cap": (np.array(T.CAP_SEAT, float), IDENT), "monitor": (np.array(T.MON_POS, float), IDENT)}
RESTED = {"cap": (np.array(T.CAP_REST, float), IDENT), "monitor": (np.array(T.MON_POS, float), IDENT)}


def gas(**kw):
    d = {"H2S": 0.0, "CO": 0.0, "LEL": 0.0, "O2": 20.9, "seal": False, "on": True, "flow": 0.0,
         "valve": 0.0, "cap_seated": False, "opened": False, "stable": False, "order_error": False,
         "leak_event": False, "vented": False, "safe": True, "open_t": -1.0, "f": 0.0, "s": 0.0}
    d.update(kw)
    return d


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()
    yield


def _home(side):
    return T.GripperState(back=T._START_TCP[side] - T.PAD_BACK * np.array([0.0, 0.0, -1.0]),
                          tips=T._START_TCP[side] + T.PAD_TIP_PAST_TCP * np.array([0.0, 0.0, -1.0]),
                          opening=0.094, contact=0.0)


class Episode:
    """A made-up episode: the cap and monitor at ``poses``, a gas snapshot and both grippers at home; ``step`` advances
    0.05 s ticks. The gas model itself is tested directly below."""

    def __init__(self, poses=None, gas_state=None):
        self.t = 0.0
        self.poses = {n: (np.array(p, float), np.array(q, float)) for n, (p, q) in (poses or START).items()}
        self.gas = gas_state or gas()
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {n: _state(p, q) for n, (p, q) in self.poses.items()}
        cp, mp = self.poses["cap"][0], self.poses["monitor"][0]
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t,
                   gas=self.gas, cap_tabs=(0.0, 0.0), cap_local=tuple(cp - mp), cap_tilt=0.0, cap_yaw=0.0,
                   valve_deg=self.gas["valve"], bars=(0.0, 0.0, 0.0, 0.0))
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            elif k == "gas":
                self.gas = v
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


def test_instruction_is_goal_only_and_15_to_35_words():
    import json
    words = len(json.loads((HERE / "task.json").read_text())["instruction"].split())
    assert 15 <= words <= 35, words


def test_start_scores_zero():
    assert Episode().step(1.0) == (0.0, False)


# -- the gas path: the declared finite model --------------------------------------------------------------------------
def _run(g, seconds, seal, on, valve_deg):
    for _ in range(int(round(seconds / 0.05))):
        g.update(0.05, seal, on, math.radians(valve_deg))
    return g


def test_a_sealed_cap_with_the_valve_open_reaches_the_cylinder_and_stabilises():
    g = _run(T._Gas(), 35.0, True, True, 90.0)
    assert abs(g.readings["H2S"] - 20.0) <= 2.0
    assert abs(g.readings["CO"] - 60.0) <= 6.0
    assert abs(g.readings["LEL"] - 29.0) <= 2.9
    assert abs(g.readings["O2"] - 15.0) <= 0.5
    assert g.stable and g.safe is False


def test_an_unsealed_cap_never_comes_up():
    g = _run(T._Gas(), 35.0, False, True, 90.0)
    assert g.readings["H2S"] < 1.0 and g.readings["CO"] < 3.0
    assert not g.stable
    assert g.leak_event and not g.cap_seated


def test_a_closed_valve_never_comes_up():
    g = _run(T._Gas(), 35.0, True, True, 0.0)
    assert g.readings == pytest.approx(dict(T.AMBIENT))
    assert not g.stable and not g.opened


def test_the_cap_off_the_monitor_does_not_see_gas():
    g = _run(T._Gas(), 35.0, True, False, 90.0)
    assert g.readings == pytest.approx(dict(T.AMBIENT))
    assert not g.stable


def test_the_reading_decays_after_the_gas_is_shut_off_and_the_cap_removed():
    g = _run(T._Gas(), 35.0, True, True, 90.0)
    assert g.stable
    g = _run(g, 25.0, False, False, 0.0)  # gas off, cap off: the sensor comes back to fresh air
    assert g.readings["H2S"] < 1.0 and g.readings["CO"] < 3.0 and g.readings["LEL"] < 1.5
    assert 20.4 <= g.readings["O2"] <= 21.4 and g.safe


def test_gas_opened_before_the_cap_seals_is_an_order_error_and_a_leak():
    g = _run(T._Gas(), 1.0, False, True, 90.0)
    assert g.order_error and g.leak_event and not g.cap_seated


def test_pulling_the_cap_while_the_gas_is_on_is_vented():
    g = _run(T._Gas(), 5.0, True, True, 90.0)
    g.update(0.05, False, True, math.radians(90.0))
    assert g.vented and g.leak_event


# -- the scoring ladder ------------------------------------------------------------------------------------------------
def test_cap_seated_scores_a_quarter():
    e = Episode(SEATED, gas(cap_seated=True, opened=False, stable=False, seal=True, valve=0.0))
    assert e.step(0.2) == (0.25, False)
    assert e.done() == [True, False, False, False]


def test_regulator_opened_scores_half():
    e = Episode(SEATED, gas(cap_seated=True, opened=True, stable=False, seal=True, valve=90.0, flow=0.25 / 60.0))
    assert e.step(0.2) == (0.5, False)


def test_stable_reading_scores_three_quarters():
    e = Episode(SEATED, gas(cap_seated=True, opened=True, stable=True, seal=True, valve=90.0, flow=0.25 / 60.0,
                            H2S=20.0, CO=60.0, LEL=29.0, O2=15.0))
    assert e.step(0.2) == (0.75, False)


def test_the_full_score_needs_the_final_state_to_hold_one_second():
    g = gas(cap_seated=True, opened=True, stable=True, seal=False, valve=-2.0, safe=True,
            H2S=0.5, CO=1.5, LEL=0.7, O2=20.7)
    e = Episode(RESTED, g)
    assert e.step(0.5) == (0.75, False)
    assert e.done() == [True, True, True, True]
    score, ok = e.step(1.0)
    assert score == 1.0 and ok


def test_a_lapse_restarts_the_second():
    g = gas(cap_seated=True, opened=True, stable=True, seal=False, valve=-2.0, safe=True,
            H2S=0.5, CO=1.5, LEL=0.7, O2=20.7)
    e = Episode(RESTED, g)
    e.step(0.8)
    p, q = e.poses["cap"]
    e.step(0.1, cap=(p + np.array([0.004, 0.0, 0.0]), q))
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok


def test_a_valve_left_open_caps_the_score_at_three_quarters():
    g = gas(cap_seated=True, opened=True, stable=True, seal=False, valve=90.0, safe=True,
            H2S=0.5, CO=1.5, LEL=0.7, O2=20.7)
    e = Episode(RESTED, g)
    assert e.step(2.0)[0] == 0.75
    assert e.done()[3] is False


def test_a_reading_that_never_stabilises_caps_the_score_at_half():
    g = gas(cap_seated=True, opened=True, stable=False, seal=True, valve=90.0, flow=0.25 / 60.0,
            H2S=10.0, CO=30.0, LEL=15.0, O2=17.5)
    e = Episode(SEATED, g)
    assert e.step(2.0)[0] == 0.5


def test_an_unsealed_cap_gas_run_caps_the_score_at_a_quarter():
    # the valve was opened before the cap sealed (order error); later it seals but the run is capped
    g = gas(cap_seated=True, opened=True, stable=True, seal=True, valve=90.0, flow=0.25 / 60.0,
            order_error=True, leak_event=True, H2S=20.0, CO=60.0, LEL=29.0, O2=15.0)
    e = Episode(SEATED, g)
    assert e.step(2.0)[0] == 0.25


def test_a_cap_pulled_while_the_gas_was_on_caps_the_score_at_a_quarter():
    g = gas(cap_seated=True, opened=True, stable=True, seal=False, valve=-2.0, safe=True, vented=True, leak_event=True,
            H2S=0.5, CO=1.5, LEL=0.7, O2=20.7)
    e = Episode(RESTED, g)
    assert e.step(2.0)[0] == 0.25


def test_the_cap_still_held_is_not_the_final_state():
    g = gas(cap_seated=True, opened=True, stable=True, seal=False, valve=-2.0, safe=True,
            H2S=0.5, CO=1.5, LEL=0.7, O2=20.7)
    e = Episode(RESTED, g)
    near = T.GripperState(back=np.array(T.CAP_REST) - T.PAD_BACK * np.array([0.0, 0.0, -1.0]),
                          tips=np.array(T.CAP_REST) + T.PAD_TIP_PAST_TCP * np.array([0.0, 0.0, -1.0]),
                          opening=0.02, contact=10.0)
    e.step(0.2, right=near)
    assert e.done()[3] is False
    assert e.step(2.0)[0] < 1.0


@pytest.mark.parametrize("name", ["cap", "monitor"])
def test_a_part_off_the_table_scores_zero_for_the_rest_of_the_run(name):
    e = Episode()
    e.step(0.3)
    p, q = e.poses[name]
    e.step(0.1, **{name: (np.array([p[0], p[1], T.TOP - 0.06]), q)})
    assert e.step(0.1)[0] == 0.0
    e.step(0.1, **{name: (p, q)})
    assert e.step(2.0)[0] == 0.0


def test_not_finite_scores_zero():
    e = Episode()
    p, q = e.poses["cap"]
    assert e.step(0.1, cap=(np.array([np.nan, 0.0, 0.0]), q))[0] == 0.0


def test_the_seal_window_accepts_a_small_nudge_and_rejects_a_large_one():
    seat = (T.SENSOR_X, 0.0, T.MON_H)
    assert T._seal_ok(seat, 0.0, 0.0, (0.0, 0.0))
    assert T._seal_ok((seat[0] + 0.002, seat[1] - 0.002, seat[2]), math.radians(2.0), math.radians(2.0),
                      (math.radians(2.0), -math.radians(2.0)))
    assert not T._seal_ok((seat[0] + 0.004, seat[1], seat[2]), 0.0, 0.0, (0.0, 0.0))
    assert not T._seal_ok(seat, math.radians(4.0), 0.0, (0.0, 0.0))
    assert not T._seal_ok(seat, 0.0, 0.0, (math.radians(4.0), 0.0))


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S >= 1.0
    assert T.POINTS * 3 <= 0.75 + 1e-9


# -- the scripted demo -------------------------------------------------------------------------------------------------
def test_right_demo_is_reachable():
    plan = ArmPlan(T.demo()["right"], BASES["right"])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing_fits_the_run_limit():
    d = T.demo()
    assert max(sum(w.duration for w in d[s]) for s in d) < 120.0
