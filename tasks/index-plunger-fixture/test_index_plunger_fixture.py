"""Offline checks (no GPU, no network): the task file loads and its assets exist; the scoring ladder covers idle,
plunger pulled, plate at the next stop, pin released into the hole, the left-indexed 1 s hold and its restart; each cap
(plate overshoot, plunger misses the hole, forced pull while the pin is engaged) holds the score at 0.5 and blocks 1.0;
the demo fits the run limit, returns home and reaches every waypoint with the YAM arms.

Run in the yam_arena environment:  python -m pytest -q test_index_plunger_fixture.py
"""
import math
import pathlib

import numpy as np
import pytest
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "index_plunger_fixture.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(pulled=False, rotated=False, seated=False, indexed=False,
             overshoot=False, miss_hole=False, forced=False,
             pull=None, plate_yaw=None, grippers=None):
    return {
        "pulled": pulled,
        "rotated": rotated,
        "pin_seated": seated,
        "indexed": indexed,
        "overshoot": overshoot,
        "miss_hole": miss_hole,
        "forced": forced,
        "pull": T.SEAT_TOL / 2.0 if pull is None else pull,
        "plate_yaw": T.PITCH if plate_yaw is None else plate_yaw,
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    kw.setdefault("pulled", True)
    kw.setdefault("rotated", True)
    kw.setdefault("seated", True)
    kw.setdefault("indexed", True)
    return _objects(**kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("base", "plate", "plunger"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    plate = (HERE / "assets" / "plate.usda").read_text()
    for token in ("Disc", "Hub", "IndexHole0", "IndexHole7"):
        assert token in plate
    base = (HERE / "assets" / "base.usda").read_text()
    for token in ("Pedestal", "BearingRing", "PlungerBracket"):
        assert token in base


def test_idle_scores_zero():
    o = _objects(pull=0.0, plate_yaw=0.0)
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_pulled_quarter():
    o = _objects(pulled=True, pull=T.PULL_MAX)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_rotated_half():
    o = _objects(pulled=True, rotated=True, pull=T.PULL_MAX)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_pin_seated_three_quarters():
    o = _objects(pulled=True, rotated=True, seated=True)
    assert T.steps_done(o) == (True, True, True, False)
    assert T.score(o) == pytest.approx(0.75)
    assert T.success(o) is False


def test_full_one_second_hold():
    o = _full()
    assert T.steps_done(o) == (True, True, True, True)
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True
    assert T.score(o) == pytest.approx(1.0)


def test_hold_restarts_after_a_break():
    o = _full()
    for _ in range(10):
        T.success(o)
    broken = _objects(pulled=True, rotated=True, seated=True)  # not indexed yet
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["overshoot", "miss_hole", "forced"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(pulled=True, rotated=True, seated=True, forced=True)
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.CXY[0], T.CXY[1], T.TOP + 0.11]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    assert T.CXY == pytest.approx((0.250, 0.0))
    assert T.PLUNGER_Z == pytest.approx(0.110)
    # a 16 mm class index plunger: 16 mm knob, 7 mm pin, 8 stops at 45 deg, 150 mm plate (REFERENCE.md)
    assert 2 * T.KNOB_R == pytest.approx(0.016)
    assert 2 * T.HOLE_R == pytest.approx(0.007)
    assert T.N_HOLES == 8
    assert math.degrees(T.PITCH) == pytest.approx(45.0)
    assert 2 * T.PLATE_R == pytest.approx(0.150)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_mechanism_numbers():
    assert T.PULL_MAX == pytest.approx(0.016)
    assert T.PULL_RETRACT == pytest.approx(0.012)
    assert math.degrees(T.ROT_TOL) == pytest.approx(8.0)
    assert math.degrees(T.ALIGN_TOL) == pytest.approx(8.0)
    assert math.degrees(T.OVER_TOL) == pytest.approx(12.0)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
    assert T.GRASP_UP == pytest.approx(0.012)


def test_hole_error_zero_at_every_stop():
    for k in range(T.N_HOLES):
        assert abs(T._hole_error(k * T.PITCH)) < 1e-12
    assert abs(T._hole_error(T.PITCH / 2.0)) > 0.3  # between two stops


def test_both_arms_reach_every_demo_waypoint():
    plan = T.demo()
    for side, wps in plan.items():
        for w in wps:
            rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot),
                                 q_init=np.array([0, 1, 1, -0.6, 0, 0]), base_pos=BASES[side], iters=900)
            assert pe < 0.005, (side, w.label, pe, w.pos)
            assert math.degrees(re) < 5.0, (side, w.label, math.degrees(re), w.pos)


def test_demo_fits_limit_and_returns_home():
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for side, wps in plan.items():
        assert np.allclose(wps[-1].pos, T._START_TCP[side])
        assert wps[-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
        assert sum(w.duration for w in wps) < 1000 / 20.0 - 2.0
    def start_time(side, label):
        t = 0.0
        for w in plan[side]:
            if w.label == label:
                return t
            t += w.duration
        raise AssertionError((side, label))

    # the pin is pulled before the plate is turned, and released only after the hub is let go
    assert start_time("left", "pull the plunger out") < start_time("right", "rotate plate one stop")
    assert start_time("right", "release the hub") < start_time("left", "release the plunger")
