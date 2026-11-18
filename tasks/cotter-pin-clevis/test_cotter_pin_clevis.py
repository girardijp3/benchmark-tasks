"""Offline checks (no GPU, no network): the task file loads and the assets exist; every scoring rung and cap; the 1 s
hold and its restart; the real 3 mm product sizes; the leg-bend and pull geometry; all demo waypoints reachable with the
YAM arms; the plan fits the run limit and returns home.

Run in the yam_arena environment:  python -m pytest -q test_cotter_pin_clevis.py
"""
import math
import pathlib

import numpy as np
import pytest
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.task_file import load
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "cotter_pin_clevis.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(aligned=False, through=False, spread=False, pull=False, seated=False,
             not_through=False, not_spread=False, dropped=False, grippers=None):
    bend = {leg: (T.SPREAD_RAD if spread else 0.0) for leg in T.LEGS}
    return {
        "aligned": aligned, "through": through, "spread": spread, "pull_ok": pull, "seated": seated,
        "not_through": not_through, "not_spread": not_spread, "dropped": dropped, "bend": bend,
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    base = dict(aligned=True, through=True, spread=True, pull=True, seated=True)
    base.update(kw)
    return _objects(**base)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("clevis", "cotter", "leg_long", "leg_short"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    fixture = (HERE / "assets" / "clevis.usda").read_text()
    for token in ("BenchPlate", "Stand", "UpperProng", "LowerProng", "ClevisPin", "CrossHole",
                  "RodEnd", "PullTabRing", "CotterHolder"):
        assert token in fixture
    body = (HERE / "assets" / "cotter.usda").read_text()
    for token in ("EyeLoop", "ShankLegA", "ShankLegB", "SeatBand"):
        assert token in body
    for name, token in (("leg_long", "TipCap"), ("leg_short", "TipCap")):
        assert token in (HERE / "assets" / f"{name}.usda").read_text()


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_aligned_quarter():
    assert T.steps_done(_objects(aligned=True)) == (True, False, False, False)
    assert T.score(_objects(aligned=True)) == pytest.approx(0.25)


def test_through_half():
    o = _objects(aligned=True, through=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_spread_three_quarters():
    o = _objects(aligned=True, through=True, spread=True)
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
    broken = _objects(aligned=True, through=True, spread=True, seated=True)  # pull not done
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["not_through", "not_spread", "dropped"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    assert T.score(_objects(aligned=True, through=True, spread=True, dropped=True)) == pytest.approx(0.5)


def test_not_seated_keeps_full_false():
    o = _full(seated=False)
    assert T._current_ok(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.PIVOT_XY[0], T.PIVOT_XY[1], T.HOLE_Z]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    assert T._clear(_full(grippers={"left": _gripper("left", opening=0.020)})) is False


def test_real_product_sizes():
    assert 2 * T.WIRE_R == pytest.approx(0.0028)     # 2.8 mm wire for a 3 mm nominal split pin
    assert T.SHANK == pytest.approx(0.012)
    assert T.LONG == pytest.approx(0.036)
    assert T.SHORT == pytest.approx(0.028)
    assert T.PIN_R == pytest.approx(0.005)           # 10 mm clevis pin shank
    assert T.TOP == pytest.approx(0.7475)
    assert T.HOLE_Z == pytest.approx(0.8025)


def test_declared_mechanism_numbers():
    assert math.degrees(T.SPREAD_RAD) == pytest.approx(55.0)
    assert math.degrees(T.MAX_BEND_RAD) == pytest.approx(70.0)
    assert math.degrees(T.AXIS_TOL) == pytest.approx(10.0)
    assert T.XY_TOL == pytest.approx(0.006)
    assert T.Z_TOL == pytest.approx(0.006)
    assert T.PULL_DIST == pytest.approx(0.008)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
    assert T.GRASP_UP == pytest.approx(0.012)


def test_pull_tab_is_beyond_the_leg_tips():
    assert T.PULL_TAB[0] > max(T._straight_tip(leg)[0] for leg in T.LEGS)
    assert T.PULL_TAB[0] > max(T._bent_station(leg)[0] for leg in T.LEGS)


def test_leg_bend_geometry_and_hemisymmetry():
    R = np.eye(3)
    for leg in T.LEGS:
        straight, _ = T._leg_tip(T.HOLE_CENTER, R, leg, 0.0)
        assert np.allclose(straight, T._straight_tip(leg))
        bent, _ = T._leg_tip(T.HOLE_CENTER, R, leg, T.SPREAD_RAD)
        assert np.allclose(bent, T._bent_station(leg))
        # the two legs bend to opposite sides of the cotter axis
        assert np.sign(bent[1] - T.HOLE_CENTER[1]) == T.LEGS[leg][2]
    assert T._bent_station("long")[1] > T.HOLE_CENTER[1]
    assert T._bent_station("short")[1] < T.HOLE_CENTER[1]


def test_bend_follows_a_finger_and_latches():
    R = np.eye(3)
    for leg in T.LEGS:
        tip = T._straight_tip(leg)
        mark = T._bent_station(leg)
        b = T._bend_from_finger(T.HOLE_CENTER, R, leg, 0.0, tip)
        assert b == pytest.approx(0.0, abs=1e-6)
        # sweep the finger from the straight tip to the spread mark; the leg follows and latches
        for i in range(1, 41):
            finger_tip = tip + (mark - tip) * (i / 40.0)
            b = T._bend_from_finger(T.HOLE_CENTER, R, leg, b, finger_tip)
        assert b == pytest.approx(T.SPREAD_RAD, abs=1e-3)
        # a finger pulling away cannot unbend it
        assert T._bend_from_finger(T.HOLE_CENTER, R, leg, b, tip) == pytest.approx(b)


def test_both_arms_reach_every_demo_waypoint():
    plan = T.demo()
    for side, wps in plan.items():
        for w in wps:
            rot = w.rot if w.rot is not None else yam_demo.grip_rotation(w.yaw)
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
        assert sum(w.duration for w in wps) < 900 / 20.0 - 2.0
    labels = [w.label for w in plan["left"]]
    assert labels.index("grip the cotter") < labels.index("align cotter with hole")
    assert labels.index("align cotter with hole") < labels.index("push cotter through the hole")
    assert labels.index("push cotter through the hole") < labels.index("spread the long leg out")
    assert labels.index("spread the short leg out") < labels.index("pull-test the joint")
