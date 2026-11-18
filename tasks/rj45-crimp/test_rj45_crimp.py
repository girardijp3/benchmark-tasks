"""Offline checks (no GPU, no network): the task file loads and the four assets exist; the scoring ladder covers
idle, eight wires inserted, plug seated, crimped + strain relief and the 1 s pull-test hold (and its restart); each
cap (wrong wire order, not seated, weak crimp) holds 0.5 and blocks 1.0; the real sizes and the declared-force mapping
are right; the insertion/seat geometry is consistent; every demo waypoint is reachable with both YAM arms and the plan
fits the run limit and returns both arms home.

Run in the yam_arena environment:  python -m pytest -q test_rj45_crimp.py
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
TASK = load(HERE / "rj45_crimp.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(wired=False, seated=False, crimped=False, pulling=False,
             wrong_order=False, not_seated=False, weak_crimp=False):
    return {
        "wires_in": wired, "seated": seated, "crimped": crimped, "pulling": pulling,
        "wrong_order": wrong_order, "not_seated": not_seated, "weak_crimp": weak_crimp,
        "left_gripper": _gripper("left"), "right_gripper": _gripper("right"),
    }


def _full(**kw):
    base = dict(wired=True, seated=True, crimped=True, pulling=True)
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
    for name in ("crimper", "handles", "plug", "cable"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    crimper = (HERE / "assets" / "crimper.usda").read_text()
    for token in ("CrimperBase", "SeatFloor", "SeatRail", "ContactTooth1", "ContactTooth8",
                  "StrainReliefJaw", "RatchetWheel"):
        assert token in crimper
    plug = (HERE / "assets" / "plug.usda").read_text()
    for token in ("PlugBody", "Pin1", "Pin8", "Latch", "WireEntry", "Pin1Index"):
        assert token in plug
    cable = (HERE / "assets" / "cable.usda").read_text()
    for token in ("Jacket", "Wire1", "Wire8", "IndexBand"):
        assert token in cable
    assert "RatchetPawl" in (HERE / "assets" / "handles.usda").read_text()


def test_four_bodies_in_scene():
    assert [p.name for p in T.OBJECTS] == ["crimper", "handles", "cable", "plug"]
    assert T.CRIMPER.fixed is True
    assert T.CABLE.free and T.PLUG.free


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_wires_inserted_quarter():
    o = _objects(wired=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_plug_seated_half():
    o = _objects(wired=True, seated=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_crimped_three_quarters():
    o = _objects(wired=True, seated=True, crimped=True)
    assert T.steps_done(o) == (True, True, True, False)
    assert T.score(o) == pytest.approx(0.75)
    assert T.success(o) is False


def test_pull_step_holds_at_three_quarters_until_the_one_second():
    o = _objects(wired=True, seated=True, crimped=True, pulling=True)
    assert T.steps_done(o) == (True, True, True, True)
    assert T.score(o) == pytest.approx(0.75)   # the 1 s hold, not the step, earns the last point
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
    broken = _objects(wired=True, seated=True, crimped=True)  # pull lost
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["wrong_order", "not_seated", "weak_crimp"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T._full(o) is False
    assert T.score(o) == pytest.approx(0.5)
    assert T.success(o) is False


@pytest.mark.parametrize("capname", ["wrong_order", "not_seated", "weak_crimp"])
def test_a_cap_fails_even_the_crimped_step(capname):
    o = _objects(wired=True, seated=True, crimped=True, **{capname: True})
    assert T.score(o) == pytest.approx(0.5)


def test_pull_requires_crimp_and_order():
    assert T.steps_done(_objects(pulling=True)) == (False, False, False, False)
    assert T.score(_objects(wired=True, seated=True, pulling=True)) == pytest.approx(0.5)


def test_real_product_sizes():
    assert T.PITCH == pytest.approx(0.00102)             # 0.040 in 8P8C pitch
    assert T.PLUG_L == pytest.approx(0.0116)
    assert T.PLUG_W == pytest.approx(0.01168)
    assert T.PLUG_H == pytest.approx(0.0066)
    assert T.CABLE_OD == pytest.approx(0.0062)
    assert T.SEAT_CZ == pytest.approx(0.018)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_force_mapping_is_in_range():
    assert T.OPEN_REF == pytest.approx(0.0939)           # two finger joints, fully open
    # a real closed grip reads 0.024 m (squeeze) or 0.028 m (cable); both are 4-7 N
    assert 4.0 <= T._declared_force(2 * abs(T.HOLD_SQUEEZE)) <= 7.0
    assert 4.0 <= T._declared_force(2 * abs(T.HOLD_CABLE)) <= 7.0
    assert T._declared_force(0.060) < T.CRIMP_FORCE_MIN   # a half-closed grip is a weak crimp
    assert T.CRIMP_FORCE_MIN == pytest.approx(4.0)
    assert T.PULL_MIN == pytest.approx(4.0)
    assert T.PULL_MAX == pytest.approx(7.0)


def test_cap_thresholds_and_tolerances():
    assert T.SQUEEZE_ATTEMPT_OPEN > 2 * abs(T.HOLD_SQUEEZE)   # the real squeeze is inside the attempt zone
    assert T.CRIMP_DWELL == pytest.approx(0.35)
    assert math.degrees(T.ORDER_TOL) == pytest.approx(15.0)
    assert math.degrees(T.SEAT_ROT_TOL) == pytest.approx(12.0)
    assert T.SEAT_XY_TOL == pytest.approx(0.005)
    assert T.PULL_DIST == pytest.approx(0.012)


def test_order_geometry_is_not_180_symmetric():
    # wire 1 must land on pin 1: a half turn is WRONG order, not an equivalent phase
    idx = np.array([0.0, 1.0, 0.0])
    assert T._ang(idx, idx) == pytest.approx(0.0, abs=1e-9)
    assert T._ang(-idx, idx) == pytest.approx(math.pi, abs=1e-9)
    assert T._ang(np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])) == pytest.approx(0.0, abs=1e-9)


def test_insert_and_seat_geometry_is_consistent():
    # wired plug = cable + fixed offset; at the seat, the plug is at the seat centre
    assert np.allclose(T.CABLE_INSERT + T.PLUG_REL,
                       np.array([T.PLUG_STAGE[0], T.PLUG_STAGE[1], T.TOP + T.PLUG_STAGE_CZ]))
    assert np.allclose(T.SEAT_CABLE + T.PLUG_REL,
                       np.array([T.SEAT_XY[0], T.SEAT_XY[1], T.TOP + T.SEAT_CZ]))
    # the handle grip is well inside the right arm's working radius
    d = np.linalg.norm(T.GRIP_ZONE - np.array(BASES["right"]))
    assert d < 0.30


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
    # the right arm returns home; the left arm deliberately ends holding the pull-test so the 1 s
    # success hold is gated by the policy's final hold of that pose
    assert np.allclose(plan["right"][-1].pos, T._START_TCP["right"])
    assert plan["right"][-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
    assert plan["left"][-1].label == "hold the pull-test"
    assert plan["left"][-1].finger == pytest.approx(T.HOLD_CABLE)
    for side, wps in plan.items():
        assert sum(w.duration for w in wps) < 1000 / 20.0 - 2.0
    labels = [w.label for w in plan["left"]]
    assert labels.index("insert the eight wires") < labels.index("seat the plug in the crimper")
    assert labels.index("seat the plug in the crimper") < labels.index("wait for the ratchet to lock")
    assert labels.index("wait for the ratchet to lock") < labels.index("pull-test the crimp")
    rlabels = [w.label for w in plan["right"]]
    assert rlabels.index("squeeze the ratchet") < rlabels.index("right arm home")


def test_step_names_four():
    assert len(T.STEPS) == 4
    assert "wires" in T.STEPS[0] and "seat" in T.STEPS[1]
    assert "crimp" in T.STEPS[2] and "pull" in T.STEPS[3]
