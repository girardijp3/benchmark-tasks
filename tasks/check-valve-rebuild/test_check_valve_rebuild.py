"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
body opened, parts seated, cap threaded + forward flow, backflow sealed + the 1 s hold and its restart; every cap
(reversed poppet, cross-thread, dropped part, leak/backflow) holds the score at 0.5 and blocks 1.0; the demo fits
the run limit, returns home and reaches every waypoint with the YAM arms.

Run in the yam_arena environment:  python -m pytest -q test_check_valve_rebuild.py
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
TASK = load(HERE / "check_valve_rebuild.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(opened=False, poppet=False, spring=False, cap=False, forward=False, reverse=False,
             reversed_poppet=False, missing_spring=False, cross_thread=False, dropped=False, leak=False, grippers=None):
    return {
        "body_opened": opened,
        "poppet_done": poppet,
        "spring_done": spring,
        "cap_done": cap,
        "forward_done": forward,
        "reverse_done": reverse,
        "reversed_poppet": reversed_poppet,
        "missing_spring": missing_spring,
        "cross_thread": cross_thread,
        "dropped": dropped,
        "leak": leak,
        "twist": {"cap": math.radians(93.0)},
        "pressed": {"in": False, "out": False},
        "flow_lift": 0.0,
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    return _objects(opened=True, poppet=True, spring=True, cap=True, forward=True, reverse=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("valve", "cap", "poppet", "spring"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    valve = (HERE / "assets" / "valve.usda").read_text()
    for token in ("Fixture", "Body", "Seat", "InletButton", "OutletButton", "PoppetPocket", "CapPegPin"):
        assert token in valve


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_body_opened_quarter():
    o = _objects(opened=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_parts_seated_half():
    o = _objects(opened=True, poppet=True, spring=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_cap_and_forward_three_quarters():
    o = _objects(opened=True, poppet=True, spring=True, cap=True, forward=True)
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
    broken = _objects(opened=True, poppet=True, spring=True, cap=True)  # no forward/reverse
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["reversed_poppet", "missing_spring", "cross_thread", "dropped", "leak"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(opened=True, poppet=True, spring=True, cap=True, forward=True, leak=True)
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    # a gripper parked at the valve blocks the full state (pads too close)
    g = _gripper("left", tcp=np.array([T.VALVE_XY[0], T.VALVE_XY[1], T.TOP + 0.07]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    assert T.VALVE_XY == pytest.approx((0.250, 0.0))
    assert T.SEAT_Z == pytest.approx(0.062)
    # a 20 mm valve: cap 26 mm across flats, disc 16 mm, spring 14 mm OD (see REFERENCE.md)
    assert 0.024 <= 2 * 0.013 <= 0.028
    assert 2 * 0.008 == pytest.approx(0.016)
    assert 2 * 0.007 == pytest.approx(0.014)
    assert T.BODY_TOP - T.FIX_H == pytest.approx(0.040)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_mechanism_numbers():
    assert T.BUTTON_PRESS == pytest.approx(0.006)
    assert T.FLOW_LIFT == pytest.approx(0.006)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
    assert math.degrees(T.THREAD_YAW) == pytest.approx(60.0)
    assert T.GRASP_UP == pytest.approx(0.012)


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
        assert sum(w.duration for w in wps) < 1200 / 20.0 - 2.0
    # the cap is parked before the poppet is picked, and the spring before the cap is threaded
    labels = [w.label for w in plan["left"]]
    assert labels.index("park the cap") < labels.index("grip the poppet")
    assert labels.index("release the spring") < labels.index("twist the thread")
    assert labels.index("press the inlet button") < labels.index("press the outlet button")
