"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
gusset seated, first clamp over-centre, second clamp over-centre, clamped-square + the 1 s hold and its restart;
every cap (gusset shift, clamp not over-centre, dropped part) holds the score at 0.5 and blocks 1.0; the demo fits
the run limit, returns home and reaches every waypoint with the YAM arms.

Run in the yam_arena environment:  python -m pytest -q test_toggle_clamp_gusset.py
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
TASK = load(HERE / "toggle_clamp_gusset.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
OVER = math.radians(70.0)


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(seated=False, ca=False, cb=False, gusset_shift=False, clamp_not_over=False, dropped=False, grippers=None,
             gusset_pos=None, gusset_quat=None, twist=None):
    seat_pos = gusset_pos if gusset_pos is not None else np.array([T.FX, T.FY, T.TOP + T.GUSSET_SEAT_CZ])
    tw = {"clamp_a": OVER, "clamp_b": OVER}
    if twist is not None:
        tw.update(twist)
    return {
        "gusset_seated": seated,
        "clamp_a_done": ca,
        "clamp_b_done": cb,
        "gusset_shift": gusset_shift,
        "clamp_not_over": clamp_not_over,
        "dropped": dropped,
        "twist": tw,
        "gusset": ObjectState(pos=seat_pos, quat_xyzw=IDENT if gusset_quat is None else gusset_quat,
                              lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=seat_pos, high=seat_pos),
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    return _objects(seated=True, ca=True, cb=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("fixture", "gusset", "clamp"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    fx = (HERE / "assets" / "fixture.usda").read_text()
    for token in ("FixturePlate", "LocatorBackWall", "LocatorPin", "ClampBase", "ClampPivotPost", "StageTray"):
        assert token in fx
    gus = (HERE / "assets" / "gusset.usda").read_text()
    for token in ("Foot", "Upright", "WebLow0"):
        assert token in gus


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_gusset_seated_quarter():
    o = _objects(seated=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_clamp_a_half():
    o = _objects(seated=True, ca=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_clamp_b_three_quarters():
    o = _objects(seated=True, ca=True, cb=True)
    assert T.steps_done(o) == (True, True, True, False)
    assert T.score(o) == pytest.approx(0.75)
    assert T.success(o) is False


def test_full_one_second_hold():
    o = _full()
    assert T.steps_done(o) == (True, True, True, False)  # the 4th step is the timed hold
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True
    assert T.steps_done(o) == (True, True, True, True)
    assert T.score(o) == pytest.approx(1.0)


def test_hold_restarts_after_a_break():
    o = _full()
    for _ in range(10):
        T.success(o)
    broken = _objects(seated=True, ca=True)  # clamp B not thrown
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["gusset_shift", "clamp_not_over", "dropped"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, False)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(seated=True, ca=True, cb=True, dropped=True)
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.FX, T.FY, T.TOP + 0.115]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_shifted_gusset_blocks_full():
    shifted = np.array([T.FX + 0.03, T.FY, T.TOP + T.GUSSET_SEAT_CZ])
    o = _full(gusset_pos=shifted)
    assert T._current_ok(o) is False
    assert T._full(o) is False


def test_clamp_not_over_twist_blocks_full():
    o = _full(twist={"clamp_b": math.radians(30.0)})
    assert T._current_ok(o) is False
    assert T._full(o) is False


def test_real_product_sizes():
    assert T.FX == pytest.approx(0.280)
    assert T.FY == pytest.approx(0.0)
    assert T.FIX_TOP == pytest.approx(0.045)
    # a 100 mm gusset: 100 mm foot and upright (see REFERENCE.md)
    assert T.GUSSET_STAGE[1] == pytest.approx(0.100)
    assert T.CLAMP_X[1] - T.CLAMP_X[0] == pytest.approx(0.070)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_mechanism_numbers():
    assert math.degrees(T.OVER_YAW) == pytest.approx(70.0)
    assert math.degrees(T.OVER_TOL) == pytest.approx(15.0)
    assert T.SEAT_XY_TOL == pytest.approx(0.007)
    assert T.CLAMP_POS_OK == pytest.approx(0.012)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
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
    left_labels = [w.label for w in plan["left"]]
    right_labels = [w.label for w in plan["right"]]
    assert "release the gusset" in left_labels
    assert left_labels.index("release the gusset") < left_labels.index("home")
    assert right_labels[0] == "wait for gusset"
    assert right_labels.index("flip clamp A over-centre") < right_labels.index("grip the clamp B")
    assert right_labels.index("flip clamp B over-centre") < right_labels.index("home")
