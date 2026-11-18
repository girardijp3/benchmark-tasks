"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
cam seated, bolt engaged, cam turned and panels flush + the 1 s hold and its restart; each cap (cam not seated, bolt
cross-threaded, panels not flush) holds the score at 0.5 and blocks 1.0; the geometry helpers behave; every demo
waypoint is reachable with the YAM arms and the plan fits the run limit and returns home.

Run in the yam_arena environment:  python -m pytest -q test_flat_pack_cam_lock.py
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
TASK = load(HERE / "flat_pack_cam_lock.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(cam=False, bolt=False, turned=False, flush=False, torqued=False,
             cap_cam=False, cap_bolt=False, cap_panel=False, grippers=None):
    return {
        "cam_seated": cam, "seated_latched": cam,
        "bolt_engaged": bolt, "bolt_latched": bolt,
        "turned": turned, "flushed": flush, "torqued": torqued,
        "cap_cam": cap_cam, "cap_bolt": cap_bolt, "cap_panel": cap_panel,
        "turn": math.radians(180.0), "gap": 0.0,
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    base = dict(cam=True, bolt=True, turned=True, flush=True, torqued=True)
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
    for name in ("fixture", "panel_b", "cam", "bolt", "screwdriver"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    fixture = (HERE / "assets" / "fixture.usda").read_text()
    for token in ("FacePanel", "CamBore", "BoreRim", "EdgeArrow", "CamPocket", "ScrewHolder"):
        assert token in fixture
    panel = (HERE / "assets" / "panel_b.usda").read_text()
    for token in ("MatingPanelBody", "BoltHole", "HoleRim", "HoleMark"):
        assert token in panel
    cam = (HERE / "assets" / "cam.usda").read_text()
    for token in ("CamDrum", "CamCollider", "CrossRecessA", "CrossRecessB", "EccentricSlot", "IndexDot"):
        assert token in cam
    bolt = (HERE / "assets" / "bolt.usda").read_text()
    for token in ("Thread", "StopCollar", "Projection", "BoltHead", "BoltCollider"):
        assert token in bolt
    screw = (HERE / "assets" / "screwdriver.usda").read_text()
    for token in ("Handle", "Shaft", "Tip", "ScrewCollider"):
        assert token in screw


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_cam_seated_quarter():
    o = _objects(cam=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_bolt_engaged_half():
    o = _objects(cam=True, bolt=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_cam_turned_three_quarters():
    o = _objects(cam=True, bolt=True, turned=True)
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
    broken = _objects(cam=True, bolt=True, turned=True, flush=True)  # torque not held
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["cap_cam", "cap_bolt", "cap_panel"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(cam=True, bolt=True, turned=True, cap_bolt=True)
    assert T.score(o) == pytest.approx(0.5)


def test_not_flush_keeps_full_false():
    o = _full(flush=False)
    assert T._current_ok(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_no_torque_hold_keeps_full_false():
    o = _full(torqued=False)
    assert T.steps_done(o)[3] is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.CAM_XY[0], T.CAM_XY[1], T.TOP + 0.015]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    assert 2 * T.CAM_R == pytest.approx(0.015)          # true 15 mm cam
    assert 2 * T.CAM_BORE_R == pytest.approx(0.015)     # 15 mm bore
    assert T.CAM_H == pytest.approx(0.012)
    assert 2 * T.BOLT_R == pytest.approx(0.006)         # M6 shank
    assert 2 * T.BOLT_HEAD_R == pytest.approx(0.007)    # 7 mm head
    assert 2 * T.BOLT_HOLE_R == pytest.approx(0.008)    # 8 mm hole
    assert T.PANEL_A_TOP == pytest.approx(0.016)        # 16 mm board
    assert T.TOP == pytest.approx(0.7475)
    # drilling distance B = 24 mm from the joining edge to the cam centre
    assert T.CAM_XY[1] - T.A_EDGE_Y == pytest.approx(0.024)
    # bolt head reaches the cam centre: 8 mm head + projection >= 24 mm drilling distance
    assert 0.025 + T.BOLT_HEAD_R >= T.CAM_XY[1] - T.A_EDGE_Y - T.CAM_BORE_R


def test_declared_mechanism_numbers():
    assert math.degrees(T.TURN_TARGET) == pytest.approx(180.0)
    assert math.degrees(T.TURN_DONE) == pytest.approx(165.0)
    assert math.degrees(T.TURN_CAPTURE) == pytest.approx(90.0)
    assert math.degrees(T.TURN_STRIP) == pytest.approx(215.0)
    assert math.degrees(T.CROSS_TILT) == pytest.approx(20.0)
    assert T.PANEL_GAP_START == pytest.approx(0.003)
    assert T.PANEL_FLUSH_TOL == pytest.approx(0.0005)
    assert T.PRESS_DWELL == pytest.approx(0.35)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
    assert T.GRASP_UP == pytest.approx(0.012)


def test_cap_thresholds_are_wider_than_alignment():
    # a bolt can be outside the 8 deg square window without cross-threading until 20 deg
    assert T.CROSS_TILT > T.BOLT_TILT_TOL
    assert T.TURN_STRIP > T.TURN_TARGET > T.TURN_DONE > T.TURN_CAPTURE


def test_geometry_helpers():
    assert T._tilt_error(np.eye(3)) == pytest.approx(0.0, abs=1e-9)
    assert T._tilt_error(T.BOLT_TO_Z) == pytest.approx(math.pi / 2, abs=1e-9)
    assert T._bolt_axis_error(T.BOLT_TO_Z) == pytest.approx(0.0, abs=1e-9)
    assert T._bolt_axis_error(np.eye(3)) == pytest.approx(math.pi / 2, abs=1e-9)
    # yaw wraps across +/-pi without a 360 deg jump (the 180 deg turn accumulates monotonically)
    assert T._wrap(-math.pi - 0.1) == pytest.approx(math.pi - 0.1, abs=1e-9)
    assert T._wrap(2 * math.pi + 0.1) == pytest.approx(0.1, abs=1e-9)


def test_bolt_head_reaches_bore_in_the_inserted_pose():
    # local +z of the bolt maps to world +y; the head centre must land across the cam bore
    head_world = np.array([T.BOLT_XY[0], T.BOLT_XY[1], 0.0]) + np.array([0.0, 0.011, 0.0])
    assert abs(head_world[0] - T.CAM_XY[0]) < 0.001
    assert abs(head_world[1] - T.CAM_XY[1]) < 0.010
    assert abs(T.BOLT_TO_Z @ np.array([0.0, 0.0, 1.0]) - np.array([0.0, 1.0, 0.0])).sum() < 1e-9


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
    assert labels.index("grip the cam") < labels.index("the cam in its bore")
    assert labels.index("grip the bolt") < labels.index("the bolt in the hole")
    assert labels.index("the bolt in the hole") < labels.index("engage the cam recess")
    assert labels.index("engage the cam recess") < labels.index("turn cam 180 deg")
    assert labels.index("hold the flush joint") < labels.index("home")
