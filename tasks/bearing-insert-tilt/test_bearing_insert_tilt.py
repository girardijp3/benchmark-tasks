"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
bearing lifted, tilted to seat axis, grease hole registered, seated flush + pressed and the 1 s hold and its restart;
every cap (bearing jams, housing scored, forced press) holds the score at 0.5 and blocks 1.0; the tilt/index geometry
functions behave; all demo waypoints are reachable with the YAM arms and the plan fits the run limit and returns home.

Run in the yam_arena environment:  python -m pytest -q test_bearing_insert_tilt.py
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
TASK = load(HERE / "bearing_insert_tilt.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(lifted=False, tilted=False, registered=False, flush=False, pressed=False,
             jammed=False, scored=False, forced=False, grippers=None):
    return {
        "lifted": lifted, "tilted_ok": tilted, "registered_ok": registered,
        "seated_flush": flush, "pressed": pressed,
        "jammed": jammed, "scored": scored, "forced": forced,
        "tilt_err": math.radians(0.2), "yaw_err": math.radians(0.0),
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    base = dict(lifted=True, tilted=True, registered=True, flush=True, pressed=True)
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
    for name in ("housing", "bearing"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    housing = (HERE / "assets" / "housing.usda").read_text()
    for token in ("BasePlate", "HousingBoss", "SphericalSeat", "SeatRim", "GreasePort", "PocketFloor"):
        assert token in housing
    bearing = (HERE / "assets" / "bearing.usda").read_text()
    for token in ("SphericalOD", "BearingCollider", "BoreSleeve", "IndexDot", "GreaseHole"):
        assert token in bearing


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_bearing_lifted_quarter():
    o = _objects(lifted=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_tilted_to_seat_axis_half():
    o = _objects(lifted=True, tilted=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_grease_hole_registered_three_quarters():
    o = _objects(lifted=True, tilted=True, registered=True)
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
    broken = _objects(lifted=True, tilted=True, registered=True, flush=True)  # press not done
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["jammed", "scored", "forced"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(lifted=True, tilted=True, registered=True, forced=True)
    assert T.score(o) == pytest.approx(0.5)


def test_not_flush_keeps_full_false():
    o = _full(flush=False)
    assert T._current_ok(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.SEAT_XY[0], T.SEAT_XY[1], T.TOP + 0.024]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    assert T.HOUSING_XY == pytest.approx((0.250, 0.0))
    assert 2 * T.BALL_R == pytest.approx(0.020)      # true 20 mm spherical OD
    assert 2 * T.BORE_R == pytest.approx(0.008)      # 8 mm shaft bore
    assert T.BEARING_W == pytest.approx(0.007)
    assert math.degrees(T.SEAT_TILT) == pytest.approx(8.0)
    assert T.SEAT_CZ == pytest.approx(0.024)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_mechanism_numbers():
    assert math.degrees(T.PORT_PSI) == pytest.approx(60.0)
    assert math.degrees(T.TILT_TOL) == pytest.approx(4.0)
    assert math.degrees(T.JAM_TILT) == pytest.approx(6.0)
    assert math.degrees(T.YAW_TOL) == pytest.approx(15.0)
    assert T.SEAT_Z_TOL == pytest.approx(0.005)
    assert T.FORCE_TOL == pytest.approx(0.004)
    assert T.PRESS_DWELL == pytest.approx(0.35)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
    assert T.GRASP_UP == pytest.approx(0.012)


def test_cap_thresholds_are_wider_than_alignment():
    # a bearing can be outside the 4 deg align window without jamming until 6 deg
    assert T.JAM_TILT > T.TILT_TOL
    assert T.YAW_TOL < T.PORT_PSI / 2


def test_orientation_target_geometry():
    # the fully seated target has zero tilt and zero index error
    assert T._tilt_error(T.SEAT_ROT) == pytest.approx(0.0, abs=1e-9)
    assert T._yaw_error(T.SEAT_ROT) == pytest.approx(0.0, abs=1e-9)
    # minimal tilt (axis on seat, index at default azimuth) leaves exactly PORT_PSI of index error
    assert T._tilt_error(T.TILT_ROT) == pytest.approx(0.0, abs=1e-9)
    assert T._yaw_error(T.TILT_ROT) == pytest.approx(math.radians(60.0), abs=1e-9)
    # the upright start bearing is SEAT_TILT off axis and PORT_PSI off index
    assert T._tilt_error(np.eye(3)) == pytest.approx(math.radians(8.0), abs=1e-9)
    assert T._yaw_error(np.eye(3)) == pytest.approx(math.radians(60.0), abs=1e-9)


def test_index_error_is_180_degree_symmetric():
    # the two lubrication holes 180 deg apart make those orientations equivalent
    def rot_about_axis(axis, ang):
        axis = np.asarray(axis, float)
        axis /= np.linalg.norm(axis)
        x, y, z = axis
        c, s = math.cos(ang), math.sin(ang)
        C = 1 - c
        return np.array([[c + x*x*C, x*y*C - z*s, x*z*C + y*s],
                         [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
                         [z*x*C - y*s, z*y*C + x*s, c + z*z*C]])
    flipped = rot_about_axis(T.SEAT_AXIS, math.pi) @ T.SEAT_ROT
    assert T._tilt_error(flipped) == pytest.approx(0.0, abs=1e-9)
    assert T._yaw_error(flipped) == pytest.approx(0.0, abs=1e-9)


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
    assert labels.index("grip the bearing") < labels.index("tilt axis to the seat")
    assert labels.index("tilt axis to the seat") < labels.index("rotate grease hole to port")
    assert labels.index("rotate grease hole to port") < labels.index("press bearing flush")
    assert labels.index("press bearing flush") < labels.index("release the bearing")
