"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
alignment, press, seat, the 1 s hold and its restart; every cap (magnets grab off-centre, boot left unseated,
press forced past the shoulder) holds the score at 0.5 and blocks 1.0; real boot/nose sizes; both YAM arms reach
every demo waypoint (warm-started IK, as the scripted policy does); the demo fits the run limit and returns home.

Run in the yam_arena environment:  python -m pytest -q test_magnetic_dust_boot.py
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
TASK = load(HERE / "magnetic_dust_boot.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
CAPS = ("off_centre", "not_seated", "forced_press")


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(aligned=False, pressed=False, seated=False, released=False, centred=True, grippers=None, **caps):
    o = {
        "aligned": aligned, "pressed": pressed, "seated": seated, "released": released, "centred": centred,
        "press_depth": 1.0 if pressed else 0.0, "bottom_z": T.SHOULDER_Z, "dxy": 0.0,
        "pressing": False, "held": None,
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }
    for c in CAPS:
        o[c] = bool(caps.get(c, False))
    return o


def _full(**kw):
    return _objects(aligned=True, pressed=True, seated=True, released=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("spindle", "boot"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    spindle = (HERE / "assets" / "spindle.usda").read_text()
    for token in ("Nose65", "ShoulderFace", "StageBase"):
        assert token in spindle
    boot = (HERE / "assets" / "boot.usda").read_text()
    assert "MagnetRing" in boot and "Bore65" in boot and "ElastomerBody" in boot


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_aligned_quarter():
    o = _objects(aligned=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_pressed_half():
    o = _objects(aligned=True, pressed=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_seated_three_quarters():
    o = _objects(aligned=True, pressed=True, seated=True)
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
    broken = _objects(aligned=True, pressed=True, seated=True)  # release no longer done
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", CAPS)
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(aligned=True, pressed=True, seated=True, off_centre=True)
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.SPINDLE_XY[0], T.SPINDLE_XY[1], T.TOP + 0.07]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_not_centred_blocks_current_ok():
    o = _full(centred=False)
    assert T._current_ok(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_real_product_sizes():
    # 80 mm boot OD, 65 mm bore and spindle nose, 40 mm tall boot
    assert T.SPINDLE_XY == pytest.approx((0.300, 0.0))
    assert 2 * T.BOOT_OD / 2 == pytest.approx(0.080)
    assert 2 * T.BOOT_BORE / 2 == pytest.approx(0.065)
    assert T.NOSE_D == pytest.approx(0.065)
    assert T.BOOT_H == pytest.approx(0.040)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_mechanism_numbers():
    assert T.FORCE_N == pytest.approx((4.0, 7.0))
    assert T.ALIGN_TOL == pytest.approx(0.006)
    assert T.SEAT_Z_TOL == pytest.approx(0.005)
    assert T.OVERPRESS_TOL == pytest.approx(0.004)
    assert T.PRESS_LOCK_FRAC == pytest.approx(0.90)
    assert 0.0 < T.OVERPRESS_TOL < T.SEAT_Z_TOL < T.ALIGN_TOL
    assert T.PRESS_STROKE == pytest.approx(T.APPROACH_CZ - T.BOOT_CZ_SEAT)
    assert T.SEAT_TCP_Z == pytest.approx(T.BOOT_CZ_SEAT + T.GRASP_UP)
    assert T.SHOULDER_Z < T.NOSE_TOP_Z


def test_both_arms_reach_every_demo_waypoint():
    """Warm-started continuation IK, the way ArmPlan.action7 tracks a demo (a single cold solve is too strict)."""
    plan = T.demo()
    for side, wps in plan.items():
        q = np.array([0, 1, 1, -0.6, 0, 0], float)
        for w in wps:
            rot = w.rot if w.rot is not None else yam_demo.grip_rotation(w.yaw)
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot),
                                 q_init=q, base_pos=BASES[side], iters=90)
            assert pe < 0.020, (side, w.label, pe, w.pos)
            assert math.degrees(re) < 6.0, (side, w.label, math.degrees(re), w.pos)


def test_demo_fits_limit_and_returns_home():
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for side, wps in plan.items():
        assert np.allclose(wps[-1].pos, T._START_TCP[side])
        assert wps[-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
        assert sum(w.duration for w in wps) < 60.0  # 1,200 steps at 20 Hz
    labels = [w.label for w in plan["left"]]
    assert labels.index("align the magnets concentrically") < labels.index("press the boot onto the nose")
    assert labels.index("press the boot onto the nose") < labels.index("release the seated boot")
    assert labels.index("release the seated boot") < labels.index("lift away from the spindle")
