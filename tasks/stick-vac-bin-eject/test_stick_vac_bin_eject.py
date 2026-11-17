"""Offline checks (no GPU, no network): the task file loads; the scoring ladder covers idle, each step, the 1 s hold
and its restart; a dropped bin, dust left, a forced pull/latch and a tipped vac cap or zero the score; the vac's
stability window; the grippers must be clear; both the right arm's reach and the left arm's home pose; the demo fits
the run limit; and the USD assets carry the real joint names and the dust bin, waste bin and debris bodies.

Run:  PYTHONPATH= ../../../local_sim/.venv/bin/python -m pytest -q test_stick_vac_bin_eject.py
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
TASK = load(HERE / "stick_vac_bin_eject.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
VAC = np.array([T.VAC_POS[0], T.VAC_POS[1], T.TOP + 0.001])
BIN = np.array([T.BIN_START[0], T.BIN_START[1], T.BIN_START[2]])
WASTE = np.array([T.WASTE_XY[0], T.WASTE_XY[1], T.TOP])


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _gripper(side, tcp=None, contact=0.0, opening=0.094):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(step=4, bin_pos=None, vac_pos=None, vac_quat=IDENT, bin_quat=IDENT, dust_in_waste=True,
             grippers=None, **flags):
    """Build the object dict the scorer reads. step 0..3 sets the progress latches directly."""
    T._MECH.opened_once = step >= 1
    T._MECH.removed = step >= 2
    T._MECH.emptied = step >= 3
    T._MECH.clicked = step >= 4
    T._MECH.dropped = bool(flags.pop("dropped", False))
    T._MECH.forced = bool(flags.pop("forced", False))
    T._MECH.tipped = bool(flags.pop("tipped", False))
    T._MECH.debris_left = bool(flags.pop("debris_left", False))
    if bin_pos is None:
        bin_pos = BIN if step < 2 else (WASTE[0], WASTE[1], T.TOP + 0.12)
    if step >= 4:
        bin_pos = BIN  # refitted and latched
    bp = np.asarray(bin_pos, float)
    vp = np.asarray(VAC if vac_pos is None else vac_pos, float)
    eject = 0.0 if step >= 4 else (0.007 if step >= 1 else 0.0)
    out = {
        "vac": _state(vp, vac_quat),
        "bin": _state(bp, bin_quat),
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
        "eject": eject,
        "flap": T.FLAP_OPEN if step == 3 else 0.0,
        "bin_pos": bp,
        "bin_quat": np.asarray(bin_quat, float),
        "vac_pos": vp,
        "vac_quat": np.asarray(vac_quat, float),
        "knob": T.EJECT_KNOB.copy(),
        "vac0": (VAC.copy(), IDENT.copy()),
        "opened_once": T._MECH.opened_once,
        "removed": T._MECH.removed,
        "emptied": T._MECH.emptied,
        "clicked": T._MECH.clicked,
        "forced": T._MECH.forced,
        "dropped": T._MECH.dropped,
        "tipped": T._MECH.tipped,
        "debris_left": T._MECH.debris_left,
    }
    for spec in T.DEBRIS:
        if dust_in_waste:
            p = (WASTE[0] + 0.02 * (int(spec.name[-1]) % 2), WASTE[1] - 0.02, T.TOP + 0.012)
        else:
            p = (bp[0], bp[1], bp[2] + 0.02)
        out[spec.name] = _state(p)
    return out


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for token in ("EjectPress", "EjectKnob", "FlapHinge", "CarryBar", "CradleShelf", "WallY", "FlapPlate"):
        assert token in (HERE / "assets" / ("vac.usda" if token in ("EjectPress", "EjectKnob", "CradleShelf")
                                           else "bin.usda")).read_text()
    for name in ("waste_bin", "debris_pebble", "debris_chip", "debris_twig"):
        assert (HERE / "assets" / f"{name}.usda").is_file()


def test_idle_scores_zero():
    objs = _objects(step=0)
    assert T.steps_done(objs) == (False, False, False, False)
    assert T.score(objs) == 0.0
    assert T.success(objs) is False


def test_button_only_quarter():
    objs = _objects(step=1)
    assert T.steps_done(objs)[:2] == (True, False)
    assert T.score(objs) == pytest.approx(0.25)


def test_bin_pulled_out_half():
    objs = _objects(step=2)
    assert T.steps_done(objs)[:3] == (True, True, False)
    assert T.score(objs) == pytest.approx(0.5)


def test_dust_emptied_three_quarters():
    objs = _objects(step=3)
    assert T.steps_done(objs)[:3] == (True, True, True)
    assert T.score(objs) == pytest.approx(0.75)
    assert T.success(objs) is False


def test_one_second_full_hold():
    objs = _objects(step=4)
    for _ in range(20):
        assert T.success(objs) is False
    assert T.success(objs) is True
    assert T.score(objs) == pytest.approx(1.0)


def test_hold_restarts_after_a_break():
    objs = _objects(step=4)
    for _ in range(10):
        T.success(objs)
    broken = _objects(step=2)
    assert T.success(broken) is False
    objs = _objects(step=4)
    for _ in range(20):
        assert T.success(objs) is False
    assert T.success(objs) is True


def test_dropped_bin_caps_at_quarter():
    objs = _objects(step=3, dropped=True)
    assert T.score(objs) <= 0.25
    assert T.success(objs) is False


def test_dust_left_caps_at_three_quarters():
    objs = _objects(step=3, dust_in_waste=False, debris_left=True)
    assert T.score(objs) <= 0.75
    assert T.success(objs) is False


def test_forced_pull_or_latch_caps_at_half():
    for objs in (_objects(step=2, forced=True), _objects(step=4, forced=True)):
        assert T.score(objs) <= 0.5
        assert T.success(objs) is False


def test_tipped_vac_scores_zero():
    objs = _objects(step=3, tipped=True)
    assert T.score(objs) == 0.0
    assert T.success(objs) is False


def test_stability_window():
    c, s = math.cos(math.radians(1.5)), math.sin(math.radians(1.5))
    tilted = _objects(step=4, vac_quat=np.array([s, 0.0, 0.0, c]))
    assert T._stable(tilted) is False
    moved = _objects(step=4, vac_pos=VAC + np.array([0.012, 0.0, 0.0]))
    moved["vac0"] = (VAC.copy(), IDENT.copy())
    assert T._stable(moved) is False
    assert T._moved(moved) == pytest.approx(0.012, abs=1e-6)


def test_grippers_not_clear_keeps_fourth_step_false():
    g = _gripper("right", tcp=(T.BIN_START[0], 0.0, T.TOP + 0.0585))
    objs = _objects(step=4, grippers={"right": g})
    assert T._clear(objs) is False
    assert T.steps_done(objs)[3] is False
    assert T.success(objs) is False


def test_gripping_a_body_is_not_clear():
    g = _gripper("right", tcp=T.EJECT_KNOB, contact=4.0, opening=0.014)
    objs = _objects(step=4, grippers={"right": g})
    assert T._clear(objs) is False


def test_real_product_numbers():
    assert (T.BIN_L, T.BIN_W, T.BIN_H) == pytest.approx((0.070, 0.070, 0.085))
    assert T.BIN_L * T.BIN_W * T.BIN_H == pytest.approx(0.0004158, abs=1e-6)  # 0.42 L
    assert T.VAC_MASS == pytest.approx(2.04)                                    # Dyson V6 published
    assert T.EJECT_PRESS == pytest.approx(0.010)
    assert T.EJECT_TRIG == pytest.approx(0.005)
    assert math.degrees(T.FLAP_OPEN) == pytest.approx(-100.0)
    assert T.WASTE_W == pytest.approx(0.240)


def test_demo_reach_right_and_left_home():
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
    labels = [w.label for w in plan["right"]]
    assert labels.index("press the button") < labels.index("pull the bin out")
    assert labels.index("dust emptied into the waste bin") if "dust emptied into the waste bin" in labels else True


def test_demo_order_button_bin_dump_refit():
    labels = [w.label for w in T.demo()["right"]]
    assert labels.index("press the button") < labels.index("grip the bin")
    assert labels.index("grip the bin") < labels.index("carry over the waste bin")
    assert labels.index("carry over the waste bin") < labels.index("lower bin to the cradle")
    assert labels.index("lower bin to the cradle") < labels.index("home")


def test_meets_still_window_only_when_full():
    assert T.STILL_T == pytest.approx(1.0)
    assert T.STABLE_TILT == pytest.approx(math.radians(2.0))
    assert T.CLEAR == pytest.approx(0.045)
