"""Offline checks for lighting-track-adapter: the scoring ladder and caps, the 1 s hold, the real product values,
the USD parts and joints, the demo plan, both arms' static reach, and a pass-through check on the grips.

Run from this folder:  PYTHONPATH=../../../local_sim ../../../local_sim/.venv/bin/python -m pytest -q
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _load():
    spec = importlib.util.spec_from_file_location("lighting_track_adapter_task", HERE / "lighting_track_adapter.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = m
    spec.loader.exec_module(m)
    return m


T = _load()


def _gripper(tcp=(0.0, -0.60, 1.00), contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    axis = np.array([0.0, 0.0, 1.0])
    return T.GripperState(back=tcp - 0.025 * axis, tips=tcp + 0.013 * axis, opening=opening, contact=contact,
                          tcp=tcp, rot=np.eye(3))


def _objects(**kw):
    o = dict(inserted=True, locked=False, tug_passed=False, wrong_way=False, dropped=False, bent_track=False,
             failed_tug=False, lock_deg=0.0, insert_mm=0.0, insert_rate=0.0, twist_rate=0.0, episode_time=0.0,
             left_gripper=_gripper((0.0, 0.60, 1.0)), right_gripper=_gripper((0.0, -0.60, 1.0)))
    o.update(kw)
    return o


def _reset():
    T._HOLD.__init__()


# -- scoring ladder --------------------------------------------------------------------------------------------------
def test_idle_is_zero():
    _reset()
    assert T.score(_objects(inserted=False, lock_deg=0.0)) == 0.0
    assert T.success(_objects(inserted=False)) is False


def test_inserted_turn_scores_quarter():
    _reset()
    assert T.score(_objects(inserted=True, lock_deg=-30.0)) == 0.25


def test_locked_scores_half():
    _reset()
    assert T.score(_objects(locked=True, lock_deg=-90.0)) == 0.5


def test_locked_and_tug_scores_three_quarters():
    _reset()
    assert T.score(_objects(locked=True, tug_passed=True, lock_deg=-90.0)) == 0.75


def test_one_second_hold_scores_one():
    _reset()
    o = _objects(locked=True, tug_passed=True, lock_deg=-90.0)
    assert T.success(o) is False
    o["episode_time"] = 0.5
    assert T.success(o) is False
    o["episode_time"] = 1.0
    assert T.success(o) is True
    assert T.score(o) == 1.0


def test_hold_restarts_if_lock_slips():
    _reset()
    o = _objects(locked=True, tug_passed=True, lock_deg=-90.0, episode_time=0.0)
    o["episode_time"] = 0.9
    o["lock_deg"] = -86.0
    assert T.success(o) is False
    o["episode_time"] = 1.4
    assert T.success(o) is False  # the second restarted after the slip
    o["episode_time"] = 2.0
    assert T.success(o) is True


def test_clear_requires_both_grippers_outside_the_head_box():
    assert T.clear(_objects()) is True
    assert T.clear(_objects(right_gripper=_gripper((0.42, -0.10, 0.947)))) is False
    assert T.clear(_objects(left_gripper=_gripper((0.42, -0.05, 0.950)))) is False


@pytest.mark.parametrize("fault", ["wrong_way", "bent_track"])
def test_orientation_and_bent_cap_at_quarter(fault):
    _reset()
    assert T.score(_objects(**{fault: True}, lock_deg=-90.0)) == 0.25


def test_dropped_is_zero():
    _reset()
    assert T.score(_objects(dropped=True, locked=True, tug_passed=True)) == 0.0
    assert T.success(_objects(dropped=True, locked=True, tug_passed=True)) is False


def test_failed_tug_caps_at_half():
    _reset()
    assert T.score(_objects(failed_tug=True, locked=True, lock_deg=-90.0)) == 0.5
    assert T.score(_objects(failed_tug=True, inserted=True, lock_deg=-30.0)) == 0.25


def test_faults_block_success():
    for fault in ("wrong_way", "dropped", "bent_track", "failed_tug"):
        _reset()
        assert T.success(_objects(locked=True, tug_passed=True, lock_deg=-90.0, **{fault: True})) is False


def test_not_locked_has_no_half_score():
    _reset()
    assert T.score(_objects(inserted=True, lock_deg=-60.0)) == 0.25
    assert T.score(_objects(inserted=False, lock_deg=0.0)) == 0.0


# -- real product values ------------------------------------------------------------------------------------------
def test_real_track_and_head_dimensions():
    assert T.TRACK_L == pytest.approx(1.000)
    assert T.TRACK_W == pytest.approx(0.035)
    assert T.TRACK_H == pytest.approx(0.0175)
    assert T.SLOT_W == pytest.approx(0.024)
    assert T.HEAD_R == pytest.approx(0.034925)
    assert T.HEAD_H == pytest.approx(0.0762)
    assert T.LEVER_R == pytest.approx(0.090)


def test_bench_height_and_torque_values():
    assert T.TRACK_Z - T.TOP == pytest.approx(0.250, abs=1e-9)
    assert T.REAL_LOCK_TORQUE == (0.30, 0.60)
    assert T.TUG_FORCE_EST == pytest.approx(10.0)
    assert T.TUG_DOWN == pytest.approx(0.004)


def test_lock_and_wrong_angles():
    assert T.LOCK_DEG == (-95.0, -85.0)
    assert T.WRONG_HI == 10.0 and T.WRONG_LO == -100.0
    assert T.DETENT_ANGLE == pytest.approx(-70.0)


def test_assets_exist():
    for n in ("lighting_track.usda", "track_head.usda"):
        assert (T.ASSETS / n).is_file(), n


def test_track_asset_has_slot_lips_and_polarity_groove():
    from pxr import Usd
    stage = Usd.Stage.Open(str(T.ASSETS / "lighting_track.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    for n in ("TrackLip", "BusBar", "PolarityGroove", "Pedestal", "LockoutTag"):
        assert any(x.startswith(n) for x in names), n


def test_head_asset_has_adapter_lugs_lever_and_joints():
    from pxr import Usd
    stage = Usd.Stage.Open(str(T.ASSETS / "track_head.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    for n in ("AdapterBody", "Tab", "LeverKnob", "HeadBody", "HeadCollider"):
        assert any(x.startswith(n) for x in names), n
    joints = {p.GetName() for p in stage.Traverse() if p.GetName() in ("Twist", "Insert")}
    assert joints == {"Twist", "Insert"}


# -- demo plan and reach ------------------------------------------------------------------------------------------
def test_demo_turns_lever_and_tugs_with_both_hands():
    plan = T.demo()
    left = [w.label for w in plan["left"]]
    right = [w.label for w in plan["right"]]
    assert any("turn the lever 90" in s for s in left)
    assert any("grip the lever" in s for s in left)
    assert any("pull the stem" in s for s in right)
    assert any("wait for the lock" in s for s in right)


def test_demo_arms_do_not_overlap_in_time():
    plan = T.demo()
    lwait = next(i for i, w in enumerate(plan["left"]) if "hold the lock" in w.label)
    rgrip = next(i for i, w in enumerate(plan["right"]) if "grip the stem" in w.label)
    assert rgrip > 0
    assert lwait >= 0


def _best(pos, rot, side, seeds=8):
    from yam_arena import ik
    from yam_arena.station import BASES

    q4 = ik.matrix_to_quat_xyzw(rot)
    rng = np.random.default_rng(4)
    lo, hi = np.array(ik.LOWER), np.array(ik.UPPER)
    out = (1e9, 0.0)
    for s in [np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])] + [rng.uniform(lo, hi) for _ in range(seeds)]:
        q, pe, re = ik.solve(pos, q4, s, base_pos=BASES[side], iters=100)
        out = min(out, (pe * 1000.0, math.degrees(re)), key=sum)
    return out


def test_left_arm_reaches_lever_open_and_locked():
    z = T.SEAT_Z + T.LEVER_Z
    for yaw in (0.0, -45.0, -90.0):
        pe, re = _best(np.array(T._lever(yaw, z)), T._lever_rot(yaw), "left")
        assert pe < 2.0 and re < 2.0, (yaw, pe, re)


def test_right_arm_reaches_stem_tug():
    pe, re = _best(np.array([T.TRACK_X, T.TRACK_Y, T.STEM_Z]), T._SIDE, "right")
    assert pe < 2.0 and re < 2.0, (pe, re)


# -- pass-through / grasp surface -----------------------------------------------------------------------------------
def test_grip_points_are_on_real_surfaces_not_inside_solids():
    """The left pads are on the 24 mm lever knob and the right pads on the 20 mm stem; neither pad centre may be
    deeper than 0.5 mm inside the 40 mm adapter body or the 69.85 mm head body at those TCPs."""
    zk = T.SEAT_Z + T.LEVER_Z
    knob = np.array(T._lever(0.0, zk))
    stem = np.array([T.TRACK_X, T.TRACK_Y, T.STEM_Z])
    # pad centres sit on the two sides of each handled cylinder (knob r 12 mm, stem r 10 mm)
    left_pad = knob + np.array([-0.012, 0.0, 0.0])
    right_pad = stem + np.array([-0.010, 0.0, 0.0])
    axis_xy = np.array([T.TRACK_X, T.TRACK_Y])
    for pad, handled_r in ((left_pad, 0.012), (right_pad, 0.010)):
        # pad is on the handled cylinder, outside the adapter body (r 20 mm) and the head (r 34.925 mm)
        d = float(np.linalg.norm(pad[:2] - axis_xy))
        assert d >= handled_r - 0.0005
        assert d > 0.020 or pad[2] < T.SEAT_Z - 0.030  # clear of the adapter body
