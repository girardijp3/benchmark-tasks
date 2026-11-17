"""Offline checks for headlamp-aim-adjustment: the scoring ladder and caps, the beam geometry, the real
asset's parts, the two keys, the demo plan, and both arms' static reach at the keys and the steady pad.

Run from this folder:  PYTHONPATH=../../../local_sim ../../../local_sim/.venv/bin/python -m pytest -q
"""
import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _load_task():
    spec = importlib.util.spec_from_file_location("headlamp_aim_adjustment_task", HERE / "headlamp_aim_adjustment.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


T = _load_task()


def _gripper(tcp=(0.0, -0.60, 1.00), contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    axis = np.array([0.0, 0.0, 1.0])
    return T.GripperState(back=tcp - 0.025 * axis, tips=tcp + 0.013 * axis, opening=opening, contact=contact,
                          tcp=tcp, rot=np.eye(3))


def _objects(**kw):
    o = dict(v_err=0.0, h_err=0.0, on_screen=True, v_engaged=True, wrong_way=False, stripped=False,
             forced=False, unsteady=False, cut_u=T.SCREEN_Y, cut_h=T.TARGET_H, episode_time=0.0,
             lamp_speed=0.0, lamp_rate=0.0, left_gripper=_gripper((0.0, 0.60, 1.0)),
             right_gripper=_gripper((0.0, -0.60, 1.0)))
    o.update(kw)
    return o


def _reset_hold():
    T._HOLD.__init__()


# -- the scoring ladder ------------------------------------------------------------------------------------------------
def test_idle_is_zero():
    _reset_hold()
    o = _objects(v_engaged=False, v_err=-40.0, h_err=40.0)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_vertical_screw_engaged_scores_quarter():
    _reset_hold()
    assert T.score(_objects(v_err=-40.0, h_err=40.0)) == 0.25


def test_vertical_cutoff_in_band_scores_half():
    _reset_hold()
    assert T.score(_objects(v_err=0.0, h_err=60.0)) == 0.5


def test_both_in_band_scores_three_quarters():
    _reset_hold()
    assert T.score(_objects()) == 0.75


def test_band_edges():
    assert T.v_in_band(_objects(v_err=T.BAND_V * 1000.0)) is True
    assert T.v_in_band(_objects(v_err=T.BAND_V * 1000.0 + 0.1)) is False
    assert T.h_in_band(_objects(h_err=-T.BAND_H * 1000.0)) is True
    assert T.h_in_band(_objects(h_err=-T.BAND_H * 1000.0 - 0.1)) is False


def test_one_second_hold_scores_one():
    _reset_hold()
    o = _objects(episode_time=0.0)
    assert T.success(o) is False
    o["episode_time"] = 0.5
    assert T.success(o) is False
    o["episode_time"] = 1.0
    assert T.success(o) is True
    assert T.score(o) == 1.0


def test_hold_restarts_when_the_beam_moves():
    _reset_hold()
    o = _objects(episode_time=0.0)
    assert T.success(o) is False
    o["episode_time"] = 0.9
    o["cut_u"] += 0.004
    assert T.success(o) is False
    o["episode_time"] = 1.4
    o["cut_u"] -= 0.004
    assert T.success(o) is False
    o["episode_time"] = 2.4
    assert T.success(o) is True


def test_clear_requires_both_grippers_outside_the_lamp_box():
    assert T.clear(_objects()) is True
    assert T.clear(_objects(right_gripper=_gripper((0.46, -0.10, 0.90)))) is False
    assert T.clear(_objects(left_gripper=_gripper((0.46, -0.08, 0.90)))) is False


@pytest.mark.parametrize("fault", ["wrong_way", "stripped", "forced", "unsteady"])
def test_faults_cap_at_quarter(fault):
    _reset_hold()
    o = _objects(**{fault: True})
    assert T.score(o) == 0.25
    assert T.success(o) is False


def test_beam_off_screen_caps_at_quarter():
    _reset_hold()
    assert T.score(_objects(on_screen=False)) == 0.25
    assert T.success(_objects(on_screen=False)) is False


def test_missing_vertical_engagement_has_no_quarter():
    _reset_hold()
    o = _objects(v_engaged=False, stripped=True)
    assert T.score(o) == 0.0


# -- the beam geometry and the real values ------------------------------------------------------------------------------
def test_three_metre_screen_and_ece_target():
    assert T.SCREEN_X == pytest.approx(T.LENS_X + 3.000, abs=1e-9)
    assert T.TARGET_DROP == pytest.approx(0.036, abs=1e-9)          # 0.69 deg / 1.2 % at 3.00 m
    assert T.TARGET_H == pytest.approx(0.824, abs=1e-9)
    assert T.SCREEN_Y == pytest.approx(-0.130, abs=1e-9)
    assert T.BAND_V == pytest.approx(0.015, abs=1e-9)
    assert T.BAND_H == pytest.approx(0.026, abs=1e-9)


def test_beam_projection_from_a_level_lamp():
    """A level lamp with its lens on the target height puts the cutoff 36 mm below the lamp centre."""
    # Synthetic lamp pose: the lens centre 0.950 m, level, aimed at the screen; the beam's centre ray.
    class _Rig:
        body_names = ["AimPitch"]

    class _Data:
        body_link_pose_w = None

    lens = np.array([0.508, -0.130, 0.950])
    R = np.eye(3)
    d = R @ np.array([1.0, 0.0, 0.0]) * math.cos(T.ALPHA0) - R @ np.array([0.0, 0.0, 1.0]) * math.sin(T.ALPHA0)
    t = (T.SCREEN_X - lens[0]) / d[0]
    p = lens + t * d
    assert p[1] == pytest.approx(-0.130, abs=1e-6)
    assert p[2] == pytest.approx(0.950 - 0.036, abs=1e-6)
    assert T.ALPHA0 == pytest.approx(math.atan(0.012), abs=1e-12)


def test_adjuster_ratios_and_strip_torque_are_declared():
    assert T.K_V == pytest.approx(-1.20 * math.pi / 180 / (2 * math.pi), rel=1e-12)
    assert T.K_H == pytest.approx(1.40 * math.pi / 180 / (2 * math.pi), rel=1e-12)
    assert T.STRIP_TORQUE == pytest.approx(0.9)
    assert T.STRIP_SLIP == pytest.approx(20.0 * math.pi / 180, rel=1e-12)


# -- the real assets ---------------------------------------------------------------------------------------------------
def test_assets_exist():
    for name in ("headlamp_rig.usda", "beam_screen.usda", "beam_patch.usda"):
        assert (T.ASSETS / name).is_file(), name


def test_rig_has_the_two_keys_the_two_screws_and_the_gimbal():
    from pxr import Usd
    stage = Usd.Stage.Open(str(T.ASSETS / "headlamp_rig.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    for name in ("AimYaw", "AimPitch", "ScrewHeadV", "ScrewHeadH", "ScrewV", "ScrewH", "KeyV", "KeyH"):
        assert name in names, name
    joints = {p.GetName() for p in stage.Traverse() if p.GetName().startswith(("Aim", "Screw", "KeyFix"))}
    for name in ("AimYaw", "AimPitch", "ScrewV", "ScrewH", "ScrewHeadV", "ScrewHeadH", "KeyFixV", "KeyFixH"):
        assert name in joints, name


def test_screen_marks_and_the_beam_patch():
    from pxr import Usd
    stage = Usd.Stage.Open(str(T.ASSETS / "beam_screen.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    for name in ("TargetLine", "BandLine", "CentreLine"):
        assert any(n.startswith(name) for n in names), name
    stage = Usd.Stage.Open(str(T.ASSETS / "beam_patch.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    assert any(n.startswith("Beam") for n in names)


# -- the demo plan and both arms' reach ---------------------------------------------------------------------------------
def test_demo_uses_both_keys_and_braces_first():
    plan = T.demo()
    right = plan["right"]
    left = plan["left"]
    labels = [w.label for w in right]
    assert "grip the V key" in labels and "grip the H key" in labels
    assert any("turn V" in s for s in labels) and any("turn H" in s for s in labels)
    assert any("steady" in w.label or "brace" in w.label for w in left)
    # the right waits for the left brace before it approaches the V key
    wait = next(w for w in right if w.label == "wait for the brace")
    brace = next(w for w in left if w.label == "brace the housing")
    assert wait.duration >= 2.0
    assert brace.pos == pytest.approx(tuple(T.STEADY_TCP))


def test_both_arms_can_reach_the_keys_and_the_pad():
    from yam_arena import ik
    from yam_arena.station import BASES

    def best(pos, rot, side, seeds=8):
        q4 = ik.matrix_to_quat_xyzw(rot)
        rng = np.random.default_rng(4)
        lo, hi = np.array(ik.LOWER), np.array(ik.UPPER)
        out = (1e9, 0.0)
        for s in [np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])] + [rng.uniform(lo, hi) for _ in range(seeds)]:
            q, pe, re = ik.solve(pos, q4, s, base_pos=BASES[side], iters=100)
            out = min(out, (pe * 1000.0, math.degrees(re)), key=sum)
        return out

    for y in (T.SCREW_Y["V"], T.SCREW_Y["H"]):
        pe, re = best(np.array([0.331, y, 0.900]), T.tool_rot(0 if y == T.SCREW_Y["V"] else 180), "right")
        assert pe < 2.0 and re < 2.0, (y, pe, re)
    pe, re = best(T.STEADY_TCP, T.STEADY_ROT, "left")
    assert pe < 2.0 and re < 2.0, (pe, re)
