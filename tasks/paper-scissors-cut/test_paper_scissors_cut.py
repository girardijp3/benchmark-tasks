"""Offline checks for paper-scissors-cut.

The tests cover the scoring ladder and every cap with synthetic object states (no Isaac needed), the real product
dimensions, the USD assets, the scripted demo's two-hand/open-close plan, static reach at the four critical TCPs, and
the geometric predicates (fingertip press, handle grasp, tray catch and scissor rest).

Run from this folder:
  PYTHONPATH=../../../local_sim ../../../local_sim/.venv/bin/python -m pytest -q
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
    spec = importlib.util.spec_from_file_location("paper_scissors_cut_task", HERE / "paper_scissors_cut.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


T = _load_task()


def _reset():
    T._MECH.reset()
    T._HOLD.reset()
    T._PROG.reset()


def _gripper(side, tcp=None, opening=0.0, contact=0.0):
    """A gripper state with its approach axis pointing down (world -Z)."""
    tcp = np.array(T._START_TCP[side] if tcp is None else tcp, float)
    rot = np.diag([1.0, -1.0, -1.0])
    axis = rot[:, 2]
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis,
                          opening=opening, contact=contact, tcp=tcp, rot=rot)


def _paper():
    return SimpleNamespace(pos=np.array([0.465, -0.0515, T.PAPER_Z], float), quat_xyzw=np.array([0, 0, 0, 1], float))


def _offcut(in_tray=True):
    pos = np.array([0.520, -0.276, T.TRAY_FLOOR + 0.0001], float) if in_tray else np.array([0.9, 0.3, 0.9], float)
    return SimpleNamespace(pos=pos, quat_xyzw=np.array([0, 0, 0, 1], float), low=pos - np.array([0, 0, 0.00005]))


def _objects(**kw):
    """A fully-passing post-cut state; individual tests knock out one condition at a time."""
    pivot = np.array([T.LINE_END - T.BLADE_LEN, T.CUT_Y, T.PAPER_Z], float)
    tip = pivot + np.array([T.BLADE_LEN, 0, 0])
    press_tcp = np.array([T.PRESS_XY[0], T.PRESS_XY[1], T.PAPER_Z + T.PAPER_T / 2 + T.PAD_BELOW_TIP + 0.013])
    handle_tcp = np.array([T.REST_XY[0] - 0.074, T.REST_XY[1], T.REST_Z + 0.0185])
    o = dict(
        episode_time=0.0,
        paper=_paper(), offcut=_offcut(),
        scissor_pivot=pivot, scissor_tip=tip, scissor_bis=np.array([1.0, 0.0]),
        blade_open=0.30, hinge=-0.30, scissor_speed=0.0, scissor_rate=0.0,
        scissor_low=T.PAPER_Z - T.BLADE_T,
        main_shift=0.0, main_turn=0.0, main_moved=False, bond_error=False,
        pressed=True, on_handles=False, grasped=True, started=True,
        cut_front=T.LINE_END + 0.0002, cut_dev=0.0, cut_start_pivot=T.LINE_START,
        cut_started=True, cut_full=True, cut_line_ok=True, cut_off_line=False,
        released=True, pressed_during_cut=True,
        through_tray=False, dropped=False, jam=False,
        offcut_in_tray=True, scissor_back=True, scissor_home_clear=True,
        left_gripper=_gripper("left"), right_gripper=_gripper("right"),
    )
    o.update(kw)
    return o


# -- the scoring ladder -----------------------------------------------------------------------------------------------
def test_idle_is_zero():
    _reset()
    o = _objects(grasped=False, started=False, cut_full=False, released=False, offcut_in_tray=False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_scissors_grasped_scores_quarter():
    _reset()
    o = _objects(started=False, cut_full=False, released=False, offcut_in_tray=False)
    assert T.score(o) == 0.25


def test_paper_aligned_scores_half():
    _reset()
    o = _objects(cut_full=False, released=False, offcut_in_tray=False)
    assert T.score(o) == 0.5


def test_full_cut_scores_three_quarters():
    _reset()
    o = _objects(released=False, offcut_in_tray=False)
    assert T.score(o) == 0.75


def test_success_needs_one_second_still():
    _reset()
    o = _objects(episode_time=0.0)
    assert T.success(o) is False
    assert T.score(o) == 0.75                   # everything done but the 1 s hold
    o["episode_time"] = 0.5
    assert T.success(o) is False
    o["episode_time"] = 1.0
    assert T.success(o) is True
    assert T.score(o) == 1.0


def test_hold_restarts_when_the_offcut_moves():
    _reset()
    o = _objects(episode_time=0.0)
    assert T.success(o) is False
    o["episode_time"] = 0.5
    o["offcut"] = _offcut(False)                # a nudge restarts the still timer
    assert T.success(o) is False
    o["episode_time"] = 1.4
    assert T.success(o) is False
    o["episode_time"] = 1.6
    assert T.success(o) is True


def test_hold_restarts_when_the_blade_tip_moves():
    _reset()
    o = _objects(episode_time=0.0)
    assert T.success(o) is False
    o["episode_time"] = 0.6
    o["scissor_tip"] = o["scissor_tip"] + np.array([0.01, 0, 0])
    assert T.success(o) is False
    o["episode_time"] = 1.7
    assert T.success(o) is True


# -- caps -------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("fault", ["through_tray", "dropped", "jam"])
def test_severe_fault_caps_at_quarter(fault):
    _reset()
    o = _objects(**{fault: True})
    assert T.score(o) == 0.25
    assert T.success(o) is False


@pytest.mark.parametrize("fault", ["bond_error", "main_moved", "cut_off_line"])
def test_quality_fault_caps_at_half(fault):
    _reset()
    if fault == "cut_off_line":
        o = _objects(cut_off_line=True, cut_dev=T.DEV_CAP + 0.001)
    else:
        o = _objects(**{fault: True})
    assert T.score(o) == 0.5
    assert T.success(o) is False


def test_cap_with_no_progress_is_zero():
    _reset()
    o = _objects(grasped=False, started=False, cut_full=False, released=False,
                 offcut_in_tray=False, dropped=True)
    assert T.score(o) == 0.0


def test_off_line_cut_cannot_score_one():
    _reset()
    o = _objects(cut_dev=T.DEV_OK + 0.0005, cut_off_line=False)  # >1 mm, <=2 mm
    assert T._step3(o) is True                                   # still a counted cut
    assert T.score(o) == 0.75
    assert T.success(o) is False


def test_torn_seam_blocks_success_everywhere():
    _reset()
    o = _objects(bond_error=True)
    o["episode_time"] = 2.0
    assert T.success(o) is False
    assert T.score(o) == 0.5


# -- real product and layout -------------------------------------------------------------------------------------------
def test_product_dimensions_and_masses():
    # The task stores half extents; the full sheet is 210 x 297 mm, the strip 210 x 100 mm.
    assert T.PAPER_HALF == (0.105, 0.0985)             # 210 x 197 mm kept part
    assert T.OFFCUT_HALF == (0.105, 0.050)             # 210 x 100 mm strip
    assert math.isclose(T.OFFCUT_HALF[1] * 2, 0.100, abs_tol=1e-9)
    assert math.isclose(T.PAPER_HALF[0] * 2, 0.210, abs_tol=1e-9)
    assert math.isclose(sum(T.OFFCUT_REL), -0.1485, abs_tol=1e-9)
    assert T.LINE_END - T.LINE_START == pytest.approx(0.210, abs=1e-9)
    assert T.BLADE_LEN == pytest.approx(0.090, abs=1e-9)
    assert T.SCISSORS.mass == pytest.approx(0.060, abs=1e-9)
    assert T.PAPER.mass == pytest.approx(0.00331, abs=1e-9)
    assert T.OFFCUT.mass == pytest.approx(0.00168, abs=1e-9)


def test_cut_line_and_tray_layout():
    assert T.CUT_Y == pytest.approx(-0.150, abs=1e-9)
    assert T.LINE_START == pytest.approx(0.360, abs=1e-9)
    assert T.LINE_END == pytest.approx(0.570, abs=1e-9)
    assert T.TRAY_X == (0.318, 0.722)
    assert T.TRAY_Y == (-0.408, -0.144)
    assert T.TRAY_FLOOR == pytest.approx(0.7535, abs=1e-9)
    assert T.TRAY_RIM == pytest.approx(0.7600, abs=1e-9)


def test_hinge_and_cap_thresholds():
    assert T.HINGE_OPEN == pytest.approx(-0.35, abs=1e-9)
    assert T.CUT_OPEN_MAX == pytest.approx(0.10, abs=1e-9)
    assert T.DEV_OK == 0.001                          # full score: within +-1 mm
    assert T.DEV_CAP == 0.002                         # off-line cut cap: > 2 mm
    assert T.CUT_Y_TOL == 0.0015
    assert T.STILL_S == 1.0


def test_run_limit_and_json_fields():
    import json
    d = json.loads((HERE / "task.json").read_text())
    assert d["run_limit"].startswith("2,400 steps")
    assert len(d["title"]) <= 120
    assert len(d["context"]) <= 1000
    assert len(d["setup"]) <= 500
    assert len(d["fidelity"]) <= 500
    assert len(d["scores"]) <= 300
    assert len(str(d["keywords"])) <= 200
    assert 15 <= len(d["instruction"].split()) <= 35


# -- assets ------------------------------------------------------------------------------------------------------------
def test_assets_exist():
    for name in ("scissors.usda", "paper_main.usda", "paper_offcut.usda", "fixtures.usda"):
        assert (T.ASSETS / name).is_file(), name


def test_scissors_asset_has_both_links_blades_and_hinge():
    from pxr import Usd
    stage = Usd.Stage.Open(str(T.ASSETS / "scissors.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    for name in ("ScissorA", "ScissorB", "Shear", "BladeA", "BladeB",
                 "HandleLoopA", "HandleLoopB", "GripPlateA", "GripPlateB"):
        assert any(n.startswith(name) for n in names), name


def test_paper_and_fixture_assets():
    from pxr import Usd
    stage = Usd.Stage.Open(str(T.ASSETS / "paper_main.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    assert any(n.startswith("Sheet") for n in names)
    assert any(n.startswith("CutLine") for n in names)
    stage = Usd.Stage.Open(str(T.ASSETS / "fixtures.usda"))
    names = {p.GetName() for p in stage.Traverse()}
    for name in ("Riser", "CutMat", "TrayFloor", "TrayNear", "RestMat"):
        assert any(n.startswith(name) for n in names), name


# -- demo plan and reach -----------------------------------------------------------------------------------------------
def test_demo_uses_both_hands_with_open_close_and_return():
    plan = T.demo()
    left = [w.label for w in plan["left"]]
    right = [w.label for w in plan["right"]]
    assert any("press the sheet" in s for s in left)
    assert any("grip the handles" in s for s in right)
    assert "open the blades" in right and "close the blades" in right
    assert any("cut along the line" in s for s in right)
    assert "let go" in right and "home" in left and "home" in right


def test_static_reach_at_four_critical_points():
    from yam_arena import ik
    from yam_arena.station import BASES

    def solve(pos, rot, side):
        q4 = ik.matrix_to_quat_xyzw(rot)
        q, pe, re = ik.solve(np.asarray(pos, float), q4, np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]),
                             base_pos=BASES[side], iters=150)
        return pe * 1000.0, math.degrees(re)

    cases = [
        ("rest grasp", T._rest_tcp(), T.CUT_ROT, "right"),
        ("cut start", T._cut_tcp(T.LINE_START), T.CUT_ROT, "right"),
        ("cut end", T._cut_tcp(T.LINE_END - T.BLADE_LEN), T.CUT_ROT, "right"),
        ("left press", T.PRESS_TCP, T.PRESS_ROT, "left"),
    ]
    for name, pos, rot, side in cases:
        pe, re = solve(pos, rot, side)
        assert pe < 1.0 and re < 2.0, (name, pe, re)


# -- geometric predicates ----------------------------------------------------------------------------------------------
def test_left_press_predicate():
    _reset()
    tcp = np.array([T.PRESS_XY[0], T.PRESS_XY[1], T.PAPER_Z + T.PAPER_T / 2 + T.PAD_BELOW_TIP + 0.013])
    o = _objects(left_gripper=_gripper("left", tcp))
    assert T._left_press(o) is True
    o["left_gripper"] = _gripper("left", tcp + np.array([0, 0, 0.05]))
    assert T._left_press(o) is False


def test_on_handles_predicate():
    _reset()
    pivot = np.array([T.LINE_START, T.CUT_Y, T.PAPER_Z], float)
    want = pivot + np.array([-0.074, 0.0, 0.0135])
    o = _objects(scissor_pivot=pivot, right_gripper=_gripper("right", want, opening=0.056, contact=1.2))
    assert T._on_handles(o) is True
    o["right_gripper"] = _gripper("right", want + np.array([0.1, 0, 0]), opening=0.056, contact=1.2)
    assert T._on_handles(o) is False


def test_offcut_in_tray_predicate():
    _reset()
    o = _objects(offcut=_offcut(True))
    assert T._in_tray(o) is True
    o = _objects(offcut=_offcut(False))
    assert T._in_tray(o) is False


def test_scissor_back_predicate():
    _reset()
    pivot = np.array([*T.REST_XY, T.REST_Z])
    o = _objects(scissor_pivot=pivot, scissor_bis=np.array([1.0, 0.0]), blade_open=0.35)
    assert T._scissor_back(o) is True
    o = _objects(scissor_pivot=np.array([0.5, 0.2, T.REST_Z]), scissor_bis=np.array([1.0, 0.0]), blade_open=0.35)
    assert T._scissor_back(o) is False
