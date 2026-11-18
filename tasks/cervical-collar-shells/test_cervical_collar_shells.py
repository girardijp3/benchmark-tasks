"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
front seated, front+back seated, left tab, right tab + the 1 s hold and its restart; every cap (neck moves, tab
miss, forced fit) holds the score at 0.5 and blocks 1.0; the demo fits the run limit, returns home and reaches
every waypoint with the YAM arms.

Run in the yam_arena environment:  python -m pytest -q test_cervical_collar_shells.py
"""
import pathlib

import numpy as np
import pytest
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.task_file import load
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "cervical_collar_shells.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(front=False, back=False, left=False, right=False,
             neck_moves=False, tab_miss=False, forced_fit=False, grippers=None):
    return {
        "front_done": front,
        "back_done": back,
        "left_tab_done": left,
        "right_tab_done": right,
        "neck_moves": neck_moves,
        "tab_miss": tab_miss,
        "forced_fit": forced_fit,
        "depth": {"left": 0.008, "right": 0.008},
        "tilt": {"front": 0.0, "back": 0.0},
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    return _objects(front=True, back=True, left=True, right=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("manikin", "front", "back"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    man = (HERE / "assets" / "manikin.usda").read_text()
    for token in ("BasePlate", "Shoulders", "Neck", "Head", "Chin"):
        assert token in man
    front = (HERE / "assets" / "front.usda").read_text()
    for token in ("ChinCup", "TabL", "TabR"):
        assert token in front
    back = (HERE / "assets" / "back.usda").read_text()
    assert "OccipitalRim" in back


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_front_quarter():
    o = _objects(front=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_front_back_half():
    o = _objects(front=True, back=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_left_tab_three_quarters():
    o = _objects(front=True, back=True, left=True)
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
    broken = _objects(front=True, back=True, left=True)  # right tab slips
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["neck_moves", "tab_miss", "forced_fit"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(front=True, back=True, left=True, forced_fit=True)
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.NECK_XY[0], T.NECK_XY[1], T.TOP + 0.12]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    assert T.NECK_XY == pytest.approx((0.250, 0.0))
    assert T.TAB_Y == pytest.approx(0.050)
    # 30 mm side tabs, 0.3 kg total collar (see REFERENCE.md)
    assert 2 * 0.015 == pytest.approx(0.030)
    assert 0.140 + 0.160 == pytest.approx(0.300)
    assert T.TOP == pytest.approx(0.7475)
    # the press gate is 4-7 N over a 700 N/m compliant tab; every work height is below 0.30 m
    assert (4.0 / T.K_TAB) == pytest.approx(T.DEPTH_SEAT)
    assert (7.0 / T.K_TAB) == pytest.approx(T.DEPTH_FORCE)
    assert T.FRONT_SEAT[2] < 0.30 and T.BACK_SEAT[2] < 0.30


def test_declared_mechanism_numbers():
    assert T.K_TAB == pytest.approx(700.0)
    assert T.PRESS_DEPTH == pytest.approx(0.008)   # 5.6 N, inside 4-7 N
    assert 4.0 < T.K_TAB * T.PRESS_DEPTH < 7.0
    assert T.GRIP_CLOSE == pytest.approx(0.070)
    assert T.GRIP_RELEASE == pytest.approx(0.085)
    assert T.GRASP_UP == pytest.approx(0.010)


def test_both_arms_reach_every_demo_waypoint():
    plan = T.demo()
    for side, wps in plan.items():
        for w in wps:
            rot = w.rot if w.rot is not None else yam_demo.grip_rotation(w.yaw)
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot),
                                 q_init=np.array([0, 1, 1, -0.6, 0, 0]), base_pos=BASES[side], iters=900)
            assert pe < 0.005, (side, w.label, pe, w.pos)
            assert np.degrees(re) < 5.0, (side, w.label, np.degrees(re), w.pos)


def test_demo_fits_limit_and_returns_home():
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for side, wps in plan.items():
        assert np.allclose(wps[-1].pos, T._START_TCP[side])
        assert wps[-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
        assert sum(w.duration for w in wps) < 1200 / 20.0 - 2.0
    # front shell seated and left tab pressed by the left arm; back shell and right tab by the right arm
    ll = [w.label for w in plan["left"]]
    rl = [w.label for w in plan["right"]]
    assert ll.index("release the front shell") < ll.index("press the left tab")
    assert rl.index("release the back shell") < rl.index("press the right tab")
