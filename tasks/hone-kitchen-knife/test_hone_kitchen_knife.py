"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
angle set, heel-to-tip sweep, both sides + clean edge, knife parked + the 1 s hold and its restart; every cap
(wrong angle, heavy press, dropped knife) holds the score at 0.5 and blocks 1.0; the demo fits the run limit,
returns home, stays within YAM reach and holds a real ~20 deg light-pressure geometry.

Run:  python -m pytest -q test_hone_kitchen_knife.py
"""
import math
import pathlib

import numpy as np
import pytest
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.task_file import load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "hone_kitchen_knife.py")
T = TASK.module


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = T._DOWN
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 0.013 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(angle_set=False, sweep_a=False, clean=False, parked=False, grasped_ever=True,
             wrong_angle=False, heavy_press=False, dropped=False, grippers=None):
    return {
        "angle_set": angle_set, "sweep_a": sweep_a, "clean": clean, "parked": parked,
        "grasped_ever": grasped_ever, "wrong_angle": wrong_angle, "heavy_press": heavy_press,
        "dropped": dropped,
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    return _objects(angle_set=True, sweep_a=True, clean=True, parked=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("knife", "steel", "rest"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    knife = (HERE / "assets" / "knife.usda").read_text()
    steel = (HERE / "assets" / "steel.usda").read_text()
    for token in ("Blade", "EdgeBevel", "HandleCore", "RivetA", "Bolster"):
        assert token in knife
    for token in ("Rod", "HoneSurface", "Groove", "Handle", "HandleClamp", "TipRest"):
        assert token in steel


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_angle_set_quarter():
    o = _objects(angle_set=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_sweep_half():
    o = _objects(angle_set=True, sweep_a=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_both_sides_three_quarters():
    o = _objects(angle_set=True, sweep_a=True, clean=True)
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
    broken = _objects(angle_set=True, sweep_a=True, clean=True)  # knife not parked
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["wrong_angle", "heavy_press", "dropped"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(angle_set=True, sweep_a=True, clean=True, wrong_angle=True)
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([0.250, -0.150, T.TOP + 0.028]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    # 200 mm chef's knife and 300 mm steel (16 mm rod), real size throughout
    assert T.STEEL_ORIGIN == pytest.approx((0.250, -0.150, 0.7755))
    assert 2 * T.STEEL_R == pytest.approx(0.016)
    assert T.ROD_LEN == pytest.approx(0.200)
    assert T.ROD_LEN + 0.100 == pytest.approx(0.300)  # 200 mm rod + 100 mm handle = 300 mm steel
    assert T.EDGE_PROBE[0] == pytest.approx(0.0)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_mechanism_numbers():
    assert math.degrees(T.HONE_ANGLE) == pytest.approx(20.0)
    assert math.degrees(T.ANGLE_WINDOW) == pytest.approx(5.0)
    assert math.degrees(T.ANGLE_CAP) == pytest.approx(10.0)
    assert T.PRESS_TARGET == pytest.approx(0.0015)
    assert T.PRESS_LIGHT_MAX == pytest.approx(0.003)
    assert T.PRESS_HEAVY == pytest.approx(0.005)
    assert T.PROGRESS_OK == pytest.approx(0.170)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)


def test_hone_target_is_twenty_degrees_with_light_pressure():
    for side in (1, -1):
        origin, tcp, rot = T._hone_target(side, T.SWEEP_X0)
        # knife quat = g.rot @ rot_off, with rot_off = DOWN.T at the flat-knife grasp
        knife_rot = rot @ T._DOWN.T
        blade = knife_rot @ np.array([1.0, 0.0, 0.0])
        assert math.degrees(abs(math.atan2(blade[1], blade[0]))) == pytest.approx(20.0, abs=0.2)
        edge = origin + knife_rot @ T.EDGE_PROBE
        assert edge[1] == pytest.approx(T.STEEL_ORIGIN[1], abs=1e-6)
        assert T.ROD_TOP - edge[2] == pytest.approx(T.PRESS_TARGET, abs=1e-6)


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
    labels = [w.label for w in plan["right"]]
    assert labels.index("set 20 deg at the heel, side A") < labels.index("sweep side A 4/4")
    assert labels.index("sweep side A 4/4") < labels.index("set 20 deg at the heel, side B")
    assert labels.index("sweep side B 4/4") < labels.index("park the knife")
    assert labels.index("park the knife") < labels.index("home")
