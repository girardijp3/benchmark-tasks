"""Offline checks (no GPU, no network): the task file loads; the scorer covers idle, one axis only, both axes within
0.5 deg, level but not yet let go, the 1 s full-score hold, the bubble window, an over-tightened foot and a foot off
its thread; nudge starts score 0 at the start and 0.75 when level; both arms reach the wheels at the nominal and
nudged poses; the demo's strokes and timing fit the run limit; and the balance asset has the two front feet on slide
joints, the two wheels on revolute joints and the fixed rear foot.

Run in the yam_arena environment:  python -m pytest -q test_balance_level_feet.py
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
TASK = load(HERE / "balance_level_feet.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])
BODY = np.array([T.BAL_XY[0], T.BAL_XY[1], T.TOP + 0.040])


def _quat(pitch_deg=0.0, roll_deg=0.0):
    """A quaternion for Ry(pitch) @ Rx(roll) (the body's z axis tilted by the two angles the scorer reads)."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return ik.matrix_to_quat_xyzw(Ry @ Rx)


def _state(pos=BODY, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _gripper(side, contact=0.0, opening=0.094, tcp=None):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2], tips=tcp + 3.14e-3 * rot[:, 2],
                          opening=opening, contact=contact, rot=rot)


def _objects(pitch=0.0, roll=0.0, bubble=None, e=(T.FOOT_NOM, T.FOOT_NOM), damaged=False, tipped=False,
             off_table=False, left=None, right=None):
    q = _quat(pitch, roll)
    bx, by = T._bubble_xy(q)
    return {
        "balance": _state(BODY, q),
        "bubble": _state(BODY + np.array([-0.160, -0.073, 0.0784]), q),
        "foot_ext": tuple(e),
        "wheel_angle": (0.0, 0.0),
        "tilt": (pitch, roll),
        "bubble_mm": (bx * 1000, by * 1000) if bubble is None else bubble,
        "damaged": damaged,
        "tipped": tipped,
        "off_table": off_table,
        "left_gripper": left or _gripper("left"),
        "right_gripper": right or _gripper("right"),
    }


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    assert hasattr(T, "demo") and hasattr(T, "score") and hasattr(T, "success")
    text = (HERE / "assets" / "balance_aux220.usda").read_text()
    for token in ("FootL_Slide", "FootR_Slide", "WheelL_Spin", "WheelR_Spin", "RearPad", "VialGlass",
                  "WheelCol"):
        assert token in text
    assert (HERE / "assets" / "level_bubble.usda").is_file()


def test_idle_scores_zero():
    for pitch, roll in ((0.55, 0.55), (0.8, 0.7), (1.0, 1.0)):
        objects = _objects(pitch, roll, e=(T.E_L0, T.E_R0))
        assert T.score(objects) == 0.0
        assert T.success(objects) is False


def test_one_foot_only_scores_a_quarter():
    # the left foot lowered to level, the right still high: pitch is inside 0.5 deg, roll is not
    objects = _objects(0.20, -0.73, e=(T.FOOT_NOM, T.E_R0))
    assert T.score(objects) == pytest.approx(0.25)
    assert T.success(objects) is False
    # and one foot alone cannot give more than 0.25 unless it happens to bring both axes in
    objects = _objects(0.45, -0.45, e=(T.FOOT_NOM, T.E_R0))
    assert T.score(objects) == pytest.approx(0.5)


def test_half_degree_window():
    # extensions consistent with the tilted body: the three pads stay on the bench
    assert T.score(_objects(0.45, 0.45, e=(0.0430, 0.0415))) == pytest.approx(0.5)
    assert T.score(_objects(0.51, 0.45, e=(0.0435, 0.0415))) == pytest.approx(0.25)


def test_level_scores_three_quarters_until_held_and_clear():
    objects = _objects(0.05, -0.04)
    assert T.score(objects) == pytest.approx(0.75)
    assert T.success(objects) is False
    # while a gripper is on a wheel it is not a full success
    wheel = T._wheel_center(BODY, _quat(0.05, -0.04), 1, T.FOOT_NOM)
    held = _gripper("left", contact=20.0, opening=0.034, tcp=wheel)
    assert T._clear(_objects(0.05, -0.04, left=held)) is False
    assert T.success(_objects(0.05, -0.04, left=held)) is False


def test_one_second_hold_gives_full_score():
    objects = _objects(0.03, 0.02)
    assert T.success(objects) is False
    for _ in range(19):
        assert T.success(objects) is False
    assert T.success(objects) is True


def test_stillness_restarts_the_second():
    objects = _objects(0.03, 0.02)
    for _ in range(10):
        T.success(objects)
    moved = _objects(0.05, 0.02)
    assert T.success(moved) is False
    for _ in range(20):
        T.success(moved)
    assert T.success(moved) is True


def test_bubble_window():
    # 4 mm per degree: one axis at 0.1 deg is 0.4 mm; both axes at 0.1 deg is 0.57 mm, over the 0.5 mm window
    assert T.score(_objects(0.08, 0.08)) == pytest.approx(0.75)
    assert T.score(_objects(0.09, 0.09)) == pytest.approx(0.5)
    assert T.score(_objects(0.09, 0.09, bubble=(0.9, 0.9))) == pytest.approx(0.5)
    bx, by = T._bubble_xy(_quat(0.1, 0.0))
    assert math.hypot(bx, by) * 1000 == pytest.approx(0.40, abs=0.01)


def test_over_tightened_foot_caps_at_half():
    # 55 mm extension: 15 mm above nominal, past the 12 mm up-travel
    objects = _objects(0.02, 0.02, e=(0.055, T.FOOT_NOM), damaged=True)
    assert T.score(objects) == pytest.approx(0.5)
    assert T.success(objects) is False
    assert T.E_MAX == 0.052 and T.E_MIN == 0.030


def test_foot_off_its_thread_caps_at_half():
    # the low foot is retracted into the air, so the body is not standing on three feet: 0 is allowed, but no 1.0
    objects = _objects(0.02, 0.02, e=(0.025, T.FOOT_NOM), damaged=True)
    assert T.score(objects) <= 0.5
    assert T.success(objects) is False


def test_off_table_and_tip_score_zero():
    assert T.score(_objects(0.02, 0.02, off_table=True)) == 0.0
    assert T.score(_objects(0.02, 0.02, tipped=True)) == 0.0


def test_foot_off_the_bench_scores_zero():
    # the left front foot is 5 mm short: the body could show a level pose on two feet, but it is not standing on three
    assert T._feet_down(_objects(0.0, 0.0, e=(0.035, T.FOOT_NOM))) is False
    assert T.score(_objects(0.0, 0.0, e=(0.035, T.FOOT_NOM))) == 0.0
    assert T.success(_objects(0.0, 0.0, e=(0.035, T.FOOT_NOM))) is False
    assert T._feet_down(_objects(0.0, 0.0)) is True


def test_nudge_starts_all_level_at_the_same_target():
    # The level state is the same for every start: both front extensions equal the 40 mm rear foot.  The scorer is
    # state-based, so a nudge changes only the start's score, never the level's.
    for e_l, e_r in ((0.0430, 0.0428), (0.0450, 0.0438), (0.0418, 0.0404), (0.0445, 0.0415)):
        # the start's pitch/roll from the three support heights (the rear foot is 40 mm)
        pitch = math.degrees(math.atan(((e_l + e_r) / 2 - T.FOOT_NOM) / (abs(T.FRONT_X) + T.REAR_X)))
        roll = math.degrees(math.atan((e_l - e_r) / (2 * T.FRONT_Y)))
        start = _objects(pitch, roll, e=(e_l, e_r))
        assert T.score(start) == 0.0 if abs(pitch) > 0.5 and abs(roll) > 0.5 else True
        level = _objects(0.0, 0.0, e=(T.FOOT_NOM, T.FOOT_NOM))
        assert T.score(level) == pytest.approx(0.75)
    # the demo's own turns reach the 40 mm target from the nominal start
    assert T.E_L0 - T.PITCH * T.TURN_L / 360.0 == pytest.approx(T.FOOT_NOM, abs=1e-9)
    assert T.E_R0 - T.PITCH * T.TURN_R / 360.0 == pytest.approx(T.FOOT_NOM, abs=1e-9)


def test_both_arms_reach_the_wheels_and_nudges():
    for side, s in (("left", 1), ("right", -1)):
        yaws = (-45.0, 0.0, 45.0) + tuple(T.GRIP_YAW[s])
        for dx in (-0.006, 0.0, 0.006):
            for dz in (-0.004, 0.0, 0.004):
                for yaw in yaws:
                    p = (T.WHEEL_X + dx, s * T.FRONT_Y, T.WHEEL_TOP + dz)
                    q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(T._grip_frame(yaw)), q_init=np.array([0, 1, 1, -0.6, 0, 0]),
                                         base_pos=BASES[side], iters=500)
                    assert pe < 0.004 and math.degrees(re) < 4.0, (side, dx, dz, yaw, pe, math.degrees(re))


def test_demo_plan_fits_the_limit_and_returns_home():
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for side, wps in plan.items():
        assert wps[0].rot is not None
        assert np.allclose(wps[-1].pos, T._START_TCP[side])
        duration = sum(w.duration for w in wps)
        assert duration < 2400 / 20.0 - 2.0
    left = plan["left"]
    right = plan["right"]
    # the left wheel needs 802.3 deg = six 120 deg arcs and an 82.3 deg trim; the right 452.6 = five 90 deg arcs
    assert sum(1 for w in left if w.label.startswith("turn the wheel")) == 7
    assert sum(1 for w in right if w.label.startswith("turn the wheel")) == 5


def test_declared_mechanism_numbers():
    assert T.PITCH == pytest.approx(0.00175)
    assert T.FOOT_NOM == pytest.approx(0.040)
    assert T.WHEEL_R == pytest.approx(0.017)
    assert T.BUBBLE_FULL == pytest.approx(0.0005)
    assert T.WINDOW_FULL == 0.1 and T.WINDOW_PART == 0.5
    # six threads from nominal to the 52 mm upper limit; the wheel's 26 mm height is the real knob
    assert T.E_MAX - T.E_MIN == pytest.approx(0.022)
