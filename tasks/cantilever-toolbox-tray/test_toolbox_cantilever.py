"""Offline checks (no GPU, no network): the task file loads; the scoring ladder covers idle, one latch, both latches,
the lid lift, the open trays, the full 1 s hold and its restart; a dropped tool, a pinch, a forced latch and a tipped
box cap or zero the score; the box-stability window; both arms reach every demo waypoint; the demo fits the run limit
and returns home; the asset has the two latches, two arms, two level trays and the lid joint.

Run in the yam_arena environment:  python -m pytest -q test_toolbox_cantilever.py
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
TASK = load(HERE / "toolbox_cantilever.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
BOX = np.array([T.BOX_XY[0], T.BOX_XY[1], T.TOP + 0.001])
BOX0 = (BOX.copy(), IDENT.copy())


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _gripper(side, tcp=None, contact=0.0, opening=0.094):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2], tips=tcp + 3.14e-3 * rot[:, 2],
                          opening=opening, contact=contact, rot=rot)


def _wing(side, open_: bool):
    if side == 1:
        return (_state((T.BOX_XY[0], 0.310, T.TOP + 0.225)) if open_ else _state((T.BOX_XY[0], 0.076, T.TOP + 0.100)))
    return (_state((T.BOX_XY[0], -0.325, T.TOP + 0.217)) if open_ else _state((T.BOX_XY[0], -0.076, T.TOP + 0.100)))


def _open_objects(latch=(-25.0, -25.0), lift=0.115, arm=(-130.0, 135.0), box=BOX, quat=IDENT, dropped=False,
                  pinched=False, forced=False, tipped=False, off_table=False, wrench_pos=None, screw_pos=None,
                  hammer_pos=None, grippers=None):
    wl, wr = _wing(1, True), _wing(-1, True)
    if wrench_pos is None:
        wrench_pos = (T.BOX_XY[0], 0.310 - 0.075, T.TOP + 0.225 + 0.006)
    if screw_pos is None:
        screw_pos = (T.BOX_XY[0], -0.325 + 0.075, T.TOP + 0.217 + 0.054)
    if hammer_pos is None:
        hammer_pos = (T.BOX_XY[0], -0.090, T.TOP + 0.079)
    objs = {
        "toolbox": _state(box, quat),
        "hammer": _state(hammer_pos),
        "wrench": _state(wrench_pos),
        "screwdriver": _state(screw_pos),
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
        "joints": {"ArmL_Swing": math.radians(arm[0]), "WingL_Level": math.radians(-arm[0]),
                   "ArmR_Swing": math.radians(arm[1]), "WingR_Level": math.radians(-arm[1]),
                   "Top_Lift": lift, "LatchL_Swing": math.radians(latch[0]),
                   "LatchR_Swing": math.radians(latch[1])},
        "latch": (math.radians(latch[0]), math.radians(latch[1])),
        "lift": lift,
        "base": (np.asarray(box, float), np.asarray(quat, float)),
        "wingl": (wl.pos, wl.quat_xyzw),
        "wingr": (wr.pos, wr.quat_xyzw),
        "top": (np.array([T.BOX_XY[0], 0.0, T.TOP + T.LID_BOT + lift]), IDENT.copy()),
        "latchl": (np.array([T.BOX_XY[0] - 0.114, T.LATCH_KNOB[1][1], T.TOP + 0.195]), IDENT.copy()),
        "latchr": (np.array([T.BOX_XY[0] - 0.114, T.LATCH_KNOB[-1][1], T.TOP + 0.195]), IDENT.copy()),
        "handle_bar": np.array([T.BOX_XY[0] - 0.060, 0.0, T.TOP + 0.216 + lift]),
        "knobs": (np.array([T.BOX_XY[0] - 0.122, 0.160, T.TOP + 0.165]),
                  np.array([T.BOX_XY[0] - 0.122, -0.160, T.TOP + 0.165])),
        "box0": (np.asarray(box, float), np.asarray(quat, float)),
        "dropped": dropped,
        "pinched": pinched,
        "forced": forced,
        "tipped": tipped,
        "off_table": off_table,
    }
    return objs


def _closed_objects():
    return _open_objects(latch=(0.0, 0.0), lift=0.0, arm=(0.0, 0.0), box=BOX)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for token in ("ArmL_Swing", "WingL_Level", "ArmR_Swing", "WingR_Level", "Top_Lift", "LatchL_Swing",
                  "LatchR_Swing", "LatchKnob", "WingLTray", "WingRTray", "LidKeeper"):
        assert token in (HERE / "assets" / "toolbox.usda").read_text()
    for name in ("hammer", "wrench", "screwdriver"):
        assert (HERE / "assets" / f"{name}.usda").is_file()


def test_idle_scores_zero():
    objs = _closed_objects()
    assert T.steps_done(objs) == (False, False, False, False)
    assert T.score(objs) == 0.0
    assert T.success(objs) is False


def test_one_latch_only_scores_zero():
    objs = _open_objects(latch=(-25.0, 0.0))
    assert T.steps_done(objs)[0] is False
    assert T.score(objs) == 0.0


def test_both_latches_quarter():
    objs = _open_objects(latch=(-25.0, -25.0), lift=0.0, arm=(0.0, 0.0))
    assert T.steps_done(objs)[0] is True
    assert T.score(objs) == pytest.approx(0.25)


def test_lid_lifted_half():
    objs = _open_objects(latch=(-25.0, -25.0), lift=0.04, arm=(0.0, 0.0))
    assert T.steps_done(objs)[:2] == (True, True)
    assert T.score(objs) == pytest.approx(0.5)


def test_open_trays_three_quarters():
    objs = _open_objects()
    assert T.steps_done(objs) == (True, True, True, True)
    assert T.tools_in_place(objs) is True
    assert T.score(objs) == pytest.approx(0.75)
    assert T.success(objs) is False


def test_one_second_full_hold():
    objs = _open_objects()
    for _ in range(20):
        assert T.success(objs) is False
    assert T.success(objs) is True
    assert T.score(objs) == pytest.approx(1.0)


def test_hold_restarts_after_a_break():
    objs = _open_objects()
    for _ in range(10):
        T.success(objs)
    broken = _open_objects(lift=0.04, arm=(0.0, 0.0))
    assert T.success(broken) is False
    for _ in range(20):  # a fresh full second is needed, not the earlier 0.5 s
        assert T.success(objs) is False
    assert T.success(objs) is True


def test_dropped_tool_caps_at_half():
    objs = _open_objects(wrench_pos=(T.BOX_XY[0] + 0.20, 0.10, T.TOP + 0.012), dropped=True)
    assert T.tools_in_place(objs) is False
    assert T.score(objs) <= 0.5
    assert T.success(objs) is False


def test_a_tool_out_of_its_tray_is_detected():
    objs = _open_objects(wrench_pos=(T.BOX_XY[0], 0.310 - 0.075, T.TOP + 0.400))  # 170 mm above the tray
    assert T.tools_in_place(objs) is False


def test_pinch_caps_at_half():
    objs = _open_objects(pinched=True)
    assert T.steps_done(objs)[2] is True
    assert T.score(objs) <= 0.5
    assert T.success(objs) is False


def test_forced_latch_caps_at_half():
    objs = _open_objects(forced=True)
    assert T.score(objs) <= 0.5
    assert T.success(objs) is False


def test_tipped_box_scores_zero():
    objs = _open_objects(tipped=True)
    assert T.score(objs) == 0.0
    assert T.success(objs) is False


def test_stability_window():
    # a 3 deg tilt is past the 2 deg final window but not past the 5 deg tip latch
    c, s = math.cos(math.radians(1.5)), math.sin(math.radians(1.5))
    q = np.array([s, 0.0, 0.0, c])
    tilted = _open_objects(quat=q)
    assert T._tilt_deg(tilted) == pytest.approx(3.0, abs=0.05)
    assert T._stable(tilted) is False
    assert T.score(tilted) == pytest.approx(0.75)
    # a 12 mm slide is past the 10 mm final window and under the 30 mm tip latch
    moved = _open_objects(box=BOX + np.array([0.012, 0.0, 0.0]))
    moved["box0"] = (BOX.copy(), IDENT.copy())
    assert T._moved(moved) == pytest.approx(0.012, abs=1e-6)
    assert T._stable(moved) is False


def test_grippers_not_clear_keeps_the_fourth_step_false():
    g = _gripper("left", tcp=(T.BOX_XY[0] - 0.060, 0.0, T.TOP + 0.216))
    objs = _open_objects(grippers={"left": g})
    assert T._clear(objs) is False
    assert T.steps_done(objs)[3] is False
    assert T.success(objs) is False


def test_latch_and_lift_thresholds():
    assert T.LATCH_OPEN == pytest.approx(math.radians(-18.0))
    assert T.LIFT_START == pytest.approx(0.030)
    assert T.LIFT_OPEN == pytest.approx(0.105)
    assert T.ARM_L_PART == pytest.approx(math.radians(-120.0))
    assert T.ARM_R_PART == pytest.approx(math.radians(130.0))


def test_box_dimensions_and_masses():
    assert (T.Y_LEN, T.X_DEP) == pytest.approx((0.460, 0.208))
    assert T.BASE_TOP + 0.103 == pytest.approx(0.208, abs=0.001)  # the box is 208 mm tall
    assert T.BOX_MASS + T.HAMMER_MASS + T.WRENCH_MASS + T.SCREWDRIVER_MASS == pytest.approx(5.74)
    # the real box is 4.8 kg; the model's box is 4.74 kg including the 0.4 kg lid (EST split)
    assert 4.5 <= T.BOX_MASS <= 4.9


def test_both_arms_reach_every_demo_waypoint():
    plan = T.demo()
    for side, wps in plan.items():
        for w in wps:
            rot = w.rot if w.rot is not None else yam_demo.grip_rotation(w.yaw)
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot),
                                 q_init=np.array([0, 1, 1, -0.6, 0, 0]), base_pos=BASES[side], iters=900)
            assert pe < 0.005, (side, w.label, pe, w.pos)
            assert math.degrees(re) < 5.0, (side, w.label, math.degrees(re), w.pos)


def test_demo_fits_the_limit_and_returns_home():
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for side, wps in plan.items():
        assert np.allclose(wps[-1].pos, T._START_TCP[side])
        assert wps[-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
        assert sum(w.duration for w in wps) < 1200 / 20.0 - 2.0
    # both latches are pulled before the left arm leaves for the handle, and the right is home by then
    left = plan["left"]
    labels = [w.label for w in left]
    assert labels.index("wait for the right latch") > labels.index("pull the latch")
    assert sum(w.duration for w in plan["right"]) < sum(w.duration for w in left) - 2.0


def test_declared_mechanism_numbers():
    assert T.LATCH_PULL == pytest.approx(0.012)
    assert math.degrees(T.ARM_L_OPEN) == pytest.approx(-130.0)
    assert math.degrees(T.ARM_R_OPEN) == pytest.approx(135.0)
    assert T.LATCH_FULL == pytest.approx(math.radians(-25.0))
    assert T.CLEAR_DIST == pytest.approx(0.050)
    assert T.LIFT_MAX == pytest.approx(0.115)
