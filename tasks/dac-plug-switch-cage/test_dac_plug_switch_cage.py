"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
aligned, latched, boot seated and the tug-hold 1 s (with its restart); every cap (wrong port, forced plug, latch
miss, boot not seated, failed tug) holds the score at 0.5 and blocks 1.0; the real plug/cage sizes and the YAM
reach/press gate hold; the demo fits the run limit, returns home and reaches every waypoint with the YAM arms.

Run in the yam_arena environment:  python -m pytest -q test_dac_plug_switch_cage.py
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
TASK = load(HERE / "dac_plug_switch_cage.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(aligned=False, latched=False, boot=False, tug=False, wrong_port=False, forced=False,
             latch_miss=False, boot_missing=False, tug_failed=False, grippers=None):
    return {
        "aligned": aligned, "latched": latched, "boot_seated": boot, "tug_held": tug,
        "wrong_port": wrong_port, "forced": forced, "latch_miss": latch_miss,
        "boot_missing": boot_missing, "tug_failed": tug_failed,
        "depth": T.DEPTH_SEAT, "keyed": True, "held": None,
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    return _objects(aligned=True, latched=True, boot=True, tug=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("switch", "plug"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    sw = (HERE / "assets" / "switch.usda").read_text()
    for token in ("Chassis", "TargetSocket", "DecoySocket", "TargetLatchNotch", "CombTooth0", "CradlePlate"):
        assert token in sw
    pl = (HERE / "assets" / "plug.usda").read_text()
    for token in ("Body", "Contacts", "LatchTab", "BootFlange", "KeyRib"):
        assert token in pl


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_aligned_quarter():
    o = _objects(aligned=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_latched_half():
    o = _objects(aligned=True, latched=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_boot_seated_three_quarters():
    o = _objects(aligned=True, latched=True, boot=True)
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
    broken = _objects(aligned=True, latched=True)  # no boot / tug
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["wrong_port", "forced", "latch_miss", "boot_missing", "tug_failed"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_a_cap_fails_even_the_three_quarter_step():
    o = _objects(aligned=True, latched=True, boot=True, forced=True)
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.SEAT_X, T.TARGET_Y, T.TOP + T.PORT_Z]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    # a 40 mm DAC/SFP-style plug, 13.5 x 8.6 mm body, 12 mm boot, in a 14.5 x 9.5 mm cage
    assert T.PLUG_LEN == pytest.approx(0.040)
    assert T.PLUG_W == pytest.approx(0.0135)
    assert T.PLUG_H == pytest.approx(0.0086)
    assert T.BOOT_W == pytest.approx(0.012)
    assert T.PLUG_LEN * 2 * (T.PLUG_TIP) != 0  # the axis convention is intact
    assert 2 * T.PLUG_TIP == pytest.approx(T.PLUG_LEN)
    assert T.PORT_W == pytest.approx(0.0145)
    assert T.PORT_H == pytest.approx(0.0095)
    assert T.PORT_W > T.PLUG_W and T.PORT_H > T.PLUG_H
    assert T.TARGET_Y == pytest.approx(0.015)
    assert T.DECOY_Y == pytest.approx(-0.015)
    assert T.TOP == pytest.approx(0.7475)


def test_declared_mechanism_numbers():
    assert T.DEPTH_ALIGN == pytest.approx(0.004)
    assert T.DEPTH_LATCH == pytest.approx(0.036)
    assert T.DEPTH_SEAT == pytest.approx(0.040)
    assert T.TUG_PULL == pytest.approx(0.004)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
    assert T.GRASP_UP == pytest.approx(0.012)
    # the plug centre x for each depth follows depth = tip_x - PORT_X
    assert T.ALIGN_X == pytest.approx(T.PORT_X - T.PLUG_TIP + T.DEPTH_ALIGN)
    assert T.LATCH_X == pytest.approx(T.PORT_X - T.PLUG_TIP + T.DEPTH_LATCH)
    assert T.SEAT_X == pytest.approx(T.PORT_X - T.PLUG_TIP + T.DEPTH_SEAT)


def test_yam_reach_press_handling_gate():
    # the work point (x 0.31, y 0.015, h 0.10 above the table) must be reachable and orientable by the left arm
    rot = yam_demo.grip_rotation(0.0)
    p = np.array([T.PORT_X - 0.016, T.TARGET_Y, T.TOP + T.PORT_Z])
    q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(rot), q_init=np.array([0, 1, 1, -0.6, 0, 0]),
                         base_pos=BASES["left"], iters=900)
    assert pe < 0.005, pe
    assert math.degrees(re) < 5.0, math.degrees(re)
    # a real 4-7 N push is applied over the 36 mm stroke; the boot seats after a further 4 mm
    assert T.DEPTH_SEAT > T.DEPTH_LATCH > T.DEPTH_ALIGN > 0


def test_both_arms_reach_every_demo_waypoint():
    plan = T.demo()
    for side, wps in plan.items():
        for w in wps:
            rot = w.rot if w.rot is not None else yam_demo.grip_rotation(w.yaw)
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot),
                                 q_init=np.array([0, 1, 1, -0.6, 0, 0]), base_pos=BASES[side], iters=900)
            assert pe < 0.005, (side, w.label, pe, w.pos)
            assert math.degrees(re) < 8.0, (side, w.label, math.degrees(re), w.pos)


def test_demo_fits_limit_and_returns_home():
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for side, wps in plan.items():
        assert np.allclose(wps[-1].pos, T._START_TCP[side])
        assert wps[-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
        assert sum(w.duration for w in wps) < 1200 / 20.0 - 2.0
    labels = [w.label for w in plan["left"]]
    assert labels.index("align plug at the port") < labels.index("push plug in / latch clicks")
    assert labels.index("push plug in / latch clicks") < labels.index("seat boot in the comb")
    assert labels.index("seat boot in the comb") < labels.index("tug the plug")
