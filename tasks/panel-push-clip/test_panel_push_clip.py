"""Offline checks (no GPU, no network): the task file loads and its assets exist; the scoring ladder covers idle,
aligned, pressed, barbs seated and the 1 s pull-test hold; every cap (cross-insert, not seated, dropped, forced fit)
holds the score at 0.5 and blocks 1.0; the 1 s pull hold and its restart; grippers clear and home; real product sizes,
the 4-7 N bands; and every demo waypoint is reachable by the right YAM arm.

Run:  python -m pytest -q test_panel_push_clip.py
"""
import pathlib

import numpy as np
import pytest
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "panel_push_clip.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _clip(pos=None):
    pos = T.SEAT_POS.copy() if pos is None else np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=IDENT.copy(), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _objects(aligned=False, pressed=False, seated=False, verified=False,
             cross=False, unseated=False, dropped=False, forced=False,
             clip_pos=None, grippers=None):
    grippers = grippers or {}
    return {
        "clip": _clip(clip_pos),
        "aligned_done": aligned,
        "pressed_done": pressed,
        "seated_done": seated,
        "pull_verified": verified,
        "cross_insert": cross,
        "not_seated": unseated,
        "dropped": dropped,
        "forced_fit": forced,
        "depth": 0.008,
        "pull_depth": 0.016,
        "tilt": 0.0,
        "left_gripper": grippers.get("left") or _gripper("left"),
        "right_gripper": grippers.get("right") or _gripper("right"),
    }


def _done(**kw):
    return _objects(aligned=True, pressed=True, seated=True, verified=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._MECH.reset()
    yield


def test_loads_and_assets():
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("panel", "tray", "clip"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    panel = (HERE / "assets" / "panel.usda").read_text()
    for token in ("Bore", "HoleRim", "PlateYPos", "Screw1"):
        assert token in panel
    clip = (HERE / "assets" / "clip.usda").read_text()
    for token in ("Cap", "Stem", "Tip", "Barb0", "Barb3"):
        assert token in clip
    assert "Tray" in (HERE / "assets" / "tray.usda").read_text() or "Socket" in (HERE / "assets" / "tray.usda").read_text()


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_aligned_quarter():
    o = _objects(aligned=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_aligned_pressed_half():
    o = _objects(aligned=True, pressed=True)
    assert T.steps_done(o) == (True, True, False, False)
    assert T.score(o) == pytest.approx(0.5)


def test_barbs_seated_three_quarters():
    o = _objects(aligned=True, pressed=True, seated=True)
    assert T.steps_done(o) == (True, True, True, False)
    assert T.score(o) == pytest.approx(0.75)
    assert T.success(o) is False


def test_full_state_scores_one():
    o = _done()
    assert T.steps_done(o) == (True, True, True, True)
    assert T.success(o) is True
    assert T.score(o) == pytest.approx(1.0)


def test_success_needs_grippers_clear_and_home():
    g = _gripper("right", tcp=np.array([T.HOLE_XY[0], T.HOLE_XY[1], T.TOP + 0.05]))
    o = _done(grippers={"right": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False
    assert T.score(o) == pytest.approx(1.0)  # the ladder's 4 steps still sum to 1.0; the harness gate is not met


def test_partially_closed_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_done(grippers={"left": g})) is False


def test_clip_pulled_out_is_not_intact():
    o = _done(clip_pos=np.array([T.HOLE_XY[0] + 0.05, T.HOLE_XY[1], T.TOP + 0.003]))
    assert T._clip_intact(o) is False
    assert T.success(o) is False


@pytest.mark.parametrize("cap", ["cross", "unseated", "dropped", "forced"])
def test_caps_hold_score_at_half(cap):
    o = _done(**{cap: True})
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


@pytest.mark.parametrize("cap", ["cross", "unseated", "dropped", "forced"])
def test_a_cap_fails_the_three_quarter_step(cap):
    o = _objects(aligned=True, pressed=True, seated=True, **{cap: True})
    assert T.score(o) == pytest.approx(0.5)


def test_real_product_sizes():
    # an 8 mm nylon panel push-clip: 20 mm cap, 8 mm stem, ~18 mm stem, 2 g (see REFERENCE.md)
    assert T.STEM_R == pytest.approx(0.004)
    assert 2 * T.CAP_R == pytest.approx(0.020)
    assert T.CLIP_MASS == pytest.approx(0.002)
    assert T.TOP == pytest.approx(0.7475)
    assert T.HOLE_XY == pytest.approx((0.280, 0.0))
    assert T.SEAT_CAP_Z < 0.30  # work height below the 0.30 m limit


def test_press_and_pull_force_bands():
    # both the insertion and the retention pull use a declared 350 N/m travel, 4-7 N band
    assert T.K_CLIP == pytest.approx(350.0)
    assert T.K_PULL == pytest.approx(350.0)
    assert 4.0 < T.K_CLIP * (T.PRESS_REF_Z - T.SEAT_CAP_Z) < 7.0
    assert 4.0 < T.K_PULL * 0.016 < 7.0
    assert T._in_pull_band(0.016) is True
    assert T._in_pull_band(0.030) is False
    assert T._in_pull_band(0.002) is False


def test_pull_hold_needs_one_second():
    hold, done = 0.0, False
    for _ in range(19):
        hold, done = T._advance_pull_hold(hold, 0.05, 0.016, True, True)
        assert done is False
    hold, done = T._advance_pull_hold(hold, 0.05, 0.016, True, True)
    assert done is True
    assert hold == pytest.approx(1.0)


def test_pull_hold_restarts_after_a_break():
    hold = 0.0
    for _ in range(15):
        hold, _ = T._advance_pull_hold(hold, 0.05, 0.016, True, True)
    hold, done = T._advance_pull_hold(hold, 0.05, 0.016, False, True)  # gripper slips
    assert hold == 0.0 and done is False
    for _ in range(19):
        hold, done = T._advance_pull_hold(hold, 0.05, 0.016, True, True)
        assert done is False
    _, done = T._advance_pull_hold(hold, 0.05, 0.016, True, True)
    assert done is True


def test_out_of_band_or_unseated_pull_does_not_advance():
    assert T._advance_pull_hold(0.5, 0.05, 0.030, True, True) == (0.0, False)
    assert T._advance_pull_hold(0.5, 0.05, 0.016, True, False) == (0.0, False)


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
    assert set(plan) == {"right"}
    wps = plan["right"]
    assert np.allclose(wps[-1].pos, T._START_TCP["right"])
    assert wps[-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
    assert sum(w.duration for w in wps) < 1000 / 20.0 - 2.0
    labels = [w.label for w in wps]
    assert labels.index("align the stem with the hole") < labels.index("press with 4-7 N into the hole")
    assert labels.index("press the barbs home, cap flush") < labels.index("pull up on the cap")
    assert labels.index("hold the pull for 1 s") < labels.index("home")
