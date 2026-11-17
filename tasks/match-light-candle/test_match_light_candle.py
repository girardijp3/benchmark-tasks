"""Offline checks (no GPU, no network): the task file loads, the declared strike / flame / burn-down / contact rules
do what the instruction says (a strike needs contact with the strip, >= 0.3 m/s and >= 15 mm of it -- no force threshold;
the wick lights after 2 s in the flame; a flame that reaches a held pinch, a pad within 4 mm of the flame, or a touch of
the candle body caps the score at 0.5; a lit match on the table outside the ashtray or a knocked-over candle scores 0),
the scorer covers each outcome (idle 0, 1.0 only with every condition for 1 s) and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_match_light_candle.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena import ik
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "match_light_candle.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()
    T._fx_init()
    yield


# -- a stand-in environment for the declared effects ----------------------------------------------------------------------
class _Data:
    pass


class FakeEnv:
    def __init__(self):
        self.step_dt = 0.05
        self.episode_length_buf = [0]
        self.poses = {}
        self.push = 0.0
        self.grippers = {}
        self.written = {}

    def advance(self):
        self.episode_length_buf[0] += 1


@pytest.fixture
def env(monkeypatch):
    e = FakeEnv()
    base = {"box": ((0.27, 0.0, T.TOP + 0.13), IDENT), "match": (T.MATCH_START, T._QX90N), "candle": (T.CANDLE_START, IDENT),
            "holder": ((T.HOLDER_XY[0], T.HOLDER_XY[1], T.TOP), IDENT), "ashtray": ((T.ASH_XY[0], T.ASH_XY[1], T.TOP), IDENT)}
    e.poses.update(base)
    monkeypatch.setattr(T, "_root", lambda env_, name: (np.array(e.poses[name][0], float), np.array(e.poses[name][1], float)))
    monkeypatch.setattr(T, "_panel_push", lambda env_: e.push)
    monkeypatch.setattr(T, "_write_pose", lambda env_, ids, name, pos, quat=IDENT: e.written.__setitem__(name, np.array(pos, float)))

    def grips(env_):
        far = T.GripperState(back=np.array([5.0, 5.0, 5.0]), tips=np.array([5.0, 5.0, 4.9]), opening=0.09, contact=0.0)
        return {"left_gripper": e.grippers.get("left", far), "right_gripper": e.grippers.get("right", far)}

    monkeypatch.setattr(T, "_gripper_states", grips)
    return e


QUAT_DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # a match standing head down (z along -z)


def _place_head(e, q_local, push=0.0):
    """Put the match upright head down so that its head is at ``q_local`` (box frame) and set the panel push (m)."""
    box = e.poses["box"]
    Rb = T.quat_xyzw_to_matrix(np.array(box[1], float))
    head = np.array(box[0], float) + Rb @ np.array(q_local, float)
    e.poses["match"] = (head - np.array([0.0, 0.0, T.HZ]) * np.array([0.0, 0.0, -1.0]) * -1.0 + np.array([0.0, 0.0, 0.0]), QUAT_DOWN)
    # a match with z along -z has its head at pos - z*HZ: put the base end HZ above the head
    e.poses["match"] = (head + np.array([0.0, 0.0, T.HZ]), QUAT_DOWN)
    e.push = push


def _strike(e, speed, push, steps=3):
    """Draw the strip under a pressed head: ``steps`` control steps at ``speed`` (m/s) along the strip's long axis with
    the panel pushed in by ``push`` (m)."""
    x0 = 0.022
    face = -(T.BY / 2 + T.PANEL_T) + push
    qy = face - T.M_HR
    dx = speed * e.step_dt
    for k in range(steps + 1):
        _place_head(e, (x0 - k * dx, qy, 0.0), push)
        e.advance()
        T._fx_step(e, None)
    return T.FX["lit"]


def test_strike_lights_the_match_only_when_drawn_across_the_strip(env):
    assert _strike(env, 0.40, 0.0006)
    T._fx_init()
    assert _strike(env, 0.40, 0.0006, steps=1), "one control step at 0.4 m/s is 20 mm of the strip"
    T._fx_init()
    assert not _strike(env, 0.25, 0.0006, steps=3), "too slow"
    T._fx_init()
    assert not _strike(env, 0.40, 0.0, steps=3), "in the air / not pressed on the strip"
    T._fx_init()
    assert not _strike(env, 0.40, 0.00005, steps=3), "a graze below the 0.1 mm contact gate does nothing"
    T._fx_init()
    assert _strike(env, 0.40, 0.00015, steps=3), "a light 0.6 N press is enough: there is no force threshold"
    T._fx_init()
    assert not _strike(env, 0.40, 0.0006, steps=0), "a single touching sample has no stroke"


def test_stroke_must_be_unbroken(env):
    """A step in which the head is off the strip (or not pressed on it) restarts the stroke."""
    _place_head(env, (0.022, -(T.BY / 2 + T.PANEL_T) - T.M_HR, 0.0), 0.0006)
    env.advance()
    T._fx_step(env, None)
    _place_head(env, (0.0, -(T.BY / 2 + T.PANEL_T) - T.M_HR - 0.02, 0.0), 0.0)  # lifted off
    env.advance()
    T._fx_step(env, None)
    _place_head(env, (-0.022, -(T.BY / 2 + T.PANEL_T) - T.M_HR, 0.0), 0.0006)
    env.advance()
    T._fx_step(env, None)
    assert not T.FX["lit"]


def test_head_in_the_air_does_not_light(env):
    for k in range(5):
        _place_head(env, (0.02 - 0.02 * k, -0.06, 0.0), 0.0)
        env.advance()
        T._fx_step(env, None)
    assert not T.FX["lit"]


def test_flame_burns_down_the_stick_and_the_wick_catches_after_two_seconds(env):
    assert _strike(env, 0.40, 0.0006)
    t_ign = T.FX["t_ign"]
    # hold the burning point 3 mm beside and 1 mm under the wick tip
    wick = T.candle_wick(*env.poses["candle"])
    for _ in range(60):
        front = T.V_BURN * (env.episode_length_buf[0] * env.step_dt - t_ign)
        bp = wick + np.array([-0.003, 0.0, -0.001 - T.FLAME_LO])
        # a match standing head down: the burning point is the head plus the front along the stick
        env.poses["match"] = (bp - np.array([0.0, 0.0, front]) + np.array([0.0, 0.0, T.HZ]), QUAT_DOWN)
        env.advance()
        T._fx_step(env, None)
        if T.FX["wick"]:
            break
    assert T.FX["wick"] and T.FX["near_s"] >= T.WICK_S - 1e-6
    assert "flame_c" in env.written


def test_flame_away_from_the_wick_never_lights_it(env):
    assert _strike(env, 0.40, 0.0006)
    for _ in range(60):
        env.poses["match"] = (np.array([0.3, 0.0, T.TOP + 0.2]), QUAT_DOWN)
        env.advance()
        T._fx_step(env, None)
    assert not T.FX["wick"]


def test_flame_reaching_the_held_pinch_burns_the_fingers(env):
    assert _strike(env, 0.40, 0.0006)
    pos = np.array([0.3, 0.0, T.TOP + 0.20])
    env.poses["match"] = (pos, QUAT_DOWN)
    # the pads pinch the stick 8 mm from its base end, as the demo does (pads along the pointing axis, here +y, through the stick)
    base = pos
    tcp = base + np.array([0.0, -0.012, -0.008])
    env.grippers["right"] = T.GripperState(back=tcp - 0.025 * np.array([0.0, 1.0, 0.0]), tips=tcp + 0.013 * np.array([0.0, 1.0, 0.0]), opening=0.0021, contact=4.0)
    for _ in range(int(14 / 0.05)):
        env.advance()
        T._fx_step(env, None)
        if any(k == "burn" for k, _ in T.FX["events"]):
            break
    assert any(k == "burn" for k, _ in T.FX["events"])
    assert T._cap(T.FX["events"]) == 0.5


def test_lit_match_on_the_table_is_a_fire_but_in_the_ashtray_it_is_fine(env):
    assert _strike(env, 0.40, 0.0006)
    env.poses["match"] = (np.array([0.2, -0.1, T.TOP + 0.001]), T._QX90N)
    env.advance()
    T._fx_step(env, None)
    assert any(k == "fire" for k, _ in T.FX["events"])
    T._fx_init()
    assert _strike(env, 0.40, 0.0006)
    env.poses["match"] = (np.array([T.ASH_XY[0], T.ASH_XY[1], T.TOP + 0.007]), T._QX90N)
    env.advance()
    T._fx_step(env, None)
    assert not T.FX["events"]


def test_a_knocked_over_candle_is_a_fire(env):
    a = math.radians(40) / 2
    env.poses["candle"] = (T.CANDLE_START, np.array([math.sin(a), 0.0, 0.0, math.cos(a)]))
    env.advance()
    T._fx_step(env, None)
    assert any(k == "fire" for k, _ in T.FX["events"])


def test_touching_the_candle_body_caps_at_half(env):
    """The match stick must not touch the wax body; only its flame may come near the wick."""
    assert _strike(env, 0.40, 0.0006)
    env.poses["match"] = (np.array([T.HOLDER_XY[0], T.HOLDER_XY[1], T.TOP + T.FLOOR_Z + 0.12]), IDENT)
    env.advance()
    T._fx_step(env, None)
    assert any(k == "touch" for k, _ in T.FX["events"])
    assert T._cap(T.FX["events"]) == 0.5


def test_a_pad_within_four_mm_of_the_match_flame_caps_at_half(env):
    """The real-trial heat shield stands in this gap; the pad itself must stay clear of the flame."""
    assert _strike(env, 0.40, 0.0006)
    pos = np.array([0.30, 0.0, T.TOP + 0.20])
    env.poses["match"] = (pos, QUAT_DOWN)
    head = T.match_geo(pos, QUAT_DOWN)[0]
    env.grippers["right"] = T.GripperState(back=head + np.array([0.0, 0.0, 0.030]), tips=head + np.array([0.0, 0.0, 0.002]),
                                           opening=0.05, contact=0.0)
    for _ in range(2):
        env.advance()
        T._fx_step(env, None)
    assert any(k == "scorch" for k, _ in T.FX["events"])
    assert T._cap(T.FX["events"]) == 0.5


# -- scoring --------------------------------------------------------------------------------------------------------------
def _fx(**kw):
    d = {"lit": False, "lit_ever": False, "spent": False, "wick": False, "near_ever": False, "near_s": 0.0, "events": [], "front": 0.0, "push": 0.0, "speed": 0.0,
         "stroke": 0.0, "dist": 9.9}
    d.update(kw)
    return d


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening, contact=contact)


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


HOME = {s: _gripper(T._START_TCP[s]) for s in ("left", "right")}


def objects(t=0.0, fx=None, match_in_tray=False, box_down=True, **over):
    o = {"box": _state(T.BOX_START if box_down else (0.27, 0.0, T.TOP + 0.13)), "candle": _state(T.CANDLE_START),
         "holder": _state((T.HOLDER_XY[0], T.HOLDER_XY[1], T.TOP)), "ashtray": _state((T.ASH_XY[0], T.ASH_XY[1], T.TOP)),
         "match": _state(T.MATCH_START, T._QX90N) if not match_in_tray else _state((T.ASH_XY[0], T.ASH_XY[1] + T.M_L / 2, T.TOP + 0.007), T._QX90N),
         "left_gripper": HOME["left"], "right_gripper": HOME["right"], "episode_time": t, "fx": fx or _fx()}
    o.update(over)
    return o


def test_idle_scores_zero():
    for t in (0.0, 5.0, 60.0):
        assert T.score(objects(t)) == 0.0 and not T.success(objects(t))


def test_ladder_points():
    assert T.score(objects(1.0, _fx(lit_ever=True, lit=True))) == 0.25
    assert T.score(objects(2.0, _fx(lit_ever=True, near_ever=True))) == 0.5
    assert T.score(objects(3.0, _fx(lit_ever=True, near_ever=True, wick=True))) == 0.75


FULL = dict(lit_ever=True, near_ever=True, wick=True, spent=False)


def test_full_success_needs_the_second_and_every_condition():
    seq = [T.score(objects(t, _fx(**FULL), match_in_tray=True)) for t in (10.0, 10.5, 10.9, 11.0, 11.4)]
    assert seq[:3] == [0.75] * 3 and seq[3] == 1.0, seq
    T._PROGRESS.__init__()
    assert T.score(objects(0.0, _fx(**FULL), match_in_tray=True)) < 1.0
    # the match not in the tray, the box up, a hand holding the match, the candle moved, the wick not lit: never 1.0
    for kw in (dict(match_in_tray=False), dict(match_in_tray=True, box_down=False)):
        T._PROGRESS.__init__()
        assert all(T.score(objects(t, _fx(**FULL), **kw)) < 1.0 for t in np.arange(0.0, 4.0, 0.5))
    T._PROGRESS.__init__()
    moved = {"candle": _state((T.HOLDER_XY[0] + 0.03, T.HOLDER_XY[1], T.TOP + T.FLOOR_Z))}
    assert all(T.score(objects(t, _fx(**FULL), match_in_tray=True, **moved)) < 1.0 for t in np.arange(0.0, 4.0, 0.5))
    T._PROGRESS.__init__()
    holding = {"right_gripper": _gripper((T.ASH_XY[0], T.ASH_XY[1] + T.M_L / 2, T.TOP + 0.02), contact=4.0, opening=0.0021)}
    assert all(T.score(objects(t, _fx(**FULL), match_in_tray=True, **holding)) < 1.0 for t in np.arange(0.0, 4.0, 0.5))


def test_burn_caps_at_half_and_blocks_success():
    fx = _fx(events=[("burn", "x")], **FULL)
    assert all(T.score(objects(t, fx, match_in_tray=True)) == 0.5 for t in np.arange(0.0, 5.0, 0.5))
    assert not T.success(objects(5.0, fx, match_in_tray=True))


def test_fire_scores_zero_for_the_rest_of_the_run():
    fx = _fx(events=[("fire", "x")], **FULL)
    assert T.score(objects(1.0, fx)) == 0.0
    assert T.score(objects(2.0, _fx(**FULL), match_in_tray=True)) <= 0.75


def test_one_second_hold_restarts_when_a_part_moves():
    assert T.score(objects(10.0, _fx(**FULL), match_in_tray=True)) == 0.75
    assert T.score(objects(10.6, _fx(**FULL), match_in_tray=True)) == 0.75
    shifted = {"box": _state((T.BOX_START[0] + 0.01, T.BOX_START[1], T.BOX_START[2]))}
    assert T.score(objects(11.2, _fx(**FULL), match_in_tray=True, **shifted)) == 0.75
    assert T.score(objects(11.8, _fx(**FULL), match_in_tray=True, **shifted)) == 0.75
    assert T.score(objects(12.4, _fx(**FULL), match_in_tray=True, **shifted)) == 1.0


# -- the demo -------------------------------------------------------------------------------------------------------------
def test_demo_reaches_every_waypoint_and_is_inside_the_run_limit():
    d = T.demo()
    lens = {}
    for side in ("left", "right"):
        plan = ArmPlan(d[side], BASES[side])
        lens[side] = sum(w.duration for w in d[side])
        q = None
        worst = 0.0
        for t in np.arange(0.0, lens[side], 0.2):
            pos, rot, _, _ = plan.target(float(t))
            q4 = ik.matrix_to_quat_xyzw(rot)
            seed = np.array([0, 1, 1, -0.6, 0, 0], float) if q is None else q
            q, pe, re = ik.solve(pos, q4, seed, base_pos=BASES[side], iters=100)
            worst = max(worst, pe * 1000.0)
        assert worst < 2.0, (side, worst)
    assert max(lens.values()) + 0.5 + 3.0 <= 620 / 20.0 + 1.0


def test_match_on_the_bare_table_never_scores_even_if_the_free_ashtray_is_moved():
    """The ashtray is free: the match must be in the bowl where the ashtray really is, not where it started."""
    moved = {"ashtray": _state((T.ASH_XY[0] + 0.15, T.ASH_XY[1] + 0.1, T.TOP))}
    assert all(T.score(objects(t, _fx(**FULL), match_in_tray=True, **moved)) < 1.0 for t in np.arange(0.0, 4.0, 0.5))
    T._PROGRESS.__init__()
    on_table = {"match": _state((T.ASH_XY[0], T.ASH_XY[1] + T.M_L / 2, T.TOP + 0.001), T._QX90N), "ashtray": _state((T.ASH_XY[0] + 0.15, T.ASH_XY[1], T.TOP))}
    assert all(T.score(objects(t, _fx(**FULL), **on_table)) < 1.0 for t in np.arange(0.0, 4.0, 0.5))


def test_contact_must_hold_over_two_samples(env):
    """A single pressed sample (one 50 ms step) is not a strike: contact must be there on the step before too."""
    x0, push = 0.022, 0.0006
    qy = -(T.BY / 2 + T.PANEL_T) + push - T.M_HR
    _place_head(env, (x0, qy, 0.0), 0.0)  # touching but not pressed
    env.advance()
    T._fx_step(env, None)
    _place_head(env, (x0 - 0.02, qy, 0.0), push)
    env.advance()
    T._fx_step(env, None)
    assert not T.FX["lit"]
