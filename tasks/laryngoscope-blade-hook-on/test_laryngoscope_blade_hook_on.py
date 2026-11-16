"""Offline checks (no GPU, no network): the task file loads, the hook-on window (bar in the slot, in line, flat), the declared hinge
(the bar stays in the slot at every swing angle), the detent and its release, the unhooking pull, the step ladder, the 1 s
full-success timing, the latches (a part off the table) and the demo's reach.
Run in the yam_arena environment:  python -m pytest -q test_laryngoscope_blade_hook_on.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "laryngoscope_blade_hook_on.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _yaw(a):
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def blade_pose(phi_deg, depth=None, lat=0.0, yaw_extra=0.0, dz=0.0, handle=None):
    """The blade lying on the tray swung ``phi_deg`` about the bar with the bar ``depth`` into the slot (default home)."""
    d = T.PIVOT if depth is None else depth
    h = T.HANDLE_START if handle is None else handle
    bar = np.asarray(h, float) + T.BAR_LOC
    rb = T._rz(math.radians(phi_deg + yaw_extra) - math.pi / 2)
    pos = bar + np.array([0.0, 0.0, T.BLADE_DZ + dz]) - rb @ np.array([d, lat, 0.0])
    return (pos, T._mat_quat(rb))


START = {"handle": (np.array(T.HANDLE_START, float), IDENT), "blade": (np.array(T.BLADE_START, float), np.array(T.BLADE_QUAT, float))}
SOLVED = {"handle": START["handle"], "blade": blade_pose(90.0)}
NEAR_TCP = np.array(SOLVED["blade"][0], float) + np.array([0.06, 0.0, 0.03])
OFF_TABLE = (np.array([0.3, 0.0, T.TOP - 0.06]), IDENT)
RULE_CASES = [
    lambda e: {"handle": OFF_TABLE},  # the handle falls off the table
    lambda e: {"blade": OFF_TABLE},  # the blade falls off the table
]


# -- shared checks ----------------------------------------------------------------------------------------------------
DOWN = np.array([0.0, 0.0, -1.0])
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()
    yield


def _gripper(tcp, contact=0.0, opening=0.09, axis=DOWN):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


class Episode:
    """A made-up episode: the parts at ``START`` (or ``SOLVED``), both grippers at home; ``step`` advances 0.05 s ticks."""

    def __init__(self, poses=None):
        self.t = 0.0
        self.poses = {n: (np.array(p, float), np.array(q, float)) for n, (p, q) in (poses or START).items()}
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {n: _state(p, q) for n, (p, q) in self.poses.items()}
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            else:
                self.poses[k] = (np.array(v[0], float), np.array(v[1], float))
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def done(self):
        return T.steps_done(self.objects())


def test_loads():
    assert TASK.module is T
    assert hasattr(T, "demo") and hasattr(T, "score") and hasattr(T, "success")


def test_start_scores_zero():
    assert Episode().step(1.0) == (0.0, False)


def test_solved_succeeds_after_one_second():
    e = Episode(SOLVED)
    score, ok = e.step(0.5)
    assert score == 0.8 and not ok  # all four steps done, the second not yet up: below 1.0
    score, ok = e.step(1.0)
    assert score == 1.0 and ok


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode(SOLVED)
    for _ in range(19):
        score, ok = e.step()
        assert score < 1.0 and not ok


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S >= 1.0 and T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def test_a_lapse_restarts_the_second():
    e = Episode(SOLVED)
    e.step(0.8)
    name = T.MOVERS[0]
    p, q = e.poses[name]
    e.step(0.1, **{name: (p + np.array([0.004, 0.0, 0.0]), q)})  # moved 4 mm: the count starts over
    score, ok = e.step(0.8)
    assert score < 1.0 and not ok
    score, ok = e.step(1.0)
    assert ok


def test_a_gripper_still_near_a_part_is_not_clear():
    e = Episode(SOLVED)
    e.step(0.3, right=_gripper(NEAR_TCP))
    assert e.done()[-1] is False
    assert e.step(3.0)[0] < 1.0


def test_not_finite_scores_zero():
    e = Episode(SOLVED)
    p, q = e.poses[T.MOVERS[0]]
    assert e.step(0.1, **{T.MOVERS[0]: (np.array([np.nan, 0, 0]), q)})[0] == 0.0


@pytest.mark.parametrize("case", range(len(RULE_CASES)))
def test_a_rule_event_scores_zero_for_the_rest_of_the_run(case):
    e = Episode(SOLVED)
    e.step(0.3)
    e.step(0.1, **RULE_CASES[case](e))
    assert e.step(0.1)[0] == 0.0
    e.step(0.1, **SOLVED)  # put right again
    score, ok = e.step(3.0)
    assert score == 0.0 and not ok  # the event stays


@pytest.mark.parametrize("side", ["left", "right"])
def test_demo_is_reachable(side):
    plan = ArmPlan(T.demo()[side], BASES[side])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    assert max(sum(w.duration for w in d[s]) for s in d) < 150.0


@pytest.fixture(autouse=True)
def fresh_hinge():
    T._HINGE.__init__()
    yield


class Episode(Episode):  # noqa: F811
    def __init__(self, poses=None):
        super().__init__(poses)
        if poses is SOLVED:  # hooked on and clicked in earlier
            T._HINGE.hooked, T._HINGE.locked, T._HINGE.phi = True, True, math.pi / 2


def _hooked(e, phi=0.0, **kw):
    e.step(0.05, blade=blade_pose(phi, **kw))
    return e


def test_the_start_is_not_hooked():
    e = Episode()
    e.step(0.5)
    assert e.done() == [False] * 4 and not T._HINGE.hooked


def test_bar_in_the_slot_hooks_the_blade():
    e = _hooked(Episode())
    assert T._HINGE.hooked and e.done()[0] is True and e.done()[1] is False


@pytest.mark.parametrize("kw", [dict(depth=2.0 * T.MM), dict(lat=2.0 * T.MM), dict(yaw_extra=12.0), dict(dz=6.0 * T.MM), dict(depth=9.0 * T.MM)])
def test_a_bar_outside_the_hook_window_does_not_hook(kw):
    e = _hooked(Episode(), **kw)
    assert not T._HINGE.hooked and e.done()[0] is False


def test_a_tilted_blade_does_not_hook():
    e = Episode()
    p, q = blade_pose(0.0)
    h = math.radians(8.0) / 2
    qt = np.array([math.sin(h), 0.0, 0.0, math.cos(h)])
    q2 = np.array([q[3] * qt[0] + q[0] * qt[3] + q[1] * qt[2] - q[2] * qt[1], q[3] * qt[1] - q[0] * qt[2] + q[1] * qt[3] + q[2] * qt[0],
                   q[3] * qt[2] + q[0] * qt[1] - q[1] * qt[0] + q[2] * qt[3], q[3] * qt[3] - q[0] * qt[0] - q[1] * qt[1] - q[2] * qt[2]])
    e.step(0.05, blade=(p, q2 / np.linalg.norm(q2)))
    assert not T._HINGE.hooked


@pytest.mark.parametrize("phi", [0.0, 20.0, 45.0, 70.0, 86.0])
def test_the_declared_hinge_keeps_the_bar_in_the_slot_at_every_angle(phi):
    e = Episode()
    e.step(0.05, blade=blade_pose(0.0))
    # the hand swings the blade but drags it 3 mm out of place: the hinge puts it back on the bar
    p, q = blade_pose(phi, depth=T.PIVOT - 0.0)
    e.step(0.05, blade=(p + np.array([0.002, 0.001, 0.0]), q))
    pos, quat = T._HINGE.target
    objs = {"handle": T.ObjectState(pos=np.array(T.HANDLE_START, float), quat_xyzw=IDENT, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3)),
            "blade": T.ObjectState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))}
    depth, lat, _ = T.bar_in_blade(objs)
    assert depth == pytest.approx(T.PIVOT, abs=1e-6) and abs(lat) < 1e-6
    assert math.degrees(T.swing_of(objs)) == pytest.approx(phi, abs=0.01)


def test_swing_steps():
    e = _hooked(Episode(), 0.0)
    e.step(0.05, blade=blade_pose(30.0))
    assert e.done() == [True, False, False, False]
    e.step(0.05, blade=blade_pose(60.0))
    assert e.done() == [True, True, False, False]
    assert not T._HINGE.locked


def test_the_detent_clicks_in_from_87_degrees_and_lights_the_lamp():
    e = _hooked(Episode(), 0.0)
    e.step(0.05, blade=blade_pose(86.0))
    assert not T._HINGE.locked and not T._HINGE.lit()
    e.step(0.05, blade=blade_pose(88.0))
    assert T._HINGE.locked and T._HINGE.lit() and e.done()[2] is True
    assert math.degrees(T._HINGE.phi) == pytest.approx(90.0)


def test_the_blade_cannot_swing_past_the_detent_or_back_past_in_line():
    e = _hooked(Episode(), 0.0)
    e.step(0.05, blade=blade_pose(-8.0))
    assert T._HINGE.phi == 0.0
    e.step(0.05, blade=blade_pose(100.0))
    assert math.degrees(T._HINGE.phi) == pytest.approx(90.0)


def test_a_small_push_does_not_free_the_detent_but_a_pull_does():
    e = _hooked(Episode(), 0.0)
    e.step(0.05, blade=blade_pose(90.0))
    assert T._HINGE.locked
    for _ in range(20):
        e.step(0.05, blade=blade_pose(88.0))  # 2 degrees of push-back, however long
    assert T._HINGE.locked
    e.step(0.05, blade=blade_pose(85.0))
    assert not T._HINGE.locked and not T._HINGE.lit()


def test_a_hooked_blade_in_line_comes_off_only_by_a_real_pull():
    e = _hooked(Episode(), 0.0)
    for _ in range(10):
        e.step(0.05, blade=blade_pose(0.0, depth=T.PIVOT - 0.5 * T.MM))  # half a millimetre of give: stays
    assert T._HINGE.hooked
    for _ in range(3):
        e.step(0.05, blade=blade_pose(0.0, depth=T.PIVOT - 2.0 * T.MM))
    assert not T._HINGE.hooked and e.done()[0] is False


def test_a_swung_blade_is_kept_on_the_bar_by_the_hook():
    e = _hooked(Episode(), 0.0)
    e.step(0.05, blade=blade_pose(40.0))
    for _ in range(10):
        e.step(0.05, blade=blade_pose(40.0, depth=T.PIVOT - 3.0 * T.MM))
    assert T._HINGE.hooked


def test_a_swung_blade_released_keeps_its_angle_without_a_lit_lamp():
    e = _hooked(Episode(), 0.0)
    e.step(1.0, blade=blade_pose(60.0))
    assert not T._HINGE.lit() and e.done()[3] is False


def test_full_score_needs_the_blade_in_the_detent_not_just_near_it():
    e = Episode(SOLVED)
    e.step(0.2)
    e.step(0.2, blade=blade_pose(80.0))  # pulled out of the detent: not lit, not 1.0
    assert not T._HINGE.locked
    assert e.step(2.0)[0] < 1.0


def test_the_lit_scope_off_the_tray_is_not_done():
    e = Episode(SOLVED)
    p, q = SOLVED["blade"]
    far = (np.array(T.HANDLE_START, float) + np.array([0.0, 0.0, 0.0]), IDENT)
    e.step(0.2, handle=(far[0] + np.array([0.0, 0.5, 0.0]), IDENT))
    assert e.done()[3] is False


def test_the_demo_reaches_the_clicked_position():
    d = T.demo()
    right = d["right"]
    assert any(w.label == "click" for w in right)
    end = [w for w in right if w.label == "click"][0]
    bar = np.array(T.BAR_START, float)
    r = np.hypot(end.pos[0] - bar[0], end.pos[1] - bar[1])
    assert r == pytest.approx(T.RADIUS, abs=0.002)
