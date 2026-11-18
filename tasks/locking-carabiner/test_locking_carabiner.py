"""Offline checks (no GPU, no network): the task file loads, the scorer covers each rung of the ladder (gate opened,
eye through the loop, gate closed, sleeve locked + pull-test), the 1 s hold and its restart, every cap (gate not
closed, sleeve not locked, dropped), the threading/pull-test geometry, real carabiner size, the YAM reach gate and
the demo's IK.  Run in the yam_arena environment:  python -m pytest -q test_locking_carabiner.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "locking_carabiner.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])


def _q(rot):
    return np.array(T._quat(rot))


def _lying(x, y, dz=0.0, rot=None):
    return (np.array([x, y, T.TOP + T.BOARD_T + T.WIRE_R + dz]), _q(T._R_FLAT if rot is None else rot))


START = {"carabiner": (np.array(T.CARAB_START), _q(T._R_FLAT))}
# A solved pull-test: hooked at the gate end of the loop, gate shut, sleeve locked, right gripper still holding.
SOLVED_X, SOLVED_Y = T.POST_XY[0] - 0.046, T.POST_XY[1] + 0.008
SOLVED = {"carabiner": _lying(SOLVED_X, SOLVED_Y, dz=T.LIFT)}
NEAR_TCP = np.array([SOLVED_X, SOLVED_Y, SOLVED["carabiner"][0][2]])


def _off(name):
    p, q = SOLVED[name]
    return (np.array([p[0], p[1], T.TOP - 0.06]), q)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()
    T._LOCK.__init__()
    yield


def _gripper(tcp, contact=0.0, opening=0.09, rot=None):
    tcp = np.asarray(tcp, float)
    if rot is None:
        axis = DOWN
        rot = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    else:
        axis = np.asarray(rot)[:, 2]
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis,
                          opening=opening, contact=contact, rot=np.asarray(rot, float))


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


class Episode:
    """A made-up episode: the parts at ``START`` (or ``SOLVED``), both grippers at home; ``step`` advances 0.05 s ticks."""

    def __init__(self, poses=None, gate=0.0, locked=False, holding=False):
        self.t = 0.0
        self.poses = {n: (np.array(p, float), np.array(q, float)) for n, (p, q) in (poses or START).items()}
        self.left, self.right = _home("left"), _home("right")
        self.gate = gate
        if poses is SOLVED:
            T._PROGRESS.mem["gate_opened"] = True
            T._LOCK.locked = locked
            if holding:
                self.right = _gripper(NEAR_TCP, contact=15.0, opening=0.006)

    def objects(self):
        out = {n: _state(p, q) for n, (p, q) in self.poses.items()}
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t, gate=self.gate)
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            elif k == "gate":
                self.gate = v
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


def solved_episode():
    return Episode(SOLVED, locked=True, holding=True)


# -- shared checks ----------------------------------------------------------------------------------------------------
def test_loads():
    assert TASK.module is T
    assert all(hasattr(T, f) for f in ("demo", "score", "success", "steps_done"))


def test_start_scores_zero():
    assert Episode().step(1.0) == (0.0, False)


def test_solved_succeeds_after_one_second():
    e = solved_episode()
    score, ok = e.step(0.5)
    assert score == 1.0 and not ok  # all four rungs done and taut, but the second is not yet up: below 1.0
    score, ok = e.step(1.0)
    assert score == 1.0 and ok


def test_no_full_score_before_one_second_of_every_condition():
    e = solved_episode()
    for _ in range(19):
        score, ok = e.step()
        assert not ok


def test_success_is_timed_in_the_task_file():
    assert T.STILL_S >= 1.0 and T.POINTS * len(T.STEPS) <= 1.0 + 1e-9


def test_a_lapse_restarts_the_second():
    e = solved_episode()
    e.step(0.8)
    p, q = e.poses["carabiner"]
    e.step(0.1, carabiner=(p + np.array([0.0, 0.005, 0.0]), q))  # 5 mm sideways: still hooked and taut
    score, ok = e.step(0.5)
    assert not ok
    e.step(0.1, carabiner=SOLVED["carabiner"])  # back to the solved pose: the second starts over
    score, ok = e.step(0.6)
    assert not ok
    score, ok = e.step(0.6)
    assert ok


def test_not_finite_scores_zero():
    e = solved_episode()
    assert e.step(0.1, carabiner=(np.array([np.nan, 0, 0]), IDENT))[0] == 0.0


# -- the task's own rungs ---------------------------------------------------------------------------------------------
def test_real_carabiner_size_and_mass():
    assert 0.055 <= 2 * 0.030 <= 0.065        # ~60 mm cross-section outline (the brief's real carabiner)
    assert abs(T.CARAB_MASS - 0.050) < 1e-6    # 50 g: a real compact locking carabiner
    assert T.WIRE_R == 0.003                   # 6 mm wire
    assert np.isclose(T.LOCK_TURN, math.radians(75.0))


def test_gate_open_is_the_first_rung_and_remembered():
    e = Episode()
    p, q = START["carabiner"]
    e.step(0.2, gate=math.radians(-30.0))
    assert e.done()[0] is True and not any(e.done()[1:])
    e.step(0.2, gate=0.0)
    assert e.done()[0] is True


def test_hook_geometry_inside_vs_outside():
    e = Episode()
    p, q = START["carabiner"]
    o = e.objects()
    assert T.threaded(o) is False
    e.step(0.1, carabiner=(np.array([T.POST_XY[0] - 0.028, T.POST_XY[1] + 0.008, o["carabiner"].pos[2]]), q))
    assert e.done()[1] is True


@pytest.mark.parametrize("dx,dy", [(-0.045, 0.0), (0.0, 0.05), (0.0, -0.05), (0.075, 0.0)])
def test_post_outside_the_loop_is_not_threaded(dx, dy):
    e = Episode(SOLVED, locked=True, holding=True)
    p, q = SOLVED["carabiner"]
    e.step(0.1, carabiner=(p + np.array([dx, dy, 0.0]), q))
    assert e.done()[1] is False


def test_carabiner_standing_on_edge_is_not_threaded():
    e = Episode(SOLVED, locked=True, holding=True)
    e.step(0.1, carabiner=(np.array([T.POST_XY[0] - 0.027, T.POST_XY[1], T.TOP + 0.035]), IDENT))
    assert e.done()[1] is False


def test_gate_open_while_hooked_is_not_gate_closed():
    e = Episode(SOLVED, gate=math.radians(-30.0), locked=True, holding=True)
    e.step(0.1)
    assert e.done()[1] is True and e.done()[2] is False


def test_pull_test_needs_taut():
    e = Episode(SOLVED, locked=True, holding=True)
    p, q = SOLVED["carabiner"]
    e.step(0.1, carabiner=(p + np.array([0.020, 0.0, 0.0]), q))  # post back in the middle of the loop
    assert e.done()[3] is False


def test_solved_completes_all_four_steps():
    assert solved_episode().done() == [True, True, True, True]


# -- caps -------------------------------------------------------------------------------------------------------------
def test_drop_scores_zero_for_the_rest_of_the_run():
    e = solved_episode()
    e.step(0.3)
    e.step(0.1, carabiner=_off("carabiner"))
    assert e.step(0.1)[0] == 0.0
    e.step(0.1, **SOLVED)
    assert e.step(3.0)[0] == 0.0


def test_gate_not_closed_cap():
    e = Episode(SOLVED, gate=math.radians(-30.0))  # threaded, gate open, nothing holding it
    e.step(0.2)
    assert e.step(0.1)[0] <= 0.5


def test_sleeve_not_locked_cap():
    e = Episode(SOLVED, locked=False, holding=False)  # pull-test state reached with no lock
    e.step(0.2)
    assert e.step(0.1)[0] <= 0.5


# -- the scripted demo ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("side", ["right"])
def test_demo_is_reachable(side):
    d = T.demo()
    assert d["left"] == []
    plan = ArmPlan(d[side], BASES[side])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    assert max(sum(w.duration for w in d[s]) for s in d) < 150.0
