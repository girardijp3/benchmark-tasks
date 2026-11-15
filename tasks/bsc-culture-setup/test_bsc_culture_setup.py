"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, the 1 s full-success
timing, and the demo's poses are reachable. Run in the yam_arena environment:  python -m pytest -q test_bsc_culture_setup.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena import ik
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "bsc_culture_setup.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(tcp, axis=(0.0, 0.0, -1.0), contact=0.0, opening=0.09):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening, contact=contact)


HOME = {"left_gripper": _gripper(T._START_TCP["left"]), "right_gripper": _gripper(T._START_TCP["right"])}


def _root(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos.copy(), quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def at(name):
    x, y = T.PLAN_XY[name]
    return np.array([x, y, T.TOP + T.REST_GAP])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos = {n: np.array([*T.ITEMS[n][1], T.TOP + T.REST_GAP]) for n in T.NAMES}
        self.quat = {n: IDENT.copy() for n in T.NAMES}
        self.left, self.right = HOME["left_gripper"], HOME["right_gripper"]

    def objects(self):
        o = {n: _root(self.pos[n], self.quat[n]) for n in T.NAMES}
        o.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return o

    def step(self, seconds=0.05, **ch):
        for k, v in ch.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            elif k.startswith("quat_"):
                self.quat[k[5:]] = v
            else:
                self.pos[k] = np.asarray(v, float)
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            T.score(self.objects())
            if self.t >= end - 1e-9:
                break
        return T.score(self.objects())


def fresh():
    T._PROGRESS.__init__()
    return Episode()


def laid_out():
    ep = fresh()
    ep.step(bottle=at("bottle"), tips=at("tips"), pan=at("pan"))
    return ep


def test_nothing_scores_zero():
    assert fresh().step(seconds=1.0) == 0.0


def test_each_item_is_one_step():
    for n in T.NAMES:
        assert fresh().step(**{n: at(n)}) == pytest.approx(0.2)


def test_item_on_the_front_grille_is_not_placed():
    assert fresh().step(bottle=at("bottle") + np.array([-0.06, 0.0, 0.0])) == 0.0


def test_item_over_the_rear_grille_is_not_placed():
    assert fresh().step(tips=at("tips") + np.array([0.05, 0.0, 0.0])) == 0.0


def test_pan_on_the_clean_side_is_not_placed():
    assert fresh().step(pan=np.array([0.425, 0.15, T.TOP + T.REST_GAP])) == 0.0


def test_bottle_on_the_dirty_side_is_not_placed():
    assert fresh().step(bottle=np.array([0.425, -0.08, T.TOP + T.REST_GAP])) == 0.0


def test_tilted_item_is_not_placed():
    h = math.radians(15) / 2
    assert fresh().step(bottle=at("bottle"), quat_bottle=np.array([math.sin(h), 0, 0, math.cos(h)])) == 0.0


def test_items_too_close_do_not_finish():
    ep = fresh()
    ep.step(bottle=np.array([0.425, 0.14, T.TOP + T.REST_GAP]), tips=at("tips"), pan=at("pan"))
    assert ep.step(seconds=2.0) == pytest.approx(0.6)


def test_all_laid_out_but_a_gripper_near_blocks_the_last_step():
    ep = fresh()
    near = _gripper([0.425, 0.06, T.TOP + 0.10], contact=0.0, opening=0.09)
    assert ep.step(bottle=at("bottle"), tips=at("tips"), pan=at("pan"), left=near, seconds=2.0) == pytest.approx(0.6)


def test_full_success_needs_one_second():
    ep = laid_out()
    assert ep.step(seconds=0.5) < 1.0
    assert ep.step(seconds=0.4) < 1.0
    assert ep.step(seconds=0.1) == 1.0
    assert T.SUCCESS_HOLD_S == 0.0


def test_step_points_stop_below_one():
    ep = laid_out()
    assert ep.step(seconds=0.5) == pytest.approx(0.8)


def test_motion_restarts_the_second():
    ep = laid_out()
    ep.step(seconds=0.8)
    ep.step(tips=ep.pos["tips"] + np.array([0.0, 0.003, 0.0]))
    assert ep.step(seconds=0.5) < 1.0
    assert ep.step(seconds=0.6) == 1.0


def test_fallen_item_scores_zero_even_after_recovery():
    ep = laid_out()
    ep.step(quat_pan=np.array([0.0, math.sin(math.radians(30)), 0.0, math.cos(math.radians(30))]))
    assert ep.step() == 0.0
    ep.step(quat_pan=IDENT)
    assert ep.step(seconds=2.0) == 0.0


def test_item_off_the_table_scores_zero():
    ep = laid_out()
    ep.step(tips=np.array([0.3, 0.0, T.TOP - 0.1]))
    assert ep.step() == 0.0


@pytest.mark.parametrize("side", ["left", "right"])
def test_demo_waypoints_are_reachable(side):
    rng = np.random.default_rng(0)
    seeds = [np.array([0, 1, 1, -0.6, 0, 0], float)] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(15)]
    for w in TASK.module.demo()[side]:
        best = 1e9
        for s in seeds:
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(w.rot), s, base_pos=BASES[side], iters=150)
            best = min(best, pe * 1000 + math.degrees(re))
            if best < 1.0:
                break
        assert best < 15.0, (side, w.label, best)


def test_demo_plans_build():
    for side, wps in TASK.module.demo().items():
        assert ArmPlan(wps, BASES[side]).duration > 10.0
