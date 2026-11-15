"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, the 1 s full-success
timing, and the demo's poses are reachable. Run in the yam_arena environment:  python -m pytest -q test_tagged_mixer_cart.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena import ik
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "tagged_mixer_cart.py")
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


def tray_pos():
    return np.array([*T.CART_XY, T.TOP + T.DECK])


def start(name):
    return np.array([*T.MIXER_XY[name], T.TOP + T.REST_GAP])


def hand(name, z=0.03):
    """A left gripper pinching ``name`` at base height z above its start."""
    p = start(name) + np.array([0.0, 0.0, z + 0.057])
    return _gripper(p, contact=18.0, opening=0.07)


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos = {n: start(n) for n in T.NAMES}
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


def done_ep():
    ep = fresh()
    up = start(T.RED) + np.array([0.0, 0.0, 0.03])
    ep.step(**{T.RED: up}, left=hand(T.RED))
    ep.step(**{T.RED: tray_pos()}, left=HOME["left_gripper"])
    return ep


def test_nothing_scores_zero():
    assert fresh().step(seconds=1.0) == 0.0


def test_lifted_in_a_gripper_is_step_one():
    ep = fresh()
    assert ep.step(**{T.RED: start(T.RED) + np.array([0.0, 0.0, 0.03])}, left=hand(T.RED)) == pytest.approx(0.2)


def test_lifted_without_a_grip_is_nothing():
    ep = fresh()
    assert ep.step(**{T.RED: start(T.RED) + np.array([0.0, 0.0, 0.03])}) == 0.0


def test_slid_into_the_tray_without_lifting_gets_nothing_for_it():
    ep = fresh()
    assert ep.step(**{T.RED: tray_pos()}) == 0.0


def test_on_the_tray_with_the_gripper_still_near_is_three_steps():
    ep = fresh()
    ep.step(**{T.RED: start(T.RED) + np.array([0.0, 0.0, 0.03])}, left=hand(T.RED))
    near = _gripper(tray_pos() + np.array([0.0, 0.0, 0.06]), contact=0.0, opening=0.09)
    assert ep.step(**{T.RED: tray_pos()}, left=near, seconds=1.0) == pytest.approx(0.6)


def test_red_unit_tipped_on_the_tray_is_not_standing():
    ep = done_ep()
    ep.step(quat_mixer_b=np.array([math.sin(math.radians(15)), 0, 0, math.cos(math.radians(15))]))
    assert ep.step(seconds=1.5) < 1.0


def test_moved_working_unit_blocks_step_three():
    ep = done_ep()
    ep.step(mixer_a=start("mixer_a") + np.array([0.0, 0.03, 0.0]))
    assert ep.step(seconds=1.5) <= 0.4 + 1e-9


def test_working_unit_on_the_cart_scores_zero():
    ep = done_ep()
    ep.step(mixer_a=tray_pos() + np.array([0.0, 0.0, 0.0]))
    assert ep.step() == 0.0


def test_full_success_needs_one_second():
    ep = done_ep()
    assert ep.step(seconds=0.5) < 1.0
    assert ep.step(seconds=0.4) < 1.0
    assert ep.step(seconds=0.1) == 1.0
    assert T.SUCCESS_HOLD_S == 0.0


def test_step_points_stop_below_one():
    ep = done_ep()
    assert ep.step(seconds=0.5) == pytest.approx(0.8)


def test_motion_restarts_the_second():
    ep = done_ep()
    ep.step(seconds=0.8)
    ep.step(**{T.RED: ep.pos[T.RED] + np.array([0.003, 0.0, 0.0])})
    assert ep.step(seconds=0.5) < 1.0
    assert ep.step(seconds=0.6) == 1.0


def test_fallen_unit_scores_zero_even_after_recovery():
    ep = done_ep()
    ep.step(quat_mixer_c=np.array([0.0, math.sin(math.radians(30)), 0.0, math.cos(math.radians(30))]))
    assert ep.step() == 0.0
    ep.step(quat_mixer_c=IDENT)
    assert ep.step(seconds=2.0) == 0.0


def test_unit_off_the_table_scores_zero():
    ep = done_ep()
    ep.step(mixer_c=np.array([0.3, -0.3, T.TOP - 0.1]))
    assert ep.step() == 0.0


def test_gripper_near_a_unit_blocks_success():
    ep = done_ep()
    ep.step(left=_gripper(tray_pos() + np.array([0.0, 0.0, 0.05]), contact=0.0, opening=0.09))
    assert ep.step(seconds=2.0) < 1.0


def test_demo_waypoints_are_reachable():
    rng = np.random.default_rng(0)
    seeds = [np.array([0, 1, 1, -0.6, 0, 0], float)] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(15)]
    for w in TASK.module.demo()["left"]:
        best = 1e9
        for s in seeds:
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(w.rot), s, base_pos=BASES["left"], iters=150)
            best = min(best, pe * 1000 + math.degrees(re))
            if best < 1.0:
                break
        assert best < 2.0, (w.label, best)


def test_demo_plan_builds():
    plans = TASK.module.demo()
    assert ArmPlan(plans["left"], BASES["left"]).duration > 10.0
