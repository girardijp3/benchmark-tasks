"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, the 1 s full-success
timing, and the demo's poses are reachable. Run in the yam_arena environment:  python -m pytest -q test_flask_stopper_match.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena import ik
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "flask_stopper_match.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
NAMES = ("flask_big", "flask_small", "stopper_big", "stopper_small")


def _gripper(tcp, axis=(0.0, 0.0, -1.0), contact=0.0, opening=0.09):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening, contact=contact)


HOME = {"left_gripper": _gripper(T._START_TCP["left"]), "right_gripper": _gripper(T._START_TCP["right"])}


def _root(pos, quat=IDENT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos.copy(), quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def seat_pos(key, mm=0.0):
    s = T.BIG if key == "big" else T.SMALL
    x, y = T.FLASK_XY[key]
    return np.array([x, y, T.TOP + s["rim"] + mm * T.MM])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos = {"flask_big": np.array([*T.FLASK_XY["big"], T.TOP]), "flask_small": np.array([*T.FLASK_XY["small"], T.TOP]),
                    "stopper_big": T.STOPPERS["big"].pos and np.array(T.STOPPERS["big"].pos),
                    "stopper_small": np.array(T.STOPPERS["small"].pos)}
        self.quat = {n: IDENT.copy() for n in NAMES}
        self.left, self.right = HOME["left_gripper"], HOME["right_gripper"]

    def objects(self):
        o = {n: _root(self.pos[n], self.quat[n]) for n in NAMES}
        o.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return o

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
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


def both_seated():
    ep = fresh()
    ep.step(stopper_big=seat_pos("big", 0.3), stopper_small=seat_pos("small", 0.3))
    return ep


def test_nothing_scores_zero():
    assert fresh().step(seconds=1.0) == 0.0


def test_big_stopper_alone_is_one_step():
    ep = fresh()
    assert ep.step(stopper_big=seat_pos("big", 0.3)) == pytest.approx(0.2)


def test_small_stopper_alone_is_one_step():
    ep = fresh()
    assert ep.step(stopper_small=seat_pos("small", 0.3)) == pytest.approx(0.2)


def test_stopper_resting_above_the_rim_is_not_seated():
    ep = fresh()
    assert ep.step(stopper_big=seat_pos("big", 4.0)) == 0.0


def test_stopper_off_axis_is_not_seated():
    ep = fresh()
    assert ep.step(stopper_big=seat_pos("big", 0.3) + np.array([0.003, 0.0, 0.0])) == 0.0


def test_stopper_tilted_is_not_seated():
    ep = fresh()
    h = math.radians(5.0) / 2
    assert ep.step(stopper_big=seat_pos("big", 0.3), quat_stopper_big=np.array([math.sin(h), 0, 0, math.cos(h)])) == 0.0


def test_both_seated_with_grippers_close_is_three_steps():
    ep = both_seated()
    near = _gripper(seat_pos("big", 10.0) + np.array([0.0, 0.0, 0.01]), contact=3.0, opening=0.04)
    assert ep.step(left=near, seconds=1.5) == pytest.approx(0.6)


def test_wrong_stopper_down_the_big_neck_scores_zero():
    ep = both_seated()
    ep.step(stopper_small=seat_pos("big", -20.0))
    assert ep.step() == 0.0


def test_big_stopper_on_small_flask_gets_nothing():
    ep = fresh()
    assert ep.step(stopper_big=seat_pos("small", 4.0)) == 0.0


def test_full_success_needs_one_second():
    ep = both_seated()
    assert ep.step(seconds=0.5) < 1.0
    assert ep.step(seconds=0.4) < 1.0
    assert ep.step(seconds=0.1) == 1.0
    assert T.SUCCESS_HOLD_S == 0.0


def test_step_points_stop_below_one():
    ep = both_seated()
    assert ep.step(seconds=0.5) == pytest.approx(0.8)


def test_motion_restarts_the_second():
    ep = both_seated()
    ep.step(seconds=0.8)
    ep.step(stopper_big=ep.pos["stopper_big"] + np.array([0.0, 0.0, 0.0008]))  # head creeps up 0.8 mm (still seated)
    ep.step(flask_big=ep.pos["flask_big"] + np.array([0.003, 0.0, 0.0]), stopper_big=ep.pos["stopper_big"] + np.array([0.003, 0.0, 0.0]))
    assert ep.step(seconds=0.5) < 1.0
    assert ep.step(seconds=0.6) == 1.0


def test_fallen_flask_scores_zero_even_after_recovery():
    ep = both_seated()
    ep.step(quat_flask_small=np.array([0.0, math.sin(math.radians(30)), 0.0, math.cos(math.radians(30))]))
    assert ep.step() == 0.0
    ep.step(quat_flask_small=IDENT)
    assert ep.step(seconds=2.0) == 0.0


def test_stopper_off_the_table_scores_zero():
    ep = both_seated()
    ep.step(stopper_big=np.array([0.3, 0.0, T.TOP - 0.1]))
    assert ep.step() == 0.0


def test_gripper_on_a_flask_blocks_success():
    ep = both_seated()
    on = _gripper(seat_pos("small", 20.0), contact=3.0, opening=0.02)
    ep.step(left=on)
    assert ep.step(seconds=2.0) < 1.0


@pytest.mark.parametrize("side", ["left", "right"])
def test_demo_waypoints_are_reachable(side):
    rng = np.random.default_rng(0)
    seeds = [np.array([0, 1, 1, -0.6, 0, 0], float)] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(15)]
    for w in TASK.module.demo()[side]:
        rot = w.rot if getattr(w, "rot", None) is not None else None
        best = 1e9
        for s in seeds:
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot), s, base_pos=BASES[side], iters=150)
            best = min(best, pe * 1000 + math.degrees(re))
            if best < 1.0:
                break
        assert best < 2.0, (side, w.label, best)


def test_demo_plans_build():
    plans = TASK.module.demo()
    assert set(plans) == {"left", "right"}
    for side, wps in plans.items():
        assert ArmPlan(wps, BASES[side]).duration > 10.0
