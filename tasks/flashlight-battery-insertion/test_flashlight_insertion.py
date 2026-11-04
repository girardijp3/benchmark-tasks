"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_flashlight_insertion.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "flashlight_insertion.py")
T = TASK.module

HEAD_DOWN = np.array(T.HEAD_DOWN_XYZW)
NUB_UP = np.array([0.0, 0.0, 0.0, 1.0])  # the cell's frame unturned: its nub toward +z, the tail of the head-down body
BODY0 = np.array([*T.WORK_XY, T.TOP + 0.0495])
DOWN = np.array([0.0, 0.0, -1.0])  # the head-down flashlight's axis, tail to head
CELL_IN = BODY0 + T.CELL_AT * DOWN
CAP_SEATED = BODY0 + T.CAP_AT * DOWN
CELL_LYING = np.array([*T.CELL_XY, T.CELL_Z])
CAP_LYING = np.array([*T.CAP_XY, T.TOP + 0.0105])
OPEN = T.GripperState(back=np.array([0.2, 0.5, 1.2]), tips=np.array([0.2, 0.5, 1.15]), opening=0.09, contact=0.0)
HALF = {"flashlight_body": 0.055, "flashlight_cap": 0.0105, "flashlight_battery": 0.033}


def _turned(deg):
    """Head-down, then turned ``deg`` about the vertical (positive: the cap's tightening direction)."""
    a = math.radians(-deg) / 2
    x1, y1, z1, w1 = 0.0, 0.0, math.sin(a), math.cos(a)
    x2, y2, z2, w2 = HEAD_DOWN
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _state(name, pos, quat):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - HALF[name], high=pos + HALF[name])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos = {"flashlight_body": BODY0, "flashlight_battery": CELL_LYING, "flashlight_cap": CAP_LYING}
        self.quat = {"flashlight_body": HEAD_DOWN, "flashlight_battery": T.LYING_XYZW, "flashlight_cap": HEAD_DOWN}

    def objects(self, right=None):
        out = {n: _state(n, self.pos[n], self.quat[n]) for n in self.pos}
        out["left_gripper"] = OPEN
        if right:
            p = np.asarray(self.pos[right], float)
            out["right_gripper"] = T.GripperState(back=p - (0, 0.03, 0), tips=p + (0, 0.01, 0), opening=0.018,
                                                  contact=10.0)
        else:
            out["right_gripper"] = OPEN
        out["episode_time"] = self.t
        return out

    def step(self, seconds=0.05, right=None, **moved):
        for k, v in moved.items():
            name, what = k.rsplit("__", 1) if "__" in k else (k, "pos")
            getattr(self, what)[name] = v
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects(right)
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def done(self):
        return T.steps_done(self.objects())

    def put_cell_in(self, quat=HEAD_DOWN):
        self.step(right="flashlight_battery", flashlight_battery=CELL_LYING + (0, 0, 0.05))
        self.step(right="flashlight_battery", flashlight_battery=CELL_IN + (0, 0, 0.03),
                  flashlight_battery__quat=quat)
        self.step(flashlight_battery=CELL_IN)

    def screw_cap(self, degrees=400.0, seat=True):
        top = CAP_SEATED + 0.012 * -DOWN
        for k in range(1, int(degrees // 20) + 1):
            f = min(k * 20 / 400.0, 1.0) if seat else 0.0
            self.step(right="flashlight_cap", flashlight_cap=top + 0.012 * f * DOWN,
                      flashlight_cap__quat=_turned(k * 20))
        self.step()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"flashlight_body", "flashlight_cap", "flashlight_battery"}


def test_start_scores_zero():
    assert Episode().step() == (0.0, False)


def test_battery_picked_up_is_a_quarter():
    e = Episode()
    e.step(right="flashlight_battery", flashlight_battery=CELL_LYING + (0, 0, 0.05))
    assert e.step()[0] == 0.25


def test_battery_in_the_right_way_is_a_half():
    e = Episode()
    e.put_cell_in()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.5


def test_battery_in_the_wrong_way_round_scores_zero():
    e = Episode()
    e.put_cell_in(quat=NUB_UP)
    assert e.step()[0] == 0.0


def test_cap_placed_but_not_turned_is_not_on_the_thread():
    e = Episode()
    e.put_cell_in()
    e.step(flashlight_cap=CAP_SEATED + 0.012 * -DOWN)
    assert e.done()[2] is False


def test_cap_threaded_but_not_down_is_three_quarters():
    e = Episode()
    e.put_cell_in()
    e.screw_cap(degrees=120, seat=False)
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.75


def test_full_course_succeeds():
    e = Episode()
    e.put_cell_in()
    e.screw_cap()
    assert e.step(1.0) == (1.0, True)


def test_cap_screwed_on_an_empty_flashlight_earns_no_cap_steps():
    e = Episode()
    e.screw_cap()
    assert e.done() == [False, False, False, False]


def test_cap_forced_down_without_turning_scores_zero():
    e = Episode()
    e.put_cell_in()
    e.step(flashlight_cap=CAP_SEATED)
    assert e.step()[0] == 0.0


def test_part_off_the_table_scores_zero():
    e = Episode()
    e.put_cell_in()
    assert e.step(flashlight_cap=(0.4, 0.8, T.TOP - 0.3))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
