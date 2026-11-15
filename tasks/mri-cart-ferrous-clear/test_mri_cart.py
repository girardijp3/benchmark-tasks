"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_mri_cart.py"""

import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "mri_cart.py")
T = TASK.module
DOWN = np.array([0.0, 0.0, -1.0])


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


TOTE_AT = {"hemostat": (0.06, -0.005), "radio": (-0.07, 0.07), "oximeter": (-0.06, -0.075)}


class Episode:
    """The parts, the grippers and the clock, as ``object_states`` would give them."""

    def __init__(self):
        self.t = 0.0
        self.pos = {n: np.array([*T.start_xy(n), T.TRAY + T.REST_GAP]) for n in T.RIGID}
        self.quat = {n: np.array(T._yaw_xyzw(T.ON_TRAY[n][2])) for n in T.RIGID}
        self.left = _gripper(T._START_TCP)
        self.right = _gripper(T._START_TCP)

    def to_tote(self, name):
        dx, dy = TOTE_AT[name]
        self.pos[name] = np.array([T.BIN_XY[0] + dx, T.BIN_XY[1] + dy, T.BIN_FLOOR_TOP + 0.0005])
        self.quat[name] = np.array([0.0, 0.0, 0.0, 1.0])

    def objects(self):
        out = {n: _state(self.pos[n], self.quat[n]) for n in T.RIGID}
        out["left_gripper"], out["right_gripper"], out["episode_time"] = self.left, self.right, self.t
        return out

    def tick(self, dt=0.05):
        self.t += dt
        return self.objects()


@pytest.fixture(autouse=True)
def fresh():
    T._PROGRESS.__init__()
    yield


def _run(ep, seconds):
    s = 0.0
    last = None
    while s < seconds - 1e-9:
        last = ep.tick()
        T.score(last)
        s += 0.05
    return last


def _cleared():
    ep = Episode()
    for n in T.UNSAFE:
        ep.to_tote(n)
    return ep


def test_loads_and_names():
    assert TASK.module.SCENE == "station"
    assert set(T.UNSAFE) | set(T.SAFE) == set(T.RIGID)
    assert len(T.STEPS) == 4 and T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def test_start_scores_zero():
    ep = Episode()
    assert T.score(ep.tick()) == 0.0
    assert not any(T.steps_done(ep.objects()))


@pytest.mark.parametrize("name,step", [("hemostat", 0), ("radio", 1), ("oximeter", 2)])
def test_each_item_in_tote_scores_its_step(name, step):
    ep = Episode()
    ep.to_tote(name)
    done = T.steps_done(ep.tick())
    assert done[step] and sum(done) == 1
    assert T.score(ep.objects()) == pytest.approx(0.2)


def test_item_half_over_the_rim_is_not_in():
    ep = Episode()
    ep.to_tote("radio")
    ep.pos["radio"][0] = T.BIN_XY[0] + T.BIN_X / 2 - 0.02  # antenna end through the wall
    assert not T.in_tote(ep.objects(), "radio")


def test_item_on_the_table_beside_the_tote_is_not_in():
    ep = Episode()
    ep.pos["hemostat"] = np.array([T.BIN_XY[0], T.BIN_XY[1] + 0.20, T.TOP + 0.001])
    assert not T.in_tote(ep.objects(), "hemostat")


def test_all_cleared_steps_but_no_hold_yet():
    ep = _cleared()
    done = T.steps_done(ep.tick())
    assert all(done)
    assert T.score(ep.objects()) <= 0.8 + 1e-9


def test_full_success_needs_one_second_still():
    ep = _cleared()
    _run(ep, 0.95)
    assert T.score(ep.objects()) <= 0.8 + 1e-9
    assert not T.success(ep.objects())
    last = _run(ep, 0.15)
    assert T.success(last) and T.score(last) == 1.0


def test_motion_restarts_the_second():
    ep = _cleared()
    _run(ep, 0.6)
    ep.pos["radio"][0] += 0.01
    _run(ep, 0.1)
    ep.pos["radio"][0] += 0.0
    last = _run(ep, 0.5)
    assert not T.success(last)
    last = _run(ep, 0.7)
    assert T.success(last)


def test_safe_item_in_the_tote_blocks_success():
    ep = _cleared()
    ep.pos["forceps"] = np.array([T.BIN_XY[0] - 0.1, T.BIN_XY[1] - 0.09, T.BIN_FLOOR_TOP + 0.0005])
    last = _run(ep, 2.0)
    assert not T.safe_on_cart(last, "forceps")
    assert not T.success(last) and T.score(last) <= 0.6 + 1e-9


def test_safe_item_knocked_over_blocks_success():
    ep = _cleared()
    ep.quat["cup"] = np.array([0.7071, 0.0, 0.0, 0.7071])  # on its side
    last = _run(ep, 2.0)
    assert not T.success(last)


def test_safe_item_moved_a_little_is_fine():
    ep = _cleared()
    ep.pos["tape"][:2] += (0.01, -0.01)
    last = _run(ep, 1.2)
    assert T.success(last)


def test_any_unsafe_item_left_blocks_success():
    for left in T.UNSAFE:
        T._PROGRESS.__init__()
        ep = _cleared()
        ep.pos[left] = np.array([*T.start_xy(left), T.TRAY + T.REST_GAP])
        ep.quat[left] = np.array(T._yaw_xyzw(T.ON_TRAY[left][2]))
        last = _run(ep, 2.0)
        assert not T.success(last) and T.score(last) <= 0.4 + 1e-9


def test_gripper_still_near_blocks_success():
    ep = _cleared()
    tcp = ep.pos["radio"] + (0.0, 0.0, 0.05)
    ep.right = _gripper(tcp, contact=10.0, opening=0.05)
    last = _run(ep, 2.0)
    assert not T.success(last) and not T.steps_done(last)[3]
    ep.right = _gripper(T._START_TCP)
    assert T.success(_run(ep, 1.2))


def test_item_off_the_table_scores_zero():
    ep = _cleared()
    ep.pos["oximeter"] = np.array([0.5, 0.5, T.TOP - 0.5])
    last = _run(ep, 1.5)
    assert T.score(last) == 0.0


def test_pushing_an_item_off_the_cart_onto_the_table_is_not_clearing():
    ep = Episode()
    ep.pos["hemostat"] = np.array([T.CART_XY[0], T.CART_XY[1] + 0.30, T.TOP + 0.001])
    assert not T.steps_done(ep.tick())[0]


@pytest.mark.parametrize("seconds", [0.0, 1.0])
def test_score_after_every_condition_except_stillness_is_below_one(seconds):
    ep = _cleared()
    last = _run(ep, max(seconds, 0.05)) if seconds else ep.tick()
    assert T.score(last) < 1.0 or seconds >= 1.0


def test_demo_reach_right_arm():
    plan = T.demo()["right"]
    arm = ArmPlan(plan, BASES["right"])
    assert len(plan) > 20
    assert T.demo()["left"] == []
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    rng = np.random.default_rng(0)
    seeds = [np.array([0, 1, 1, -0.6, 0, 0], float)] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(15)]
    for w in plan:
        if w.label.startswith("home"):
            continue
        rot = w.rot if getattr(w, "rot", None) is not None else grip_rotation(w.yaw)
        q4 = ik.matrix_to_quat_xyzw(rot)
        best = min(ik.solve(np.asarray(w.pos, float), q4, s, base_pos=BASES["right"], iters=150)[1] for s in seeds)
        assert best < 0.002, w.label
    assert arm is not None
