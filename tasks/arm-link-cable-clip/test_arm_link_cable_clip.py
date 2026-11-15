"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_arm_link_cable_clip.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "arm_link_cable_clip.py")
T = TASK.module
SEATED = np.array([T.JIG_XY[0], T.JIG_XY[1], T.TUBE_Z - 0.001])  # the bore's centre with the ring top on the link
START = np.array([*T.CLIP_XY, T.CLIP_Z])
PRELOAD = math.radians(2.2)  # a jaw held open by the link
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _quat_z(deg):
    a = math.radians(deg)
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def _gripper(tcp, contact=0.0, opening=0.09, axis=(0.0, 0.0, -1.0)):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _on_clip(pos, contact=20.0):
    """The right gripper closed on the clip's loop (pad tips 20 mm above the bore's centre)."""
    return _gripper(np.asarray(pos) + np.array([0.0, 0.0, T.GRIP_UP]), contact=contact, opening=0.016)


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos, self.quat = START.copy(), IDENT.copy()
        self.jaw_r, self.jaw_l = 0.0, 0.0
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        clip = ObjectState(pos=self.pos.copy(), quat_xyzw=self.quat.copy(), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                           low=self.pos.copy(), high=self.pos.copy())
        return {"clip": clip, "jaw_r": self.jaw_r, "jaw_l": self.jaw_l, "left_gripper": self.left,
                "right_gripper": self.right, "episode_time": self.t}

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            setattr(self, k, v)
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def scores(self, seconds, **changes):
        for k, v in changes.items():
            setattr(self, k, v)
        out, end = [], self.t + seconds
        while self.t < end - 1e-9:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            out.append((T.score(objects), T.success(objects)))
        return out

    def done(self):
        return T.steps_done(self.objects())

    def pick(self):
        self.step(right=_on_clip(self.pos))
        self.step(pos=self.pos + np.array([0.0, 0.0, 0.03]), right=_on_clip(self.pos + np.array([0.0, 0.0, 0.03])))

    def press_on(self, at=None):
        at = SEATED if at is None else np.asarray(at, float)
        mid = at + np.array([0.0, 0.0, 0.012])
        self.step(pos=mid, jaw_r=-0.24, jaw_l=0.24, right=_on_clip(mid))  # the jaws spread over the link's widest part
        self.step(pos=at, jaw_r=-PRELOAD, jaw_l=PRELOAD, right=_on_clip(at))

    def let_go(self):
        self.step(right=_home("right"))

    def full_course(self):
        self.pick()
        self.press_on()
        self.let_go()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert [o.name for o in TASK.objects] == ["jig", "clip"]


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False] * 4


def test_clip_held_and_lifted_is_a_step():
    e = Episode()
    e.pick()
    assert e.done() == [True, False, False, False]
    assert e.step() == (0.2, False)


def test_holding_the_clip_on_the_table_is_not_a_step():
    e = Episode()
    e.step(right=_on_clip(e.pos))
    assert e.done() == [False] * 4


def test_a_gripper_beside_the_clip_does_not_pick_it_up():
    e = Episode()
    e.step(right=_on_clip(e.pos + np.array([0.08, 0.0, 0.0])))
    e.step(pos=e.pos + np.array([0.0, 0.0, 0.03]), right=_on_clip(e.pos + np.array([0.08, 0.0, 0.0])))
    assert e.done()[0] is False


def test_clip_pressed_home_is_a_step_and_the_jaws_a_second():
    e = Episode()
    e.pick()
    e.press_on()
    assert e.done() == [True, True, True, False]
    assert e.step() == (0.6, False)


def test_jaws_still_spread_are_not_snapped():
    e = Episode()
    e.pick()
    e.press_on()
    e.step(jaw_r=-math.radians(12.0))
    assert e.done() == [True, True, False, False]
    e.step(jaw_r=-PRELOAD, jaw_l=math.radians(12.0))
    assert e.done() == [True, True, False, False]


def test_clip_half_on_is_not_on():
    e = Episode()
    e.pick()
    e.step(pos=SEATED + np.array([0.0, 0.0, 0.010]), jaw_r=-0.24, jaw_l=0.24)
    assert e.done() == [True, False, False, False]


def test_clip_resting_on_top_of_the_link_is_not_on():
    e = Episode()
    e.pick()
    e.step(pos=SEATED + np.array([0.0, 0.0, 0.017]))
    assert e.done() == [True, False, False, False]


def test_clip_off_the_painted_band_is_not_on():
    e = Episode()
    e.pick()
    e.press_on(SEATED + np.array([0.0, 0.010, 0.0]))
    assert e.done()[1] is False


def test_clip_at_the_band_edge_still_counts():
    e = Episode()
    e.pick()
    e.press_on(SEATED + np.array([0.0, 0.005, 0.0]))
    assert e.done()[1] is True


def test_clip_crooked_on_the_link_is_not_on():
    e = Episode()
    e.pick()
    e.step(quat=_quat_z(12.0))
    e.press_on()
    e.step(quat=_quat_z(12.0))
    assert e.done()[1] is False


def test_clip_beside_the_link_is_not_on():
    e = Episode()
    e.pick()
    e.press_on(SEATED + np.array([0.02, 0.0, 0.0]))
    assert e.done()[1] is False


def test_still_holding_the_clip_is_not_let_go():
    e = Episode()
    e.pick()
    e.press_on()
    assert e.done() == [True, True, True, False]


def test_a_gripper_near_the_clip_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(right=_gripper(SEATED + np.array([0.0, 0.0, 0.07])))  # pads within 3 cm of the clip's loop
    assert e.done()[3] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.1) == (1.0, True)


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step() == (1.0, True)


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0


def test_clip_put_on_the_link_without_being_held_scores_nothing():
    """Dropped onto the link, or placed by anything but a gripper: steps 2 to 4 need a gripper to have held it."""
    e = Episode()
    e.step(pos=SEATED, jaw_r=-PRELOAD, jaw_l=PRELOAD)
    history = e.scores(3.0)
    assert e.done() == [False] * 4
    assert all(s == 0.0 and not ok for s, ok in history)


def test_clip_carried_by_pads_that_do_not_press_does_not_count():
    e = Episode()
    e.step(right=_on_clip(e.pos, contact=0.0))
    e.step(pos=e.pos + np.array([0.0, 0.0, 0.03]), right=_on_clip(e.pos + np.array([0.0, 0.0, 0.03]), contact=0.0))
    assert e.done()[0] is False


MISSING = {
    "jaws spread": lambda e: (e.pick(), e.press_on(), e.let_go(), e.step(jaw_r=-math.radians(10.0))),
    "left jaw spread": lambda e: (e.pick(), e.press_on(), e.let_go(), e.step(jaw_l=math.radians(10.0))),
    "still held": lambda e: (e.pick(), e.press_on()),
    "off the band": lambda e: (e.pick(), e.press_on(SEATED + np.array([0.0, 0.012, 0.0])), e.let_go()),
    "resting on top": lambda e: (e.pick(), e.step(pos=SEATED + np.array([0.0, 0.0, 0.017])), e.let_go()),
    "gripper within 3 cm": lambda e: (e.full_course(), e.step(right=_gripper(SEATED + np.array([0.0, 0.0, 0.07])))),
    "clip crooked": lambda e: (e.pick(), e.press_on(), e.let_go(), e.step(quat=_quat_z(12.0))),
}


@pytest.mark.parametrize("missing", sorted(MISSING))
def test_no_full_score_while_any_success_condition_is_missing(missing):
    e = Episode()
    MISSING[missing](e)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history), missing
    assert max(s for s, _ in history) <= 0.8


def test_no_full_score_while_the_clip_keeps_moving():
    e = Episode()
    e.full_course()
    history = []
    for k in range(60):
        sign = 1.0 if k % 2 else -1.0
        e.pos = SEATED + np.array([0.0, 0.0025 * sign, 0.0])
        history.append(e.step())
    assert all(s == 0.8 and not ok for s, ok in history)


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    e.step(0.5)
    e.step(jaw_r=-math.radians(12.0))  # a jaw springs open for a moment
    e.step(jaw_r=-PRELOAD)
    assert e.step(0.9) == (0.8, False)
    assert e.step(0.15) == (1.0, True)


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    e.step(right=_gripper(T._START_TCP["right"] + np.array([0.0, 0.0, 0.2])))
    assert e.step(1.1) == (1.0, True)


def test_clip_pulled_off_again_loses_the_steps():
    e = Episode()
    e.full_course()
    e.step(pos=SEATED + np.array([0.0, 0.0, 0.05]))
    assert e.done() == [True, False, False, False]
    assert e.step(3.0)[0] <= 0.2


def test_clip_off_the_table_scores_zero():
    e = Episode()
    e.full_course()
    e.step(pos=np.array([0.2, -0.2, T.TOP - 0.2]))
    assert e.step() == (0.0, False)


def test_not_finite_scores_zero():
    e = Episode()
    e.full_course()
    e.step(jaw_r=float("nan"))
    assert e.step() == (0.0, False)


def test_demo_is_reachable():
    plan = ArmPlan(T.demo()["right"], BASES["right"])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    assert max(sum(w.duration for w in d[s]) for s in ("left", "right")) < 40.0


def test_closed_fingers_pushing_the_clip_do_not_hold_it():
    e = Episode()
    closed = _gripper(e.pos + np.array([0.0, 0.0, T.GRIP_UP]), contact=20.0, opening=0.0)
    e.step(right=closed)
    e.step(pos=e.pos + np.array([0.0, 0.0, 0.03]), right=closed)
    assert e.done()[0] is False
