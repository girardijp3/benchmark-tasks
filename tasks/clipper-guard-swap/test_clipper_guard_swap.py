"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome and shortcut, and the
demo's poses are reachable. Run in the yam_arena environment:  python -m pytest -q test_clipper_guard_swap.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "clipper_guard_swap.py")
T = TASK.module
DT = 0.05
DEG = math.radians
CQ = T._yaw_xyzw(T.CLIP_YAW)
AXIS = np.array([0.0, 0.0, -1.0])


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * AXIS, tips=tcp + T.PAD_TIP_PAST_TCP * AXIS, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _on_guard(origin, contact=20.0):
    """A gripper pinching a guide whose origin is at ``origin`` (pad span through its middle)."""
    p = T._gw(origin, [T.GRIP_XC, 0.0, 0.012])
    return _gripper(p + (0.0, 0.0, T.PAD_TIP_PAST_TCP), contact=contact, opening=0.046)


class Episode:
    """A fake episode: the scorer sees the clipper, the guides and the grippers one control step at a time."""

    def __init__(self):
        T._PROGRESS.__init__()
        self.t = 0.0
        self.pos = {T.CLIPPER.name: np.array(T.CLIPPER_POS, float)}
        self.quat = {T.CLIPPER.name: CQ}
        for n in T.GUARDS:
            self.pos[T.NAMES[n]] = np.array(T.start_pos(n), float)
            self.quat[T.NAMES[n]] = CQ
        self.clip = {n: 0.0 for n in T.GUARDS}
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        def state(pos, quat):
            pos = np.asarray(pos, float)
            return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                               low=pos.copy(), high=pos.copy())

        out = {k: state(self.pos[k], self.quat[k]) for k in self.pos}
        out["tray"] = state(T.TRAY_POS, (0, 0, 0, 1))
        for n in T.GUARDS:
            out[f"{T.NAMES[n]}_clip"] = self.clip[n]
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return out

    def step(self, seconds=DT, **change):
        """Applies ``change`` and runs ``seconds`` of control steps; returns (score, success) after the last."""
        for k, v in change.items():
            setattr(self, k, v)
        out = None
        for _ in range(max(1, round(seconds / DT))):
            self.t += DT
            o = self.objects()
            out = (T.score(o), T.success(o))
        return out

    def done(self):
        return T.steps_done(self.objects())

    def put(self, n, origin):
        self.pos[T.NAMES[n]] = np.array(origin, float)

    def at_gap(self, gap):
        """The clipper-frame point ``gap`` along the blade from the collar's face."""
        return T._gw(T.CLIPPER_POS, [gap, 0.0, 0.0])

    def remove_two(self):
        self.step(0.3, right=_on_guard(T.CLIPPER_POS))
        self.put(2, self.at_gap(0.06))
        self.step(0.3, right=_on_guard(self.at_gap(0.06)))
        self.put(2, T.slot_origin(2))
        self.step(0.3, right=_on_guard(T.slot_origin(2)))
        self.step(0.3, right=_home("right"))

    def lift_four(self):
        o = T.slot_origin(4) + (0.0, 0.0, 0.02)
        self.step(0.3, right=_on_guard(T.slot_origin(4)))
        self.put(4, o)
        self.step(0.3, right=_on_guard(o))

    def start_four(self, gap=0.025):
        o = self.at_gap(gap)
        self.put(4, o)
        self.step(0.3, right=_on_guard(o))

    def seat_four(self, gap=0.0, clip=0.0):
        o = self.at_gap(gap)
        self.put(4, o)
        self.clip[4] = clip
        self.step(0.3, right=_on_guard(o))

    def let_go(self):
        return self.step(0.5, right=_home("right"), left=_home("left"))

    def full_course(self):
        self.step(0.3, left=_gripper(T._clipper_tcp(), contact=20.0, opening=0.042))
        self.remove_two()
        self.lift_four()
        self.start_four()
        self.seat_four()
        self.let_go()


def test_loads():
    assert TASK.name and [o.name for o in T.OBJECTS][:2] == ["tray", "clipper"]
    assert set(TASK.free_objects) == {"clipper", *T.NAMES.values()}


def test_start_scores_zero():
    assert Episode().step(2.0) == (0.0, False)


def test_removing_the_number_two_is_a_step():
    e = Episode()
    e.remove_two()
    assert e.done() == [True, False, False, False]
    assert e.step()[0] == pytest.approx(0.2)


def test_guide_not_pulled_clear_does_not_count():
    e = Episode()
    e.put(2, e.at_gap(0.03))  # slid out 30 mm: the blade is still inside it
    assert e.done()[0] is False


def test_lifting_the_number_four_is_a_step_only_if_held():
    e = Episode()
    e.put(4, T.slot_origin(4) + (0.0, 0.0, 0.02))
    assert e.done()[1] is False  # lifted by nothing
    e.lift_four()
    assert e.done()[1] is True


def test_number_four_on_the_blade_needs_a_hand_and_alignment():
    e = Episode()
    e.put(4, e.at_gap(0.025))
    assert e.done()[2] is False  # no hand on it
    e = Episode()
    e.step(0.3, right=_on_guard(e.at_gap(0.025)))
    e.put(4, e.at_gap(0.025) + T.CLIP_R @ np.array([0.0, 0.007, 0.0]))  # 7 mm off to the side
    assert e.done()[2] is False
    e = Episode()
    e.start_four()
    assert e.done()[2] is True


def test_not_far_enough_on_the_blade_does_not_count():
    e = Episode()
    e.start_four(gap=0.040)  # only 5 mm of the blade inside
    assert e.done()[2] is False


def test_clipped_is_the_fourth_step_after_the_first_two():
    e = Episode()
    e.remove_two()
    e.lift_four()
    e.start_four()
    e.seat_four()
    assert e.done() == [True, True, True, True]
    assert e.step()[0] == pytest.approx(0.8) or e.step()[0] == 1.0


def test_seated_with_the_hook_still_up_is_not_clipped():
    e = Episode()
    e.remove_two()
    e.lift_four()
    e.start_four()
    e.seat_four(clip=DEG(5))  # hook riding on the ridge
    assert e.done()[3] is False


def test_pushed_on_but_short_of_the_seat_is_not_clipped():
    e = Episode()
    e.remove_two()
    e.lift_four()
    e.start_four()
    e.seat_four(gap=0.006)
    assert e.done()[3] is False


def test_number_two_left_on_the_blade_blocks_the_fit():
    e = Episode()
    e.lift_four()
    e.start_four()
    e.seat_four()  # (number 2 never taken off)
    assert e.done()[3] is False and e.done()[0] is False


def test_wrong_guide_clipped_on_is_not_the_task():
    e = Episode()
    e.remove_two()
    e.step(0.3, right=_on_guard(T.slot_origin(3)))
    e.put(3, e.at_gap(0.0))
    e.step(0.3, right=_on_guard(e.at_gap(0.0)))
    e.let_go()
    assert e.done()[3] is False and e.step(1.5)[1] is False


def test_both_guides_on_the_blade_is_not_success():
    e = Episode()
    e.remove_two()
    e.lift_four()
    e.start_four()
    e.seat_four()
    e.put(2, e.at_gap(0.02))  # the number 2 pushed back over the tip
    assert e.done()[3] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.step(1.1) == (1.0, True)


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode()
    e.step(0.3, left=_gripper(T._clipper_tcp(), contact=20.0, opening=0.042))
    e.remove_two()
    e.lift_four()
    e.start_four()
    e.seat_four()
    e.step(DT, right=_home("right"), left=_home("left"))  # every condition holds from this step on
    assert e.step(T.STILL_S - DT) == (0.8, False)  # held 0.95 s
    assert e.step() == (1.0, True)  # held 1 s


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0


def test_gripper_still_pinching_the_guide_is_not_success():
    e = Episode()
    e.full_course()
    e.step(0.1, right=_on_guard(e.at_gap(0.0)))
    assert e.step(2.0, right=_on_guard(e.at_gap(0.0))) == (0.8, False)


def test_gripper_hovering_next_to_the_guide_is_not_success():
    e = Episode()
    e.full_course()
    near = _gripper(T._gw(T.CLIPPER_POS, [T.GRIP_XC, 0.0, 0.0]) + (0.0, 0.0, 0.05))  # open, 5 cm above the clipper
    assert e.step(2.0, right=near) == (0.8, False)


def test_left_hand_still_on_the_clipper_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(2.0, left=_gripper(T._clipper_tcp(), contact=0.0, opening=0.06))[1] is False


def test_disturbing_a_tray_guide_is_not_success():
    e = Episode()
    e.full_course()
    e.put(5, T.start_pos(5) + (0.03, 0.0, 0.0))
    assert e.step(2.0) == (0.8, False)


def test_knocking_a_tray_guide_over_is_not_success():
    e = Episode()
    e.full_course()
    a = DEG(80)
    e.quat[T.NAMES[1]] = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))
    assert e.step(2.0)[1] is False


def test_parts_moving_restart_the_second():
    e = Episode()
    e.full_course()
    for k in range(40):
        e.pos[T.CLIPPER.name] = np.array(T.CLIPPER_POS, float) + (0.0025 * (1 if k % 2 else -1), 0.0, 0.0)
        assert e.step()[1] is False


def test_a_guide_off_the_table_scores_zero():
    e = Episode()
    e.remove_two()
    assert e.step(pos={**e.pos, T.NAMES[6]: np.array([0.4, 0.2, T.TOP - 0.3])})[0] == 0.0


def test_non_finite_state_scores_zero():
    e = Episode()
    e.remove_two()
    e.pos[T.CLIPPER.name] = np.array([float("nan"), 0.0, T.TOP])
    assert e.step()[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_left_hand_holds_while_the_right_works():
    d = T.demo()

    def span(plan, label):
        t = 0.0
        for w in plan:
            if w.label == label:
                return t, t + w.duration
            t += w.duration

    grip = span(d["left"], "grip the housing")
    release = span(d["left"], "let go")
    first = span(d["right"], "pull it off the blade")
    last = span(d["right"], "slide it on and push")
    assert grip[1] < first[0] and last[1] < release[0]
