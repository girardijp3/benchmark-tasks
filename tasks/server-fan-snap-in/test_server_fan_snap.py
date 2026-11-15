"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, the 1 s timing, and the demo's
poses are reachable. Run in the yam_arena environment:  python -m pytest -q test_server_fan_snap.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "server_fan_snap.py")
T = TASK.module
UP = np.eye(3)

NAMES = ("bracket", "fan")
FIXTURE_POS = {"bracket": T.BRACKET_POS}
START_POSE = (T.FAN_START, UP)
CREEP_DIR = np.array([0.0, 0.0, 1.0])
SHUT = (0.0, 0.0)
OPEN = (-math.radians(15.0), math.radians(15.0))  # both latches swung open (the +y latch's opening is negative)


def GRIP_TCP(pos, rot):
    return T._tcp_for(pos)


def EXTRA_OBJECTS(e):
    return {"joints": getattr(e, "joints", SHUT)}


def _at(low, dy=0.0, dx=0.0, rot=UP, joints=SHUT):
    """The fan's origin (middle of its lower face) over the bracket, ``low`` above the table."""
    return (np.array([T.BRACKET_POS[0] + dx, T.BRACKET_POS[1] + dy, T.TOP + low]), rot, joints)


def SEATED_POSE():
    return _at(T.LEDGE + 0.0003)


def ENGAGED_POSE():
    return _at(T.LEDGE + 0.010, joints=OPEN)


def UNSEATED_POSES():
    return [_at(T.LEDGE + 0.003), _at(T.LEDGE + 0.0003, joints=OPEN), _at(T.LEDGE + 0.0003, dy=0.0035),
            _at(T.LEDGE + 0.0003, rot=_rx(math.radians(2.5)))]


def WRONG_POSES():
    return [_at(T.LEDGE + 0.030, joints=OPEN), _at(T.LEDGE + 0.010, dy=0.005, joints=OPEN),
            _at(T.LEDGE + 0.010, rot=_rx(math.radians(6.0)), joints=OPEN), _at(0.0, dy=0.15)]


def PULLED_OUT_POSE():
    return _at(T.LEDGE + 0.040, joints=OPEN)


# ---- generic episode tests (the part is lifted by the left gripper, brought to the fixture, seated, let go) ----------------
def _rx(a):
    return T.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), a)


def _ry(a):
    return T.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), a)


def _state(pos, rot):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos.copy(), quat_xyzw=np.asarray(T.ik.matrix_to_quat_xyzw(rot), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _gripper(tcp, axis, contact=0.0, opening=0.09):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side="left"):
    return _gripper(T._START_TCP[side], T._START_ROT[side][:, 2])


def _on_part(pos, rot, contact=20.0):
    """The left gripper closed on the part at ``pos`` / ``rot`` (the demo's grip, pointing down)."""
    return _gripper(GRIP_TCP(pos, rot), np.array([0.0, 0.0, -1.0]), contact=contact, opening=0.02)


def _jk(pose):
    return {"joints": pose[2]} if len(pose) > 2 else {}


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos, self.rot = START_POSE[0].copy(), START_POSE[1].copy()
        self.fix = {k: np.array(v, float) for k, v in FIXTURE_POS.items()}
        self.left, self.right = _home("left"), _home("right")
        self.extra = {}

    def objects(self):
        o = {k: _state(v, np.eye(3)) for k, v in self.fix.items()}
        o[T.PART] = _state(self.pos, self.rot)
        o.update({"left_gripper": self.left, "right_gripper": self.right, "episode_time": self.t})
        o.update(EXTRA_OBJECTS(self))
        return o

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

    def done(self):
        return T.steps_done(self.objects())

    def carry(self, pose):
        pos, rot = pose[0], pose[1]
        extra = {"joints": pose[2]} if len(pose) > 2 else {}
        self.step(pos=np.asarray(pos, float), rot=rot, left=_on_part(pos, rot), **extra)

    def pick(self):
        self.step(left=_on_part(self.pos, self.rot))
        self.carry((self.pos + (0, 0, 0.04), self.rot))

    def engage(self):
        self.carry(ENGAGED_POSE())

    def lower(self):
        self.carry(SEATED_POSE())

    def let_go(self):
        self.step(left=_home("left"), pos=SEATED_POSE()[0], rot=SEATED_POSE()[1], **_jk(SEATED_POSE()))

    def full_course(self):
        self.pick()
        self.engage()
        self.lower()
        self.let_go()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert TASK.names == NAMES and TASK.free_objects == (T.PART,)
    assert T.SUCCESS_HOLD_S == 0.0


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_picked_is_a_step():
    e = Episode()
    e.pick()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.2


def test_moved_without_a_grip_is_not_picked():
    e = Episode()
    e.step(pos=e.pos + (0, 0, 0.05))
    assert e.done() == [False, False, False, False]


def test_engaged_is_a_step():
    e = Episode()
    e.pick()
    e.engage()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.4


def test_wrong_pose_is_not_engaged():
    for pose in WRONG_POSES():
        T._PROGRESS.__init__()
        e = Episode()
        e.pick()
        e.carry(pose)
        assert e.done()[1] is False, pose


def test_seated_is_a_step():
    e = Episode()
    e.pick()
    e.engage()
    e.lower()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.6


def test_engaged_but_not_home_is_not_seated():
    for pose in UNSEATED_POSES():
        T._PROGRESS.__init__()
        e = Episode()
        e.pick()
        e.engage()
        e.carry(pose)
        d = e.done()
        assert d[2] is False and d[3] is False, pose


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.1) == (1.0, True)


def test_still_holding_the_part_is_not_let_go():
    e = Episode()
    e.pick()
    e.engage()
    e.lower()
    e.step(2.0)
    assert e.done() == [True, True, True, False]
    assert e.step(1.5) == (0.6, False)


def test_a_gripper_beside_the_part_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(left=_gripper(GRIP_TCP(e.pos, e.rot) + (0.0, 0.0, 0.02), np.array([0.0, 0.0, -1.0])))
    assert e.done()[3] is False


def test_success_is_timed_in_the_task_file():
    e = Episode()
    e.pick()
    e.engage()
    e.lower()
    e.step(left=_home("left"))
    assert e.step(0.8)[1] is False and T.score(e.objects()) <= 0.8
    assert e.step(0.3)[1] is True and T.score(e.objects()) == 1.0


def test_no_full_score_without_the_pick():
    e = Episode()
    e.step(pos=SEATED_POSE()[0], rot=SEATED_POSE()[1], **_jk(SEATED_POSE()))  # simply placed at the seat without a gripper
    assert e.step(3.0) == (0.0, False)


@pytest.mark.parametrize("what", ["held", "unseated", "wrong"])
def test_no_full_score_while_a_condition_is_missing(what):
    e = Episode()
    e.pick()
    e.engage()
    if what == "held":
        e.lower()
        e.step(2.5)
    elif what == "unseated":
        e.carry(UNSEATED_POSES()[0])
        e.step(left=_home("left"))
    else:
        e.carry(WRONG_POSES()[0])
        e.step(left=_home("left"))
    assert e.step(2.0)[1] is False and e.step(0.1)[0] < 1.0


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    e.step(0.6)
    e.step(pos=UNSEATED_POSES()[0][0], rot=UNSEATED_POSES()[0][1], **_jk(UNSEATED_POSES()[0]))
    e.step(0.1)
    e.step(pos=SEATED_POSE()[0], rot=SEATED_POSE()[1], **_jk(SEATED_POSE()))
    assert e.step(0.6)[1] is False
    assert e.step(0.5)[1] is True


def test_part_creeping_restarts_the_second():
    e2 = Episode()
    e2.full_course()
    p0 = SEATED_POSE()[0]
    for k in range(40):
        e2.step(pos=p0 + CREEP_DIR * 0.0004 * k)  # 0.4 mm every 0.05 s
    assert e2.step()[1] is False


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    away = _gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1), np.array([0.0, 0.0, -1.0]))
    assert e.step(1.2, left=away) == (1.0, True)


def test_part_pulled_out_loses_the_steps():
    e = Episode()
    e.full_course()
    e.step(pos=PULLED_OUT_POSE()[0], rot=PULLED_OUT_POSE()[1], **_jk(PULLED_OUT_POSE()))
    assert e.done()[1:] == [False, False, False]


def test_part_knocked_over_scores_zero():
    e = Episode()
    e.pick()
    e.engage()
    assert e.step(rot=_rx(math.radians(90.0)))[0] == 0.0


def test_part_off_the_table_scores_zero():
    e = Episode()
    e.pick()
    assert e.step(pos=np.array([0.3, 0.6, T.TOP - 0.3]))[0] == 0.0


def test_fixture_torn_loose_scores_zero():
    e = Episode()
    e.full_course()
    k = next(iter(FIXTURE_POS))
    assert e.step(fix={**e.fix, k: e.fix[k] + (0.02, 0.0, 0.0)})[0] == 0.0


def test_not_finite_scores_zero():
    e = Episode()
    e.pick()
    assert e.step(pos=np.array([math.nan, 0.0, 1.0]))[0] == 0.0


def test_demo_uses_the_left_arm_only_and_is_reachable():
    assert set(T.demo()) == {"left"}
    plan = ArmPlan(T.demo()["left"], BASES["left"])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    assert sum(w.duration for w in T.demo()["left"]) < 35.0
