"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, the demo's welds land on
the cells' + terminals, and the demo's poses are reachable. Run in the yam_arena environment:
python -m pytest -q test_welder_strip.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "welder_strip.py")
T = TASK.module

DOWN = np.array([0.0, 0.0, -1.0])
STRIP_QUAT = np.array(T._yaw_xyzw(T.STRIP_YAW))
LEVEL_QUAT = np.array(T._yaw_xyzw(T.WELDER_YAW))
ON_PACK = np.array([*T.PACK_XY, T.NUB_TOP])
LIFTED = np.array(T.STRIP_POS) + (0.0, 0.0, 0.06)


def _matrix_to_quat(r) -> np.ndarray:
    """xyzw of a rotation matrix (its trace is positive for the rotations used here, or w is chosen largest)."""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])
    x = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2
    return np.array([x, (r[0, 1] + r[1, 0]) / (4 * x), (r[0, 2] + r[2, 0]) / (4 * x), (r[2, 1] - r[1, 2]) / (4 * x)])


TILTED_QUAT = _matrix_to_quat(T.WELDER_TILTED)


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point, opening=0.01):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.01), contact=10.0, opening=opening)


def _home(side):
    return _gripper(T._START_TCP[side])


def _welder_grip(origin, quat):
    return np.asarray(origin, float) + T.quat_xyzw_to_matrix(quat) @ np.array([T.WELDER_GRIP_AT, 0.0, T.WELDER_HEIGHT / 2])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), speed=0.0, spin=0.0):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([speed, 0.0, 0.0]),
                       ang_vel=np.array([0.0, 0.0, spin]), low=pos.copy(), high=pos.copy())


class Episode:
    def __init__(self):
        self.t = 0.0
        self.origin, self.quat = np.array(T.WELDER_POS), LEVEL_QUAT.copy()
        self.strip, self.strip_quat = np.array(T.STRIP_POS), STRIP_QUAT.copy()
        self.pack, self.pack_quat = np.array(T.PACK_POS), np.array([0.0, 0.0, 0.0, 1.0])
        self.left, self.right = _home("left"), _home("right")
        self.speed = 0.0
        self.pack_speed = self.pack_spin = 0.0

    def objects(self):
        housing = self.origin + T.quat_xyzw_to_matrix(self.quat) @ T.BODY_SHIFT
        out = {
            "welder": _state(housing, self.quat, self.speed),
            "housing": T.PartPose(pos=housing, quat_xyzw=self.quat),
            "welder_cell": _state(self.origin + T.quat_xyzw_to_matrix(self.quat) @ T.CELL_IN_WELDER),
            "welder_holder": _state(T.HOLDER_POS, T._yaw_xyzw(T.HOLDER_YAW)),
            "pack": _state(self.pack, self.pack_quat, self.pack_speed, self.pack_spin),
            "strip": _state(self.strip, self.strip_quat, self.speed),
            "strip_tray": _state(T.TRAY_POS),
            "left_gripper": self.left,
            "right_gripper": self.right,
            "episode_time": self.t,
        }
        return out

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

    def take_strip(self):
        self.step(right=_holding(np.array(T.STRIP_POS) + (0.0, 0.0, 0.005), opening=T.TAB_ACROSS))
        self.step(strip=LIFTED.copy(), right=_holding(LIFTED + (0.0, 0.0, 0.005), opening=T.TAB_ACROSS))

    def lay_strip(self, at=ON_PACK, quat=STRIP_QUAT):
        self.take_strip()
        self.step(strip=np.asarray(at, float).copy(), strip_quat=np.asarray(quat, float).copy(), right=_home("right"))

    def take_welder(self):
        self.step(left=_holding(_welder_grip(self.origin, self.quat)))
        lifted = self.origin + (0.0, 0.0, 0.05)
        self.step(origin=lifted, left=_holding(_welder_grip(lifted, self.quat)))

    def weld(self, cell, seconds=0.4, shift=(0.0, 0.0, 0.0), held=True):
        """The welder tilted with both tips on the strip over ``cell`` (``shift`` moves it) for ``seconds``."""
        origin = T.weld_origin(cell) + np.asarray(shift, float)
        left = _holding(_welder_grip(origin, TILTED_QUAT)) if held else _home("left")
        self.step(origin=origin, quat=TILTED_QUAT.copy(), left=left)
        self.step(seconds)
        self.step(origin=origin + (0.0, 0.0, 0.01), left=_holding(_welder_grip(origin + (0.0, 0.0, 0.01), TILTED_QUAT)))

    def put_welder_back(self):
        origin = np.array(T.WELDER_POS)
        self.step(origin=origin, quat=LEVEL_QUAT.copy(), left=_holding(_welder_grip(origin, LEVEL_QUAT)))
        self.step(left=_home("left"))

    def full_course(self):
        self.lay_strip()
        self.take_welder()
        for cell in T.WELD_ORDER:
            self.weld(cell)
        self.put_welder_back()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"welder", "welder_cell", "pack", "strip"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False] * 6


def test_strip_laid_on_the_pack_is_a_quarter():
    e = Episode()
    e.lay_strip()
    assert e.done() == [True, False, False, False, False, False] and e.step()[0] == 0.2


def test_strip_laid_a_quarter_turn_round_counts():
    e = Episode()
    e.lay_strip(quat=np.array(T._yaw_xyzw(T.STRIP_YAW + math.pi / 2)))
    assert e.done()[0] is True


def test_strip_still_held_on_the_pack_does_not_count():
    e = Episode()
    e.take_strip()
    e.step(strip=ON_PACK.copy(), right=_holding(ON_PACK + (0.0, 0.0, 0.005), opening=T.TAB_ACROSS))
    assert e.done()[0] is False


def test_strip_on_the_pack_without_a_gripper_does_not_count():
    e = Episode()
    e.step(strip=ON_PACK.copy())
    assert e.done()[0] is False


@pytest.mark.parametrize("change", ["off centre", "turned", "tilted", "floating"])
def test_strip_badly_laid_does_not_count(change):
    at, quat = ON_PACK.copy(), STRIP_QUAT.copy()
    if change == "off centre":
        at += (0.004, 0.0, 0.0)
    elif change == "turned":
        quat = np.array(T._yaw_xyzw(T.STRIP_YAW + math.radians(30)))
    elif change == "tilted":  # 14 degrees
        quat = np.array(T._qmul(tuple(STRIP_QUAT), (math.sin(math.radians(7)), 0.0, 0.0, math.cos(math.radians(7)))))
    else:
        at += (0.0, 0.0, 0.003)
    e = Episode()
    e.lay_strip(at=at, quat=quat)
    assert e.done()[0] is False


def test_demo_welds_land_on_the_terminals():
    objects = Episode().objects()
    objects["strip"] = _state(ON_PACK, STRIP_QUAT)
    for cell in range(1, 5):
        objects["housing"] = T.PartPose(pos=T.weld_origin(cell) + T.WELDER_TILTED @ T.BODY_SHIFT, quat_xyzw=TILTED_QUAT)
        assert T.touching_cell(objects) == cell
        for tip in T.probe_tips(objects):
            assert np.hypot(*(tip[:2] - np.array(T.CELL_POS[cell - 1])[:2])) < T.NUB_RADIUS - 0.001


def test_tips_are_the_welders_lowest_points_when_tilted():
    tips = [T.WELDER_TILTED @ c - (0.0, 0.0, T.TIP_RADIUS) for c in T.TIP_BALLS]
    lo, hi = T.WELDER_BOX  # the housing's front corners (x 124 mm) and its feet (0.5 mm below its bottom)
    corners = [np.array([0.124, y, z]) for y in (lo[1], hi[1]) for z in (-T.FEET, hi[2])]
    assert min(t[2] for t in tips) + 0.003 < min((T.WELDER_TILTED @ c)[2] for c in corners)


def test_one_weld_is_an_eighth():
    e = Episode()
    e.lay_strip()
    e.take_welder()
    e.weld(2)
    assert e.done() == [True, False, True, False, False, False] and e.step()[0] == pytest.approx(0.3)


def test_too_short_a_touch_is_not_a_weld():
    e = Episode()
    e.lay_strip()
    e.take_welder()
    e.weld(2, seconds=0.05)
    assert e.done()[2] is False


@pytest.mark.parametrize("shift", [(0.004, 0.0, 0.0), (0.0, 0.0, 0.0015), (0.0, 0.0, -0.002)])
def test_tips_off_the_terminal_or_not_on_the_strip_are_not_a_weld(shift):
    e = Episode()
    e.lay_strip()
    e.take_welder()
    e.weld(3, shift=shift)
    assert not any(e.done()[1:5])


def test_a_tip_off_the_strips_edge_is_not_a_weld():
    e = Episode()
    e.lay_strip(at=ON_PACK + (0.0, -0.0025, 0.0))  # the strip 2.5 mm to the right: the outer tip hangs past the bar
    e.take_welder()
    e.weld(2)
    assert e.done()[2] is False


def test_welding_before_the_strip_is_laid_does_not_count():
    e = Episode()
    e.take_welder()
    e.weld(1)
    e.lay_strip()
    assert e.done() == [True, False, False, False, False, False]


def test_welder_not_in_a_gripper_does_not_weld():
    e = Episode()
    e.lay_strip()
    e.weld(4, held=False)
    assert e.done()[4] is False


def test_welds_are_recorded_where_the_tips_touched():
    e = Episode()
    e.lay_strip()
    e.take_welder()
    e.weld(4)
    a, b = T._PROGRESS.welds[4]
    cell_in_strip = T.quat_xyzw_to_matrix(STRIP_QUAT).T @ (np.array(T.CELL_POS[3]) - ON_PACK)
    assert np.hypot(*((a + b)[:2] / 2 - cell_in_strip[:2])) < 1e-4 and abs(np.linalg.norm(a - b) - 0.00318) < 1e-4


def test_demo_welds_show_the_spots_laid_out_under_them():
    """Each planned weld shows two different spots, and the strip asset's own places for them (its quarter's cell axis,
    1.6 mm either side across the bar) are where the tips touch, so the marks look right even if they could not move."""
    e = Episode()
    e.lay_strip()
    e.take_welder()
    names = set()
    for cell in T.WELD_ORDER:
        e.weld(cell)
        spots = T._PROGRESS.welds[cell]
        for name, at in zip(T.spot_names(spots), spots, strict=True):
            k, tip = int(name[5]), name[6]
            default = np.array([0.01 if k > 2 else -0.01, 0.01 if k % 2 == 0 else -0.01])
            default[0] += -0.0016 if tip == "a" else 0.0016
            assert np.linalg.norm(at[:2] - default) < 0.0005, (cell, name, at)
            names.add(name)
    assert len(names) == 8


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True] * 6
    assert e.step(1.0) == (1.0, True)


def test_no_full_score_before_one_second_of_stillness():
    """The success conditions first hold on the step the left arm is back home (the last of ``full_course``): 0.8
    until they have held for 1 s, then 1.0."""
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # held 0.95 s
    assert e.step(0.05) == (1.0, True)  # held 1.0 s


@pytest.mark.parametrize("motion", [{"speed": 0.02}, {"pack_speed": 0.02}, {"pack_spin": 1.0}])
def test_motion_restarts_the_second(motion):
    """The welder and the strip (``speed``) or the pack moving at 2 cm/s, or the pack turning at 1 rad/s."""
    e = Episode()
    e.full_course()
    assert e.step(0.6) == (0.8, False)
    assert e.step(**motion) == (0.8, False)  # not still: the count stops
    assert e.step(**{k: 0.0 for k in motion}) == (0.8, False)  # still again: the second starts over on this step
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # 1.65 s since the conditions first held, but 0.95 s unbroken
    assert e.step(0.05) == (1.0, True)


def test_arm_leaving_home_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.6) == (0.8, False)
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (0.8, False)
    assert e.step(right=_home("right")) == (0.8, False)
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step(0.05) == (1.0, True)


def test_full_score_drops_when_a_part_moves_after_success():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S) == (1.0, True)
    assert e.step(pack_speed=0.05) == (0.8, False)


def _course_missing(missing):
    """The full course with one full-success condition left out, during the course or after it."""
    e = Episode()
    if missing == "strip off centre on the pack":
        e.lay_strip(at=ON_PACK + (0.004, 0.0, 0.0))
    else:
        e.lay_strip()
    e.take_welder()
    for cell in T.WELD_ORDER:
        if missing != f"cell {cell} not welded":
            e.weld(cell)
    if missing == "welder never put back":
        e.step(left=_home("left"))
        return e
    e.put_welder_back()
    changes = {
        "welder left beside its holder": {"origin": np.array(T.WELDER_POS) + (0.0, -0.06, -0.0035)},
        "left gripper still holding the welder": {"left": _holding(_welder_grip(T.WELDER_POS, LEVEL_QUAT))},
        "left arm not home": {"left": _gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))},
        "right arm not home": {"right": _gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))},
        "welder and strip moving": {"speed": 0.02},
        "pack moving": {"pack_speed": 0.02},
        "pack turning": {"pack_spin": 1.0},
    }.get(missing, {})
    for k, v in changes.items():
        setattr(e, k, v)
    return e


@pytest.mark.parametrize("missing", ["strip off centre on the pack", *(f"cell {c} not welded" for c in range(1, 5)),
                                     "welder never put back", "welder left beside its holder",
                                     "left gripper still holding the welder", "left arm not home",
                                     "right arm not home", "welder and strip moving", "pack moving", "pack turning"])
def test_no_full_score_while_any_success_condition_is_missing(missing):
    """With each full-success condition missing in turn, 3 s go by and score() never reaches 1.0."""
    e = _course_missing(missing)
    for _ in range(60):
        got, ok = e.step()
        assert got < 1.0 and not ok, (missing, e.t, got)


def test_run_ends_when_success_first_holds():
    """success() already needs the conditions held for 1 s, so the run is not held a second more."""
    assert TASK.success_hold_s == 0.0


def test_welder_never_taken_out_is_not_back():
    e = Episode()
    e.lay_strip()
    e.step(left=_holding(_welder_grip(T.WELDER_POS, LEVEL_QUAT)))
    e.step(left=_home("left"))
    assert e.done()[5] is False


def test_welder_still_held_in_its_holder_is_not_back():
    e = Episode()
    e.full_course()
    e.step(left=_holding(_welder_grip(T.WELDER_POS, LEVEL_QUAT)))
    assert e.done()[5] is False


def test_welder_left_beside_its_holder_is_not_back():
    e = Episode()
    e.full_course()
    e.step(origin=np.array(T.WELDER_POS) + (0.0, -0.06, -0.0035))
    assert e.done()[5] is False


def test_arms_not_home_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(right=_gripper(T._START_TCP["right"] + (0.0, 0.0, -0.1))) == (0.8, False)  # 6 steps, not success


def test_moving_parts_are_not_success():
    e = Episode()
    e.full_course()
    assert e.step(speed=0.05) == (0.8, False)  # 6 steps, not success


def test_pack_knocked_over_scores_zero():
    e = Episode()
    e.lay_strip()
    tipped = np.array([math.sin(math.radians(15)), 0.0, 0.0, math.cos(math.radians(15))])  # 30 degrees
    assert e.step(pack_quat=tipped)[0] == 0.0


def test_pack_pushed_along_still_takes_the_strip_and_the_welds():
    e = Episode()
    shift = np.array([0.01, -0.005, 0.0])
    e.pack = e.pack + shift
    e.lay_strip(at=ON_PACK + shift)
    e.take_welder()
    e.weld(2, shift=shift)
    assert e.done()[:3] == [True, False, True]


def test_welder_off_the_table_scores_zero():
    e = Episode()
    e.lay_strip()
    assert e.step(origin=np.array([0.3, 0.4, T.TOP - 0.3]))[0] == 0.0


def test_strip_off_the_table_scores_zero():
    e = Episode()
    e.take_strip()
    assert e.step(strip=np.array([0.3, -0.5, T.TOP - 0.3]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 3.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 2.0, plan.worst_ik_deg
