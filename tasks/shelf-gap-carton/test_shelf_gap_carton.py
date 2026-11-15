"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_shelf_gap_carton.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "shelf_gap_carton.py")
T = TASK.module

DOWN = np.array([0.0, 0.0, -1.0])
FINAL = {"yellow": T.ROW_Y["yellow"], **T.FINAL_Y}


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _yaw(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _row_pos(name, y, lift=0.0005):
    return np.array([T.ROW_FRONT + T.CARTONS[name][1] / 2, y, T.SHELF_TOP + lift])


class Episode:
    """A scripted episode: cartons as a dict name -> (pos, quat); grippers; time in 0.05 s steps."""

    def __init__(self):
        self.t = 0.0
        self.c = {n: (_row_pos(n, T.ROW_Y[n]), np.array([0.0, 0.0, 0.0, 1.0])) for n in T.ROW}
        self.c["green"] = (np.array([*T.GREEN_START, T.TOP + 0.0005]), np.array([0.0, 0.0, 0.0, 1.0]))
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {n: _state(*self.c[n]) for n in T.NAMES}
        out["shelf"] = _state((*T.SHELF_XY, T.TOP))
        out.update(left_gripper=self.left, right_gripper=self.right, episode_time=self.t)
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            if k in ("left", "right"):
                setattr(self, k, v)
            else:
                self.c[k] = v if isinstance(v, tuple) else (np.asarray(v, float), self.c[k][1])
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def done(self):
        return T.steps_done(self.objects())

    def solved(self, **over):
        """Every carton where the demo leaves it (``over``: name -> pose replaces one), hands home."""
        poses = {n: (_row_pos(n, FINAL[n]), np.array([0.0, 0.0, 0.0, 1.0])) for n in T.NAMES}
        poses["green"] = (_row_pos("green", FINAL["green"]), np.array([0.0, 0.0, 0.0, 1.0]))
        poses.update(over)
        self.step(left=_home("left"), right=_home("right"), **poses)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set(T.NAMES)
    assert T.SUCCESS_HOLD_S == 0.0 and T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def test_geometry_no_single_gap_fits_but_the_free_width_does():
    widths = {n: T.CARTONS[n][0] for n in T.ROW}
    edges = [-T.HALF_CLEAR]
    gaps = []
    for n in T.ROW:
        gaps.append(T.ROW_Y[n] - widths[n] / 2 - edges[-1])
        edges.append(T.ROW_Y[n] + widths[n] / 2)
    gaps.append(T.HALF_CLEAR - edges[-1])
    green = T.CARTONS["green"][0]
    assert max(gaps) < green  # no single gap takes the green carton
    assert sum(gaps) >= green + T.GAP_EXTRA + 0.02  # but the free width does, with margin
    assert min(gaps) > 0.0


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False] * 4
    assert e.step(3.0) == (0.0, False)  # doing nothing, for as long as you like


def test_full_course_scores_one_only_after_one_second_still():
    e = Episode()
    e.solved()
    assert e.done() == [True] * 4
    s, ok = e.step(0.9)
    assert not ok and s == pytest.approx(0.8)
    s, ok = e.step(0.2)
    assert ok and s == 1.0


def test_gap_alone_is_one_step():
    e = Episode()
    e.step(blue=_row_pos("blue", -0.065), red=_row_pos("red", 0.093))
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.2


def test_gap_a_little_too_narrow_is_not_open():
    e = Episode()
    e.step(blue=_row_pos("blue", -0.049), red=_row_pos("red", 0.093))  # the gap is 72 mm
    assert e.done()[0] is False


def test_green_in_the_unopened_gap_is_not_possible_but_scores_nothing_if_forced():
    """If the green carton were somehow in the 26 mm gap (overlapping the others), the gap step is still false."""
    e = Episode()
    e.step(green=_row_pos("green", 0.0))
    assert e.done()[0] is False and e.done()[2] is False


def test_green_in_front_of_the_row_is_not_in_it():
    e = Episode()
    e.solved(green=(np.array([T.ROW_FRONT - 0.07, FINAL["green"], T.SHELF_TOP + 0.0005]), np.array([0, 0, 0, 1.0])))
    assert e.done() == [True, False, False, False]


def test_green_on_top_of_the_row_is_not_in_it():
    e = Episode()
    e.solved(green=_row_pos("green", FINAL["green"], lift=0.130))
    assert e.done()[1] is False and e.step(3.0)[1] is False


def test_green_at_the_end_of_the_row_breaks_the_order():
    e = Episode()
    e.solved(green=_row_pos("green", 0.19 - 0.037))  # right of the white carton
    assert e.done()[2] is False and e.step(3.0)[1] is False


def test_green_lying_down_is_not_placed():
    e = Episode()
    s = math.sqrt(0.5)
    e.solved(green=(np.array([T.ROW_FRONT + 0.05, FINAL["green"], T.SHELF_TOP + 0.037]), np.array([0, s, 0, s])))
    assert e.done()[1] is False and e.step(3.0)[1] is False


def test_a_toppled_neighbour_is_not_orderly():
    e = Episode()
    s = math.sqrt(0.5)
    e.solved(white=(np.array([T.ROW_FRONT + 0.05, FINAL["white"], T.SHELF_TOP + 0.03]), np.array([0, s, 0, s])))
    assert e.done()[:2] == [True, True] and e.done()[2] is False


def test_swapped_neighbours_break_the_order():
    e = Episode()
    e.solved(yellow=_row_pos("yellow", 0.0), blue=_row_pos("blue", -0.14))
    assert e.done()[2] is False


def test_removing_and_replacing_a_carton_is_not_penalised():
    """Final-state scoring: a carton lifted off the shelf and put back leaves no mark (it was never forbidden)."""
    e = Episode()
    e.step(blue=_row_pos("blue", -0.065, lift=0.15))
    e.step(blue=_row_pos("blue", -0.065))
    e.solved()
    assert e.done() == [True] * 4 and not T.violation(e.objects())
    assert e.step(1.2) == (1.0, True)


def test_a_carton_off_the_table_scores_zero_for_good():
    e = Episode()
    e.solved()
    e.step(yellow=np.array([0.3, -0.3, T.TOP - 0.2]))
    e.solved()
    assert e.step(2.0) == (0.0, False)


def test_a_carton_dropped_off_the_table_then_returned_still_scores_zero():
    e = Episode()
    e.step(green=np.array([0.3, -0.3, T.TOP - 0.2]))
    e.solved()
    assert T.violation(e.objects()) and e.step(2.0) == (0.0, False)


def test_a_gripper_still_at_the_row_blocks_success():
    e = Episode()
    e.solved()
    tcp = np.array([T.ROW_FRONT + 0.03, FINAL["green"], T.SHELF_TOP + 0.10])
    e.step(right=_gripper(tcp, contact=0.0))
    assert e.done() == [True, True, True, False]
    assert e.step(3.0) == (0.6, False)


def test_a_moving_carton_restarts_the_timing():
    e = Episode()
    e.solved()
    e.step(0.6)
    e.step(red=_row_pos("red", 0.0935))  # 0.5 mm: within the 2 mm tolerance
    assert e.step(0.5)[1] is True
    e2 = Episode()
    T._PROGRESS.__init__()
    e2.solved()
    e2.step(0.6)
    e2.step(red=_row_pos("red", 0.0975))  # 4.5 mm: the timing restarts
    assert e2.step(0.5)[1] is False
    assert e2.step(0.7)[1] is True


def test_yawed_cartons_use_their_true_footprint():
    e = Episode()
    e.solved(blue=(_row_pos("blue", -0.056), np.array(_yaw(math.radians(25)))))  # swung into the gap
    assert e.done()[0] is False


def test_demo_waypoints_reachable():
    plans = TASK.module.demo()
    for side in ("left", "right"):
        plan = ArmPlan(plans[side], BASES[side])
        assert plan.waypoints
        # static reach is checked with the agent kit's static_ik; here only that every waypoint is finite
        assert all(np.all(np.isfinite(np.asarray(w.pos, float))) for w in plan.waypoints)


def test_arm_reach_static():
    from yam_arena import ik

    lo, hi = np.array(ik.LOWER), np.array(ik.UPPER)
    rng = np.random.default_rng(0)
    seeds = [np.array([0, 1, 1, -0.6, 0, 0], float)] + [rng.uniform(lo, hi) for _ in range(12)]
    for side, plan in T.demo().items():
        for w in plan:
            if w.label == "wait":
                continue
            q4 = ik.matrix_to_quat_xyzw(w.rot)
            best = 1e9
            for s in seeds:
                q, pe, re = ik.solve(np.asarray(w.pos, float), q4, s, base_pos=BASES[side], iters=150)
                best = min(best, pe * 1000 + math.degrees(re))
                if best < 1.0:
                    break
            assert best < 3.0, (side, w.label, best)
