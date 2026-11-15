"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, contamination (an
instrument or its gripper touching the border band or the bare tabletop) is persistent, the 1 s full-success timing, the assets match the scoring geometry, and the demo's poses are reachable.
Run in the yam_arena environment:  python -m pytest -q test_sterile_field_border.py"""

import math
import pathlib

import numpy as np
import pytest
from pxr import Usd, UsdGeom, UsdPhysics
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "sterile_field_border.py")
T = TASK.module
NAMES = T.NAMES


def _quat(yaw, pitch=0.0):
    r = T._rz(yaw)
    if pitch:
        r = r @ T.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), pitch)
    return np.array(T._xyzw(r))


def _gripper(tcp, contact=0.0, opening=0.09, axis=(0.0, 0.0, -1.0), closing=(0.0, 1.0, 0.0)):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact, closing=np.asarray(closing, float))


def _home():
    return _gripper(T._START_TCP["left"])


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pose = {n: (T.START[n].copy(), _quat(T.START_YAW)) for n in NAMES}
        self.left, self.right = _home(), _home()

    def objects(self):
        out = {}
        for n, (pos, quat) in self.pose.items():
            out[n] = ObjectState(pos=pos.copy(), quat_xyzw=quat.copy(), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                                 low=pos.copy(), high=pos.copy())
        out.update({"left_gripper": self.left, "right_gripper": self.right, "episode_time": self.t})
        return out

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            setattr(self, k, v)
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            o = self.objects()
            last = (T.score(o), T.success(o))
            if self.t >= end - 1e-9:
                return last

    def scores(self, seconds):
        out, end = [], self.t + seconds
        while self.t < end - 1e-9:
            self.t = round(self.t + 0.05, 6)
            o = self.objects()
            out.append((T.score(o), T.success(o)))
        return out

    def done(self):
        return T.steps_done(self.objects())

    def move(self, name, pos=None, quat=None):
        p, q = self.pose[name]
        self.pose[name] = (np.asarray(pos, float) if pos is not None else p, quat if quat is not None else q)

    def pick(self, name, lift=0.03):
        """A gripper closed on the instrument at its start (fingers across it), the instrument lifted."""
        pos, quat = self.pose[name]
        up = pos + np.array([0.0, 0.0, lift])
        r = T._rot(ObjectState(pos=up, quat_xyzw=quat, lin_vel=0, ang_vel=0, low=up, high=up))
        g = _gripper(up + np.array([0.0, 0.0, 0.002 + T.PAD_TIP_PAST_TCP]), contact=20.0, opening=0.011,
                     closing=r[:, 1])
        self.step(left=g)
        self.move(name, pos=up)
        self.step(left=g)

    def lay(self, name, pos=None, yaw=0.0):
        self.move(name, pos=T.PLACE[name] if pos is None else pos, quat=_quat(yaw))
        self.step(left=_home())

    def full_course(self):
        for n in T.ORDER:
            self.pick(n)
            self.lay(n)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(NAMES) <= set(TASK.names)


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False] * 4


def test_each_instrument_is_a_step():
    e = Episode()
    for k, n in enumerate(T.ORDER[:3]):
        e.pick(n)
        e.lay(n)
        assert e.step()[0] == pytest.approx(0.2 * (k + 1))


def test_full_course_succeeds_after_one_second():
    e = Episode()
    e.full_course()
    assert e.done() == [True] * 4
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # held 0.95 s
    assert e.step() == (1.0, True)


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0 and T.POINTS * 4 <= 0.8 + 1e-9


def test_not_picked_up_does_not_count():
    """Laid on the field without a gripper ever holding it (pushed, thrown): no step."""
    e = Episode()
    e.lay("needle_holder")
    assert e.done()[NAMES.index("needle_holder")] is False


def _over_border(name):
    """The instrument lying across the border's band, on the panel's top face."""
    return np.array([T.FIELD_C[0] - T.FIELD_OUT[0] / 2 + 0.02 - 0.5 * (T.BOXES[name][0][0] + T.BOXES[name][1][0]),
                     T.LANE_Y[name], T.DRAPE_TOP])


def test_touching_the_border_is_for_good():
    e = Episode()
    e.pick("pean_forceps")
    e.lay("pean_forceps", pos=_over_border("pean_forceps"))
    assert T._PROGRESS.touched["pean_forceps"] is True
    e.lay("pean_forceps")  # moved inward afterwards, laid perfectly
    assert e.done()[NAMES.index("pean_forceps")] is False
    for n in T.ORDER:
        if n != "pean_forceps":
            e.pick(n)
            e.lay(n)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history)
    assert max(s for s, _ in history) <= 0.6 + 1e-9


def test_dragging_over_the_border_low_is_a_touch_but_carrying_high_is_not():
    e = Episode()
    e.pick("needle_holder")
    for x in np.linspace(T.FIELD_C[0] - 0.2, T.FIELD_C[0], 12):  # carried 3 cm above the panel across the band
        e.move("needle_holder", pos=(x, T.FIELD_C[1] - 0.17, T.DRAPE_TOP + 0.03))
        e.step()
    assert T._PROGRESS.touched["needle_holder"] is False
    e.move("needle_holder", pos=(T.FIELD_C[0], T.FIELD_C[1] - 0.17, T.DRAPE_TOP))  # set down on the band
    e.step()
    assert T._PROGRESS.touched["needle_holder"] is True


def test_a_long_instrument_across_a_narrow_band_with_both_ends_inside_still_touches():
    """The band is 25.4 mm; an instrument whose ends are far apart but whose middle crosses it touches it (surface
    points are sampled all along the underside, not only at the ends)."""
    e = Episode()
    e.pick("scalpel_handle")
    # turned along y across the +x band: the handle's ends are 135 mm apart, the middle lies in the band
    x_band = T.FIELD_C[0] + T.FIELD_OUT[0] / 2 - T.BORDER / 2
    e.move("scalpel_handle", pos=(x_band, T.FIELD_C[1], T.DRAPE_TOP), quat=_quat(math.pi / 2))
    e.step()
    assert T._PROGRESS.touched["scalpel_handle"] is True


def test_the_rack_and_the_sterile_field_are_not_nonsterile():
    e = Episode()
    e.step(1.0)  # on the rack, both grippers at home
    assert not any(T._PROGRESS.touched.values()) and not any(T._PROGRESS.pad_soiled.values())
    e.pick("needle_holder")
    e.lay("needle_holder")  # laid on the blue area
    assert not any(T._PROGRESS.touched.values())


def _table_spot(name):
    """Well beyond the panel and the rack: the bare tabletop."""
    return np.array([0.20, -0.10, T.TOP])


def _pads_on(point):
    """A gripper whose pad tips rest on ``point`` (z of the surface), fingers open."""
    p = np.asarray(point, float)
    return _gripper(p + np.array([0.0, 0.0, T.PAD_TIP_PAST_TCP + 0.0005]), opening=0.011)


def test_an_instrument_touching_the_bare_table_is_contaminated_for_good():
    e = Episode()
    e.pick("needle_holder")
    e.lay("needle_holder", pos=_table_spot("needle_holder"))  # set down on the tabletop
    assert T._PROGRESS.touched["needle_holder"] is True
    e.pick("needle_holder")
    e.lay("needle_holder")  # picked up again and laid perfectly on the field
    assert e.done()[NAMES.index("needle_holder")] is False
    for n in T.ORDER:
        if n != "needle_holder":
            e.pick(n)
            e.lay(n)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history)
    assert max(s for s, _ in history) <= 0.6 + 1e-9


def test_an_instrument_dropped_on_the_table_then_returned_is_still_contaminated():
    """It touches the tabletop only for an instant (one step), then is back on the rack: still latched."""
    e = Episode()
    e.pick("scalpel_handle")
    e.move("scalpel_handle", pos=(0.20, -0.10, T.TOP))
    e.step()
    e.move("scalpel_handle", pos=T.START["scalpel_handle"])
    e.step(1.0)
    assert T._PROGRESS.touched["scalpel_handle"] is True


def test_gripper_touching_the_tabletop_contaminates_what_it_then_handles():
    e = Episode()
    e.step(left=_pads_on(_table_spot("x")))  # the left gripper's pads touch the bare table: no instrument involved
    assert T._PROGRESS.pad_soiled["left_gripper"] is True
    assert not any(T._PROGRESS.touched.values())  # touching the table alone costs nothing yet
    e.step(left=_home())
    e.pick("needle_holder")  # but the instrument it now picks up is contaminated
    assert T._PROGRESS.touched["needle_holder"] is True
    e.lay("needle_holder")
    assert e.done()[NAMES.index("needle_holder")] is False
    for n in T.ORDER:
        if n != "needle_holder":
            e.pick(n)
            e.lay(n)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history)
    # every pickup was by the same soiled left gripper, so no instrument is clean
    assert max(s for s, _ in history) == 0.0


def test_a_clean_arm_after_a_soiled_one_can_still_score():
    """Only the gripper that touched a nonsterile surface is soiled: the other one handles instruments cleanly."""
    e = Episode()
    e.step(right=_pads_on(_table_spot("x")))
    assert T._PROGRESS.pad_soiled["right_gripper"] is True and T._PROGRESS.pad_soiled["left_gripper"] is False
    e.full_course()  # all four by the left gripper
    assert e.done() == [True] * 4
    e.step(T.STILL_S - 0.05)
    assert e.step() == (1.0, True)


def test_gripper_pads_on_the_border_band_are_soiled_and_on_the_blue_area_or_rack_are_not():
    e = Episode()
    on_inner = np.array([T.FIELD_C[0], T.FIELD_C[1], T.DRAPE_TOP])
    on_tray = np.array([T.RACK_C[0], T.RACK_C[1], T.TOP + T.RACK_PLATE])
    e.step(left=_pads_on(on_inner), right=_pads_on(on_tray))
    assert not any(T._PROGRESS.pad_soiled.values())
    band = np.array([T.FIELD_C[0], T.FIELD_C[1] - T.FIELD_OUT[1] / 2 + 0.01, T.DRAPE_TOP])
    e.step(left=_pads_on(band))
    assert T._PROGRESS.pad_soiled["left_gripper"] is True and T._PROGRESS.pad_soiled["right_gripper"] is False


def test_a_soiled_gripper_touching_an_instrument_already_laid_contaminates_it():
    e = Episode()
    e.full_course()
    assert e.done() == [True] * 4
    e.step(left=_pads_on(_table_spot("x")))  # the arm then touches the bare table
    assert T._PROGRESS.pad_soiled["left_gripper"] is True and e.done() == [True] * 4
    e.step(left=_gripper(T.PLACE["pean_forceps"] + (0.0, 0.0, T.PAD_TIP_PAST_TCP + 0.004), opening=0.011,
                         closing=(0.0, 1.0, 0.0)))  # and reaches back down onto a laid instrument
    assert T._PROGRESS.touched["pean_forceps"] is True
    assert max(s for s, _ in e.scores(3.0)) <= 0.6 + 1e-9


def test_the_home_poses_and_the_demo_pads_stay_sterile():
    assert not T.gripper_touching_nonsterile(_home())
    for arm in ("left", "right"):
        for w in T.demo()[arm]:
            rot = w.rot if w.rot is not None else T._START_ROT[arm]
            tip = np.asarray(w.pos, float) + T.PAD_TIP_PAST_TCP * (rot @ np.array([0.0, 0.0, 1.0]))
            assert not np.any(T.nonsterile_hit(tip[None, :])), (arm, w.label)


def test_heading_and_lane_are_not_scored():
    """An instrument pointing at the robot and the scalpel handle in a middle lane still score 1.0."""
    e = Episode()
    swap = {"scalpel_handle": "needle_holder", "needle_holder": "scalpel_handle"}
    for n in T.ORDER:
        e.pick(n)
        pos = T.PLACE[n].copy()
        if n in swap:
            pos[1] = T.LANE_Y[swap[n]]
        e.lay(n, pos=pos, yaw=math.pi if n == "pean_forceps" else 0.0)
    assert e.done() == [True] * 4
    e.step(T.STILL_S - 0.05)
    assert e.step() == (1.0, True)


MISSING = {
    "one instrument never picked": lambda e: [(e.pick(n), e.lay(n)) for n in T.ORDER[:3]] + [e.lay(T.ORDER[3])],
    "instrument outside the sterile area": lambda e: (e.full_course(), e.move("pean_forceps", pos=T.PLACE["pean_forceps"] + (0.0, -0.07, 0.0)), e.step()),
    "instrument leaning": lambda e: (e.full_course(), e.move("metzenbaum_scissors", quat=_quat(0.0, math.radians(8))), e.step()),
    "instrument lying on another": lambda e: (e.full_course(), e.move("needle_holder", pos=T.PLACE["metzenbaum_scissors"] + (0.0, 0.004, 0.0)), e.step()),
    "instrument held above the panel": lambda e: (e.full_course(), e.move("needle_holder", pos=T.PLACE["needle_holder"] + (0.0, 0.0, 0.02)), e.step()),
    "instrument touched the bare table in passing": lambda e: (e.full_course(), e.move("pean_forceps", pos=(0.20, -0.10, T.TOP)), e.step(), e.move("pean_forceps", pos=T.PLACE["pean_forceps"]), e.step()),
    "gripper within 3 cm": lambda e: (e.full_course(), e.step(left=_gripper(T.PLACE["pean_forceps"] + (0.0, 0.0, 0.05)))),
}


@pytest.mark.parametrize("missing", sorted(MISSING))
def test_no_full_score_while_any_success_condition_is_missing(missing):
    e = Episode()
    MISSING[missing](e)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history), missing
    assert max(s for s, _ in history) <= 0.8 + 1e-9


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.5) == (0.8, False)
    near = _gripper(T.PLACE["pean_forceps"] + (0.0, 0.0, 0.05))
    assert e.step(left=near) == (0.8, False)  # not released: the second stops counting
    assert e.step(left=_home()) == (0.8, False)
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step() == (1.0, True)


def test_an_instrument_creeping_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.9) == (0.8, False)
    e.move("pean_forceps", pos=T.PLACE["pean_forceps"] + (0.0, 0.003, 0.0))
    assert e.step() == (0.8, False)
    assert e.step(T.STILL_S - 0.1) == (0.8, False)
    assert e.step(0.1) == (1.0, True)


def test_full_score_drops_when_something_moves_after_success():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S) == (1.0, True)
    e.move("needle_holder", pos=T.PLACE["needle_holder"] + (0.0, 0.0, 0.03))
    assert e.step()[1] is False


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    assert e.step(1.1, left=_gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))) == (1.0, True)


def test_off_the_table_scores_zero():
    e = Episode()
    e.full_course()
    assert e.step(pos=None, pose={**e.pose, "needle_holder": (np.array([0.3, 0.0, T.TOP - 0.2]), _quat(0.0))})[0] == 0.0


def test_not_finite_scores_zero():
    e = Episode()
    e.move("needle_holder", pos=(float("nan"), 0.0, T.TOP))
    assert e.step()[0] == 0.0


def test_geometry_is_true_size():
    nominal = {"metzenbaum_scissors": 0.180, "pean_forceps": 0.160, "scalpel_handle": 0.135, "needle_holder": 0.152}
    for n, L in nominal.items():
        lo, hi = T._extent(n)
        assert hi[0] - lo[0] == pytest.approx(L, abs=0.0015), n
    assert T.BORDER == pytest.approx(0.0254)


def test_assets_match_scoring_geometry():
    for n in NAMES:
        stage = Usd.Stage.Open(str(HERE / "assets" / f"{n}.usda"))
        pts = []
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
                assert len(UsdGeom.Mesh(prim).GetPointsAttr().Get()) <= 64
                pts += [tuple(p) for p in UsdGeom.Mesh(prim).GetPointsAttr().Get()]
        pts = np.array(pts)
        lo, hi = T._extent(n)
        assert np.allclose(pts[:, :2].min(0), lo, atol=2e-4) and np.allclose(pts[:, :2].max(0), hi, atol=2e-4), n
        assert pts[:, 2].min() == pytest.approx(0.0, abs=1e-6)
    field = Usd.Stage.Open(str(HERE / "assets" / "sterile_field.usda"))
    panel = [p for p in field.Traverse() if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)][0]
    a = np.array([tuple(p) for p in UsdGeom.Mesh(panel).GetPointsAttr().Get()])
    assert np.allclose(a.max(0) - a.min(0), [T.FIELD_OUT[0], T.FIELD_OUT[1], T.DRAPE_T], atol=1e-6)


def test_files_are_small():
    assert all(p.stat().st_size < 1_870_000 for p in HERE.rglob("*") if p.is_file())


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
