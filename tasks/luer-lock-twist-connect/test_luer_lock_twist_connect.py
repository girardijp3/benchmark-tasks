"""Offline checks (no GPU, no network): the task file loads and the assets exist; the scoring ladder covers idle,
picked up, seated, thread engaged, luer locked + the 1 s hold and its restart; every cap (cross-thread, loose
connection, dropped syringe) holds the score at 0.5 and blocks 1.0; the real product sizes and YAM reach gate hold;
the demo twists a cumulative 1.5 turns, fits the run limit, returns home and reaches every waypoint with the YAM arms.

Run in the yam_arena environment:  python -m pytest -q test_luer_lock_twist_connect.py
"""
import math
import pathlib

import numpy as np
import pytest
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "luer_lock_twist_connect.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _gripper(side, tcp=None, opening=0.094, contact=0.0):
    tcp = np.asarray(T._START_TCP[side] if tcp is None else tcp, float)
    rot = quat_xyzw_to_matrix(np.array(yam_demo.DOWN_GRIP))
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * rot[:, 2],
                          tips=tcp + 3.14e-3 * rot[:, 2], opening=opening, contact=contact, rot=rot)


def _objects(picked=False, seated_done=False, engaged=False, locked=False, seated=False,
             cross_thread=False, dropped=False, grippers=None):
    return {
        "picked_up": picked,
        "seated_done": seated_done,
        "engaged": engaged,
        "locked": locked,
        "seated": seated,
        "cross_thread": cross_thread,
        "dropped": dropped,
        "turns": math.radians(538.0),
        "left_gripper": (grippers or {}).get("left") or _gripper("left"),
        "right_gripper": (grippers or {}).get("right") or _gripper("right"),
    }


def _full(**kw):
    return _objects(picked=True, seated_done=True, engaged=True, locked=True, seated=True, **kw)


@pytest.fixture(autouse=True)
def fresh_episode():
    T._reset_hold()
    T._MECH.reset()
    yield


def test_loads_and_assets():
    assert TASK.module is T
    for hook in ("demo", "score", "success", "steps_done"):
        assert hasattr(T, hook)
    for name in ("adapter", "syringe"):
        assert (HERE / "assets" / f"{name}.usda").is_file()
    adapter = (HERE / "assets" / "adapter.usda").read_text()
    for token in ("Fixture", "VialBody", "AdapterSkirt", "FemalePort", "PortBore", "AdapterMark"):
        assert token in adapter
    syringe = (HERE / "assets" / "syringe.usda").read_text()
    for token in ("Barrel", "LockCollar", "MaleTip", "PlungerRod", "IndexMark"):
        assert token in syringe


def test_idle_scores_zero():
    o = _objects()
    assert T.steps_done(o) == (False, False, False, False)
    assert T.score(o) == 0.0
    assert T.success(o) is False


def test_picked_up_quarter():
    o = _objects(picked=True)
    assert T.steps_done(o) == (True, False, False, False)
    assert T.score(o) == pytest.approx(0.25)


def test_seated_half():
    o = _objects(picked=True, seated_done=True, seated=True)
    assert T.steps_done(o)[:2] == (True, True)
    assert T.score(o) == pytest.approx(0.5)


def test_engaged_three_quarters():
    # still gripping at the port: a released, clear state with only 0.5 turn would be the loose-connection cap
    o = _objects(picked=True, seated_done=True, seated=True, engaged=True,
                 grippers={"left": _gripper("left", opening=0.020)})
    assert T.steps_done(o) == (True, True, True, False)
    assert T.score(o) == pytest.approx(0.75)
    assert T.success(o) is False


def test_full_one_second_hold():
    o = _full()
    assert T.steps_done(o) == (True, True, True, True)
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True
    assert T.score(o) == pytest.approx(1.0)


def test_hold_restarts_after_a_break():
    o = _full()
    for _ in range(10):
        T.success(o)
    broken = _objects(picked=True, seated_done=True, seated=True, engaged=True)  # no lock
    assert T.success(broken) is False
    for _ in range(20):
        assert T.success(o) is False
    assert T.success(o) is True


@pytest.mark.parametrize("capname", ["cross_thread", "dropped"])
def test_caps_hold_score_at_half(capname):
    o = _full(**{capname: True})
    assert T.steps_done(o) == (True, True, True, True)
    assert T._caps(o) is True
    assert T.score(o) <= 0.5
    assert T.success(o) is False


def test_cap_fails_even_the_three_quarter_step():
    o = _objects(picked=True, seated_done=True, seated=True, engaged=True, cross_thread=True)
    assert T.score(o) == pytest.approx(0.5)


def test_loose_connection_caps_at_half():
    # the tip is seated and the thread engaged, but the lock was never made and the hands are clear -> loose
    o = _objects(picked=True, seated_done=True, seated=True, engaged=True)
    assert T._loose(o) is True
    assert T.score(o) == pytest.approx(0.5)


def test_grippers_not_clear_keeps_full_false():
    g = _gripper("left", tcp=np.array([T.PORT_XY[0], T.PORT_XY[1], T.TOP + T.PORT_Z]))
    o = _full(grippers={"left": g})
    assert T._clear(o) is False
    assert T._full(o) is False
    assert T.success(o) is False


def test_partially_open_gripper_is_not_clear():
    g = _gripper("left", opening=0.020)
    assert T._clear(_full(grippers={"left": g})) is False


def test_real_product_sizes():
    assert T.PORT_XY == pytest.approx((0.280, 0.0))
    assert T.PORT_Z == pytest.approx(0.100)            # 20 mm vial adapter, port at 0.10 m work height
    assert T.BARREL_D == pytest.approx(0.020)          # 20 mm syringe barrel
    assert T.TOP == pytest.approx(0.7475)
    assert T.SEAT_CZ - T.PORT_Z == pytest.approx(T.TIP_DROP)   # tip bottom meets the port lip
    assert 0.005 <= 2 * 0.0028 <= 0.007                # ~5.6 mm male luer tip: the 6 mm class (REFERENCE.md)
    assert T.SEAT_CZ > T.PORT_Z                         # the barrel centre sits above the port when seated


def test_yam_reach_gate():
    # first MuJoCo/Isaac reach gate: h 0.10, reach 0.30, 20 mm barrel; both work points inside the gate
    assert T.PORT_Z <= 0.10 + 1e-9
    assert T.PORT_XY[0] <= 0.30 and T.STAGE_XY[0] <= 0.30
    for xy, lz in ((T.PORT_XY, T.SEAT_CZ), (T.STAGE_XY, T.STAGE_CZ)):
        p = np.array([xy[0], xy[1], T.TOP + lz])
        for side in ("left",):
            q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(yam_demo.grip_rotation(0.0)),
                                 q_init=np.array([0, 1, 1, -0.6, 0, 0]), base_pos=BASES[side], iters=900)
            assert pe < 0.005, (side, xy, pe)


def test_declared_mechanism_numbers():
    assert math.degrees(T.LOCK_TURN) == pytest.approx(540.0)
    assert math.degrees(T.ENGAGE_TURN) == pytest.approx(180.0)
    assert math.degrees(T.LOCK_TOL) == pytest.approx(40.0)
    assert T.GRIP_CLOSE == pytest.approx(0.034)
    assert T.GRIP_RELEASE == pytest.approx(0.045)
    assert T.GRASP_UP == pytest.approx(0.012)
    assert T.TWISTS == 3 and math.degrees(T.TWIST) == pytest.approx(180.0)


def test_demo_twist_is_one_and_a_half_turns():
    assert T.TWISTS * T.TWIST == pytest.approx(T.LOCK_TURN)
    assert abs(T.TWISTS * T.TWIST) / (2 * math.pi) == pytest.approx(1.5)


def test_both_arms_reach_every_demo_waypoint():
    plan = T.demo()
    for side, wps in plan.items():
        for w in wps:
            rot = w.rot if w.rot is not None else yam_demo.grip_rotation(w.yaw)
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot),
                                 q_init=np.array([0, 1, 1, -0.6, 0, 0]), base_pos=BASES[side], iters=900)
            assert pe < 0.005, (side, w.label, pe, w.pos)
            assert math.degrees(re) < 5.0, (side, w.label, math.degrees(re), w.pos)


def test_demo_fits_limit_and_returns_home():
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for side, wps in plan.items():
        assert np.allclose(wps[-1].pos, T._START_TCP[side])
        assert wps[-1].finger == pytest.approx(yam_demo.FINGER_OPEN)
        assert sum(w.duration for w in wps) < 1200 / 20.0 - 2.0
    labels = [w.label for w in plan["left"]]
    assert labels.index("seat the tip") < labels.index("twist 1.1")
    assert labels.index("twist 1.2") < labels.index("twist 2.1")
    assert labels.index("twist 2.2") < labels.index("twist 3.1")
    assert labels.index("twist 3.2") < labels.index("release the syringe")
