"""Offline checks for the pad-only adhesive wipe (no simulator): the 5 mm cell ladder, pad-placed 0.25, contact-only
removal, the hover and idle zeros, the dwell/soft state, a gritty-pad scratch, the dirty-pad redeposit, paste outside
the patch, the 1 s hold and a nudged pan.  Run:
  PYTHONPATH= ../../../local_sim/.venv/bin/python -m pytest -q test_adhesive_residue_removal.py
"""
import math
import pathlib

import numpy as np
import pytest
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "adhesive_residue_removal.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _pan(pos=(0.270, 0.0, 0.7505), quat=IDENT):
    p = np.asarray(pos, float)
    return {"pan": ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3),
                               ang_vel=np.zeros(3), low=p.copy(), high=p.copy())}


def _objects(t, contacts=(), left=True, pan_off=False, pan_pos=(0.270, 0.0, 0.7505), pan_quat=IDENT):
    out = _pan(pan_pos, pan_quat)
    out.update(episode_time=float(t), contacts=list(contacts), left_holding=bool(left), pan_off=bool(pan_off))
    return out


def _pad_contact(kind="pad2", x=0.0, y=0.0, force=1.0, ds=0.0, grit=False, dry=False, z=0.064, hx=0.038, hy=0.051):
    return T.Contact(kind, force, 0.0, ds, ((x, y, z),), pad_box=(x, y, hx, hy, z), grit=grit, dry=dry)


@pytest.fixture(autouse=True)
def fresh():
    T._RES.__init__()
    T._HOLD.update(t=None, pos=None, quat=None)
    for k in T._TOOL_LAST:
        T._TOOL_LAST[k] = None
    yield


def _soft_all():
    T._RES.dwell[:] = T.DWELL_S
    T._RES.soft_until[:] = 1e9
    T._RES.last_wet[:] = 0.0
    T._RES.wet[:] = 1.0
    T._RES.soft[:] = True


def test_loads():
    assert TASK.module is T
    assert callable(T.demo) and callable(T.score) and callable(T.success)
    assert T.NC == 128


def test_idle_is_zero():
    for k in range(40):
        T.score(_objects(k * 0.05))
    assert T.score(_objects(2.0)) == 0.0


def test_hover_removes_nothing():
    _soft_all()
    for k in range(40):
        T.score(_objects(k * 0.05, contacts=[]))
    assert T._RES.clean() == 0
    assert T.score(_objects(2.0)) == 0.0


def test_pad_wetted_and_placed_is_a_quarter():
    for k in range(10):
        s = T.score(_objects(k * 0.05, contacts=[_pad_contact(force=1.0)]))
    assert T._RES.placed and T._RES.wet_max() > 0.5
    assert s == 0.25
    assert T._RES.clean() == 0


def test_force_below_the_minimum_does_not_place():
    for k in range(20):
        s = T.score(_objects(k * 0.05, contacts=[_pad_contact(force=0.3)]))
    assert not T._RES.placed
    assert s == 0.0


def test_dry_wipe_does_not_wet_or_clean():
    for k in range(20):
        s = T.score(_objects(k * 0.05, contacts=[_pad_contact(force=1.0, ds=0.001, dry=True)]))
    assert T._RES.placed
    assert not T._RES.wetted
    assert T._RES.wet_max() == 0.0
    assert T._RES.clean() < 60
    assert s == 0.0


def test_dwell_softens_all_cells_and_a_wipe_removes_them():
    pad = _pad_contact(force=1.0)
    for k in range(420):  # 21 s
        T.score(_objects(k * 0.05, contacts=[pad]))
    assert int(T._RES.soft.sum()) == 128
    assert T._RES.clean() == 0
    for k in range(10):
        T.score(_objects(21.0 + k * 0.05, contacts=[_pad_contact(force=1.0, ds=0.001)]))
    assert T._RES.load['pad2'] > T.PAD_LOAD_MAX
    assert np.sum(T._RES.r < 0.5) >= 100
    for k in range(30):
        T.score(_objects(21.6 + k * 0.05, contacts=[_pad_contact(kind='pad3', force=1.0, ds=0.001)]))
    assert T._RES.clean() == 128
    before = T._RES.clean()
    for k in range(20):
        T.score(_objects(23.2 + k * 0.05, contacts=[]))
    assert T._RES.clean() == before


def test_dirty_pad_redeposits_and_caps_the_score():
    T._RES.r[:] = 0.0
    T._RES.placed = True
    T._RES.wetted = True
    T._RES.wet[:] = 1.0
    T._RES.load["pad2"] = 0.10
    T._RES.last_tool = "pad2"
    T._RES.last_pad = "pad2"
    T._RES.pad_time["pad2"] = 5.0
    T.score(_objects(0.05, contacts=[_pad_contact(ds=0.001)]))
    assert T._RES.r.max() >= T.REDEPOSIT - 1e-9
    assert T._RES.fresh_final_pad() is False
    assert T.score(_objects(0.10)) <= 0.5


def test_paste_outside_the_patch_scores_zero():
    T._RES.placed = True
    T._RES.wetted = True
    T._RES.wet[:] = 1.0
    T._RES.load["pad2"] = 0.10
    T.score(_objects(0.05, contacts=[_pad_contact(x=0.060, y=0.0, ds=0.001)]))
    assert T._RES.outside > T.OUTSIDE
    assert T.score(_objects(0.10)) == 0.0


def test_gritty_pad_scratch_latches_zero():
    T._RES.r[:] = 0.0
    T._RES.placed = True
    T._RES.wetted = True
    T._RES.wet[:] = 1.0
    T.score(_objects(0.05, contacts=[_pad_contact(grit=True, ds=0.001)]))
    assert T._RES.scratch
    T._RES.r[:] = 0.0
    assert T.score(_objects(0.10)) == 0.0


def test_ladder_and_fresh_final_pad():
    def set_state(n, final_load=0.0):
        T._RES.r[:] = 1.0
        T._RES.r[:n] = 0.0
        T._RES.placed = True
        T._RES.wetted = True
        T._RES.wet[:] = 0.0
        T._RES.soft[:] = False
        T._RES.scratch = False
        T._RES.outside = 0.0
        T._RES.last_tool = "pad3"
        T._RES.last_pad = "pad3"
        T._RES.pad_time["pad3"] = 0.5
        T._RES.load["pad3"] = final_load
        T._RES.pad_wiped["pad3"] = 0.02

    set_state(126)
    assert T.score(_objects(0.05)) == 1.0
    set_state(126, final_load=0.10)
    assert T.score(_objects(0.10)) == 0.5
    set_state(116)
    assert T.score(_objects(0.15)) == 0.75
    set_state(77)
    assert T.score(_objects(0.20)) == 0.5
    set_state(31)
    assert T.score(_objects(0.25)) == 0.25
    set_state(0)
    T._RES.placed = False
    assert T.score(_objects(0.30)) == 0.0


def test_one_second_hold_times_in_success():
    T._RES.r[:] = 0.0
    T._RES.placed = True
    T._RES.wetted = True
    T._RES.wet[:] = 0.0
    T._RES.soft[:] = False
    T._RES.scratch = False
    T._RES.outside = 0.0
    T._RES.last_tool = "pad3"
    T._RES.last_pad = "pad3"
    T._RES.pad_time["pad3"] = 0.5
    T._RES.load["pad3"] = 0.0
    T._RES.pad_wiped["pad3"] = 0.02
    ok = False
    for k in range(20):
        ok = T.success(_objects(k * 0.05))
        if k < 19:
            assert not ok
    assert ok


def test_nudged_pan_still_places_and_scores():
    q = np.array([0.0, 0.0, math.sin(math.radians(1.5)), math.cos(math.radians(1.5))])
    for k in range(10):
        s = T.score(_objects(k * 0.05, contacts=[_pad_contact(force=1.0)], pan_quat=q))
    assert T._RES.placed and s == 0.25
    T._RES.r[:] = 0.0
    T._RES.wet[:] = 0.0
    T._RES.last_wet[:] = -1e9
    T._RES.last_tool = "pad3"
    T._RES.last_pad = "pad3"
    T._RES.pad_time["pad3"] = 0.5
    T._RES.load["pad3"] = 0.0
    T._RES.pad_wiped["pad3"] = 0.02
    assert T.score(_objects(1.0, pan_quat=q)) == 1.0
