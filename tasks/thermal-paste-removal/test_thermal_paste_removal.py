"""Offline scoring checks and MuJoCo demo/nudge checks for thermal-paste-removal.

    ../../../local_sim/.venv/bin/python -m pytest -q test_thermal_paste_removal.py

The offline checks replay control-step histories (no simulator): the pad's real contact force and its rubbing motion
drive the 122-cell paste layer. They cover idle, hover, a dry wipe, a saturated wipe's redeposit, paste on the
socket/pins, a scratch, the 0.25/0.5/0.75/1.0 ladder, the 1 s hold and the clear-pad end state. The last test runs
the scripted demo in mjkit at the nominal layout and three nudged starts (the 3 mm object jitter and a 10 degree yaw).
"""

import math
import os
import pathlib
import subprocess

import numpy as np
import pytest
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TASK = load(HERE / "thermal_paste_removal.py")
T = TASK.module
IDENT = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])


def quat_z(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, 0.0, math.sin(a), math.cos(a)])


def state(pos, quat=IDENT):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p.copy(), high=p.copy())


def pad(dx=0.0, dy=0.0, dz=0.0, yaw=0.0):
    """The wipe's contact face at the IHS top (dt 20 um into it for contact), or at an offset."""
    return state((T.CPU_X + dx, T.CPU_Y + dy, T.IHS_TOP - 0.00002 + dz), quat_z(yaw))


def gripper(tcp, opening=0.0198, contact=3.0):
    tcp = np.asarray(tcp, float)
    return T.GripperState(tcp=tcp, back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN,
                          opening=opening, contact=contact, rot=np.eye(3))


def held_gripper(p):
    """A gripper on the 20 mm applicator bar, 25 mm above the contact face."""
    return gripper(np.asarray(p.pos, float) + np.array([0.0, 0.0, T.PAD_TCP_ABOVE]))


def clear_gripper(side="right"):
    return gripper(np.array([0.10, -0.30, T.TOP + 0.20]))


def objects(p, right=None, left=None, contacts=(), t=0.0):
    out = {"wipe": p, "episode_time": float(t), "contacts": list(contacts)}
    if right is not None:
        out["right_gripper"] = right
    if left is not None:
        out["left_gripper"] = left
    return out


def tick(p, right, n=1, contacts=(T.Contact("pad", 0.8),), t0=0.0, move=None):
    """Run n control steps, calling score() then success() as the harness does; returns the last (score, success)."""
    t = t0
    last = (0.0, False)
    for k in range(n):
        t += 0.05
        q = move(k) if move else p
        last = (T.score(objects(q, right, clear_gripper("left"), contacts, t)),
                T.success(objects(q, right, clear_gripper("left"), contacts, t)))
    return last


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PASTE.reset()
    T._PASTE.last_yaw = None
    T._HOLD.update(t=None, pos=None, quat=None)
    yield


def test_loads():
    assert TASK.module is T
    assert T.NC == 122
    assert hasattr(T, "demo") and hasattr(T, "score") and hasattr(T, "success")


def test_idle_scores_zero():
    p = pad()
    assert tick(p, None, 40, contacts=(), move=lambda k: p)[0] == 0.0


def test_picked_wet_wipe_scores_quarter():
    p = pad()
    assert tick(p, held_gripper(p), 40, contacts=(), move=lambda k: p)[0] == 0.25
    assert T._PASTE.picked and T._PASTE.wet_seen


def test_hover_removes_nothing():
    p = pad(dz=0.0010)
    move = lambda k: pad(dx=-0.010 + 0.020 * k / 39, dz=0.0010)
    assert tick(p, held_gripper(p), 40, contacts=(), move=move)[0] == 0.25
    assert T._PASTE.clean() == 0


def test_contact_but_no_rubbing_removes_nothing():
    p = pad()
    assert tick(p, held_gripper(p), 40, move=lambda k: p)[0] == 0.25
    assert T._PASTE.clean() == 0


def test_wet_rub_removes_paste_and_climbs_the_ladder():
    # one slow pass across the 30 mm patch; 0.8 N is the demo's contact force
    move = lambda k: pad(dx=-0.014 + 0.028 * k / 79)
    score, _ = tick(pad(dx=-0.014), held_gripper(pad(dx=-0.014)), 80, move=move)
    # after this one pass most of the middle is clean; the score is at least the 60 % rung
    assert T._PASTE.clean() > 0
    assert score >= 0.5


def test_dry_wipe_removes_nothing_and_leaves_lint():
    p = pad()
    T._PASTE.pad_wet = 0.0
    move = lambda k: pad(dx=-0.010 + 0.020 * k / 39)
    tick(p, held_gripper(p), 40, move=move)
    assert T._PASTE.clean() == 0
    assert int(T._PASTE.lint.sum()) > 0
    # a dry wipe has not been picked wet: it cannot earn the 0.25 rung
    assert T._PASTE.wet_seen is False
    assert T.score(objects(p, held_gripper(p), clear_gripper("left"), [T.Contact("pad", 0.8)], 3.0)) == 0.0


def test_dirty_pad_redeposits_and_caps():
    # the pad starts saturated with 0.20 g of old paste (its limit is 0.12 g); every cell is clean before the pass
    T._PASTE.picked = True
    T._PASTE.wet_seen = True
    T._PASTE.r[:] = 0.0
    T._PASTE.wet[:] = 0.0
    T._PASTE.load = T.PAD_CAP + 0.08
    p = pad()
    move = lambda k: pad(dx=-0.010 + 0.020 * k / 39)
    tick(p, held_gripper(p), 40, move=move)
    assert T._PASTE.dirty_used is True
    assert T._PASTE.clean() < T.NC
    # the redeposit caps the run at 0.5 even though the pad touched only a few clean cells
    assert T.score(objects(p, held_gripper(p), clear_gripper("left"), [T.Contact("pad", 0.8)], 5.0)) <= 0.5


def test_paste_on_socket_caps_at_quarter():
    T._PASTE.picked = True
    T._PASTE.wet_seen = True
    T._PASTE.r[:] = 0.0
    T._PASTE.wet[:] = 0.0
    p = pad()
    sc, _ = tick(p, held_gripper(p), 5, contacts=(T.Contact("pad", 0.8), T.Contact("socket", 0.5)),
                 move=lambda k: p)
    assert T._PASTE.pins is True
    assert sc == 0.25


def test_scratched_ihs_caps_at_half():
    T._PASTE.picked = True
    T._PASTE.wet_seen = True
    T._PASTE.r[:] = 0.0
    T._PASTE.wet[:] = 0.0
    p = pad()
    sc, _ = tick(p, held_gripper(p), 5, contacts=(T.Contact("pad", 0.8), T.Contact("hard", 1.0)),
                 move=lambda k: p)
    assert T._PASTE.scratch is True
    assert sc == 0.5


def test_dry_hard_press_scratches():
    T._PASTE.picked = True
    T._PASTE.wet_seen = True
    T._PASTE.pad_wet = 0.0
    p = pad()
    tick(p, held_gripper(p), 5, contacts=(T.Contact("pad", 3.0),), move=lambda k: p)
    assert T._PASTE.scratch is True


def test_ladder_60_and_95_percent():
    T._PASTE.picked = True
    T._PASTE.wet_seen = True
    p = pad(dx=0.100, dy=0.100)  # on the bench, clear of the CPU
    r = clear_gripper("right")
    # 60 % clean exactly
    T._PASTE.r[:] = 1.0
    T._PASTE.r[:math.ceil(0.60 * T.NC)] = 0.0
    T._PASTE.wet[:] = 0.0
    assert T.score(objects(p, r, clear_gripper("left"), ())) == 0.5
    # 95 % clean
    T._PASTE.r[:] = 1.0
    T._PASTE.r[:math.ceil(0.95 * T.NC)] = 0.0
    assert T.score(objects(p, r, clear_gripper("left"), ())) == 0.75
    # 1.0 at every cell clean, dry, no lint, no faults; the pad and both grippers are clear
    T._PASTE.r[:] = 0.0
    T._PASTE.wet[:] = 0.0
    T._PASTE.lint[:] = 0
    T._PASTE.pins = T._PASTE.scratch = T._PASTE.dirty_used = False
    T._PASTE.load = 0.0
    sc, ok = tick(p, r, 1, contacts=(), move=lambda k: p)
    assert sc == 1.0 and ok is False
    # the 1 s hold: 20 more static steps
    sc, ok = tick(p, r, 20, contacts=(), move=lambda k: p, t0=0.05)
    assert sc == 1.0 and ok is True


def test_full_score_needs_clear_grippers_and_pad():
    T._PASTE.picked = True
    T._PASTE.wet_seen = True
    T._PASTE.r[:] = 0.0
    T._PASTE.wet[:] = 0.0
    p = pad()  # on the IHS: not clear
    assert T.score(objects(p, held_gripper(p), clear_gripper("left"), ())) < 1.0
    # the pad on the tray and a gripper still over the CPU: not clear
    p2 = state((0.46, -0.24, T.TOP + 0.006))
    near = gripper(np.array([T.CPU_X, T.CPU_Y, T.TOP + 0.12]))
    assert T.score(objects(p2, near, clear_gripper("left"), ())) < 1.0
    # both clear: 1.0
    assert T.score(objects(p2, clear_gripper("right"), clear_gripper("left"), ())) == 1.0


# -- the MuJoCo demo and its nudged starts ---------------------------------------------------------------------------
def _demo(stdout_env):
    env = os.environ.copy()
    env.update(stdout_env)
    env["MJ_CONTACTS_AT"] = ""  # no debug
    cmd = [str(ROOT / "local_sim" / ".venv" / "bin" / "python"), "-m", "mjkit.run",
           str(HERE / "thermal_paste_removal.py"), "--steps", "900", "--no-video"]
    p = subprocess.run(cmd, cwd=str(ROOT / "local_sim"), env=env, capture_output=True, text=True, timeout=240)
    return p


@pytest.mark.parametrize("nudge", [{"TP_NUDGE_X": "0", "TP_NUDGE_Y": "0", "TP_NUDGE_YAW": "0"},
                                   {"TP_NUDGE_X": "3", "TP_NUDGE_Y": "3", "TP_NUDGE_YAW": "0"},
                                   {"TP_NUDGE_X": "-3", "TP_NUDGE_Y": "-3", "TP_NUDGE_YAW": "0"},
                                   {"TP_NUDGE_X": "0", "TP_NUDGE_Y": "0", "TP_NUDGE_YAW": "10"}])
def test_mujoco_demo_is_one_and_survives_nudges(nudge):
    p = _demo(nudge)
    assert p.returncode == 0, p.stderr[-2000:]
    assert "task_score 1.0" in p.stdout, p.stdout[-3000:]
    assert "success at step" in p.stdout
