# Snap an 8 mm panel push-clip into a mounting hole

A bench task from routine auto and appliance service. A fixed trim panel with an 8 mm mounting hole lies on the table;
a real nylon push-clip (20 mm cap, 8 mm stem, barbs near the tip, about 2 g) stands upright in a small tray beside it.
The robot picks the clip by its cap, carries it over the hole, aligns the stem, presses it straight down with a 4-7 N
hand press until the barbs snap behind the panel and the cap is flush, then pulls up on the seated cap with a 4-7 N
retention pull and holds it for 1 s. The instruction states only the goal; that a crooked press, a proud clip, a drop
or a forced fit is wrong is for the robot to work out.

**What is new against the board (fresh query 2026-11-17T22:00:58Z).** A fresh `gh pr list --repo
robocurve/benchmark-tasks --state all --limit 3000` returned **2,538 PRs**, newest **#2539** (466 OPEN, 1,865 CLOSED,
207 MERGED). No live OPEN task combines this object (an 8 mm panel push-clip), this mechanism (align + 4-7 N press +
barbs seated + 4-7 N pull-test) and this outcome. The flagged near twin **#1986** is **CLOSED, not merged**, and is a
bimanual splash-shield refit with a loose panel and no force gate or pull-test. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `panel_push_clip.py` | The `yam_arena` task file: scene, the carried clip, the press/seat/pull mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/panel.usda`, `assets/tray.usda`, `assets/clip.usda` | The fixed panel with its hole, the staging tray, and the real 8 mm gravity-free clip with barbs |
| `build_assets.py` | Writes the three USD files with pxr (boxes and cylinders) |
| `test_panel_push_clip.py` | Offline checks (24): every ladder rung and cap, the 1 s pull hold and its restart, gripper-clear/home, real sizes and force bands, every demo waypoint reachable, the plan and its timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its sources, and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Top-view frame-by-frame and position evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query, #1986 and the other near PRs |
| `make_media.py` | Writes the internal top-view `setup.png`, `demo.mp4` and `demo_top.gif` |

## Why this is routine, not occasional (importance case)

Fastening trim panels with push clips is a **routine, high-repetition service step** for a large workforce, not a rare
one-off. The US BLS counts roughly 805,600 automotive service technicians and mechanics (SOC 49-3023) and a further
large population of appliance repairers and maintenance workers — well over **500,000 workers** combined. Fender
liners, splash shields, door trims and appliance panels are secured by nylon push retainers, many per panel, and
refitting them after service is a drilled, repeated task. A clip put in crooked, left standing proud, dropped, or
forced past its stop fails to hold the panel and has to be redone, so the fit quality — alignment, a measured press,
a flush cap, and a pull that holds — is what this task tests. The headcount is an approximate public employment
level; the routine character of the skill, not an exact count, is the point.

## Real product and values

The product class is an **8 mm nylon panel push-clip (push-type retainer)**: an 8 mm hole fit, a 20 mm cap/head, a
roughly 20 mm stem with spring barbs near the tip, black or white nylon, about 2 g. Sources are public catalogue
specifications (Lawson KT11397/P53958, Auveco 17216, Lawson 1680277, Hi-Line SR5901, Disco 1861PK) plus the brief's
force envelope. Catalogue dimensions are used at true size; the 4-7 N press and the individual 2 g mass are from the
brief / flagged as estimates. Full split in `REFERENCE.md`.

**Measured (catalogues + brief):** 8 mm hole, 20 mm head, ~20 mm stem, nylon construction, 4-7 N press, two-arm-height
work at <= 0.30 m.

**Estimated (each flagged in `REFERENCE.md` and `task.json` fidelity):** 18 mm stem, 3 mm cap thickness, barbs flaring
to ~13 mm, 120 x 100 x 3 mm panel, 2 g clip mass, a declared 350 N/m press/pull travel (4 N = 11.43 mm, 7 N = 20 mm),
a 16 mm (5.6 N) retention pull, and the seat/alignment tolerances below 0.30 m work height.

## Layout

Positions are metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0 (world
z 0.7475). The fixed panel's hole is at **x 0.280, y 0**; the clip starts upright in its tray at **x 0.200, y 0.090**,
cap top at local z 0.030. The arm hovers with the cap top at z 0.026, presses at z 0.007 (4.55 N), then seats at
z 0.003 (5.95 N, cap flush). A clip is aligned within 10 mm / 12 degrees and seated within 4 mm in x/y, 2.5 mm in z
and 8 degrees of tilt. All grasps are at local z 0.013-0.054 (< 0.30 m) and within 0.55 m of the right arm base. Only
the right arm moves; the left stays at READY.

## Mechanism

The clip is a gravity-free rigid body with a declared carry: while the right gripper's fingers close across its cap the
clip follows the gripper, so the robot must still reach, close on, align, press and pull it. Once the barbs seat the
clip is **locked in the panel** and no longer follows the gripper, so an upward pull is measured as gripper travel
against a fixed clip (the barbs' retention), not an extraction. Panel and tray are welded, visual-only bodies with no
colliders, as in the family fixtures, which lets the gripper close at the cap and the clip pass through the hole.
Insertion and pull forces are a declared 350 N/m travel from the real gripper TCP displacement (4 N = 11.43 mm,
7 N = 20 mm), not contact dynamics. A cross-insert (press tilted past 12 degrees), a clip released in the hole with
barbs unseated, a drop, and a forced fit over 7 N are explicit score caps.

## Demo

Open loop, known geometry, right arm only. Pick the clip from its tray, carry it above the hole, dwell to align, press
to 4.55 N, press home to 5.95 N with the cap flush, let go, re-grip the seated cap, pull up 16 mm (5.6 N), hold the
pull for 1 s, let go and go home.

**MuJoCo 1.0 at step 423 of 1,000 (21.15 s after the settle; t=21.65 s in the task log).** Step timeline: aligned 128,
pressed 172, barbs seated 203, the 1 s pull hold completes at 337, and the grippers finish clear and home at 423.
Final state: cap 4.7 mm (1.7 mm proud of the 3 mm seat target, inside the 2.5 mm seat tolerance), 2.1 mm off-axis,
tilt 0.1 degrees, 5.4 N insertion, 5.0 N retention pull, no cap. Idle 0; aligned 0.25; pressed 0.5; barbs seated 0.75;
any cap caps at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| aligned | clip upright, centred on the hole, above the barbs | 0.25 |
| pressed | with that, a 4-7 N insertion into the hole | 0.5 |
| barbs seated | with that, barbs behind the panel and cap flush | 0.75 |
| pull-test held 1 s | with that, a 4-7 N pull held 1 s, clip intact, grippers clear and home | **1.0** |

Caps (0.5 ceiling): cross-insert, clip left unseated, drop, forced fit over 7 N. Idle 0.

## Media

`demo.mp4` (Isaac top view only, 640x360, 255,891 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
916,282 B) and `setup.png` (640x360, 27,710 B) are in `benchmark-media-panel-push-clip/`, together with `dream.mp4`
(completed Dream-1 top camera, 320x176, 746,237 B). Each PR file is under 1.87 MB; `task.json` media M-PG3VN7.

## Gate status

**MuJoCo 1.0 at step 423; Isaac 1.0 at step 423** (`adhoc_bc55c9f7.json`); 24/24 offline tests pass.
Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed). Fields and consistency pass; `task.json` media M-PG3VN7.
No PR, upload or push was made.
