# Fit a two-piece rigid cervical collar on a training manikin

A bench task from emergency and first-aid care. A care-training manikin lies supine with its neck exposed; a
two-piece rigid cervical collar is staged on the bench beside it as a front chin-support shell and a back
occipital-support shell, each with two 30 mm side tabs. The robot seats the front shell under the chin, seats the
back shell behind the neck, then presses each side tab with a firm 4-7 N hand press until it clicks and stays
seated, leaving the collar correctly fitted. The instruction states only the goal: which shell goes where, that
the tabs must be pressed to a seat rather than slammed, and that a mis-seated or forced collar is wrong are the
robot's to work out.

**What is new against the board (fresh query 2026-11-17T21:54:36Z).** `gh pr list --repo
robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state` returned **2,538 PRs**, newest
**#2539** (466 OPEN, 1,865 CLOSED, 207 MERGED). No live OPEN task has this object + mechanism + outcome.
`cervical` matches **0** titles; of 17 `manikin` PRs none fits a two-piece collar; the queue's listed near
neighbour **#1719 "Jigsaw final piece" is CLOSED and unrelated**. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `cervical_collar_shells.py` | The `yam_arena` task file: scene, the two carried shells, the seat/tab mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/manikin.usda` | The welded, visual-only training manikin (base, shoulders, neck, head, chin) |
| `assets/front.usda`, `assets/back.usda` | The two real gravity-free rigid shells with 30 mm tabs, chin cup / occipital rim and foam liners |
| `build_assets.py` | Writes the three USD files with pxr (boxes, cylinders, spheres) |
| `test_cervical_collar_shells.py` | Offline checks (17): every scoring outcome and cap, the 1 s hold and its restart, gripper-clear, real sizes, all demo waypoints reachable, the plan and its timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, sources, and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Top-view frame-by-frame and position evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |
| `make_media.py` | Writes the internal top-view `setup.png`, `demo.mp4` and `demo_top.gif` |

## Why this is routine, not occasional (importance case)

Cervical-collar application is a **routine, repeatedly practised first-aid skill** for a large workforce, not a
rare one-off. EMTs and paramedics (US BLS OEWS SOC 29-2011, roughly 1.4e5 workers) and registered nurses (SOC
29-1141, roughly 3.1e6 workers) — well over **500,000 workers** combined, before adding nursing aides,
occupational-health staff and trained first-aid responders — encounter suspected spinal trauma regularly in
pre-hospital and emergency care. Fitting a rigid collar is a standing competency and a drilled procedure: the
front shell under the chin, the back shell behind the neck, and the side tabs locked with a firm, controlled
press. A loose, rotated or over-tightened collar fails its purpose, and an incorrectly forced one can harm the
patient, so the fit quality — right shells, right order, right tab force — is what this task tests. The numbers are
approximate public BLS employment levels; the skill's routine character, not an exact headcount, is the point.

## Real product and values

The product class is a **two-piece rigid cervical collar** (~300 mm across the tabs, 0.3 kg): a rigid front
chin-support shell, a rigid back occipital shell, foam liners and two 30 mm side tabs pressed together with a
4-7 N hand press. Sources are public catalogue/clinical-training descriptions of Philadelphia/Aspen/Miami-class
collars and the brief's force/size/mass envelope; the individual shell dimensions are stated estimates, none
scaled down. Full split in `REFERENCE.md`.

**Measured (product class + brief):** two rigid shells, chin cup + occipital rim, 30 mm side tabs, 4-7 N press,
~300 mm width, 0.3 kg total, two-hand fit at <= 0.3 m work height.

**Estimated (each flagged in `REFERENCE.md` and `task.json` fidelity):** front/back masses 0.14/0.16 kg; shells
~110 mm wide x ~65 mm tall, 6-10 mm thick with 3 mm foam; manikin neck radius 42 mm from local z 0.040 to 0.160;
head radius 55 mm; tab press travel 5.7 mm = 4 N to 10 mm = 7 N over a declared 700 N/m; shell seats at x 0.285
(front) and x 0.215 (back), y 0, z 0.120.

## Layout

Positions are metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The fixed manikin's neck axis is at **x 0.250, y 0**, neck local z 0.040-0.160, chin at local
z 0.160. The front shell starts at **x 0.205, y 0.095** and seats under the chin at **x 0.285, y 0, z 0.120**;
the back shell starts at **x 0.205, y -0.095** and seats behind the neck at **x 0.215, y 0, z 0.120**. Tab press
points are **x 0.250, y +-0.050, local z 0.132**. All grasps are at local z 0.012-0.150 (< 0.30 m) and within
0.55 m of an arm base. Both arms start at READY.

## Mechanism

The two shells are gravity-free rigid bodies with a declared carry: while a gripper's fingers close across a
shell's grasp point the shell follows the gripper, so the robot must still reach, close on, orient, carry and seat
it. The manikin and shells have no colliders (as in the family's welded fixtures), which lets the gripper close at
the grasp point. A shell counts as seated when its centre is within 12 mm in x/y and 10 mm in z of its target and
its tilt is under 20 degrees. Each tab seats when a gripper dwells at its press point 5.7-10 mm below rest: the
real gripper TCP displacement is measured in MuJoCo/Isaac, with a declared 700 N/m giving 4-7 N. A press deeper
than 10 mm is a forced fit; a press that leaves the tab zone without reaching 4 N is a tab miss. The two arms work
one at a time because they collide if both reach the neck together; both hands are still used and both end home.

## Demo

Open loop, known geometry, both arms used in sequence. **Left:** pick the front shell, seat it under the chin, go
home and wait. **Right:** pick the back shell, seat it behind the neck, go home and wait. **Left:** press the left
tab 8 mm (5.6 N), hold, go home. **Right:** press the right tab 8 mm, hold, go home. Every approach is from above
with the fingers closing across the part.

**MuJoCo 1.0 at step 830 of 1,200 (41.50 s after the settle; t=42.00 s in the task log).** Step timeline: front
shell seated 132, back shell seated 382, left tab engaged 556, right tab engaged 704; the full 1 s hold completes
at 830. Final state: both shells seated and upright (tilt < 3 degrees), both tabs pressed to ~5.6 N, no cap, both
grippers home and clear. Idle 0; front 0.25; front+back 0.5; left tab 0.75; neck-moves, tab-miss and forced-fit
each cap at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| front shell under chin | front shell seated under the manikin's chin | 0.25 |
| back shell behind neck | front seated and back shell seated behind the neck | 0.5 |
| left tab engaged | both shells seated and the left tab pressed at 4-7 N | 0.75 |
| right tab engaged + hold | right tab pressed at 4-7 N, held 1 s, collar intact, grippers clear and home | **1.0** |

Caps (0.5 ceiling): neck moves (shell knocked off the bench or jammed over 20 degrees), tab miss (press never
reached 4 N), forced fit (press over 7 N). Idle 0.

## Media

`demo.mp4` (Isaac top view only, 640x360, 596,043 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
1,793,353 B) and `setup.png` (640x360, 30,178 B) are in `benchmark-media-cervical-collar-shells/`, together with
`dream.mp4` (completed Dream-1 top camera, 320x176, 779,649 B). Each PR file is under 1.87 MB; `task.json` media M-MJNL2S.

## Gate status

**MuJoCo first gate: 1.0 at step 830; 17/17 offline tests pass; all `task.json` fields within limits; field and
consistency checks pass.** Isaac demo **1.0 at step 830** (`adhoc_4df9aa1d.json`); Dream-1 **0.0 at 2,400 steps**
(2,401 action rows, completed). No PR, upload or push was made.
