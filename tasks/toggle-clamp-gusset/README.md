# Clamp a 100 mm steel gusset square on its fixture

A bench task from a fabrication shop. A real 100 mm steel angle-bracket gusset waits upright in a staging tray on a
bench fixture; the fixture's locator nest (back wall, end stop and two pins) is bare, and two hold-down toggle
clamps stand open with their red handles. The robot has to pick the gusset by the top edge of its upright, seat it
square on the locators, then flip both clamp levers over-centre (4-7 N each) so their rubber jaws press the foot
down, leave it clamped square for 1 s, and go home. The instruction states only the goal: that the gusset must be
square on its pins, that each lever must pass its toggle point, and that a part must not be dropped are the robot's
to work out.

**What is new against the board (fresh query 2026-11-17T21:38:08Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned 2,534 PRs, newest #2535 (462 OPEN, 1,865 CLOSED, 207
MERGED). No live OPEN task has this object + mechanism + outcome. **#1879 (OPEN)** is a *jump-start with sprung
booster clamps* — electrical booster clamps and a connection order, a different object, mechanism and outcome;
it is documented in `TWIN_CHECK.md`, not treated as a twin. The nearest mechanical neighbours (#1219 release
clamps so a blank reaches locators, #2010 locating pins and a bracket, #1801 toggle-clamp cable tension) are all
CLOSED and different outcomes. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `toggle_clamp_gusset.py` | The `yam_arena` task file: scene, the free parts and their declared carry, the locator/seat and over-centre mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/fixture.usda` | The welded fixture, locator nest, clamp bases and gusset staging tray (visual only, no colliders) |
| `assets/gusset.usda`, `assets/clamp.usda` | The real, gravity-free gusset and toggle-clamp lever; one clamp asset is instanced twice |
| `build_assets.py` | Writes the three USD files (pxr; boxes and cylinders, colliders on the parts) |
| `test_toggle_clamp_gusset.py` | Offline checks (19): every scoring outcome and cap, the 1 s hold and its restart, grippers-clear, real sizes, all demo waypoints reachable, the plan and its timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the sources, the measured vs estimated numbers and the explicit importance case |
| `PASS_THROUGH_QA.md` | Frame-by-frame top/contact evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |
| `make_media.py` | Renders the internal top-view media |

## Real product and values

The product class is a **100 mm steel angle-bracket gusset** (90 degree L, 100 mm legs) held on a bench fixture by
**two 90 mm hold-down toggle clamps**, each taking a 4-7 N hand force to snap over-centre. The job is routine
fixturing before a tack, drill or grind: locate the part square, throw both clamps, then work on it. The true
foot/upright/clamp sizes are estimates within the product class and are flagged in `REFERENCE.md`; nothing is
scaled down.

**Measured (product class):** 100 mm gusset angle bracket; two hold-down toggle clamps with rubber jaws and an
over-centre lock; 4-7 N per clamp; a locator nest with back wall and pins.

**Estimated (each in `REFERENCE.md` and in `task.json` fidelity):** foot 100 x 30 x 4 mm, upright 100 x 4 x 100 mm,
210 g; lever ~90 mm with a 45 mm link and a 16 x 14 mm pad, 90 g; 160 x 240 x 45 mm fixture; 6 mm locator pins;
70 mm clamp pivot spacing; a declared 70 degree over-centre yaw with a 55 degree latch threshold; 7 mm seat
tolerance. No size, mass or force is scaled down.

## Why this matters (routine, not occasional)

Locating a bracket and throwing toggle clamps is a repeated production step, not a rare one-off. Official US BLS
Occupational Outlook Handbook 2025 employment: **437,700** welders/cutters/solderers/brazers (SOC 51-4121) and
**131,100** sheet-metal workers (47-2211), **568,800 combined — over 500,000 workers**. The frequency claim is
reasoned from the OOH duty descriptions, not measured; the caveat and both source URLs are in `REFERENCE.md`.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The fixture is at **x 0.28, y 0**; its locator nest is a back wall, end stop and two pins around
the gusset's inner corner at **local z 0.045**. The gusset waits upright in a tray at **x 0.28, y 0.10, z 0.045**.
The two clamp levers pivot at **x 0.245 and 0.315, y -0.037, z 0.080**, 70 mm apart so their open handles do not
touch. Both arms start at READY. Every grasp point lies 0.045-0.14 m above the table and within 0.42 m of an arm
base (YAM h 0.15 / reach 0.40 gate).

## Mechanism

The gusset and both levers are real gravity-free rigid bodies. While a gripper's fingers close across a grasp
point, the part follows the gripper (a declared carry, the same family device as the check valve's poppet/spring/
cap); the robot must still reach, close on, carry, orient and release each part. Squareness is real geometry: the
gusset's foot must lie along x and its upright along z (cos > 0.90) at the two pins. A clamp is "over-centre" when
its lever yaw passes 55 degrees while its grip is still at the pivot and the gusset is seated; the demo reaches 70
degrees, an 8.6 degree margin. Flipping a clamp onto an unseated gusset, releasing a lever between 0.25 and 0.85
of the flip, or dropping a part sets a cap. The fixture is welded and collider-free, so the jaw's press is a
declared latch, not a simulated contact force; the 4-7 N hand force is stated, not integrated.

## Demo

Open loop, known geometry. The **left arm** picks the gusset by the top edge of its upright, carries it to the nest
and lowers it square onto the pins. The **right arm** waits at home until the gusset is seated, then picks each
clamp lever by its red handle, yaws it 70 degrees over-centre in place, releases and lifts away. Both arms go home
and the assembly is held 1 s. Every approach is from above with the fingers closing across the grasp point.

**MuJoCo 1.0 at step 614 of 1,200** (31.20 s in the task log). Step timeline: gusset seated square 163, clamp A
over-centre 331, clamp B over-centre 499, full assembly held 1 s at 595, success at 614. Final clamp twists 70 and
69 degrees, gusset upright on the nest, both grippers home and clear. Idle 0; gusset seated 0.25; clamp A 0.5;
clamp B 0.75; shifted gusset, clamp short of over-centre and dropped part each cap at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| gusset seated square | foot on the locator pins, upright on the back wall | 0.25 |
| clamp A over-centre | lever yaw past 55 deg at its pivot, gusset seated | 0.5 |
| clamp B over-centre | same for the second clamp | 0.75 |
| clamped square + hold | assembly held 1 s, grippers clear and home | **1.0** |

Caps (0.5 ceiling): gusset shifted off its locators, a clamp released short of over-centre, a dropped part. Idle 0.

## Media

`demo.mp4` (Isaac top view only, 640x360, 415,200 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
1,442,451 B) and `setup.png` (640x360, 29,196 B) are in `benchmark-media-toggle-clamp-gusset/`, together with
`dream.mp4` (completed Dream-1 top camera). All PR files are under 1.87 MB; `task.json` media M-JLB9C2.

## Gate status

**MuJoCo 1.0 at step 614; Isaac 1.0 at step 614** (`adhoc_d16440a9.json`); 19/19 offline tests pass.
Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed). Fields and consistency pass; `task.json` media M-JLB9C2.
No push, upload or PR was made. No `BLOCKED.md`: the measured MuJoCo gate reached 1.0, so no blocker is recorded.
