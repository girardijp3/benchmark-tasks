# Fit a 20 mm spherical-OD bearing insert into a pressed housing

A benchtop machine-maintenance / robot-service task. A real 20 mm spherical-OD (self-aligning) bearing insert waits
upright in a pocket beside a pressed pillow-block-style housing. The housing's concave spherical seat has its axis
inclined 8 deg from vertical (it is bolted to a sloped conveyor frame) and carries one green grease port at a fixed
azimuth. The robot must lift the bearing, tilt its bore axis onto the inclined seat axis, rotate it about that axis so
its grease hole registers with the port, lower it flush into the seat with a light 4-7 N press, release it and go home.
A steep entry jams the rim, a press with the hole unregistered scores the bore, and driving the bearing below flush is
a forced press. The instruction states only the goal; the tilt, port and press are the robot's to work out.

**Importance (routine, not occasional).** Industrial machinery mechanics, machinery maintenance workers and
millwrights held **547,300 US jobs in 2025** (BLS, ~51,900 openings a year; SOC 49-9041 / 49-9043 / 49-9044), and
robot-maintenance techs add more. Replacing a failed spherical-OD insert bearing in a pillow-block housing on a
conveyor or automation line is scheduled preventive/corrective maintenance — recurring work of a large occupation, not
a one-off. Category: **Impact on jobs**. The full case, sources and measured/estimated split are in `REFERENCE.md`.

**What is new against the board (fresh query 2026-11-17T21:28:18Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned **2,533 PRs, newest #2534** (461 OPEN, 1,865 CLOSED,
207 MERGED). No PR in the whole board has this object + mechanism + outcome: a spherical-OD bearing insert tilted and
rotated to a matched spherical seat and pressed flush with jam/score/force caps. The nearest bearing PR, **#1880
(MERGED), sets a telescoping gauge to a bearing housing's bore** — a measurement task on a stand, not an insert fit.
Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `bearing_insert_tilt.py` | The `yam_arena` task file: scene, the free bearing and its declared carry, the tilt/index/seat/press mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/housing.usda` | The pressed housing: base, cast boss, inclined concave spherical seat, seat rim, green grease port and staging pocket (visual-only, no colliders) |
| `assets/bearing.usda` | The free 20 mm spherical-OD bearing: spherical outer band, 8 mm bore, two grease holes 180 deg apart, red index dot and a cylindrical collider |
| `build_assets.py` | Writes the two USD files with pxr (generated revolved meshes; no outside meshes) |
| `test_bearing_insert_tilt.py` | Offline checks (21): every scoring outcome and cap, the 1 s hold and its restart, gripper-clear, real sizes, tilt/index geometry and symmetry, all demo waypoints reachable, the plan and its timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the routine-importance case, sources and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame-by-frame and `data.contact` evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |
| `make_media.py` | Renders the internal top-view media into `benchmark-media-bearing-insert-tilt/` |

## Real product and values

The product class is a **radial insert ball bearing with a spherical outside surface** for a matched spherical housing
seat (ISO 9628:2019 inserts / ISO 3228:2013 housings, UC/UCP and SY/P2B families): the outer ring's spherical OD mates
a concave spherical bore in the housing so the unit self-aligns for shaft misalignment. Recognisable features: convex
spherical outer ring, wide inner ring with set-screw locking, two lubrication holes 180 deg apart in one plane of the
outer ring, a grease fitting in the housing, and a one-piece pillow-block or flange housing. The bearing is a
replaceable wear part fitted to the existing housing (NTN/Y-bearing catalogues). Catalogue/standard literature gives
the self-aligning geometry, the two-hole lubrication layout and a typical aligning angle of about 3-5 deg; the brief
gives the 20 mm size and the 4-7 N press. Measured/given: 20 mm spherical OD, spherical seat, 4-7 N press, jam and
scoring failure modes. Estimated (each flagged in `REFERENCE.md`): 8 mm bore, 7 mm width, 30 g, 8 deg seat inclination,
60 deg port azimuth, 10.3 mm seat radius, 4 deg/15 deg alignment windows, 0.35 s press dwell. No size is scaled down.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The pressed housing is fixed at **x 0.25, y 0**. Its concave spherical seat has an axis inclined
**8 deg in +x**; its green grease port is at a fixed azimuth **60 deg about the seat axis** from the bearing's default
index direction. The bearing waits upright in a pocket at **x 0.25, y 0.062**, centre z 0.010. The correct seated
centre is **x 0.25, y 0, z 0.024**, with the bore axis on the inclined seat and the grease hole on the port. Both arms
start at READY. Every grasp point lies about 0.07-0.09 m above the table and within 0.40 m of an arm base.

## Mechanism

The bearing is a real gravity-free rigid body. While the gripper's fingers close across its grasp point it follows the
gripper (a declared carry, the family's device); the robot must still reach, close on, tilt, rotate, carry and seat it.
The spherical outer band, concave seat, seat rim, grease port and index hole are real geometry. The robot tilts the
wrist 8 deg to put the bore axis on the seat axis, then rolls it 60 deg about that axis to bring the index hole to the
port; the two grease holes 180 deg apart make half-turn phases equivalent. A bearing entering more than 6 deg off the
seat axis sets `jammed`; one pressed with the hole outside 15 deg sets `scored`; one pushed more than 4 mm below flush
sets `forced`. A light 4-7 N press is a closed gripper dwelling 0.35 s at the flush seat. The 4-7 N force is **declared,
not integrated** into the dynamics, and the housing is visual-only with no colliders; the caps are geometric state
flags, as in the family's other fit tasks.

## Demo

Open loop, known geometry, one arm (the left; the right stays home). The left arm: (A) grips the upright bearing in its
pocket, closes and lifts it clear; (B) carries it over the seat and tilts its axis 8 deg to the seat axis; (C) rotates
it 60 deg about that axis to register the grease hole with the port; (D) lowers it flush, holds the light press,
releases and lifts away; (E) goes home. Every approach is from above.

**MuJoCo 1.0 at step 352 of 900 (17.60 s after the settle).** Step timeline: bearing lifted 122, tilted to the seat
axis 160, grease hole registered 197, seated flush + pressed 232; the 1 s hold completes at 352. Final: bearing at
x 0.2505 y 0.0007 z 0.7720 (within 0.5 mm of the flush seat), tilt error 0.2 deg, index error 0.0 deg, no cap latched,
both grippers home. Idle 0; lifted 0.25; tilted 0.5; registered 0.75; jam, score and forced press each cap at 0.5.
Three checks were run: the clean demo and a start pose moved +3 mm in x/y/z and +3 deg both reached 1.0 at step 352;
an upright entry tripped `jammed` + `scored` (0.25), and an 8 mm over-press tripped `forced` (0.5).

## Scoring

| Step | Done when | Points |
|---|---|---|
| bearing lifted clear | bearing raised clear of its staging pocket | 0.25 |
| tilted to seat axis | over the seat, bore axis within 4 deg of the inclined seat axis | 0.5 |
| grease hole registered | index within 15 deg of the port (two holes 180 deg apart) | 0.75 |
| seated flush + pressed | flush at the seat centre with a 0.35 s light press, held 1 s, caps clear, grippers home | **1.0** |

Caps (0.5 ceiling): bearing jams on a steep entry, housing scored by an unregistered press, forced press below flush.
Idle 0. The 1 s hold is timed in the task's `success()`; a break restarts it and step points stop at 0.75.

## Media

`demo.mp4` (Isaac top view only, 640x360, 231,974 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 5 fps,
1,274,731 B) and `setup.png` (640x360, 27,881 B) are in `benchmark-media-bearing-insert-tilt/`, together with
`dream.mp4` (completed Dream-1 top camera, 320x176, 843,350 B). Each PR file is under 1.87 MB; `task.json` media M-K4G9S4.

## Gate status

**MuJoCo 1.0 at step 352; Isaac 1.0 at step 352** (`adhoc_7e0015ad.json`); 21/21 offline tests pass, no live twin
(TWIN_CHECK). Dream-1: **0.25 at 2,400 steps** (2,401 action rows, completed), the model's partial attempt.
Fields and consistency pass; `task.json` media M-K4G9S4. No PR, upload or push was made.
