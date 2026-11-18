# Tighten a 15 mm flat-pack furniture cam lock

A benchtop furniture-assembly task. A real 15 mm eccentric cam waits upright in a pocket beside a 16 mm face panel whose
15 mm blind bore is empty; a mating M6 connecting bolt lies on a pad and the mating panel sits 3 mm short of the joint.
The robot must drop the cam into its bore, insert the bolt square into the mating panel's 8 mm hole, engage the
screwdriver in the cam's cross recess and turn the cam about its vertical axis ~180 deg. The eccentric ramp draws the
bolt head in and pulls the two panels flush; the joint is scored after that flush state has held for 1 s with both
grippers home. A cam turned while out of its pocket, a bolt driven in crooked, or panels forced shut before the joint
is captured are damage latches and scored caps. The instruction states only the goal; the placement order, square
insertion, half turn and flush feel are the robot's to work out.

**Importance (routine, not occasional).** Furniture movers and assemblers do this at every flat-pack joint. BLS reports
**1,799,500 assemblers and fabricators**, **889,700 carpenters** (installing cabinets, fixtures and hardware) and
**2,940,300 hand laborers and material movers** (furniture movers included) in 2025 — each occupation individually
above 500,000 — and 15 mm eccentric cams are the dominant connector in ready-to-assemble furniture. Category:
**Impact on jobs**. The full case, sources and measured/estimated split are in `REFERENCE.md`.

**What is new against the board (fresh query 2026-11-17T21:58:40Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned **2,538 PRs, newest #2539** (466 OPEN, 1,865 CLOSED,
207 MERGED). No PR has this object + mechanism + outcome: one 15 mm cam and one bolt *placed by the robot*, then a
*carried* screwdriver turned ~180 deg to draw two panels flush, with cam-not-seated / cross-threaded / forced-flush
caps. The nearest PR, **#1520 (OPEN, three times not accepted)**, pre-installs two cams and bolts in a shelf-onto-cabinet
assembly, turns with screwdrivers standing in holders (two quarter turns with a regrip), and uses force/geometry failure
gates. It is documented as a near twin, not a duplicate, in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `flat_pack_cam_lock.py` | The `yam_arena` task file: scene, the three carried parts and the sliding mating panel, the seat/insert/turn/draw mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/fixture.usda` | The 16 mm face panel: 15 mm blind cam bore 24 mm from the edge, green edge arrow, cam pocket, bolt pad, screwdriver holder (visual-only, no colliders) |
| `assets/panel_b.usda` | The 16 mm mating panel: 8 mm bolt hole in its joining edge; slides toward the face panel as the cam draws it in (visual-only) |
| `assets/cam.usda` | The free 15 mm die-cast cam: drum, cross/PZ recess, eccentric side slot, red index dot and a cylindrical collider |
| `assets/bolt.usda` | The free M6 connecting bolt: 27 mm thread, stop collar, 25 mm projection, 7 mm head and a collider |
| `assets/screwdriver.usda` | The free PZ2 screwdriver: handle, shaft, cross tip and a collider |
| `build_assets.py` | Writes the five USD files with pxr (boxes, cylinders and a generated rim mesh; no outside meshes) |
| `test_flat_pack_cam_lock.py` | Offline checks (22): every scoring outcome and cap, the 1 s hold and its restart, real sizes, geometry helpers, reach of all demo waypoints, plan order and timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the routine-importance case, sources and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame-by-frame and `data.contact` evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query, the near twin #1520 and the nearest furniture-assembly PRs |
| `make_media.py` | Renders the internal top-view media into `benchmark-media-flat-pack-cam-lock/` |

## Real product and values

The product class is a **15 mm eccentric cam connector for ready-to-assemble furniture** (Hettich Rastex 15, Häfele
Minifix 15, Titus Cam5000, Würth cam-lock nut 15): a zinc die-cast cam dropped into a 15 mm blind bore in one panel and
an M6 connecting bolt with a 7 mm head in an 8 mm hole in the other's edge. Recognisable features: the 15 mm drum with
a cross/PZ drive recess and an arrow, an eccentric spiral track that catches the bolt head, a collar-stopped bolt, and
a ~180 deg (up to 210 deg) hand turn that pulls the panels flush. Catalogue and installer guidance gives the 15 mm
bore, the ~2-3 mm clamping stroke, the 180-210 deg turn and hand-tightening only; the brief gives the 0.3-0.6 N m
torque. Measured/given: 15 mm cam, M6 bolt, 8 mm hole, 7 mm head, 24 mm drilling distance, 16 mm panels, 3 mm start
gap, three failure modes. Estimated (each flagged in `REFERENCE.md`): 12 mm cam height, 27 mm thread / 25 mm
projection, 0.5 mm flush window, 0.35 s torque dwell, 20 deg cross-thread and 9 deg square windows. No size is scaled
down.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The face panel lies flat at **x 0.250, y 0.030**, its joining edge at y -0.040 and its empty 15 mm
bore at **(0.250, -0.016)**, 24 mm in from that edge (the standard B = 24 mm drilling distance). The mating panel sits
beside it with an 8 mm bolt hole at **(0.250, -0.030)** and a 3 mm gap. The cam waits in a pocket at **(0.250, 0.065)**,
the bolt (axis along +y) on a pad at **(0.210, 0.135)** and the screwdriver in a holder at **(0.325, 0.135)**. Both
arms start at READY; one arm does the whole task. Every grasp point lies about 0.05-0.08 m above the table and within
0.23 m of the left arm base.

## Mechanism

The cam, bolt and screwdriver are real gravity-free rigid bodies. A closed gripper within 11 mm of a loose part's
grasp point carries it (a declared carry, the family's device); a part already seated in its bore or hole is no longer
grabbable. The robot still has to reach, close, orient, place, engage and turn each part. The bolt lies along +y while
the wrist stays near-vertical (the fixed carry offset holds its axis) because the YAM wrist cannot insert a bolt
horizontally at bench height. The screwdriver tip must reach the cam's cross recess within 12 mm with the cam seated;
once engaged the cam follows the wrist's yaw. The 180 deg turn is done as 150 deg, wrist straightened, re-engage,
30 deg — the way a person regrips a screwdriver — because the wrist cannot yaw a full 180 deg at that height. Past
90 deg the eccentric draws the mating panel in; at 180 deg the gap is 0. The 0.3-0.6 N m torque is **declared, not
integrated**: it is read as a closed engaged tool dwelling 0.35 s at the flush joint. The panels, bore and hole are
visual-only with no colliders, and damage is scored by geometric flags (`cap_cam`, `cap_bolt`, `cap_panel`), as in the
family's other fit tasks.

## Demo

Open loop, known geometry, one arm (the left; the right stays home). The left arm: (A) picks the cam and lowers it
upright into the 15 mm bore; (B) picks the bolt and inserts it square into the mating panel's hole; (C) picks the
screwdriver and sets its tip on the cam recess; (D) turns the cam 150 deg, straightens the wrist, re-engages and turns
the last 30 deg, drawing the panels flush, then holds the declared hand torque; (E) sets the screwdriver back in its
holder and goes home. Every approach is from above.

**MuJoCo 1.0 at step 865 of 900 (43.25 s after the settle).** Step timeline: cam seated 127, bolt engaged 295, cam
turned 661, panels flush + torque 667; the 1 s hold completes at 865. Final: cam centred in its bore within 0.8 mm,
bolt in its hole within 6 mm, cam turn +179.5 deg, panel gap 0.0 mm, no cap latched, both grippers home. Idle 0;
seated 0.25; bolt engaged 0.5; turned 0.75. Three in-sim bad demos (temporary copies, removed after the check) each
blocked 1.0: cam left in its pocket latched `cap_cam` and scored **0.25**; a bolt driven 45 deg cross-threaded latched
`cap_bolt` and scored **0.25**; panels forced shut before the turn latched `cap_panel` and held the score at **0.5**.

## Scoring

| Step | Done when | Points |
|---|---|---|
| cam seated | cam centred in its bore, upright, at seated depth | 0.25 |
| bolt engaged | bolt centred in its hole, square, at inserted depth | 0.5 |
| cam turned | cam turned at least 165 deg with cam seated and bolt engaged | 0.75 |
| panels flush + torque held | gap at most 0.5 mm after the turn, 0.35 s declared hand torque, held 1 s, caps clear, grippers home | **1.0** |

Caps (0.5 ceiling): `cap_cam` (turning with the cam out of its pocket), `cap_bolt` (bolt inserted more than 20 deg off
the hole axis), `cap_panel` (panels pushed flush before the joint is captured, or the cam turned past 215 deg). Idle 0.
The 1 s hold is timed in the task's `success()`; a break restarts it and step points stop at 0.75.

## Media

`demo.mp4` (Isaac top view only, 640x360, 491,268 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
1,706,626 B) and `setup.png` (640x360, 28,477 B) are in `benchmark-media-flat-pack-cam-lock/`, together with
`dream.mp4` (completed Dream-1 top camera, 320x176, 874,405 B). Each PR file is under 1.87 MB; `task.json` media M-7CEH62.

## Gate status

**MuJoCo 1.0 at step 865; Isaac 1.0 at step 865** (`adhoc_10b21daa.json`); 22/22 offline tests pass, no live twin
(TWIN_CHECK). Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed). Fields and consistency pass;
`task.json` media M-7CEH62. No PR, upload or push was made.
