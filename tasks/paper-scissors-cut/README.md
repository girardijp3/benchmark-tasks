# Cut a 100 mm strip off a real A4 sheet with office scissors, leaving the offcut in the tray

An office/dexterity task with real stationery. A **Fiskars-class 8 in office scissors** (203 mm, 60 g, orange loop
handles, stainless blades) rests open on a grey mat. An **80 gsm A4 sheet** (210 x 297 mm, 0.10 mm, 5.0 g) lies on a
20 mm riser and a 1 mm green cutting mat, its far **100 mm overhanging a shallow letter tray**. A red printed line at
100 mm from the far edge marks the cut. The robot presses the kept sheet with its left hand, takes both handles with
its right gripper, **opens and closes the blades at the start of the line**, sweeps the blades down the line, lets the
separated strip drop into the tray, and puts the scissors back on their mat.

The scissors are a real two-link articulation on a Z hinge at the rivet (0.4 N m/rad spring, 3 N m drive limit). The
paper is two rigid bodies with a seam bond: the offcut is servo-held to the kept sheet until the blade tip has swept
the whole printed line with the blades closed. The bond then releases and the offcut falls into the tray. The cut
length, lateral deviation, blade height, paper movement and hand positions are measured from body poses; the sheet is
not pre-cut and the drop is not scripted.

**Result: the scripted demo reaches 1.0 at step 545 (27.25 s after the settle) in MuJoCo (3/3 runs) and on Isaac.**
The blade tip sweeps
212 mm (the 90 mm blade past the end of the 210 mm line) with a measured line deviation of 0.02 mm; the blade opens
10.1 degrees at the line start and closes to about 1.3 degrees for the cut; the offcut lands in the tray at
z 0.7535 m; the scissors return to (0.329, -0.465) on the mat; the kept sheet does not move; both grippers go home and
all is still for 1 s.

## Board / twin check (2026-11-17)

`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000` returned 2501 PRs; `gh search prs` was run for
"scissors", "paper scissors", "cut paper" and "office scissors", plus title searches for scissors/paper/cut/shear/tray.
No live twin that cuts paper with scissors was found. The nearest tasks are:

- **#1441 (MERGED) cut a cable tie off a bundle** — diagonal cutters on a plastic tie; one squeeze, no paper, no
  printed line, no strip, no catch tray. This task shears a sheet along a printed line with scissors.
- **#1978 (CLOSED) squeegee a wet upright glass pane dry** — a rubber squeegee on glass; no cutting at all.
- **#1851 (MERGED) hand-held rotary can opener** — a rotary wheel cutting a can rim; no flat sheet, no line, no
  scissors, no two-hand paper hold.
- **#1135 (OPEN) trim card on a guillotine cutter** and **#1245 (CLOSED) trim paper on a guillotine** — a single
  guillotine blade on a card/paper; no scissors, no scissor hinge, no shear along a line, no offcut tray.
- **#2197 (OPEN) wrap a gift box with pre-creased paper** — folding pre-creased paper, no cut.
- **#531 (CLOSED) tear a paper towel** — a hand tear, not a scissor cut; no line or tray.
- **#100 (CLOSED) scissor handoff**, **#2044 (CLOSED) take kitchen shears apart for cleaning**, **#2500 (OPEN) clear a
  cable-tie offcut** — handling, cleaning or a different offcut; none cut paper with scissors.

## Files

| File | What it is |
|---|---|
| `paper_scissors_cut.py` | The `yam_arena` task: scene, two-link scissor articulation, seam bond, scoring (`success`, `score`, `steps_done`), caps and the scripted `demo()` |
| `build_assets.py` | Writes the four USD assets with pxr (blade wedges, elliptical loop handles, grip plates/lips, sheet, riser, tray, mats) |
| `assets/scissors.usda` | 203 mm scissors: `ScissorA`/`ScissorB` links, `Shear` hinge, blades, handles, rivet; origin at the rivet |
| `assets/paper_main.usda`, `paper_offcut.usda` | The kept 210 x 197 mm sheet and the 210 x 100 mm strip, each 0.10 mm, with the printed red line halves |
| `assets/fixtures.usda` | 20 mm riser, 1 mm green mat, 6 mm-floored letter tray and the 30 mm riser with the grey scissor mat |
| `test_paper_scissors_cut.py` | 29 offline checks: ladder/caps, nudges, geometry, assets, demo plan, static reach and predicates |
| `task.json` | Board metadata; all field limits pass `submission_tools/check_fields.py` |
| `../../../benchmark-media-paper-scissors-cut/` | `demo.mp4`, `demo_top.gif`, `setup.png` (top view) |
| `REFERENCE.md` | Sources, measured/estimated values, mechanism model, calibration and limitations |
| `PASS_THROUGH_QA.md` | Contact audit from the final run: pairs, counts, minimum distances, filtered pairs and clearances |

## Real product and values

- **Fiskars 8 in office scissors** (Fiskars 01-004761 class): 203 mm overall, ~60 g, 90 mm stainless blade from the
  rivet, 10 mm orange plastic loop handles. Trial item is a normal pair of office scissors (~$12–20).
- **80 gsm A4 paper**: 210 x 297 mm, 0.10 mm thick, 4.99 g; the 100 x 210 mm strip is 1.68 g. The printed line is
  100 mm from the long edge; a red rule and two end ticks mark the 210 mm cut.
- **Tray and mats**: a shallow letter tray (404 x 264 mm opening, 6 mm floor, 12.5 mm rim), a 20 mm riser with a
  1 mm green cutting mat, and a 30 mm rest with a 1 mm grey mat. All cheap stationery; trial well under $50.

## Layout

World frame: x forward, y to the robot's left, z up; table top 0.7475 m. The printed line is at **y = -0.150**, from
**x = 0.360 to 0.570**, on the sheet top at z 0.76855. The kept part is centred at (0.465, -0.0515); the offcut at
(0.465, -0.200); the left hand presses at (0.420, 0.040). The tray opening is x 0.318–0.722, y -0.408 to -0.144,
floor 0.7535, rim 0.7600. The scissors' rivet starts at (0.330, -0.465, 0.7798) on the rest mat; the arms start at
READY. The riser stops 50 mm short of the line so the right gripper pads and the upper blade clear it.

## Mechanism

1. **Scissors**: `ScissorA` is the root link (upper blade and its handle); `ScissorB` is the child on the `Shear` Z
   revolute at the rivet. A USD drive (stiffness 0.4 N m/rad, damping 0.02, max 3 N m) is the return spring. The
   gripper's closing squeeze drives the hinge target (94 mm gripper open = fully open, 50 mm = shut).
2. **Grasp**: while the right pads are on the handles, a servo grasp holds the rivet 74 mm ahead of and 13.5 mm below
   the commanded TCP with the blade bisector along world +X. This absorbs the arm's 1–2 mm tracking error so the
   blade follows the commanded line; the hinge remains a free, hand-driven joint. Opening the hand releases the tool.
3. **Cut**: the sheet is a kept body plus an offcut body, held together by a rigid seam servo. The tip is the midpoint
   of the two 90 mm blade tips. When the blades are shut (<= 5.7 degrees), the tip and rivet are at the paper plane and
   along the line, the tip sweeps from x 0.360 past x 0.570, and the sheet has not been pulled, the seam releases. The
   offcut is then given a downward velocity and falls 15 mm onto the tray floor.
4. **Faults**: a > 2 mm line deviation, a seam that pulls apart mid-cut, or the kept sheet shifting > 4 mm / 3 degrees
   caps at 0.5; a dropped scissors, a jammed hinge (> 0.8 s with the hand shut but the blades open), or a blade below
   the tray rim over the tray caps at 0.25.
5. **Finish**: the offcut must lie wholly in the tray footprint below the rim, the scissors must be back on the grey
   mat with open blades and near-zero speed, both grippers home and clear, and everything still for 1 s.

## Demo

The left hand presses the sheet from t=0 to about t=20.3 s. The right hand takes the scissors at about t=5.8 s, lifts
them, carries them to the line, aligns at x 0.360 at about t=10 s, opens the blades to 10.1 degrees, closes them, and
sweeps the tip from 0.360 to 0.482 m over 4 s (the 90 mm blade reaches x 0.572). The seam releases at about t=18 s, the
offcut drops into the tray, the scissors are carried back, released on the mat at about t=22.6 s, and both arms go
home. The 1 s still hold completes at step 545.

## Scoring

| Step | Done when | Points |
|---|---|---|
| scissors taken | right pads on the handles, in contact, scissors lifted off the mat | 0.25 |
| sheet held, start | left hand pressing, scissors at the line start, blades open | 0.5 |
| cut along line | blades shut and tip swept the full 210 mm line, seam intact, sheet unmoved | 0.75 |
| full | cut within +-1 mm, offcut in tray, scissors on mat, grippers home, 1 s still | **1.0** |

* Doing nothing is 0. A 1–2 mm cut is a counted cut (0.75) but cannot score 1.0; a > 2 mm cut caps at 0.5.
* Severe faults (dropped tool, jam, blade into the tray) cap at 0.25; a torn seam or a moved kept sheet caps at 0.5.
* The kept sheet is a rigid body, so "uncrumpled" is scored as no shift/tilt; a real trial checks the sheet flat.

## Checks

```bash
cd benchmark-submission/tasks/paper-scissors-cut
PYTHONPATH=../../../local_sim ../../../local_sim/.venv/bin/python -m pytest -q
cd ../../../local_sim
PYTHONPATH=. .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/paper-scissors-cut/paper_scissors_cut.py --steps 620 --no-video
python3 ../../submission_tools/check_fields.py paper-scissors-cut
```

* pytest: **29 passed**.
* Fields: all limits pass; instruction 32 words, keywords string 185/200.
* MuJoCo: **1.0 at step 545, 3/3**; final cut front 211.97 mm, deviation 0.02 mm; offcut in tray; scissors on mat.
* Static IK reach at the four critical TCPs: < 1 mm / 2 degrees (see pytest).
* Pass-through audit: see `PASS_THROUGH_QA.md`.

## Media

The prepared top-view package (outside the task folder) contains:

* `setup.png` — top view at t=0: sheet, red line, scissors on the grey mat, empty tray, arms at READY.
* `demo.mp4` — Isaac top view only, 640 x 360, 20 fps, 482,474 bytes (< 1.87 MB), 615 frames with the final **1.0**
  frame held for 3 s. No close-up in the PR media.
* `demo_top.gif` — 480 x 270 top-view preview, 4 fps, 1,579,884 bytes (< 1.87 MB).
* `dream.mp4` — the completed Dream-1 top-camera run, 320 x 176, 877,221 bytes, 2,401 frames / 120.05 s.

## Status

- MuJoCo demo: **1.0 at step 545 (27.25 s), 3/3**; no loosened scoring.
- Isaac demo: **1.0 at step 545** (`adhoc_75eed636.json`); cut front 212.0 mm, deviation 0.0 mm, full-line release,
  offcut in tray, scissors back on the mat. The sheet/offcut are declared collision-free on Isaac (see
  `PASS_THROUGH_QA.md`); the cut measurement and release gate are unchanged.
- Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed, hit the step limit); top-camera `dream.mp4`.
- Media: top-view `demo.mp4`/`demo_top.gif`/`setup.png` plus the separate top-camera `dream.mp4`; `task.json` media M-JCPM86.
- No push, upload or PR was made.
