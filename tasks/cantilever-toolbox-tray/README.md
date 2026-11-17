# Open a cantilever toolbox's two latches and unfold its five trays

A bench task from a mechanic's day. A STANLEY 1-94-738 18 in. steel cantilever tool box (460 x 208 x 208 mm, 4.8 kg, five trays, two metal latches) stands closed on the station table, loaded with a 150 mm adjustable wrench in its left moving tray, a 120 mm screwdriver in its right moving tray and a 300 mm claw hammer in the upper fixed tray. The robot has to release both front draw latches by their knobs, then lift the lid's wire tray handle until the parallelogram arms have swung both moving trays fully out and the box has settled with the tools still in their trays. The instruction states only the goal. That the lid cannot rise until both latches are out, that the left tray hangs lower than the right, and that the lid's detent holds the trays open after the hand lets go are the robot's to work out.

**What is new against the board (live check 2026-11-17T19:31:21Z).** `gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state` returned 2,511 PRs, newest #2512. No live OPEN task has this object + mechanism + outcome. #576 (CLOSED) latches a toolbox **closed** before carrying it by its lid handle (the opposite outcome and no tray unfolding); #36 (MERGED) unloads a cantilevered plank (different object); #1665 (CLOSED) takes a telescope off a steel toolbox (different step); #2374 (OPEN) holds a window's tilt latches (different object and mechanism); #1734 (CLOSED) draws-latches an instrument case (different object). A different colour, size or start is not the difference.

| File | What it is |
|---|---|
| `toolbox_cantilever.py` | The `yam_arena` task file: scene, the free box articulation, the declared latch detents and lid-to-arm coupling, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/toolbox.usda` | The Stanley box: black base and fixed nested trays, two steel parallelogram arms, the two moving yellow trays and risers, the lid with its wire tray handle and keepers, and the two draw latches on revolute joints (free articulation root `Base`) |
| `assets/wrench.usda`, `assets/screwdriver.usda`, `assets/hammer.usda` | The three free tool bodies (150 mm adjustable wrench 0.35 kg, 120 mm screwdriver 0.15 kg, 300 mm claw hammer 0.50 kg) |
| `build_assets.py` | Writes the four USD files (pxr; boxes and cylinders; colliders are boxes/cylinders) |
| `test_toolbox_cantilever.py` | Offline checks (20): every scoring outcome, idle, one latch, the lid and open thresholds, the 1 s hold and its restart, dropped/pinched/forced/tipped caps, the stability window, both arms' reach at every demo waypoint, the plan and its timing, the asset's joints |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the sources, and every measured vs estimated number |

## Real product and values

The **STANLEY 1-94-738 18 in. Metal Cantilever Tool Box** is a 460 x 208 x 208 mm, 4.8 kg steel box with five compartments, metal latches and two carry handles; the manufacturer lists a 40 L capacity and a 35 kg load (Stanley product page). The product photo shows the two black carry bails folded down, the yellow trays, and the silver parallelogram straps with rivets on the ends. The retailer listings give the outside as 20.8 x 20.8 x 45 cm and the "double latch for greater strength" (Amazon/Klium); the Stanley page lists the latch material as metal. The user lifts the box by a bail and opens it on the bench: release the two draw latches, then lift the lid's tray handle; the arms fan the trays out.

**Measured (published):** 460 x 208 x 208 mm, 4.8 kg, five compartments, metal latches, two carry handles, 40 L, 35 kg load, one internal tote tray.

**Estimated (each one is in `REFERENCE.md` and in `task.json`'s fidelity):**
- base 105 mm, lid 13 mm, 0.40 kg lid, 4.74 kg box split 3.20/0.30/0.80/0.40/0.04 (base/arms/wings/lid/latches);
- moving trays 160 x 150 x 30 mm; fixed nested trays 420 x 180 x 25 mm; the right tray 40 mm higher than the left;
- two 149 mm arms pivoting at the base side rims (y +-0.225, z 0.107), swinging -130/+135 degrees, the tray leveling hinge standing in for the second parallel arm;
- two 35 mm draw latches with a 10 mm knob, a 12 mm forward pull and a 25 degree swing; the latch spring detent 0.25 N m/rad = about 7 N at the knob (the brief's 4-7 N);
- tool masses 0.35/0.15/0.50 kg and the tray positions;
- the latch, handle and tray positions from the product photos.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z. The box stands at **x 0.43, y 0**, its 460 mm length along y and its 208 mm front face toward the robot. The black base is 105 mm tall; the lid bottom is 195 mm and the closed lid top 208 mm. The five trays are the black base tray (floor 12 mm), two fixed nested trays inside it (16-65 and 65-90 mm), and two moving trays: the left at 100-130 mm and the right at 140-170 mm closed, each 160 x 150 x 30 mm with 2 mm walls. The lid has a wire tray handle; its 60 mm grip bar is at **x 0.370, y 0, z 0.9635**, and the lid rises 115 mm. The two latches hang on the front corner posts at **y +-0.160**; each 35 mm plate ends in a 10 mm knob at **x 0.308, z 0.9125**, and its hook sits under a keeper on the lid. The tools: the wrench at y 0.001, z 0.8475 (left tray), the screwdriver at y -0.001, z 0.8905 (right tray), the hammer at y -0.090, z 0.8255 (upper fixed tray). The box is free on four 30 x 40 mm rubber feet; the two YAM arms are at READY.

When open, the left arm tip is at y +0.310, z +0.225 and the left tray spans y +0.161 to +0.311, 225-255 mm above the table; the right arm tip is at y -0.325, z +0.217 and the right tray spans -0.331 to -0.181, 257-287 mm. The lid is at 310 mm. The trays are inside the 460 mm table footprint and the arms stay at their READY poses at the sides.

## Mechanism

The box is one articulation. `Base` is free on four rubber feet. `Top` is the lid on a vertical prismatic joint (0-115 mm). `ArmL`/`ArmR` are steel arms on revolute joints at the base side rims (0.149 m long; -130/+135 degrees open); each carries `WingL`/`WingR`, the moving tray, on a second revolute joint that cancels the arm's roll so the tray stays level. `LatchL`/`LatchR` are the two draw latches on revolute joints at the front corner posts (-30 to +5 degrees; -25 open), their hooks under the lid's keepers.

The latch is a real draw latch: the 35 mm plate hooks under a keeper on the lid, and its spring detent holds the plate closed (0.25 N m/rad, about 7 N at the 10 mm knob). The robot grips the knob with its fingers one above and one below, pulls it 12 mm forward; the plate swings 25 degrees and the hook slides out from under the keeper. The knob is a cylinder along y, so it turns under the pads while the hand keeps its orientation. Once the plate passes -10 degrees it is latched open; the other latch is independent. The lid cannot rise while either hook is under its keeper: the two hooks are the physical interlock. A lid forced up with a latch still closed sets the `forced` latch, caps the score at 0.5 and makes 1.0 impossible.

The lid's lift drives the two arms through a declared coupling: at lift L the arm targets are -130 * L/0.110 and +135 * L/0.110 (clamped), and the level joints cancel them. The real box's trays are linked to the lid by the parallelogram straps; the coupling stands in for those links, as the balance task's declared screw stands in for its thread. The arms, trays and level joints are real bodies with real hinges; a 0.4 kg lid and 0.75/0.60 kg loaded trays are carried by the drives. The 115 mm lid detent holds the trays open after the hand lets go.

The three tools are free rigid bodies in their trays. The tray floors and 2 mm walls carry them; the tray's leveling hinge keeps the floor level, and the slow 3.5 s unfold keeps the inertial force far below friction. A tool that leaves its tray sets `dropped` and caps the score at 0.5. A gripper pad inside the closing tray/lid gap sets `pinched` and also caps at 0.5.

## Demo

Open loop, known geometry. The right arm goes to the right latch knob at (0.308, -0.160, 0.9125), grips it with a 20-degree-down forward frame, pulls it 12 mm forward (the latch swings to -25 degrees), lets go and goes home. The left arm does the same on the left latch, then goes to the lid handle at (0.370, 0, 0.9635) with a 45-degree-down frame, grips the 8 mm bar and lifts it in three steps to 115 mm; the two trays swing out and stay level. The left hand lets go and goes home; the lid detent holds the open state. The right arm is home by 8 s, before the trays open; it never stands in their path.

Demo: **1.0 at step 359 (17.95 s after the settle), twice in MuJoCo and on Isaac**; the 1 s hold ends at
t=18.45 s in the task log. Isaac final: lift 115.0 mm, arms -130/+135 deg, both latches -25 deg, tools in their
trays, no fault. The latch step is reached at step 87 (t=4.85 s), the lid-lifted step at 240 (12.50 s), the trays-unfolded step at 252 (13.10 s) and the full state at 339 (17.45 s); the success is step 359 (17.95 s). Final: lift 114.2 mm, arms -130/+135 degrees, latches -25/-25 degrees, box tilt under 0.1 degree, base moved 1.1 mm, tools in their trays, no arm-box contacts at the scored state. Idle scores 0; one latch 0; both latches 0.25; lid lifted 0.5; trays open 0.75.

## Scoring

| Step | Done when | Points |
|---|---|---|
| both latches | each latch plate at least 18 degrees out | 0.25 |
| lid lifted | both latches out and the lid at least 30 mm up | 0.5 |
| trays unfolded | lid at least 105 mm up and both arms past -120/+130 degrees, all tools in their trays | 0.75 |
| full | that state held 1 s with the box stable (tilt <= 2 degrees, base <= 10 mm), all four feet down, both grippers clear (no grip, no pad within 50 mm of the box sides) | **1.0** |

- **0** otherwise, including doing nothing. The closed start scores 0; holding still does not score.
- **Caps:** a latch forced open without a hand on its knob, a tool dropped out of its tray, or a gripper pad inside the closing tray/lid gap caps the score at **0.5** for the rest of the run and makes 1.0 impossible. A box tilted over 5 degrees, moved over 30 mm or off the table scores **0**.
- The scored state is the box's own joint positions and body poses: the latch angles, the lid's prismatic joint, the two arm joints and the two wing level joints; the tools' poses in their tray frames. The tools' real masses and the box's free root are unchanged.
- **Real station:** latch angles with a protractor; lid height with a ruler; tray angles from the arm links; latch force with a small gauge (4-7 N at the knob); a stopwatch on the video for only the 1 s hold, after both hands are clear.

## Checks

```bash
cd benchmark-submission/tasks/cantilever-toolbox-tray
PYTHONPATH= ../../../local_sim/.venv/bin/python -m pytest -q test_toolbox_cantilever.py
cd ../../../local_sim
PYTHONPATH= .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/cantilever-toolbox-tray/toolbox_cantilever.py --steps 1200 --trace
```

- pytest: **20 passed**.
- MuJoCo: **1.0 at step 359, twice**; idle 0; one latch 0; both latches 0.25; lid lifted 0.5; trays open 0.75; dropped tool 0.5; pinch 0.5; forced latch 0.5; tipped 0; the stability window 0.75; the 1 s hold 1.0; nudge starts 0 or 0.25 at the start and 1.0 when open.
- Both arms reach every demo waypoint within 5 mm and 5 degrees (the handle top is the worst at 2 mm); the demo's left plan is 17.1 s and the right 7.9 s, both under the 60 s run limit.
- **Run limit: 2,400 steps (120 s at 20 Hz), the Dream evaluation budget.** The scripted demo needs about 359
  steps (18 s), well inside the budget.
- Isaac: **1.0 at step 359** (`adhoc_e4e52215.json`); both latches -25 deg, lift 115.0 mm, arms -130/+135 deg,
  tools in place, no fault; filmed with a top camera and two render-only latch close-ups (internal only).
- Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed, hit the step limit); the arms move but never pull
  the latches or lift the lid. Top-camera `dream.mp4`.
- **Pass-through check:** frames at 0, 10, 25, 45, 60, 80, 95 and 100 % of the demo, plus the wrist-camera close-ups at 4.0 s (left latch), 10.0 s (handle grip), 12.0 s (mid-lift) and 17.0 s (open). No gripper, pad, forearm or held part intersects the box: the only arm-body contacts are the two latch knobs and the handle bar, both intended, and the tools contact their tray floors/walls. The final contact list has no arm-toolbox pair. Cutaways: none.
- **Media:** `benchmark-media-cantilever-toolbox-tray/demo.mp4` (Isaac top view only, 640 x 360, 21.45 s with a
  3 s held final frame, 352,903 bytes), `demo_top.gif` (480 x 270, 6 fps, 1,672,925 bytes), `setup.png`
  (640 x 360, 31,036 bytes), `dream.mp4` (completed Dream-1 top camera, 320 x 176, 893,472 bytes).

## Reset

Close both latches (0 degrees), let the lid down (0 mm), both arms at 0 (trays folded and level), and put the wrench, screwdriver and hammer in their trays. Set the box at x 0.43, y 0, table top + 1 mm and let it settle onto its four feet; set both arms at READY. Wipe the feet and the bench before the trial.

## Status

MuJoCo demo: **1.0 at step 359, twice**; Isaac demo: **1.0 at step 359**; Dream-1: **0.0 at 2,400 steps**
(2,401 action rows). pytest: **20 passed**; fields and consistency pass. Media in
`benchmark-media-cantilever-toolbox-tray` is top-view only for the PR, with the Dream video and the internal latch
close-ups kept separate. `task.json` media M-KB3KQQ. No push or upload was made.
