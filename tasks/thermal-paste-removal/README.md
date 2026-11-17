# Clean the old thermal paste off a desktop CPU's heat spreader with an alcohol wipe

A PC-repair bench. A real ATX motherboard sits on 6.5 mm brass standoffs on a dark anti-static mat, with an AM4-class
CPU latched in its socket and its nickel-plated IHS up. The cooler has been lifted off, leaving a 30 x 20 mm grey patch
of old thermal paste. On a foil tray at the near left lies a folded 91% IPA lint-free wipe on a small bench applicator;
a waste tray waits at the near right. The instruction states only the goal: clean the paste off and leave the heat
spreader dry and lint-free.

The core is a wet-wipe coverage process with three hazards a real technician watches: the socket frame, load plate and
lever around the IHS must not be touched (paste or a bent contact there is worse than a smudge), the IHS must not be
scratched, and the wipe must not redeposit paste from a loaded face. The paste is a 122-cell, 2 mm coverage layer on the
IHS. A cell is removed only when the wipe's soft contact face is really on the lid (the live MuJoCo pad-CPU normal force
is at least 0.12 N, the face flat within 20 degrees) and rubbing across it; a hover, a pad held still, or a pad dragged
off the lid removes nothing.

| File | What it is |
|---|---|
| `thermal_paste_removal.py` | The `yam_arena` task file: scene, the paste/wipe model, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/bench.usda` | The bench: anti-static mat, wipe foil tray, waste tray, paste tube, CPU box, screwdriver, wipe packet, anti-static bag (fixed) |
| `assets/board.usda` | 305 x 244 mm ATX motherboard: AM4 socket frame, load plate, steel lever, 54 x 90 mm cooler holes, DIMMs, 24-pin ATX, VRM heatsinks, chipset, PCIe, M.2, capacitors, I/O shield (fixed) |
| `assets/cpu.usda` | AM4 CPU: 40 x 40 mm package, 37.5 x 37.5 x 3.6 mm nickel-plated IHS, copper foot, etched lid, pin-1 triangle, gold substrate edge (fixed in the socket) |
| `assets/paste.usda` | The old paste: 122 grey 2 x 2 x 0.05 mm cells inside a 30 x 20 mm ellipse on 0.5 mm slides (visual only) |
| `assets/wipe.usda` | The folded 50 x 50 x 2 mm lint-free wipe (24 x 24 mm contact face on a 50 x 50 mm flange) on a 40 x 40 x 6 mm backing and 20 x 20 x 26 mm grip bar; free body |
| `build_assets.py` | Writes the USD files with `pxr` (boxes, closed lathes, convex prisms) |
| `test_thermal_paste_removal.py` | Offline scoring histories plus the full MuJoCo demo at the nominal layout and three nudged starts |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | Real products, sizes, sources and the modelling choices |

## Run a trial

MuJoCo (this PC):

```bash
cd local_sim && .venv/bin/python -m mjkit.run \
  ../benchmark-submission/tasks/thermal-paste-removal/thermal_paste_removal.py --steps 900 --trace
```

The demo scores 1.0 at step 658 (32.9 s after the 0.5 s settle). The live log prints the clean cells, the wipe's IPA
and paste load, the pad-IHS force, the rubbing distance and every latch.

Pytest (the ROS pytest plugins in this environment need this variable):

```bash
cd benchmark-submission/tasks/thermal-paste-removal && \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../../../local_sim/.venv/bin/python -m pytest -q test_thermal_paste_removal.py
```

17 tests pass: 13 offline scoring histories and 4 full MuJoCo runs (nominal, two 3 mm object jitters, one 10 degree
yaw). The MuJoCo runs use `--no-video`; the media below is rendered separately.

On the Isaac Lab-Arena YAM station the task file is the same `yam_arena` file; the finisher must read the pad-CPU
normal force from the PhysX contact reports (this build's live contact reader is the MuJoCo solver path). The scoring
rule and the geometry do not change.

## Scoring

| Step | Done when |
|---|---|
| 1. Wet wipe (0.25) | a gripper holds the wipe by its 20 mm applicator bar (16-24 mm opening, a real grip force) and the wipe is wet (its IPA wetness was above 0.30 at some point) |
| 2. Most paste (0.5) | at least 60 % of the 122 paste cells are clean |
| 3. 95 % (0.75) | at least 95 % of the cells are clean |
| 4. Clean and dry (1.0) | every cell clean, the IPA film dry (max cell wetness <= 0.05), no lint, no paste on the socket/pins, no scratch, the wipe below its 0.12 g paste load, and the wipe and both grippers at least 6 cm from the CPU for 1 s |

- **0** until the wet wipe is picked up; doing nothing is 0. A dry wipe removes nothing (and can leave lint, which a wet pass lifts), so it cannot reach step 1.
- **Caps:** pad contact with the board/socket, or with the CPU outside the IHS, is paste on the pins/contacts and caps
  the run at **0.25**. A scratch caps it at **0.5**. A wipe loaded above 0.12 g redeposits paste on clean cells, is
  flagged, and caps the run at **0.5**. Caps apply after the ladder.
- **1.0** only once every full condition holds together for 1 s, timed in `success()` (the CPU is fixed; the wipe must
  be still within 2 mm and 2 degrees). The wipe and both grippers must be clear of the CPU for the second.
- **A clean cell** is one with less than 2 % of its paste left. The patch is a 30 x 20 mm ellipse of 2 x 2 mm cells
  (122 cells) on the 37.5 mm IHS.
- **A cell is removed only by contact and rubbing**: the pad's real normal force on the CPU is at least 0.12 N, the
  pad's contact face is flat within 20 degrees, the pad moved at least 0.2 mm across the lid since the previous control
  step, and the wipe is wet. The removal is 0.06 of a cell per mm of rubbing per sqrt(N). The live scorer reads the
  force from MuJoCo's contact solver, not a scripted flag.
- **Real station:** the same rules with the eyes and a loupe: the IHS under a raking light for paste streaks and
  scratches; the socket frame, load plate, lever, pins and the CPU's substrate/pin field inspected for grey paste; the
  wipe's soiled face checked (a clean final face; a white final wipe comes away clean); the IHS dry after the IPA has
  flashed off (2-5 min). The sim's contact force and rubbing are the real instrument's stand-in.

### What the model does

- The pad's **normal force** and the pad's **pose** are read every control step. The force is the sum of the pad-CPU
  contacts from MuJoCo's solver; the pad's pose gives the footprint and the rubbing distance. The 122 paste cells are a
  coverage layer: a 2 mm grid on a 30 x 20 mm ellipse, each 0.05 mm thick and 0.00065 g. The layer is visual only; the
  physical evidence is the pad's contact with the IHS.
- **Wetness.** The wipe starts at 1.0 (1.5 mL of 91% IPA on the folded pad). It loses 0.0006 per control step in
  contact and dries with a 240 s time constant when off the lid; below 0.10 it is a dry wipe. The IPA film it leaves on
  the IHS decays with a 1.5 s time constant; the final dry check is max cell wetness <= 0.05 after the 9 s hold.
- **Lint.** A dry wipe (wetness < 0.10) dragged on the IHS at more than 0.20 N leaves a lint mark in every cell it
  covers; the wipe is low-lint when wet. The 1.0 check requires zero lint marks.
- **Scratch.** A hard part (a gripper link) on the CPU, a wet press over 6 N, a dry press over 2 N, or a wipe leaning
  more than 25 degrees on the lid with over 0.5 N sets the scratch latch.
- **Pins/socket.** Any pad contact with the board (socket frame, load plate, lever, PCB or a component) or with the CPU
  outside the IHS (the substrate ring or the IHS side) is paste on the socket/pins. The AM4 pins are under the substrate
  in the socket; the real check is a loupe inspection of the socket and the CPU's pin field.
- **Dirty wipe.** Each removed cell adds 0.00065 g to the wipe. Above 0.12 g the wipe redeposits 0.30 paste fraction on
  clean cells it crosses and the run is capped at 0.5. The demo's full 0.080 g patch stays below the limit.

## The demo

Open loop, known geometry. The right arm picks the wipe by its 20 mm grip bar, lifts it clear of the bench, carries it
over the CPU, lowers it until the 24 x 24 mm contact face is 20 um into the IHS (about 0.2-1.4 N at the soft contact),
wipes four overlapping passes across the 30 x 20 mm patch (x +-14 mm, y -4/-1.3/+1.3/+4 mm), then one slow pass down
each edge column where the raster ends. It lifts, carries the wipe to the waste tray, releases it, parks at the near
right and waits 9 s for the IPA film to dry. The left arm stays at ready. The wipe is held by the applicator bar, so the
fingers are 12 mm above the IHS and outside the socket; the 40 x 40 mm backing overhangs the socket but its underside
stays 2 mm above the IHS, 3.65 mm above the lever and 6.7 mm above the socket frame.

## Assets and real products

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. The detailed sources are in `REFERENCE.md`.

- **CPU:** AMD AM4 Ryzen-class, 40 x 40 mm package, 37.5 x 37.5 mm nickel-plated copper IHS 3.6 mm thick, 7.35 mm from
  the motherboard's top to the IHS top (a caliper measurement on a Ryzen 5 1600 and an AX370 board, [H]ardForum;
  package 40 x 40 mm, Socket AM4). The IHS has a 0.15 mm top chamfer, a copper foot, a laser-etched lid area, a pin-1
  triangle and a dark-green substrate with gold edge traces. The 1331 PGA pins are on the underside, hidden in the
  socket; the socket frame, load plate and steel lever are modelled around the IHS.
- **Motherboard:** ATX 305 x 244 x 1.6 mm on nine 6.5 mm brass standoffs; a 54 x 54 mm AM4 socket (40.2 mm opening),
  the four 54 x 90 mm cooler holes, four DDR4 DIMM slots, the 24-pin ATX header, two finned VRM heatsinks, the chipset
  heatsink, two PCIe slots, an M.2 slot and screw, six electrolytic capacitors, a CPU fan header, the rear I/O shield,
  silkscreen and traces. A dark anti-static mat, a blue wipe tray with a foil packet, a grey waste tray, a 4 g paste
  tube, the CPU's box, a Phillips #2 screwdriver, a folded-wipe packet and an anti-static bag complete the bench.
- **Paste:** an Arctic MX-4-class grey paste, 2.5 g/cm3; the 30 x 20 x 0.05 mm patch is 0.08 g (estimated residual
  after a 0.3-0.5 g application has been spread and the cooler lifted; most of the applied paste leaves with the
  cooler). The 122 cells are a coverage measure.
- **Wipe:** a Kimtech W4 / Kimwipes-class folded 91% IPA lint-free wipe, 50 x 50 mm and 2 mm thick, with a 24 x 24 mm
  soft contact face standing 0.8 mm proud of its flange; 1.5 mL of 91% IPA. The 40 x 40 x 6 mm backing and 20 x 20 x
  26 mm grip bar are a small bench applicator (the only non-catalogue part): the parallel-jaw gripper holds the bar,
  exactly as a hand holds the folded wipe. The wipe, its contact face, its IPA and the white-pad check are unchanged.

## Reset

Put a fresh folded wipe on its foil tray at (0.24, -0.24), bar up, with 1.5 mL of 91% IPA. Re-apply the 30 x 20 mm,
0.05 mm grey paste patch on the IHS (0.08 g) after cleaning the lid; the CPU stays in the socket and the board stays
fixed. Empty the waste tray and put the used wipe in it (or discard it). Check the socket, the load plate, the lever and
the CPU's pin field are clean and dry. The task's paste layer and wipe are reset by the simulator; the real station is
reset as above.

## Status (2026-11-16)

- **MuJoCo demo: 1.0 at step 658** (32.9 s after the 0.5 s settle; 900-step run limit). The full log:
  0.25 at step 76, 0.5 at 323, 0.75 at 423, 1.0 at 638, success at 658. The wipe's contact force was 0.2-1.4 N; the
  full 0.080 g patch loaded the wipe below its 0.12 g limit; the IHS film dried in the 9 s hold; no lint, no socket
  contact, no scratch.
- **Pytest: 17 passed** (13 offline + 4 MuJoCo: nominal, +3/+3 mm, -3/-3 mm, 10 degree yaw). The offline suite covers
  idle, hover, contact without rubbing, a dry wipe's lint, a saturated pad's redeposit, paste on the socket, a scratch,
  dry hard press, the 0.25/0.5/0.75/1.0 ladder, the 1 s hold and the clear-pad end state.
- **Isaac: 1.0 at step 658**, the same step; the PhysX contact reporter reads the wipe-to-CPU force (0.1-0.6 N steady, 0.8-3.5 N touch-down peaks), 122/122 clean at 23 s, dry and 1 s hold at 32.5 s.
- **Dream-1: 0.0 at 2,400 steps** (the policy never picks the wipe up; log `adhoc_67ba74da.json`).
- **Media:** `demo.mp4` (top view only, 320 x 176, 32.9 s, H.264), `setup.png` (top camera), `demo_top.gif` (top view)
  and `demo_review.mp4` (internal 2x2: top, right wrist, left wrist, CPU close-up). **`demo_review.mp4` is internal and
  must not be uploaded to the PR.** Nothing has been uploaded or pushed; the publisher will upload the top-view media and set the media ID.

## Run limit and seeds

- **2,400 steps (120 seconds at 20 Hz).** The demo succeeds at 658 (32.9 s); the hold and the dry wait are inside 900.
- One fixed layout. No random draw; object jitter of +-3 mm in x/y and a 10 degree wipe yaw still reach 1.0 (the four
  MuJoCo runs).

## Pass-through check (PASS_THROUGH_QA)

Frames at 0, 10, 25, 45, 60, 80, 95 and 100 % of the demo and the right-wrist close-up at 20 s were inspected. The
contact pairs at steps 66, 164, 296, 395, 526, 625 and 657 are: bench-wipe (the wipe resting in its tray), the two
right finger tips-wipe (the gripper holding the bar), and CPU-wipe (the scored contact of the soft face on the IHS).
There is no robot-board, robot-CPU, pad-board or pad-socket contact. The pad's contact face intersects the IHS by 0.02
mm (the soft contact, below the 0.5 mm latch); the fingers' 19.8 mm opening on the 20 mm bar is the same 0.1 mm soft
sink; the applicator backing is 2 mm above the IHS, 3.65 mm above the lever and 6.7 mm above the socket frame. No
cutaways. **No gripper, finger, forearm or held part passes through a visible solid.**

## Twin check

Board #132 (closed final) seats a heatsink straight down on a plain block; #969 (closed) takes a heatsink off a
processor; #891 (merged) installs a CPU in a socket; #1013 (merged) scrubs an IV connector's hub; #1619 (open) strips
paint; #1154 (closed) scrapes a print off a build plate. None removes old thermal paste from a real CPU IHS with a wet
lint-free wipe and the socket/IHS hazards. This is the other half of the paste job #132 leaves out. A pan-label adhesive-removal build would share the wipe
coverage process, but its object (a frying pan), tool (a scraper and a Chempad) and mechanism (holding a free pan)
differ from a CPU IHS in a socket, a scraper-free wet wipe, and the socket/pin and IHS-scratch hazards scored here.

## Remaining risks

- The **applicator** is a non-catalogue part (25 g backing + bar). It replaces the hand's grip, not the wipe: the
  91% IPA wipe, its 24 x 24 mm contact face and the white-pad check are the real ones. Its mass and the 1.5 mL IPA load
  are estimates.
- The **paste cell layer** is a coverage model, not 122 physical paste blobs: the cells are visual, while the pad-IHS
  normal force and the pad's motion are the solver's. The real check is the after-photo/loupe; a real paste film is not
  a 2 mm grid.
- The **scratch, lint and dry/press thresholds** (6 N wet, 2 N dry, 25 degrees, 0.10 wetness, 0.12 g load, 1.5 s IPA
  half-life) are estimates, not measured product limits. The real trial checks them with a raking light, a loupe, a
  white final wipe and the 2-5 min dry time.
- The **holder** keeps the fingers and wrist clear of the socket; a real hand holds the folded wipe directly. A real
  trial should use the folded Kimtech W4 with 91% IPA and the same four-pass + edge-pass pattern, or the technician's
  own circles; the scorer checks the same contact/rubbing/coverage.
- The **pins** are under the seated CPU. The 'paste on the pins' check is any pad contact with the board/socket or the
  CPU outside the IHS; a real trial inspects the socket and the CPU's pin field with a loupe.
- The **Isaac port** must add the pad-CPU force read from PhysX contact reports; the MuJoCo build's live reader is
  `sim_backend`. The geometry, the coverage rule and the scoring are unchanged.
