# Lay a nickel strip on a 2x2 18650 battery pack and spot-weld it to all four cells

A bimanual battery-pack assembly task. An assembled 2x2 pack of 18650 cells stands loose on the table; a nickel strip
lies in a small tray; a handheld battery spot welder sits in its holder, probes toward the robot.
- One hand picks the strip up by its folded tab and lays it on top of the pack, its two bars over the cell pairs.
- The other takes the welder out of its holder, tilts it probes-down and touches both copper tips to the strip over
  each cell's + terminal in turn: four welds, each leaving two dark spots on the strip. It puts the welder back and
  both arms go back.

The strip weighs 1 g and its tab is 3 mm across. The welder's tips are 3.2 mm apart and must both meet the strip
over the 6.4 mm + terminal without the housing touching it: they are the welder's lowest points only once it is tilted
more than about 35°.

| File | What it is |
|---|---|
| `welder_strip.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`), the welds and their spots, and the scripted `demo()` |
| `assets/nickel_strip_2x2.usda` | The nickel strip (OpenUSD), one rigid body, with eight hidden weld spots |
| `assets/strip_tray.usda` | The strip's tray, fixed to the table |
| `assets/battery_pack_2x2_rigid.usda` | The assembled pack as one rigid body: both holder halves and four cells, referenced from the next two files |
| `assets/holder_18650_2x2.usda` | One half of a 2x2 18650 cell holder, used twice (the top half is the bottom one flipped over) |
| `assets/battery_18650_yellow.usda` | The pack's 18650 cell, yellow wrap, with flat-ended prism colliders |
| `assets/spot_welder_18650.usdc` | The spot welder: housing with its copper probes, battery lid and latch, sprung + contact |
| `assets/welder_task.usda` | The task's wrapper: the welder as one articulation, its cell a separate body |
| `assets/battery_18650.usda` | The welder's own cell (blue wrap), in its bay under the latched lid |
| `assets/spot_welder_holder.usda` | The welder's holder, an orange PLA tray open at the middle of its long sides |
| `assets/textures/` | Plastic grain and PLA layer maps and the cells' wrap labels (no brand) |
| `test_welder_strip.py` | Offline checks: the file loads, every scoring outcome (1.0 only after 1 s of the success conditions, restarted by motion), the demo's welds land on the terminals, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task and the earlier spot-welder tasks by their authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up. Nothing is
downloaded.

- **Nickel strip:** an H of 0.15 mm pure nickel for a 2x2 pack with its cells in parallel (two 8 x 28 mm bars, 20 mm
  apart, joined by an 8 mm bridge; 28 x 28 mm), as sold for 18650 holders with 20 mm pitch. 0.955 g (nickel,
  8.9 g/cm³).
  - A grip tab is bent up from the bridge's middle and folded into a loop, 8 mm wide, 10 mm tall and 3 mm across: it
    stands in for the tweezers or fingertip a person uses.
  - It collides as boxes 0.3 mm thick (the plate) and three boxes for the tab; friction 0.5 / 0.4.
  - Eight weld spots (dark discs 1.2 mm across) are authored hidden on the plate's top, two per cell, 1.6 mm either
    side of the cell's axis across the bar.
- **Strip tray:** a 40 x 40 x 8 mm grey PLA block with a 29 x 29 x 1 mm pocket (0.5 mm play round the strip),
  colliding as its five boxes, fixed to the table.
- **Pack:** two halves of a black ABS 2x2 18650 holder (41.2 x 41.2 x 9.5 mm, 6.5 g each, 20 mm pitch, r9.30 bores
  for 0.30 mm radial clearance, 1.5 mm ledges with r7.0 windows), colliding as signed distance fields, and four
  18 x 65 mm cells (47 g, yellow wrap), + up. The top half rests on the cells' shoulders; their + nubs end 0.2 mm
  proud of its face (the stack's axial play), so the strip lies on the four nubs. 201 g, 68 mm tall.
  - **One rigid body** (`battery_pack_2x2_rigid.usda`, a wrapper that references the holder and the cell and takes
    their own rigid-body schemas off; mass, centre of mass and inertia from the parts). Real holders are a snug press
    fit, so an assembled pack moves as one part. As six separate bodies, the cells' contacts in the holders'
    signed-distance-field cups kept a ~1 cm/s jitter, and the jittering nubs walked the 1 g strip 2 mm off-centre in
    1.5 s. As one body the pack and the strip on it rest under 0.5 mm/s.
  - Each cell collides as two plain prisms with flat ends (a 30-sided body and a 16-sided + nub): PhysX's cylinder
    shape doesn't collide with signed-distance-field meshes, and a convex hull of the wrap mesh rocks.
- **Spot welder:** a mini 18650 spot welder, 124 x 24 x 22 mm black ABS housing, 138.7 mm long with its two copper
  probes, 127 g with its cell. The probes are 2 mm rods converging to tapered tips that end in 0.22 mm balls, 3.2 mm
  apart; the task's weld check uses those balls. The lid is shut and latched over the cell (a reset event shuts the
  lid and puts the latch home; their springs keep the targets written in the welder's USD).
- **Welder holder:** 142 x 39 x 14 mm orange PLA, 33 convex pieces, open at the middle of its long sides (80 mm) so a
  gripper takes the welder by its flat sides; screwed to the table.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=welder_strip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/strip-dream

# Scripted demo (both arms), 1,200 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=welder_strip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-demo
```

- **Score.** The run prints `task_score` at the end: the step points done, 1.0 on success (the success conditions
  held for 1 s). With `-E object_jitter=0` the layout is fixed and no seed is needed. The job log prints a
  `[welder_strip] t=...  steps done: ...` line whenever a step is done or undone, a `weld on cell k` line with where
  the tips touched the strip, and `success conditions met` (and `lapsed`) when the 1 s starts (or restarts).
- **The demo.** It reaches 1.0 at step 1044. Log times, which count the 0.5 s settle before step 1: strip on the
  pack at 11.6 s, welds on cells 2, 4, 3 and 1 at 20.05, 26.7, 33.25 and 39.55 s, welder back in its holder at
  47.45 s (step 939), every success condition holding from 51.7 s (step 1024), 1.0 one second later. (Before the 1 s
  was timed in the task file, the run ended at step 1043.) A trial takes about
  6 minutes on an L40S, including startup.
  - The right arm points down onto the strip's tab, leaned 15° about its closing axis (straight down, its wrist's
    joint 4 runs into its limit over the pack), and closes across the tab's 3 mm loop at about 5 N a finger. It lifts
    the strip out of its tray, lowers it until it is 0.5 mm over the + nubs, waits 0.6 s for the arm to catch up and
    lets go: the strip lands within 0.1 mm of the pack's centre. It goes home.
  - Meanwhile the left arm takes the welder out of its holder from above, across its middle in the holder's open
    sides (leaned 10° toward the probes), lifts it 5 cm, tilts it 60° probes-down about its own y axis and waits
    beside the pack. The probes keep pointing toward the robot and the housing rises away from it over the far side
    of the pack, clear of the strip's tab; the strip lies with its bars front to back, so the tab runs across the
    welder and the two tips sit side by side across a bar.
  - For each cell it carries the tips 2 cm over the strip, lowers them to 3 mm over it, waits 1 s and brings them
    down onto the strip over the + terminal in 1.2 s, holds 1 s and lifts them off. The arm's position drives carry
    the 127 g welder 4-6 mm low and a few millimetres short (their gravity compensation covers the arm only, and the
    wrist is soft at 10 N m/rad); pressed with that error, the tilted welder rested its weight on its tips, skidded
    and dragged the strip along. Each weld is aimed past its terminal by the error measured in the demo
    (`WELD_TRIM`), so the tips meet the strip gently and the strip stays put (within 0.4 mm).
  - It carries the welder back level over its holder, lets it drop into the pocket from 1.5 mm above the walls and
    goes home.
- **Dream-1:** 0.0 with this instruction (it never picked up the strip or took the welder out; media ID M-WDMK67).
- **Videos:** `inspect-robots video ~/yam-eval/strip-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_welder_strip.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order: the right lays the strip on the pack and goes back, then the left takes
the welder, welds the strip to each of the four cells, puts the welder back and goes back. It leaves out how to weld
and where things are, which the model works out from the cameras.

## Category: Impact on jobs

Joining cells into packs is regular work in two assembly occupations:
- **Team assemblers** (O*NET 51-2092.00; the BLS counts them with other miscellaneous assemblers and fabricators,
  about 1.4 million US jobs). They rotate through every task of their line's production process, and assembling
  electrical or electronic equipment is one of their listed work activities. On a battery-pack line, one of those
  tasks is welding the cells' connections.
- **Electrical and electronic equipment assemblers** (O*NET 51-2022.00, SOC 51-2028). Their core task is installing
  components "using rivets, bolts, soldering or micro-welding equipment", and batteries are among the equipment they
  assemble.

On lithium-ion pack lines and repair benches that work is laying nickel strip over the cells' terminals and
resistance spot-welding it to each cell. That is a standard way to join cylindrical cells into a pack: a pack maker's
process note (Lithion Battery, "Resistance Welding for Cell Interconnect") and a 2022 study of tab welding to 18650
cells (Int. J. Adv. Manuf. Technol. 121:6581-6597) describe it. The task tests the same steps at bench scale: placing
a 1 g strip squarely on the pack, then pressing the welder's two copper tips onto it right over each terminal without
knocking the pack apart. A missed or misplaced weld leaves that cell out of the circuit.

## Scoring

Six steps (`STEPS` and `POINTS` in the task file).

| Step | Points | Done when | Judged |
|---|---|---|---|
| 1. Strip on the pack | 0.2 | the strip lies flat (within 10°) on the cells' + nubs (within 1 mm), its centre within 3 mm of the pack's, its bars over the cell pairs (within 10° of a quarter turn), no gripper holds it, and a gripper took it out of its tray | at the end |
| 2-5. Cell 1, 2, 3, 4 welded | 0.1 each | with the strip on the pack and the welder in a gripper, both probe tips touch the strip's metal (within 0.5 mm of its top face, not more than 1 mm into it, not past its edges) over that cell's + terminal (within 3.5 mm of its axis) for 0.2 s; two dark spots appear on the strip where the tips touched | once done |
| 6. Welder back in its holder | 0.2 | after a gripper took it out, the welder sits in its holder's pocket (within 5 mm of its seat, on the floor within 2 mm, upright within 10°) and no gripper holds it | at the end |

- **1.0 (success, ends the run):** only once all of these have held together, without a break, for 1 s
  (`STILL_S`):
  - all six steps;
  - both arms back where they started (within 3 cm), holding nothing;
  - the welder, the strip and the pack still: under 1 cm/s and 0.5 rad/s (`STILL_SPEED`, `STILL_SPIN`).
- **The 1 s is timed in the task file:** `_Progress` notes, once per control step, the time the conditions first held
  (`conditions_met`). Any lapse, such as a part moving faster or an arm leaving home, clears it, and the second starts
  over the next time they all hold. `success()` and `score()` give 1.0 only once that second is complete.
  `SUCCESS_HOLD_S` is 0, so the run ends on that same step instead of waiting another second.
- **Until then, all six steps score 0.8:** the step points stop at 0.8, also while the second is being timed.
- **0** at the end, whatever the steps, if the pack has been knocked over (tilted more than 20°), any part has fallen
  off the table (centre more than 2 cm below its top), or the state is not finite.
- **Only a gripper's work counts:** a strip knocked onto the pack, a welder dropped onto the strip, or a welder that
  never left its holder earns nothing.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 2 mm apart (`GripperState`, `_held`).
- **The welds in simulation** are touches, not current: no heat, no bond, so the strip stays loose on the pack
  afterwards (on a real station they must bond, below). The spots are the strip's own hidden discs, moved to where
  the tips touched and shown.
- **On a real station** steps 1 and 6 are judged from the video, with a ruler for the strip's place. Steps 2-5 need a
  bond, not just a touch: after the run, each weld gets a pull test. A spring scale hooked under the strip beside that
  cell's weld pulls straight up at 10 N for 2 s (a good nickel-strip spot weld holds several times that), and the weld
  counts only if the strip stays on that cell's terminal. A touch that left marks but no bond earns nothing there.

## Real-world trials: safety

Spot-welding lithium-ion cells is safe with the usual battery-bench precautions:
- **Cells:** new or tested cells at storage charge (about 3.6 V), their wraps and top insulators undamaged; the pack
  holder insulates them from each other and from the table.
- **Bench:** a non-flammable surface (steel or ceramic tile), nothing flammable within 1 m, a fire-safe bin or a
  bucket of sand for a hot or venting cell, and a CO2 or ABC extinguisher at hand.
- **People:** a supervisor watches every trial with the welder's power switch and an emergency stop within reach;
  safety glasses for anyone near the bench; good ventilation.
- **Welder:** set to the strip's thickness (0.15 mm) before the trial; switched off and never left powered and
  unattended between trials.
- **After a trial:** check each cell's temperature by hand before touching the pack; set aside any cell that is hot,
  swollen or marked.

## Revision notes

Earlier reviews asked for these changes; each is done as follows.
- **"Include assets/spot_welder_18650.usda in the archive and verify the scene and full-score demo run from the
  packaged files."** The welder is packaged as binary USD under 2 MB (`assets/spot_welder_18650.usdc`, referenced by
  `assets/welder_task.usda`); every file is under 1.87 MB. The demo runs from files unpacked from the committed branch.
- **"Specify a practical bond check for each real-world weld, such as a controlled peel or pull test, and require
  successful bonds for full welding credit."** Scoring, "On a real station": a 10 N pull for 2 s with a spring scale
  beside each weld; a weld counts only if the strip stays on.
- **"Document safe cell preparation and fire precautions for real trials."** The section "Real-world trials:
  safety".
- **"Change the domain to Impact on jobs and explain how welding battery-pack connections relates to the regular
  work of team assemblers."** `task.json`: category Impact on jobs; its context and the section "Category: Impact on
  jobs" above give the occupations' duties and sources. (The round before suggested Other; the latest review asked
  for Impact on jobs.)
- **Also fixed (raised on another task's review, and true here too): a score of 1.0 before the stated 1 s of
  stillness.** `success()` now times the full-success conditions itself (`_Progress.met_since`, `conditions_met`,
  `STILL_S`), and `score()` gives 1.0 only once they have held for 1 s; see Scoring. Tests: no 1.0 at 0.95 s, 1.0 at
  1 s, motion restarts the second, and no 1.0 while any one condition is missing.

## Reset

- Put the strip back in its tray, bars front to back, tab up (a fresh one if it shows weld spots).
- Stand the pack on its mark, all cells + up, the top holder on.
- Seat the welder in its holder, probes toward the robot, lid shut.
