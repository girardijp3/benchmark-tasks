# Unzip an anti-static bag, take out an 18650 cell and stand it in a battery holder

A bimanual task with a bag that has to be opened first. A grey anti-static (static shielding) stand-up pouch stands on
the table with its bottom glued down. It is zipped shut with a slider zipper and holds two 18650 cells standing upright.
Next to it is the empty bottom half of a 2x2 battery holder.
- The left hand pinches the zipper's slider and pulls it along the top. The bag's mouth opens behind it.
- The right hand reaches into the open mouth, takes one cell by its top and lifts it out. It stands the cell, positive
  end up, in a cup of the holder. The other cell stays in the bag. Both arms go back.

The mouth only opens as far as its film springs; the gripper is 27 mm wide at the pads and 40 mm a little higher up,
so it has to go straight down the middle. The cups leave 0.3 mm a side around a cell.

| File | What it is |
|---|---|
| `antistatic_bag.py` | The `yam_arena` task file: scene, the zipper's interlock and reset events, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/antistatic_bag.usda` | The bag (OpenUSD): one articulation, with film panels on sprung hinges and the slider on a damped slide |
| `assets/holder_18650_2x2.usda` | The bottom half of a 2x2 18650 cell holder, one rigid body |
| `assets/battery_18650_yellow.usda` | The 18650 cell, yellow wrap, with flat-ended prism colliders |
| `assets/textures/` | The bag's print (a generic ESD caution label, no brand) and the cell's wrap |
| `test_antistatic_bag.py` | Offline checks: the file loads, every scoring outcome (1.0 only after 1 s of the success conditions, restarted by motion), both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Bag, holder and cell** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Bag:** a metal-in static shielding pouch, 100 mm wide (5 mm side seals) and 99 mm tall with a 34 mm gusset.
  - The film is a 3.1 mil (79 µm) PET/aluminium/PE laminate, about 80 g/m², rendered translucent grey.
  - The zipper has two PE profiles and a 0.9 g POM slider with a pull tab. The slider travels 77 mm.
- **One articulation:**
  - The root is the floor, the lower walls and the side seals, pinned to the table.
  - Above 42 mm, each wall is eight film panels (0.14–0.23 g each) on hinges along the mouth. Each hinge has a
    0.06 N m/rad spring, with damping and armature so PhysX stays steady with such light links. Each panel carries
    its piece of the zipper profile.
  - The slider is on a prismatic joint damped at 20 N s/m: the zipper's drag.
  - The film collides as 1 mm boxes, the profiles as capsules, the slider as boxes with rounded ends.
- **The zipper's interlock** is modelled in the task file:
  - Every control step, an interval event sets each panel's spring target from the slider's travel: shut until the
    slider's trailing end reaches the panel, open once it has run over it (11–26°, a lens-shaped mouth).
  - Isaac Lab sends no joint targets for Arena's task objects, so the targets are written to PhysX directly.
  - Pulling the slider is what opens the bag; pushing it back would shut it.
- **A contact-only zipper was tried first:**
  - The design: springs always open, the slider's C holding the panels under it, and each panel's catch holding its
    neighbour shut.
  - It held shut on PhysX's CPU solver. On the GPU solver that Arena uses, panels slipped past their catches at rest
    or as soon as the slider moved. We tried box, capsule and rod catches, heavier joint armature and 128 solver
    iterations; none held.
  - The asset's build script still builds it (`BAG_ZIPPER=contact`).
- **Holder:** the bottom half of a 2x2 18650 holder in black ABS, 41.2 x 41.2 x 9.5 mm, 6.5 g, 20 mm pitch. The
  bores are r9.30 (0.3 mm radial clearance) with a 0.8 mm lead-in. It collides as a signed distance field.
  - It is taped to the table (kinematic). Left loose, the 6.5 g holder and the cell standing in it kept rattling in
    PhysX at about 50 mm/s, even with the cell set down upright on its seat.
- **Cell:** an 18 x 65 mm 18650 cell, 47 g. It collides as two flat-ended prisms.

**At reset** a reset event shuts the bag and puts the slider at its end stop. Arena doesn't write an articulation's
joint angles at reset.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active. The instruction is `task.json`'s:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=antistatic_bag.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 820 --store-frames --log-dir ~/yam-eval/bag-demo
```

- **Score.** 0.2 per step (at most 0.8) and 1.0 only on success, once the success conditions have held for 1 s. The
  log prints `[antistatic_bag] t=...  steps done: ...` on each change, `success conditions met` (and `lapsed`) when
  the 1 s starts (or restarts), and a trace line every second: slider travel, panel angles, cells, holder, grippers.
- **The demo.** It reaches 1.0 at step 698, as it did before the 1 s was timed in the task file (3 of 3 runs plus
  the windowed one then; checked again after). Log times, which count the 0.5 s settle before step 1: unzipped at
  8.65 s, cell out at 21.0 s, in the holder at 30.55 s (step 601), every success condition holding from 34.4 s (step
  678), 1.0 one second later. Dream-1 scored 0.0 (media ID M-J5MEEM).
  - The left arm leans 20° toward itself and pinches the slider across the zipper. It pulls the slider 77 mm in 4 s:
    the bag is unzipped at 8.7 s. Then it goes home.
  - The right arm comes straight down into the mouth, fingers 29 mm apart and closing along it. It grips the cell's
    top 8 mm. It lifts the cell 115 mm (out at 21 s) and carries it over the holder. It pauses 12 mm above the cup:
    under way the arm lags about 4 mm, and the pause brings that down to 0.6 mm. It lowers the cell onto its seat,
    pressing it 0.5 mm, opens slowly and goes home.
- **Offline checks:** `python -m pytest -q test_antistatic_bag.py`.

## Instruction

It says which hand does what, in order: the left opens the zipper, and the right moves one battery into the holder
the right way up and leaves the other in the bag. It leaves out positions.

## Scoring

Four steps, **0.2 each** (`STEPS` in the task file); only full success scores 1.0.

| Step | Done when | Judged |
|---|---|---|
| 1. Bag unzipped | the slider pulled at least 90% of its travel | once reached |
| 2. Battery taken out of the bag | a gripper holds a cell whose lowest point is at least 5 mm above the bag's mouth | once reached |
| 3. Battery in the holder | with step 2 done, a cell within 3 mm of a cup's seat, positive end up within 10°, not held | at the end |
| 4. Other battery left in the bag | with step 2 done, a cell still stands in the bag (upright within 30°), not held | at the end |

- **1.0 (success, ends the run):** only once all of these have held together, without a break, for 1 s
  (`STILL_S`):
  - all four steps;
  - both arms back where they started (within 3 cm), holding nothing;
  - the seated cell and the holder still: under 1 cm/s and 0.5 rad/s (`STILL_SPEED`, `STILL_SPIN`).
- **The 1 s is timed in the task file:** `_Progress` notes, once per control step, the time the conditions first held
  (`conditions_met`). Any lapse, such as the cell moving faster or an arm leaving home, clears it, and the second
  starts over the next time they all hold. `success()` and `score()` give 1.0 only once that second is complete.
  `SUCCESS_HOLD_S` is 0, so the run ends on that same step instead of waiting another second.
- **Until then, all four steps score 0.8:** the step points stop at 0.8, also while the second is being timed.
- **0:** a cell or the holder off the table at the end, or a state that is not finite.
- **"A gripper holds":** the finger pads reach the part's bounding box (5 mm slack), both fingers press with at least
  0.5 N, and the pads are at least 3 mm apart.

## Revision notes

Earlier reviews asked for these changes; each is done as follows.
- **"Change the domain to impact on jobs and explain how seating cells in a holder is core assembly work for team
  assemblers."** `task.json`: category Impact on jobs, and its context names the assemblers' work.
- **"Align the simulation score with the stated full-score conditions so it cannot report 1.0 before both arms
  return and the seated cell has settled."** The step points stop at 0.8, and 1.0 needs every full-success
  condition (Scoring).
- **"Update the simulation scorer to track the seated cell's continuous stillness for one second and return 1.0
  only after that period and the other full-success conditions are met."** `success()` now times it in the task
  file: `_Progress.met_since` notes, every control step, when every full-success condition (`conditions_met`: the
  four steps, both arms home and empty, the seated cell and the holder under 1 cm/s and 0.5 rad/s) first held, and
  any lapse clears it. `success()` and `score()` give 1.0 only once that has lasted `STILL_S` = 1 s;
  `SUCCESS_HOLD_S` is 0, so the run ends on that step. Tests in `test_antistatic_bag.py`: no 1.0 at 0.95 s, 1.0 at
  1 s; the cell moving or turning, or an arm leaving home, restarts the second; no 1.0 while any one condition is
  missing. The demo still reaches 1.0 at step 698, so the videos are unchanged.

## Reset

- Stand the cell back in the bag at its mark, positive end up.
- Zip the bag shut, with the slider at its right-hand end stop.
