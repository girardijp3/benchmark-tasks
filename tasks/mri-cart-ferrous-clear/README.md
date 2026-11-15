# Strip a sedation cart of magnetic and battery-powered items before the scanner room

A one-arm identification and pick-and-place task in a clinical setting. A compact rolling cart stands by a red storage
tote. On its tray are six everyday items. Three must not enter an MRI scanner room: a steel hemostat, a two-way radio and
a fingertip pulse oximeter. Three are MRI safe and stay: a pair of plastic locking forceps that look like the hemostat,
a roll of paper tape and a specimen container.
- The robot picks the three unsafe items off the tray, one at a time, and sets each in the tote.
- It leaves the safe items where they are, upright on the tray, and its gripper ends clear of everything.

The skill is telling the items apart (the blue plastic forceps are the hemostat's twin in shape) and handling six very
different true-size objects: a flat 8 mm steel instrument, a 260 g radio with an antenna, a 31 mm tall clamshell.

| File | What it is |
|---|---|
| `mri_cart.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/*.usda` | The parts (OpenUSD): cart, tote, hemostat, plastic forceps, radio, pulse oximeter, tape roll, specimen container |
| `test_mri_cart.py` | Offline checks: the file loads, every scoring outcome, the 1 s full-success timing, the demo's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are
metres and kilograms, Z up. Sizes are typical catalogue values for these product types (no one model was measured):
- **Cart** (fixture): 460 x 360 mm tray with a 3 mm rim 8 mm high, upper face 120 mm above the table, a lower shelf, four
  18 mm posts and four 35 mm casters. This is a tabletop-size cart: the station table puts the work surface at 0.75 m,
  where a floor-standing hospital cart's tray would be out of the arm's reach, so the cart is the compact kind (a bedside
  caddy or desk cart). Casters are locked, so it does not move.
- **Tote** (fixture): shallow polypropylene storage tote, 320 x 240 x 100 mm, 3 mm walls.
- **Hemostat**: Kelly locking forceps, 5.5 in (140 mm), martensitic stainless steel (magnetic), 52 g, closed and flat.
  Two 4 mm leaves, 8 mm at the box lock, 7 mm shanks, finger rings 27 x 20 mm with real holes, serrated jaws, ratchet.
- **Plastic forceps**: disposable polypropylene locking forceps, 5 in (125 mm), blue, 12 g. Same form, 3.5 mm leaves.
- **Radio**: handheld two-way radio, body 112 x 58 x 36 mm, 70 mm antenna, 260 g with battery; display, keypad, knob and
  an orange emergency button as thin coloured geometry.
- **Pulse oximeter**: fingertip clamshell 58 x 34 x 31 mm, 50 g with two AAA cells, display window with red digits.
- **Tape**: paper surgical tape roll, 1 in x 10 yd, 40 mm across, 25 mm wide, 25 g, white core.
- **Specimen container**: polypropylene, 56 mm base, 48 mm to the rim, blue screw lid 60 mm x 7 mm, 21 g.

Frictions (static/dynamic): steel 0.45/0.35, plastic 0.45/0.38, radio 0.5/0.45, oximeter 0.45/0.4. Colliders are
convex hulls (64 points or fewer) and exact boxes and cylinders; the finger rings are four boxes round their holes.

## Run it

On the YAM Isaac Lab-Arena station, from this folder, with the Arena environment active. The instruction is
`task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mri_cart.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mricart-dream

# Scripted demo (right arm), 850 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mri_cart.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 850 --store-frames --log-dir ~/yam-eval/mricart-demo
```

The job log prints a `[mri_cart] t=...  steps done: ...` line when a step is done or undone, plus a trace every
second: which unsafe items are in the tote and which safe items are still on the cart.

**The demo.** Open loop: a fixed waypoint script that never reads a simulated pose. The right arm picks each unsafe item
straight down by its flat middle and sets it down in the tote (on Isaac, L40: hemostat in at 11.3 s, radio at 24.45 s, oximeter at 37.7 s):
- hemostat: pinched across the box lock (14 mm wide, 8 mm tall, 1.5 mm pad clearance above the tray), set in the tote's
  middle;
- radio: pinched across its 58 mm width just behind the middle, carried low over the rim (the antenna end leads), set at
  the tote's far corner with the antenna inside the walls;
- oximeter: pinched across its 34 mm width, set at the tote's near side.
Then the arm goes home. Success conditions are met at 39.9 s and the run ends with 1.0 at step 808.

## Instruction

It says the goal: the cart is going into the MRI room, take the magnetic and battery-powered things off it into the red
tote, leave the MRI-safe supplies. It names no item and no technique.

## Scoring

Four steps, **0.2 each**, judged as things stand at the end.

| Step | Done when |
|---|---|
| 1. Hemostat in the tote | every corner of its bounds is inside the tote's floor plan and its lowest point is below the rim |
| 2. Radio in the tote | the same for the radio (its antenna included) |
| 3. Pulse oximeter in the tote | the same for the oximeter |
| 4. Cart left with only the safe items | with 1 to 3: the plastic forceps, tape and container are still on the tray (origin inside the rim, within 8 mm of the tray's face, tilted under 10 degrees), and no gripper's pads are within 3 cm of any item |

**Full score 1.0** (it ends the run) only once all four steps and every item's stillness have held together, without a
break, for 1 s: all six items within 2 mm and 2 degrees of where they were when that second began, timed in the task
file (`SUCCESS_HOLD_S = 0`). Any lapse restarts the second; the step points stop at 0.8, also while it is timed.
**0** if any item is off the table.

Shortcuts: putting a safe item in the tote or knocking one over blocks step 4 and success until it is undone; pushing an
unsafe item off the cart onto the table is not "in the tote"; doing nothing scores 0 (tests in `test_mri_cart.py`).

## Fidelity

Rigid bodies. Magnetism is not simulated: which items are unsafe is the task's rule and the scorer reads only where each
item ends up. Real masses and sizes throughout; the tabletop-size cart is the one scale concession, forced by the station.

## Reset

Put the six items back where they started and stand the tote on its spot. The task file resets the poses on every episode.
