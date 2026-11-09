# Assemble a 2x2 18650 battery pack in a jig

A bimanual assembly task. An orange assembly jig is screwed to the table; two identical halves of a 2x2 18650 cell
holder lie beside it (one cups up, one cups down), and a grey tray holds four 18650 cells standing positive end up.
- The left hand puts the cups-up holder half into the jig's pocket.
- The right hand takes the four cells from the tray one by one and pushes each into a cup of that holder, positive
  end up.
- The left hand lowers the other half, cups down, onto the tops of the four cells. Both arms go back.

The cups leave 0.3 mm a side around a cell, with a 0.8 mm lead-in, and the cells stand 2 mm apart: the fingers, 15 mm
wide at their tips and 26-40 mm wide further back, can't reach between two cells.

| File | What it is |
|---|---|
| `pack_assembly.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/holder_18650_2x2.usda` | One half of a 2x2 18650 holder (used twice), one rigid body, SDF collider |
| `assets/battery_18650_yellow.usda` | The 18650 cell, yellow wrap, flat-ended prism colliders |
| `assets/cell_tray_18650_2x2.usda` | The cell tray: four pockets on a 36 mm pitch, convex-prism colliders |
| `assets/pack_jig_2x2.usda` | The assembly jig: a 42 mm square pocket with finger slots, convex-prism colliders |
| `assets/textures/` | The cell's wrap label (no brand) |
| `test_pack_assembly.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are
metres and kilograms, Z up.

- **Holder half:** black ABS, 41.2 x 41.2 x 9.5 mm, 6.5 g; 20 mm pitch, r9.30 bores (0.30 mm radial clearance) with a
  0.8 mm lead-in, 8 mm cups on a 1.5 mm ledge, planar grip faces on each side. The top half is the same part turned
  over; on four seated cells its outer face is 67.8 mm above the bottom half's.
- **Cell:** 18 x 65 mm, 47 g. Two prisms with flat ends (30-sided body, 16-sided nub).
- **Cell tray:** 3D-printed PLA, 80 x 80 x 14 mm, 47 g; four r9.6 pockets 11 mm deep with a 1 mm lead-in, 36 mm apart.
- **Jig:** 3D-printed PLA, 76 x 76 x 6 mm, fixed; a 42 x 42 x 3 mm pocket (0.4 mm a side around a holder) with a 1 mm
  lead-in and a 20 mm finger slot through each side wall; four M3 screws.

## Run it

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pack_assembly.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2100 --store-frames --log-dir ~/yam-eval/pack-demo
```

The job log prints a `[pack_assembly] t=...  steps done: ...` line when a step is done or undone, plus a trace every
0.5 s: each cell's offset from the nearest cup, the bottom holder's from the pocket and the top holder's from its seat.

**The demo.** It reaches 1.0 at step 1971 (98.6 s): the bottom holder is in the jig at 9.2 s, two cells are
seated at 42.0 s, four at 83.1 s, the top holder is on at 94.6 s, and both arms are home by 97.5 s. A trial takes
about 5 minutes on an L40S, including startup.
- Left arm: pinches the cups-up half across two sides from above, its fingertips 1.5 mm above the half's lower face,
  and lowers it into the jig, the fingertips going down the jig's finger slots. It then picks up the cups-down half and
  holds it low beside the jig. When the right arm is done, it lifts the half straight up, carries it over the cells and
  lowers it onto them.
- Right arm, cells in the order FL, FR, BL, BR (F = away from the robot, L = its left):
  - The first three cells are gripped from above at mid-height. The fingers close toward cups that are still empty and
    are shifted 6 mm away from seated neighbours. The gripper's wide part stays above the seated cells' tops.
  - The last cell (BR) is gripped from the side: the gripper points along the holder's diagonal, 30 degrees below
    level, and the fingertips stop 1 mm past the cell's axis, short of its two neighbours.
  - Each cell is carried over the pack and lowered straight down, slowly between its neighbours, then steadied at the
    cup's mouth.
  - It is let go once its bottom is 0.5 mm inside the bore, below the lead-in, and drops onto its seat. The fingers
    come up over its top, close, and press on its + nub to push it fully home.
- The arm droops under a cell by up to 2 mm, differently at each pose, and the cups' lead-in catches about 1.1 mm. The
  demo therefore aims the FL and BR cups by the offsets it measured (`AIM`); the physics is deterministic.

## Instruction

It says which hand does what and the order, and leaves out where things are.

## Scoring

Four steps, **0.25 each**, judged at the end.

| Step | Done when |
|---|---|
| 1. Bottom holder in the jig | a holder half lies cups up (within 10 degrees) in the pocket: centre within 2 mm, outer face within 1.5 mm of the floor |
| 2. Two cells seated | with step 1, at least two cells have their bottom face's centre within 1 mm of a cup's axis and 1.5 mm of its ledge, + end up within 8 degrees |
| 3. Four cells seated | all four |
| 4. Top holder on | with step 3, the other half is turned over (within 10 degrees) and its outer face is within 2 mm of the bottom half's axis and 1.5 mm of resting on the cells |

- **1.0 (success):** all four steps, both arms back where they started (within 3 cm) holding nothing, and both halves
  and all cells still (under 3 cm/s and 1 rad/s) for 1 s. PhysX keeps a contact jitter of a few mm/s on the cells and
  holders, so "still" means nothing is sliding, tipping or falling.
- **0** if a holder half, a cell or the tray has fallen off the table, or the state is not finite.
- Either half may be the bottom one. "Seated" is judged by the cell's bottom, so a cell leaning a little in its cup
  still counts.

## Reset

Take the top half off and lay it cups down at its mark. Stand the cells positive end up in the tray. Lay the bottom half
cups up at its mark. Put the tray back on its mark.

## Status

Dream-1 score: 0.0 (it never picked up a holder half or a cell). Media: M-5TZ7WL.
