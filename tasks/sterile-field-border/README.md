# Lay a surgical instrument set on a sterile field without touching its unsterile border

A one-arm layout task with a keep-out zone that is judged over the whole handling, not only at the end. Four closed
surgical instruments, at true size, lie in a rack beside a sterile field. They have to be moved onto the field's blue
sterile area, flat and apart, and none may touch the field's orange border band (1 inch, 25.4 mm) at any moment on the
way. A touch is permanent: a contaminated instrument has to be replaced in practice, so here it can never score
again, however carefully it is put down afterwards.

The sterile drape is cloth in an operating theatre. In this task it is a **flat rigid panel** (300 x 440 mm, 3 mm
thick, about a Mayo stand tray's footprint) with the border drawn on it. The cloth itself (folds, sag, drape over
the table's edge) is not simulated.

| File | What it is |
|---|---|
| `sterile_field_border.py` | The `yam_arena` task file: scene, border-contact record, scoring (`success`, `score`, `steps_done`), scripted `demo()` |
| `assets/*.usda` | The four instruments, the sterile field panel, the instrument rack and a contamination flag (OpenUSD, written for this task) |
| `test_sterile_field_border.py` | Offline checks: scoring outcomes, the permanent border penalty, the 1 s full-success timing, assets match the scoring geometry, reach |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units
are metres and kilograms, Z up. Each instrument is one rigid body, closed (the box lock, ratchet and pivot are not
articulated), lying flat; its collision shape is a set of convex prisms (ring handles are 12 sectors each, so the
finger holes are real holes). Origin at the centre of mass on the underside, +x toward the working end.

| Instrument | Real model | Length used | Mass used | Cross-section (typical values) |
|---|---|---|---|---|
| Needle holder | Mayo-Hegar, 6 in, ratchet closed | 152 mm | 70 g | rings 22 mm outer, shanks 5.6 mm wide, box lock 8.4 mm tall, jaws to 10.4 mm wide |
| Scissors | Metzenbaum, 7 in, closed | 180 mm | 50 g | rings 23 mm outer (gold), blades 9 mm wide at the pivot to 3 mm, 6.4 mm tall |
| Forceps | Rochester-Pean, 6.25 in, straight, ratchet closed | 160 mm | 60 g | rings 22 mm outer, shanks 5.2 mm, jaws 9.6 mm to 2.4 mm, 7 mm tall |
| Scalpel handle | Bard-Parker-style no. 4 handle, no blade fitted | 135 mm | 22 g | 10 mm wide, 6 mm tall, neck narrowing, cm marks |

Lengths are the nominal catalogue lengths of these standard patterns. The cross-sections and masses are typical
values from comparable stainless instruments, not measurements of one product; the masses are below solid steel
because real instruments are thinner than the prisms here. Stainless steel friction 0.5 / 0.4 on the drape panel
and rack. **Sterile field panel** (`sterile_field.usda`): rigid, kinematic; blue inside, orange band, white line on
the inner edge. **Rack** (`instrument_rack.usda`): a 250 x 200 x 4 mm tray with eight 12 x 26 x 8 mm rests, the
instruments lying on them 8 mm above the tray so the fingers can reach under. **Flag** (`contam_flag.usda`): a 30 x
22 mm red tag, visual only (no collider, no gravity), waiting under the table until needed.

## Run it

On the YAM Isaac Lab-Arena station, from this folder, with the Arena environment active. The instruction is
`task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sterile_field_border.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sterile-dream

# Scripted demo, 1,100 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sterile_field_border.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval/sterile-demo
```

The log prints `[sterile_field_border] t=...  steps done: ...` when a step changes, `<instrument> touched the border:
contaminated for good` the moment a touch happens, and a trace every second (inside or outside the sterile area, gap
to the nearest instrument, heading, held).

**The demo.** Open loop (never reads a pose), left arm only, in the order needle holder, scalpel handle, Metzenbaum
scissors, Pean forceps. For each: from above, fingers across the instrument at its grip point (the centre of mass,
or on the pivot / box lock for the scissors and forceps), the pad tips 2 mm below its underside between the rests;
grip (about 20 N a finger), lift 60 mm, carry it over the border while the wrist turns it a quarter turn (working end
from +y to +x), lower to 3 mm above the sterile area, let go, lift away. Then home. Every success condition holds
from about 49.5 s and the run succeeds at 1,017 steps (51.35 s task time); On Isaac (L40): the first instrument is laid at 12.65 s, all four at 49.55 s, success conditions met at 50.3 s and 1.0 at step 1016.

## Instruction

One sentence of goal: the instruments from the rack onto the blue field, flat and apart, never touching the orange
border. It does not say where, in which order or how. Two rules from sterile practice are scored but not stated:
working ends point away from the robot, and the sharps (the scalpel handle) lie at an end of the layout, not between
other instruments.

## Scoring

Four steps, **0.2 each**, one per instrument. An instrument counts when all of these hold as things stand:

| Condition | Done when |
|---|---|
| Picked up | a gripper has held it (both pads on its box, both fingers pressing) while it was at least 10 mm above where it started |
| Never touched the border | no point of its surface has ever been within 1.5 mm of the panel's top face over the orange band |
| On the sterile area | every point of it inside the blue area; flat within 5 deg; working end within 25 deg of +x; resting within 2 mm of the panel; not in a gripper; at least 8 mm (3-D) from the other instruments; the scalpel handle outermost in y among the instruments on the panel |

Contact with the border is sampled over each instrument's whole surface (top, underside and edges, about 4 mm
grid and 2 mm along outlines), so a long instrument crossing the 25 mm band with both ends clear still touches it;
carrying one across the band 1.5 mm or more above the panel does not. It is latched for the episode: that instrument
can never score and the full score is out of reach. The red tag on the instrument shows it.

1.0 (ends the run) only once every full-success condition has held together, without a break, for 1 s: all four
steps, no instrument has ever touched the border, both grippers at least 3 cm from every instrument's box, nothing
off the table, and every instrument within 2 mm and 2 deg of where it was when that second began. The second is
timed in the task file; any lapse restarts it; step points stop at 0.8, also while it is being timed. The arms'
return is not scored. 0 if an instrument is off the table.

## What this task tests

* **Contamination over the whole handling.** Only the end layout is not enough: an instrument set on the border and
  moved inward still counts as contaminated. Dragging an instrument from the rack across the band, resting an edge
  on it or laying one half across it all latch the penalty (tests cover each).
* **Real instruments, not bars.** Ring-handled closed instruments with finger holes, crossed shanks and a tapered
  blade are grasped on the pivot or box lock between rests; a 6 mm-tall flank is the grip surface.
* **A layout, with two unstated rules,** not a single placement.

## Reset

Put the four instruments on the rack rests as in `task.json`'s setup. The border record and the tags restart with
the episode.

## Offline checks

`python -m pytest -q test_sterile_field_border.py` (needs the `yam_arena` environment).
