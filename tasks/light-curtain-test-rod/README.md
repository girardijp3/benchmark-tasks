# Test two machine light curtains with their test rods and tag out the one that fails

A start-of-shift safety check with a decision in it. Two benchtop machines stand side by side, each guarded by a safety
light curtain (a 30 mm-resolution hand-protection pair, protected height 150 mm). Each machine has its own 30 mm test
rod resting in front of its field and an empty tag holder; a red DO NOT USE tag stands in a rack between them.
- Each rod is pushed through its machine's field, tip first, at the bottom, middle and top, while the receiver's lamp
  is watched: it goes red while a beam is interrupted.
- One curtain has a dead beam channel in the middle of its field. The 30 mm rod covers a working beam at most heights,
  so that lamp still goes red at the bottom and top, but where the rod covers only the dead beam it stays green.
- The tag goes in the holder of the machine whose curtain missed the rod. The other machine is left available.

The skill is the whole check: handling a test rod so that its tip, not the hand, goes through the field level and square
at three heights, reading the lamp, and then tagging the right machine. Tagging without testing, tagging the sound
machine, or testing and leaving both available all fail.

| File | What it is |
|---|---|
| `light_curtain_test.py` | The `yam_arena` task file: scene, the lamp effect, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `build_assets.py` | Writes the USD assets below with `pxr` (boxes, cylinders, small convex prisms) |
| `assets/curtain.usda` | A light-curtain pair (emitter and receiver posts on foot plates), a fixture |
| `assets/test_rod.usda` | The 30 mm test rod, one rigid body |
| `assets/rod_saddles.usda` | Two saddles the rod rests on, a fixture |
| `assets/tag_holder.usda` | A tag holder (also used as the tag rack), a fixture |
| `assets/tag.usda` | The DO NOT USE tag, one rigid body |
| `assets/lamp_green.usda`, `assets/lamp_red.usda` | The receiver's lamp discs: visual only, no collider, no gravity |
| `test_light_curtain_test.py` | Offline checks: the file loads, the lamp rule, every scoring outcome, the 1 s full-success timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets and sizes

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are
metres and kilograms, Z up. Where a number is a typical value rather than a datasheet figure it says so.

- **Light curtain** (each pair a fixture): two 40 x 40 mm housings (typical for Type 4 curtains, which run about 30 to
  50 mm), 230 mm tall with 20 mm end caps, on 90 x 90 x 12 mm foot plates, 400 mm between the lenses. 8 beams 20 mm
  apart, 10 mm lenses: resolution = beam pitch + lens diameter = 30 mm, the usual hand-protection resolution (IEC 61496
  uses a 30 mm test piece for it). Protected height 150 mm, the smallest catalogue length (heights step in 150 mm).
  The beams are 57 to 197 mm above the table. The field is the vertical plane between the lens faces.
- **Test rod**: 30 mm diameter, 300 mm long (typical), a hollow aluminium tube with 1.5 mm wall, 110 g, yellow and black
  bands with a black grip band in the middle, where its centre of mass is. Friction (static/dynamic) 0.5/0.4.
- **Rod saddles**: 30 x 60 x 12 mm blocks 200 mm apart, each with two 6 mm cheeks 34 mm apart (2 mm a side) so the rod
  cannot roll; the rod's axis rests 27 mm above the table.
- **Tag**: 76 x 127 mm (3 x 5 in, a common danger-tag size), 46 g. Real tags are thin card hung on a cable tie; here it
  is a 5 mm HDPE sign that stands in a holder, so a gripper can take it.
- **Tag holder and rack**: a 100 x 28 mm block with a slot 8 mm wide, 90 mm long, 24 mm deep, flared to 20 mm at the
  top (the 5 mm tag has 1.5 mm a side in the narrow part, with a 6 mm lead-in).
- **Colliders**: boxes, cylinders and convex prisms. Frictions: steel 0.5/0.35, plastic 0.45/0.35, HDPE 0.4/0.3.

## The trip lamp (the one code effect)

A light curtain's receiver shows a red lamp while a beam is interrupted. The lamp is a declared code effect: each
receiver has a green and a red disc (no collider, no gravity) that the task swaps on its top cap, once per control
step. It is red while a rod covers a **working** beam: a beam is a line along the field, its lens 10 mm across, and a
rod covers it when the beam's centre is within 15 - 5 = 10 mm of the rod's axis along the rod's length (the rod holds
the whole lens). It follows only the rod's real pose. It never moves, holds, places or releases anything and plays no
part in the score; the score only reads the rods, the tag and the grippers.
- Because the pitch is 20 mm and the rod 30 mm, a rod always covers at least one whole lens, but with a rod axis within
  10 mm of the dead beam's height (107 to 127 mm) it covers only the dead beam. The faulty lamp therefore stays green
  at about 117 mm and goes red at the bottom (77 mm) and the top (165 mm). A sound curtain goes red at all three.
- Which machine is faulty is a constant in the task file (`FAULTY`); the real station's person chooses by coin flip
  (see the real-world check). The demo is a fixed open-loop script for the file's setting; it reads nothing from the
  simulator.

## Scoring

Four steps, 0.2 each, judged as things stand at the end except steps 1 and 2, which count once reached.

| Step | Done when |
|---|---|
| 1. Left field tested | a rod has been pushed through it (axis within 15 degrees of the field's normal, tip 4 cm beyond the plane, crossing between the posts, the rod's middle still in front of the plane) once with its axis in each third of the height: 52-102, 102-152 and 152-202 mm |
| 2. Right field tested | the same |
| 3. Tag decision | the tag stands in the holder of the machine whose curtain missed the rod (lower edge's middle within 12 mm along the slot, 4 mm across, -2/+4 mm of the slot floor, within 6 degrees of upright), it was put there after that curtain's three passes were done, and it is not in the other holder |
| 4. Let go | with steps 1 to 3, both rods are out of both fields and no gripper is within 3 cm of the tag or either rod |

1.0 (ends the run) only once all four steps have held together without a break for 1 s, with the rods and the tag within
2 mm and 2 degrees of where they were when that second began. It is timed in the task file; any lapse restarts it. Step
points stop at 0.8, also while it is being timed. The tests check no 1.0 at 0.95 s, 1.0 at 1 s, a lapse restarting it,
and no 1.0 while any one condition is missing. A rod or the tag off the table scores 0.

Shortcuts the tests and runs cover: doing nothing 0; tagging without testing 0; testing both and tagging the sound
machine 0.4; testing both and no tag 0.4; tagging first, then testing 0.4; testing and tagging only the faulty machine
0.4; a rod that stops short of the plane, tilts, goes beside the posts, or runs at the table is no pass.

## Run it

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=light_curtain_test.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/lc-demo
```

The log prints a `[light_curtain] t=...  steps done: ...` line when a step is done or undone, plus a trace every second:
the passes made at each curtain, both lamps' colours and which holder the tag stands in.

**The demo.** On Isaac (L40) it runs as planned: both curtains tested by 18.5 s, the tag stood in the holder at 36.45 s, success conditions met at 39.4 s and 1.0 at step 798. Timeline: both arms work at once. Each, from above with its fingers closing
across the rod's middle, lifts its rod off the saddles and pushes it through its field at 77, 117 and 165 mm, pausing
0.8 s in the field and drawing it back each time, then sets it back on the saddles and lets go (both curtains tested
by about 18.5 s). The arm of the machine that missed the rod then takes the tag from the rack by its upper part and
stands it in that machine's holder (about 36.5 s), lets go and lifts away; full success follows a second later
(step 798, about 40 s).

## Instruction

It says what to do (test each curtain with its rod at three heights; tag a machine whose curtain fails). It does not say
which machine fails, how to hold the rod, or what the lamp shows.

## Real-world check

Two real curtains (or one curtain and a covered lens) at a bench, a 30 mm test rod, a tag and a holder. Tape over one
lens of the middle beam on one receiver with a piece of black tape (a coin flip chooses which). Run the task; read
the video: each rod passes through its field at three heights with the rod's tip first and the hand outside the field,
the lamp goes red at each pass on the sound machine and misses at the middle on the taped one, and the tag ends in the
holder of the taped machine only. Take the tape off afterwards.
