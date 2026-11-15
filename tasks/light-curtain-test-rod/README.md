# Test two machine light curtains with their test rods and tag out the one that fails

A start-of-shift safety check with a decision in it. Two benchtop machines stand side by side, each guarded by a safety
light curtain (a 30 mm-resolution hand-protection pair, protected height 150 mm). Each machine has its own 30 mm test
rod resting in front of its field and an empty tag holder; a red DO NOT USE tag stands in a rack between them.
- Each rod is pushed through its machine's field, tip first, at the bottom, middle and top, while the receiver's lamp
  is watched: it goes red while a beam is interrupted.
- One curtain has a dead beam channel (an open channel) in the middle of its field; **which machine it is is drawn at
  random every trial**. The 30 mm rod covers a working beam at most heights, so that lamp still goes red at the bottom
  and top, but where the rod covers only the dead beam (axis 107 to 127 mm) it stays green.
- The tag goes in the holder of the machine whose curtain missed the rod. The other machine is left available.

The skill is the whole check: handling a test rod so that its tip, not the hand, goes through the field level and square
at three heights, reading the lamp, and then tagging the right machine. Tagging without testing, tagging the sound
machine, testing without ever covering the dead beam alone, or testing and leaving both available all fail.

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
- Which machine is faulty is **drawn at random** by the task file at the start of every run and every episode
  (`draw_faulty()`, the log prints `faulty machine this episode: ...`); the real station's person does it by coin
  flip (see the real-world check). Setting `LIGHT_CURTAIN_FAULTY=left` or `right` in the environment fixes the draw (used
  to film and to check each case); it changes nothing else. The demo is a fixed open-loop script for the machine drawn
  when it is built; it reads nothing from the simulator.

## Scoring

Four steps, 0.2 each, judged as things stand at the end except steps 1 and 2, which count once reached.

| Step | Done when |
|---|---|
| 1. Left field tested | a rod has been pushed through it (axis within 15 degrees of the field's normal, tip 4 cm beyond the plane, crossing between the posts, the rod's middle still in front of the plane) three times: with its axis at 52-102 mm (bottom), at 108-126 mm (middle: the rod covers the dead channel's beam at 117 mm and no working beam; the lamp of a faulty curtain stays green there) and at 152-202 mm (top). A middle pass anywhere else in 102-152 mm does not count: a working neighbour beam trips the lamp and hides the fault |
| 2. Right field tested | the same |
| 3. Tag decision | the tag stands in the holder of the machine whose curtain missed the rod (lower edge's middle within 12 mm along the slot, 4 mm across, -2/+4 mm of the slot floor, within 6 degrees of upright), it was put there after that curtain's three passes (the exposing middle pass included) were done, and it is not in the other holder |
| 4. Let go | with steps 1 to 3, both rods are out of both fields and no gripper is within 3 cm of the tag or either rod |

1.0 (ends the run) only once all four steps have held together without a break for 1 s, with the rods and the tag within
2 mm and 2 degrees of where they were when that second began. It is timed in the task file; any lapse restarts it. Step
points stop at 0.8, also while it is being timed. The tests check no 1.0 at 0.95 s, 1.0 at 1 s, a lapse restarting it,
and no 1.0 while any one condition is missing. A rod or the tag off the table scores 0.

Shortcuts the tests and runs cover: doing nothing 0; tagging without testing 0; testing both and tagging the sound
machine 0.4; testing both and no tag 0.4; tagging first, then testing 0.4; testing and tagging only the faulty machine
0.4; testing the sound curtain fully, the faulty one with a middle pass off the dead beam (e.g. at 140 mm), and tagging
the faulty machine 0.2 (that curtain's test never exposed the fault, so step 3 and the full score cannot count);
a rod that stops short of the plane, tilts, goes beside the posts, or runs at the table is no pass.

## Run it

```bash
# Dream-1 run (1,200 steps = run_limit); replace --policy dream by --policy yam_scripted for the demo
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=light_curtain_test.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/lc-dream1
```

The log prints a `[light_curtain] t=...  steps done: ...` line when a step is done or undone, plus a trace every second:
the passes made at each curtain, both lamps' colours and which holder the tag stands in.

**The demo.** On Isaac (L40) it ran with each machine drawn as the faulty one (`LIGHT_CURTAIN_FAULTY=left`, then `right`),
and a third time with the left one for the film: 1.0 every time, at step 799 (left, twice) and 798 (right). Both curtains tested by
18.5 s, the tag stood in the faulty machine's holder at 36.45 s, success conditions met at 39.4 s. The middle passes counted at
axis 115.2 and 115.3 mm (inside the 108-126 mm exposing band), the bottom ones at 75.4 mm and the top ones at 163.3 mm. Timeline:
both arms work at once. Each, from above with its fingers closing across the rod's middle, lifts its rod off the saddles
and pushes it through its field at 77, 117 and 165 mm (axis commanded), pausing 0.8 s in the field and drawing it back each time,
then sets it back on the saddles and lets go (both curtains tested by about 18.5 s). The arm of the machine that missed the rod
(the demo is built for the machine drawn when it starts) then takes the tag from the rack by its upper part and stands it in that
machine's holder (about 36.5 s), lets go and lifts away; full success follows a second later (step 799, about 40 s).

## Instruction

It says what to do (test each curtain with its rod at three heights; tag a machine whose curtain fails). It does not say
which machine fails, how to hold the rod, or what the lamp shows.

## Real-world check

No real safety curtain is ever altered (taped, jumpered or opened): a curtain that misses a test piece in service is
taken out of service, it is not made to miss one. The check uses two **bench test fixtures** that are built like the
simulated curtains and guard nothing (no machine behind them can move):

- **Fixture** (one per machine): two 40 x 40 mm posts 400 mm apart with 8 through-beam emitter and receiver pairs at 20 mm
  pitch behind 10 mm lenses, 57 to 197 mm above the table (30 mm resolution, 150 mm protected height). The receiver board
  ORs the 8 channels into a red/green lamp: red while any enabled channel is interrupted, green while all are clear (a
  real curtain's OFF/ON state). Each channel reaches the OR through its own labelled two-pin link on the board.
- **Documented fault: an open channel.** Channel 4 (the beam 117 mm up) has its link removed; its emitter and receiver still
  work, but its output never reaches the lamp, so that beam can no longer trip it. A fitted link is the sound state. It is
  reproducible (a link in or out), safe (no hazard, no cover over any lens) and gives exactly the simulated behaviour:
  the lamp is green only while the rod covers no working beam, which for a 30 mm rod on a 20 mm pitch means its axis
  within 10 mm of 117 mm.
- **Inspection method.** The test-piece check of IEC 61496-2 as in curtain manuals: a test piece of the curtain's
  resolution (30 mm, 300 mm long rod) is moved into the protected field, tip first and square to it, and the receiver must
  show interrupted wherever the rod sits in the field; a lamp that stays green anywhere is a failed curtain. Here the rod
  is placed at three heights (bottom, the dead channel's height, top). The task tests the manipulation and the decision;
  a real curtain's manual governs the real procedure.
- **Reset.** Before each trial a coin flip chooses which fixture has link 4 out (the other has all eight in; note the
  draw). The person checks each fixture by hand with the rod: the sound one stays red all the way up, the faulty one goes green
  only with the rod near 117 mm. Rods back on their saddles, the tag in the rack, the holders empty.
- **Review.** Each rod passes through its field at three heights with its tip first and the hand outside the field; the lamp
  goes red at each pass on the sound fixture and stays green on the faulty one at the middle pass; the tag ends in the
  holder of the faulty fixture only, after that fixture's middle pass.
