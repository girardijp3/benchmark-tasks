# Test two machine light curtains over their whole field with test rods and tag out the one that fails

A start-of-shift safety check with a decision in it. Two benchtop machines stand side by side, each guarded by a safety
light curtain (a 30 mm-resolution hand-protection pair, protected height 150 mm). Each machine has its own 30 mm test
rod resting in front of its field and an empty tag holder; a red DO NOT USE tag stands in a rack between them.
- Each rod is moved slowly through its machine's **whole protective field**, as the test-rod check of a real curtain
  prescribes (below): up along the sender post's edge, down the middle, up along the receiver post's edge, full height,
  while the receiver's lamp is watched. The lamp goes red while a beam is interrupted and must stay red throughout.
- One curtain has a dead beam channel (an open channel); **which machine it is, and which of the 8 channels, is drawn at
  random every trial**. The 30 mm rod covers a working beam at most heights, so that lamp is red nearly everywhere, but
  where the rod covers only the dead beam (axis within 9.5 mm of it) it stays green, at whatever height that channel is.
- The tag goes in the holder of the machine whose curtain missed the rod. The other machine is left available.

The skill is the whole check: handling a test rod so that its tip, not the hand, goes through the field level and square,
sweeping the entire field slowly enough to read the lamp, and then tagging the right machine. Tagging without testing,
tagging the sound machine, sweeping only some heights or lanes of a field, and testing and leaving both available all fail.

**The real procedure this follows.** SICK AG, *deTec4 operating instructions* (document 8021645), section 4.5.1 "Test rod
check": with an opaque test rod whose diameter equals the curtain's resolution, (1) move the rod slowly through the area to
be protected, (2) the receiver's field indicator must light red continuously, never green or flashing yellow, then (3)
guide the rod along the edges of the area to be protected with the same continuous-red requirement; a lamp that shows green,
even briefly, means work stops and the installation is checked. The same check is in other makers' manuals and in the
test-piece clauses of IEC 61496-2. Here the "area to be protected" is the field plane of 400 x 150 mm and the rod covers it
as a grid (below); the sender and receiver edges are the two outer lanes, the bottom and top edges are reached by the rod's
lowest and highest positions. A real curtain's own manual governs a real check.

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
| `test_light_curtain_test.py` | Offline checks: the file loads, the lamp rule, every scoring outcome, the 50 mm/s / 3 s lane gate, the 1 s full-success timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |
| `REAL_FIXTURE.md` | The buildable bench fixture: BOM with part numbers and substitutes, wiring schematic, assembly, link/fault logic, draw/reset procedure, trial checklist and safety |

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
the whole lens), plus a 0.5 mm slack so that a rod exactly midway between two beams (the 30 mm resolution limit, pitch 20
+ lens 10) does not flicker the lamp of a sound curtain; a real receiver trips on any partly blocked beam anyway. It follows only the rod's real pose. It never moves, holds, places or releases anything and plays no
part in the score; the score only reads the rods, the lamp's colour at each crossing, the tag and the grippers.
- Because the pitch is 20 mm and the rod 30 mm, a rod always covers at least one whole lens, but with a rod axis within
  9.5 mm of the dead beam's height it covers only the dead beam. The faulty lamp therefore stays green in a 19 mm band of
  axis heights around the dead channel (its 8 channels are at 57, 77, ... 197 mm) and is red everywhere else in the field.
  A sound curtain is red at every height.
- Which machine is faulty, and which channel, is **drawn at random** by the task file at the start of every run and
  every episode (`draw_faulty()`, the log prints `faulty machine this episode: ..., dead channel ...`); the real
  station's person does it by coin flip and die roll (see the real-world check). Setting `LIGHT_CURTAIN_FAULTY=left|right`
  and `LIGHT_CURTAIN_DEAD=0..7` in the environment fixes the draw (used to film and to check each case); it changes nothing
  else. The demo is a fixed open-loop script that sweeps the whole field of both machines the same way whichever channel is
  dead; it reads nothing from the simulator and is built for the machine drawn when it starts.

## Scoring

Four steps, 0.2 each, judged as things stand at the end except steps 1 and 2, which count once reached.

The field of each curtain is a **grid of 3 lanes by 15 cells**: lanes along the sender post's edge, the middle and the
receiver post's edge (rod axis crossing the plane at -150, 0 and +150 mm from the field's middle, within 25 mm), cells of
10 mm of rod-axis height from 52 to 202 mm above the table (so the rod's edge reaches the field's bottom and top edge).
A cell is crossed when, at a control step (20 Hz), a rod is in the field (axis within 15 degrees of the field's normal, tip
4 cm beyond the plane, crossing between the posts, the rod's middle still in front of the plane) with its axis in the
lane's band and the cell's height. The lamp's colour is read at that same step. A lane counts only when one continuous run
crosses all 15 of its cells with the rod's axis moving along the lane at no more than 50 mm/s for at least 3 s (the real
check's 150 mm / 3 s limit, timed with a stopwatch on the video); a faster lane does not count even if every cell was
crossed.

| Step | Done when |
|---|---|
| 1. Left field covered | every one of the 45 cells of the left curtain's grid has been crossed by a rod, with each of the three lanes swept in one continuous run of at least 3 s at no more than 50 mm/s (either rod; the order, direction and any extra movement are free) |
| 2. Right field covered | the same for the right curtain |
| 3. Tag decision | the tag stands in the holder of the machine whose curtain missed the rod (lower edge's middle within 12 mm along the slot, 4 mm across, -2/+4 mm of the slot floor, within 6 degrees of upright), where the lamp was seen not red at a crossing, it was put there after that curtain's grid was covered, and it is not in the other holder |
| 4. Let go | with steps 1 to 3, both rods are out of both fields and no gripper is within 3 cm of the tag or either rod |

Three isolated passes at one place cover about a tenth of a grid and score nothing for the field. A sound curtain's grid is
covered with the lamp red in every cell; the faulty one's grid shows the lamp not red in the cells where the rod covers
only the dead beam, in all three lanes, and the tag counts only after the whole grid, so a sweep that missed a part of
the field cannot be rescued by tagging.

1.0 (ends the run) only once all four steps have held together without a break for 1 s, with the rods and the tag within
2 mm and 2 degrees of where they were when that second began. It is timed in the task file; any lapse restarts it. Step
points stop at 0.8, also while it is being timed. The tests check no 1.0 at 0.95 s, 1.0 at 1 s, a lapse restarting it,
and no 1.0 while any one condition is missing. A rod or the tag off the table scores 0.

Shortcuts the tests and runs cover: doing nothing 0; tagging without testing 0; testing both fields fully and tagging the sound
machine 0.4; testing both and no tag 0.4; tagging first, then testing 0.4; testing and tagging only the faulty machine 0.4;
three passes at one place, or a sweep that leaves one lane, a 10 mm height band or the ends of a lane out, or one made faster
than 50 mm/s (under 3 s per 150 mm lane) even if it crossed every cell, and then tagging the faulty machine 0.2 at most (that
curtain's field was not covered, so step 3 and the full score cannot count); a rod that stops short of the plane, tilts, goes
beside the posts, or runs at the table is no crossing.

## Run it

```bash
# Dream-1 run (1,200 steps = run_limit); replace --policy dream by --policy yam_scripted for the demo
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=light_curtain_test.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/lc-dream1
```

The log prints a `[light_curtain] t=...  steps done: ...` line when a step is done or undone, a line when a field's grid is
fully covered or the lamp is first seen not red, and a trace every second: the cells crossed and lanes swept at each
curtain, both lamps' colours and which holder the tag stands in.

**The demo.** On Isaac (L40) it ran with the left machine faulty (dead channel 3, 117 mm), with the right one faulty (dead channel
6, 177 mm) and once more with the left one for the film: 1.0 every time, with no re-tuning of the MuJoCo script (the success
conditions were met at 47.25 s, 47.20 s and 47.25 s; the runs ended at step 954 with the right machine faulty and step 955 with
the left one, and the film is the step-955 run). Both fields were
fully covered by 24.6 s (the right one at 24.45 s), the tag stood in the faulty machine's holder at about 44 s. Timeline: both
arms work at once. Each, from above with its fingers closing across the rod's middle, lifts its rod off the saddles and carries it
to its field, pushes it in tip first at the bottom of the sender-edge lane, and sweeps it slowly (about 37 mm/s, 4 s per 147 mm
lane, inside the 50 mm/s / 3 s limit): up the sender-edge lane, across the top, down the middle lane, across the bottom, up the
receiver-edge lane (the rod stays in the field the whole time), then draws it back, sets it on the saddles and lets go. To reach
the top of the outer lanes the hand is rolled up to 25 degrees about the rod's own axis (the rod is a cylinder, so the grip on it
does not change). The arm of the
machine that missed the rod (the demo is built for the machine drawn when it starts) then takes the tag from the rack by its upper
part and stands it in that machine's holder, lets go and lifts away; full success follows a second later. The demo path does
not depend on which channel is dead; the lamp's not-red stretch in the faulty machine's field lands wherever that channel is
(checked in MuJoCo for channels 0, 3 and 7 and in Isaac for 3 and 6). The film ends on the full score, held 3 s.

## Instruction

It says what to do (test each curtain's whole field with its rod; tag a machine whose curtain fails). It does not say
which machine fails, which channel, how to hold the rod, which path to take, or what the lamp shows.

## Real-world check

No real safety curtain is ever altered (taped, jumpered or opened): a curtain that misses a test piece in service is
taken out of service, it is not made to miss one. The check uses two **bench test fixtures** that are built like the
simulated curtains and guard nothing (no machine behind them can move):

- **Fixture** (one per machine): two 40 x 40 mm posts 400 mm apart with 8 through-beam emitter and receiver pairs at 20 mm
  pitch behind 10 mm lenses, 57 to 197 mm above the table (30 mm resolution, 150 mm protected height). The receiver board
  ORs the 8 channels into a red/green lamp: red while any enabled channel is interrupted, green while all are clear (a
  real curtain's OFF/ON state). Each channel reaches the OR through its own labelled two-pin link on the board.
- **Build.** `REAL_FIXTURE.md` has the BOM (part numbers, quantities, low-cost sources and substitutes), the wiring
  schematic, the assembly steps, the link/fault logic, the coin-and-die draw and the real-trial checklist. In short:
  eight Vishay TSAL6200 emitters driven at 38 kHz by a 555, eight Vishay TSSP4038 light-barrier receivers behind 10 mm
  lenses, one 2-pin link per channel into an 8-channel LM339 comparator board, a 74HC14 red/green lamp driver and a
  5 V 2 A supply. The comparator outputs are wired-OR: a removed link pulls that input low, so the channel cannot make
  the lamp red; the other seven still can. The low-cost off-the-shelf alternative is eight Adafruit 2168 (or generic
  5 mm) IR break-beam pairs per fixture, wired to the same links and comparator board.
- **Documented fault: an open channel.** One channel (any of the 8) has its link removed; its emitter and receiver still
  work, but its output never reaches the lamp, so that beam can no longer trip it. A fitted link is the sound state. It is
  reproducible (a link in or out), safe (no hazard, no cover over any lens) and gives exactly the simulated behaviour:
  the lamp is green only while the rod covers no working beam, which for a 30 mm rod on a 20 mm pitch means its axis
  within about 10 mm of that channel's height.
- **Inspection method.** The test-rod check of a real curtain: SICK deTec4 operating instructions (document 8021645),
  section 4.5.1 (the same method as in other makers' manuals and the test-piece clauses of IEC 61496-2). An opaque rod of the
  curtain's resolution (30 mm, 300 mm long) is moved slowly through the area to be protected, tip first and square to it,
  then guided along the edges of that area, while the person watches the receiver's lamp, which must stay red the whole
  time; green, even briefly, is a failed curtain. Here the area is the 400 x 150 mm field, covered as three vertical lanes
  (along the sender post's edge, the middle, along the receiver post's edge) swept over the full height, which reaches all
  four edges. A person marks the lanes with tape on the table in front of the field (150 mm either side of the middle, 25 mm
  tolerance) and times each sweep at 3 s or more per 150 mm lane (at most 50 mm/s, with a stopwatch on the video; the task
  applies the same 3 s / 50 mm/s gate). The task tests the manipulation and the decision; a real curtain's manual governs the
  real procedure.
- **Reset.** Before each trial a coin flip chooses which fixture has a link out and the roll of an eight-sided die
  chooses which channel; the other fixture has all eight links in (note the draw). The link map is CH0 = 57 mm,
  CH1 = 77 mm, ..., CH7 = 197 mm; the full reset and checklist are in `REAL_FIXTURE.md`. The person checks each
  fixture by hand with the rod: the sound one stays red all the way up, the faulty one goes green only with the rod near
  that channel's height. Rods back on their saddles, the tag in the rack, the holders empty.
- **Review.** Each rod covers its field's three lanes over the full height, tip first, the hand outside the field and the
  rod level; the lamp stays red throughout on the sound fixture and goes green on the faulty one where the rod covers only the
  open channel, in every lane; the tag ends in the holder of the faulty fixture only, after that fixture's whole field was
  covered. A sweep that skips a lane, a stretch of height or goes too fast does not count.
