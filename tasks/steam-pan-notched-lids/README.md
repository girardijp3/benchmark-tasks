# Cover the serving-line pans: the right lid on each, spoon left in its pan

A bimanual lid-placing task at full size. A self-serve line holds two clear polycarbonate food pans with food in them:
a GN 1/3 pan with a serving spoon lying in it and a GN 1/6 pan without a utensil. Three clear lids lie flat on the
table beside them: a plain GN 1/6 lid, a plain GN 1/3 lid and a GN 1/3 lid with a utensil notch in one end.
- Each pan has to end up under a lid seated flat on its rim, with the food covered.
- The spoon's handle rests over the GN 1/3 pan's end rim. A plain lid set on that pan rests on the handle instead of
  the rim: about 4 mm proud and tipped, not seated. Only the notched lid, with its notch over the handle, sits flat.
- The robot is told only to cover the food and leave the spoon in its pan. Which lid goes where is for it to see.

The skill is lid handling at real size: lifting a light plastic lid by its bridge handle, carrying it level over a pan,
and setting it on an 8 mm rim flange within a few millimetres without disturbing a utensil.

| File | What it is |
|---|---|
| `steam_pan_lids.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/pan_third.usda`, `assets/pan_sixth.usda` | The two pans, fixtures (OpenUSD) |
| `assets/lid_third_notched.usda`, `assets/lid_third_solid.usda`, `assets/lid_sixth_solid.usda` | The three lids, free rigid bodies |
| `assets/spoon.usda` | The serving spoon, a free rigid body |
| `test_steam_pan_lids.py` | Offline checks: the file loads, every scoring outcome, the 1 s full-success timing, reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All parts: created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Units are metres and kilograms, Z up. Everything is at true size.

- **Pans** (fixtures): GN 1/3 (325 x 176 mm) and GN 1/6 (176 x 162 mm) food pans, 65 mm deep, the nominal footprints of
  EN 631-1 (catalogue depths run 20-200 mm). An 8 mm rim flange (typical catalogue value; its top face, 65 mm up, is
  where the lid rests), straight 2 mm walls and a 2 mm floor as box colliders (clear polycarbonate pans are 1.5-2.5 mm;
  real walls taper a few degrees). Food is a visual slab, no collider.
- **Lids** (free): 2 mm polycarbonate sheet (typical for clear GN lids), as long and wide as the pan, with a bridge
  handle (two 12 mm legs and a 7 mm bar, 80 mm long, 12 mm wide, 20 mm high; typical of molded lid handles). Masses
  follow from sheet area at 1,200 kg/m3 plus 12 g for the handle: GN 1/3 plain 149 g, GN 1/3 notched 145 g, GN 1/6
  plain 80 g. The notched lid has a 50 x 35 mm rectangular slot in the middle of one short end (catalogue utensil-notch
  lids use 40-70 mm slots; real ones are rounded). Friction (static/dynamic) 0.4/0.3, restitution 0.
- **Spoon** (free, 90 g): a 60 mm bowl (a 30 mm sphere as its collider) and a 12 x 5 mm flat handle from 26 to 210 mm
  from the bowl's centre, about 240 mm overall (catalogue serving spoons are 90-150 g). It starts as it comes to
  rest: the bowl on the pan's floor, the handle's underside on the inner edge of the far end rim, rising 19.7 degrees,
  its tip 38 mm above the rim and 70 mm past the pan's end; the handle crosses the flange inside the notch's width.
- **Not modelled:** the serving-line counter and cold or hot wells; the pans stand on the table. Food does not collide.
  Handles are boxes, the notch is rectangular, walls are straight.

## Layout (m; x forward, y to the robot's left, from the midpoint between the arm bases)

One row across the table, long axes along x, all centred at x 0.31, left to right: plain GN 1/6 lid (y 0.469), GN 1/6
pan (0.287), plain GN 1/3 lid (0.098), GN 1/3 pan with the spoon (-0.098), notched GN 1/3 lid (-0.294). The notched
lid lies with its notch end away from the robot, the way the spoon's handle points. The left arm reaches the GN 1/6
lid and pan, the right arm the notched lid; the tests check the right arm cannot reach the GN 1/6 job and the left arm
cannot reach the notched lid, so both arms are needed.

## Run it

On the YAM Isaac Lab-Arena station, from this folder, with the Arena environment active. The instruction is
`task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=steam_pan_lids.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/steampan-dream

# Scripted demo (both arms), 720 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=steam_pan_lids.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 720 --store-frames --log-dir ~/yam-eval/steampan-demo
```

The job log prints a `[steam_pan_lids] t=...  steps done: ...` line when a step is done or undone, plus a trace every
second: each lid near a pan (its underside's height over the rim, tilt, the share of the opening it covers, whether a
gripper holds it) and how far the spoon has moved.

**The demo.** Open loop: a fixed waypoint script that never reads a simulated pose; each arm in turn.
- Left arm: from above, fingers closing across the plain GN 1/6 lid's 12 mm handle bar (pad tips 6 mm below the bar's
  middle), lifts the lid to 25 mm above the rims, carries it over the GN 1/6 pan, lowers it onto the rim, opens (the
  lid drops 0.5 mm), lifts away and goes home.
- Right arm, once the left has lifted away: the same with the notched GN 1/3 lid onto the spoon pan, the notch end over
  the spoon's handle.
- The fingers closing on the handle square the lid up: the demo reaches 1.0 from starts nudged 3 mm and 3 degrees.

## Instruction

One sentence of goal: put lids on the two pans so no food is left uncovered and leave the spoon in its pan. It says
nothing about notches, lid sizes or which lid goes on which pan.

## Scoring

Four steps, **0.2 each**, judged as things stand at the end.

| Step | Done when |
|---|---|
| 1. Spoon pan covered | a lid rests on the GN 1/3 pan (in no gripper's hold, within 3 cm of its rim and within 10 degrees of level) with at least 95% of the pan's opening under its plate |
| 2. Seated flat, spoon in | with step 1, that lid's underside within 1.5 mm of the rim's top face, tipped less than 1 degree, centred within 6 mm (both ways) and square within 2.5 degrees; and the spoon within 2 cm and 6 degrees of where it lay |
| 3. Second pan covered fully | a different lid rests on the GN 1/6 pan with at least 99.5% of its opening under the plate |
| 4. Seated flat | with step 3, that lid seated on the same terms as in step 2 |

- **1.0 (success, ends the run):** every full-success condition has held together, without a break, for 1 s: all four
  steps, every gripper at least 3 cm from every lid and the spoon, and the lids and the spoon each within 2 mm and
  2 degrees of where they were when that second began. The arms' return is not scored.
- **0.2 per step, at most 0.8:** otherwise, including while that second is being timed.
- The 1 s is timed in the task file (`_Progress`, `met_since`); `SUCCESS_HOLD_S` is 0. The tests check: no 1.0 at 0.95 s,
  1.0 at 1 s, a lapse restarts the second, and no 1.0 over 3 s while any one condition is missing (either lid unseated
  or off its pan, the spoon moved or turned, a lid still in a gripper, a gripper within 3 cm, a lid or the spoon off the
  table, a lid or the spoon still moving).
- **0** if a lid or the spoon falls off the table, the spoon is thrown out of its pan, or the state is not finite.
- **Why the notched lid, by physics and by one declared rule.** On the spoon pan, a plain lid is held up by the handle
  (measured in simulation: about 3.8 mm above the rim, tipped 1.3 degrees), so it is never seated; the notched lid turned
  half a turn does the same. A lid that covers the GN 1/6 pan must leave no opening (99.5% coverage): the GN 1/3
  notched lid does, only when it lies centred (its notch end then hangs clear of the small pan); slid so the notch is
  over the opening it leaves 7%. The coverage rule is the task's one declared judgement: uncovered food is what the
  instruction asks to avoid. A plain GN 1/3 lid on the GN 1/6 pan covers it fully and scores.
- **Shortcuts that do not score:** lids cannot be pushed onto the pans (they lie on the table, the rims 65 mm up);
  nothing is scored for a hovering or held lid; the GN 1/6 lid does not cover the GN 1/3 pan.
- **Real station:** the same is judged with a ruler: a seated lid has no gap at the rim and does not rock; a lid on a
  handle does.

## Reset

Lift the lids off and lay them flat at their places (the notched lid with its notch end away from the robot), lay the
spoon in the GN 1/3 pan with its bowl on the floor and its handle resting on the far end rim.

## Status

Dream-1 score: 0.0 (both arms moved up toward the nearest lids and hovered; no lid was lifted). Demo: 1.0 at step 620 (second pan seated by step 270, all four steps by step 562). Media: M-6RX6L4.
