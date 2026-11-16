# Press and twist a taper candle into a free brass candlestick

Setting a dinner table, at home or before service in a restaurant or hotel: a candle goes into each brass candlestick. A dinner candle's
base is a short taper made to wedge in the standard 7/8 in socket; pressed and twisted down to the floor it stands upright by itself, and a
candle left half-seated leans, drips onto the cloth and can topple once lit. The candlestick is free on the table (not clamped), so the push
must not tip or slide it. The core is a free candlestick and the press-and-twist into a snug socket. In the demo one arm
pinches the candlestick's stem and steadies it (one arm would do) while the other presses the candle's tapered base into the cup with a small twist until
it is wedged to the floor and stands upright by itself. The instruction states only the goal; where to pinch each part and how to
seat the taper are the robot's to work out.

| File | What it is |
|---|---|
| `candle_in_holder.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` (both arms) |
| `assets/candle.usda` | The candle (OpenUSD); free rigid body |
| `assets/candlestick.usda` | The candlestick (OpenUSD): one free articulation, body plus 12 sprung socket walls |
| `build_assets.py` | Writes the USD files in `assets/` (pxr) |
| `test_candle_in_holder.py` | Offline checks: every scoring outcome, the 1 s timing, the tip and slide events, both arms' reach |
| `task.json` | The task's details for the board |

## Assets and sources

Created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres, kilograms, Z up.

- **Candlestick** (free): Prospect Hill Co. altar candlestick, 4 in high, 5 in (127 mm) base, 7/8 in socket, 12 oz (340 g) per the maker's
  listing. A turned polished-brass profile (beaded domed foot, knop, waisted stem, flared cup); a 4 mm 45 degree flared mouth is added
  above the 4 in rim. The socket is 12 wall segments on radial prismatic joints (7,700 N/m each, 0.6 mm travel, 22.2 mm across at rest, 22 mm deep).
- **Candle**: 8 in (203 mm) taper-base dinner candle, 46 g, wax friction 0.40 / 0.35: base end 20.0 mm across widening over 25 mm
  to 23.4 mm, 17 mm at the top. Mass scaled from 10 in beeswax tapers listed at 2 oz (e.g. Register Family Farm; a 10 in 7/8 in taper is
  listed by Keystone Candle). The base taper is a design value for a taper-base candle, not measured on a product. Friction: candle to brass
  0.40, candlestick foot to table 0.35. The press-and-twist is a 20 degree wrist turn. No wick or flame contact.

## Simplifications and estimates (stated plainly)

- The base taper (20.0 to 23.4 mm) and the wall stiffness are design values chosen so a press of about 8 N seats the candle; the **seating
  push (8 N peak, pull-out about 3.5 N) is a simulator value, not a measurement** on a real candle and cup. Measure it with a kitchen scale.
- The sprung walls stand for the wax's give; the candle and brass are otherwise rigid.
- The press is commanded about 25 mm below the floor because the arm's tip stiffness is about 0.4 N per mm (it then pushes with about 10 N).
- **The second hand is not required.** With the other arm idle the demo also scores 1.0: the 340 g, 127 mm candlestick neither tips nor
  slides under a straight 8 N press (peak sideways force 1.4 N against about 4 N of friction, stick moved 0.0 mm). The steady hand in the demo is
  real-use practice. Tip (10 degrees) and slide (10 mm) are scored rule events all the same.
- **Run limit 480 steps** = the demo's success at about step 370 plus about 30 %.
- **Start window:** the open-loop demo is reliable only at the nominal start (about 1 mm). In MuJoCo it also succeeded with the candle shifted
  1 mm to -x, 1 mm to -y, 2 mm to -x, 2 mm to +y and 2 mm to -y; it failed at 1 mm to +y, 2 mm to +x and every 3 mm shift (ends wedged, leaning).

## Layout

The socket's axis is at x 0.30, y -0.08; the candle starts at x 0.26, y -0.20. The left arm pinches the stem from the side (gripper
leaned 50 degrees, pads closing along x, 20 mm gap closing to 16 mm), the right arm (leaned 40 degrees) the candle 65 mm up.

## Scoring

| Step | Done when |
|---|---|
| 1 | the candle was lifted 15 mm off the table at some time |
| 2 | its tapered base is wedging: 16 mm below the rim, axis within 5 mm, within 8 degrees of upright |
| 3 | wedged to the floor: 20.5 mm below the rim, axis within 1.5 mm, within 2 degrees of upright |
| 4 | with step 3, no gripper holds the candle or the candlestick and none is within 3 cm of the candle |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success():
  all steps, no rule broken, and the candle and the candlestick each within 2 mm and 2 degrees of where they were when that second began.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event: the candle or candlestick off the table, the candlestick tipped over 10 degrees or slid more than 10 mm.
- **Real station:** judged from video: the candle stands upright, seated in the cup, the candlestick upright where it started (square and ruler on the candle).

## Run it

On the YAM Isaac Lab-Arena station, from this folder (`task.json`'s instruction word for word):

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=candle_in_holder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 480 --store-frames --log-dir ~/yam-eval/candle-in-holder-dream
```

## Status

Instruction wording: lengthened on 2026-11-16 to the board's 15-35 words (was "Put the candle in the candlestick so that it stands upright
by itself."); the Dream-1 run below used that earlier wording, which asks for the same goal.

Isaac Lab-Arena: scripted demo 1.0 at step 371 (18.5 s; the candlestick shifted 1.8 mm while the stem was pinched, tilt of the wedged candle 1.6 degrees); Dream-1 score: 0.2 (480 steps, hit the step limit). The simulator-only seating push and the start window are stated above. Media M-HL2LGE.
