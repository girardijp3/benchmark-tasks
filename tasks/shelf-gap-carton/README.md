# Stock a shelf row where no gap is wide enough: slide the cartons together, then stand the new one in

A shelf-stocking task. An open shelf bay holds a row of four retail cartons with small gaps between them. A fifth
carton (green) is 74 mm wide, and no single gap is wider than 30 mm, but together the gaps add up to 120 mm. The robot
has to make room by sliding neighbouring cartons together, then stand the green carton in the row between the blue and
the red one. Only the final state is scored: how the room was made (pushing, or lifting and setting down) does not
matter. The instruction names the two neighbours and says nothing else; that every other carton stays standing and in
its place in the row is part of the job.

The skill is planning what to move and in which order (the gap between blue and red opens only by shifting the blue
carton left and the red and white cartons right, each by a different amount), and handling light tall cartons without
toppling them.

| File | What it is |
|---|---|
| `shelf_gap_carton.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/*.usda` | Shelf bay and five cartons (OpenUSD, box primitives); `tools/build_assets.py` writes them |
| `test_shelf_gap_carton.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the named product class (not one catalogue entry), stated here.

- **Shelf bay** (fixed): the board is 18 mm melamine-faced chipboard (the usual shelf board thickness), 436 x 220 mm;
  two 18 mm end dividers 140 mm tall with 400 mm clear between them (a common 400 mm shelf module); a 3 mm hardboard
  back panel. Surfaces have friction 0.5 static / 0.4 dynamic.
- **Cartons** (free, printed cardboard; width x depth x height, mass; friction 0.55 / 0.40 against the board):
  yellow 100-bag tea carton 80 x 60 x 130 mm, 240 g; blue 40-bag herbal-tea carton 62 x 70 x 105 mm, 95 g; red
  instant-soup sachet carton 78 x 52 x 120 mm, 180 g; white over-the-counter tablet carton 60 x 45 x 110 mm, 60 g;
  green 80-bag tea carton 74 x 62 x 125 mm, 200 g. Heights 105-130 mm. Masses are typical filled-carton values.
  Each is one box collider with a printed panel and a band as thin visual geometry.

## Layout

Row (from the robot's right to its left, y in mm, 400 mm between the dividers): gap 20 | yellow (80) | gap 20 | blue
(62) | gap 26 | red (78) | gap 24 | white (60) | gap 30. Green stands on the table at x 0.22, y -0.14. To fit green
(74 mm, plus slack) the blue-red gap must grow from 26 to at least 78 mm; the demo shifts blue 16 mm left, red 46 mm and
white 28 mm right, leaving about 90 mm.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shelf_gap_carton.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/shelf-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shelf_gap_carton.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 640 --store-frames --log-dir ~/yam-eval/shelf-demo
```

The log prints `[shelf_gap] t=...  steps done: ...` on every change and a trace each second (the blue-red gap and the
green carton's position).

**The demo.** Open loop: fixed waypoints, no simulator reads. Each arm points its gripper down, leaned 35 degrees away
from its own base, with the pads closing across a carton's depth near its top (so the fingers stay in front of and
behind the carton, never in the gaps). To move a carton it pinches it, lifts it 8 mm, slides it along the row, sets
it down and lets go. Right arm: blue 16 mm left (against yellow). Left arm, meanwhile: white 28 mm right (against the
divider), then red 46 mm right. Then the right arm picks the green carton from the table (base lifted 16 mm over the
board), carries it to the opened gap, slides it in from the front, sets it down, lets go and rises. All steps by about
step 510, success at step 534 in testing. The gap ends about 87-93 mm wide for the 74 mm carton.

## Scoring

Final state only: lifting, pushing, sliding or replacing cartons is never penalised.

| Step | Done when |
|---|---|
| 1 | the blue-red gap is at least 78 mm (green's width plus 4 mm), blue and red standing on the board |
| 2 | green stands on the board (base within 6 mm of its top, within 10 deg of upright, front within 20 mm of the row's front line, sides inside the dividers), its centre between blue's and red's |
| 3 | all five so stand, no two overlap more than 2 mm, and from the right they run yellow, blue, green, red, white |
| 4 | step 3 and no gripper pad within 3 cm of any carton |

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file (`SUCCESS_HOLD_S` is
  0): all four steps and every carton within 2 mm and 2 deg of where it was when that second began. A lapse restarts
  the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** only if a carton falls off the table at any time (tracked all run).
- Doing nothing scores 0. Shortcuts: green on top of the row, in front of it, lying down, at the row's end, or any
  carton toppled or swapped do not complete steps 2 and 3 (tests); placing green without making room is not possible
  (the open-loop demo with the shifts removed scores 0).
- **Real station:** judged from video and a ruler: all five upright on the board in order, green between blue and red.

## Reset

Stand the four cartons in the row as at the start (gaps 20, 20, 26, 24, 30 mm), fronts 45 mm behind the board's front
edge; stand the green carton on its mark on the table.

## Status

Dream-1 score: 0.0 (both arms moved up from their bases and the right arm reached toward the green carton on the table, but nothing was lifted or slid: the gap between the blue and red cartons stayed 26 mm for the whole 2,400 steps). Demo: 1.0 (gap opened at 13.4 s, green carton in the row at 24.75 s, success conditions met at 26.2 s, success at step 534). Media: TBD.
