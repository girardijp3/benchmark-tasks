# Close each flask with the ground-glass stopper that matches its neck

Two Erlenmeyer flasks stand open, a 125 mL one with a 24/29 standard-taper neck and a 50 mL one with a 14/23 neck. In front
of them stand two solid ground-glass stoppers, a 24/29 and a 14/23. The robot closes each flask with its own stopper: the
cone goes into the tapered socket until the head rests on the rim, and everything is let go and left standing.

The size matters physically: the 24/29 cone is wider than the 14/23 neck and will not enter it; the 14/23 stopper slides
straight down the 24/29 bore and is lost inside (scores 0).

| File | What it is |
|---|---|
| `flask_stopper_match.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`), scripted `demo()` |
| `assets/flask_24_29.usda`, `assets/flask_14_23.usda` | The flasks (OpenUSD), rigid bodies, socket from 24 convex wedges |
| `assets/stopper_24_29.usda`, `assets/stopper_14_23.usda` | The stoppers (OpenUSD), rigid bodies, cone and head hulls |
| `test_flask_stopper_match.py` | Offline checks: loads, every scoring outcome, the 1 s timing, reach |
| `task.json` | The task's details for the board |

## Assets

Created for this task by its authors, licensed CC-BY 4.0. Metres and kilograms, Z up, visual meshes plus convex colliders.

**Joints.** ISO 383 / DIN 12242 standard tapers, 1:10 on the diameter: 24/29 is 24.0 mm at the large end and 29 mm long
(21.1 mm at the small end); 14/23 is 14.5 mm and 23 mm long (12.2 mm). Each socket is a ring of 24 wedge colliders whose
inner faces form the cone (plus 24 lead-in wedges, 2.5 mm chamfer). Each stopper cone is 0.1 mm smaller in radius (a ground
joint is lapped to fit; 0.1 mm of play keeps the contact model stable), so the stopper's head, not the taper, is what stops it.

**Flasks.** Typical Erlenmeyer sizes (not a specific catalogue part): 125 mL: 67 mm base, 125 mm tall, 34 mm neck, 85 g;
50 mL: 50 mm base, 95 mm tall, 22 mm neck, 45 g. The bore floor is the solid shoulder, 40 mm below the rim (the real
interior is hollow: a stopper that falls in lands in the flask, treated as lost). Joint size is printed in blue on a white
label on each flask and on each stopper's head (thin visual geometry).

**Stoppers.** Solid glass (2.23 g/cm3): 24/29 with a 40 mm head 8 mm thick, 45 g; 14/23 with a 24 mm head 6 mm thick, 15 g
(typical head sizes). Glass friction 0.5 static / 0.4 dynamic.

## Run it

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flask_stopper_match.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 450 --store-frames --log-dir ~/yam-eval/flask-demo
```

**The demo.** Open loop. Left arm: straight down on the 24/29 stopper's head (fingers across it, about 20 N), lifts it 6 mm
clear of the rim, moves over the neck, lowers the cone into the socket over 3 s (commanded 3 mm past seated: the arm presses a
few newtons), lets go and lifts away. Right arm: the same with the 14/23 stopper and the small flask. In a local test the
plan reaches 1.0 at step 333.

## Instruction

The goal only: both flasks closed with the stoppers that belong to them, pressed home, everything let go. It does not say
which stopper goes where or how to carry it.

## Scoring

Four steps, 0.2 each: (1) 24/29 stopper seated in the 125 mL flask (head underside -0.5 to +1.5 mm of the rim, centred 1 mm,
axis 2 degrees, flask upright and at its place); (2) the same for the 14/23; (3) both at once; (4) with 3, no gripper pad within
3 cm of any part. 1.0 only after every condition has held for 1 s, timed in the task file (`SUCCESS_HOLD_S = 0`), parts within
2 mm / 2 degrees; step points stop at 0.8. 0 if a stopper goes down a neck, a flask tips over 45 degrees, or a part leaves
the table.
