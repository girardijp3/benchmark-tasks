# Take the out-of-service tagged vortex mixer to the repair cart

Three mini vortex mixers stand in a row on the bench, each with a paper equipment tag and number on its front face. One tag is
red (out of service), two are green. A low repair cart with a flat tray stands beside them. The robot moves the red-tagged mixer
onto the tray, standing upright, and leaves the working mixers where they are.

| File | What it is |
|---|---|
| `tagged_mixer_cart.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`), scripted `demo()` |
| `assets/mixer_a.usda`, `mixer_b.usda`, `mixer_c.usda` | The mixers (rigid bodies); `mixer_b` carries the red tag |
| `assets/repair_cart.usda` | The cart (fixed): deck and tray with a rim |
| `test_tagged_mixer_cart.py` | Offline checks: loads, every scoring outcome, the 1 s timing, reach |
| `task.json` | The task's details for the board |

## Assets

Created for this task by its authors, licensed CC-BY 4.0. Metres and kilograms, Z up. Mixer: 110 x 80 x 65 mm body, 45 mm rubber cup 15 mm
tall, 0.45 kg (typical values for a small bench vortex mixer; the 80 mm width fits the gripper's 94 mm opening). Colliders: one box and one
cylinder per mixer, friction 0.6/0.5. Tag: a 60 x 36 mm card, red (0.82, 0.10, 0.10) or green, with the four-digit number in white
strokes 17 mm tall (thin visual geometry, no collider). Cart: 30 mm deck, flat tray 160 x 200 mm inside a 12 mm rim, five box colliders, fixed.

## Run it

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tagged_mixer_cart.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/mixer-demo
```

**The demo.** Open loop. The left arm comes straight down on the red-tagged mixer's long sides (fingers across the 80 mm width, about
20 N a finger), lifts it 7.5 cm, carries it over the cart, lowers it onto the tray floor, lets go and lifts away. In a local test it
reaches 1.0 at step 296, also with every mixer's start moved 3 mm and turned 3 degrees in three directions.

## Instruction

Goal only: the tagged unit onto the tray, upright, the working ones left alone. It names no colour, tag or technique beyond the tag itself.

## Scoring

Four steps, 0.2 each (lifted in a gripper; standing on the tray; working units undisturbed; let go). 1.0 only after every condition has
held for 1 s, timed in the task file (`SUCCESS_HOLD_S = 0`), parts within 2 mm / 2 degrees; step points stop at 0.8. 0 if any mixer tips over
45 degrees, a working mixer is put over the tray, or a mixer leaves the table. Pushing the red mixer into the tray without lifting it gets
no credit for the tray step.
