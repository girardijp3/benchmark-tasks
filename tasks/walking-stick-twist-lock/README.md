# Lengthen an adjustable walking stick to its red mark and lock it, a job for an owner who cannot twist it

A home-care task. An adjustable walking stick is stowed in its sleeve, and the robot has to lengthen it to the red mark and lock it. The instruction states only the goal; that the tube has to be turned out of its notch to be freed, drawn up the slots, and turned into the upper notch to lock, is the robot's to work out.

The skill is a turn-pull-turn sequence on a held part: a wrist twist to a slot with about 13 degrees of tolerance, a lift of 50 mm along the slot with 3 mm of side clearance, and a twist of about 52 degrees to a hard stop, with the lock verified by geometry.

| File | What it is |
|---|---|
| `walking_stick_twist_lock.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/stick_sleeve.usda` | The sleeve with its slots and notches on a base plate (OpenUSD); fixed |
| `assets/stick_inner.usda` | The upper tube with lugs, red band and T handle (OpenUSD); one free rigid body |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; boxes, capsules, spheres) |
| `test_walking_stick_twist_lock.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Sleeve** (fixed): a 32 mm ring (bore 24 mm) 100 mm tall on a 6 mm base plate, built of 10 degree sector prisms: two 56 degree slots 180 degrees apart, and, 14 mm tall, two notches that continue 38 degrees to the side (stowed, floor at 6 mm; mark, floor at 56 mm). Aluminium friction 0.35 / 0.30.
- **Stick** (free body, 0.20 kg): a 20 mm tube 120 mm long, two 6 x 8 mm lugs reaching to a 14.5 mm radius at its bottom, a red band at 44 mm and a 40 x 12 x 10 mm T handle bar on top.

**Building it for a real trial.** Sleeve: 6082 aluminium, 32 mm OD x 100 mm with the slots and notches milled (a slot 9 mm wide, notches 14 mm tall); stick: 20 mm aluminium tube with two M3 pins as the lugs, a T handle bar on top. A real stick would add a friction cam; none is modelled.

## Layout

The sleeve axis is at x 0.30, y -0.12; the stick stands in it stowed. Locked at the mark its tube bottom is 56 mm above the table and its lugs are turned about 52 degrees. The right arm does the whole demo.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=walking_stick_twist_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/walking-stick-twist-lock-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=walking_stick_twist_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 450 --store-frames --log-dir ~/yam-eval/walking-stick-twist-lock-demo
```

The log prints `[stick] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm pinches the T handle bar across its 12 mm width (closing along y, pad tips 6 mm below its top, about 20 N a finger), turns the stick back about 48 degrees until the lugs are in the slots, draws it 56 mm up to the red mark, turns the wrist 58 degrees (the notch stops the stick at about 52) so the lugs are in the upper notch, lowers it onto the notch floor, lets go and goes home. Success at about step 369 (18 s).

## Scoring

| Step | Done when |
|---|---|
| 1 | the stick was lifted 15 mm at some time (turned out of its notch) |
| 2 | drawn up to the mark: bottom within -4 to +20 mm of the locked height, on axis, within 6 degrees of upright |
| 3 | locked: bottom on the notch floor (6.5 mm at most) and turned 44 to 62 degrees |
| 4 | with step 3, no gripper pad within 3 cm of the stick and none holds it |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success():
  all steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the stick is lost off the table, or drawn right out of the sleeve); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: a stick lifted without turning it back cannot rise (the notch ceiling stops it), one at the mark but turned back to the slot is drawn and not locked, one held up 12 mm is not locked, one left stowed has not been drawn (all tests); a locked stick pulled by the gripper stays within about 4 mm (MuJoCo).
- **Real station:** judged from video: the red band shows above the rim and the stick stays at that length when released.

## Reset

Stand the stick in the sleeve at x 0.30, y -0.12, lugs resting in the bottom notches (turned 48 degrees); nothing else is moved.

## Status

Dream-1 score: 0.0 (2400 steps, hit the step limit). Demo: 1.0 at step 369 (18.4 s). Media M-NVWZKK.
