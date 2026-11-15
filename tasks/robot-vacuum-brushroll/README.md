# Refit a robot vacuum's brush roll in its bay and latch the drive-end retainer

A household maintenance task. A robot vacuum's brush roll lies on the table in front of the vacuum's open brush bay. The robot has to refit it so both ends are seated, then latch the retainer flap over the drive end. The instruction states only the goal; that the roll has to be carried over the bay and lowered square so both stubs find their slots, and the flap pushed down along its arc, are the robot's to work out.

The skills are two distinct ones: gripping a 28 mm cylinder across its diameter and lowering it so an 11 mm round stub and an 8 mm square stub enter two slots 216 mm apart (about 4 mm of play along the roll, a 45 degree lead-in on each slot), and then working a spring-held flap closed with the closed fingers until its latch catches.

| File | What it is |
|---|---|
| `robot_vacuum_brushroll.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/roll.usda` | The brush roll: round stub, body, square stub, one free rigid body (OpenUSD) |
| `assets/chassis.usda` | The bay and the sprung retainer flap: one fixed articulation with a revolute joint Retainer (OpenUSD) |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; boxes, capsules, spheres) |
| `test_robot_vacuum_brushroll.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Roll** (one free rigid body, 0.10 kg): body 28 x 200 mm, round stub 11 mm x 18 mm at -y, square stub 8 x 8 x 18 mm at +y.
- **Bay** (one fixed articulation): base plate 90 x 244 mm, two end walls (14 mm thick, 50 mm high) with U-slots (bearing slot 12 mm wide, drive socket 9.5 mm wide, 45 degree V lead-ins, floors 20.5 and 22 mm up), an ear outside the +y wall and the **Retainer** flap (34 x 30 x 3 mm, revolute about x, 0 to 75 degrees open, spring 0.01 N m/rad toward open).

## Layout

The bay's middle is at x 0.34, y -0.25; the roll starts at x 0.225, y -0.25 with its axis along y. The seated roll's axis is 26 mm above the table. The retainer's hinge is at y -0.124, outside the +y wall. The right arm does the whole demo.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=robot_vacuum_brushroll.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/robot-vacuum-brushroll-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=robot_vacuum_brushroll.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 650 --store-frames --log-dir ~/yam-eval/robot-vacuum-brushroll-demo
```

The log prints `[brushroll] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm pinches the roll's body across its diameter (closing along x, pad tips 4 mm below the axis), lifts it 60 mm, carries it 115 mm over the bay, lowers it until the stubs are in their slots, lets go, lifts away, closes its fingers, turns the hand a quarter turn, goes up and behind the open flap, and pushes it down along its arc with the closed fingers until the latch catches. Success at about step 530 (26.5 s).

## Scoring

| Step | Done when |
|---|---|
| 1 | the roll was lifted 15 mm at some time |
| 2 | both stubs seated in their slots (within 3, 4 and 2 mm; axis within 4 degrees, square stub at +y, flats within 12 degrees) |
| 3 | the retainer is down within 4 degrees and latched, with the roll seated |
| 4 | with step 3, no gripper pad within 3 cm of the roll and none holds it |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success():
  all steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the roll lost off the table, the retainer latched with the roll not seated); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: a roll resting on the funnels or turned end for end is not seated (the 11 mm stub does not enter the 9.5 mm socket); a retainer pushed shut over an empty bay is a rule event (tests).
- **Real station:** judged from video: the roll lying in both slots, the retainer flap down over the drive end.

## Reset

Roll lying on the table with its axis along y, middle at x 0.225, y -0.25, square stub toward +y; retainer open at 75 degrees, latch off.

## Status

Dream-1 score: 0.0 (2400 steps, hit the step limit). Demo: 1.0 at step 530 (26.5 s). Media M-DCNR86.
