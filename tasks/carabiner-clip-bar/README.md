# Clip a spring-gate carabiner to a steel tie-down staple

A household and outdoor task. A spring-gate key-ring carabiner lies flat on a tie-down board with its gate end toward a steel staple (a U of 6 mm wire standing on the board). The robot has to clip the carabiner to the staple. The instruction states only the goal ("clip the carabiner to the steel staple on the board, so the two are linked and the carabiner cannot be lifted off"); that the gate has to be pushed in by the staple's leg, near its free end, and the carabiner moved square to the leg, are the robot's to work out.

The skill is picking up a 5 mm wire by its spine, moving the carabiner horizontally so a 6 mm leg meets the gate, pushing the spring gate open with the leg (the gate swings about 44 degrees), carrying on until the leg is inside the loop, and putting the carabiner down with the gate closed behind the leg.

| File | What it is |
|---|---|
| `carabiner_clip_bar.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/carabiner.usda` | The carabiner: a free body and a spring-hinged gate (OpenUSD); one articulation |
| `assets/anchor.usda` | The tie-down board with the staple (OpenUSD); fixed |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; boxes, capsules, spheres) |
| `test_carabiner_clip_bar.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Carabiner** (one articulation): a free body (spine, sides and nose: capsule wires, 5 mm wire, 44 x 49 mm, 20 g) and a gate (a 28 mm capsule, 4 g) on a revolute joint at the hinge end: spring 0.04 N m/rad, damping 0.0004 N m s/rad, range 0 (closed, the nose stop) to 60 degrees inward. Aluminium friction 0.35 / 0.30.
- **Board and staple** (fixed): a 200 x 340 x 8 mm board; two steel legs (6 mm, 22 mm tall above the board, 90 mm apart) and a 6 mm bar joining their tops, axis 30 mm above the table.

## Layout

The near leg's axis is at x 0.345, y -0.22; the carabiner's spine starts at x 0.255, y -0.22, flat on the board, loop and gate toward +x. Clipped, the spine lies at about x 0.323 with the leg inside the loop. The right arm does the whole demo.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=carabiner_clip_bar.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/carabiner-clip-bar-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=carabiner_clip_bar.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 450 --store-frames --log-dir ~/yam-eval/carabiner-clip-bar-demo
```

The log prints `[carabiner] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm pinches the spine across its wire (closing along x, 1.5 mm below its middle, about 20 N a finger), lifts the carabiner 12 mm, lines the gate up 6 mm toward the hinge side of the leg, pushes it forward 3 s so the leg opens the gate and passes through, waits for the gate to close, sets the carabiner down, lets go and goes home. Success at about step 332 (17 s).

## Scoring

| Step | Done when |
|---|---|
| 1 | the carabiner was lifted 8 mm off the board at some time |
| 2 | the near leg's axis passes through the carabiner's loop (inside the wires' centre line, below the bar) |
| 3 | clipped: step 2, gate closed (within 3 degrees), the carabiner lying on the board |
| 4 | with step 3, no gripper pad within 3 cm of the carabiner and none holds it |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success():
  all steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the carabiner is lost off the table); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: a carabiner dropped over the staple with its gate open is not clipped (the gate must be closed); one lying next to the staple, standing on edge or held up off the board is not clipped (all tests).
- **Real station:** judged from video: the carabiner lies on the board with the staple's near leg inside its closed gate.

## Reset

Lay the carabiner flat on the board with its spine at x 0.255, y -0.22, its loop and gate toward +x and its gate closed; nothing else is moved.

## Status

Dream-1 score: 0.0 (2400 steps, hit the step limit). Demo: 1.0 at step 332 (16.6 s). Media M-CKTDFJ.
