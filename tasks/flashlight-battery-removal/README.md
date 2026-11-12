# Unscrew a flashlight's tail cap and drop its battery into a picking bin

A bimanual disassembly step: one hand holds a small 18650 flashlight while the other unscrews its tail cap and sets
it on the table, then the flashlight is turned tail-down so the lithium-ion cell slides out into a collection box,
and stood back on the table. The cap's 4-start
thread is modelled as real collision geometry, so it has to be turned (about one turn) to come off.

| File | What it is |
|---|---|
| `flashlight_battery.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/flashlight_18650.usdc` | The flashlight (OpenUSD): body, tail cap and battery as three rigid bodies, threads included |
| `assets/flashlight_{body,cap,battery}.usda` | One rigid body each, referencing the flashlight file, with the task's friction settings |
| `assets/textures/battery_18650_wrap.png` | The battery's wrap label (no brand) |
| `test_flashlight_battery.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Flashlight** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). 110 mm long, 34 mm head, 22 mm body tube, 26 x 21 mm tail cap, black
anodised aluminium; units metres and kilograms, Z up.

- Three rigid bodies: the body (70 g: tube, head, LED, reflector, lens, bezel and the male thread), the tail cap
  (18 g: cap, female thread, switch button) and an 18 x 65 mm 18650 cell (47 g).
- The tail-cap thread is 4-start trapezoidal, 2.5 mm pitch, 10 mm lead, right-hand: one turn counter-clockwise (seen
  from the tail) frees the cap, and a collar on the body stops it at the closed position. Thread, tube, head and cap
  collide as signed distance fields (SDF), so the threads really engage; the battery collides as its convex hull.
- The bore is 18.8 mm, the cell 18 mm: with the cap off and the tail down, the cell slides out.
- The part files set the friction: the cap's outer shell and the body tube grip like rubber (1.5 static, 1.3
  dynamic) and stand in for knurled aluminium and compliant finger pads; the threads keep aluminium's 0.35 / 0.30;
  the battery's shrink-wrap is 0.10 / 0.08 and takes the lower of the two frictions in any contact.

**Box:** NVIDIA SimReady [`box_cardboard_open_a06`](https://huggingface.co/datasets/nvidia/simready-assets/tree/2464ca32165bfff81ba34bc703ddb932822cfd27/box_cardboard_open_a06)
(`simready_usd/sm_box_cardboard_open_a06_01.usd`) from the Hugging Face dataset `nvidia/simready-assets`, pinned to
revision `2464ca32165bfff81ba34bc703ddb932822cfd27`, © NVIDIA, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/).
It is scaled to a third of its 61.1 x 50.5 x 30.2 cm and fixed in place. Measured at that scale in the scene: 20.4 cm
long (x) and 16.8 cm wide (y), its wall on the robot's left 10.1 cm tall and the one on the right 4.9 cm, the end
walls sloping between them. Its files (about 50 MB, mostly textures)
are not committed: the task file downloads them into `~/.cache/simready` the first time the scene is built, which
needs network access, as Arena's own library assets do.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flashlight_battery.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/flashlight-dream

# Scripted demo (both arms), 1,400 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flashlight_battery.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/flashlight-demo
```

- The run prints `task_score` at the end (0.18 per step done (at most 0.9), 1.0 only on success). With `-E object_jitter=0` the
  layout is fixed and no seed is needed.
- The demo reaches 1.0 and ends on success at step 953 (47.7 s at 20 Hz): the left arm grips the body from the
  side; the right arm unscrews the cap in two 200° grips, rising 5.5 mm per twist with the thread, and sets it on the
  table; the left arm carries the flashlight over the box, rolls it tail-down until the battery falls out, rolls it
  back upright, stands it head-down on its mark, opens and lifts straight off. The job log prints a
  `[flashlight_battery] t=...  steps done: ...` line whenever a step is done or undone; in the demo: body held at
  6.3 s, cap off 20.7 s, cap set down 24.6 s, battery in the box 34.5 s, flashlight set back 47.2 s. A trial takes
  about 4 minutes on an L40S, including startup.
- Videos: `inspect-robots video ~/yam-eval/flashlight-demo/<log>.json` writes one MP4 per camera next to the frames.
- Offline checks: `python -m pytest -q test_flashlight_battery.py`.
- Add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It gives the model what to do with each hand and in which order, and where the battery and the flashlight go. It
leaves out how to unscrew the cap and how to tip the battery out, and where things are: the model works those out
from the cameras.

## Scoring

Five steps, **0.18 each** (`STEPS` in the task file); only full success scores 1.0. Positions are centres in metres; heights are above the table
top. Assembled, the cap's centre is 4.7 cm from the body's and the battery's 1.1 cm.

| Step | Done when | Judged |
|---|---|---|
| 1. Body held | a gripper is closed on the flashlight body (not on the cap), both fingers touching it, for at least 1 s | once reached |
| 2. Cap off | the cap was unscrewed (turned at least 300° on the thread; one turn frees it) and its centre is more than 8 cm from the body's | once reached |
| 3. Cap set on the table | a gripper let go of the cap with its lowest point no more than 2 cm above the table and not over the box, and the cap rests on the table (lowest point within 1 cm of the top), not over the box, not held | at the end |
| 4. Battery in the box | the battery's centre is more than 9 cm from the body's, at least 1 cm inside the box's outer walls and under 6 cm high | at the end |
| 5. Flashlight set back on the table | after step 4, a gripper let go of the flashlight with its lowest point no more than 2 cm above the table and not over the box, and it rests on the table there (any orientation), not held | at the end |

- **"Set" means put down by a gripper.** A part that slips out of the gripper higher up, is dropped or knocked
  onto the table, or was let go over the box, earns nothing for steps 3 and 5. Picking it up again and setting it
  down properly counts.
- **1.0 (success, ends the run):** all five steps, and the three parts still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **All five steps without full success score 0.9:** the step points stop at 0.9, so a run that finishes
  the steps but doesn't meet the rest of the success conditions above never scores 1.0.
- **0** at the end, whatever the steps, if any part has fallen off the table (centre more than 2 cm below the top),
  the flashlight body lies in the box (centre over the box's footprint and below its 10.1 cm walls), or the state is
  not finite.
- **0** for the whole trial if the cap comes off without being unscrewed. Real threads don't pull apart; in
  simulation, two grippers pulling hard on the cap and body can force the thread, and that must not pay. The run
  log prints how far the cap had turned when it came off.
- Which arm does what is in the instruction, not the scoring: either gripper counts.
- In simulation, "a gripper holds a part" is read from the simulator: the finger pads reach the part (its bounding
  box, 5 mm slack; for the body, within the head's radius of its axis and clear of the cap), both fingers press with
  at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`, `_on_body`). The cap's turn on the
  thread is tracked from the two parts' orientations (`UNSCREW_TURN`). On a real station the same steps are judged
  from the video, with a ruler for the 2 cm set-down height.

## Reset

Put the battery back in the tube (positive end toward the head), screw the cap on until it seats against the collar,
and stand the flashlight on its bezel at its mark, cap up. The box stays taped in place.
