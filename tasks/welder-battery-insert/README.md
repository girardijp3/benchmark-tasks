# Put an 18650 cell from a battery holder into a handheld spot welder the right way round and latch its lid closed

A bimanual battery-insertion task. A small battery spot welder lies on the table, empty, its battery lid sprung open;
next to it stands the bottom half of a 2x2 battery holder with four 18650 cells, positive ends up.
- One hand picks the welder up, turns it and sets it down.
- The other takes one cell out of the holder, turns it to lie flat, lays it in the welder's bay positive end toward
  the probes, and closes the lid until the latch catches. Both arms go back.

The cells stand 2 mm apart in their cups, so a gripper reaches one only from the holder's corner, across the cell's
two free sides. The bay leaves 0.5 mm a side, and the lid stays shut only once its hook has ridden over the latch's
ramp.

| File | What it is |
|---|---|
| `welder_insert.py` | The `yam_arena` task file: scene, the open-lid start, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/spot_welder_18650.usda` | The spot welder (OpenUSD): housing, battery lid and latch, sprung + contact, USB-C receptacle with its two spring fingers |
| `assets/welder_task.usda` | The task's wrapper: the welder as one articulation, self-collisions on (the latch holds the lid by contact) |
| `assets/holder_18650_2x2.usda` | The bottom half of a 2x2 18650 cell holder, one rigid body |
| `assets/battery_18650_yellow.usda` | The 18650 cell, yellow wrap, with flat-ended prism colliders |
| `assets/textures/` | Plastic grain maps and the cell's wrap label (no brand) |
| `test_welder_insert.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Spot welder, holder and cell** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Spot welder:**
  - a mini 18650 spot welder with a 124 x 24 x 22 mm black ABS housing, 138.7 mm long with its copper probes, 80 g
    empty;
  - the battery lid covers the top from 7 to 80 mm along it, hinged at the rear end on a torsion spring that opens
    it to 105°;
  - in front of the lid, a latch slider (on a 200 N/m return spring) catches a hook under the lid's front edge. The
    slider's tongue has a 45° ramp on top: pressing the lid shut makes the hook push the slider aside and snap under
    it;
  - the bay is 19 mm wide, with a sprung + contact at its probe end.
- **Holder:** the bottom half of a 2x2 18650 holder in black ABS, 41.2 x 41.2 x 9.5 mm, 6.5 g: 20 mm pitch, r9.30
  bores (0.30 mm radial clearance on a cell) with a 0.8 mm lead-in, 8 mm cups on a 1.5 mm ledge. It collides as a
  signed distance field.
- **Cell:** an 18 x 65 mm 18650 lithium-ion cell, 47 g, yellow plastic shrink-wrap.
  - It collides as two plain prisms with flat ends: a 30-sided body (60 points, within 0.05 mm of round) and a
    16-sided positive nub.
  - A convex hull of the wrap mesh (about 380 points) is cut down by PhysX to its hull limit; its end face came out
    uneven, and standing cells kept rocking 2–3° in their cups.
  - PhysX's cylinder shape doesn't collide with signed-distance-field meshes, so it can't stand in the holder.

The welder's file has PhysX fixes for its USB-C port (polygons triangulated by ear clipping, exact box and capsule
colliders); this task doesn't use the port.

**The open lid at reset.** Isaac Lab sets every joint drive's target to 0 when it builds the scene, and starts an
articulation's joints at 0 (lid shut). A reset event in the task file opens the lid fully, puts the latch home and
sets their springs' targets back to the USD's: the lid winds toward fully open, the latch presses back with a 0.2 N
preload.

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=welder_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/insert-dream

# Scripted demo (both arms), 900 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=welder_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/insert-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With `-E object_jitter=0`
  the layout is fixed and no seed is needed. The job log prints a `[welder_insert] t=...  steps done: ...` line
  whenever a step is done or undone, with how far the nearest cell is from its seat, the lid's angle and how far the
  welder has turned.
- **The demo.** It reaches 1.0 at step 749 (37.5 s): battery taken out at 6.7 s, in the bay at 19.1 s, lid latched
  at 31.5 s, welder put down (left hand off) at 33.2 s, both arms home by 37.5 s. A trial takes about 5 minutes on an L40S, including startup.
  - The left arm grips the welder from above across its front end, its wrist leaned 50° toward the probes, lifts it
    3 cm, turns it 45° (probes toward the robot's left and back, where the right wrist lays the cell in with the most
    margin) and holds it down on the table.
  - The right arm comes at the holder's corner cell along the diagonal, pointing sideways 15° below level, its
    fingers closing across the cell's two free sides with their tips 3 mm past the cell's axis, short of its two
    neighbours. It lifts the cell out and waits while the welder is turned.
  - It carries the cell high over the open lid, turning it flat on the way (a quarter turn about the closing axis:
    the cell lies along the gripper's old pointing direction, positive end forward), then lowers it straight down
    and lets go with the pad tips just above the bay's rim; the cell drops into its seat.
  - The fingertips, closed, push the open lid over and down from behind near its free end (the lid's spring keeps it
    against them), then press it 2 mm past shut so the hook snaps under the latch. The right arm goes home, then the
    left lets go and goes home.
- **Videos:** `inspect-robots video ~/yam-eval/insert-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_welder_insert.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order: the left picks the welder up, turns it and sets it down, the right takes
a cell, lays it in the right way round and latches the lid, and both arms go back. It leaves out where things are, which the model finds from
the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Battery taken out | a gripper holds a cell lifted clear of the holder (its lowest point above the holder's top) | once reached |
| 2. Battery in the bay | a cell lies in the welder's bay within 3 mm of its seat, positive end toward the probes within 10° | at the end |
| 3. Lid latched | with step 2 done, the lid is shut (within 2°) and the latch has caught it (its slider back home) | at the end |
| 4. Welder turned and put down | the welder stands on its feet on the table (its bottom within 3 mm of it), upright within 10°, turned at least 30° from its start, and no gripper holds it | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and the welder and the cell in it still (under 1 cm/s and 0.5 rad/s) for 1 s. The cells left in the holder aren't
  part of it: PhysX keeps a sub-millimetre contact jitter on cells standing in a 0.3 mm clearance.
- **0** at the end, whatever the steps, if the welder or any cell has fallen off the table (centre more than 2 cm
  below its top), or the state is not finite.
- **The right way round:** a cell lying in the bay backwards (positive end toward the rear) never counts as in the
  bay, so the lid over it doesn't count either.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video, with a ruler for heights and a protractor for the
  welder's turn.

## Reset

- Take the cell out of the welder and stand it back in its cup in the holder, positive end up.
- Open the lid fully.
- Set the welder on its feet at its mark, probes toward the robot's left, and put the holder back on its mark.
