# Open a handheld spot welder's battery lid and tip its 18650 cell into a box

A bimanual battery-removal task. A small battery spot welder lies on the table with its 18650 cell inside, under a
latched lid; an open cardboard box stands in front of the robot.
- One hand holds the welder down while the other pushes the latch's small knob until the lid springs open.
- Then the first hand lifts the welder, turns it over above the box so the cell falls in, and puts the welder back on
  the table. Both arms go back.

The cell sits flush in its bay, 1.8 mm below the rim and 0.5 mm from each side, so no gripper can reach around it: it
has to be poured out. The latch knob stands only 1.4 mm above the lid and lets go after 1.3–1.8 mm of travel.

| File | What it is |
|---|---|
| `welder_battery.py` | The `yam_arena` task file: scene, the lid's and latch's springs, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/spot_welder_18650.usda` | The spot welder (OpenUSD): housing, battery lid and latch, sprung + contact, USB-C receptacle with its two spring fingers |
| `assets/battery_18650.usda` | The 18650 cell (the task spawns it in the bay, under the closed lid) |
| `assets/welder_task.usda` | The task's wrapper: the welder as one articulation, self-collisions on (the latch holds the lid by contact) |
| `assets/textures/` | Plastic grain maps and the cell's wrap label (no brand) |
| `test_welder_battery.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Spot welder and cell** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Spot welder:**
  - a mini 18650 spot welder with a 124 x 24 x 22 mm black ABS housing, 138.7 mm long with its copper probes, 126.7 g
    with its cell;
  - the battery lid covers the top from 7 to 80 mm along it, hinged at the rear end on a torsion spring that
    opens it to 105°;
  - in front of the lid, a latch slider's knob (4.4 x 10 mm, 1.4 mm above the lid) holds it shut: a tongue on the
    slider catches a hook under the lid's front edge. Pushing the knob toward the probes (about 0.5 N, against a
    200 N/m return spring) frees the hook after 1.3–1.8 mm of its 3 mm travel.
- **Cell:** an 18 x 65 mm 18650 lithium-ion cell, 47 g, in plastic shrink-wrap, lying in a 19 mm bay.

The welder's file has PhysX fixes for its USB-C port (polygons triangulated by ear clipping, exact box and capsule
colliders); this task doesn't use the port.

**The springs at reset.** Isaac Lab sets every joint drive's target to 0 when it builds the scene, which would leave
the lid's spring holding the lid shut. A reset event in the task file closes the lid and the latch and sets their
springs' targets back to the USD's: the lid winds toward fully open, the latch presses back with a 0.2 N preload.

**Box:** NVIDIA SimReady [`box_cardboard_open_a06`](https://huggingface.co/datasets/nvidia/simready-assets/tree/2464ca32165bfff81ba34bc703ddb932822cfd27/box_cardboard_open_a06)
(`simready_usd/sm_box_cardboard_open_a06_01.usd`) from the Hugging Face dataset `nvidia/simready-assets`, pinned to
revision `2464ca32165bfff81ba34bc703ddb932822cfd27`, © NVIDIA, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/).
It is scaled to a third of its 61.1 x 50.5 x 30.2 cm and fixed in place. Measured at that scale in the scene: 20.4 cm
long (x) and 16.8 cm wide (y), its wall on the robot's left 10.1 cm tall and the one on the right 4.9 cm, the end
walls sloping between them. Its files (about 50 MB, mostly textures) are not committed: the task file downloads them
into `~/.cache/simready` the first time the scene is built, which needs network access, as Arena's own library assets
do.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=welder_battery.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/battery-dream

# Scripted demo (both arms), 800 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=welder_battery.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/battery-demo
```

- **Score.** The run prints `task_score` at the end: a third per step done, 1.0 on success. With
  `-E object_jitter=0` the layout is fixed and no seed is needed. The job log prints a
  `[welder_battery] t=...  steps done: ...` line whenever a step is done or undone, with the lid's angle and where the
  cell and the welder are.
- **The demo.** It reaches 1.0 at step 669 (33.4 s): lid opened at 9.8 s, cell in the box at 18.8 s, welder put
  back at 28.7 s, both arms home by 33.4 s. A trial takes about 4 minutes on an L40S, including startup.
  - The left arm grips the welder from above across its front end, clear of the lid's swing, its wrist leaned 50°
    toward the probes, and holds it down on the table.
  - The right arm, fingers closed and upright, comes down on the lid just behind the knob, aimed 1.5 mm into it (the
    arm gives), and slides toward the probes until its pads are 6 mm past the knob's rear face. Without that press
    the pads' rounded ends ride up over the 1.4 mm knob.
    It lifts away, the lid springs open, and the arm goes home.
  - The left arm lifts the welder, carries it over the box turned end for end and rolled 110° about its long axis (as
    far as the wrist goes), so the open bay faces 20° below level and the cell rolls out into the box.
  - It turns the welder back upright, carries it back high over the box's wall, sets it down where it started, lets
    go and goes home.
- **Videos:** `inspect-robots video ~/yam-eval/battery-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_welder_battery.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order, including how the task ends: the cell in the box, the welder back on the
table and both arms back. It leaves out where things are, which the model finds from the cameras.

## Scoring

Three steps, **a third each** (`STEPS` in the task file), judged as things stand at the end.

| Step | Done when |
|---|---|
| 1. Lid opened | a gripper pushed the latch knob, and the lid is open more than 60° |
| 2. Cell in the box | the cell lies in the box: its centre at least 1 cm inside the box's outer walls and less than 3 cm above the table |
| 3. Welder put back | after it has been lifted, the welder stands on its feet on the table (its bottom within 3 mm of it), upright within 10°, not in the box, and no gripper holds it |

- **1.0 (success, ends the run):** all three steps, both arms back where they started (within 3 cm) holding nothing,
  and the welder and the cell still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps, if the welder or the cell has fallen off the table (centre more than 2 cm below
  its top), or the state is not finite.
- **Pushed by a gripper:** the latch counts as pushed by hand when it moved at least 1 mm while a gripper's pads were
  within 15 mm of the knob's centre. The lid can't open any other way short of breaking it, so this rules out only a
  pried or knocked-open lid.
- **Lifted first:** the welder stands on the table at the start, so step 3 counts only once a gripper has held it with
  its lowest point at least 2 cm above the table.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video, with a ruler for heights.

## Reset

- Put the cell back in the welder's bay and close the lid until the latch clicks.
- Stand the welder on its feet at its mark, rear face toward the robot's right.
- The box stays taped in place.
