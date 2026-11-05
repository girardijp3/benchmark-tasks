# Plug a power bank's USB-C cable into a handheld spot welder charging port

A bimanual connector task. One hand picks up a small battery spot welder and holds it with its rear USB-C port toward
the other hand. The other hand picks up the USB-C plug at the end of a power bank's cable and pushes it straight in.
The port has real USB-C clearances (0.10–0.23 mm a side), so the plug has to be lined up to a fraction of a
millimetre and held square. Then the plug hand lets go and goes back, and the welder is set back down with the cable
still plugged in.

| File | What it is |
|---|---|
| `welder_usbc.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/spot_welder_18650.usda` | The spot welder (OpenUSD): housing, battery lid and latch, sprung + contact, USB-C receptacle with its two spring fingers |
| `assets/battery_18650.usda` | The 18650 cell inside the welder (the task spawns it in the bay, under the closed lid) |
| `assets/powerbank_usbc.usda` | The power bank with its captive 300 mm USB-C cable and plug, one articulation |
| `assets/welder_task.usda`, `assets/powerbank_task.usda` | The task's wrappers: articulation roots, the cable's joint armature, the fingers' springy contact |
| `assets/textures/` | Plastic grain maps and the cell's wrap label (no brand) |
| `test_welder_usbc.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Spot welder and power bank** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Spot welder:**
  - a mini 18650 spot welder with a 124 x 24 x 22 mm black ABS housing, 138.7 mm long with its copper probes, 126.7 g
    with its cell;
  - the cell is held under a hinged lid by a sprung latch;
  - the USB-C receptacle is in the middle of the rear face, 11 mm above the housing's bottom. Its shell is 8.54 x
    2.86 mm inside and 6.35 mm deep, with a 0.4 mm lead-in lip and two spring fingers that press on the plug.
- **Power bank:**
  - a 92 x 46 x 14 mm grey power bank, 125 g;
  - a 300 mm, 3.5 mm-thick cable made of 24 capsule links on bending joints, with real stiffness;
  - a 4 g USB-C plug: a 12 x 6.2 x 22 mm plastic body and an 8.25 x 2.40 mm, 6.6 mm-long metal shell.

The task's copies differ from the originals in four ways, all for PhysX:

1. **Triangulated faces.** Every polygon with more than four sides is split into triangles by ear clipping. Isaac Sim
   splits polygons from one corner, which laid triangles across the housing's USB-C opening and closed it.
2. **Exact colliders for the fit.** PhysX cooks convex colliders of millimetre-sized parts up to 0.3 mm off, and
   inflates any thinner than about 1 mm; the fit is 0.1 mm. So the fit uses PhysX's exact primitives:
   - each steel piece of the receptacle is a box, 1.2 mm thick and grown outward, so its plug-side face stays put;
   - the back plate is a box, and each spring finger is a rounded bar;
   - the plug's shell is five round-nosed capsules side by side, tracing its 8.25 x 2.40 mm outline;
   - the receptacle's tongue doesn't collide.
3. **Springy contact on the fingers.** PhysX lifts these 0.5 g sliding fingers with no more than about 0.4 N, so with
   the models' 3000 N/m springs and 4 N preload the plug never got in. The task's wrapper holds the slides still and
   gives the fingers' bars a springy (compliant) contact instead: 5000 N/m, friction 0.6.
4. **Plug shell kept off the housing.** The housing collides as a signed distance field, which isn't exact to 0.1 mm
   inside the opening. The task file keeps the plug's metal shell clear of it, so the shell meets only the
   receptacle's steel, as in a real one; the plug's plastic body still seats on the housing's rear face.

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=welder_usbc.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/welder-dream

# Scripted demo (both arms), 1,200 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=welder_usbc.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/welder-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With
  `-E object_jitter=0` the layout is fixed and no seed is needed. The job log prints a
  `[welder_usbc] t=...  steps done: ...` line whenever a step is done or undone, with how far the plug is from fully
  in, how far off the port's line and how tilted.
- **The demo.** It reaches 1.0 at step 648 (32.4 s): plug lifted at 5.3 s, welder lifted at 6.1 s, plugged in at
  16.0 s, welder set down with the plug still in and both grippers open at 27.1 s, both arms home by 32.4 s. A
  trial takes about 3.5 minutes on an L40S, including startup.
  - The left arm grips the welder from above across its middle, its wrist leaned 50° away from the plug hand. It lifts
    the welder 8 cm and brings the port over, facing the right arm.
  - The right arm picks the plug up across its plastic body and lines it up 4 cm, then 8 mm, short of the port,
    settling at each.
  - The right arm pushes the plug in, lets go as it eases off, backs away along the port's line and goes home.
  - The left arm then sets the welder down, lets go and goes home.
- **Videos:** `inspect-robots video ~/yam-eval/welder-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_welder_usbc.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order, including how the task ends: let go of the plug, set the welder down and
both arms back. It leaves out where things are, which the model finds from the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Welder lifted | a gripper holds the welder with its lowest point at least 2 cm above the table | once reached |
| 2. Plug lifted | a gripper holds the plug with its lowest point at least 2 cm above the table | once reached |
| 3. Plugged in | the plug is in the welder's port: its plastic body within 1.5 mm of the welder's rear face, within 0.5 mm of the port's centre line, and tilted no more than 8° from the port's direction | at the end |
| 4. Set down, plug still in | with step 3 done, the welder rests on its feet on the table (upright within 10°), and neither gripper holds the welder or the plug | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and the welder and its cell still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps, if:
  - the welder, its cell or the plug has fallen off the table (centre more than 2 cm below its top);
  - the cell has come out of its bay (more than 1 cm from its place, so the lid was opened);
  - the state is not finite.
- **The 8° allowance:** a released plug sags a few degrees under the cable's weight inside the port's clearance (about
  6° in the demo). A plug pushed in crooked, or levered half out, tilts well past that.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video, with a ruler for heights and gaps, and a protractor
  on a side photo for the plug's tilt.

## Reset

Unplug the cable and lay it straight along the table from the power bank, plug flat at its mark, pointing at the
welder. Close the welder's lid over its cell if it was opened, and set the welder on its feet at its mark, rear face
toward the plug.
