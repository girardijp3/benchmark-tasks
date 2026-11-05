# Unplug a spot welder's USB-C charging cable and put the welder in its holder

A bimanual put-away task. A small battery spot welder lies on the table with a power bank's USB-C cable plugged into
its rear port.
- One hand holds the welder down while the other takes the plug by its plastic body, pulls it straight out and lays
  it on the table.
- Then the first hand puts the welder into its 3D-printed holder, screwed to the table, and both arms go back.

The port has real USB-C clearances (0.10–0.23 mm a side), so the plug has to come out straight. The holder's pocket
leaves 2.5 mm of play at each side and 4 mm at each end.

| File | What it is |
|---|---|
| `welder_unplug.py` | The `yam_arena` task file: scene, the plugged-in start, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/spot_welder_18650.usda` | The spot welder (OpenUSD): housing, battery lid and latch, sprung + contact, USB-C receptacle with its two spring fingers |
| `assets/battery_18650.usda` | The 18650 cell inside the welder (the task spawns it in the bay, under the closed lid) |
| `assets/powerbank_usbc.usda` | The power bank with its captive 300 mm USB-C cable and plug, one articulation |
| `assets/spot_welder_holder.usda` | The welder's holder: a PLA tray, one rigid body |
| `assets/welder_task.usda`, `assets/powerbank_task.usda` | The task's wrappers: articulation roots, the cable's joint armature, the fingers' springy contact |
| `assets/textures/` | Plastic grain and PLA layer-line maps, and the cell's wrap label (no brand) |
| `test_welder_unplug.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Spot welder, power bank and holder** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Spot welder:**
  - a mini 18650 spot welder with a 124 x 24 x 22 mm black ABS housing, 138.7 mm long with its copper probes, 126.7 g
    with its cell;
  - the cell is held under a hinged lid by a sprung latch;
  - the USB-C receptacle is in the middle of the rear face, 11 mm above the housing's bottom. Its shell is 8.54 x
    2.86 mm inside and 6.35 mm deep, with two spring fingers that press on the plug.
- **Power bank:**
  - a 92 x 46 x 14 mm grey power bank, 125 g;
  - a 300 mm, 3.5 mm-thick cable made of 24 capsule links on bending joints;
  - a 4 g USB-C plug: a 12 x 6.2 x 22 mm plastic body and an 8.25 x 2.40 mm, 6.6 mm-long metal shell.
- **Holder:**
  - a 142 x 39 x 14 mm orange PLA tray, 20.7 g, whose pocket follows the welder's outline;
  - its long sides are cut open over 80 mm so a gripper can reach the welder's sides, and slots at both ends clear
    the probes and a plug;
  - it collides as 33 convex pieces.

The task's copies differ from the originals, all for PhysX:

1. **Triangulated faces.** Every polygon with more than four sides is split into triangles by ear clipping. Isaac Sim
   splits polygons from one corner, which laid triangles across the housing's USB-C opening and closed it.
2. **Exact colliders for the fit.** PhysX cooks convex colliders of millimetre-sized parts up to 0.3 mm off, and
   inflates any thinner than about 1 mm; the fit is 0.1 mm. So the fit uses PhysX's exact primitives:
   - each steel piece of the receptacle is a box, 1.2 mm thick and grown outward;
   - the back plate is a box, and each spring finger is a rounded bar;
   - the plug's shell is five round-nosed capsules side by side;
   - the receptacle's tongue doesn't collide.
3. **Springy contact on the fingers.** PhysX lifts the 0.5 g sliding fingers with no more than about 0.4 N, so the
   wrapper holds the slides still and gives the fingers' bars a springy (compliant) contact: 5000 N/m, friction 0.6.
4. **Plug shell kept off the housing.** The housing collides as a signed distance field, which isn't exact to 0.1 mm
   inside the opening. The task file keeps the plug's metal shell clear of it; the plug's plastic body still meets
   the housing's rear face.

**The plugged-in start.** The cable starts lying on the table and rises over its last 5 cm into the port:
- it drops 13.3° from the power bank's outlet onto the table, then rises 11.4° and levels the plug at the port's
  height;
- the task file lays the links out in that shape when the power bank spawns;
- a reset event writes the same joint angles into the simulator at each episode's start (Arena resets objects'
  poses, not their joint angles);
- the cable's bending springs rest in that shape, like a cable that has taken a set, so the plug starts fully in
  and stays there until pulled.

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=welder_unplug.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/unplug-dream

# Scripted demo (both arms), 1,200 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=welder_unplug.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/unplug-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With
  `-E object_jitter=0` the layout is fixed and no seed is needed. The job log prints a
  `[welder_unplug] t=...  steps done: ...` line whenever a step is done or undone, with how far the plug is from fully
  in and how far the welder is from its seat.
- **The demo.** It reaches 1.0 at step 554 (27.7 s): unplugged at 8.1 s, plug laid on the table at 11.6 s,
  welder lifted at 17.6 s, welder in its holder at 23.2 s, both arms home by 27.7 s. A trial takes about 3.5 minutes
  on an L40S, including startup.
  - The left arm grips the welder from above across its middle, its wrist leaned 50° away from the plug hand, and
    holds it on the table.
  - The right arm grips the plug's plastic body from above right behind the welder, pulls it 3 cm straight out, lays
    it on the table and goes home.
  - The left arm then lifts the welder, carries it over the holder turning it 90°, and lets it drop in from 1.5 mm
    above the walls. It lets go and goes home.
- **Videos:** `inspect-robots video ~/yam-eval/unplug-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_welder_unplug.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order, including how the task ends: the plug laid down, the welder in its
holder with its probes toward the robot, and both arms back. It leaves out where things are, which the model finds
from the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Unplugged by hand | the plug is out of the welder's port (its plastic body more than 5 mm from the welder's rear face, or more than 2 mm off the port's centre line, or turned more than 30° from the port's direction), and a gripper held the plug as it came out | at the end |
| 2. Plug put down | with step 1 done, the plug lies on the table (its lowest point within 3 mm of it) and no gripper holds it | at the end |
| 3. Welder lifted | a gripper holds the welder with its lowest point at least 2 cm above the table | once reached |
| 4. Welder in its holder | the welder sits in the holder's pocket (within 5 mm of its seat, on the pocket floor within 2 mm, upright within 10°) and no gripper holds it | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and the welder and its cell still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps, if:
  - the welder, its cell or the plug has fallen off the table (centre more than 2 cm below its top);
  - the cell has come out of its bay (more than 1 cm from its place, so the lid was opened);
  - the state is not finite.
- **Yanked out:** a plug pulled out by its cable, because the welder was dragged or lifted away while no gripper held
  the plug, never earns steps 1 and 2 in that run, whatever happens to the plug afterwards. Unplugging means pulling
  the plug, not the device.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video, with a ruler for heights and gaps.

## Reset

- Close the welder's lid over its cell if it was opened.
- Plug the cable fully into the welder's rear port, and set the welder on its feet at its mark, rear face toward the
  power bank.
- Lay the cable on the table back to the power bank.
- Leave the holder empty.
