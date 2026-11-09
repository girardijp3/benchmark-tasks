# Switch on a bench power supply and set it to 9 V

A bimanual lab-instrument task. A bench DC power supply (0-30 V / 0-5 A) stands on the table, front panel toward
the robot, switched off, its voltage knob at 12 V.
- One hand presses the latching power button; the voltmeter lights up and reads 12.00.
- The other grips the voltage knob, turns it 27° counter-clockwise (seen from the front) to 9 V, lets go, and both
  arms go back.

The button is 28 mm above the table, so a level gripper would hit the table with its body: the finger tips push it
pointing 25° down. The knob is gripped from the front, between the fingers, and turned by rolling the wrist; the
voltage must end within 0.2 V (1.8° of knob).

| File | What it is |
|---|---|
| `psu_voltage.py` | The `yam_arena` task file: scene, the button's latch and the meters (a step event), step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/bench_psu.usda` | The supply (OpenUSD): one articulation, its case, the push button on a sprung slide and four damped knobs, with the meters' segments |
| `assets/textures/` | The front and rear panel prints (no brand) |
| `test_psu_voltage.py` | Offline checks: the file loads, every scoring outcome, the latch, the voltmeter's digits, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Bench power supply** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up; origin at the
footprint's middle on the table, front panel facing -Y.

- **Size and mass:** the common format of 30 V / 5 A single-output bench supplies (a widespread unit's manual gives
  110 x 156 x 260 mm, 4.1-4.5 kg for its linear version). This one is a switch-mode build in that case: 110 mm wide,
  151 mm tall (158 mm on its four 7 mm rubber feet), 262 mm deep with its front and rear frames, 2.5 kg. Painted
  steel cover with ventilation slots, black ABS frames, a printed front panel.
- **Front panel:**
  - a red voltmeter and a green ammeter, four 7-segment digits each (12 mm tall), and CV / CC lamps;
  - VOLTAGE and CURRENT coarse knobs: 26 mm fluted soft-touch knobs on a 30 mm skirt, 18 mm proud, over printed
    scales (0-30 V every 1 V and 0-5 A every 0.25 A, 270° from end to end, clockwise up), and two 18 mm FINE knobs
    below them (±0.5 V on the voltage side);
  - a latching POWER push button (14 mm square cap, 8 mm proud, 4.5 mm travel) and red, green and black binding posts.
- **Joints:** the button slides on a 400 N/m return spring (0.8 N preload). The knobs turn on damped hinges
  (0.05 N·m·s/rad on the coarse knobs), no spring, so they stay where they are turned; the voltage knob's hinge is
  0 at 12 V, 9° per volt.
- **Colliders:** boxes for the case, the front frame's lip, the feet and the binding posts; 32-sided prisms (64
  points) for the knobs; a box for the button's cap. Frictions: rubber feet 0.9 / 0.8, soft-touch knobs 0.8 / 0.7,
  case 0.4 / 0.35.

**Run-time parts in the task file:**
- **The latch.** A real push-on/push-off switch holds its button with a small cam. Here a step event (every control
  step) follows the button's travel: past its 3.5 mm click the supply switches on and the spring's target moves to
  2.5 mm in, where the button then stays; the next push past the click switches it off and the button springs back
  out. Arena gives objects no actuators, so Isaac Lab's joint targets never reach the simulator: the event sets the
  drive's target in PhysX directly.
- **The meters.** Each segment is its own prim, authored invisible; the same event shows the knobs' voltage on the
  voltmeter (and 0.000 A, nothing being connected) while the supply is on, by switching the segments' visibility in
  the USD stage. The cameras show it. The printed scale and the knob's pointer line read the setting too.
- **Reset:** a reset event puts the button out and every knob at its start (the simulator keeps an articulation's
  joints where they were) and switches the supply off.

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=psu_voltage.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/psu-dream

# Scripted demo (both arms), 500 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=psu_voltage.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval/psu-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With `-E object_jitter=0`
  the layout is fixed and no seed is needed. The job log prints a `[psu_voltage] t=...  steps done: ...` line
  whenever a step is done or undone, with the knob's voltage, the button's travel and how far the supply has moved,
  a line when the power switches, and a trace line every second.
- **The demo.** It reaches 1.0 at step 370 (18.5 s): power on at 5.0 s, voltage knob gripped at 11.0 s, set to
  9.00 V (all four steps) at 14.6 s, both arms home by 18.5 s. A trial takes about 4 minutes on an L40S, including
  startup.
  - The left arm comes at the button with its fingers closed, pointing at the panel 25° down (level, its body would
    reach the table), the fingertips' upper edge on the middle of the button. It pushes past the click, holds
    0.5 s, lets the button back to the latch (2.5 mm in) and goes home.
  - The right arm waits until the left fingers are off the button, then points at the panel level with the
    voltage knob's axis, fingers open 40 mm, and slides in until the pad tips are 1.5 mm short of the knob's skirt.
    It closes on the knob (about 20 N a finger), rolls the wrist 26.2° counter-clockwise seen from the front (the
    knob runs on about 0.8° as the wrist stops: 27°, 12 V to 9 V), opens, backs off and goes home.
- **Videos:** `inspect-robots video ~/yam-eval/psu-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_psu_voltage.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order: the left switches the supply on with its power button, then the right
turns the voltage knob from 12 V to 9 V and lets go, and both arms go back. It names the start and the target
voltage, which a technician would be told, and leaves out where the controls are and which way to turn, which the
model reads from the panel (its printed scale) through the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Power on | the supply is on, its button held in by the latch, and a gripper's finger tips were at the button when it switched on | at the end |
| 2. Voltage knob gripped | a gripper held the voltage knob (its pads around the knob, both fingers pressing) | once reached |
| 3. Set to 9 V | the voltage knob reads 9.0 V within 0.2 V (with the fine knob's offset), and no gripper holds it | at the end |
| 4. Nothing else changed | with steps 1 and 3, the current knob and both fine knobs within 3° of where they started, and the supply within 1 cm and 5° of its start | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and the supply still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps, if the supply has fallen off the table (more than 2 cm below its top),
  been knocked over (tilted more than 30°), or the state is not finite.
- **Switched off again:** pressing the button a second time switches the supply off, and step 1 no longer holds.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the knob's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video: the power lamp and the voltmeter, the knob's
  pointer on its scale, and a mark on the table under the supply.

## Reset

- Switch the supply off: press the power button so it comes back out.
- Turn the voltage knob back to 12 V; leave the current and fine knobs where they were.
- Put the supply back on its mark, front panel toward the robot.
