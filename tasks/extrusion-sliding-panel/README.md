# Slide a clear panel along the top and bottom slots of an aluminium frame to its end stop by its pull knob

A one-arm task on a sliding enclosure panel. Enclosures for robot cells, lab automation and compute racks use clear acrylic
panels that slide in the T-slots of a top and a bottom aluminium extrusion rail, with a pull knob on the face. Here a 100 x 80
x 3 mm panel (37 g) stands in both slots of a 220 mm frame of 20 x 20 mm 2020 profile, 60 mm short of the end stop at the
robot's left.

The slot opening is 6.2 mm (1.8 mm lips over an 11 x 6 mm cavity) and the panel is 3 mm, so it has 1.6 mm a side; its edges
are 4.5 mm into the top slot and 7.8 mm into the bottom one, so it is trapped between the rails and can only slide. The
knob is 20 mm in front of the panel's face: pulled with the hand skewed, the panel cocks in the slots and binds.

- The left hand takes the red knob between its fingers and slides the panel along the slots until its end meets the stop.
- The hand lets go and moves clear. The panel must stay at the stop, in the slots and still.

| File | What it is |
|---|---|
| `extrusion_sliding_panel.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/slide_frame.usda` | The frame: two rails with facing slots and two end posts (OpenUSD), a fixture |
| `assets/sliding_panel.usda` | The acrylic panel with its knob, one rigid body |
| `build_assets.py` | Writes the two USD files with pxr (boxes) |
| `test_extrusion_sliding_panel.py` | Offline checks: the file loads, every scoring outcome, the 1 s timing, the arm's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

Both created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres
and kilograms, Z up. Sizes: the common 2020 profile and typical 3 mm cast acrylic (1.19 g/cm3).

- **Frame** (`slide_frame.usda`, fixed): a bottom rail and a top rail, each 220 mm of 20 x 20 mm profile whose slot faces the
  other rail (opening 6.2 mm, 1.8 mm lips, 11 x 6 mm cavity, built from box walls; other faces' slots painted), the lips
  67.7 mm apart; a 20 x 20 mm post at each end (the one at +y, with a red face, is the stop); feet.
- **Panel** (`sliding_panel.usda`, 37 g): 100 x 80 x 3 mm clear acrylic (28 g) with a knob (a 12 mm stem and a 16 mm wide, 16 mm
  tall, 14 mm deep head, 9 g) on its front face, 55 mm above its lower edge. Friction on aluminium 0.4/0.3.
- **Physics:** colliders are boxes; friction aluminium 0.35/0.3, acrylic 0.4/0.3, plastic 0.45/0.35; restitution 0.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active. The instruction is `task.json`'s, word for
word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=extrusion_sliding_panel.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/slide-dream

# Scripted demo (left arm), 360 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=extrusion_sliding_panel.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 360 --store-frames --log-dir ~/yam-eval/slide-demo
```

The job log prints `[extrusion_sliding_panel] t=...  steps done: ...` when a step is done or undone, and a trace every second:
how far the panel has travelled and its end's distance from the stop, its plane's offset, its lower edge's height, its lean,
yaw and whether a gripper holds it.

**The demo** (Isaac: started 6.35 s, over half way 7.15 s, at the stop 9.4 s, let go 10.85 s, success step 238). Left arm, open loop: the gripper points ahead and 25 degrees down (pointing
straight down, its body would meet the top rail), comes up in front of the knob, closes its fingers across the knob head (along
y), pulls the panel 60 mm along the slots over 3.5 s until it meets the stop, lets go and backs off along its axis.

## Instruction

It says what to do (slide the panel along its slots until its end meets the stop at the left end) and to let go, and leaves
out which hand, the knob and how to move.

## Scoring

Four steps, **0.2 each**; step 1 once reached, steps 2 to 4 as things stand, each only with the one before.

| Step | Done when |
|---|---|
| 1. Panel started along the slots by a gripper | a gripper holding the panel or its knob (pads reach it, both fingers press) has pulled it 1 cm along the slots |
| 2. Panel pulled over half way | with step 1, in the slots (plane within 1.9 mm of the slots' axis, lower edge within 6 mm of the cavity floor, upright and square within 3 degrees) and at least 3 cm along |
| 3. Panel at the end stop | with step 2, its end within 2 mm of the stop |
| 4. Panel let go | with step 3, no gripper holds it or has its pads within 3 cm of it or its knob |

- **1.0 (success):** every full-success condition held together, without a break, for 1 s: all four steps, no violation, and
  the panel within 2 mm and 2 degrees of where it was when that second began. The second is timed in the task file
  (`SUCCESS_HOLD_S` is 0) and any lapse restarts it. The step points stop at 0.8, also while the second is being timed. The
  tests check no 1.0 at 0.8 s, 1.0 at 1 s, a lapse restarting it, and no 1.0 while a condition is missing. The arm's return is
  not scored.
- **0** if the panel falls off the table or tips over (more than 30 degrees), the frame is moved more than 5 mm, or the state is
  not finite.
- **Guards:** a panel moved without a gripper scores nothing; a panel left short of the stop (more than 2 mm) scores 0.4; a hand
  left on the knob does not get step 4; a panel cocked or leaning in the slots is not in them.
- **Real station:** a light push on the panel is checked (it stays at the stop).

## Reset

Slide the panel back along the slots so its centre is at y 0.0 (60 mm short of the stop).

## Status

Dream-1 score: 0.0 (800 steps, hit the step limit). Demo: 1.0 at step 238 (11.9 s). Media: M-DW49BM.
