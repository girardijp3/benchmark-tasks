# Press a 120 mm case fan down into its tool-less bracket until both latches snap over it

A one-arm PC and server hardware task. Case fans sit in tool-less brackets: corner guides line the fan up and spring
latches snap over its upper face once it is pressed onto the ring. Here a 147 mm bracket with four corner posts and two
spring latches (hinged arms with ramped hooks) is fixed on the table, and a 120 x 120 x 25 mm case fan (130 g) lies beside
it.

The fan has to be lowered square into the posts (3.5 mm a side) and pressed down: its lower edge pushes the two latches
open as it comes down and they snap shut over its upper face when its lower face reaches the ring (the hooks sit 0.6 mm
above the fan's upper face when it is seated). A fan left on the ramps is not held.

- The left hand lifts the fan by one side bar, carries it over the bracket, lowers it into the posts, presses it home until
  the latches snap over it, and lets go.
- The fan must stay seated and still.

| File | What it is |
|---|---|
| `server_fan_snap.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`), the latch springs' reset event and the scripted `demo()` |
| `assets/fan_bracket.usda` | The bracket (OpenUSD): one articulation, a fixed base and two spring latches |
| `assets/case_fan.usda` | The 120 mm case fan, one rigid body |
| `build_assets.py` | Writes the two USD files with pxr (boxes and small convex prisms) |
| `test_server_fan_snap.py` | Offline checks: the file loads, every scoring outcome, the 1 s timing, the arm's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All assets were created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Bracket** (`fan_bracket.usda`, articulation with a fixed base): a 147 mm square ring 6 mm high (the fan's lower face rests on
  it), four L-shaped corner posts (127 mm opening, 46 mm tall, 8 mm walls with 6 mm bevels) and two latch links on revolute
  hinges along x at the ring's level, 65.5 mm either side of the fan's axis: 34 mm arms (20 mm wide) with 7.5 mm hooks whose
  tops are 54 degree ramps and whose undersides sit 0.6 mm above a seated fan's upper face. Each hinge is a spring drive
  (0.09 N m/rad, damping 0.003 N m s/rad) preloaded 2 degrees against its inner stop and limited to 20 degrees open.
- **Fan** (`case_fan.usda`, 130 g): a 120 x 120 x 25 mm frame of four 8 mm bars (a 104 mm opening; typical 120 mm case fan), hub
  and blades painted on (no collision).
- **Physics:** colliders are boxes and small convex prisms; friction values are in the notes above; restitution 0. Plastic friction 0.35/0.3 (fan) and 0.3/0.25 (hooks); 0.45/0.35 for the bracket body.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active. The instruction is `task.json`'s, word for
word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 800 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=server_fan_snap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/server-fan-snap-in-dream

# Scripted demo, 430 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=server_fan_snap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 430 --store-frames --log-dir ~/yam-eval/server-fan-snap-in-demo
```

The job log prints `[server_fan_snap] t=...  steps done: ...` when a step is done or undone, and a trace every second.

**The demo** (Isaac: success at step 335, 16.8 s). Left arm, open loop: comes down on the fan's +x side bar from above, fingers closing along x across its 8 mm width (pad tips 14 mm down the bar, 20 N a finger); lifts the fan 56 mm, carries it over the bracket, lowers it to 50 mm over the table, then presses it down 44 mm through the posts, the latch ramps and the snap (about 3.5 s), holds, lets go, lifts off and goes home.

## Instruction

It says what to achieve (the fan seated in the bracket with both latches snapped over it) and to let go, and leaves out which hand, where things are and how to move.

## Scoring

Four steps, **0.2 each**; step 1 once reached, steps 2 to 4 as things stand, each only with the one before.

| Step | Done when |
|---|---|
| 1. Fan lifted off the table | a gripper has held it (pads through the fan's box, both fingers pressing) at least 2 cm above where it lay |
| 2. Lowered into the corner posts | with step 1, lower face at most 20 mm above the table, axis within 3.5 mm of the bracket's, within 3 deg of square and 4 deg of level |
| 3. Pressed down, latches over it | with step 2, lower face within 0.8 mm of the ring, centred within 3 mm, within 2 deg of square and 1.5 deg of level, both latches open at most 2.5 deg |
| 4. Fan let go | with step 3, no gripper holds it or has its pads within 3 cm of it |

- **1.0 (success):** every full-success condition held together, without a break, for 1 s: all four steps, no violation, and
  the part within 2 mm and 2 degrees of where it was when that second began. The second is timed inside the task's success() and any lapse restarts it. The step points stop at 0.8, also while the second is being timed. The
  tests check no 1.0 at 0.8 s, 1.0 at 1.1 s, a lapse restarting it, and no 1.0 while any one condition is missing. The arm's
  return is not scored.
- **0** if the fan falls off the table or over (more than 45 degrees), the bracket is moved more than 5 mm, or the state is not finite.
- **Guards:** a fan standing on the latch ramps or held a few millimetres up has a lower face above the ring and does not count; a fan put on the ring without a gripper lifting it scores nothing; a fan held in place but not let go scores 0.6 at most.
- **Real station:** the fan is checked by a light tug on its frame (it stays) and by seeing both hooks over its upper face.

## Reset

Lift the fan out of the bracket (open both latches) and lay it flat on the table where it started.

## Status

Dream-1 score: 0.0 (800 steps, hit the step limit). Demo: 1.0 at step 335 (16.8 s). Media M-JBUHQA.
