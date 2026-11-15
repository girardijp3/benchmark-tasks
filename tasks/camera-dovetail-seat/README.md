# Re-seat a robot head's camera module on its dovetail mount: set it on the plate and push it along the rail to the stop

A robot-maintenance task. A camera module has come off its sliding dovetail mount on a robot's head and lies on a parts
rack. The mount is a plate with a male dovetail rail, open at its end toward the robot's right and closed by a stop block
at the other. The robot has to pick the module up, set it on the plate in front of the open end with its slot over the
rail's axis, and push it along the rail until it meets the stop; the slot's lips then hold it on the rail's undercut. The
instruction names the result and nothing about how.

The skill is a guided horizontal push: align a held part to a rail within about 2.5 mm and 4 degrees, then push it
about 70 mm along the rail against friction to a hard stop without letting it climb the rail or jam. A module dropped on
top of the rail does not count (it rests on the rail's top, 6 mm too high).

| File | What it is |
|---|---|
| `camera_dovetail_seat.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/camera_module.usda` | The module (OpenUSD): two convex cheek prisms forming the slot's undercut walls, a roof; one rigid body |
| `assets/head_mount.usda` | The plate, the head below it, the dovetail rail and the stop block (fixed) |
| `assets/parts_rack.usda` | The rack the module lies on (fixed) |
| `tools/build_assets.py` | Writes all the USD files |
| `test_camera_dovetail_seat.py` | Offline checks: every scoring outcome, the 1 s timing, the arm's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values for a small robot camera module and a dovetail slide, not one catalogue entry.

- **Module** (free, 80 g, friction 0.4 / 0.3): 36 x 40 x 22 mm; a dovetail slot along its 40 mm length in the underside,
  19 mm wide at the plate and widening at the rail's slope (0.75 per mm of height) to 30 mm at 7.5 mm up, made of two
  cheek prisms with a roof over them; a lens barrel on one side and a label on top are drawn only.
- **Mount** (fixed): a plate 110 mm wide from 95 mm behind to 52 mm ahead of its middle, 50 mm above the table, on a head
  block; a rail along y from 30 mm behind to 36 mm ahead of the plate's middle: a trapezoid 14 mm wide at the plate, 23 mm
  at its top, 6 mm tall (flanks lean 0.75 per mm); a stop block 8 mm long and 16 mm tall closing its front end.
- **Rack** (fixed): 70 x 70 x 30 mm with a white square.

The slot leaves 2.5 mm of play round the rail's base and 1.5 mm at the flanks, and its lips overlap the rail's top by 2 mm
on each side, so once on the rail the module cannot be lifted off. Real dovetails have a few tenths of play and usually a
detent or screw to stop them sliding back; here the stop and the undercut alone hold it.

## Layout

Mount at x 0.34, y -0.05 (plate top 50 mm up; the rail's axis along y through x 0.34); rack at x 0.20, y -0.28 (30 mm
tall). The module is set down with its middle 56 mm behind the plate's middle (6 mm clear of the rail's rear end) and pushed
72 mm to the stop. Only the right arm is used.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=camera_dovetail_seat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/cam-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=camera_dovetail_seat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/cam-demo
```

The log prints `[cam_seat] t=...  steps done: ...` on every change and a trace each second (the module's offset from the
rail's axis, its distance along it, its yaw and tilt, and whether a gripper holds it).

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm points its gripper straight down beside the
module, turned a quarter turn so the fingers close across the module's 36 mm width (pad tips 4 mm above its underside,
about 20 N a finger), lifts it 50 mm, carries it over the plate in front of the rail's open end, sets it down (0.6 mm
skimming) with its slot over the rail's axis, and pushes it 72 mm along the rail in 3 s (commanded 8 mm past the stop,
which meets it first); holds, lets go, lifts away, goes home. Success at about step 316 (15.8 s).

## Scoring

Steps 2 to 4 count only after a gripper has held the module off the rack.

| Step | Done when |
|---|---|
| 1 | a gripper presses the module with both fingers (pads 15 mm apart or more, pads inside its outline) and its underside is at least 1 cm above the rack (counts once reached) |
| 2 | the module is on the rail: underside on the plate (-0.5 to +1.5 mm), middle within 2.5 mm of the rail's axis, slot within 4 deg of the rail and 2.5 deg of flat, middle at least 12 mm along the rail |
| 3 | step 2 and the module's middle within 2 mm of where it meets the stop |
| 4 | step 3 and no gripper pad within 3 cm of the module |

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file (`SUCCESS_HOLD_S`
  is 0): all four steps and the module within 2 mm and 2 deg of where it was when that second began. A lapse restarts
  the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** only if the module falls off the table.
- Doing nothing scores 0 (run). Shortcuts (tests and runs): the module dropped on the rail's top (run: 0.2), set down
  in front of the rail, off the axis, crooked, short of the stop, never held, or held by closed fingers or a gripper
  beside it does not complete the steps (tests).
- **Real station:** judged from video and a ruler: the module on the rail against the stop, not liftable by hand.

## Reset

Slide the module off the rail's rear end and lay it on the rack on its mark, slot along y.

## Status

Dream-1 score: 0.0 (600 steps, hit the step limit). Demo: 1.0 at step 316 (15.8 s). Media: M-6AR424.
