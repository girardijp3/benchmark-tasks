# Ream the burr off a freshly cut copper tube

A plumber's task. A piece of copper tube has just been cut, and the cut left a burr round the inside of its end. The robot has to ream it off with a hand deburring reamer: lift the tube out of its rack and hold it upright in one hand, lower the reamer's cone into the bore and turn it clockwise, and stop once the burr is gone all round, because turning on chamfers the thin wall. The instruction states only the goal; that the tube must be held, how far to turn and in what strokes, and where the tool goes between strokes are the robot's to work out.

The skill is a hand tool used on a hand-held part: side-gripping a 22 mm tube and lifting it clear of its rack, putting a 72 mm reamer's cone into a 20.2 mm bore (a cone on a 5.7 mm radius of lead-in), turning it with the wrist (about 120 degrees a stroke, so the hand has to let go, turn back and take a fresh grip between strokes while the reamer stays seated), and then returning both parts.

| File | What it is |
|---|---|
| `copper_tube_deburr_reamer.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/copper_tube.usda` | The 22 x 0.9 x 100 mm copper tube (OpenUSD): an articulation (free root) with 36 visual marker links that show the burr and the chamfer |
| `assets/reamer.usda` | The inner-outer deburring reamer (OpenUSD): conical blade, cup and handle; free rigid body |
| `assets/tube_rack.usda, assets/reamer_rest.usda` | The tube rack and the reamer rest (OpenUSD): plates with bores and lead-ins; fixed |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; boxes, cylinders and two small convex meshes) |
| `test_copper_tube_deburr_reamer.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Copper tube** (free articulation, 0.053 kg): 22 mm outside, 0.9 mm wall (EN 1057 R250, the common 22 mm size), 100 mm long, copper (friction 0.50 / 0.45). A solid cylinder stands in for the lower 86 mm; the top 14 mm is a ring of 32 boxes (a true 20.2 mm bore). Round the inside of the open end: 12 sectors of burr (two 0.2 mm layers, orange) and 12 rim marks (bright, they appear when a sector's rim is chamfered), all visual-only markers on prismatic joints.
- **Reamer** (free, 0.090 kg): an inner-outer deburring reamer of the common hand type: a conical blade (8 mm at the tip, 26 mm across, 14 mm long; a convex mesh), a 45 mm cutting cup 8 mm thick, a 30 mm handle 50 mm long; 72 mm in all. Dimensions are typical of the RIDGID 223 class, estimated, not taken from one product's drawing.
- **Tube rack** (fixed): the authors' own bench tube-rack fixture (not a catalogue product): a 90 x 90 x 4 mm plate, a 27 mm bore through a 20 mm high wall of 24 boxes, a 7 mm 45 degree lead-in.
- **Reamer rest** (fixed): a 90 x 90 x 6 mm plate, an 18 mm bore through an 18 mm high wall, a 5 mm lead-in; the cone stands in it on the bore's edge.

## Declared effects and the short offcut

- **Short offcut.** The tube is a 100 mm offcut, not a full 3 m length: it is short so that the left arm can reach it and lift it clear of its rack. Plumbers do ream short offcuts (cut-off ends and fitting pieces are common), but a long length would be held differently.
- **Declared burr, modelled cutting torque.** The burr and the chamfer are declared effects, not simulated material. The cutting torque is modelled as an equal and opposite pair: while the cone is seated and the reamer turns clockwise, the task file applies 0.08 N m about the vertical as a torque on the tube (dragging it clockwise) and the opposite 0.08 N m on the reamer (resisting its turning, so the turning hand feels it), every control step. The value is an estimate from the burr's shear, not a measurement: shear strength of annealed to half-hard copper 150-230 MPa (200 MPa used); chip section per blade 0.2 mm bite x 0.1 mm of ragged lip engaged at a time = 2e-8 m2, so about 4 N per blade at the 10 mm bore radius; two blades cut at once: 2 x 4 N x 0.010 m = 0.08 N m. It is about half of what the demo's two-pad side grip can hold (2 pads x 0.5 friction x 15 N x 11 mm = 0.17 N m) and several times what the tube's own friction on the rack plate can hold (0.017 N m), so the holding hand has to react it and an unheld tube turns with the reamer (checked on Isaac: with the left arm holding the tube only until the first stroke and then letting go, the log's tube `yaw` went from 0 to -7 to +40 degrees as the reamer turned, the tube was then knocked over, the burr stayed at 0.40 mm and the score was 0.2). The burr only drops while the cone is seated, a gripper holds the tube and the reamer turns clockwise relative to it.

## Layout

The rack's bore axis is at x 0.27, y -0.05; the reamer rest's at x 0.22, y -0.20. The left arm handles the tube (gripper pointing along -y, pads closing along x), the right arm the reamer (pointing down).

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=copper_tube_deburr_reamer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/copper-tube-deburr-reamer-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=copper_tube_deburr_reamer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1250 --store-frames --log-dir ~/yam-eval/copper-tube-deburr-reamer-demo
```

The log prints `[reamer] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The left arm side-grips the tube 84 mm up from its lower end (about 15 N a finger) and lifts it out of the rack. The right arm takes the reamer by its handle (pad tips 30 mm above the cone's tip), lifts it out of its rest, carries it over the tube and lowers the cone into the bore until it sits on the bore's edge (9.5 mm in). It then turns clockwise in four strokes of 120 degrees (4 s each); between strokes it lets go, turns its wrist back (1.6 s) and grips again. It lifts the reamer out, stands it in its rest, lets go and goes home; then the left arm lowers the tube back into the rack, lets go and goes home. Success at about step 1027 (51.4 s) on Isaac. The lowering aim (LOWER_AIM) is tuned for Isaac (the held tube hangs a little differently from MuJoCo, whose stand-in does not apply the cutting torque either).

## Scoring

| Step | Done when |
|---|---|
| 1 | the reamer's cone was seated in the bore of the tube while a gripper held it |
| 2 | all 12 sectors have had at least one blade pass (burr at least half removed all round) |
| 3 | all 12 sectors have had two passes (burr removed all round), no rim chamfered past 0.5 mm |
| 4 | with step 3: the tube stands in its rack, the cone is out of the bore, nothing held, no pad within 3 cm of either part |

A blade pass: the reamer turns clockwise with the cone seated (tip 6 mm or more below the rim, within 3 mm of the axis, under 8 degrees tilt), a gripper holding the tube (it need not be lifted out of the rack). Two blades sit half a turn apart, so a half turn gives every sector one pass. Each pass takes 0.2 mm off a sector's burr (0.4 mm), then 0.2 mm off its rim.

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file (in `success()`):
  all steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the tube or the reamer off the table, or the tube dropped or knocked over); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: turning counter-clockwise, turning with the cone not seated, turning with the tube not held, and wiggling under 2 degrees cut nothing; the tube held while the reamer stays in the rest scores nothing; turning on past the burr thins the wall (steps 3 and 4 are lost past 0.5 mm of chamfer) (all tests). Left alone, either arm alone scores 0 in MuJoCo.
- **Real station:** judged from video: a bright, even, burr-free edge round the inside of the tube's end with no visible bevel, the tube in its rack, the reamer put down. A real check is a gauge pin of 20.2 mm that enters without catching, and a caliper on the rim for the chamfer.

## Reset

Stand the tube in the rack (lower end on its plate), open end up, with its burr; stand the reamer cone-down in its rest. Nothing is held.

## Status

Dream-1 score: 0.0 (2400 steps, hit the step limit). Demo: 1.0 at step 1027 (51.4 s). Media M-C7YEFJ.

Simulator settings for the loose parts: damping of 1 per s (linear) and 5 per s (angular) and 64 position / 8 velocity solver iterations, so the tube and reamer settle in their holders instead of chattering. For the film only, the reamer's cone, cup, flutes, ferrule, grip and end cap are hidden (the hidden reamer in demo.mp4 is only drawn that way; the real parts are there) (render-only cutaway, physics unchanged) to look into the bore.
