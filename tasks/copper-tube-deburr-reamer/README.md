# Ream the burr off a freshly cut copper tube

A plumber's task. A piece of copper tube has just been cut, and the cut left a burr round the inside of its end. The robot has to ream it off with a hand deburring reamer: lift the tube out of its rack and hold it upright in one hand, lower the reamer's cone into the bore and turn it clockwise, and stop once the burr is gone all round, because turning on past the burr chamfers the thin wall. The instruction states only the goal; that the tube must be held, how far to turn and in what strokes, and where the tool goes between strokes are the robot's to work out.

The skill is a hand tool used on a hand-held part: side-gripping a 22 mm tube and lifting it clear of its rack, putting a 72 mm reamer's cone into a 20.2 mm bore (a cone on a 5.7 mm radius of lead-in), turning it with the wrist (about 120 degrees a stroke, so the hand has to let go, turn back and take a fresh grip between strokes while the blade stays engaged), and then returning both parts.

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
- **Reamer** (free, 0.090 kg): an inner-outer deburring reamer of the common hand type: a conical blade (8 mm at the tip, 26 mm across, 14 mm long; a convex mesh), a 45 mm cutting cup 8 mm thick, a 30 mm handle 50 mm long; 72 mm in all. Dimensions are typical of the RIDGID 223 class, estimated, not taken from one product's drawing. The scorer uses the real cone: radius 4 mm at the tip, 13 mm at 14 mm up (slope 9/14), against the 20.2 mm bore and the 0.40 mm burr ring at 9.70-10.10 mm.
- **Tube rack** (fixed): the authors' own bench tube-rack fixture (not a catalogue product): a 90 x 90 x 4 mm plate, a 27 mm bore through a 20 mm high wall of 24 boxes, a 7 mm 45 degree lead-in.
- **Reamer rest** (fixed): a 90 x 90 x 6 mm plate, an 18 mm bore through an 18 mm high wall, a 5 mm lead-in; the cone stands in it on the bore's edge.

## Declared effects and the short offcut

- **Short offcut.** The tube is a 100 mm offcut, not a full 3 m length: it is short so that the left arm can reach it and lift it clear of its rack. Plumbers do ream short offcuts (cut-off ends and fitting pieces are common), but a long length would be held differently.
- **Declared burr, measured engagement, modelled cutting torque.** The 0.40 mm burr and the chamfer are declared effects, not simulated material (12 sectors, two blades, 0.2 mm off a sector's burr per pass). A blade pass now needs real blade contact, not just a cone inside the bore. The cone's cutting edge has radius 4 + (9/14) x depth mm; it meets the burr only when that radius reaches the burr ring at 9.70-10.10 mm. The scored contact band is a radial bite of 0.05-0.50 mm past the burr's inner edge, i.e. a tip 8.944-9.644 mm below the rim; the natural seat on the bore's edge is 9.489 mm (bite 0.400 mm). At the old 6 mm threshold the edge radius is only 7.857 mm, 1.843 mm clear of the burr: contact force 0 N and no cut, however fast the cone turns. The per-blade contact force is 13.2 N from the cut torque (T = 2 mu F r, T 0.08 N m, mu 0.30, r 10.1 mm); the least counted is 5 N. The torque is modelled as an equal and opposite pair: while the blade is engaged and the reamer turns clockwise, the task file applies 0.08 N m about the vertical as a torque on the tube (dragging it clockwise) and the opposite 0.08 N m on the reamer (resisting its turning, so the turning hand feels it), every control step. The value is an estimate from the burr's shear, not a measurement: shear strength of annealed to half-hard copper 150-230 MPa (200 MPa used); chip section per blade 0.2 mm bite x 0.1 mm of ragged lip engaged at a time = 2e-8 m2, so about 4 N per blade at the 10 mm bore radius; two blades cut at once: 2 x 4 N x 0.010 m = 0.08 N m. It is about half of what the demo's two-pad side grip can hold (2 pads x 0.5 friction x 15 N x 11 mm = 0.17 N m) and several times what the tube's own friction on the rack plate can hold (0.017 N m), so the holding hand has to react it and an unheld tube turns with the reamer (checked on Isaac: with the left arm holding the tube only until the first stroke and then letting go, the log's tube `yaw` went from 0 to -7 to +40 degrees as the reamer turned, the tube was then knocked over, the burr stayed at 0.40 mm and the score was 0.2). The burr only drops while the blade is engaged, a gripper holds the tube and the reamer turns clockwise relative to it.

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

Offline checks (no GPU, no network): `python -m pytest -q test_copper_tube_deburr_reamer.py` (33 tests).

**The demo.** Open loop: fixed waypoints, no simulator reads. The left arm side-grips the tube 84 mm up from its lower end (about 15 N a finger) and lifts it out of the rack. The right arm takes the reamer by its handle (pad tips 30 mm above the cone's tip), lifts it out of its rest, carries it over the tube and lowers the cone into the bore until it sits on the bore's edge (9.489 mm in, a 0.400 mm bite, 13.2 N per blade). It then turns clockwise in four strokes of 120 degrees (4 s each); between strokes it lets go, turns its wrist back (1.6 s) and grips again. It lifts the reamer out, stands it in its rest, lets go and goes home; then the left arm lowers the tube back into the rack, lets go and goes home. The previous Isaac run reached 1.0 at step 1027 (51.4 s); after the revision the same plan reaches 1.0 at step 1017 (50.9 s) in MuJoCo. `LOWER_AIM` is now (2.3, 1.5) mm: it keeps the held tube inside the rack's 7 mm lead-in in both simulators (MuJoCo's stand-in does not apply the cutting torque).

## Scoring

| Step | Done when |
|---|---|
| 1 | the reamer's blade was engaged in the burr of the tube while a gripper held it (edge 0.05-0.50 mm into the 0.40 mm burr, tip 8.944-9.644 mm below the rim, within 3 mm of the axis and 8 degrees of its tilt) |
| 2 | all 12 sectors have had at least one blade pass (burr at least half removed all round) |
| 3 | all 12 sectors have had two passes (burr removed all round), no rim chamfered past 0.5 mm |
| 4 | with step 3: the tube stands in its rack, the cone is out of the bore, nothing held, no pad within 3 cm of either part |

A blade pass: the reamer turns clockwise with the blade engaged (edge radius 9.75-10.20 mm, i.e. a 0.05-0.50 mm bite into the 9.70-10.10 mm burr ring; tip 8.944-9.644 mm below the rim, within 3 mm of the axis, under 8 degrees tilt), a gripper holding the tube (it need not be lifted out of the rack). Two blades sit half a turn apart, so a half turn gives every sector one pass. Each pass takes 0.2 mm off a sector's burr (0.4 mm), then 0.2 mm off its rim. A centered cone at 6 mm has its 7.857 mm edge 1.843 mm clear of the burr: contact force 0 N, no pass, and step 1 does not count.

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file (in `success()`):
  all steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the tube or the reamer off the table, or the tube dropped or knocked over); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: turning counter-clockwise, turning with the blade not engaged, turning with the tube not held, and wiggling under 2 degrees cut nothing; a centered cone at 6 mm is 1.84 mm clear of the burr and cuts nothing; the tube held while the reamer stays in the rest scores nothing; turning on past the burr thins the wall (steps 3 and 4 are lost past 0.5 mm of chamfer) (all tests). Left alone, either arm alone scores 0 in MuJoCo.
- **Real station:** judged with a 10x loupe or a scaled macro photo and two go plug gauges, as below; the tube in its rack and the reamer put down are checked by eye.

### Real-trial measurement (simple tools)

Measure the tube on the bench after the run, not in the robot's hand. Use a 10x loupe (or a phone macro photo with a 0.1 mm reticle or a steel rule in the end-face plane), a printed 12-sector protractor overlay, and two go plug gauges: **19.7 mm** and **20.0 mm** (+0/-0.02 mm, 0.5 mm lead). The gauges assume the nominal 20.2 mm bore; the photo is the primary record and the gauge is the cross-check.

1. **Step 2, burr at least half gone all round.** Look straight down the bore and divide the end face into 12 equal 30 degree sectors. Pass if **every sector's remaining lip is 0.20 mm or less** (at least 0.20 mm of the original 0.40 mm gone); a full 0.40 mm lip in any sector fails. Gauge cross-check: the **19.7 mm** go plug enters **30 mm** by hand without force. A 0.40 mm lip in any sector blocks it; a 0.20 mm lip passes with 0.05 mm to spare.
2. **Step 3, burr gone all round.** Pass if **no burr lip is visible in any of the 12 sectors at 10x** (residual 0.05 mm or less) and the **20.0 mm** go plug enters 30 mm by hand. Cross-check: a nominal 22 mm end-feed copper fitting slides on to its stop by hand. A residual 0.20 mm lip blocks the 20.0 mm plug.
3. **Step 3, chamfer.** In the same 10x view, measure the bright bevel on the flat end face at the bore edge against the reticle or the scaled photo. Pass if **every sector is 0.50 mm or less**; the normal finish is **0.30-0.50 mm**, and a sharp 0.00-0.30 mm edge with no burr also passes. **Over 0.50 mm is an over-ream: it fails steps 3 and 4** and is the only way turning on past the burr loses the last two steps. Do not measure the outside edge; the outside burr is not this task.
4. **Step 4, put away.** The tube's lower end is on the rack plate and within 4 mm of the bore axis, leaning under 20 degrees; the reamer is in its rest; no gripper touches either part and no pad is within 30 mm.

The 0.5 mm limit follows the edge-size convention of ISO 13715:2017 (an edge callout of 0.5 mm maximum), and the 10x loupe with a reticle is the usual shop way to read a burr or edge break that small.

## Reset and the 0.40 mm burr coupon

**Every trial starts with one fresh coupon.** Stand a 100 mm piece of 22 x 0.9 mm EN 1057 R250 copper tube in the rack, lower end on the plate, open end up, with a **0.40 +/-0.05 mm inner lip all round**; stand the reamer cone-down in its rest; nothing is held. The tube is a consumable: do not ream it between trials.

**Make the coupon: wheel-cut, the real method.** The Copper Development Association's *Copper Tube Handbook* (CDA A4015-26/24, section 7, and ASTM B828-23 section 6.2) says a disc-type tube cutter is an accepted way to cut copper tube and leaves an inside burr that must be reamed off. That burr is the coupon.

1. Cut a 100 mm piece from one straight length with a wheel-type tube cutter: RIDGID 151 with an E-3469 copper wheel (or an equivalent 6-42 mm cutter). Seat the wheel, turn one full circle to score the groove, then feed the knob about **1/16 turn per revolution** (the RIDGID 151-156 operating instruction) and keep turning until the wheel parts the tube. Do not snap the last turn off; that leaves a torn lip.
2. Do not ream the cut end. Leave the continuous inside lip. Remove the outside burr only if it stops the tube entering the rack.
3. Measure the lip at **12 points 30 degrees apart** with a 10x loupe and a 0.1 mm reticle, or a macro photo with a steel rule in the end-face plane. The lip height is the radial step from the 20.2 mm bore wall to the lip's inner edge. Keep **0.35-0.45 mm** (target 0.40); reject under 0.35 or over 0.45. A fresh wheel usually leaves 0.2-0.4 mm; a wheel with a few cuts on it leaves a slightly larger, more even lip. Cut 20 coupons in one sitting, measure every one, and keep the passers in a labelled tray; use one per trial.
4. **Machined alternative.** Face the end of a 100 mm tube in a lathe (collet close to the cut, sharp HSS tool, **0.10 mm/rev, 0.2 mm depth, dry**) and keep the exit burr; measure and select the same 0.35-0.45 mm. This is the batch-coupon route for a shop: order 20 coupons to `22.0 +/-0.05 mm OD x 0.9 +/-0.05 mm wall x 100 +/-0.5 mm long, one end with a 0.40 +/-0.05 mm inner lip, outside deburred`. Burr height and its measurement are covered by Gillespie's *Deburring and Edge Finishing Handbook* (SME) and ISO 13715:2017.

**Why 0.40 mm.** It is the lip a wheel cutter leaves on this 22 x 0.9 mm tube in the task's own image and the value the declared effect uses (two 0.20 mm layers). It is also the value the scoring is built around: one blade pass takes 0.20 mm, so the half-burr step and the full-burr step are 0.40 mm apart.

## Status

Dream-1 score: 0.0 (2,400 steps, hit the step limit). Demo: the existing Isaac media run reached 1.0 at step 1027 (51.4 s); after the revision the same plan reaches 1.0 at step 1017 (50.9 s) in MuJoCo. Media M-C7YEFJ. No new media was uploaded: the scoring change only rejects a cone that is not touching the burr, and the existing demo's cone is at the natural 9.489 mm seat (0.400 mm bite, 13.2 N per blade), so the filmed run is still a valid full-score demonstration. The 1 mm lowering-aim change is inside the rack's lead-in and is not visible as a task difference.

Simulator settings for the loose parts: damping of 1 per s (linear) and 5 per s (angular) and 64 position / 8 velocity solver iterations, so the tube and reamer settle in their holders instead of chattering. For the film only, the reamer's cone, cup, flutes, ferrule, grip and end cap are hidden (the hidden reamer in demo.mp4 is only drawn that way; the real parts are there) (render-only cutaway, physics unchanged) to look into the bore.
