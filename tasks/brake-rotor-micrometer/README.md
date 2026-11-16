# Measure a brake rotor's thickness with an outside micrometer

A car-service task. A worn brake rotor stands on a bench hub stand and a micrometer lies on its rest. The robot has to measure the rotor the way a mechanic does: hold the micrometer square on the rotor's friction band with one hand, turn the ratchet knob with the other until it clicks while rocking the frame slightly across the face to find the true reading, lift the micrometer off with the reading still showing, and put it back on its rest. The instruction states only the goal and says to leave the micrometer on the reading.

The skill is a true two-hand tool job on a precision instrument: the frame is gripped by its spine (12 mm thick) and slid over the rim with 1.5 mm to spare on each face (start-offset window about +-1.6 mm in y, a real fit), and the 15 mm ratchet knob is turned end-on with wrist rolls (40 strokes of 120 degrees, letting go and re-gripping between them) until it clicks. The reading must be within +-0.02 mm of the rotor's thickness, and the hand rocks the frame about 2.5 degrees each way, about the rotor's radial axis and across it, pivoting on the faces, while the ratchet keeps turning, so the spindle follows the faces down to the smallest reading (the true thickness). This is the standard micrometer technique of the manufacturers' user guides (Mitutoyo, Starrett: turn the ratchet stop and rock the micrometer slightly on the work to find the minimum reading). A frame tilted 1 degree reads 0.11 mm high; a fixed grip cannot hold it square to +-0.02 mm. The lock lever is on the model and works, but a mechanic's reading does not need it, so it is not scored and the instruction does not ask for it.

| File | What it is |
|---|---|
| `brake_rotor_micrometer.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/micrometer.usda` | The micrometer (OpenUSD): free articulation, spindle prismatic along y, thimble and ratchet revolute about y, lock lever revolute about x |
| `assets/rotor.usda` | The 280 mm vented brake rotor on its hub stand (OpenUSD): fixed |
| `assets/rest.usda` | The micrometer's bench rest with two ribs (OpenUSD): fixed |
| `build_assets.py` | Writes the USD files in `assets/` (pxr, closed 96-sided round meshes) |
| `test_brake_rotor_micrometer.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

**How it differs from the caliper task (#601, a caliper on a pin, open jaws, one-hand read of a sliding scale), the tread-depth gauge task (#2133, a probe pushed into a tread groove) and the stand-mounted micrometer task (#2248, a micrometer clamped in a stand, closed on a 20 mm gauge block and locked, the frame never handled):** this is a ratchet thread closed on a fixed rotor. One hand holds a 12 mm spine and keeps a 6.35 mm-faced frame square on a rim while rocking it (a 1 degree tilt already costs 0.11 mm of reading), the other turns a ratchet knob end-on in 40 strokes (let go and re-grip between strokes) until it slips; the check is the discard-thickness reading of the rotor kept on the thimble. **The real check:** photograph the thimble and sleeve of the micrometer left on the rotor and compare the reading with a gauge-block or second-micrometer value of the same rotor (21.78 mm here).

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Micrometer** (free articulation, 0.176 kg): a 12 mm thick C frame (spine 9 mm, arms 9 mm, throat 30 mm deep) with heat plates, a 9.5 mm sleeve, an 18 mm thimble with graduations, a 15 mm knurled ratchet knob, carbide 6.35 mm faces, and a 16 mm lock lever. Round parts are closed 96-sided meshes. Class: Mitutoyo 103-137 (0-25 mm, ratchet stop, lock). From its retailer data sheets: mass 175 g, thimble 18 mm, faces 6.35 mm, pitch 0.5 mm, measuring force 5-10 N. **Estimates, not catalogue values:** the frame dimensions (typical of the class, not measured from one unit); the ratchet slip torque 0.006 N m (an estimate, not measured: 10 N measuring force x (0.08 mm ideal screw arm, pitch / 2 pi, + 0.15 friction x 3.5 mm thread radius), about 0.6 mm in all; the real ratchet is a spring-loaded tooth clutch and its value differs between units); the lock lever peak torque 0.03 N m (an estimate with no datasheet or measurement behind it: chosen as a light but definite finger push, about 3 N at roughly 10 mm up the 16 mm lever; a stiffer real lever would need a firmer push, which the arm can give); 20 ratchet teeth (a click every 18 degrees, an estimate).
- **Rotor** (fixed): 280 mm, 21.78 mm thick (the task's chosen measured value of a worn part), 36 vanes, hat 40 mm deep, black coated, 5 stud holes; the collider is one solid cylinder (the vented core is not modelled in the collider; the micrometer only ever touches the two friction faces).
- **Rest** (fixed): a bench stand of the authors' own design, not a catalogue product: a base and two 6 mm ribs 146 mm tall that the frame's arms lie across, with a 32 mm slot between them for the lower finger pad. (Real micrometer stands, such as Mitutoyo's clamp-type stands, hold the frame in a different way; this is a simple bench rest.) The hub stand under the rotor is likewise the authors' design.

## Layout

Rotor centre x 0.508, y 0.120, hub axis along y at 0.152 m. The micrometer starts on the rest at x 0.330, y 0.165; it is measured at x 0.375, y 0.1325. The left arm grips the spine level (pointing +x, pads closing up and down), the right arm the ratchet knob end-on (pointing +y).

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=brake_rotor_micrometer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 3200 --store-frames --log-dir ~/yam-eval/brake-rotor-micrometer-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=brake_rotor_micrometer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 3100 --store-frames --log-dir ~/yam-eval/brake-rotor-micrometer-demo
```

The log prints `[mic] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The left arm takes the spine, lifts the micrometer 25 mm, carries it beside the rotor, slides it 45 mm onto the rim and presses the anvil to the rotor's face. The right arm takes the ratchet knob and turns it in 40 wrist strokes of 120 degrees (about 65-90 degrees of knob turn each), letting go and re-gripping between strokes. From stroke 22 on, as the faces close, the left arm rocks the frame (2.5 degrees each way in two slow motions of 4.0 s and 5.3 s, pivoting on the anvil face, growing and dying away over 3 s) while the ratchet clicks; the rocking ends at the squared attitude. The left arm then slides the micrometer off the rim and sets it on the rest. Open loop: no simulator reads. Success at step 2815 on Isaac.

## Scoring

| Step | Done when |
|---|---|
| 1 | the micrometer was closed square on the friction band, spindle held back by the rotor, reading = its thickness |
| 2 | with step 1, the ratchet clicked at least twice while that held |
| 3 | with step 2, the micrometer is lifted off the rotor and its reading (the thimble, kept at the last good contact) is still within +-0.02 mm of the thickness |
| 4 | with step 3, the micrometer lies on its rest, let go, grippers clear, reading unchanged |

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file (in `success()`):
  all four steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the micrometer off the table or dropped); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: turning the knob with the micrometer away from the rotor gives no clicks, a reading taken before the faces close on the rotor does not count (step 3 fails), and clicks off the rotor do not count (all tests). Left alone, either arm alone scores 0 in MuJoCo.
- **Real station:** judged from video: the micrometer square on the rotor, the ratchet clicking, and the reading (21.78 mm, check with a second micrometer) unchanged when set down.

## Fidelity (what is real, what is declared)

- **Real:** the micrometer's 0.5 mm pitch, 6.35 mm faces, 175 g and 18 mm thimble (retailer data of the Mitutoyo 103-137 class), the rotor's 280 mm size and 21.78 mm thickness, the 36 vanes, the +-0.02 mm reading band (never widened; the same on MuJoCo and Isaac), the 20 N finger grip of the frame.
- **Declared (stated simplifications):** the thread (the ratchet knob's turn sets the spindle's drive target 0.5 mm a turn; Screw couplings are not PhysX joints); the ratchet slip torque 0.006 N m and 20 teeth (estimates); the measuring force (6 N from a 0.03 mm lag; the spindle drive is capped at 10 N). The real tool's force cap is its ratchet: the ratchet slips once the thread pushes with the measuring force, which Mitutoyo specifies as 5-10 N for this class, so the sim's 10 N cap is the top of the real ratchet's range and the 6 N working force sits inside it.
- **Rocking, stated:** after the faces have closed on the rotor the drive target follows the spindle closing and never opens by itself (the thread does not back off; the real ratchet keeps its push while the hand turns). The sim lets the spindle follow even if the knob is not turning at that moment, an easing over the real tool, where the hand keeps turning the ratchet while rocking; the demo does keep turning (rocking starts in stroke 22 of 40). A tilted frame pushes the spindle open (reading high), so the spindle settles at the smallest reading the rocking reaches; the thimble keeps the last reading that was within the band when the micrometer comes off the rotor. Clicks are counted while the faces are on the rotor and the spindle is held (not only inside the reading band, because the reading is high on every tilted pass).
- **Estimates, not measured:** the frame dimensions, the rocking motion's size and periods (chosen to sweep a 2.5 degree cone), the lever peak torque 0.03 N m (the lever is not scored).
- **MuJoCo vs Isaac:** both score 1.0 with the same code; MuJoCo's soft contact sinks the faces a little, so its kept reading is a few hundredths of a millimetre different from Isaac's.

## Reset

Lay the micrometer across the rest's ribs with the spindle fully open (25.0 mm), the lock lever unlocked and the ratchet at rest. Nothing is held.

## Status

Isaac demo 1.0 at step 2815 (the kept reading is 21.797 mm, +0.017 mm, inside the +-0.02 mm band), idle 0, MuJoCo left alone or right alone 0, Dream-1 0.0 (hit the step limit); MuJoCo 1.0 at step 2792; offline checks 33 passed.

Dream-1 score: 0.0. Demo: 1.0 (Isaac, step 2815). Media M-8A9YE2.
