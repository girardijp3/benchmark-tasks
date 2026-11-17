# Measure a new brake rotor's thickness with an outside micrometer

A car-service task. A new 280 x 22 mm vented brake rotor stands on a bench hub stand and a micrometer lies on its rest. The robot has to measure the rotor the way a mechanic does: hold the micrometer square on the rotor's friction band at the marked spot with one hand, turn the ratchet knob with the other until it clicks while rocking the frame slightly across the face to find the true reading, lift the micrometer off with the reading still showing, and put it back on its rest. The instruction states only the goal and says to leave the micrometer on the reading. The rotor is a current, readily available new part, not a worn one; each real trial measures the rotor's actual thickness at the marked spot and scores the robot's reading against that measured reference.

## The rotor and the reference thickness

- **The new rotor.** [Brembo 09.7012.11](https://www.bremboparts.com/europe/en/catalogue/disc/09-7012-11), a 280 x 22 mm internally vented front brake rotor for the 2001-05 VW Golf IV (1J1) 1.8T GTI (PR 1LS), Audi A3 8L, Skoda Octavia I, Seat Leon/Toledo II, VW Bora and New Beetle; EAN 8020584018194. Brembo's catalogue gives 280 mm diameter, 22 mm thickness, 37 mm height, 65 mm centring, 5 holes, ventilated, 19 mm minimum thickness. [This retailer listing for the same EAN](https://www.autodoc.co.uk/abs/8712853) gives 5.86 kg and grey cast iron. The part is sold new; no worn rotor is needed. The simulated rotor uses the same true sizes and mass (280 mm, 22.00 mm, 37 mm tall, 65 mm bore, 5 x 100 mm stud holes, 5.86 kg); the hat's outer diameter, the 36 visual vanes and the vent core are approximations.
- **The marked spot.** A 2 mm paint dot on the outboard friction face at 135 mm from the hub axis, 10 degrees above the 9 o'clock line (on the side facing the robot), marks the measurement location. The simulated anvil sits on the dot at the measuring pose; the scoring band is radius 100-136 mm around the hub axis and the anvil must be within 5 mm of the dot.
- **The reference measurement, each real trial.** Clean the outboard face with isopropyl alcohol and let it dry. Use a calibrated 0-25 mm outside micrometer with a ratchet stop and 0.001 mm resolution, such as the Mitutoyo 293-340-30 class (accuracy +-0.001 mm, flatness 0.0003 mm, parallelism 0.001 mm, ratchet stop), checked at the start of the session against a certified 22.000 mm or 25.000 mm grade-0 gauge block; record the zero error. Seat the faces on the dot, rock the frame slightly to the minimum, and turn the ratchet until it clicks twice. Take five readings, lifting and re-seating between each. The reference is the median of the five; the spread (max - min) must be <= 0.005 mm. If it is larger, clean the faces and repeat; if it stays larger, the rotor or the micrometer is suspect and the trial is not scored. Record the part number, dot position, micrometer ID, gauge-block zero, the five readings, the median and the date on the trial card. The robot's displayed reading is scored against that median within +-0.02 mm, the same band as the simulator. If only a 0.01 mm-resolution reference is available, use the +-0.05 mm band and write that on the card.

The skill is a true two-hand tool job on a precision instrument: the frame is gripped by its spine (12 mm thick) and slid over the rim with 1.5 mm to spare on each face (start-offset window about +-1.6 mm in y, a real fit), and the 15 mm ratchet knob is turned end-on with wrist rolls (40 strokes of 120 degrees, letting go and re-gripping between them) until it clicks. The reading must be within +-0.02 mm of the rotor's measured thickness, and the hand rocks the frame about 2.5 degrees each way, about the rotor's radial axis and across it, pivoting on the faces, while the ratchet keeps turning, so the spindle follows the faces down to the smallest reading (the true thickness). This is the standard micrometer technique of the manufacturers' user guides (Mitutoyo, Starrett: turn the ratchet stop and rock the micrometer slightly on the work to find the minimum reading). A frame tilted 1 degree reads 0.11 mm high; a fixed grip cannot hold it square to +-0.02 mm. The lock lever is on the model and works, but a mechanic's reading does not need it, so it is not scored and the instruction does not ask for it.

| File | What it is |
|---|---|
| `brake_rotor_micrometer.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/micrometer.usda` | The micrometer (OpenUSD): free articulation, spindle prismatic along y, thimble and ratchet revolute about y, lock lever revolute about x |
| `assets/rotor.usda` | The new 280 x 22 mm vented brake rotor on its hub stand (OpenUSD): fixed |
| `assets/rest.usda` | The micrometer's bench rest with two uprights (OpenUSD): fixed |
| `build_assets.py` | Writes the USD files in `assets/` (pxr, closed 96-sided round meshes) |
| `test_brake_rotor_micrometer.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

**How it differs from the caliper task (#601, a caliper on a pin, open jaws, one-hand read of a sliding scale), the tread-depth gauge task (#2133, a probe pushed into a tread groove) and the stand-mounted micrometer task (#2248, a micrometer clamped in a stand, closed on a 20 mm gauge block and locked, the frame never handled):** this is a ratchet thread closed on a fixed rotor. One hand holds a 12 mm spine and keeps a 6.35 mm-faced frame square on a rim while rocking it (a 1 degree tilt already costs 0.11 mm of reading), the other turns a ratchet knob end-on in 40 strokes (let go and re-grip between strokes) until it slips; the check is the thickness reading of the new rotor kept on the thimble. **The real check:** photograph the thimble and sleeve of the micrometer left on the rotor and compare the reading with the reference measured at the dot before the trial (median of five readings, +-0.02 mm; see above).

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are the cited product's where a source is given; estimates are marked.

- **Micrometer** (free articulation, 0.176 kg): a 12 mm thick C frame (spine 9 mm, arms 9 mm, throat 30 mm deep) with heat plates, a 9.5 mm sleeve, an 18 mm thimble with graduations, a 15 mm knurled ratchet knob, carbide 6.35 mm faces, and a 16 mm lock lever. Round parts are closed 96-sided meshes. Class: Mitutoyo 103-137 (0-25 mm, ratchet stop, lock). From its retailer data sheets: mass 175 g, thimble 18 mm, faces 6.35 mm, pitch 0.5 mm, measuring force 5-10 N. **Estimates, not catalogue values:** the frame dimensions (typical of the class, not measured from one unit); the ratchet slip torque 0.006 N m (an estimate, not measured: 10 N measuring force x (0.08 mm ideal screw arm, pitch / 2 pi, + 0.15 friction x 3.5 mm thread radius), about 0.6 mm in all; the real ratchet is a spring-loaded tooth clutch and its value differs between units); the lock lever peak torque 0.03 N m (an estimate with no datasheet or measurement behind it: chosen as a light but definite finger push, about 3 N at roughly 10 mm up the 16 mm lever; a stiffer real lever would need a firmer push, which the arm can give); 20 ratchet teeth (a click every 18 degrees, an estimate).
- **Rotor** (fixed): the new Brembo 09.7012.11, 280 mm diameter, 22.00 mm thick, 37 mm overall height (15 mm hat behind the inboard friction face), 65 mm centre bore, 5 x 100 mm stud holes, 36 visual vanes, 5.86 kg grey cast iron; the collider is one solid 280 x 22 mm cylinder plus the hat (the vented core is not modelled in the collider; the micrometer only ever touches the two friction faces). The hat's outer diameter and the vane pattern are visual approximations. A 2 mm paint dot at 135 mm from the hub axis, 10 degrees above the 9 o'clock line, marks the reference spot.
- **Rest and hub stand** (fixed, 6061-T6 aluminium, blue anodised, the authors' bench fixtures, not catalogue products): the rest has an 80 x 80 x 30 mm base and two 46 x 6 x 146 mm uprights with a 36 mm clear gap for the lower finger pad, 0.8 mm EPDM caps on top. The hub stand has a 200 x 200 x 12 mm base, a 70 x 16 x 140 mm column and a 90 mm diameter x 12 mm flange; the rotor's hat sits on the flange. (Real micrometer stands, such as Mitutoyo's clamp-type stands, hold the frame in a different way; this is a simple bench rest.)

## Layout

Rotor centre x 0.508, y 0.120, hub axis along y at 0.152 m. The reference dot is 135 mm from the hub axis, 10 degrees above the 9 o'clock line, on the outboard face (world x 0.375, z 0.9230). The micrometer starts on the rest at x 0.330, y 0.165; it is measured at x 0.375, y 0.1325, the anvil face centre on the dot. The left arm grips the spine level (pointing +x, pads closing up and down), the right arm the ratchet knob end-on (pointing +y).

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

**The demo.** Open loop: fixed waypoints, no simulator reads. The left arm takes the spine, lifts the micrometer 25 mm, carries it beside the rotor, slides it 45 mm onto the rim and presses the anvil to the rotor's face. The right arm takes the ratchet knob and turns it in 40 wrist strokes of 120 degrees (about 65-90 degrees of knob turn each), letting go and re-gripping between strokes. From stroke 22 on, as the faces close, the left arm rocks the frame (2.5 degrees each way in two slow motions of 4.0 s and 5.3 s, pivoting on the anvil face, growing and dying away over 3 s) while the ratchet clicks; the rocking ends at the squared attitude. The left arm then slides the micrometer off the rim and sets it on the rest. Open loop: no simulator reads. MuJoCo 1.0 at step 2792 (kept reading 22.015 mm, +0.015 mm). The Isaac demo needs a rerun with the new rotor; on the previous 21.78 mm rotor the same plan reached 1.0 at step 2815.

## Scoring

| Step | Done when |
|---|---|
| 1 | the micrometer was closed square on the friction band at the dot (anvil centre 100-136 mm from the hub axis and within 5 mm of the dot), spindle held back by the rotor, reading = its measured thickness |
| 2 | with step 1, the ratchet clicked at least twice while that held |
| 3 | with step 2, the micrometer is lifted off the rotor and its reading (the thimble, kept at the last good contact) is still within +-0.02 mm of the measured thickness |
| 4 | with step 3, the micrometer lies on its rest, let go, grippers clear, reading unchanged |

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file (in `success()`):
  all four steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the micrometer off the table or dropped); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: turning the knob with the micrometer away from the rotor gives no clicks, a reading taken before the faces close on the rotor does not count (step 3 fails), and clicks off the rotor do not count (all tests). Left alone, either arm alone scores 0 in MuJoCo.
- **Real station:** judged from video: the micrometer square on the rotor at the dot, the ratchet clicking, and the reading unchanged when set down. The reading is compared with the reference card (median of five readings at the dot, +-0.02 mm, or +-0.05 mm if only a 0.01 mm reference is available).

## Fidelity (what is real, what is declared)

- **Real:** the micrometer's 0.5 mm pitch, 6.35 mm faces, 175 g and 18 mm thimble (retailer data of the Mitutoyo 103-137 class), the new rotor's 280 mm diameter, 22.0 mm new thickness, 37 mm height and 5.86 kg (Brembo 09.7012.11 and its retailer data), the +-0.02 mm reading band (never widened; the same on MuJoCo and Isaac), the 20 N finger grip of the frame.
- **Declared (stated simplifications):** the thread (the ratchet knob's turn sets the spindle's drive target 0.5 mm a turn; Screw couplings are not PhysX joints); the ratchet slip torque 0.006 N m and 20 teeth (estimates); the measuring force (6 N from a 0.03 mm lag; the spindle drive is capped at 10 N). The real tool's force cap is its ratchet: the ratchet slips once the thread pushes with the measuring force, which Mitutoyo specifies as 5-10 N for this class, so the sim's 10 N cap is the top of the real ratchet's range and the 6 N working force sits inside it. The simulator's reference is the new rotor's nominal 22.00 mm; a real trial's reference is measured at the dot each time (median of five, spread <= 0.005 mm), and the same +-0.02 mm band is applied.
- **Rocking, stated:** after the faces have closed on the rotor the drive target follows the spindle closing and never opens by itself (the thread does not back off; the real ratchet keeps its push while the hand turns). The sim lets the spindle follow even if the knob is not turning at that moment, an easing over the real tool, where the hand keeps turning the ratchet while rocking; the demo does keep turning (rocking starts in stroke 22 of 40). A tilted frame pushes the spindle open (reading high), so the spindle settles at the smallest reading the rocking reaches; the thimble keeps the last reading that was within the band when the micrometer comes off the rotor. Clicks are counted while the faces are on the rotor and the spindle is held (not only inside the reading band, because the reading is high on every tilted pass).
- **Estimates, not measured:** the frame dimensions, the rocking motion's size and periods (chosen to sweep a 2.5 degree cone), the lever peak torque 0.03 N m (the lever is not scored), the hat's outer diameter and the 36-vane visual pattern.
- **MuJoCo vs Isaac:** MuJoCo scores 1.0 with the new rotor at step 2792; Isaac uses the same code and needs a rerun with the new rotor (the previous rotor scored 1.0 at step 2815). MuJoCo's soft contact sinks the faces a little, so its kept reading is a few hundredths of a millimetre different from Isaac's. The demo's kept reading is checked against the sim's 22.00 mm reference.

## Reset

Lay the micrometer across the rest's ribs with the spindle fully open (25.0 mm), the lock lever unlocked and the ratchet at rest. Nothing is held.

## Status

MuJoCo 1.0 at step 2792 (the kept reading is 22.015 mm, +0.015 mm, inside the +-0.02 mm band), idle 0, MuJoCo left alone or right alone 0, Dream-1 0.0 (hit the step limit); offline checks 38 passed. The Isaac demo and media need a rerun with the new rotor; on the previous 21.78 mm rotor the same plan reached 1.0 at step 2815 (kept reading 21.797 mm).

Dream-1 score: 0.0. Demo: 1.0 (MuJoCo, step 2792, new rotor); Isaac rerun pending. Media M-8A9YE2.
