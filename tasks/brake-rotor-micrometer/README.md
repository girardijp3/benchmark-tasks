# Inspect an installed in-service brake rotor's thickness with an outside micrometer

A routine automotive-service task. A **worn, in-service front brake rotor** stays bolted to its hub and steering knuckle inside the wheel well, with the wheel and the caliper unbolted and the caliper carrier bracket still on the knuckle. Eight scribe marks around the friction band show where a technician checks thickness; a paint dot marks the thinnest point. A 0-25 mm outside micrometer lies on its bench rest. The robot has to do the inspection the way a mechanic does: hold the micrometer square on the band at the marked point with one hand, turn the ratchet knob with the other until it clicks while rocking the frame slightly across the face to find the true minimum, lift the micrometer off with the reading still showing, and put it back on its rest. The instruction states only the goal and says to leave the reading on the micrometer.

Each real trial measures its own rotor's thickness at the marked point and scores the robot's reading against that measured reference. The simulator's reference is **21.50 mm** at the marked point — a serviceable worn example of the real 22.0 mm-new, 19 mm-discard part, not a new part on an open bench.

## The part, the installed setting and the measurement locations

- **The rotor.** [Brembo 09.7012.11](https://www.bremboparts.com/europe/en/catalogue/disc/09-7012-11), a 280 x 22 mm internally vented front brake rotor for the 2001-05 VW Golf IV (1J1) 1.8T GTI (PR 1LS), Audi A3 8L, Skoda Octavia I, Seat Leon/Toledo II, VW Bora and New Beetle; EAN 8020584018194. Brembo's catalogue gives 280 mm diameter, 22 mm thickness, 37 mm height, 65 mm centring, 5 holes, ventilated, **19 mm minimum thickness**. [The same-EAN retailer listing](https://www.autodoc.co.uk/abs/8712853) gives 5.86 kg (new) and grey cast iron. The simulated rotor keeps the real 280 mm diameter, 22.0 mm new thickness, 19 mm discard stamp, 65 mm bore and 5 x 100 mm stud holes; the worn simulated disc is 21.50 mm thick at the inspection band (~35.5 mm overall) with an estimated worn mass of 5.74 kg (0.25 mm removed per face over the 100-140 mm band at ~7200 kg/m³). The hat diameter and the 36 visual vanes are approximations.
- **Installed wheel-well context (real station change, not a backdrop).** The rotor's hat is bolted by five M12 wheel studs to a cast-iron front hub flange (110 mm dia x 12 mm) carried by a 90 mm bearing housing and a steering knuckle (upright ~180 mm, lower ball-joint boss, strut clamp, steering arm). The painted-steel caliper carrier bracket stays bolted to the knuckle at 6 o'clock, its bridge over the disc rim and its abutment rails on the two faces; the caliper body and pads are removed for the check so the band is open. A 6 mm paint dot at radius 135 mm, 10 degrees above the 9 o'clock line (facing the robot), marks the thinnest of the eight points. All of this hardware is fixed and shares the rotor's hub-centre origin; it changes the station the robot works in, while the micrometer still touches only the two friction faces.
- **The measurement locations.** Routine inspection measures thickness at **eight equally spaced circumferential points (every 45 degrees) at radius 135 mm** (5 mm inboard of the 140 mm friction rim) on the wear band, on both faces; the smallest reading is compared with the 19 mm discard stamp. The eight points are scribed on the outboard face in the sim. The scored spot is the thinnest point, marked with the 6 mm paint dot; the simulated anvil sits there, the scoring band is radius 100-136 mm, and the anvil must be within 5 mm of the dot so the reference and the reading are the same spot.

## The reference measurement, each real trial (repeatable procedure)

1. **Access and clean.** With the wheel removed and the caliper unbolted but its bracket left on the knuckle, clean the outboard wear band with isopropyl alcohol and let it dry. Leave the rotor installed, the stud nuts torqued and the bracket in place.
2. **Calibrate.** Use a calibrated 0-25 mm outside micrometer with a ratchet stop and 0.001 mm resolution, such as the Mitutoyo 293-340-30 class (accuracy +-0.001 mm, flatness 0.0003 mm, parallelism 0.001 mm), checked at the start of the session against a certified 22.000 mm or 25.000 mm grade-0 gauge block; record the zero error.
3. **Scan the eight points.** At each of the eight scribed points at radius 135 mm, seat the faces on the band, rock the frame slightly to the minimum and turn the ratchet until it clicks twice. Record all eight thicknesses and their spread (max - min): this is the disc-thickness variation that the inspection looks for. The smallest is the thinnest point; if it is at or below the 19 mm stamp, the rotor is scrapped and the trial is not run.
4. **Mark and measure the reference.** Mark the thinnest point with a 6 mm paint dot. Lift and re-seat the micrometer there five times, rocking to the minimum and clicking twice each time. The reference is the median of the five; the spread (max - min) must be <= 0.005 mm. If it is larger, clean the faces and repeat; if it stays larger, the rotor or the micrometer is suspect and the trial is not scored.
5. **Record.** On the trial card: part number, the eight readings and their spread, dot position (radius and angle), micrometer ID, gauge-block zero, the five readings, the median, the disc's discard stamp and the date. The robot's displayed reading is scored against that median within +-0.02 mm, the same band as the simulator. If only a 0.01 mm-resolution reference is available, use the +-0.05 mm band and write that on the card.
6. **Example values (this simulator).** Eight-point readings near 21.50 mm with <= 0.05 mm variation, thinnest at the marked dot; the reference median is 21.500 mm (spread 0.003 mm in the worked example). The sim's reference is 21.50 mm; the real part's value is measured per trial, never assumed.

The skill is a true two-hand tool job on a precision instrument: the frame is gripped by its spine (12 mm thick) and slid over the rim with 1.25 mm to spare on each face in this worn setting (start-offset window about +-1.6 mm in y, a real fit), and the 15 mm ratchet knob is turned end-on with wrist rolls (40 strokes of 120 degrees, letting go and re-gripping between them) until it clicks. The reading must be within +-0.02 mm of the rotor's measured thickness, and the hand rocks the frame about 1.5 degrees each way about the rotor's radial axis and across it, pivoting on the faces, while the ratchet keeps turning, so the spindle follows the faces down to the smallest reading (the true thickness). This is the standard micrometer technique of the manufacturers' user guides (Mitutoyo, Starrett: turn the ratchet stop and rock the micrometer slightly on the work to find the minimum reading); a real technician uses about 2.5 degrees, while the demo uses 1.5 degrees (2.5 degrees jammed the earlier, tighter 22.00 mm frame; the worn disc leaves 1.25 mm of clearance). A frame tilted 1 degree reads 0.11 mm high; a fixed grip cannot hold it square to +-0.02 mm. The lock lever works but is not scored, and the instruction does not ask for it.

| File | What it is |
|---|---|
| `brake_rotor_micrometer.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/micrometer.usda` | The micrometer (OpenUSD): free articulation, spindle prismatic along y, thimble and ratchet revolute about y, lock lever revolute about x |
| `assets/rotor.usda` | The worn 280 mm disc with eight inspection marks and the paint dot, installed on the hub (OpenUSD): fixed |
| `assets/hub_knuckle.usda` | The wheel hub, studs, bearing housing and steering knuckle (OpenUSD): fixed installed context |
| `assets/caliper_bracket.usda` | The caliper carrier bracket still on the knuckle at 6 o'clock (OpenUSD): fixed installed context |
| `assets/rest.usda` | The micrometer's bench rest with two uprights (OpenUSD): fixed |
| `build_assets.py` | Writes the USD files in `assets/` (pxr, closed 96-sided round meshes) |
| `test_brake_rotor_micrometer.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach, the worn reference and installed hardware (no GPU, no network) |
| `task.json` | The task's details for the board |

**How it differs from the caliper task (#601, a caliper on a pin, open jaws, one-hand read of a sliding scale), the tread-depth gauge task (#2133, a probe pushed into a tread groove) and the stand-mounted micrometer task (#2248, a micrometer clamped in a stand, closed on a 20 mm gauge block and locked, the frame never handled):** this is a ratchet thread closed on a worn part still installed in the wheel well. One hand holds a 12 mm spine and keeps a 6.35 mm-faced frame square on a rim while rocking it (a 1 degree tilt already costs 0.11 mm of reading), the other turns a ratchet knob end-on in 40 strokes (let go and re-grip between strokes) until it slips; the check is the thickness of the in-service disc at its thinnest marked point, against the discard stamp and a reference measured on the trial's own rotor. **The real check:** photograph the thimble and sleeve, and compare the reading with the reference card (median of five readings at the marked thinnest point, +-0.02 mm, or +-0.05 mm with a 0.01 mm reference).

**Difference from the earlier version of this task:** the previous version presented a **new** 22.00 mm rotor on an open bench hub stand, measured at one marked spot with no installed surroundings. This revision puts the same real product in service: a worn rotor (21.50 mm in the sim) bolted to a hub and knuckle with the caliper bracket in place, eight specified inspection locations, a real-trial eight-point scan with a marked thinnest point and a five-reading median reference, and a repeatable installed-setting reset. The micrometer, force model, scoring ladder, 1 s hold and demo are otherwise the same; the measurement physics of the ladder is unchanged.

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres, kilograms, Z up. Sizes are the cited product's where a source is given; estimates are marked.

- **Micrometer** (free articulation, 0.176 kg): a 12 mm thick C frame (spine 9 mm, arms 9 mm, throat 30 mm deep) with heat plates, a 9.5 mm sleeve, an 18 mm thimble with graduations, a 15 mm knurled ratchet knob, carbide 6.35 mm faces, and a 16 mm lock lever. Class: Mitutoyo 103-137 (0-25 mm, ratchet stop, lock). Retailer data: 175 g, thimble 18 mm, faces 6.35 mm, pitch 0.5 mm, measuring force 5-10 N. **Estimates, not catalogue values:** frame dimensions; ratchet slip torque 0.006 N m (10 N x thread torque arm, a spring-loaded tooth clutch differs between units); lever peak torque 0.03 N m (not scored); 20 teeth (a click every 18 degrees).
- **Rotor** (fixed): the worn Brembo 09.7012.11, 280 mm diameter, 21.50 mm thick at the inspection band (22.0 mm new, 19 mm discard), ~35.5 mm overall height, 65 mm bore, 5 x 100 mm stud holes, 36 visual vanes, ~5.74 kg grey cast iron (estimated worn mass). The collider is one solid 280 x 21.50 mm cylinder plus the hat; the vented core is not modelled (the micrometer only touches the faces). Eight 2 mm scribe marks and a 6 mm paint dot at radius 135 mm show the inspection points.
- **Hub, knuckle and caliper bracket** (fixed installed context, cast iron and painted steel, representative of a 2001-05 VW Golf IV front corner — **estimated, not catalogue dimensions**): hub flange 110 mm dia x 12 mm, bearing housing 90 mm dia x 40 mm, upright ~180 mm, lower ball-joint and tie-rod bosses, strut clamp, five M12 studs; caliper carrier with a bridge over the rim and two pad abutment rails, bolted to the knuckle at 6 o'clock. The caliper body and pads are absent (removed for the check).
- **Rest** (fixed, 6061-T6 aluminium, blue anodised, the authors' bench fixture, not a catalogue product): 80 x 80 x 30 mm base, two 46 x 6 x 146 mm uprights with a 36 mm clear gap for the lower finger pad, 0.8 mm EPDM caps on top.

## Layout

Rotor hub centre x 0.508, y ~0.1208, hub axis along y at z 0.152 m above the table. The marked thinnest point is at radius 135 mm, 10 degrees above the 9 o'clock line, on the outboard face. The micrometer starts on the rest at x 0.330, y 0.165; it is measured at x 0.375, y 0.1325, the anvil face centre on the dot. The knuckle is inboard (-y) of the disc, the caliper bracket at the bottom (6 o'clock), both clear of the 9 o'clock measuring path. The left arm grips the spine level (pointing +x, pads closing up and down), the right arm the ratchet knob end-on (pointing +y).

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

**The demo.** Open loop: fixed waypoints, no simulator reads. The left arm takes the spine, lifts the micrometer 25 mm, carries it beside the wheel-well rotor, slides it 45 mm onto the rim and presses the anvil to the rotor's face at the marked point. The right arm takes the ratchet knob and turns it in 40 wrist strokes of 120 degrees (about 65-90 degrees of knob turn each), letting go and re-gripping between strokes. From stroke 22 on, as the faces close, the left arm rocks the frame (1.5 degrees each way in two slow motions of 4.0 s and 5.3 s, pivoting on the anvil face, growing and dying away over 3 s) while the ratchet clicks; the rocking ends at the squared attitude. The left arm then slides the micrometer off the rim and sets it on the rest. The installed hub, knuckle and caliper bracket are fixed and are not touched.

## Scoring

| Step | Done when |
|---|---|
| 1 | the micrometer was closed square on the worn rotor's friction band at the marked thinnest point (anvil centre 100-136 mm from the hub axis and within 5 mm of the dot), spindle held back by the rotor, reading = its measured thickness |
| 2 | with step 1, the ratchet clicked at least twice while that held |
| 3 | with step 2, the micrometer is lifted off the rotor and its reading (the thimble, kept at the last good contact) is still within +-0.02 mm of the measured thickness |
| 4 | with step 3, the micrometer lies on its rest, let go, grippers clear, reading unchanged |

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file's `success()`: all four steps, no rule event, and the parts within 2 mm and 2 degrees of where they were when that second began. A lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the micrometer off the table or dropped); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: turning the knob with the micrometer away from the rotor gives no clicks, a reading taken before the faces close on the rotor does not count, and clicks off the rotor do not count.
- **Real station:** judged from video: the micrometer square on the worn rotor at the marked thinnest point, the ratchet clicking, and the reading unchanged when set down. The reading is compared with the reference card (eight-point scan, then median of five readings at the dot, +-0.02 mm, or +-0.05 mm with a 0.01 mm reference).

## Fidelity (what is real, what is declared)

- **Real:** the micrometer's 0.5 mm pitch, 6.35 mm faces, 175 g and 18 mm thimble (retailer data of the Mitutoyo 103-137 class); the rotor's 280 mm diameter, 22.0 mm new thickness, 19 mm discard stamp, 65 mm bore and 5.86 kg new mass (Brembo 09.7012.11 and retailer data), worn to 21.50 mm at the inspection band here; the +-0.02 mm simulator reading band (the same on MuJoCo and Isaac).
- **Declared (stated simplifications):** the thread (the knob's turn sets the spindle drive target 0.5 mm a turn; screw couplings are not PhysX joints); ratchet slip torque 0.006 N m and 20 teeth (estimates); measuring force 6 N from a 0.03 mm lag with the drive capped at 10 N (the top of the real ratchet's 5-10 N spec). The hub, knuckle and caliper bracket are fixed installed context at representative estimated sizes; their colliders are away from the measuring path, and the micrometer contacts only the rotor faces.
- **Rocking, stated:** after the faces close on the rotor the drive target follows the spindle closing and never opens by itself (the thread does not back off; the real ratchet keeps its push while the hand turns). The sim lets the spindle follow even if the knob is not turning at that moment, an easing over the real tool; the demo keeps turning (rocking starts in stroke 22 of 40). A tilted frame pushes the spindle open (reading high), so the reading settles at the smallest the rocking reaches; the thimble keeps the last good reading when the micrometer comes off. Clicks count while the faces are on the rotor and the spindle is held.
- **Estimates, not measured:** the worn mass (5.74 kg), the frame and lever dimensions, the rocking amplitude and periods, the lock-lever torque, the hat diameter and 36-vane visual pattern, and all hub/knuckle/bracket sizes.

## Reset (repeatable, installed setting)

The wheel stays installed between trials: rotor on the hub, five stud nuts torqued, caliper bracket on the knuckle, caliper and pads off, the eight marks and the paint dot in place. Clean the wear band with isopropyl alcohol and let it dry; re-scan the eight points and re-mark the thinnest with a 6 mm dot if the mark has rubbed off. Lay the micrometer across the rest's ribs with the spindle fully open (25.0 mm), the lock lever unlocked and the ratchet at rest. Nothing is held. Re-measure the five-reading median reference at the dot and record it on the trial card before scoring the robot.

## Media and the model run

The scene changed materially (fixed hub, knuckle and caliper bracket; 0.5 mm thinner disc), so the top-view demo video (640 x 360), GIF and setup image were **regenerated from a new Isaac film run** of the installed scene, and the model run was **re-run** rather than assumed unchanged. The re-run scored 0.0 by hitting the 3,200-step limit, the same as before; the measuring-ladder physics, action space and scoring conditions are unchanged, so the score is not sensitive to the new context. The scripted demo is the feasibility demonstration and was re-run too: 1.0 on Isaac (kept reading 21.518 mm, +0.018 mm) and 1.0 on MuJoCo (21.516 mm, +0.016 mm), both inside the +-0.02 mm band.

## Status

MuJoCo demo 1.0 at step 2815 (kept 21.516 mm, +0.016 mm) and Isaac demo 1.0 (kept 21.518 mm, +0.018 mm); Dream-1 re-run 0.0 (3,200-step limit); offline checks 40 passed. The exact step numbers are in `task.json`'s `scores` field and the PR body.
