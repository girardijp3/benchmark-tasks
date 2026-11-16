# Bump-test a four-gas monitor before a confined-space entry

A pre-entry instrument check. An MSA ALTAIR 4X four-gas monitor lies face up on the bench with its calibration cap on
a rest beside it, the cap's hose running to a 0.25 L/min fixed-flow regulator on a 34 L quad-gas cylinder in a bench
cradle. The robot has to seat and snap the cap until its gasket seals, open the regulator, let the four readings come
up to the cylinder's values and stay there, then close the gas, pull the cap off and leave the monitor back on fresh
air. The instruction states only that goal.

The physical core is the **cap seal + regulator valve + a sensor reading that stabilises**. The cap is a real 62 x
84 x 12 mm shell with an orange silicone gasket, two side snap tabs and a 6 mm hose barb; the regulator has a knurled
on/off knob with a red pointer, a red CLOSE mark and a green OPEN mark; the cylinder is a real 34 L aluminium quad-gas
cylinder. The gas reading is a declared finite model of the cap's volume, the regulator's flow, the cap's seal or
leak, and the sensor's first-order response; it is not a pass stamp and it cannot be made to read the cylinder's
values without the cap actually sealing and the valve actually opening.

| File | What it is |
|---|---|
| `gas_monitor_bump_test.py` | The `yam_arena` task file: scene, the gas path, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/monitor.usda` | The ALTAIR 4X monitor, one free articulation: body and four LCD bar pointers on prismatic joints |
| `assets/cap.usda` | The 10089321 calibration cap, one free articulation: shell, two spring snap tabs on revolute joints, visual hose |
| `assets/cylinder.usda` | The 34 L quad-gas cylinder with its C-10 valve, label and bench cradle (fixed) |
| `assets/regulator.usda` | The 0.25 L/min fixed-flow regulator, one fixed-root articulation: body and on/off knob on a revolute joint |
| `assets/cap_rest.usda` | The cap's bench rest (fixed) |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; 96-sided round meshes, rounded boxes; colliders stay simple) |
| `meshkit_inline.py`, `meshkit2_inline.py` | The embedded visual-mesh helpers used by `build_assets.py` |
| `test_gas_monitor_bump_test.py` | Offline checks: the gas model, every scoring outcome and cap, the 1 s timing, the seal window, the demo's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Twin check

Board pull 2026-11-16 (`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000`: 2,356 PRs, newest
#2357). No gas-monitor, bump-test or calibration-cap task. The nearest are #1813 fit an oxygen regulator on a
cylinder valve (different object and step), #1990 refit a gas burner head and cap (burner, no monitor/reading) and
#1932 secure a gas cylinder in its clamp (a different job). No twin by skill, object or mechanism.

## The real products and values

- **Monitor: MSA ALTAIR 4X.** 112 x 76 x 33 mm, 224 g with battery and clip (MSA technical data, operating manual
  10105903). Four sensors: catalytic combustible (LEL), electrochemical O2, CO, H2S; a backlit LCD; three front
  buttons; a green SAFE LED and red alarm LEDs. The model is the 4X, not the 4XR.
- **Calibration cap: MSA 10089321.** A plastic cap that seals over the sensor openings. The manual's fit is: insert
  its tab in the slot, press it until it seats, press both side tabs down until they snap, and ensure it is seated.
  The cap's silicone gasket is 2 mm wide and 1 mm thick; the hose is 1/8 in ID (3.2 mm) Superthane ester tubing, up to
  3 ft (1 m); here one foot.
- **Bump-test gas: MSA 34 L quad mix, part 10048280.** 1.45 % CH4, 15 % O2, 60 ppm CO, 20 ppm H2S in nitrogen
  (MSA's ordering data; the manual's bump test equipment list). The cylinder is 289 mm x 76 mm, 0.85 L water capacity,
  500 psi, C-10 (5/8-18 UNF) valve; 2.0 kg full in the sim (0.44 kg empty + gas and valve, an estimate).
- **Regulator: 0.25 L/min fixed-flow.** The ALTAIR 4X manual specifies **0.25 liters/min** for calibration and bump
  testing (MSA FAQ: "a standard 0.25 lpm gas flow rate"; bump test under 15 s). The row's brief says "~0.5 L/min";
  that is the common competitor flow, not the ALTAIR 4X's, and the manual's 0.25 is used here. The regulator body is
  the BW REG-0.5 class, 0.239 kg, 109 x 97 x 91 mm envelope, C-10 inlet, 6 mm outlet barb.
- **Sensor response.** MSA quotes the 4X's response and reset time as "less than 15 seconds"; a 10 s T90 is used.
- **Tolerances.** The cylinder certificate is +-5 %, the sensors add their own accuracy; the bump band used is +-10 %
  of the cylinder's value per channel, and O2 +-0.5 % vol (the O2 sensor's display resolution and accuracy class).

**The one value that differs from the source row:** the V5 row says "~0.5 L/min for 30 s". The cited MSA ALTAIR 4X
manual says 0.25 L/min; the task uses 0.25 L/min and a 30 s settle (the manual's bump test is under 15 s; 30 s is a
real pre-entry wait, not a widened pass window). Nothing else is reduced: the monitor, cap, cylinder and regulator
are at true size and mass; the gas concentrations are the cylinder's; the seal is a real gasket and snap-tab fit.

## The gas path (declared effects, not contact physics)

The task file integrates a well-mixed cap volume every control step:

- **Cap volume** V = 25 mL (the cap cavity plus the hose dead volume, estimated from the modelled geometry). The cap's
  gasket ring is at its inner rim; the sensor plate is inside the ring.
- **Flow** Q = 0.25 L/min x clip((knob angle - 0.02 rad) / (88 deg - 0.02 rad), 0, 1). The knob is closed at 0 deg,
  fully open at about 90 deg; the joint range is -2 to 92 deg.
- **Leak** L = 40 L/min when the cap is not sealed (a deliberately large, stated leak: far more than the regulator
  supplies, so an unsealed cap cannot come up). Sealed: L = 0.
- **Gas fraction** dC/dt = (Q (1 - C) - L C) / V in the cap; C = 0 in room air. The sensors follow
  dS/dt = (C - S) / tau with tau = T90 / ln 10 = 4.34 s (T90 = 10 s).
- **Readings**: H2S = 20 S ppm, CO = 60 S ppm, LEL = 29 S % (1.45 % CH4 / 5 % LEL), O2 = 20.9 - 5.9 S %.
- **Seal**: the cap's local pose on the sensor plate (x +-3 mm, y +-3 mm, z +-1.8 mm about the seat), tilt and yaw
  under 3 deg, and both side-tab angles under 3 deg (closed). The tabs snap under the monitor's 1.6 mm side ridges.
- **Stable**: all four readings within the band and their 3 s min/max span under 2 % of the cylinder's value, inside
  the real 5-45 s window after the valve opens. The window is the real bump-test window; a reading still climbing at
  the window's edge cannot count.
- **Caps**: gas opened before the cap seals, a cap leak, or the cap pulled while gas is on -> 0.25; a reading that
  never stabilises -> 0.50; gas left on at the end -> 0.75. A run with the cap off the table or the monitor off the
  table scores 0 for the rest of the run.

The monitor's LCD is drawn as four **bar pointers** (H2S yellow, CO orange, LEL red, O2 blue) on prismatic joints
driven from the same readings. This is a declared visual simplification of the digital LCD; the scored reading is the
simulated sensor value, printed in the log. The cap's hose is a **visual tube** (no collider, no joints), so it cannot
push the cap or the arm; the regulator's outlet barb is visible under its end. The hose is not scored.

## Assets and sizes

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are the real products' (above); shapes marked EST are estimates.

- **Monitor** (224 g): 112 x 76 x 33 mm rounded body, 4 mm corner radius, black rubberised shell with a yellow bumper
  band, a dark green LCD with a silver frame, four pointer rows with white tick marks, a raised 50 x 40 x 4 mm sensor
  plate with four 4.6 mm port rings, two side ridges (1.6 mm proud, 3 mm tall, 46 mm long) for the cap tabs, a white
  label with a red band, three front buttons and green/red LEDs. Free articulation; the body is the root.
- **Cap** (78 g including the visual hose): 62 x 84 x 12 mm black shell, 1.5 mm walls, a 2 mm orange silicone gasket,
  a 6 mm hose barb with a lip, two 30 mm side tabs on revolute joints (19 mm long, 2.5 mm thick, 5 mm inward hooks
  with a top release ramp and a bottom lead-in), and one foot of 6 mm black hose. The cap's tabs are spring-loaded
  (0.020 N m/rad) and snap under the monitor's side ridges.
- **Cylinder** (2.0 kg full, fixed): 76 mm diameter x 289 mm, horizontal in a 290 x 120 mm cradle with two V blocks and
  a strap, C-10 valve at the near end, white label with a red band and four gas-colour squares. The label is on the
  upper side for the top camera.
- **Regulator** (0.239 kg, fixed root): C-10 inlet nut, 42 x 44 x 40 mm body, 6 mm outlet barb, 32 mm knurled knob with
  a red pointer, red CLOSE and green OPEN marks. The knob is a revolute joint about the vertical axis.
- **Cap rest** (0.30 kg, fixed): a base, two posts and a 60 x 32 mm pad; the cap's sealing face sits at 60 mm, its tabs
  hang outside the rest.
- **Colliders**: boxes and cylinders at true size; the round visuals are 96-sided closed meshes. The cap's gasket is a
  compliant contact (2500 N/m, 4 N s/m); the monitor and cap use rubber/plastic friction. The visual detail is in
  separate meshes; the colliders are simple.

## Layout

Left and right are the robot's. Positions in metres from the midpoint between the arm bases: x forward, y left.

- Monitor: centre (0.400, 0.000, table top 0.7475), top face 33 mm up; sensor plate centre x 0.374.
- Calibration cap: starts on its rest at (0.290, -0.100), sealing face 60 mm up; seated at (0.374, 0.000, 0.7805).
- Cylinder: valve face (0.300, -0.220, 0.7475), axis +x to 0.589; regulator inlet on the valve face at 38 mm up, knob
  at (0.274, -0.220) 58 mm up.
- The right arm works the cap and the regulator knob; the left stays home (the monitor's 224 g and the vertical cap
  press do not need a second hand, and the cap's side tabs are the only two-hand-looking feature but are worked by the
  cap's own spring action). The cap rest, monitor and regulator are all on the robot's right side.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=gas_monitor_bump_test.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gas-monitor-bump-test-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=gas_monitor_bump_test.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gas-monitor-bump-test-demo
```

Locally, with MuJoCo:

```bash
cd local_sim && .venv/bin/python -m mjkit.run \
  ../benchmark-submission/tasks/gas-monitor-bump-test/gas_monitor_bump_test.py --steps 2400 --trace
```

**Run limit: 2,400 steps (120 seconds at 20 Hz).** The scripted demo needs 1,951 steps (97.55 s) to 1.0. The log prints
`[gasmon] t=... steps done: ...` on every change and a trace each second with all four readings, the valve angle, flow,
the seal, the cap's local pose/tilt/yaw, the tab angles and the stability/safety flags.

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm takes the cap from its rest (pads on the
62 mm ends), carries it over the sensor plate, sets it down, opens its pads, presses the cap top with the closed pads,
and gives two small side presses to settle the gasket and seat both tabs. It then grips the regulator knob (32 mm),
opens it in two 45 deg wrist strokes to 90 deg, waits 30 s for the four readings to stabilise, closes it in two strokes
past 0 deg to the -2 deg stop, takes the cap by its ends again, pulls the tabs out and lifts it, sets it back on its
rest, waits 25 s for the monitor to come back to fresh air and lets go. The left arm stays home.

The removal re-grip aims at the cap's mid-height (TCP 11 mm above the seat plane, 6 mm forward of the nominal seat x):
at that reach the Isaac arm's TCP tracks about 5 mm high, and with the original seat+20 mm aim the 10 mm pad flats
closed just above the cap top (1 N graze, no pull). The aim change is demo-only; the seal, flow, reading and scoring
rules are untouched, and the MuJoCo demo still reaches 1.0 at step 1,951.

## Scoring

| Step | Done when |
|---|---|
| 1. Cap sealed | the cap's sealing face on the sensor plate (x +-3 mm, y +-3 mm, z +-1.8 mm), level and square within 3 deg, and both side tabs closed within 3 deg |
| 2. Regulator opened | the knob has reached 75 deg or more, after step 1 |
| 3. Reading stable | all four readings within +-10 % of the cylinder's values (O2 +-0.5 % vol) and holding inside that band for 3 s in the 5-45 s window after the valve opens |
| 4. Final | gas off (knob below 3 deg), the cap off the sensor plate (its local x or y more than 30 mm from the sensor axis) and clear, the monitor on the table with all four readings back in the fresh-air bands |

- **1.0** only when all four states have held together for 1 s, timed inside the task's `success()`, with the cap and
  monitor within 2 mm and 2 degrees of where they were when that second began. A lapse restarts the second.
- **0.25 / 0.50 / 0.75** for steps 1, 2 and 3 reached; the fourth state is only worth 1.0 after its 1 s hold.
- **Caps:** gas opened before the cap seals, a cap leak, or the cap pulled while gas is on -> **0.25**; a reading that
  never stabilises -> **0.50**; gas left on at the end -> **0.75**.
- **0** after the cap or monitor falls off the table; the event is kept for the whole run. Idle 0.
- **Real station:** judged from video: the cap seated and its side tabs down, the regulator knob at OPEN and then
  CLOSED, the monitor's four readings at the cylinder's values (1.45 % CH4 / 15 % O2 / 60 ppm CO / 20 ppm H2S) and back
  to fresh air after the cap comes off.

## Fidelity (sim vs real)

| Quantity | Sim | Real | Source |
|---|---|---|---|
| Monitor | 112 x 76 x 33 mm, 224 g | same | MSA 10105903 |
| Cap | 62 x 84 x 12 mm, gasket 2 x 1 mm, two snap tabs | MSA 10089321 | MSA manual |
| Cylinder | 76 mm x 289 mm, 2.0 kg full | 34 L, 500 psi, 289 x 76 mm, 0.44 kg empty | FTI / Gasco data |
| Regulator flow | 0.25 L/min | 0.25 L/min | MSA manual |
| Gas mix | 1.45 % CH4, 15 % O2, 60 ppm CO, 20 ppm H2S | MSA 10048280 | MSA ordering data |
| Cap volume | 25 mL | not published; EST from the cap + 1 ft of 3.2 mm hose | authors |
| Leak when unsealed | 40 L/min | not measured; deliberately large | authors |
| Sensor T90 | 10 s | "<15 s" | MSA technical data |
| Bump band | +-10 %, O2 +-0.5 % vol | cylinder +-5%, sensor accuracy | MSA / cylinder CoA |
| Stable window | 3 s inside 5-45 s after open | real 30 s bump | MSA manual |
| Cap pull-off | spring tabs, 20 mN m/rad, ramp release | real side tabs | authors' estimate |

**Stated simplifications:** the hose is a visual tube with no collider or joints; the LCD is four bar pointers rather
than a digital display; the monitor is a free body on the bench (no charger cradle); the cap gasket's compliant contact
is a 2500 N/m spring; the sensor response is a single first-order lag for all four channels; the cylinder's gas
pressure is not depleted (0.125 L used in a 30 s bump).

## Checks (MuJoCo, local stand-in)

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q test_gas_monitor_bump_test.py` -> **27 passed**. The
tests cover: the task loads; the instruction's 15-35 words; the gas model (sealed -> cylinder values and stable;
unsealed -> low; closed valve -> no rise; cap off -> no gas; decay to fresh air; order error/leak; vented); every
scoring outcome and cap (0.25/0.5/0.75/1.0, valve left open, no stable reading, cap leak, vented, cap still held, part
off the table, non-finite); the 1 s hold and a lapse; the seal window's small-nudge acceptance and large-nudge
rejection; the scripted right arm's reach (worst IK 0.09 mm) and the demo's timing.

**MuJoCo demo:** 1.0 at step 1,951 (97.55 s), repeatable. **Nudges:** a +1 mm start offset on both the cap and the monitor reaches 0.75
(steps 1-3; the open-loop cap-removal grip misses the 1 mm-shifted cap); a +2 mm offset scores 0.0. This is a known
open-loop sensitivity of the last removal, not of the gas core; the cap seal, valve, reading and final gas-off state
are robust, and the real trial is judged by a person on the video. No widened seal or reading window is used to hide
it.

**Isaac demo (2026-11-16):** 1.0 at step 1,951 (97.55 s) twice, then a third 1.0 at 1,951 in the film run with the
cap and regulator cameras. A first attempt failed at reset with `'tuple' object has no attribute 'shape'` because the
reset event passed joint ids as a Python tuple (the Isaac backend calls `.shape` on them); the reset now passes a
torch tensor, a compatibility fix only. The next attempt scored 0.75: the cap seal, valve opening, four readings and
gas-off all worked, but the removal grip closed just above the cap top (opening 56.5 mm, 1.0 N) and the cap never left
the plate. The per-step trace showed the actual TCP 5 mm above the command at that reach; the removal re-grip was
lowered 9 mm and moved 6 mm forward (see the demo paragraph), after which the grip closed on the cap at 20.6 N, both
tabs released and the cap went to its rest. No scoring, seal, reading, flow or safety value was changed.

**Pass-through check (PASS_THROUGH_QA.md):** the film's top-view and cap/regulator close-up frames at 0, 10, 25, 45,
60, 80, 95 and 100 % of the demo (steps 1, 196, 490, 882, 1177, 1569, 1863, 1961), plus the cap reach (steps 120 and
196), the seating press (300), the regulator opening (600) and the cap removal (1660). No gripper, finger, forearm or
held part visibly intersects the monitor, the cap, the cylinder, the regulator body or the bench; the pads only touch
the cap ends and the knob, both scored interactions. Contacts in the run are only cap/rest, cap/monitor, cap tab/
monitor, monitor/table, gripper/cap and gripper/regulator-knob; no arm link touches the cylinder, the regulator body or
the bench. Cutaways: none.

**Media:** `benchmark-media-gas-monitor-bump-test/demo.mp4` (top view only, 640 x 360, 98.05 s + a 3 s held last frame;
the PR video), `demo_top.gif` (1.14 MB), `setup.png` (top view), `dream.mp4` (top camera of the Dream-1 run) and
`demo_review.mp4` (2 x 2: top, cap/monitor, regulator knob, scoring panel; internal only). No media was uploaded and
no PR was opened.

## Reset

Put the cap on its rest with its sealing face down and its hose end at the regulator barb; the monitor lies face up at
(0.400, 0.000). The regulator knob is closed (0 deg), the cylinder is in its cradle, and nothing is held.

## Status

Dream-1: 0.0 (2,400 steps, hit the step limit; the top-camera video is in `benchmark-media-gas-monitor-bump-test/
dream.mp4`). MuJoCo demo: 1.0 at step 1,951, repeatable; pytest: 27 passed. Isaac demo: 1.0 at step 1,951 twice and
1.0 at 1,951 in the film run. Nudges: +1 mm offset 0.75, +2 mm 0.0 (cap-removal grip; stated above). Media local
(top view only for the PR); no upload.
