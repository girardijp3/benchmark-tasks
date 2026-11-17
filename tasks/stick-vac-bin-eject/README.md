# Release a stick vacuum's dust bin, empty it, refit and latch it

A household/janitorial task. A **Dyson V6-class cordless stick vacuum** motor/body module (2.04 kg, published bin
capacity 0.4 L) stands on the station table with its clear dust bin seated on the front. The robot presses the red
bin-release button, pulls the bin forward off its cradle, carries it over a waste bin, empties the dust through the
bottom flap, then slides the bin back until the button clicks. The instruction states only the goal — that the latch
must be pressed before the bin will come out honestly, that the dust must land in the waste bin and not on the bench,
and that the vac must stay put are the robot's to work out.

**Live-twin check (2026-11-17T19:49:30Z).** `gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000`
returned 2,512 PRs, newest #2513. No live OPEN task has this object + mechanism + outcome. The nearest, **#1262**
(OPEN), empties a **robot** vacuum's dustbin by lifting it off by a grip fin and holding an orange lid button to open
**bottom doors** — a different product and release mechanism. See [TWIN_CHECK.md](TWIN_CHECK.md) for every neighbour
and the fresh query.

## Files

| File | What it is |
|---|---|
| `stick_vac_bin_eject.py` | The `yam_arena` task file: scene, the free vac/bin articulations, the declared dump flap, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/vac.usda` | The Dyson V6-class body: shell, motor, wand/head props, four feet, front cradle and the red prismatic release button (600 N/m, 10 mm, 6 N) |
| `assets/bin.usda` | The clear 0.4 L bin (70 x 70 x 85 mm): walls, carry bar and the hinged bottom dump flap |
| `assets/waste_bin.usda`, `assets/debris_*.usda` | The open waste bin and six light pieces of dust |
| `build_assets.py` | Writes the USD files (pxr; boxes, cylinders, small closed meshes) |
| `test_stick_vac_bin_eject.py` | 19 offline checks: every scoring outcome, idle, the 1 s hold and restart, all five caps, stability, grip clearance, real numbers, reach and demo order |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its features, sources and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame and contact evidence for every demo phase |
| `TWIN_CHECK.md` | The fresh live-twin query and why #1262 is not a stop |

## Real product and values

The **Dyson V6 Cord-free** is a 0.4 L bagless stick vacuum weighing 2.04 kg (Dyson support specifications); the whole
vac is 211 x 1180 x 208 mm. The manual and emptying guides describe the **red "bin release" button** that detaches the
clear bin, the **bottom release** that drops the dust into a bin, and refitting **until it clicks**. Maids and
housekeeping cleaners (BLS SOC 37-2012, ~900,000 jobs) and janitors (SOC 37-2011, ~2.2 million) do this work.

**Published:** 0.4 L bin, 2.04 kg, 211 x 1180 x 208 mm, red bin-release button, bottom dump.

**Estimated (each in `REFERENCE.md` and the fidelity field):** the 120 x 150 x 195 mm simulated upper module; the
70 x 70 x 85 mm (0.42 L) clear bin at 0.14 kg plus 0.06 kg of debris; the 10 mm, 6 N button drive; the -100 deg flap;
the 240 x 240 x 35 mm waste bin; and all button/bin/waste positions. No mass, size or force is reduced. The wand and
floor head are visual props.

## Layout

The vac stands at **x 0.36, y 0** on four rubber feet. The bin seats in the front cradle at **x 0.29, y 0**, its bottom
16 mm above the table; the red button is on a thin bracket at **x 0.292, y 0, z 0.215**. The 240 mm waste bin is at
**x 0.30, y 0.22**. The right arm performs the whole sequence; the left stays at READY (closest arm–arm approach
101 mm). All waypoints are inside the 0.3 m height gate and the arm's reachable workspace.

## Mechanism

The vac is one articulation: `Body` is free on four feet and `Eject` is a 10 mm prismatic red button with a 600 N/m
spring (6 N at the stop). The bin is a second free articulation: `Can` holds six loose debris and a hinged `Flap`
bottom (0 shut, -100 deg open). There is **no pin and no hole** — the button's rule and a low open cradle replace the
catch, avoiding the pin/ear jam that blocked the earlier canister task.

Pressing the button past 5 mm by a hand sets `opened_once`. Pulling the bin 50 mm from its seat sets `removed`;
seating it with the button back out sets `clicked`. The flap is a **declared stand-in** for the real bottom red tab: it
opens only while the held bin is over the waste bin and closes as soon as all six debris have landed. The flap, hinge,
bin articulation and debris are real bodies.

## Demo

The right arm presses the button (about 6 N), releases it, grips the bin's side walls at its centre of mass, pulls it
75 mm off the cradle, lifts it over the waste bin, opens the flap until the dust has fallen in, closes the flap, carries
the empty bin back, lowers it to the cradle and lifts straight clear. The vac is never pushed or lifted.

Demo: **1.0 at step 523 twice in MuJoCo** (26.15 s after the settle; the 1 s hold ends at t=26.65 s). Steps: button
0.25 at t=4.30 s, bin out 0.5 at 10.75 s, dust emptied 0.75 at 13.70 s, refit+hold 1.0 at 25.65 s. Final bin offset
4 mm, tilt well inside 8 deg; vac tilt and travel far inside the stability window. Idle scores 0.

## Scoring

| Step | Done when | Points |
|---|---|---|
| button pressed | red button pressed past 5 mm by a hand | 0.25 |
| bin pulled out | button done and bin >= 50 mm from its seat | 0.5 |
| dust emptied | all six debris in the waste bin | 0.75 |
| refit + hold | bin within 12 mm / 8 deg of seat, button clicked, vac stable, both hands clear, held 1 s | **1.0** |

- **0** otherwise, including doing nothing.
- **Caps:** dropped bin 0.25; dust left or unlatched refit 0.75; forced pull or latch 0.5; vac tilted over 8 deg,
  moved over 25 mm or off the table 0.
- Scored from the vac's and bin's own joints and poses; the real masses and free roots are unchanged.
- **Real station:** 4-7 N gauge and ruler for the button, video for bin travel and dust landing, stopwatch for the
  1 s hold after both hands are clear.

## Checks

```bash
cd benchmark-submission/tasks/stick-vac-bin-eject
PYTHONPATH= ../../../local_sim/.venv/bin/python -m pytest -q test_stick_vac_bin_eject.py
cd ../../../local_sim
PYTHONPATH= .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/stick-vac-bin-eject/stick_vac_bin_eject.py --steps 620 --trace
```

- pytest: **19 passed**. `check_fields.py`: **ok** (keywords 136 < 200).
- MuJoCo: **1.0 at step 523, twice**; Isaac: **1.0 at step 520** (`adhoc_e3f66aa5.json`); idle 0; button 0.25;
  bin out 0.5; emptied 0.75; refit+hold 1.0; all five caps verified in the unit tests.
- YAM reach: **18/18** right-arm waypoints at 0.0 mm / 0.0 deg static IK, per-waypoint path error under 1 mm / 1 deg; arm–arm closest approach 101 mm.
- **Run limit: 2,400 steps (120 s at 20 Hz), the Dream evaluation budget.** The scripted demo needs about 520
  steps (26 s) plus the 1 s hold. Dream-1: **0.0 at 2,400 steps** (2,401 action rows).

## Media

`benchmark-media-stick-vac-bin-eject/`: `demo.mp4` (Isaac top view only, 640x360, 429,755 B, final 1.0 frame held
3 s), `demo_top.gif` (480x270, 5 fps, 1,411,428 B), `setup.png` (640x360, 31,892 B), `dream.mp4` (completed
Dream-1 top camera, 320x176, 486,558 B). Each PR file is under 1.87 MB; `task.json` media M-YDWAFV.

## Status

MuJoCo demo **1.0 at step 523, twice**; Isaac demo **1.0 at step 520**; Dream-1 **0.0 at 2,400 steps**
(2,401 action rows). pytest **19 passed**; fields and consistency ok. Media in `benchmark-media-stick-vac-bin-eject`
is top-view only for the PR, with the Dream video separate. `task.json` media M-YDWAFV. No push or upload was made.
