# Lock a real H-track lighting head into its track and tug it to prove it holds

An electrical-installation task. A real **Halo L651P H-track** segment (1.000 m cut from the 1.121 m extrusion,
35 x 17.5 mm, 24 mm slot, 120 V 20 A) is mounted with its slot **down** – the real ceiling orientation, lowered to
bench height. A real **Satco TH489** 12 W forged H-track LED head (69.85 mm dia x 76.2 mm, 0.29 kg, 159 mm
extension, 120 V) hangs below it on a **40 mm H-style twist-lock adapter**. The circuit is locked out. The robot
grips the lever knob, turns it **90 degrees** so the adapter's two contact tabs rotate under the track lips and an
angular cam detent engages, lets go, then tugs the head's stem down ~10 N to prove the fixture cannot be pulled
out. The head is left alone for 1 s.

**Result: the scripted demo scores 1.0 at step 396 (19.80 s after the settle, t = 20.30 s in the task log) in
MuJoCo, 3/3 runs.** Ladder: 0.25 at step 118, 0.5 at step 156, 0.75 at step 287, 1.0 at step 396. No wrong-way,
bent, dropped or failed-tug latch. 24 offline tests pass.

## Board / twin check (2026-11-17)

A fresh `gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state,createdAt`
returned **2,507 PRs, newest #2508** (437 OPEN at that query). Title search for track / track light / lighting
track / H-track / track head / track adapter / fixture / 120 V found no live OPEN task with the same object,
mechanism and outcome. The nearest row is **#2293 CLOSED** "Lock a spring E-track fitting into a van floor track" –
a van cargo tie-down ratchet, not a 120 V H-track lighting head with a quarter-turn electrical cam. #2482
(hanging cleaned centrifuge buckets) and the shower-curtain / soda-tab builders are unrelated. No live twin; no
`BLOCKED.md` was found for this folder.

## Files

| File | What it is |
|---|---|
| `lighting_track_adapter.py` | Task file: scene patch, fixed-root articulation (Twist/Insert), latched faults, scoring ladder, 1 s hold, scripted two-arm demo |
| `assets/lighting_track.usda` | The 1 m H-track, lips/slot, busbars, polarity groove, end caps, bench pedestal, lockout tag |
| `assets/track_head.usda` | The Satco head on its 40 mm adapter: two contact tabs, 90 mm lever knob, 2.6 N m/rad cam drive, 20 kN/m axial drive |
| `build_assets.py` | Writes both assets with pxr (lathe meshes for round parts, simple true-size colliders) |
| `test_lighting_track_adapter.py` | 24 offline tests: ladder/caps, 1 s hold, real values, USD parts/joints, demo plan, IK reach, pass-through |
| `task.json` | Board metadata (all field limits checked; keywords 132/200) |
| `README.md`, `REFERENCE.md`, `PASS_THROUGH_QA.md` | This note, sources/limits, contact audit |
| `make_media.py` | Internal renderer: top-view-only `setup.png`, `demo.mp4`, `demo_top.gif` |

## Real product and values

- **Halo L651P** single-circuit H-track: 35 mm wide x 17.5 mm high, 24 mm slot, 1-1/8 in actual 44-1/8 in
  (1.121 m); the task uses a 1.000 m cut. Cooper Lighting L650/L651 spec.
- **Satco TH489** 12 W forged track head: 69.85 mm (2.75 in) dia, 76.2 mm (3 in) long, 0.64 lb (0.29 kg),
  6.25 in (159 mm) extension, 1020 lm, 36 deg, 120 V; Satco spec sheet.
- **H-style twist-lock**: 40 mm polycarbonate adapter body, 24 mm contact neck, 90 deg quarter turn,
  0.3-0.6 N m detent; Satco/Nora H-style track-head installation sheets ("pull down lever, insert contacts,
  rotate 90 deg to locked position").

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top
0.7475. The track centre is (0.420, -0.100, 0.9975), 0.25 m above the table; the track bottom / articulation root
is z 0.98875. The head root is fixed there and its two joints are `Twist` (0 deg = tabs aligned with the slot,
-90 deg = locked) and `Insert` (0 = seated, negative = pulled out). The 24 mm lever knob is at 90 mm radius, world
z 0.950; the right hand's stem tug grip is at (0.420, -0.100, 0.943). Both arms start at READY.

## Mechanism

The head is the moving link of a **fixed-root articulation**, so it cannot fall when the gripper twists the lever;
the real load path is the two contact tabs on the 24 mm cavity floor and the stiff `Insert` drive. `Twist` is a
2.6 N m/rad angular drive capped at 0.9 N m; the real 0.3-0.6 N m detent engages below -70 deg and drives the cam to
the -90 deg stop. The robot's hand angle is **code-keyed** to the lever while it holds the knob (a handling
simplification, stated in REFERENCE.md); the real trial is a hand twist. The track is a fixed rigid body.

## Demo

The left hand approaches the lever knob at 0 deg, grips it, and turns it to -90 deg over 3.2 s; the detent seats the
cam. It releases and clears. The right hand waits, grips the exposed stem, pulls down 4 mm and holds ~1.5 s; the
adapter stays seated (insert within 0.02 mm). Both arms go home and the fixture is still for 1 s.

## Scoring

| Done when | Points |
|---|---|
| Lever turned >= 15 deg, adapter seated | 0.25 |
| Cam locked: -90 +/-5 deg, adapter seated | 0.5 |
| Locked + sustained tug on the stem | 0.75 |
| Above + both grippers clear, fixture still 1 s, no fault | **1.0** |

Doing nothing is 0 (lever at 0, not turned). **Caps:** 0 for a dropped fixture; 0.25 for a wrong-direction turn /
over-twist or a forced/bent track; 0.5 for a failed tug; an unlocked cam at the end caps at 0.25.

## Checks

```bash
cd benchmark-submission/tasks/lighting-track-adapter
PYTHONPATH=../../../local_sim ../../../local_sim/.venv/bin/python -m pytest -q
cd ../../../local_sim
PYTHONPATH=. .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/lighting-track-adapter/lighting_track_adapter.py --steps 900 --trace
```

pytest: **24 passed**. MuJoCo: 1.0 at step 396, 3/3 deterministic runs; Isaac: **1.0 at step 396**
(`adhoc_395d23de.json`). Dream-1: **0.0 at 2,400 steps** (2,401 action rows). Run limit: 2,400 steps (120 s at
20 Hz), the Dream evaluation budget.

## Media

The prepared package is kept outside the task folder in `benchmark-media-lighting-track-adapter/`: a **top-view**
`setup.png` (32,486 B), `demo.mp4` (Isaac top camera, 640 x 360, 357,661 B, final 1.0 frame held 3 s),
`demo_top.gif` (480 x 270, 5 fps, 1,598,580 B) and `dream.mp4` (completed Dream-1 top camera, 320 x 176,
741,004 B). No cutaways, no wrist-camera inserts in the PR media. `task.json` media M-X4QN25.

## Reset

Set `Twist` = 0 deg, `Insert` = 0, both arms to READY with open grippers, track/pedestal/lockout tag fixed, 120 V
off. The first 15 deg of lever turn arms the score.

## Status

- MuJoCo demo: **1.0 at step 396 (19.80 s), 3/3**; no fault latched.
- Isaac demo: **1.0 at step 396** (`adhoc_395d23de.json`); lever -90 deg, tug passed, no fault. Two render-only
  close-ups were filmed on a copy of the task; see `PASS_THROUGH_QA.md`.
- Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed); top-camera `dream.mp4`.
- Board: no live exact twin on the 2026-11-17 2507-PR query.
- No push, upload or PR was made; no media was published.
