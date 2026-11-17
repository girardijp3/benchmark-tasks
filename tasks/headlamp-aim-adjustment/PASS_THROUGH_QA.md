# PASS_THROUGH_QA — headlamp-aim-adjustment

The task was run end to end in MuJoCo (mjkit, the station physics at 240 Hz, 20 Hz control) with a contact
log on every control step and every contact body pair recorded. The run reached **1.0 at step 433**. The same
scripted demo was then run on Isaac and also reached **1.0 at step 433**. The pair table below is the MuJoCo
contact log; Isaac does not expose the same per-step pair log, so the Isaac evidence is the top-cam frames
(0, 5, 10, 15, 20 s and the final pose), which were reviewed frame by frame.

## Arms vs. the product

| Pair | Contacts | Min distance | What it is |
|---|---|---|---|
| `rig/KeyV` <-> `right/tip_left` | 420 | -1.215 mm | the right pads gripping the V key's 6 mm T-bar |
| `rig/KeyV` <-> `right/tip_right` | 364 | -0.664 mm | the other pad on the V T-bar |
| `rig/KeyH` <-> `right/tip_right` | 509 | -0.770 mm | the right pads gripping the H key's T-bar |
| `rig/KeyH` <-> `right/tip_left` | 534 | -0.698 mm | the other pad on the H T-bar |
| `left/tip_left` <-> `rig/AimPitch` | 1,678 | -0.059 mm | the left closed tips on the rubber steady pad |

The 1.215 mm is the soft contact spring of the task's 1 mm contact offset on the 6 mm T-bar; it is the
intended key grip, not a pass-through. The keys are the screwdriver tools: their 8 mm shafts sit in the
screws' 8.2 mm sockets by design, and the keys/heads/shafts are one fixed chain, so the key/head contact is
not an arm pass-through. The left contact is the rubber steady pad on the lamp's left side; the left tips
press 2 mm into the 4 mm rubber, and the pad plate and the lamp housing are not crossed.

## Isaac run

The Isaac demo reached **1.0 at step 433** (`adhoc_969eeeab.json`); V +163.34 deg, H -177.62 deg, cut-off
823.7 mm / -128.4 mm. The top-cam frames (320 x 176, 434 frames) show the left tips on the rubber pad, the
right pads on the key T-bars, the two turns, the release and both arms going home; no arm, key or forearm is
seen passing through the lamp housing, stand, screen or table. The 3.00 m screen is outside the top camera;
the beam result is in the separate `beam.png` still and the numeric cut-off above.

## No other contact

There were **no** contacts between any left/right arm body and the lamp housing, the stand, the screen or the
table; no key against the stand, screen or table; no key through the lamp housing (the key's shaft is in the
open socket); no gripper, finger or forearm through the screen, the stand or the table. The beam patch is a
collision-free visual (`collisionEnabled = false` on every geom); it is not a collider and the score never
reads it. The screen and the stand are fixed, outside the arms' reach.

## Cutaway

There is no cutaway. The lamp is a complete 178 mm H4 insert; the two keys are at the rear housing; the top
camera (and, in the internal MuJoCo render, the 3.00 m screen camera) sees the outside surfaces. The beam
patch is drawn on the screen's front.

## Clearance at the end

The 1.0 hold requires both grippers at least 5 cm clear of the lamp box (0.28-0.56 x -0.24-0.02 x
0.76-0.98 m) and no grip force; the right arm's READY TCP (0.323, -0.305, 1.008) is 6.5 cm outside the box's
y edge and 2.8 cm above its z top, and the left arm's READY (0.323, +0.305, 1.008) is 28 cm outside. The
arms' final home in the run is the READY pose; the demo's clear check passes at step 433.
