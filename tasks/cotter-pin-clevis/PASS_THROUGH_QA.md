# PASS_THROUGH_QA — cotter-pin-clevis

No part passes through a solid in the demo. The clevis fixture (bench plate, stand, fork prongs, rod end, clevis pin,
cross-hole, red pull tab and white cotter holder) is one **welded, visual-only** body with **no colliders at all**, so
there is no task-side solid for the cotter or a finger to pass through; the only colliders on the task side are the
cotter body's and the two legs' own small cylindrical colliders. The mjkit importer confirms this
(`clevis: no mass authored and no colliders to compute one; using 10 g`). Checked in MuJoCo (top camera plus
`data.contact`) at ten sampled control steps of the 1.0 demo, and on every captured film frame. Source:
`cotter_pin_clevis.py`, `benchmark-media-cotter-pin-clevis/demo.mp4` (top view only).

## 1. Frames checked (top view, 0/25/50/75/90/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | straight 3 mm cotter lying in the white holder, green and blue leg tips together, clevis cross-hole bare, red pull tab clear, both grippers at READY (score 0) |
| 10 | 136 | left pads close on the eye/shank and lift the cotter clear; cotter axis aligned with the +x cross-hole (step rung 1, score 0.25) |
| 25 | 183 | cotter pushed fully through the cross-hole, eye band at the pin's near face, legs protruding past the far face (rung 2, score 0.5) |
| 50 | 300 | cotter released and seated, gripper lifting away; legs still straight and together |
| 75 | 448 | gripper closes on the long leg's green tip and sweeps it out to the spread mark; short leg still straight (rung 3, score 0.75) |
| 90 | 549 | both legs spread to opposite sides (+57.9 / +55.2 deg), gripper at the red pull tab, joint intact (rung 4, score 1.0) |
| 100 | 688 | scored state: cotter centre x 0.2449 y -0.0553 z 0.8038, axis 0.2 deg, both legs spread, no cap, both grippers home |

The cotter's motion is a smooth declared carry: straight in the holder, lifted and aligned, pushed straight down the
cross-hole axis, released, then two outward leg sweeps and an outward pull at the tab. It never crosses the fixture:
every approach is from above and the insertion follows the axis of the cross-hole. No forearm, finger or held cotter
crosses the fixture in any film frame.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving the cotter or either leg |
|---:|---|---|
| 1 | reset | none |
| 90 | close on the cotter | none — the cotter is gravity-free and posed by the declared carry; its collider sits inside the grasp, not at the pad tips |
| 136 | aligned over the hole | none |
| 183 | pushed through | none — the clevis/cross-hole is visual-only; insertion is a geometric pose test, not contact |
| 300 | released and seated | none |
| 448 | spread the long leg | none — the leg bend is a declared rotation about its root, not contact with the pin |
| 500 | spread the short leg | none |
| 549 | pull test holds | none — the pull is a geometric drag at the tab; no fixture colliders |
| 688 | scored state | none |

At **all ten sampled steps** the total contact count in the whole scene was **0**, so there were **no unintended
contacts at any sampled step**: the fixture has no colliders and the cotter/leg colliders never touch a solid. The grip
and the leg bend are declared (a closed gripper at the grasp point / leg tip), so no contact evidence is expected for
this family device. The insertion depth, axis angle, leg angles and pull displacement are read from the poses written
by the task each step.

## 3. Cap protection (in-sim bad demos)

The scorer has explicit caps, each blocking 1.0. Temporary bad demos (removed after the check) were run in MuJoCo:

| Bad demo | Latched cap | Measured score / steps |
|---|---|---|
| pull the tab with the legs left straight | `not_spread` | **0.5** (no success, 490 steps); bend long/short only +1.1/+0.0 deg when the pull latched |
| pull the tab with the cotter still in the holder | `not_through` | latched at t 11.95 s; align/through/spread all 0, so 0.0 overall |
| install correctly, then carry the cotter to y 0.205 (off the bench) and release | `dropped` | **0.5** (no success, 825 steps); cotter at x 0.2500 y 0.2047 z 0.7885 when the cap latched |

None reached 1.0. The 1.0 demo triggers none of them: it aligns to 0.2 deg, inserts to within 0.6 mm of the hole, bends
both legs past 55 deg to opposite sides, pulls 8 mm and stays seated and home.

## 4. Media and cutaways

* `demo.mp4`: Isaac top camera only, 640x360, 400,417 B (< 1.87 MB), 758 frames with the final 1.0 frame held 3 s.
  No close-up or cutaway in the PR media.
* `demo_top.gif`: 480x270, 4 fps, 1,679,751 B (< 1.87 MB), same top view.
* `setup.png`: 640x360, 27,866 B, Isaac top camera at reset.


## Isaac run

The scripted demo reached **1.0 at step 688 on Isaac** (`adhoc_db58460c.json`): the cotter was aligned to 0.2 deg,
pushed through the 3.2 mm cross-hole, both legs bent past 55 deg to opposite sides, the joint pulled 8 mm without
backing out and the hands went home for the 1 s hold; no not-through, not-spread or dropped latch fired. The welded
visual-only clevis fixture stays collider-free on Isaac; only the cotter body and its two legs carry colliders. A
task-local demo re-tune aims the leg sweep 4 deg past the declared spread mark (the arm tracking otherwise landed
~0.5 deg short of the 55 deg gate); the gate itself and all forces/geometry are unchanged. It was filmed with the
top camera and two render-only close-ups (`film_clevis`, `film_stage`) on a copy of the task; no finger, forearm or
cotter is seen crossing the clevis prongs, pin, pull tab or holder.
