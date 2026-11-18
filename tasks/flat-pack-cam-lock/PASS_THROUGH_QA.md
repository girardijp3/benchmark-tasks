# PASS_THROUGH_QA — flat-pack-cam-lock

No part passes through a solid in the demo. The two 16 mm panels (base face panel and sliding mating panel), the 15 mm
cam bore and the 8 mm bolt hole are one welded, **visual-only** fixture with **no colliders at all**, so there is no
solid panel for the cam, bolt, screwdriver or a finger to pass through. The mjkit importer confirms this:
`fixture: no mass authored and no colliders to compute one; using 10 g` and the same for `panel_b` (which is free,
gravity-off, and posed by the declared draw so it can slide). The only colliders on the task side are the cam's, bolt's
and screwdriver's own simple cylinders. Checked in MuJoCo (top camera plus `data.contact`) every control step of the
1.0 demo. Source: `flat_pack_cam_lock.py`,
`benchmark-media-flat-pack-cam-lock/demo.mp4` (top view only).

## 1. Frames checked (top view, 0/25/50/75/90/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | cam upright in its pocket, bolt on its pad, screwdriver in its holder; both panels 3 mm apart, bore and hole bare, grippers at READY |
| 25 | 295 | cam seated upright in the 15 mm bore (green edge arrow visible); bolt just released square in the mating panel's 8 mm hole, its 7 mm head at the cam; screwdriver still in its holder |
| 50 | 500 | screwdriver tip engaged in the cam recess; cam turned ~33 deg, panel gap still 3.0 mm; all approaches from above |
| 75 | 661 | cam turned 150 deg, wrist straightened; the eccentric has drawn the mating panel in so the gap is ~1 mm; no panel overlap |
| 90 | 667 | after the regrip the cam has turned 178 deg, the gap is 0.1 mm and the panels are flush; the scored step latches here |
| 100 | 865 | scored state: cam centred in its bore within 0.8 mm, bolt within 6 mm of the hole centre, turn +179.5 deg, gap 0.0 mm, no cap, both grippers home |

The cam's motion is a straight carried descent into its bore; the bolt is carried above the bench and lowered into the
edge hole along +y with a near-vertical wrist; the screwdriver descends onto the recess and turns about the vertical.
The panel gap closes monotonically from 3.0 mm to 0.0 mm as the cam turns past 90 deg. No forearm, finger or held part
crosses either panel in any frame, and the panels meet face to face rather than overlapping.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving task bodies |
|---:|---|---|
| 1 | reset | `screwdriver`–`world` only — the screwdriver rests in its bench holder; no other task contact |
| 127 | cam seated | `screwdriver`–`world` only; the cam is posed in its bore, which has no collider, so there is no contact |
| 295 | bolt inserted | `bolt`–`left/tip_right` (the declared grip at the fingertip) and `screwdriver`–`world`; no panel or bore contact |
| 500 | cam turning | `bolt`–`cam` (the bolt head sitting in the cam's slot — the actual joint, expected) and nothing else |
| 661 | 150 deg / regrip | `bolt`–`cam` only |
| 667 | flush + torque | `bolt`–`cam` only |
| 865 | scored state | `bolt`–`cam` and `screwdriver`–`world` (the screwdriver set back in its holder) |

There are **no unintended contacts through a solid**: the fixture and both panels have no colliders, so nothing can
penetrate them. The `bolt`–`cam` contact is the intended head-in-slot engagement, not a pass-through; both bodies are
posed by the declared mechanism each step, so the contact never drives the pose or accumulates penetration. The grip is
declared (a closed gripper within 11 mm of a loose part's grasp point), so `bolt`–fingertip contact is expected. Both
the flush gap and the cam turn are read from the poses written by the task, not from contact forces.

## 3. Cap protection

The scorer has explicit caps, each holding the score at 0.5 and blocking 1.0:

* `cap_cam` — the screwdriver turns while the cam is not centred and upright in its bore (the cam cracks the bore);
* `cap_bolt` — the bolt comes within 12 mm of the hole more than 20 deg off the hole axis (cross-threaded);
* `cap_panel` — the panels are pushed flush before the bolt is engaged and the cam has turned (forced shut), or the cam
  is turned past 215 deg (stripped).

These were checked in-sim with **temporary bad demos** (separate copies of the task directory, removed after the run):
leaving the cam in its pocket latched `cap_cam` and scored **0.25**; driving the bolt 45 deg off the hole axis latched
`cap_bolt` and scored **0.25**; forcing the mating panel flush before the turn latched `cap_panel` and held the score at
**0.5**. None reached 1.0. The 1.0 demo triggers none of them: it seats the cam within 0.8 mm, squares the bolt within
6 mm, turns +179.5 deg and never forces the panels.

## 4. Media and cutaways

* `demo.mp4`: Isaac top camera only, 640x360, 491,268 B (< 1.87 MB), 935 frames with the final 1.0 frame held 3 s.
  No close-up or cutaway in the PR media.
* `demo_top.gif`: 480x270, 4 fps, 1,706,626 B (< 1.87 MB), same top view.
* `setup.png`: 640x360 close-in top camera at reset; the empty 15 mm bore and green arrow, the 8 mm bolt hole and red
  mark, the 3 mm panel gap, the staged cam/bolt and the screwdriver in its holder are all legible. The overlay shows
  the four ladder steps and the starting turn (0 deg) and gap (3.0 mm) the robot must remove.


## Isaac run

The scripted demo reached **1.0 at step 865 on Isaac** (`adhoc_10b21daa.json`): the 15 mm cam seated within 0.6 mm,
the M6 bolt squared within 1 mm and turned +177.6 deg, the panels closed to a 0.1 mm gap, the declared torque gate
passed and the grippers cleared for the hold; no mis-seat, cross-thread, forced-panel or drop fault. The panels and
bore stay welded/posed visual bodies with no collider on Isaac; the cam, bolt and screwdriver keep their simple
cylinder colliders. No task-local scoring or force change was needed. It was filmed with the top camera and two
render-only close-ups (`film_cam`, `film_bolt`) on a copy of the task; no finger, forearm or tool is seen crossing the
panels, bore or table.
