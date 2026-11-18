# PASS_THROUGH_QA — index-plunger-fixture

No part passes through a solid in the demo. The base (pedestal, bearing ring, plunger bracket and home mark) is one
**welded, visual-only** body with no colliders at all, so there is no solid in the fixture for a part or finger to pass
through. The two moving bodies (plate, plunger) carry colliders, as do the grippers. Checked in MuJoCo (top camera plus
`data.contact`) at every control step of the 1.0 demo. Source: `index_plunger_fixture.py`,
`benchmark-media-index-plunger-fixture/demo.mp4` (top view only).

## 1. Frames checked (top view, 0/15/25/45/60/80/95/100 % of step 356)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | pin in the home hole, plate at 0 deg, both grippers at READY |
| 15 | 54 | left pads closing across the 16 mm knob; right pads descending to the hub |
| 25 | 89 | knob gripped and pulled clear (pin retracted); right pads on the hub |
| 45 | 160 | plate turning, pin held clear; left pads still on the knob |
| 60 | 214 | plate at +45 deg (next stop), right pads opening off the hub; pin still held out |
| 80 | 285 | left pads returning the pin to the next hole and opening; plate stays at +45 deg |
| 95 | 338 | both grippers away from the fixture and homing; pin seated at the stop |
| 100 | 356 | scored state: plate +44.9 deg (0.1 deg off the stop), pull 0, grippers home and clear |

The two carried bodies stay on their declared tracks: the plunger slides axially with the left pads from the home hole
out 14-15 mm and back into the next hole; the plate's yaw follows the right wrist from 0 to +45 deg about its centre,
with the first hole ending at the plunger. No finger or held part crosses the base or the plate in any frame.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving the moving bodies |
|---:|---|---|
| 80 | grip both features | `left/tip_left`+`left/tip_right`--`plunger` (intended knob grip); `plate`--`right/tip_left`+`right/tip_right` (intended hub grip); `plate`--`plunger` (pin in the home hole) |
| 140 | pin held clear | `left/tip_left`+`left/tip_right`--`plunger` (still gripped); `plate`--`right/tip_left`+`right/tip_right` (hub grip); no plate--plunger (pin retracted) |
| 200 | plate turned | `left/tip_left`+`left/tip_right`--`plunger` only: the pin is held clear of the plate while the right arm has released the hub |
| 260 | pin released | `plate`--`plunger` only: the pin is back in the hole at the next stop; no gripper-part contact |
| 356 | scored hold | `plate`--`plunger` only: the correct seated pin/plate state; **both grippers home, no contact with either body** |

Every sustained contact is either an intended pad-on-feature grip or the correct pin-in-hole contact. There are no
fixture colliders, so no unintended solid penetration is possible in the fixture. At step 80, while both arms are
simultaneously descending onto the knob and the hub, the forearms (`left/link4`--`right/link4`) brush once; it is
transient (gone by step 140) and does not displace either grip from its feature — the pull, turn and release still
complete to 1.0. The gap between the knob (y 0.150) and the hub (y 0) keeps the sustained grips clear.

## 3. Failure-cap protection

The scorer has three explicit caps, each holding the score at 0.5 and blocking 1.0: a plate turned more than one pitch
+ 12 deg sets `overshoot`; a pin released with the plate more than 8 deg off any of the 8 stops sets `miss_hole`; a
plate turned more than 8 deg while the pin is still engaged (pull < 12 mm) sets `forced`. The offline suite exercises
all three. The demo triggers none: final plate +44.9 deg, hole error -0.1 deg, pin pulled 14.5 mm clear during the
turn, released at the stop.

## Isaac run

The same scripted demo reached **1.0 at step 356 on Isaac** (`adhoc_835e33d4.json`): the plunger was pulled 14.5 mm
clear, the plate turned one 45 deg stop to +44.0 deg (hole error -1.0 deg) and the pin released into the hole, then
the hands cleared for the 1 s hold; no overshoot, miss-hole or forced fault. The welded visual-only base remains
collider-free on Isaac; plate, plunger and grippers keep their colliders. No task-local scoring or force change was
needed. It was filmed with the top camera and two render-only close-ups (`film_plate`, `film_plunger`) on a copy of
the task; no finger, forearm or moving part is seen crossing the base, bracket or table.

## 4. Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 277,316 B (< 1.87 MB), 426 frames with the final 1.0 frame held 3 s.
  No close-up or cutaway in the PR media.
- `demo_top.gif`: 480x270, 5 fps, 1,302,922 B (< 1.87 MB), same top view.
- `setup.png`: 640x360, 29,648 B, Isaac top camera at reset.
