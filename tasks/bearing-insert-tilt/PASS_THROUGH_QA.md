# PASS_THROUGH_QA — bearing-insert-tilt

No part passes through a solid in the demo. The pressed housing (base plate, cast boss, inclined spherical seat, seat
rim, grease port, staging pocket) is one **welded, visual-only** body with **no colliders at all**, so there is no solid
for the bearing or a finger to pass through; the only collider on the task side is the bearing's own cylindrical
collider. The mjkit importer confirms this (`housing: no mass authored and no colliders to compute one; using 10 g`).
Checked in MuJoCo (top camera plus `data.contact`) every control step of the 1.0 demo. Source:
`bearing_insert_tilt.py`, `benchmark-media-bearing-insert-tilt/demo.mp4` (top view only).

## 1. Frames checked (top view, 0/25/50/75/90/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | bearing upright in its staging pocket, spherical seat and green grease port bare, grippers at READY (tilt +8 deg, yaw +60 deg) |
| 10 | 90 | left pads close on the bearing in the pocket; it stays upright in the pocket (no colliders to interpenetrate) |
| 25 | 160 | bearing lifted clear and carried over the seat, axis tilted 8 deg to the seat axis; still above the seat mouth |
| 50 | 197 | bearing rolled 60 deg about the seat axis; index dot on the green port (index error 0 deg), held above the seat |
| 75 | 232 | bearing lowered flush into the spherical seat, pads opening, axis still on the inclined seat, grease hole on port |
| 90 | 300 | gripper lifted away, bearing seated flush and clear on the spherical seat, port registered |
| 100 | 352 | scored state: bearing flush at x 0.2505 y 0.0007 z 0.7720, tilt 0.2 deg, index 0 deg, no cap, grippers home |

The bearing's motion is a smooth declared carry: upright in the pocket, a pure 8 deg tilt over the seat, a 60 deg roll
about the inclined seat axis, then a straight descent to flush. It never crosses the housing body: every approach is
from above down the seat axis, and the tilted bearing enters along the axis of the concave seat rather than through the
cast boss. No forearm, finger or held bearing crosses the housing in any frame.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving the bearing |
|---:|---|---|
| 90 | grip the bearing | none — the bearing is gravity-free and posed by the declared carry; its collider sits inside the grasp, not at the pad tips |
| 160 | tilt the axis | none — the bearing is above the seat, clear of the housing boss |
| 197 | rotate to port | none — rotation is in place above the seat mouth |
| 232 | press flush | none — the housing is visual-only; the press is a geometric flush-depth/dwell test, not contact |
| 352 | scored state | none — the bearing is held by the declared carry state, not by contact |

There are **no unintended contacts at any sampled step**: the housing has no colliders and the bearing's collider never
touches a solid. The grip is declared (a closed gripper within 16 mm of the grasp point), so no contact evidence is
expected for this family device. Both the flush seat and the tilt/index orientation are read from the bearing's root
pose written by the task; the light press is read from a closed gripper dwelling at the flush depth.

## 3. Cap protection

The scorer has explicit caps, each holding the score at 0.5 and blocking 1.0:

* `jammed` — a bearing entering the seat more than 6 deg off the inclined seat axis (the rim, not the spherical
  surfaces, takes the load);
* `scored` — a bearing pressed down while its grease hole is more than 15 deg from the housing port (the unregistered
  hole/lug scrapes the bore);
* `forced` — a bearing driven more than 4 mm below the flush depth, or released off the bench.

These were checked in-sim with temporary bad demos (removed after the check): an upright entry (8 deg tilt, 60 deg
index error) latched `jammed` + `scored` and scored **0.25**; an 8 mm over-deep press latched `forced` and scored **0.5**;
neither reached 1.0. The 1.0 demo triggers none of them: it tilts to 0.2 deg, registers to 0.0 deg and seats at flush
within 0.5 mm, and never pushes below the seat.

## Isaac run

The same scripted demo reached **1.0 at step 352 on Isaac** (`adhoc_7e0015ad.json`): the spherical bearing was
tilted within 0.2 deg of the 8 deg seat axis, the grease-port index registered to 0.0 deg, pressed to flush within
0.5 mm and held 1 s with the grippers clear; no jammed, forced, wrong-port or drop fault. The housing remains a
welded visual-only body with no collider on Isaac as in MuJoCo. No task-local scoring or force change was needed.
It was filmed with the top camera and two render-only close-ups (`film_seat`, `film_stage`) on a copy of the task;
no finger, forearm or held bearing is seen crossing the seat rim, boss, pocket or table.

## 4. Media and cutaways

* `demo.mp4`: Isaac top camera only, 640x360, 231,974 B (< 1.87 MB), 422 frames with the final 1.0 frame held 3 s.
  No close-up or cutaway in the PR media.
* `demo_top.gif`: 480x270, 5 fps, 1,274,731 B (< 1.87 MB), same top view.
* `setup.png`: 640x360 close-in top camera at reset; the bearing in its pocket, the inclined spherical seat, the green
  grease port and the empty seat rim are legible. The overlay shows the start tilt (+8 deg) and index error (+60 deg)
  that the robot must remove.
