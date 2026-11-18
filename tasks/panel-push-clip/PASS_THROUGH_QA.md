# PASS_THROUGH_QA — panel-push-clip

No part passes through a solid in the demo. The panel and tray are **welded / visual-only bodies with no colliders at
all** (the same device as the family's fixture bodies), and the clip is a **gravity-free rigid body with no colliders**;
the only colliders in the task are the robot arms and grippers. There is therefore no solid task body for a clip, a stem,
a barb or a finger to pass through. Checked in MuJoCo (top camera plus the declared-carry and seat tracks) on every
control step of the 1.0 demo. Source: `panel_push_clip.py` and
`benchmark-media-panel-push-clip/demo.mp4` (top view only).

## 1. Frames checked (top view, 0/25/45/60/85/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | clip upright in the tray at x 0.200 y 0.090; panel hole clear at x 0.280 y 0; both grippers READY |
| 25 | 130 | right pads closed on the clip cap; clip carried toward the hole, still upright, no overlap with the panel |
| 45 | 205 | clip stem centred over the hole, cap ~5.7 mm (5.0 N insertion), barbs at the hole; cap not through the plate |
| 60 | 240 | cap ~4.7 mm, barbs seated behind the panel, cap flush within tolerance; gripper released, clip locked in the panel |
| 85 | 340 | gripper re-grips the seated cap and pulls up 16 mm (5.0 N); clip stays at its seat (barbs hold), panel unchanged |
| 100 | 423 | scored state: clip seated and intact, 5.4 N insertion / 5.0 N pull recorded, no cap, both grippers clear and home, 1 s pull hold complete |

The clip stays on its declared carry track: it follows the pads from the tray to the hole with no jump, descends
straight down through the hole (tilt stays below 1 degree), locks at the seat when the barbs pass the panel, and the
pull is a straight vertical rise of the gripper against the now-fixed clip. Only the right arm reaches the work area,
so no forearm, finger or held clip crosses the left arm in any frame.

## 2. Position / force evidence (MuJoCo)

The panel, tray and clip carry no `CollisionAPI`, so they appear in no contact pair. The only solver contacts are the
intended gripper-on-gripper/arm contacts of the station and the gripper pads closing at the cap; there is no task-object
contact pair that could indicate a pass-through. The carried clip is posed by the mechanism from the gripper TCP each
step, so its position is always exactly `tcp + rot @ offset` — a continuous, jump-free track with no penetration of a
solid (there is no solid task body to penetrate). Once seated, the clip pose is held fixed at the seat during the pull.

| step | phase | position / force evidence |
|---:|---|---|
| 70-90 | grip the cap | pads closed around the clip cap at its tray spot; no panel contact (panel has no collider) |
| 128-150 | align | cap 24.4 mm, off-axis 2.1 mm, tilt 0.2 deg; aligned milestone latched |
| 172 | press | cap 5.7 mm, insertion 5.0 N inside 4-7 N; pressed milestone latched |
| 203 | seat | cap 4.7 mm (inside 2.5 mm of the 3 mm target), tilt 0.1 deg, insertion 5.4 N; barbs-seated milestone latched |
| 337 | pull verified | gripper 16 mm above the seated cap (5.0 N pull) for a full 1 s; clip pose unchanged |
| 423 | scored state | clip seated, both grippers home and clear, caps cross/unseat/drop/force all 0 |

## 3. Damage-cap protection

The scorer has explicit caps: a press tilted past 12 degrees sets `cross_insert`; releasing a clip in the hole before
it seats sets `not_seated`; a clip dropped off the bench sets `dropped`; a press over 7 N (or a pull that yanks the clip
past its retention) sets `forced_fit`/`not_seated`. Each holds the score at 0.5 and blocks 1.0. The demo triggers none:
tilt is at most 0.5 degrees, the press peaks at 5.4 N and the pull at 5.0 N, and the clip is released only after seating.

## 4. Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 255,891 B (< 1.87 MB), 493 frames with the final 1.0 frame held 3 s.
- `demo_top.gif`: 480x270, 4 fps, 916,282 B (< 1.87 MB), same top view.
- `setup.png`: 640x360, 27,710 B, Isaac top camera at reset.


## Isaac run

The scripted demo reached **1.0 at step 423 on Isaac** (`adhoc_bc55c9f7.json`): the clip was aligned to the panel hole
within 0.2 mm / 0.1 deg, pressed with the declared 5.4 N until the mounting ears seated flush, released only after
seating, then pull-tested at 5.0 N for the 1 s hold; no cross-insert, not-seated, drop or forced-fit cap fired. The
panel/tray remain welded visual-only bodies with no collider on Isaac; the clip remains a gravity-free posed body.
No task-local scoring or force change was needed. It was filmed with the top camera and two render-only close-ups
(`film_hole`, `film_stage`) on a copy of the task; no finger, forearm or clip is seen crossing the panel, tray or table.
