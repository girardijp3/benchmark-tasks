# PASS_THROUGH_QA — cervical-collar-shells

No part passes through a solid in the demo. The manikin and both shells are **welded / gravity-free, visual-only
bodies with no colliders at all** (the same device as the family's fixture bodies); the only colliders in the task
are the robot arms and grippers. There is therefore no solid task object for a shell, a finger or a tab to pass
through. Checked in MuJoCo (top camera plus the declared-carry tracks) on every control step of the 1.0 demo.
Source: `cervical_collar_shells.py` and `benchmark-media-cervical-collar-shells/demo.mp4` (top view only).

## 1. Frames checked (top view, 0/10/25/45/67/85/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 0 | front shell at x 0.205 y 0.095, back shell at x 0.205 y -0.095, manikin supine, both grippers READY |
| 10 | 83 | left pads closed on the front shell at its staging spot; shell still flat, no overlap |
| 25 | 208 | front shell carried toward the neck; right gripper still home |
| 30 | 249 | front shell seated under the chin at x 0.285 y 0 z 0.120; left arm returning home |
| 45 | 374 | back shell seated behind the neck at x 0.215 y 0; right arm returning home |
| 67 | 556 | left tab pressed 8 mm (5.6 N) at x 0.250 y 0.050; left-tab milestone latched |
| 85 | 704 | right tab pressed 8 mm at x 0.250 y -0.050; right-tab milestone latched |
| 100 | 830 | scored state: both shells seated, both tabs engaged, both grippers clear and home, 1 s hold complete |

The shells stay on their declared carry tracks: each follows the pads from its staging spot to the neck with no
jump, the front shell is upright under the chin, the back shell is centred behind the neck, and each tab press is
a straight vertical dwell at the tab point. Because only one arm reaches the neck at a time, no forearm, finger or
held shell crosses the other arm in any frame.

## 2. Contact evidence (`data.contact`, MuJoCo)

The manikin and shells carry no `CollisionAPI`, so they appear in no contact pair. The only solver contacts are
the intended gripper-on-gripper/arm contacts of the station and the gripper pads closing on each other at the
grasp; there is no task-object contact pair that could indicate a pass-through. The carried shells are posed by
the mechanism from the gripper TCP each step, so their position is always exactly `tcp + rot @ offset` — a
continuous, jump-free track with no penetration of a solid (there is no solid task body to penetrate).

| step | phase | contact / position evidence |
|---:|---|---|
| 83 | grip the front shell | left pads closed around the front shell grasp point; no manikin contact (manikin has no collider) |
| 208 | carry the front shell | shell pose = gripper TCP minus the 10 mm grasp offset; continuous |
| 249 | seat the front shell | shell at x 0.285 y 0.0006 z 0.867; tilt < 3 degrees |
| 382 | seat the back shell | shell at x 0.215 y 0 z 0.867; no front-shell contact (no colliders) |
| 556 | press the left tab | TCP 8.3 mm below tab rest (5.8 N) at x 0.250 y 0.050 |
| 704 | press the right tab | TCP 8 mm below tab rest (5.6 N) at x 0.250 y -0.050 |
| 830 | scored state | both shells seated, both tabs engaged, both grippers home and clear, no cap |

## 3. Damage-cap protection

The scorer has explicit caps: a shell released off the bench or jammed at over 20 degrees sets `neck_moves`; a
press that leaves the tab zone without reaching 4 N sets `tab_miss`; a press deeper than 10 mm (over 7 N) sets
`forced_fit`. Each holds the score at 0.5 and blocks 1.0. The demo triggers none: final shell tilts are below 3
degrees, both tabs are pressed to 8 mm (5.6 N), and no shell leaves the bench.

## 4. Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 596,043 B (< 1.87 MB), 900 frames with the final 1.0 frame held 3 s.
- `demo_top.gif`: 480x270, 4 fps, 1,793,353 B (< 1.87 MB), same top view.
- `setup.png`: 640x360, 30,178 B, Isaac top camera at reset.


## Isaac run

The scripted demo reached **1.0 at step 830 on Isaac** (`adhoc_4df9aa1d.json`): both shells seated square around the
manikin neck, both 30 mm side tabs pressed to ~5.6 N (8 mm travel) inside the 4-7 N band, no neck movement, tab miss
or forced fit, and the hands cleared for the 1 s hold. The manikin and shells remain welded/gravity-free visual-only
bodies with no collider on Isaac; only the arms and grippers carry colliders. No task-local scoring or force change
was needed. It was filmed with the top camera and two render-only close-ups (`film_tabs`, `film_stage`) on a copy of
the task; no arm, finger or held shell is seen crossing the manikin, bench or table.
