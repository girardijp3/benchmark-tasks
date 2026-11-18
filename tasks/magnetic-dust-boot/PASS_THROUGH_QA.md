# PASS_THROUGH_QA — magnetic-dust-boot

No part passes through a solid in the demo. The spindle fixture (base, shoulder boss and 65 mm nose) and the boot's
80 mm elastomer body, bore and magnet ring are **welded, visual-only** geometry with no colliders, so there is no
solid for a finger or the boot body to pass through; the only colliders in the task are the boot's small central
**GraspBoss** and the robot's own pads. Checked in MuJoCo (top camera plus `data.contact`) at every control step of
the 1.0 demo. Source: `magnetic_dust_boot.py`, `benchmark-media-magnetic-dust-boot/demo.mp4` (top view only).

## 1. Frames checked (top view, 0/10/25/45/60/80/95/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | boot in its staging pocket (black elastomer body, magnet ring, yellow alignment tick), spindle nose bare, grippers at READY |
| 10 | 35 | left pads approach the staged boot from above; no overlap with the spindle fixture |
| 25 | 88 | left pads closed on the boot's central grasp boss; the boot is still in the pocket |
| 45 | 159 | boot carried at z 0.150 toward the spindle, clear of the 65 mm nose; pads still on the boss |
| 60 | 212 | boot centred over the nose, pads pressing it down through the stroke |
| 80 | 283 | boot released on the shoulder, pads lifting away; nose centre and boot bore concentric |
| 95 | 336 | both grippers returning home; boot still seated on the shoulder |
| 100 | 354 | scored state: boot seated against the shoulder, centred, both grippers home and clear |

The carried boot stays on its declared track: it follows the pads from the pocket to the spindle, sits centred over
the nose (bore centre within 2.2 mm of the spindle axis), its bottom face stops on the shoulder (0.0504 m vs
0.050 m), and the release leaves it there. No forearm, finger or held part crosses the spindle body in any frame.

## 2. Contact evidence (`data.contact`, MuJoCo, full 1.0 run)

| step | phase | contacts |
|---:|---|---|
| 145 | carrying the boot | `boot/GraspBoss7 -- left/tip_right/col5`, `boot/GraspBoss7 -- left/tip_right/col8` — intended pad-on-boss grip |
| 159–160 | carrying toward the spindle | same two intended pairs |
| 212 | pressing on the nose | none — the seat is a geometric pose check; no spindle collider |
| 283 | released on the shoulder | none — the boot rests by its declared seated pose |
| 354 | scored state | none — boot seated, grippers home and clear |

Every contact is the intended pad-on-grasp-boss grip. The spindle, its nose and the boot body have no colliders, so
an unintended solid penetration is impossible; the concentricity, press-depth, seat and overpress tests are
geometric TCP/pose checks. A "forced press" is precisely driving the boot bottom >4 mm through the shoulder stop,
which is capped rather than allowed to pass through.

## 3. Pinch / wrong-state protection

The scorer has three explicit caps, all latched and all holding the score at 0.5 while blocking 1.0:

* **off_centre** — a press is applied while the bore centre is outside 6 mm of the spindle axis, or the boot is
  released off-centre in the spindle zone (the magnets grabbed an off-centre bite of the rim).
* **not_seated** — the boot is released with its bottom face more than 5 mm above the shoulder (shallow push or
  premature release).
* **forced_press** — a closed-grip press drives the boot bottom more than 4 mm past the shoulder (jamming it through
  the solid stop).

The demo triggers none: the boot is centred before the press (dxy ~0.002 m), the press reaches 100 % of its stroke,
the bottom stops 0.0004 m from the shoulder, and the release happens only after the seat.

## Isaac run

The same scripted demo reached **1.0 at step 354 on Isaac** (`adhoc_14d8794a.json`): the 80 mm boot was centred on
the 65 mm nose (dxy 0.0003 m), pressed 100 % of its stroke until the bottom was 0.0003 m from the shoulder, and
the grippers cleared for the 1 s hold; no forced press, off-centre or drop fault. The task-local Isaac change was
`disable_reset_pose()` on the spindle and boot (the task writes their root poses itself; Arena's float64 reset pose
is rejected by PhysX's transform kernel). The welded visual-only body, real GraspBoss collider, forces and scoring
are unchanged. It was filmed with the top camera and two render-only close-ups (`film_seat`, `film_stage`) on a
copy of the task; no finger, forearm or held boot is seen crossing the nose, shoulder, bench or table.

## 4. Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 239,165 bytes (< 1.87 MB), 424 frames with the final 1.0 frame held
  3 s. No close-up or cutaway in the PR media.
- `demo_top.gif`: 480x270, 5 fps, 1,304,623 bytes (< 1.87 MB), same top view.
- `setup.png`: 640x360, 28,870 bytes, Isaac top camera at reset.
