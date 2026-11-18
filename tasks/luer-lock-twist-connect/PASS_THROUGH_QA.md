# PASS_THROUGH_QA — luer-lock-twist-connect

No part passes through a solid in the demo. The bench fixture, vial, adapter skirt and female port are one **welded,
visual-only** body with no colliders at all, so there is no solid for the syringe or a finger to pass through; the only
colliders are the syringe barrel and the two robot grippers. The contact table below is MuJoCo (top camera plus
`data.contact`, every control step). The same scripted demo transfers to Isaac with the same welded, collider-free
fixture and the same single free syringe body; the only task-local Isaac change is `disable_reset_pose()` on the two
USD objects (the task writes their root poses itself).

## 1. Frames checked (top view, 0/10/25/45/60/80/95/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | syringe upright in its holder, adapter port empty, both grippers at READY |
| 10 | 70 | left pads close on the staged 20 mm barrel; no overlap with the holder |
| 25 | 176 | syringe lifted clear and carried over the adapter, tip above the port |
| 45 | 317 | first half-turn complete, syringe seated and released, wrist winding back clear |
| 60 | 422 | second half-turn: pads across the barrel, tip still down in the port |
| 80 | 563 | third half-turn under way, cumulative twist approaching the lock window |
| 95 | 669 | locked at 1.49 turns, syringe released, left gripper retiring, index mark turned |
| 100 | 704 | scored state: syringe seated and free, both grippers home and clear, 1 s hold done |

The syringe stays on its declared track: it follows the pads from the holder to the port with no jump, the tip is
coaxial in the female port throughout all three twists, and the barrel stays upright. No forearm, finger or held
syringe crosses the fixture, the vial or the adapter in any frame. The red index mark on the thumb pad rotates with
the barrel, so the cumulative twist is visible in the film.

## 2. Contact evidence (`data.contact`, MuJoCo)

Geom 673 is the syringe barrel collider; 253/254/290/293/295 are the left gripper's pad/finger geoms (the imported
model exposes geom ids, not names).

| step | phase | contacts involving the syringe |
|---:|---|---|
| 1 | reset | none — nothing held, both grippers at READY |
| 110 | close on the staged syringe | none yet — pads approaching, not yet pressing |
| 220 | seat the tip | `673--253` (2), `673--254`, `673--290` (2), `673--293` — intended barrel grasp |
| 260 | first twist | same intended barrel-pad pairs; tip seated, no fixture contact (fixture has no collider) |
| 300 | released between grips | none — syringe seated and static, gripper clear |
| 370 | regrip for the second twist | `673--253`, `673--254` (2), `673--290`, `673--293`, `673--295` — intended regrasp |
| 520 | third twist | `673--253` (2), `673--254`, `673--290` (2), `673--293` — intended barrel grasp |
| 590 | locked, still held | same intended barrel-pad pairs, cumulative twist 522 deg |
| 700 | scored state | none — both grippers home, syringe seated and free |

Every contact is an intended pad-on-barrel grip. There are no fixture/vial/adapter colliders, so no unintended solid
penetration is possible. The seat and twist are geometric TCP/pose tests in the task file, not contact forces.

## 3. Cross-thread / loose / drop protection

The scorer has explicit caps: turning more than 25 deg while the tip is off the port sets `cross_thread`; a clear,
finished state with the tip seated but no lock (twist below the 500-580 deg window) is the loose-connection cap;
releasing the syringe outside the bench zone sets `dropped`. Each caps the score at 0.5 and blocks 1.0. The demo
triggers none of them: all three twists happen with the tip seated, the final twist is 538 deg (1.49 turns) inside the
lock window, and the syringe is released on the port with both grippers going home.

## 4. Media

- `demo.mp4`: Isaac top camera only, 640x360, 396,076 bytes (< 1.87 MB), 774 frames with the final 1.0 frame held
  3 s. No close-up or cutaway views in the PR media.
- `demo_top.gif`: 480x270, 4 fps, 1,635,867 bytes (< 1.87 MB), same top view.
- `setup.png`: 640x360, 28,088 bytes, Isaac top camera at reset.

## Isaac result

The scripted demo reached **1.0 at step 704 on Isaac** (`adhoc_e4234126.json`), the same step as MuJoCo: the syringe
was picked up by its 20 mm barrel, the tip seated in the female port, three 180 deg regrips took the twist to 538 deg
inside the 500-580 deg lock window with the tip seated throughout, and the syringe was released on the port with both
grippers going home for the 1 s hold; no cross-thread, loose-connection or drop fault. The welded visual-only fixture
stays collider-free, and the only task-local Isaac change is `disable_reset_pose()` on the adapter and syringe (the
task writes their root poses itself). It was filmed with the top camera and two render-only close-ups (`film_port`,
`film_stage`) on a copy of the task; no arm, finger or carried syringe is seen crossing the vial, port or bench.
