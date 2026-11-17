# PASS_THROUGH_QA — check-valve-rebuild

No part passes through a solid in the demo. The valve fixture, brass body, seat and buttons are one **welded,
visual-only** body with no colliders at all, so there is no solid for a part or finger to pass through; the only
colliders are the three parts and the two robot grippers. The contact table below is MuJoCo (top camera plus
`data.contact`, every control step). The same scripted demo was run on Isaac and reached **1.0 at step 1092** with
the same welded, collider-free body and the same three real part bodies; the Isaac evidence is the top camera and
two render-only close-ups (`film_parts`, `film_cap`) saved on a filming copy. Source: `check_valve_rebuild.py`,
`benchmark-media-check-valve-rebuild/demo.mp4` (Isaac top view only).

## 1. Frames checked (top view, 0/10/25/45/60/80/95/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 0 | cap seated on the body, poppet and spring in their pockets, grippers at READY |
| 10 | 110 | left pads close on the cap rim; cap still seated, no overlap |
| 25 | 280 | cap parked on the peg, left pads close on the poppet in its pocket |
| 45 | 490 | poppet seated on the seat; pads close on the spring in its pocket |
| 60 | 620 | spring seated on the poppet; cap back on the peg being picked up |
| 80 | 850 | cap twisted and lowered onto the body; internals stacked under it |
| 95 | 1000 | two button presses; left gripper over the outlet button, no overlap with the valve |
| 100 | 1092 | scored state: cap threaded, parts stacked seated, grippers home and clear |

The carried parts stay on their declared tracks: each follows the pads from pocket to seat with no jump, the poppet
is upright (rubber seal down) at the seat, the spring is coaxial on its stem, and the cap stays over the bore while
it twists. No forearm, finger or held part crosses the fixture or the body in any frame.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving the loose parts |
|---:|---|---|
| 80 | grip the cap | `cap/CapHex--left/tip_left` (2), `cap/CapHex--left/tip_right` (2) — intended knob grip |
| 140 | park the cap | same two intended cap-pad pairs |
| 280 | grip the poppet | `poppet/Disc--left/tip_left` (2), `--left/tip_right` (2) — intended disc grip |
| 360 | lower the poppet | same two intended pairs; poppet on the seat, no body contact (body has no collider) |
| 480 | grip the spring | `spring/Coil--left/tip_left` (2), `--left/tip_right` (2) — intended spring grip |
| 560 | lower the spring | same two intended pairs |
| 720 | thread the cap | `cap/CapHex--left/tip_left` (2), `--left/tip_right` (2) — intended cap grip |
| 880 | cap seated | `cap--spring` and `poppet--spring` — the real seated stack, plus cap-pad |
| 980 | flow tests | `cap--spring`, `poppet--spring` — stack intact; cap-gripper clearing |
| 1092 | scored state | `cap--spring` only — the correct cap/spring/poppet stack; **no gripper-part contact** |

Every contact is either an intended pad-on-part grip or the correct part-on-part stack (cap on spring on poppet on
seat). There are no body/fixture colliders, so no unintended solid penetration is possible. Both flow-button presses
are geometric TCP tests; the gripper passes the visual-only button without a spurious contact.

## Isaac run

Isaac demo: **1.0 at step 1092** (`adhoc_8bc9dd91.json`): body opened, poppet seated seal-face-down, spring fitted,
cap threaded home past 45 degrees, both flow-button tests passed, parts in place, grippers clear, 1 s still. The
top view and both close-ups were reviewed: no gripper, finger, forearm or part crosses a solid; the fixture remains
a welded visual-only body with no collider on either backend. The only task-local Isaac change is
`disable_reset_pose()` on the four USD objects (the task writes their root poses itself; Arena's float64 reset
pose is rejected by PhysX's transform kernel). Scoring, geometry, parts and forces are unchanged.

## 3. Pinch / cross-thread protection

The scorer has explicit caps: a cap seated with less than 45 degrees of twist sets `cross_thread`; a poppet seated
flipped sets `reversed_poppet`; a part released outside the bench zone sets `dropped`; a button press with the wrong
internals sets `leak`. Each holds the score at 0.5 and blocks 1.0. The demo triggers none of them (final cap twist
70 degrees, poppet upright, all parts in place, both pressure tests passed).

## 4. Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 617,004 bytes (< 1.87 MB), 1,162 frames with the final 1.0 frame
  held 3 s. No close-up or cutaway views in the PR media.
- `demo_top.gif`: 480x270, 4 fps, 1,755,786 bytes (< 1.87 MB), same top view.
- `setup.png`: 640x360, 28,184 bytes, Isaac top camera at reset.
