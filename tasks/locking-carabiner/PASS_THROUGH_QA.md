# PASS_THROUGH_QA — locking-carabiner

No part passes through a solid in the demo. The evidence below is from the scripted MuJoCo demo (top camera plus
`data.contact` at every logged control step). The only colliders are the carabiner body, its gate, the fixed anchor eye
(base + post + ring) and the two robot grippers; the knurled sleeve's notches and the anchor's ring are visual-only.
The same task file transfers to Isaac with the same two USD objects (one articulation, one kinematic fixture); the
gate is a real revolute joint in both, and the sleeve twist is declared identically.

## 1. Frames checked (top view, 0/10/25/45/60/80/95/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | carabiner flat on the base, anchor eye empty, both grippers at READY; contact `anchor--carabiner` |
| 10 | ~70 | right pads close across the 60 mm carabiner's spine; no overlap with the post |
| 25 | 205 | gate pressed open against the post (gate about -12 deg), post entering the loop; rung 1 done |
| 35 | 206 | post's axis inside the loop polygon; rung 2 done |
| 45 | 240 | gate shut behind the post (gate 0 deg), carabiner held, rung 3 done |
| 55 | 300 | carabiner released, resting flat on the base with the post through the loop; contact `anchor--carabiner` |
| 60 | 470 | right wrist turning above the sleeve (0.28 turns), carabiner still on the post — not dragged |
| 80 | 556 | sleeve LOCKED, carabiner regripped by the spine and pulled; rung 4 done, score 1.0 |
| 100 | 597 | pull-test held 1 s: eye at -46 mm in the loop against the shut gate, sleeve locked, no caps |

The carabiner stays on its declared track: it is lifted clear of the base, carried until the gate meets the post, the
post passes inside the loop while the gate is open, and the gate snaps shut behind it. The post stays inside the loop
from step 206 to the end. During the declared sleeve turn the gripper hovers 18 mm above the sleeve, so it does not
push the carabiner off the post (the post-local coordinates stay at about -8,-28 mm through steps 300-470). No forearm,
finger or held carabiner crosses the anchor's ring or the table edge in any frame.

## 2. Contact evidence (`data.contact`, MuJoCo)

Bodies: `carabiner` (the body), `carabiner/Gate` (the hinged gate), `anchor` (the fixed eye), `right/tip_left` and
`right/tip_right` (the gripper pads). The left arm stays at READY and makes no contacts.

| step | phase | contacts (body pairs, count) |
|---:|---|---|
| 1 | reset | `anchor--carabiner` (2), `anchor--carabiner/Gate` (2) — lying on the base |
| 110 | close on the spine | `carabiner--right/tip_left` (4), `carabiner--right/tip_right` (4) — intended pinch |
| 205 | gate opening on the post | both right-tip pairs (4 each), `anchor--carabiner/Gate` (3) — gate pushed by the post |
| 240 | gate closed, clipped | both right-tip pairs (4 each); no gate-post contact (the gate is shut clear of the post) |
| 300 | released, resting on the post | `anchor--carabiner` (2), `anchor--carabiner/Gate` (2) — lying on the base, post in the loop |
| 470 | sleeve turning | same two `anchor--carabiner*` pairs — no gripper contact, so the carabiner is not dragged |
| 556 | pull-test starts | both right-tip pairs (4 each), `anchor--carabiner/Gate` (1) — the shut gate pulled taut on the post |
| 597 | pull-test held | both right-tip pairs (4 each), `anchor--carabiner/Gate` (3) — held against the post |

Every contact is intended: the carabiner lying on the base, the pads pinching it, the gate meeting the post, or the
shut gate bearing on the post under the pull-test. There are no arm-to-arm, arm-to-anchor, or gripper-through-carabiner
contacts. The hook, gate, sleeve and pull-test outcomes are geometric pose tests in the task file, not contact forces.

## 3. Cap protection

The scorer caps a threaded carabiner released with its gate open, and a pull-test started before the sleeve is locked,
at 0.5; a carabiner below the table scores 0 for the whole run. The demo triggers none of them: the gate is shut before
release, the sleeve reaches 0.28 turns (75 +/- 15 degree lock window) before the pull-test, and the carabiner stays on
the base/through the post throughout. A carabiner standing on edge is rejected because `threaded` needs the plane
within 45 degrees of horizontal.

## 4. Media

- `demo.mp4`: Isaac top camera only, 640x360, 348,203 B (< 1.87 MB), 650 frames with the final 1.0 frame held 3 s.
- `demo_top.gif`: 480x270, 4 fps, 1,196,625 B (< 1.87 MB), same top view.
- `setup.png`: 640x360, 27,873 B, Isaac top camera at reset.


## Isaac run

The scripted demo reached **1.0 at step 640 on Isaac** (`adhoc_4d08203e.json`): the real articulated gate was opened
around the anchor eye and closed, the knurled screw sleeve was flipped over-centre with the declared right-wrist
quarter-turn (hovering above the sleeve so it does not drag the hook), and the pull-test held for 1 s with both
grippers clear; no gate-not-closed, sleeve-not-locked or dropped fault. The carabiner articulation, anchor fixture and
colliders are unchanged on Isaac. It was filmed with the top camera and two render-only close-ups (`film_gate`,
`film_sleeve`) on a copy of the task; no wrist, finger or hook is seen crossing the anchor ring or board.
