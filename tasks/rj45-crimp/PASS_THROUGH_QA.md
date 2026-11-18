# PASS_THROUGH_QA — rj45-crimp

No part passes through a solid in the 1.0 demo. The crimper (base, seat rail, contact teeth, strain-relief jaw, ratchet
wheel and stops) and the hinged handles are **visual-only bodies with no colliders at all**; the plug and cable are
gravity-free rigid bodies posed by the task's declared carry, not by contact. There is therefore no solid in the work
area for the plug, cable or a finger to pass through. Checked in MuJoCo (top camera plus `data.contact`) every control
step of the demo. Source: `rj45_crimp.py` and `benchmark-media-rj45-crimp/demo.mp4` (top view only).

## 1. Frames checked (top view)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | stripped CAT6 cable at left with the red wire-1 index band, 8P8C plug in its pocket on the crimper head, seat rail and contact teeth bare, handle open, both arms at READY (all four step LEDs dark, score 0.00) |
| 25 | 133 | left pads have pushed all eight colour-coded wires into the plug channels with wire 1 on the pin-1 notch; LED 1 green, score 0.25 |
| 50 | 226 | wired plug carried to the crimper and seated in the rail; its index still on pin 1, cable locked at the seat, left pads opening; LEDs 1-2 green, score 0.50 |
| 75 | 293 | right arm has closed on the handle grips, the handle body has swept shut about its ratchet pivot, the contact teeth and strain-relief jaw are over the seated plug; LEDs 1-3 green, score 0.75 |
| 90 | 362 | right arm back at home, left pads closed on the cable jacket and drawing it back along -x; the plug stays seated (seated lock), LED 4 turning green |
| 100 | 401 | scored state: pull held 1 s, no cap, all four LEDs green, score 1.00; plug still at x 0.2656 y -0.0282, cable locked at x 0.2596 y -0.0282 |

Every approach is from above or along the cable axis. The cable is carried with a declared grasp offset; after the wires
are in, the plug follows with a fixed +6 mm offset, so the pair translates rigidly. The only rotation is the handle
body sweeping about its real hinge; nothing is teleported through the crimper head or the rail.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving task bodies |
|---:|---|---|
| 130 | inserting the eight wires | **none** — `[]` |
| 226 | seating the plug in the rail | **none** — `[]` |
| 293 | handle partly squeezed | none involving the plug, cable, crimper or handles; only `left/link3-right/link4`, `left/link4-right/link4` (the two arms brushing as the right arm reaches across) and one `left/tip_right-world` bench graze |
| 380 | pull-test | **none** — `[]` |
| 401 | scored state | **none** — `[]` |

There are **no contacts between any task body and a solid at any sampled step**. The arm-to-arm brushes during the
squeeze are the two robot forearms passing close over the work area; they are contacts (resting/brushing), not
penetrations, and they do not involve the plug, cable or tool. The single fingertip-to-world touch is a graze on the
bench, not a pass-through; the pull-test and final frames have no contacts at all. The crimper and handles have no
colliders, so "the contacts pierce the wires / the jaw grips the jacket" is a geometric state (teeth and jaw drawn at the
plug, squeeze force and dwell checked in code), not a simulated contact — stated openly in `REFERENCE.md`.

## 3. Cap protection (measured bad demos)

The scorer caps at 0.5 and blocks 1.0 on three latches; each was exercised in-sim with a temporary bad demo (removed
after the check):

* `wrong_order` — a cable flipped about z at insertion (wire 1 on pin 8) latched the cap and scored **0.5**; a fully
  misrouted flipped cable that never seated scored **0.0**.
* `not_seated` — a right arm that squeezed at 2 s, before the plug was in the rail, latched the cap and scored **0.5**.
* `weak_crimp` — a handle squeeze held at a half-closed finger gap (declared force below 4 N) latched the cap and scored
  **0.5**.

The 1.0 demo triggers none of them: wire 1 registers within the 15 deg order window, the plug seats within 5 mm and
12 deg, and the squeeze is held at a declared 6.0 N for 0.35 s. A start pose moved +3 mm in x/y/z and +3 deg about z
also reached 1.0 at step 401 with no cap.

## 4. Media and cutaways

* `demo.mp4`: Isaac top camera only, 640x360, 286,471 B (< 1.87 MB), 471 frames with the final 1.0 frame held 3 s.
  No close-up, side or cutaway in the PR media.
* `demo_top.gif`: 480x270, 4 fps, 1,056,667 B (< 1.87 MB), same top view.
* `setup.png`: 640x360 close-in top camera at reset; the red wire-1 index, the staged plug on the crimper head, the seat
  rail and the open handle are legible. The overlay states the four step conditions.


## Isaac run

The scripted demo reached **1.0 at step 401 on Isaac** (`adhoc_f75e0410.json`), the same step as MuJoCo: the eight
wires were inserted in T568B order, the plug seated in the seat rail within 5 mm / 12 deg, the crimper handle was
squeezed with the declared 6.0 N for 0.35 s until the ratchet locked, and the cable pull-test held for 1 s with the
grippers clear; no wrong order, mis-seat, weak squeeze or dropped-plug fault. The crimper and handles stay visual-only
with no collider on Isaac; the plug and cable remain gravity-free posed bodies. No task-local scoring or force change
was needed. It was filmed with the top camera and two render-only close-ups (`film_crimp`, `film_stage`) on a copy of
the task; no finger, forearm, plug or cable is seen crossing the crimper, seat rail or table.
