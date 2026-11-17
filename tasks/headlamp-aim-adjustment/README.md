# Aim a real H4 headlamp's two screws until the low-beam cutoff sits on the 3 m screen line

An automotive-service task. A true-size **Hella 1058 / 1L6 002 395-131 178 mm H4 headlamp** (178 mm dia,
88 mm deep, 0.55 kg) stands in a bench gimbal with its lens **3.00 m** from a matte-white beam screen. Its
two real aim screws are 8 mm hex-socket screws on the rear housing, 90 mm apart, each with a **T-handle 8 mm
hex key left in the socket**. The lamp starts about 30 mm low and 39 mm left of the ECE cut-off (32.1 mm in
MuJoCo with its gravity droop, 29.7 mm on Isaac's pinned root). The robot has to
brace the lamp housing with one hand and turn the vertical aim screw and then the horizontal aim screw with
the other, until the low-beam cut-off sits on the screen's red line and inside the +-15 mm / +-26 mm aiming
band, then let go and leave the beam still for 1 s.

The lamp is on a real two-axis gimbal; each key/head is coupled to the reflector's aim through a shaft and a
**0.9 N m torque fuse** (the adjuster's strip limit). The cut-off on the screen is the projection of the
lamp's actual optical axis, with its **0.69 deg (1.2 %) ECE down-angle**, onto the screen plane: it is not a
scripted line. A wrong-direction turn, a beam off the screen, a pushed housing, a screw turned without the
housing braced, or a stripped socket all reduce the score.

**Result: the scripted demo reaches 1.0 at step 433 (21.65 s after the settle) in MuJoCo (3/3 runs) and on
Isaac.** The MuJoCo final cut-off is **4.1 mm below and 1.2 mm left of the target**, with the vertical key at
**+156.5 deg** and the horizontal key at **-179.2 deg**; the Isaac final cut-off is **0.3 mm below and 1.6 mm
left of the target**, with the vertical key at **+163.34 deg** and the horizontal key at **-177.62 deg**. No
strip, no wrong-way, no forced or unsteady fault in either run.

## Board / twin check (2026-11-17)

`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000` and title searches for
headlamp/headlight/aim/beam/lamp. The nearest board tasks are:

* **#2312 (OPEN) headlight-bulb-swap** — replaces an H11 bulb: unlatches the connector, quarter-turns the old
  bulb out and the new one in by its plastic base, and keeps the quartz glass clean. It never touches an aim
  screw, never moves the reflector/housing, and never measures a beam. This task leaves the H4 bulb in place
  and turns the two **aim screws** until the **cut-off** is on a **3 m** screen line.
* **#2396 (OPEN) Aim a spring-balanced desk lamp onto a dot in a notebook** — the nearest live "aim" task, but
  it moves a four-joint **Anglepoise-style shade** by hand, has no headlamp, no reflector, no adjuster screws
  and no cut-off; it scores where a beam spot lands on a page. This task has a fixed H4 lamp on a two-axis
  gimbal, two real 8 mm hex aim screws, a real ECE low-beam cut-off and the real +-15 / +-26 mm aim limits.
* **#172 (MERGED) Clear a photo-eye's beam** — removes an obstruction from an existing beam; no lamp aim.
* **#370 (CLOSED) Steer a laser onto a detector with two mirrors** — a laser and two mirrors, not a headlamp,
  no cut-off and no adjustment screws.
* #1756 (aim an inspection camera), #1542 (solar panel by shadow), #1597 (ORTF mics), #1365 (optical bench
  lens/screen) are different objects and mechanisms. No live headlamp-aim-adjustment twin was found.

## Files

| File | What it is |
|---|---|
| `headlamp_aim_adjustment.py` | The `yam_arena` task: scene, gimbal/screw/keys, the beam projection, the scoring (`success`, `score`, `steps_done`), the faults and the scripted `demo()` |
| `assets/headlamp_rig.usda` | The stand, the 178 mm H4 lamp on its `AimYaw`/`AimPitch` gimbal, the two 8 mm hex screw heads/shafts, the 0.9 N m fuses, the two T-handle keys and the steady pad |
| `assets/beam_screen.usda` | The 1.600 x 1.200 m screen on two legs, 3.00 m from the lens: the red 36 mm target line, the green +-15 mm band, the blue centreline, the 100 mm ticks |
| `assets/beam_patch.usda` | The collision-free low-beam light patch (horizontal cut-off, 15 deg kick-up, hot spot) that the task moves from the lamp's real pose |
| `build_assets.py` | Writes the three USDA assets with pxr (boxes, cylinders, hex prisms, lathes, a flat mesh; primitives and convex hulls as colliders) |
| `test_headlamp_aim_adjustment.py` | The offline pytest suite (22 checks): the ladder and caps, the 1 s hold, the band edges, the clear check, the beam geometry, the real values, the assets, the two keys, the demo plan and both arms' static reach |
| `benchmark-media-headlamp-aim-adjustment/make_media.py` | Internal MuJoCo renderer kept with the media package (not in the task folder): top view plus a 3.00 m beam-screen inset, `setup.png`, `demo.mp4`, `demo_top.gif`; the inset is not used in the PR video |
| `task.json` | The board metadata |
| `REFERENCE.md` | Sources, measured/estimated values, the beam/gear/fuse model, the calibration, the pass-through audit and the limitations |
| `PASS_THROUGH_QA.md` | The contact audit: the only arm/rig contacts are the two keys and the steady pad |

## Real product and values

The **Hella 1058 / 1L6 002 395-131** is a 7-inch (178 mm) round H4 halogen headlamp insert: 178 mm diameter,
88 mm deep, 0.55 kg, ECE E4 53, a 12 V H4 60/55 W bulb (OSRAM 64193-class, P43t, 23 g), a chromed reflector
and a glass lens. The two aim screws are 8 mm hex-socket adjusters (14 mm hex heads) on the rear housing,
90 mm apart; Hella sells 8 mm "adjuster screws" for its headlamp modules (P/N 152977001), and period adjuster
assemblies are M5-0.8 x 40/55 mm screws with a plastic grommet nut (Car-Pak 91-6886/91-6887). The task leaves
a real **T-handle 8 mm hex key** in each socket, so the robot grips the key rather than inserting a loose bit
(see the limitation note).

The aiming values are the published ones: **the horizontal cut-off sits 36 mm below the lamp centre at 3.00 m**
— 0.69 deg or 1.2 % — with the limits **+-15 mm vertically** and **+-26 mm horizontally** (Mitsubishi i-MiEV
service manual, headlamp aiming; UN R112/ECE R48 use the 1 % below H-H line, and the StVZO/Hella 1.2 %
passenger-car setting is 36 mm at 3 m). The brief's example "50 cm below the lamp centre" would be 9.5 deg,
which is not an ECE low-beam aim; this task uses the real 36 mm / 0.69 deg.

The screen is a 1.600 x 1.200 m white aiming board on two legs; its red line is at **0.824 m**, the lamp centre
is at **0.860 m**, the target is 36 mm below it. The 12 V supply is a bench box on the table; the lamp is on.

**Measured demo:** MuJoCo: initial cut-off **791.9 mm** (32.1 mm low) and **u -91.2 mm** (38.8 mm left of the
centreline); final **819.9 mm** (4.1 mm low) and **u -128.8 mm** (1.2 mm left); the V key **+156.5 deg** and the
H key **-179.2 deg**; the head/shaft fuse peak difference **7.3 deg** (below the 20 deg strip threshold);
no wrong-way, forced, unsteady, off-screen or strip fault. **1.0 at step 433.** Isaac: initial **794.3 mm**
(29.7 mm low) and **u -91.2 mm**; final **823.7 mm** (0.3 mm low) and **u -128.4 mm** (1.6 mm left); V
**+163.34 deg**, H **-177.62 deg**; same step, no fault.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top
0.7475. The lamp's gimbal centre is **x 0.420, y -0.130, z 0.860**; the lens is at x 0.620 (3.00 m from the
screen at x 3.508); the lamp is 0.58 kg with its centre of mass 44 mm forward and a 298 N m/rad `AimPitch`
drive (the gravity droop is about 0.05 deg). The two screw axes are at x 0.366, **y -0.175 (V)** and
**y -0.085 (H)**, z 0.900; their key T-bars are at **x 0.331**, the same y, z 0.900, 40 mm tall. The left steady
pad's outer face is at **(0.458, -0.0395, 0.860)**; the left gripper's TCP is (0.458, -0.0285, 0.860) so the
closed tips press 2 mm into the rubber pad. The screen centreline is at y -0.130; the target line at z 0.824;
the target band is +-15 mm, the horizontal limit +-26 mm. The arms start at READY; the left braces first, the
right then turns the keys and goes home. The 12 V supply is at (0.19, -0.33).

## Mechanism

* **Lamp pose:** the fixed stand is the articulation root; `AimYaw` is a Z revolute and `AimPitch` a Y revolute
  at the gimbal centre. The lamp's actual pose is read every control step; the two-axis pose is the only source
  of the beam geometry. The `AimPitch` drive is a 5.2 N m/deg position servo (EST; the real lamp's aim spring
  holds the reflector).
* **Screws and keys:** each `ScrewHeadV/H` is an X-axis revolute head (14 mm hex, 8 mm socket) with a
  coaxial `ScrewV/H` shaft coupled through a **0.9 N m torque fuse** (a `maxForce` drive). A T-handle 8 mm hex
  key is fixed in each socket. The shaft drives the lamp's aim through a gear ratio: **V 1.20 deg per screw
  turn**, **H 1.40 deg per screw turn**. The robot grips the key's T-bar; the wrist roll is the key turn. The
  demo turns V +156.5 deg and H -179.2 deg; the lamp's actual `AimPitch`/`AimYaw` are read from the joints.
* **Beam:** the cut-off ray is the lamp's actual optical axis with the 0.687 deg ECE down-angle
  (`ALPHA0 = atan(0.012)`); the center ray is projected onto the plane x = 3.508. The kink's screen height
  `h` and horizontal position `u` are compared with 0.824 and -0.130. The 15 deg kick-up is in the visual
  beam patch and the screen pattern. A lamp that is pushed, or a screw that does not move, changes the beam.
* **Bracing:** the left gripper's closed tips press the rubber steady pad; the `unsteady` fault latches if a
  screw is actually moving (a screw joint rate > 0.02 rad/s) with the left hand off the pad. Releasing after
  the turns is not a fault.
* **Faults:** `wrong_way` if the V screw is turned below -8 deg or the H screw above +8 deg; `stripped` if a
  key head/shaft slips more than 20 deg (the 0.9 N m fuse); `forced` if the lamp pose is more than 1.5 deg
  from what the screw shaft asks (a pushed housing or a forced cut-off); `on_screen` false if the cut-off is
  outside the board; `unsteady` as above.
* **The 1 s hold:** `success()` itself times 1.00 s with both cut-off coordinates within 2 mm, both grippers
  at least 5 cm clear of the lamp box, no grip force, the lamp speed below 10 mm/s and its rate below 0.05
  rad/s.

## Demo

The left arm goes to the pad from t 0 to 3.0 s and braces. The right arm waits until 3.0 s, grips the V key's
T-bar at x 0.331, turns it +165 deg over 2.6 s, lets go, grips the H key's T-bar at the other y, turns it
-180 deg over 2.9 s, lets go, and goes home. The left lets go at 20.5 s and goes home. The beam is held for 1 s;
success at step 433 (22.15 s in the task log).

The task's `demo()` is the scripted open-loop plan; the two keys are the only tools. The left hand braces the
housing, the right turns the keys; no cutaway.

## Scoring

| Step | Done when | Points |
|---|---|---|
| vertical screw engaged | the V key has turned at least 15 deg and the cut-off has moved toward the band | 0.25 |
| vertical cut-off in band | the cut-off is within +-15 mm of 0.824 m and on the screen | 0.5 |
| horizontal in band | plus the kink within +-26 mm of the screen centreline | 0.75 |
| full | both, no fault, both grippers clear, 1 s still, the lamp on its stand, no strip | **1.0** |

* **0** otherwise, including doing nothing (the initial beam is about 30 mm low and 39 mm left).
* **Caps 0.25:** a wrong-direction turn, a stripped key/socket, a forced cut-off (a pushed housing), a screw turn without the left hand bracing the housing, or the beam off the screen. If none of the 0.25 steps was reached,
the cap score is 0.
* **Moved housing / stand:** the stand root is fixed and the lamp is on its gimbal; a pushed housing shows as
  `forced`; the real trial checks that the stand did not move.
* **Real station:** the cut-off height is read on the 3.00 m screen (a tape line at 36 mm below the lamp
  centre and a +-15 mm band); the horizontal kink is checked on the centreline (+-26 mm); the two key turns are
  read from the 1/4-turn strokes; the 0.9 N m torque wrench checks the adjuster; the 1 s still hold is watched
  with both hands clear.

## Checks

```bash
cd benchmark-submission/tasks/headlamp-aim-adjustment
PYTHONPATH=../../../local_sim ../../../local_sim/.venv/bin/python -m pytest -q
cd ../../../local_sim
PYTHONPATH=. .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/headlamp-aim-adjustment/headlamp_aim_adjustment.py --steps 600 --trace
```

* pytest: **22 passed**. The static reach: right arm at the V key TCP (0.331, -0.175, 0.900) with the key
  frame — 0.03 mm / 0.01 deg; H key — 0.02 mm / 0.01 deg; left at the pad — 0.01 mm / 0.01 deg.
* Demo: **1.0 at step 433 (21.65 s after the settle), 3/3** in MuJoCo; V +156.5 deg, H -179.2 deg; final cut-off
  819.9 mm / -128.8 mm (4.1 mm low, 1.2 mm left); no strip, no fault; both grippers clear.
* Isaac: **1.0 at step 433** (`adhoc_969eeeab.json`); V +163.34 deg, H -177.62 deg; final cut-off
  823.7 mm / -128.4 mm (0.3 mm low, 1.6 mm left); no fault; both grippers clear. The top camera does not see
  the 3.00 m screen; the beam still is separate (`beam.png`).
* Dream-1: **0.0 at 2,400 steps** (2,401 action rows, `dreamlabs/dream-1-beta`, `adhoc_b0fa2b3f.json`); the
  arms move but never turn the keys, so V/H stay at 0.0 deg and the cut-off stays 794.3 mm / -91.2 mm.
* **Run limit: 2,400 steps (120 s at 20 Hz), the Dream evaluation budget; the scripted demo needs 433 (21.65 s) and the plan is 23.5 s.**
* **Pass-through:** the only arm/rig contacts are the intended right pads on the two key T-bars (max contact
  spring 1.215 mm on the V key, 0.770 mm on the H key) and the left tips on the steady pad (0.059 mm); no
  gripper, finger, key or forearm intersects the lamp housing, stand, screen or table. See
  `PASS_THROUGH_QA.md`.

## Media

The prepared media package (kept outside the task folder) has:

* `setup.png` — the **top view** at t 0: the lamp about 30 mm low and 39 mm left, the left and right arms at READY.
* `demo.mp4` — **top view only**, 640 x 360, 20 fps, 566,693 bytes (< 1.87 MB): the Isaac top camera, the key
  turns, the arms going clear, and the final **1.0** banner held for 3 s. No beam-screen inset, no wrist or
  4-view frame.
* `demo_top.gif` — the 640 x 360 top-view preview, 4 fps, 1,722,149 bytes.
* `beam.png` — a separate **beam-screen still** at the Isaac final aim (V +163.34 deg, H -177.62 deg): the
  shared screen geometry, the red 36 mm line, the green band and the measured cut-off **823.7 mm / -128.4 mm**.
  The Isaac top camera cannot see the 3.00 m screen, so this still carries the beam evidence; it is not in the
  video.
* `dream.mp4` — the **top-camera Dream-1 run**, 320 x 176, 20 fps, 705,261 bytes, 2,401 frames / 120.05 s: the
  completed 2,400-step trial (score 0.0). The arms move but never turn the keys, so V/H stay at 0.0 deg and the
  cut-off stays 794.3 mm / -91.2 mm.

Media M-XTE9G5. Dream-1: 0.0 (2,400 steps, 2,401 action rows), completed; top-camera `dream.mp4`.

## Reset

The lamp is at `AimPitch` +0.55 deg (low) and `AimYaw` +0.72 deg (left), the two keys at 0; the left arm at
READY, the right at READY; the screen and patch in place; the 12 V lamp on. The first key turn arms the
mechanism. The screen's target line at 0.824 m and the band are fixed; the left steady pad is clean; the
keys stay in their sockets. The scripted demo waits for the left brace before touching a key.

## Status

- MuJoCo demo: **1.0 at step 433 (21.65 s after the settle), 3/3**; V +156.5 deg, H -179.2 deg; cut-off
  4.1 mm low, 1.2 mm left; no fault; both grippers clear.
- Isaac demo: **1.0 at step 433** (`adhoc_969eeeab.json`); V +163.34 deg, H -177.62 deg; cut-off 0.3 mm low,
  1.6 mm left; no fault; both grippers clear.
- Dream-1: **0.0 at 2,400 steps** (2,401 action rows), completed and hit the step limit; the arms move but
  never turn the keys, so V/H stay at 0.0 deg and the cut-off stays 794.3 mm / -91.2 mm. Top-camera
  `dream.mp4`.
- Media: `demo.mp4` and `demo_top.gif` are **top-view only**; `setup.png` is the top view; `beam.png` is the
  separate beam still; `dream.mp4` is the top-camera Dream trial. `task.json` media M-XTE9G5.
- Publisher status: the technical package is ready (22 tests, fields pass, MuJoCo and Isaac 1.0, Dream-1 0.0,
  top-view media, separate beam still); the remaining gates are the board's importance/twin review and the
  media upload.
- No push, upload or PR was made.
