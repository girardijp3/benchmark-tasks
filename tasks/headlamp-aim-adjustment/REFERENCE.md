# Reference: headlamp-aim-adjustment

Everything this task uses, with the real source or an explicit **EST**. The lamp is a true-size Hella 1058
178 mm H4 insert on a bench gimbal 3.00 m from a white aiming screen; the two aim screws are 8 mm hex-socket
screws with T-handle keys left in the sockets. The bench gimbal, the screen, the stand, the keys and the 0.9 N m
fuse are bench equipment; the lamp, the bulb, the screw size, the ECE cut-off and the aiming limits are real.

## Sources

- **Hella 1058 / 1L6 002 395-131** 178 mm H4 headlamp insert, Hella New Zealand product page:
  <https://www.hella.co.nz/en/products/driving-headlamp/7-round-headlamps-and-housings/178mm-h4-halogen-headlamp-highlow-beam-front-position-insert.html>
  — 178 mm, 88 mm deep, H4 12 V, ECE E4 53, the outline drawing; "178 88" in the Hella Headlamps and Inserts
  catalogue (PDF): <https://www.hella.co.nz/content/showfile.php?downloadid=832>
- **Hella adjuster screw** P/N 152977001 for the 90 mm Classic series headlamp modules:
  <https://www.hptautosport.com/products/hella-adjuster-screws-for-90mm-classic-series-headlamp-modules>
  — the real 8 mm headlamp adjuster screw family. Period M5-0.8 x 40/55 mm adjuster screws with a plastic
  grommet nut (Car-Pak 91-6886/91-6887, Ford Probe) from the Car-Pak headlamp-parts catalogue:
  <https://www.pdqautosupplies.com/_files/ugd/6d5643_5faf964540cf480baa6448e287befe87.pdf>
- **OSRAM 64193 H4 12 V 60/55 W** (P43t, 82 mm, 17 mm glass, 23 g): the standard H4 bulb in the 1058.
- **Mitsubishi i-MiEV service manual, headlamp aiming**:
  <https://mmc-manuals.ru/manuals/i-miev/online/Service_Manual/2011/54/html/M154010050027400ENG.HTM>
  — "Vertical direction; 36 mm (0.69 deg) below horizontal (H); horizontal direction; elbow point intersects
  the vertical line (V); limit: vertical +-15 mm (+-0.29 deg); horizontal +-26 mm (+-0.5 deg)". That is the
  36 mm at 3.00 m used here.
- **UN Regulation No. 112** (headlamps with filament lamps, asymmetrical passing beam), the cut-off:
  <https://unece.org/sites/default/files/2025-01/R112r4e.pdf> — the vertical aim is on the line B, one per
  cent below H-H; the horizontal adjustment puts the cut-off kink on the V line.
- **UN Regulation No. 48 / ECE R48** (installation of lighting; the initial low-beam inclination is marked by
  the manufacturer, typically 1.0-1.5 % for a car; 1.2 % = 36 mm at 3 m).
- **Hella headlamp adjustment instructions** (the beamsetter and the 1 % / forward-inclination explanation):
  <https://www.hella.com/techworld/us/technical/automotive-lighting/headlight-adjustment>
- **BLS OES 49-3023** (automotive service technicians and mechanics, about 700,000 in the US): the job family
  that aims headlamps after a bulb, a collision repair or an inspection. <https://www.bls.gov/oes/current/oes493023.htm>

## Measured (published, real)

| Item | Value | Source |
|---|---|---|
| Lamp | Hella 1058 / 1L6 002 395-131, 178 mm dia, 88 mm deep | Hella page / catalogue |
| Lamp mass | 0.55 kg | Hella / trade data |
| Bulb | H4 12 V 60/55 W, P43t, 82 mm, 23 g | OSRAM 64193 |
| Aim screws | 2 x 8 mm hex-socket adjuster, 14 mm hex head, 90 mm apart | Hella adjuster screws / the 1058 rear view |
| Cut-off at 3 m | 36 mm below the lamp centre, 0.69 deg / 1.2 % | i-MiEV manual; UN R112; Hella 1.2 % |
| Vertical limit | +-15 mm (+-0.29 deg) | i-MiEV manual |
| Horizontal limit | +-26 mm (+-0.5 deg) | i-MiEV manual |
| Screen | 1.600 x 1.200 m, 3.00 m from the lens | bench equipment |
| Lamp centre / target | 0.860 m / 0.824 m | layout |
| Aiming torque | 0.9 N m strip limit (the fuse) | the task's declared adjuster limit |

The brief's example "50 cm below the lamp centre" would be 9.5 deg at 3 m, not an ECE low-beam aim; the
published ECE value is 36 mm at 3 m (0.69 deg), and this task uses it. The screen's target line is 12 mm wide
(a real aiming board's marked line, not a code value): the beam patch is a separate collision-free visual
moved from the lamp pose.

## Estimated / bench (EST)

| Item | Value | Why |
|---|---|---|
| Bench gimbal | a two-axis `AimYaw`/`AimPitch` at (0.420, -0.130, 0.860) | a bench stand, not the product |
| Lamp mass in sim | 0.58 kg (0.55 lamp + 0.03 fitting) | Hella + fitting |
| Lamp COM | 44 mm forward of the gimbal | the reflector/housing; EST |
| Aim pitch drive | 5.2 N m/deg (298 N m/rad), damping 0.10 N m s/deg | a stiff aim servo holding the reflector; EST |
| Gear ratios | V 1.20 deg/turn, H 1.40 deg/turn | a fine adjuster; EST |
| Torque fuse | 0.9 N m (`maxForce`), head/shaft coaxial | the declared adjuster strip limit; EST |
| Strip slip | 20 deg head/shaft | a stripped 8 mm socket, not a small fuse lag; EST |
| Running friction | the shaft drive; the key turns freely | EST |
| Key | 8 mm hex shaft, 40 mm T-bar, 6 mm dia, 30 g | a field T-handle hex key; EST |
| Screen and stand | 1.6 x 1.2 m board, two legs, floor | bench equipment |
| 12 V supply | a bench box at (0.19, -0.33); the lamp is on | bench equipment |
| Cut-off beam patch | the ECE horizontal part + 15 deg kick-up; 0.50 m deep | a visual beam patch; the score reads the lamp pose |
| Hold / clear | 1.00 s, 2 mm, 5 cm, 10 mm/s, 0.05 rad/s | an acceptance envelope |
| Run limit | 2,400 steps (120 s at 20 Hz) | the Dream evaluation budget; the demo needs 433 |

## Beam geometry

The lamp's `AimPitch` link origin is the gimbal centre; its local +X is the optical axis and its local +Z the
lamp up. The lens centre is `p = pos + R (0.088, 0, 0)`. The cut-off centre ray is

```
axis = R (1, 0, 0)
up   = R (0, 0, 1)
d    = axis cos(ALPHA0) - up sin(ALPHA0),   ALPHA0 = atan(0.012) = 0.687 deg
t    = (SCREEN_X - p_x) / d_x
u, h = (p + t d)_y, (p + t d)_z
```

`SCREEN_X = 3.508 m`, `p_x = 0.620 m` at the nominal pose, so the nominal `h = 0.860 - 3.00 * 0.012 =
0.824 m`. The screen centreline is `SCREEN_Y = -0.130 m` (the lamp's y). The task's `v_err = h - 0.824` in mm
and `h_err = u + 0.130` in mm. The 15 deg kick-up and the horizontal part are drawn on the beam patch; the
score is the horizontal cut-off at the kink. The screen is `on_screen` when the kink is at least 5 cm inside
the 1.6 x 1.2 m board and the lamp's local +Y is still world +Y (no roll); otherwise the cut-off is off the
screen and the score caps at 0.25.

The lamp starts at `AimPitch = +0.55 deg` (nose down; the lamp's positive pitch is nose down in this joint) and
`AimYaw = +0.72 deg` (left), giving `h = 0.7919 m` and `u = -0.0912 m` — 32.1 mm low and 38.8 mm left of the
target. The V key turns +156.5 deg (the V gear target +165 and the arm's key stop; the beam is 4.1 mm low);
the H key turns -179.2 deg (the H gear target -180; the beam is 1.2 mm left). The final `h = 0.8199 m`,
`u = -0.1288 m`.

## Screw / gear / fuse model

Each screw head is a 14 mm hex with an 8 mm hex socket; the head's socket walls are 6 boxes at the 8.2 mm
across-flats; the head's outer ring is six plates. The head is a revolute `ScrewHeadV/H` about X; the shaft
`ScrewV/H` is a coaxial revolute; the head drives the shaft through a `maxForce = 0.9 N m` drive (`stiffness
0.5 N m/deg`, damping 0.02). The shaft's angle `s` drives the lamp:

```
AimPitch_target = +0.55 deg + K_V * s_V,   K_V = -1.20 deg/turn
AimYaw_target   = +0.72 deg + K_H * s_H,   K_H = +1.40 deg/turn
```

The task writes these targets every control step; the lamp's actual joints are read. The `ScrewHead` and
`Screw` angles are the joint states. The T-handle key is **fixed** to the head: its 8 mm hex shaft sits in the
socket and its 40 mm T-bar is the robot's grip. The key/head/shaft therefore turn as one; the fuse's
head/shaft angle difference is the strip check. A head/shaft difference over 20 deg latches `stripped`; the
shaft drive saturates at 0.9 N m, so a forced key cannot turn the shaft past the fuse.

The robot grips the T-bar with the right gripper: the gripper's +Z is +X (the key axis); its fingers close
along local Y; the 6 mm T-bar is between the pads. The wrist roll around +X is the key turn. The left gripper
is closed and presses the rubber steady pad on the lamp's left side with its tips 2 mm into the 4 mm rubber;
the pad's outer face is at (0.458, -0.0395, 0.860). A screw joint rate above 0.02 rad/s with the left tips
not within 5 mm of the pad latches `unsteady`.

## Calibration map (MuJoCo, one run per row)

The arm plans are the same; the rows are the initial/key-turn/target variants in the offline tests and the
demo. The beam values are read from the lamp's actual pose.

| Case | V key | H key | cut-off h | cut-off u | faults | score |
|---|---|---|---|---|---|---|
| idle | 0 | 0 | 791.9 mm (-32.1) | -91.2 mm (+38.8) | none | 0 |
| V only | 156.5 | 0 | 819.9 mm (-4.1) | -93.6 mm (+36.4) | none | 0.5 |
| both, final demo | 156.5 | -179.2 | **819.9 mm (-4.1)** | **-128.8 mm (+1.2)** | none | **1.0** |
| wrong V | -20 | 0 | 789 mm (below) | - | wrong_way | 0.25 |
| wrong H | 0 | +20 | - | - | wrong_way | 0.25 |
| strip | a head/shaft slip > 20 deg | - | - | - | stripped | 0.25 |
| pushed lamp | a lamp pose > 1.5 deg from the screw target | - | - | - | forced | 0.25 |
| beam off screen | - | - | above/below the board | outside | off-screen | 0.25 |

The 0.9 N m fuse and the 20 deg slip are the strip model; the task never reaches 20 deg in the demo (the peak
head/shaft difference is 7.3 deg during the V turn, a fuse lag, not a strip). The wrong-direction threshold
is 8 deg; the cap is 0.25, and the demo never crosses it.

## Demo timeline

| t (s) | left | right |
|---|---|---|
| 0-3.0 | to the steady pad; brace | wait at READY |
| 3.0-4.2 | hold the pad | behind the V key |
| 4.2-5.7 | hold | around and grip the V key |
| 5.7-9.3 | hold | turn V +165 deg (actual +156.5) |
| 9.3-10.4 | hold | let go, clear the V key |
| 10.4-13.1 | hold | behind/grip the H key |
| 13.1-16.0 | hold | turn H -180 deg (actual -179.2) |
| 16.0-17.0 | hold | let go, clear the H key |
| 17.0-19.5 | hold | home |
| 19.5-20.5 | let go | home |
| 20.5-23.5 | home | hold; **1.0 at step 433** (22.15 s) |

The mjkit log: steps done 0.25 at 128 (V engaged), 0.5 at 149 (V in band), 0.75 at 273 (both in band), 1.0 at
433 (the 1 s clear hold). The score is 1.0 in 3/3 runs.

## Pass-through audit

The task was run end to end with a contact log on every control step. The only arm/rig contacts were:

| Pair | Contacts | Min distance |
|---|---|---|
| `rig/KeyV` <-> `right/tip_left` | 420 | -1.215 mm |
| `rig/KeyV` <-> `right/tip_right` | 364 | -0.664 mm |
| `rig/KeyH` <-> `right/tip_right` | 509 | -0.770 mm |
| `rig/KeyH` <-> `right/tip_left` | 534 | -0.698 mm |
| `left/tip_left` <-> `rig/AimPitch` (the steady pad) | 1,678 | -0.059 mm |

These are the intended key grips and the rubber-pad brace. The 1.215 mm is the soft contact spring of the
1 mm contact offset on the 6 mm T-bar; it is not a pass-through of the lamp, stand, screen or table. There
were **no** right or left arm links against the lamp housing, the stand, the screen or the table; no key
against the stand/screen/table; no key through the lamp (the key's 8 mm shaft sits in the socket, by design);
no gripper through the lamp. See `PASS_THROUGH_QA.md`.

## Limitations (stated)

- **The screwdriver is a T-handle 8 mm hex key left in each socket**, not a loose Phillips driver picked from a
  holster. The robot grips the T-bar; the key is keyed to the screw head and its 8 mm shaft sits in the 8.2 mm
  socket. This is a real adjuster tool and avoids the bit-insertion tolerance of a loose 135 mm driver (the
  first build attempt measured a 2-3 mm bit offset from the arm's wrist tracking at the real 8 mm socket; it
  could not engage honestly and was replaced by the key). The task's two keys are the real 8 mm hex drivers;
  the left hand braces the housing throughout.
- The bench stand/gimbal, the screen, the two keys, the 0.9 N m fuse and the 5.2 N m/deg lamp aim drive are
  **bench equipment**, not the Hella product. The lamp, bulb, 8 mm screw, ECE cut-off and the +-15/+-26 mm
  limits are real.
- The beam is a **geometric optical model** (the actual lamp pose, the 0.687 deg cut-off, the screen plane),
  not a photometric render; the beam patch is a visual on the screen and the score reads the pose.
- The lamp is 0.58 kg and its COM is 44 mm forward; the aim drive's static droop is about 0.05 deg. The 0.9 N m
  fuse is the declared adjuster strip limit; the task does not measure the running torque with a torque sensor;
  the key turn is free (the shaft is not loaded by the lamp's aim drive, which is a separate joint).
- The 3.00 m screen is exactly 3.000 m from the lens; the target is 36 mm below the lamp centre (0.69 deg),
  not the brief's 50 cm. The 50 cm would be 9.5 deg and is not an ECE low-beam aim.
- The Isaac scripted demo was run and reaches **1.0 at step 433** (`adhoc_969eeeab.json`): V +163.34 deg,
  H -177.62 deg; cut-off 823.7 mm / -128.4 mm (0.3 mm low, 1.6 mm left) from an initial 794.3 mm / -91.2 mm;
  no strip, wrong-way, forced, unsteady or off-screen fault. The fixed rig root is replaced on Isaac by a free
  root pinned at every step (PhysX drops the fixed-root articulation); MuJoCo keeps the fixed root and the same
  1.0 at step 433, with its gravity droop putting the initial cut-off at 791.9 mm.
- The PR video is **top-view only** (the Isaac top camera); the 3.00 m beam-screen still (`beam.png`) is
  separate and is not an inset in the video.
- Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed, hit the step limit); the arms move but
  never turn the keys. The top-camera trial is `dream.mp4`. No push, upload or PR was made.
