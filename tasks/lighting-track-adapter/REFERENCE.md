# Reference, limits and model decisions

## Sources (public catalogue / instruction sheets)

- **Halo L650/L651 single-circuit track** (Cooper Lighting spec sheet ADV082385 / TD518195EN): extruded
  aluminium, 0.060 in (1.5 mm) wall, 1-3/8 in (35 mm) wide, 11/16 in (17.5 mm) deep, 24 mm slot, 120 V 20 A,
  L650 20-1/8 in / L651 44-1/8 in actual length. The task cuts a 1.000 m segment. The track's two copper busbars,
  separate ground strip and visual polarity groove are modelled; the circuit is locked out and no voltage is scored.
- **Satco TH489** 12 W forged LED H-track head (Satco spec sheet / snap2satco TH489): 3.00 in (76.2 mm) long,
  2.75 in (69.85 mm) wide, 6.25 in (158.75 mm) extension, 0.64 lb (0.290 kg), 120 V, 1020 lm, 36 deg, CRI 90+,
  cETLus. The head hangs below the track in the real orientation; the bench stand puts the track at 0.25 m.
- **H-style twist lock** (Satco H-STYLE TRACK HEADS 1/24 instructions; Nora IS-TRACK001-R6; Liton LPTM4
  instructions): "Pull down the locking lever and insert the Adapter Contact into the slot; rotate the adapter
  90 degrees to the locked position; the polarity line of the two-prong side must point to the groove on the
  track". Typical H-style lever detent torque **0.3-0.6 N m** and a 4-7 N seating push (queue values, consistent
  with the product sheets). The real trial checks the lock with a ~10 N pull on the head.

## Recognisable features modelled

Track: U-shaped extrusion with the 24 mm bottom slot, two thick lips, machined lip edges, two copper busbars, a
ground strip, a polarity groove and line, end caps, a black pedestal on a rubber-footed base, a top arm/clamp and a
red lockout tag. Head: 40 mm chamfered polycarbonate adapter, 22 mm neck, two steel contact tabs, a white polarity
arrow, a knurled lever knob on a post/arm below the gripper, a swivel collar, 20 mm stem, a 12-finned forged
cylinder, a lens bezel, model band and label plate. All round visuals are closed 64/96-segment lathe meshes; all
colliders are simple true-size boxes/cylinders.

## Sim values vs real values

| Quantity | Real | Sim | Note |
|---|---|---|---|
| Track width / height / slot | 35 / 17.5 / 24 mm | same | true size |
| Head diameter / length / mass | 69.85 / 76.2 mm / 0.290 kg | same | true size, authored inertia |
| Head extension | 159 mm | 159 mm | from track bottom to lens |
| Lever turn to lock | 90 deg | -90 deg (clockwise viewed from above) | sign follows the satco clockwise instruction; geometry symmetric |
| Lever detent | 0.3-0.6 N m | 2.6 N m/rad angular drive, 0.9 N m cap, target -90 deg below -70 deg | the drive supplies the detent; the real 0.45 N m is inside the cap |
| Axial load / tug | ~10 N spring scale, fixture held | 20000 N/m linear drive, target 0, 0.17 mm gravity sag | the joint is the load path; a sustained >20 N stem grip with the cam locked is the sim proxy for the real pull |
| Lever hand-follow | fingers twist the lever | wrist angle code-keyed to the Twist drive target while the hand holds the knob | **handling simplification**: the real hand supplies the torque; no extra geometry or score is hidden |
| Work height | ceiling install | track at 0.25 m above table | bench-height only, as the queue requires |
| Electrical | 120 V live when installed | locked out, off | no voltage contacts are scored |

## Measured demo results

- MuJoCo (mjkit, 240 Hz physics / 20 Hz control), 3/3 identical runs: **1.0 at step 396**, t = 19.80 s after the
  0.5 s settle (t = 20.30 s in the task log). Ladder latches: 0.25 step 118, 0.5 step 156, 0.75 step 287,
  1.0 step 396. Final `Twist` -90.0 deg, `Insert` 0.00 mm; no wrong-way, dropped, bent or failed-tug latch.
- Score ladder, caps and hold: 24 offline pytest checks.
- The 1 s hold is timed inside `success()` from the first step where the cam is locked, the tug has passed, both
  grippers are >= 5 cm clear, the insert rate < 0.0002 m/s and the twist rate < 0.02 rad/s; any lapse restarts it.

## Limitations

- The lever grip is code-keyed: a VLA that reaches the knob and turns the wrist is credited even if the simulated
  pad contact is soft; the real station uses a hand twist and a 0.3-0.6 N m torque check.
- The cam/axial lock is a joint drive, not a rigid-body cam surface; the tabs, lips and contact geometry are real
  but the resisting torque/load comes from the drives. A real bent track would be a physical deflection; sim uses
  a latched `bent_track` flag from abnormal insert displacement.
- No live voltage, no photometric beam, no ceiling-height station; the head's lens is a dark visual.
- Isaac and Dream runs are pending; the finisher must re-run and record the numbers before publishing.

## Live-twin line (fresh query, 2026-11-17)

`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state,createdAt` returned
**2,507 PRs, newest #2508** (437 OPEN). No live OPEN task with the same 120 V H-track head + quarter-turn cam +
tug outcome. #2293 CLOSED is a van E-track tie-down ratchet (different object, mechanism and outcome). No exact
twin; no `BLOCKED.md` for this folder.
