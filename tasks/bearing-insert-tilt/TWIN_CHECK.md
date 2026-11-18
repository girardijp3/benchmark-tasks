# TWIN_CHECK — bearing-insert-tilt

## Fresh live query (builder, 2026-11-17T21:28:18Z)

```
gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 \
  --json number,title,state
```

Returned **2,533 PRs, newest #2534**: **461 OPEN, 1,865 CLOSED, 207 MERGED**. This is a newer board than the
BUILD_QUEUE snapshot (2,512 PRs, newest #2513, 19:45Z), so the query was re-run at build time as the queue policy
requires. The full result was searched for the object + mechanism + outcome; a different tool alone is not a
different task.

## Object + mechanism + outcome of this task

**Object:** a real 20 mm spherical-OD (self-aligning) bearing insert for a matched pressed pillow-block housing.
**Mechanism:** the bearing's spherical outer ring seats in the housing's concave spherical seat whose axis is inclined
8 deg; the bearing axis must be tilted to the seat axis and its grease hole rotated (about that axis) to the housing's
single grease port, then it is pressed flush 4-7 N.
**Outcome:** a flush, registered, undamaged fit held 1 s; a steep entry jams the rim, an unregistered press scores the
bore, and an over-deep press is forced — all scored caps.

## Search result

No PR in the whole 2,533-row board has this object + mechanism + outcome. In particular there is **no OPEN task** that
lifts a spherical-OD bearing, tilts and rotates it to a matched spherical seat and presses it flush with jam/score/force
caps. The live-twin search is **clear**.

The specific queue note is confirmed: **#1880 (MERGED) "Set a telescoping gauge to a bearing housing's bore diameter
and leave it on its stand"** shares the words *bearing housing* but is a **measurement / tool-setting task**: a
telescoping gauge is expanded to a bore and parked on a stand. It does not handle a bearing insert, does not tilt or
rotate anything into a spherical seat, has no press and no jam/score/force outcome. MERGED and a different mechanism and
outcome, so it is not a twin. It is documented here rather than treated as a block.

## Nearest PRs (and why each is a different task)

| PR | State | Title | Difference |
|---:|---|---|---|
| **#1880** | MERGED | Set a telescoping gauge to a bearing housing's bore diameter and leave it on its stand | Measurement/tool-setting on a stand; no bearing insert, no tilt/rotate/press, no damage outcome. The queue-note twin; different. |
| **#1800** | MERGED | Add oil-feed sleeve assembly with radial port registration | A sleeve with a radial oil port registered to a fitting — similar *port registration* idea, but a sleeve/oil-feed assembly, not a spherical-OD bearing tilted into a spherical seat and pressed flush. MERGED. |
| **#1199** | CLOSED | Drive a bushing to its depth below the housing face, choosing the stepped driver over the plain drift | Depth-driving a bushing with a chosen tool; no spherical seat, no tilt/rotate alignment, no jam/score caps. CLOSED. |
| **#40** | CLOSED | Press a bearing onto a robot link's shaft | Bearing pressed onto a shaft (through the rings), not an insert tilted into a spherical housing seat. CLOSED. |
| **#995** | CLOSED | Drive a bearing onto its shaft through the inner ring, never through the balls | Correct bearing-on-shaft pressing technique; different object placement and outcome, CLOSED. |
| **#1113** | MERGED | Feed a hex shaft through two bearings | Shaft-through-bearings, not bearing-into-housing. MERGED. |
| **#259** | CLOSED | Line up two bearings with a motor's shaft | Aligning bearings to a shaft, not spherical-seat insertion. CLOSED. |
| **#1890** | CLOSED | Seat a strain-wave flexspline unit in a robot joint housing | Housing seating, but a harmonic-drive flexspline with meshing teeth; no spherical OD, tilt, grease port or press. CLOSED. |
| **#2440** | OPEN | Seat a 100 mm wafer on a mask aligner's chuck: flat on two pins, rim on the third, then slide it in | Flat wafer seating with a slide; different object, mechanism and outcome. |
| **#1905** | OPEN | Push a shower valve's cartridge flush against its O-rings and hold it while sliding the U-clip | Flush push-and-hold, but a valve cartridge and U-clip, no spherical self-aligning seat or jam/score caps. |
| **#2162** | OPEN | Fit a glass screen protector with its alignment tray: seat the tray square, press, lift it off | Alignment tray and press, but a flat screen protector; no spherical bearing or tilt-to-seat. |

## Blocked/excluded memory checked

The queue's excluded list (shower curtain #2487, soda tab #1873, Oldham/jaw coupling #2484, ball-nut arbor #2504,
peristaltic pump head, split-ring key threading, plant potting, etc.) and the existing
`benchmark-submission/tasks/*/BLOCKED.md` family were reviewed. None is this object or mechanism. No builder
`BLOCKED.md` applies to this task.

**Verdict:** no live OPEN twin, no builder block; the task is clear to build. #1880 is MERGED and a different
measurement task, documented above as the queue required.
