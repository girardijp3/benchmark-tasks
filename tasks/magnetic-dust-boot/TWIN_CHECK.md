# Live twin check: magnetic-dust-boot

**Query at launch:** 2026-11-17T21:28:30Z, fresh —
`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state`.
Returned **2,533 PRs**, newest **#2534** (**461 OPEN, 1,865 CLOSED, 207 MERGED**). The analyst's
`submission_tools/filter/BUILD_QUEUE.md` was written at 19:45Z against 2,512 PRs / newest #2513; the live board
advanced by 21 PRs, so the check was re-run at launch rather than taken on trust. Rank 23 listed **#2212 OPEN
"Fit a shaft-end magnetic encoder and verify its installation"** as the near neighbour and called it different;
that PR is confirmed still OPEN and is examined in full below.

## Title searches (this board snapshot)

| Pattern | Hits |
|---|---:|
| `magnet` | 17 |
| `dust boot` / `dust shoe` / `boot` | 0 / 0 / 0 |
| `spindle` | 7 |
| `seal` | 24 |
| `lathe` / `chuck` | 6 / 8 |
| `shoulder` | 3 |
| `concentric` | 1 |
| `encoder` | 2 |
| `65 mm` / `80 mm` / `oil-resistant` / `elastomer` | 0 / 0 / 0 / 0 |

No title combines a dust boot with a spindle nose, an 80 mm boot, a 65 mm nose, a magnet ring, concentric
alignment or a shoulder seat.

## Nearest PRs and why they are not this task

* **#2212 OPEN** "Fit a shaft-end magnetic encoder and verify its installation" — body: "Adds an unpowered
  assembly benchmark for fitting a shaft-end magnetic encoder. The YAM arms seat the D-keyed magnet carrier,
  place the sensor bracket on its guides, close the catch, and run the local field check … scripted YAM scored
  1.0 in 581 steps, Dream-1 0.0." This is the queue's named near neighbour and is **different on all three axes**:
  - **Object:** a D-keyed *magnetic encoder* (a position/speed sensor target) plus a sensor bracket; not an
    oil-resistant elastomer *dust boot* seal.
  - **Mechanism:** seat a D-keyed carrier, place a bracket on guides, close a catch, run a field check; not
    centre a boot bore concentrically on a spindle nose and press it to a shoulder with 4-7 N.
  - **Outcome:** a verified encoder signal (green field check); not a dust seal seated against the shoulder.
  There is no dust boot, no 65 mm spindle nose, no concentric magnet-centring and no shoulder seat. OPEN, but not
  the same task. (#811 CLOSED, a linear-encoder readhead commission, is likewise a sensor task.)
* **#1359 OPEN** "Add fit-the-last-magnet-into-a-motor-rotor task" — inserts one magnet into a motor rotor;
  different object, mechanism and outcome.
* **#2463 OPEN** "Add magnetic stirring-bar retention during laboratory decanting" — lab decanting, different
  object and outcome.
* **#2440 OPEN** "Seat a 100 mm wafer on a mask aligner's chuck: flat on two pins, rim on the third" — a wafer on
  a three-pin kinematic chuck; different object and mechanism (no magnet ring, no press to a shoulder).
* **#2371 CLOSED** "Add lathe chuck setup task" / **#400 CLOSED** "Chuck a bar in a lathe" / **#1492 CLOSED**
  "Take the chuck key out … before starting the lathe" — lathe chuck work, not fitting a spindle-nose dust boot.
* **#1325 CLOSED** "load-a-tool-holder-up-into-the-spindle" — a tool holder into a spindle taper; different part,
  different mechanism (taper lock, not a magnet-centred boot pressed to a shoulder).
* **#1040 CLOSED** "Set a small part on a surface grinder's magnetic chuck" — a magnetic chuck holding a workpiece;
  different object and outcome.
* **#2390 CLOSED** "Tighten a circular saw's blade bolt … holding the spindle lock in" — a saw-blade bolt and
  spindle lock; different object and outcome.
* **#971 CLOSED** "Open a concentric knockout to the connector's size and fit the connector" — the only
  "concentric" title; a connector knockout, different object and mechanism.

A different magnet, spindle, boot or seal alone is not the same task; the object + mechanism + outcome combination
(an 80 mm oil-resistant elastomer magnetic dust boot centred concentrically on a 65 mm spindle nose, pressed
against the shoulder with 4-7 N, released and left seated) has no live twin.

## Result

**No live OPEN task with the same object (80 mm magnetic dust boot with embedded magnet ring on a 65 mm spindle
nose), mechanism (centre the bore concentrically, press against the shoulder with 4-7 N, no off-centre grab) and
outcome (boot seated against the shoulder, grippers clear) — #2212 OPEN is a shaft-end magnetic encoder with a
different object, mechanism and outcome and is explicitly documented above. No CLOSED/MERGED exact-final twin
either. Clear to build** — proceed.
