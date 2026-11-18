# Live twin check: toggle-clamp gusset

**Query at launch:** 2026-11-17T21:38:08Z, fresh
`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state`.
Returned **2,534 PRs**, newest **#2535** (462 OPEN, 1,865 CLOSED, 207 MERGED). The analyst's queue note
(`submission_tools/filter/BUILD_QUEUE.md`, 19:45Z) recorded 2,512 PRs / newest #2513; the board has grown by 22 PRs
since, so this is a genuinely fresh query, not a restatement. No same-object + same-mechanism + same-outcome live
OPEN task was found.

## Title searches (this board snapshot)

| Pattern | Hits | OPEN |
|---|---:|---:|
| `gusset` | 0 | 0 |
| `angle bracket` | 0 | 0 |
| `toggle` | 3 | 1 |
| `clamp` | 37 | 9 |
| `over-centre` | 1 | 0 |
| `bracket` | 18 | 1 |
| `fixture` | 10 | 1 |
| `locator` | 2 | 0 |
| `hold-down` | 0 | 0 |
| `welder` / `weld` | 6 / 14 | 0 / 0 |
| `square` | 33 | 4 |

## Nearest PRs and why they are not this task

* **#1879 OPEN** "Jump-start a flat battery with sprung booster clamps in the safe order" — the queue's listed near
  neighbour, checked explicitly. The object is booster/jumper cables and a flat car battery; the mechanism is a
  polarity/safe-connection order; the outcome is a jump-start. "Clamps" here are electrical booster clamps, not
  mechanical hold-down toggle clamps. **Different object, mechanism and outcome** — documented, not a stop.
* **#998 OPEN** "Retain a housing in a rollover fixture, turn it over and lock it, and fit a pad on its far face" —
  a rollover fixture and a housing; outcome is turning it over and fitting a pad, not seating a gusset square and
  throwing two toggle clamps. Closest OPEN fixture task, but different.
* **#1219 CLOSED** "Release the end clamp before the side clamp so the blank reaches both locators" — the nearest
  mechanical neighbour (clamps + locators) but the action is *releasing* clamps so a blank reaches locators, not
  seating a gusset and *engaging* two clamps over-centre. CLOSED and a different outcome.
* **#2010 CLOSED** "Fit locating pins to a fixture and load a bracket onto them so it is located" — locating pins and
  a bracket, but no toggle clamps and no over-centre lock; CLOSED.
* **#1343 CLOSED** "Locate a bracket on one round pin and one diamond pin, flats facing the round pin" — pin
  orientation, no clamps; CLOSED.
* **#1801 CLOSED** "Tension a cable on its spring gauge and lock it in a toggle clamp" — a cable and a spring gauge,
  not a gusset; CLOSED.
* **#1400 CLOSED** "Unlock an over-centre strut by its knee to stow a radiator panel" — an over-centre strut, not a
  clamp lever, and the action is unlocking; CLOSED.
* **#1198 CLOSED** "Seat a jig plate on its round and diamond locating pins, the one way round it fits" — jig plate
  and pins, no clamps; CLOSED.
* **#1984 OPEN** "Clamp a wooden strip in a screw vise and saw a 150 mm piece to length" — a screw vise and sawing.
* **#1892 MERGED** "hose-clamp-barb" — a hose clamp, different object and outcome.
* **#2308 CLOSED** "Tack-weld a T-joint square: hold a steel coupon on edge and TIG-tack it at three marks" — welding
  a T-joint, not fixturing a gusset with toggle clamps; CLOSED.

## Result

**No live OPEN task with the same object (100 mm angle-bracket gusset), mechanism (seat it on a two-pin locator nest
and flip two hold-down toggle clamps over-centre) and outcome (left clamped square, 1 s hold) exists on the board.**
No CLOSED/MERGED exact-final twin either. A different clamp, a different fixture, or a different action on a
similar object is not this task. **Clear to build** — proceed. No blocker was raised by this check.
