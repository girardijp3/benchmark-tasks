# TWIN_CHECK — cotter-pin-clevis

## Fresh live query (builder, 2026-11-17T21:53:56Z)

```
gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 \
  --json number,title,state
```

Returned **2,536 PRs, newest #2537**: **464 OPEN, 1,865 CLOSED, 207 MERGED**. The query was run at build time, as
the queue policy requires. The full result was searched for the object + mechanism + outcome and for every
pin/cotter phrase (`cotter`, `clevis`, `split pin`, `circlip`, `retaining ring`, `snap ring`, `spring clip`, `dowel`,
`R-clip`, `linchpin`, `pin`, `spread`). A different tool alone is not a different task.

## Object + mechanism + outcome of this task

**Object:** a real **3 mm nominal ISO 1234 steel split (cotter) pin** (2.8 mm wire), an **8 mm-class clevis pin with a
3.2 mm cross-hole**, a fork clevis with a rod end, and a pull tab.
**Mechanism:** the robot lifts the straight cotter from a white holder, aligns its wire axis with the cross-hole,pushes
it fully through with a 4-7 N push, bends its two legs to opposite sides with a finger, and drags the tab outward to
pull-test the joint.
**Outcome:** a cotter that is through with both legs spread, resisting an 8 mm pull held 1 s; pulling with the pin not
through, pulling with the legs not spread, or dropping the cotter are scored caps.

## Live-twin result — the two near cotter-pin PRs, both rejected

There is **no accepted or MERGED cotter-pin task** on the whole board. The two closest PRs were found and read in full,
including their review threads. **Both were rejected by review, and neither has an accepted implementation on the
board.** They are documented here as the queue requires for near pin/cotter tasks.

| PR | State now | Title | Review outcome (measured) |
|---:|---|---|---|
| **#994** | **OPEN**, label `Not accepted` (created 2026-11-07, no revision since) | *Add Secure a clevis pin with a cotter pin by bending its legs task* (`cotter-pin-spread-legs`) | **Not accepted** for construct validity and feasibility: a **10 mm steel cotter with 5 x 8 mm legs** was said to yield at **0.01 N m**, an impossible resistance for that steel section. Required realistic dimensions/material and a full-score demo. Its 48 h idea hold lapsed about **2026-11-09**; no revision was ever pushed. |
| **#2060** | **CLOSED**, not merged (closed **2026-11-16T21:16:30Z**) | *Secure a clevis pin with its cotter pin by bending the legs one each way* (`cotter-pin-clevis`) | **Not accepted: final.** First rejected because bending to the same side could score full credit; the revision fixed opposite-side bends. Final rejection was **importance**: the cited farm/heavy-equipment/aircraft mechanics did **not establish 500,000+ workers**, "too narrow for acceptance as specified." The reviewer wrote: *"You're welcome to submit a different task in a new pull request."* Its 48 h hold lapses 2026-11-18 21:16Z. |

### Why this task is not a twin of either

The author explicitly invited a different task in a new PR after #2060 was rejected. This submission is that different
task, and it is also measurably distinct in object, mechanism and outcome:

| Dimension | This task | #2060 (CLOSED, rejected) | #994 (OPEN, not accepted) |
|---|---|---|---|
| Cotter size | **3 mm** nominal, **2.8 mm wire**, 1.6 g | **2 mm** cotter | **10 mm** cotter with 5 x 8 mm steel legs |
| Starting state | cotter **straight in a holder**, joint bare | cotter **already pushed home**, legs straight | cotter on a holder, to be inserted |
| Scored ladder | align 0.25 -> **push through** 0.5 -> spread 0.75 -> **pull-test holds** 1.0 | only **leg bend angle** (0/0.25/0.5/0.75/1.0) on an inserted pin | seat face + leg bends; **no pull test** |
| Failure model | not-through / not-spread / dropped caps | leg bent back past 15 deg after 30 deg -> scrap | unrealistic 0.01 N m bend resistance |
| Importance case | **500k+ maintenance workers** (BLS maintenance & repair ~1.5 M; machinery mechanics/millwrights 547,300) | failed the 500k bar | not reached (rejected on physics first) |
| Real-world check | protractor for axis/bend, callipers for 3.2 mm hole / 2.8 mm wire, 4-7 N gauge, stopwatch for 1 s | protractor on leg angle | protractor on leg angle |

The rejection reasons of both PRs are therefore **addressed, not copied**: the impossible 10 mm / 0.01 N m physics of
#994 is replaced by a real 3 mm pin with a matched 3.2 mm clearance hole and a declared 4-7 N push; the failed
importance case of #2060 is replaced by an explicit routine-maintenance case of over 500,000 workers (see
`REFERENCE.md`). The added alignment rung and the pull-test outcome are mechanisms and outcomes absent from both.

## Other near pin/cotter PRs (all different object + mechanism)

| PR | State | Title | Difference |
|---:|---|---|---|
| **#1561** | MERGED | Fit a circlip into its shaft groove with circlip pliers: spread it just enough, seat it square, turn it to check | A 20 mm external retaining ring expanded with pliers into a groove; no clevis, no cotter wire, no push-through or pull test. |
| **#2159** | CLOSED | Clip a spring-gate carabiner to a steel tie-down staple | Spring-gate clip, not a split pin through a cross-hole; no leg spreading or pull test. |
| **#2010** | CLOSED | Fit locating pins to a fixture and load a bracket onto them so it is located | Dowel-style locating pins locating a bracket; fixed pins, not a deformable cotter fastener. |
| **#1986** | CLOSED | Refit a splash-shield panel with a push-in clip ... push the clip home through both | Push-in plastic panel clip; one-way barbed clip, not a two-leg split pin. |
| **#739** | CLOSED | Set an inspection cover onto its hidden locating dowel | Dowel seating; no cotter, no legs, no pull test. |
| **#2440** | OPEN | Seat a 100 mm wafer on a mask aligner's chuck: flat on two pins, rim on the third, then slide it in | Three-point wafer seating on kinematic pins; different object and outcome. |
| **#2468** | OPEN | Add robot foot tread wrapping and retaining-bar assembly task | Retaining bar and tread, not a clevis cotter. |

## Blocked / excluded memory checked

The queue's excluded list (shower curtain #2487, soda tab #1873, Oldham/jaw coupling #2484, ball-nut arbor #2504,
peristaltic pump head, split-ring key threading, plant potting, etc.) and the existing
`benchmark-submission/tasks/*/BLOCKED.md` family were reviewed. None is a 3 mm cotter pin through a clevis cross-hole
with align/push/spread/pull-test. No builder `BLOCKED.md` applies to this task.

**Verdict:** no live accepted twin and no builder block. The two near cotter-pin PRs are both rejected by review, are
documented above with their measured rejection reasons, and this submission fixes those reasons with a real 3 mm pin,
a matched clearance hole, an explicit 500,000+ worker case and a distinct pull-test outcome. **Clear to build.**
