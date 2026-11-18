# Live twin check: index-plunger-fixture

**Query at launch:** 2026-11-17T21:41:18Z, fresh
`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state`.
Returned **2,536 PRs**, newest **#2537** (464 OPEN, 1,865 CLOSED, 207 MERGED). The board has moved since the analyst's
BUILD_QUEUE snapshot (19:45Z: 2,512 PRs, newest #2513), so it was re-run at launch rather than taken on trust.

## Title searches (this board snapshot)

| Pattern | Hits / note |
|---|---|
| `index plunger`, `indexing`, `index pin`, `index plate` | 0 |
| `index` | 7: #1837 MERGED finger oximeter, #1463 CLOSED cryo canister, #1117 CLOSED Geneva drive, #934/#186 CLOSED footrest index latch, #829 CLOSED dividing head, #233 CLOSED rotary table |
| `fixture plate` | 1: #332 CLOSED (change over against locators) |
| `fixture` | 10, all CLOSED except #998 OPEN (rollover fixture) — different actions |
| `plunger` | 7: French press #2139 OPEN, sink #2092 OPEN, syringe #2012 CLOSED, syringe pump #892 OPEN, carriage #402 CLOSED, spring plungers #292 CLOSED, panel #223 CLOSED |
| `detent` | 5: tape gun #2043, nutrunner #2040, bottle #1087, training handle #933/#187 — different objects |
| `rotary table` / `dividing head` / `turret` | #233 CLOSED / #829 CLOSED / 0 |

## Nearest PRs and why they are not this task

* **#332 CLOSED** "Add Change over a machine's fixture plate against its locators task" — the queue's listed near
  neighbour. It is a **fixture changeover against locators** (unclamp, lift off, set the new fixture on its locators,
  re-clamp); there is no spring index plunger, no pull-to-retract, no rotate-to-next-stop and no pin release into a
  ring of holes. Different action, different mechanism, different outcome, and CLOSED. **Documented here as required.**
* **#233 CLOSED** "Index an off-balance rotary table 90 degrees and lock it with a pin" — the closest mechanism: an
  off-balance rotary table turned 90 degrees and locked with a separate pin. Different object (off-balance rotary
  table, not a fixture plate with a built-in spring index plunger), different mechanism (a separate lock pin and a
  90 degree move, not pull-knob / 45 degree next-stop / spring release), different outcome, and CLOSED.
* **#829 CLOSED** "Index a workpiece one of 7 equal divisions on a dividing head" — a dividing head and 7 divisions;
  a different tool and object, no spring index plunger or hole-release, and CLOSED.
* **#1117 CLOSED** "Index a Geneva drive and prove its dwell" — a Geneva mechanism and dwell proof, not a plunger.
* **#292 CLOSED** "Test two recessed spring plungers with a pin tool" — testing recessed plungers with a tool, not
  pulling a knob to index a plate.
* **#223 CLOSED** "Open a panel held shut by a spring plunger" — releasing a panel latch, not indexing.
* **#998 OPEN** "Retain a housing in a rollover fixture, turn it over and lock it..." — a rollover fixture and a
  housing; a different object and mechanism (turn over + lock + pad), not an index plunger.
* **#2139 OPEN / #2092 OPEN / #892 OPEN** — French press, sink and syringe-pump plungers: unrelated objects that
  share the word "plunger".

## Result

**No live OPEN task with the same object (16 mm spring index plunger on a fixture plate), mechanism (pull the knob to
retract the pin, rotate the plate one hole pitch, release the pin into the next hole) and outcome (the plate left
indexed at the next stop).** The nearest indexing/fixture PRs are all CLOSED and differ in object, mechanism or
outcome; a different tool or a CLOSED exact-final task is not an OPEN live twin. **Clear to build** — proceed.
