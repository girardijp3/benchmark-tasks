# TWIN_CHECK — rj45-crimp

## Fresh live query (builder, 2026-11-17T21:53:21Z)

```
gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 \
  --json number,title,state
```

Returned **2,538 PRs, newest #2539**: **466 OPEN, 1,865 CLOSED, 207 MERGED**. This is a newer board than the
BUILD_QUEUE snapshot, so the query was re-run at build time as the queue policy requires. The full result was searched
for the object + mechanism + outcome; a task that shares only a tool name or a cable is not a twin.

## Object + mechanism + outcome of this task

**Object:** a real 8P8C (RJ45) CAT6 plug and a stripped CAT6 cable with eight T568B-ordered conductors, plus a
ratcheting crimper with a plug seat, contact teeth, strain-relief jaw and hinged handle.
**Mechanism:** align wire 1 to pin 1 and insert all eight wires, carry and seat the wired plug fully in the seat rail,
squeeze the ratchet handle with a declared 4-7 N so the contacts pierce the insulation and the jaw grips the jacket,
then draw the cable back axially with a declared 4-7 N pull.
**Outcome:** a flush, ordered, undamaged termination that holds a 1 s axial pull-test; a flipped cable, an early
squeeze and a weak crimp are scored caps.

## Search result

No PR in the whole 2,538-row board has this object + mechanism + outcome. In particular **no OPEN task** wires an
eight-conductor RJ45 plug, seats it in a ratcheting crimper and pull-tests it. The live-twin search is **clear**. The
two nearest crimp/cable items are documented below as required.

### The queue's near neighbour: #1972 (MERGED) is a different task

**#1972, "Heat-shrink a sleeve over a crimped cable lug: one hand turns the cable, the other works a heat gun along
it"** shares the words *crimped cable* but is a **thermal, exposure-over-time** job: an electrician works a real 745 g
heat gun along an adhesive-lined sleeve, rotating the assembly or taking the gun round it, never letting the nozzle
within 3 mm of the sleeve and never charring it. Full credit needs all 8 stations recovered with adhesive showing at
both ends, the gun back in its stand switched off and every fingertip clear, held 2 s. It has **no RJ45 plug, no eight
wire order, no ratchet seat, no contact-piercing or strain-relief jaw, no 4-7 N squeeze and no axial pull-test**.
Different object, mechanism and outcome, and MERGED; it is recorded here rather than treated as a block.

### The nearest OPEN crimp item: #741 is a different task

**#741 (OPEN), "Add crimp-a-terminal-onto-a-cable-end task"** pushes **one bare wire end** into the barrel of **one
crimp terminal** held in a crimper, holds it with one hand and presses the lever down until it closes with the other,
then lets the lever spring open. It has **no eight-conductor plug, no T568B wire order or pin-1 registration, no plug
seat rail, no contact teeth, no strain-relief jaw, no ratchet force range, no pull-test and no damage caps**. Its
outcome is a closed lever; this task's outcome is a verified eight-wire termination. Same verb *crimp*, different
object + mechanism + outcome. OPEN, but not a twin.

## Nearest PRs (and why each is a different task)

| PR | State | Title | Difference |
|---:|---|---|---|
| **#741** | OPEN | Add crimp-a-terminal-onto-a-cable-end task | One bare wire into one terminal barrel, lever closed and springs open; no RJ45 plug, eight-wire order, seat, strain relief, force range or pull-test. |
| **#1972** | MERGED | Heat-shrink a sleeve over a crimped cable lug: one hand turns the cable, the other works a heat gun along it | Thermal shrinking with a heat gun, 8 heated stations and a 2 s hold; different mechanism and outcome, and MERGED. |
| **#2045** | MERGED | Crimp the board lid onto a filled foil take-out pan by folding its rim down all the way round | Food packaging folded rim, not an electrical connector. |
| **#2210** | OPEN | Draft: unlatch and relocate a plug in a crowded Ethernet panel | Unlatching/relocating an existing plug, not terminating a cable; no crimp or pull-test. |
| **#2141** | OPEN | Unplug a latched Ethernet cable from a desktop modem | Unplugging, the reverse action; no wiring, seat, squeeze or verification. |
| **#22** | MERGED | Plug both ends of a patch cord into a cable tester | Testing a ready-made cord, not crimping a plug onto a cable. |
| **#2309** | CLOSED | Solder a wire into a terminal lug with an iron and solder wire | Soldering, not a crimp; one wire, no plug or ratchet. |
| **#1411** | CLOSED | Finish a wire splice: even the wire tips, then twist the connector on clockwise until snug | Twist-on splice connector, no RJ45 plug or crimp tool. |
| **#1925** | MERGED | Clean an SC fibre connector's ferrule on a cassette cleaner, then latch it into a panel adapter | Fibre ferrule cleaning and latching, not copper CAT6 termination. |
| **#1970** | CLOSED | Load four Han-Modular modules into a HARTING hinged frame its spring holds open, then squeeze it shut until it clicks | Hinged industrial connector frame, different object and mechanism. |

## Blocked/excluded memory checked

The queue's excluded list (shower curtain #2487, soda tab #1873, Oldham/jaw coupling #2484, ball-nut arbor #2504,
peristaltic pump head, split-ring key threading, plant potting, etc.) and the existing
`benchmark-submission/tasks/*/BLOCKED.md` family were reviewed. None is this object or mechanism. No builder
`BLOCKED.md` applies to this task.

**Verdict:** no live OPEN twin, no builder block; the task is clear to build. #741 (OPEN) and #1972 (MERGED) are
documented above as the queue and the fresh search require, and both are different objects, mechanisms and outcomes.
