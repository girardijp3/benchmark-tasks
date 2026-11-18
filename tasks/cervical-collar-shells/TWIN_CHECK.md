# Live twin check: cervical-collar-shells

**Query at launch:** 2026-11-17T21:54:36Z, fresh
`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state`.
Returned **2,538 PRs**, newest **#2539** (466 OPEN, 1,865 CLOSED, 207 MERGED). This is a fresh query, later
than the analyst's 19:45Z snapshot in `submission_tools/filter/BUILD_QUEUE.md` (2,512 PRs, newest #2513),
so it includes all newer PRs.

## Title searches (this board snapshot)

| Pattern | Hits | Notes |
|---|---:|---|
| `cervical` | 0 | no cervical/spinal-collar task at all |
| `collar` | 6 | sprinkler trip collars, dumbbell-plate collars, opaque-collar transport, tether collar, screw puller — none a neck collar |
| `manikin` | 17 | care/clinical manikin tasks (meds, inhaler, glasses, temperature, shave, eye drops, airway, AED, feeding) — none a collar fit |
| `neck` | 5 | necktie, flask neck, ampoule neck, water-cooler neck-down, weld-neck flanges |
| `jigsaw` | 2 | #2052 OPEN bevel setting, #1719 CLOSED "Jigsaw final piece" |
| `tourniquet` / `first aid` / `spinal` | 2 / 0 / 0 | different first-aid object |

## Nearest PRs and why they are not this task

* **#1719 CLOSED** "Jigsaw final piece" — the rank-27 queue's listed near neighbour, carried forward as
  "different". It is a jigsaw-puzzle final piece: different object, different mechanism (piece insertion),
  different outcome. CLOSED, and not a collar or a manikin task. **Different.**
* **#1026 OPEN** "Tighten a tourniquet's windlass to the green and lock it" — a limb tourniquet windlass, a
  different first-aid object and mechanism (rotational windlass vs two rigid shells with pressed side tabs).
* **#1507 CLOSED** "Give a manikin two breaths with a bag-valve mask, lifting the jaw into the mask" — airway
  ventilation on a manikin; no collar, shells or tabs.
* **#1326 CLOSED** "Open a manikin's airway with an oral airway without pushing the tongue back" — airway
  adjunct placement, not collar fitting.
* **#2467/#2464/#2322/#2284/#2275 OPEN** — oral syringe, inhaler+spacer, reading glasses, temporal temperature,
  cheek shave on a manikin. All use a manikin but none seats a two-piece collar or presses side tabs.
* **#2004 CLOSED / #2007 CLOSED / #1981 MERGED / #1912 MERGED** — glasses, infant move, eye drop, ear temperature:
  different manikin tasks.
* **#2388/#2005 CLOSED** "impulse sprinkler trip collars" — irrigation hardware, unrelated.

No OPEN or CLOSED/MERGED PR combines the object (**two-piece rigid cervical collar on a manikin**), the
mechanism (**seat a front chin shell and a back occipital shell, then press 30 mm side tabs**) and the outcome
(**collar correctly fitted at 4-7 N, held 1 s**). The board has no cervical-collar task under any title.

## Result

**Clear to build.** No live OPEN twin; #1719 (the queue's listed nearest) is a CLOSED jigsaw task and is
unrelated; none of the 17 manikin PRs is a collar fit. A different manikin, a different first-aid item or a
different tool is not the same task.
