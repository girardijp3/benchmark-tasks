# Live twin check: luer-lock-twist-connect

**Query at launch:** fresh `gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000
--json number,title,state`, run at build time. Returned **2,537 PRs**, newest **#2538**
(465 OPEN, 1,865 CLOSED, 207 MERGED). The analyst's `BUILD_QUEUE.md` snapshot was 2,512 PRs / newest #2513
at 2026-11-17T19:45Z; the board has advanced by about 25 PRs, so this check is genuinely fresh. Every line below
was searched against this live snapshot, not taken on trust.

## Title searches (this board snapshot)

| Pattern | Hits |
|---:|---:|
| `luer` | **0** |
| `syringe` | 12 |
| `vial` | 7 |
| `adapter` | 4 |
| `twist` | 18 |
| `connect` | 33 |
| `bayonet` | 3 |
| `thread` | 14 |
| `reconstit` / `draw 10` / `luer lock` | 0 |

No PR title contains the word "luer" at all. No OPEN task combines a luer-lock syringe tip, a vial adapter, and a
twist-to-lock outcome.

## Nearest PRs and why they are not this task

* **#1095 MERGED** "Add draw-10-ml-from-a-vial-into-a-syringe task" — the queue's listed near neighbour. Same two
  products (a syringe and a vial), but the scored step is **drawing liquid by pulling the plunger to a 10 mL mark**.
  This task never moves the plunger and never measures a volume: it **connects** a 6 mm luer-lock tip to a 20 mm
  vial-adapter port by seating and twisting ~1.5 turns until locked. A different step, not a twin.
* **#1013 MERGED** "Scrub the IV connector's hub for 15 seconds before attaching the flush syringe" — the scored
  action is a 15 s scrub; the syringe attachment is context, not the mechanism or outcome.
* **#1003 CLOSED** "Connect a 24/29 expansion adapter cone to its corresponding socket" — a smooth ground-glass cone
  joint (friction fit, no thread, no lock), a different object and mechanism.
* **#34 MERGED** "Lock a bayonet cap into its socket" — a quarter-turn bayonet, not a threaded luer-lock collar; no
  vial or syringe.
* **#706 CLOSED** "Add twist-lock-plug task" — a twist-lock plug, not a syringe/adapter connection.
* **#793 OPEN** "Add twist-a-glass-tube-into-a-rubber-stopper task" — pushing a tube through a rubber stopper, a
  friction fit with no 1.5-turn thread and no cross-thread/loose-connection outcome.
* **#1711 CLOSED** / **#1489 CLOSED** — removing a spring-loaded / multistage bayonet sleeve; removal, not connection.
* **#1576 MERGED** "Fit an O-ring onto a hydraulic plug ... seat it square and untwisted" — an O-ring on a thread;
  the outcome is explicitly *untwisted*, the opposite of this task.
* **#892 OPEN** "Load a filled syringe into a syringe pump" — a syringe pump, no luer connection.
* **#2012 CLOSED** "Draw a 10 mL syringe's plunger back to the 5 mL line" — plunger positioning, no connection.
* **#2128 MERGED** "Load an anaesthetic cartridge into a dental syringe" — a dental cartridge, different object.
* **#2031 OPEN** "IV bag spiking and drip-chamber priming" — spiking a bag, not a luer lock.

## Result

**No live OPEN task with the same object (6 mm luer-lock syringe tip + 20 mm vial adapter), mechanism (seat the male
tip in the female port and twist a cumulative ~1.5 turns against a 0.3-0.6 N m lock), and outcome (locked luer
connection, no cross-thread, no loose connection).** No CLOSED or MERGED exact-final twin exists either; #1095 is a
different step and is documented above. **Clear to build — proceed.**
