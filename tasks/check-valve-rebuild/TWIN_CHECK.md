# Live twin check: check-valve-rebuild

**Query at launch:** 2026-11-17T19:50:53Z, fresh
`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state`.
Returned **2,512 PRs**, newest **#2513** (442 OPEN, 1,863 CLOSED, 207 MERGED). This matches the
analyst's re-check in `submission_tools/filter/BUILD_QUEUE.md` (2,512 PRs, newest #2513), verified
again at launch rather than taken on trust.

## Title searches (this board snapshot)

| Pattern | Hits |
|---|---:|
| `check valve` / `one-way` / `non-return` / `backflow` | 4 (none a valve) |
| `valve` | 34 |
| `poppet` / `seat` / `seal` | 76 (all generic seating/sealing, no check-valve rebuild) |
| `spring` / `compress` / `cartridge` | 67 |
| `rebuild` / `service` / `overhaul` | 1 (#1647 counterweighted service arm) |
| `threaded cap` / `bonnet` | 2 (#1079 jug cap, #1002 car bonnet) |
| `flow test` / `verify flow` / `prime` | 5 |

## Nearest PRs and why they are not this task

* **#1286 CLOSED** "Add compress-and-lock-spring-cartridge" — a spring cartridge compressed and locked; a
  different object (no valve body, seat or poppet) and a different outcome (no cap thread, no one-way flow test).
  This is the queue's listed near neighbour; CLOSED and not the same mechanism.
* **#1905 OPEN** "Push a shower valve's cartridge flush against its O-rings and hold it there while sliding the
  U-clip down past its ears" — a shower mixing cartridge and U-clip, no poppet/spring/cap or flow test.
* **#2382 OPEN** "Replace a faucet's cartridge so hot stays on the left" — a faucet cartridge, orientation by
  temperature, not a check valve.
* **#2418 OPEN** "Shut off a pre-rinse spray valve left locked on over a dish sink" — closing a spray valve.
* **#2421 OPEN** "Empty a urinary drainage bag ... then close the valve and clip the tube back" — a drainage bag.
* **#1246 CLOSED** "TPR relief valve test and report task" — a relief-valve test/report, different object and
  outcome (no rebuild).
* **#1934 CLOSED** "Lock out a ball valve ... clamshell lockout" — a ball-valve lockout, no rebuild or flow test.
* **#1711 CLOSED** "Take a spring-loaded bayonet coupling sleeve off its shaft ..." — a bayonet sleeve, not a valve.

## Result

**No live OPEN task with the same object (20 mm brass spring check valve), mechanism (remove threaded cap, seat a
new poppet and spring in the correct orientation, thread the cap back) and outcome (one-way flow verified against a
real seat/seal).** No CLOSED/MERGED exact-final twin either. A different valve, cartridge, colour or cap is not a
difference. **Clear to build** — proceed.
