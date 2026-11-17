# Live twin check — DAC plug into switch cage

Fresh query run for this build (2026-11-17):

```
gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 \
  --json number,title,state,body,url,closedAt,mergedAt,labels
```

Result: **2,523 PRs, newest #2524** — 451 OPEN, 1,865 CLOSED, 207 MERGED. (The queue's 19:45Z snapshot had 2,512 PRs, newest #2513, 442 OPEN / 1,863 CLOSED / 207 MERGED; the 11 newer PRs #2514-2524 contain no DAC/switch-plug task.)

Stop rule used: stop on a **live (OPEN) same-object + mechanism + outcome** task, or on a **closed-final importance twin** (a CLOSED "Not accepted: final" PR in the same importance category that is the same object/mechanism/outcome). A different action is not a twin — as the parent noted, **#14 (CLOSED, dress three cables into a comb) is a different action** from inserting a plug.

## Task being checked

Object: a 40 mm DAC/SFP-style copper plug and a switch port cage. Mechanism: keyed insertion, a spring latch that clicks, a strain-relief boot seated in the comb, and a retention tug (4-7 N push). Outcome: plug latched in the target port, boot in the comb, tug holds. Importance: Impact on AI timelines.

## Live OPEN candidates — none is a same-object+mechanism+outcome twin

| PR | State | Title | Why not a twin |
|---:|---|---|---|
| #2210 | OPEN (draft) | Unlatch and relocate a plug in a crowded Ethernet panel | Draft, no `task.json`; action is unlatch/transfer/re-latch among six *connected* RJ45 plugs, not fresh DAC insertion with boot + tug. |
| #2219 | OPEN | Connect two workstation ports and route the slack | Plain patch-cord port connection; no latch click, no boot/comb, no tug. |
| #2141 | OPEN | Unplug a latched Ethernet cable from a desktop modem | Removal, not insertion; RJ45 not DAC; different outcome. |
| #2501 | OPEN | Install a plug-on breaker in a load center | Different object/mechanism (bus stab), electrical panel. |
| #2514 | OPEN | Fit a thermostatic head onto a radiator valve | Different product (snap onto valve). |

No OPEN PR inserts a DAC/SFP plug into a switch cage with a latch click, boot-in-comb and tug retention.

## Closed-final candidates in the adjacent data-centre / connector cluster

| PR | State | Title | Why not a twin |
|---:|---|---|---|
| #1516 | CLOSED "Not accepted: final" | Plug a fibre patch cable's SFP end into a network card | Category "Impact on jobs"; **fibre SFP into a NIC, not a DAC copper plug into a switch**; its own fidelity states "No latch, click, optics or insertion force"; no boot, no comb, no tug; plain 0.13 mm peg-in-hole. |
| #2143 | CLOSED "Not accepted: final" | Unlatch and remove a QSFP28 optical transceiver from a 1U switch | Same importance (AI timelines) and same switch/cage family, but the **action is removal** (pull-tab cams the pawls open, module comes out), the opposite outcome. |
| #1726 | CLOSED "Not accepted: final" | Hand off and insert a duplex fibre connector | Fibre SC handoff + keyed adapter insertion; no latch/boot/comb/tug. |
| #253 / #1925 | CLOSED / MERGED | Clean a fibre connector, then plug it in | Cleaning + SC adapter insertion; different object and outcome. |
| #640 | CLOSED "Not accepted: final" | Swap a failed optical transceiver, keeping cage and optic capped | Swapping/capping an optic; different action. |
| #14 | CLOSED "Not accepted: final" | Dress three cables into a comb | **Different action** (comb dressing of three cables), as the parent clarified; no plug insertion, latch or tug. |

Each is adjacent but differs in object, mechanism or outcome. In particular the benchmark already contains "remove transceiver from switch" (#2143) and "plug SFP into NIC without latch" (#1516); this task's distinctive combination — **DAC copper plug + switch cage + latch click + boot in comb + tug retention + 4-7 N push** — is not covered by either.

## Decision

**No live same-object+mechanism+outcome task and no closed-final importance twin. No stop.** The queue's rank-6 assessment ("no live exact", live twin #14 different) is confirmed against the fresh board. Build proceeded.
