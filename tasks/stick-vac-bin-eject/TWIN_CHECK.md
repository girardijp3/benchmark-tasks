# Live-twin check — stick-vac bin eject (rank 9 of BUILD_QUEUE.md)

## Fresh query

Run immediately before the build:

```
date -u +%Y-%m-%dT%H:%M:%SZ                     -> 2026-11-17T19:49:30Z
gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 \
    --json number,title,state
```

Result: **2,512 PRs, newest #2513**; states 442 OPEN / 1,863 CLOSED / 207 MERGED.
(The queue's own 2026-11-17T18:38:53Z snapshot had 2,501 PRs, newest #2502; this check is fresher.)

Policy (BUILD_QUEUE.md line 9): a live OPEN task with the **same object + mechanism + outcome** is a stop;
a different tool alone is not a difference.

## Closest live OPEN task — judged NOT a twin

| PR | State | Title | Difference from this task |
|---|---|---|---|
| **#1262** | OPEN | "Add empty-a-robot-vacuums-dustbin-over-the-waste-box task" | **Robot** vacuum (Roomba-class), not a cordless **stick** vac. Mechanism: lift the bin off the robot by a grip fin, hold an orange release button on the lid so **bottom doors** swing open, then seat it back. This task: press a red release **button on the stick-vac body**, pull the 0.4 L bin **forward off a cradle**, open a **single hinged bottom flap**, slide it back until the button clicks. Different product, different release, different bin geometry. The **outcome** (empty dust into a waste box and refit) is similar, but object and mechanism both differ, so the triple match is not met. |

The #1262 task also has no eject-button press step and no slide-off-cradle travel; its "click" is re-seating on the robot, not a sprung button returning.

## Other vacuum/dust PRs found

- **#1235 CLOSED** — "Empty a vacuum's dust cup through its bottom flap into the bin and click it back in" — bottom-flap cup, different mechanism; not live.
- **#2334 OPEN** — "Add vacuum-bag-swap task" — canister vacuum **dust bag** collar swap; different object and outcome.
- **#2103 CLOSED** — "Open a robot vacuum's brush door to clean its rollers: squeeze both catch buttons..." — brush door, different object.
- **#2466 OPEN** — "Add vacuum cup and restrictor assembly through a mounting plate" — industrial vacuum-gripper subassembly; different object/outcome.
- **#2033 OPEN** — "Robot vacuum" (title only, no body) — not this object/mechanism.
- **#687 / #676** — "Vacuum desiccator" — laboratory desiccator, unrelated.

The queue's own note for this row is **live twin "#2161 CLOSED walking stick; different" — clear**. No new live OPEN same-object + same-mechanism + same-outcome task appeared between the queue snapshot and this build.

## Prior measured block avoided

`benchmark-submission/tasks/vacuum-canister-empty/BLOCKED.md` (canister dust-cup empty, same broad outcome) was dropped after a spring **pin through an ear hole** jammed at 1 mm of 15 mm travel and the cup never cleared the body. This build deliberately uses **no pin and no hole**: the red button is a clear-travel 10 mm prismatic press on a thin bracket, and the dust bin is a free body that slides forward off a low open cradle. The reach path was checked with `static_ik.py` / `per_wp.py` before the MuJoCo gate (18/18 waypoints at 0.0 mm / 0.0 deg), so the pin/ear failure cannot recur.

## Decision

**Keep and build.** No stop condition. This file is the record for the Isaac finisher; no PR, push or upload was made.
