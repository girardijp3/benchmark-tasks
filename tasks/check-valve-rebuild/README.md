# Rebuild a 20 mm brass spring check valve

A bench task from a plumber's day. A 20 mm brass spring check valve sits in a small aluminium bench fixture at
x 0.25, y 0, its axis vertical and its threaded cap seated. Its seat is bare; the replacement poppet/disc (rubber
seal face down, guide stem up) and the compression spring wait in pockets beside it. The robot has to unscrew and
lift the cap off, seat the new poppet on the seal, fit the spring on top, thread the cap back down, then press the
inlet and outlet test buttons to prove flow passes one way and not the other. The instruction states only the goal:
that the poppet must face a certain way, that the spring sits between it and the cap, that the cap must be threaded
not forced, and that the two button presses mean different things are the robot's to work out.

**What is new against the board (fresh query 2026-11-17T19:50:53Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned 2,512 PRs, newest #2513 (442 OPEN, 1,863 CLOSED, 207
MERGED). No live OPEN task has this object + mechanism + outcome. #1286 (CLOSED) compresses/locks a spring cartridge
(no valve body, seat, poppet, cap or flow test); #1905/#2382 (OPEN) fit shower/faucet cartridges; #2418/#2421 (OPEN)
close spray/drainage valves; #1246 (CLOSED) tests a TPR relief valve. A different valve or cartridge is not the same
task. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `check_valve_rebuild.py` | The `yam_arena` task file: scene, the free parts and their declared carry, the seat/thread/flow mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/valve.usda` | The welded fixture, brass body, seat ledge, inlet/outlet collars, the two test buttons and the three parts' staging pockets/peg |
| `assets/cap.usda`, `assets/poppet.usda`, `assets/spring.usda` | The three real, gravity-free loose parts (26 mm cap, 16 mm poppet, 14 mm spring) |
| `build_assets.py` | Writes the four USD files (pxr; boxes and cylinders, colliders on the parts) |
| `test_check_valve_rebuild.py` | Offline checks (19): every scoring outcome and cap, the 1 s hold and its restart, gripper-clear, real sizes, all demo waypoints reachable, the plan and its timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the sources, and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame-by-frame top/close-up and `data.contact` evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |

## Real product and values

The product class is a **20 mm (3/4 in) brass spring check valve**: a hex brass body with female inlet/outlet, an
internal seat, a poppet/disc with a soft seal face, a compression spring and a threaded bonnet/cap that closes the
body and sets the spring preload. The job is a rebuild: the cap comes off, the worn internals are replaced in order
and orientation, the cap is threaded back and the valve is flow-tested. Representative catalogue numbers for a
20 mm brass spring check valve give a 4-7 N poppet spring force and a 0.3-0.6 N m cap torque; the true body/cap/disc
sizes are listed below. Sources and the measured/estimated split are in `REFERENCE.md`.

**Measured (product class):** 20 mm nominal port; brass body; internal seat; poppet/disc with elastomer seal;
compression spring; threaded cap; one-way flow; 4-7 N spring, 0.3-0.6 N m cap.

**Estimated (each in `REFERENCE.md` and in `task.json` fidelity):** cap 26 mm across flats x 14 mm; poppet disc
16 mm x 3 mm with a 14 mm rubber face and a 5 mm stem; spring 14 mm OD x 24 mm free; seat bore 12 mm at z 0.062;
body 28 mm dia x 40 mm tall on a 90 x 90 x 50 mm fixture; part masses 30/8/4 g; a 60 degree effective cap thread
and a 6 mm button press. No size, mass or force is scaled down.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The valve axis is at **x 0.25, y 0**; the brass body runs from z 0.050 to 0.090 with its seat
ledge at **z 0.062**. The new poppet waits in a pocket at **x 0.25, y 0.045**, the spring at **x 0.25, y 0.070**,
and the cap (seated at reset) is parked on a peg at **x 0.25, y -0.045**. The two test buttons are on the fixture's
front edge at **x 0.218, y +-0.025**; a gripper presses one 6 mm down to apply inlet (forward) or outlet (reverse)
pressure. The fixture is fixed to the bench; both arms start at READY. Every grasp point lies 0.07-0.10 m above the
table and within 0.40 m of an arm base.

## Mechanism

The three loose parts are real gravity-free rigid bodies. While a gripper's fingers close across a part's grasp
point the part follows the gripper (a declared carry, the same family device as the cantilever box's declared latch
detents and lid coupling); the robot must still reach, close on, carry and place every part. Orientation is a real
consequence of the gripper's rotation: a poppet held upright has its rubber seal face down onto the seat; one held
flipped 180 degrees has the face up and leaks. The cap's thread is a declared 60 degree twist (the left wrist reaches
70 degrees here) with an 8 degree margin in the demo: a cap lowered without the twist sets `cross_thread`. The seat
and seal are real geometry -- the poppet's 14 mm rubber face meets the ledge at z 0.062.

The two test buttons stand in for a pressure source (no fluid is simulated). Pressing the inlet button applies
forward pressure: with the cap threaded, spring present and poppet upright, the poppet lifts 6 mm off its seat and
`forward_done` sets. Pressing the outlet button applies reverse pressure: the poppet is driven onto its seat and
`reverse_done` sets only if it seals (correct orientation, spring present, cap threaded). A wrong state at either
button sets `leak`. That is the real seat/seal check.

## Demo

Open loop, known geometry, one arm (the left; the right stays home). The left arm: (A) grips the seated cap, lifts it
clear and parks it on the peg -- the body is now open; (B) grips the poppet by its rim/stem, carries it over the bore
and lowers it seal-face-down onto the seat; (C) grips the spring, seats it on the poppet; (D) picks the cap back up,
twists the thread in 70 degrees over the bore and screws it down; (E) presses the inlet button (forward flow, poppet
lifts) and then the outlet button (backflow blocked, poppet stays seated), and goes home. Every approach is from
above with the fingers closing across the part.

**MuJoCo 1.0 at step 1092 of 1,200 (54.60 s after the settle, t=55.10 s in the task log).** Step timeline: body
open 122, parts seated 571, cap+forward 881, backflow sealed 973; full 1 s hold completes at 1092. Final: cap
twist 70 degrees seated at z 0.8445, poppet at z 0.8112 (0.2 mm over the seat), spring seated at z 0.8257, no cap
latch, both grippers home. Idle 0; body open 0.25; parts seated 0.5; cap+forward 0.75; reversed poppet, missing
spring, cross-thread, dropped part and leak/backflow each cap at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| body opened | cap lifted clear and set aside | 0.25 |
| parts seated | poppet upright on the seat and spring seated on it | 0.5 |
| cap + forward flow | cap threaded down and the inlet button lifts the poppet | 0.75 |
| backflow sealed | outlet button leaves the poppet seated and sealed, held 1 s, valve intact, grippers clear and home | **1.0** |

Caps (0.5 ceiling): reversed poppet, missing spring, cross-threaded cap, a dropped part, leak/backflow. Idle 0.

## Media

`demo.mp4` (Isaac top view only, 640x360, 617,004 bytes, final 1.0 frame held 3 s), `demo_top.gif` (480x270,
1,755,786 bytes) and `setup.png` (640x360, 28,184 bytes) are in `benchmark-media-check-valve-rebuild/`, together
with `dream.mp4` (completed Dream-1 top camera, 320x176, 664,395 bytes).

## Gate status

**MuJoCo 1.0 at step 1092, Isaac 1.0 at step 1092, 19/19 offline tests pass.** Dream-1: **0.0 at 2,400 steps**
(2,401 action rows, completed). Fields and consistency pass; `task.json` media M-74SEKC. No PR, upload or push was made.
