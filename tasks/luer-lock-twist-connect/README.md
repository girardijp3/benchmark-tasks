# Connect a 6 mm luer-lock syringe tip to a 20 mm vial adapter

A bench task from a nurse's or pharmacy technician's day. A 20 mm-neck vial sits in a small aluminium bench fixture at
x 0.28, y 0, its adapter's female luer-lock port upright at z 0.100. A 10 mL luer-lock syringe with a 20 mm barrel
stands in a holder beside it. The robot picks the syringe up by the barrel, seats its 6 mm male luer tip down in the
female port, twists the collar a cumulative ~1.5 turns (0.3-0.6 N m at the real bench) until the lock is made, lets
go and goes home. The instruction states only the goal: that the tip must be seated before it is turned, that
"locked" means about one and a half turns rather than a quick nudge, and that turning early or stopping short are
both wrong are the robot's to work out.

## Why this is routine, not occasional (importance case)

This is core, repeated handling in two large occupations, not a rare trick. BLS/O*NET lists **3,465,400 registered
nurses** (SOC 29-1141) and **471,200 pharmacy technicians** (SOC 29-2052) in 2025 — about **3.9 million US workers**,
well past the 500,000 threshold. Nurses "administer medicines and treatments" and run immunization clinics;
pharmacy technicians "measure amounts of medication", prepare intravenous medications and, in many states, administer
vaccines. Reconstituting a drug, drawing up a vaccine or building an IV admixture means attaching a luer-lock syringe
to a vial adapter many times a shift. A tip twisted before it is seated cross-threads and ruins a sterile connection;
a collar released early leaves a loose joint that can disconnect and spill a drug. (Sources: BLS Occupational Outlook
Handbook, 2025 employment figures, pages for Registered Nurses and Pharmacy Technicians.)

**What is new against the board (fresh live query at build time).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned **2,537 PRs, newest #2538** (465 OPEN, 1,865 CLOSED, 207
MERGED). No PR title contains "luer" and no OPEN task has this object + mechanism + outcome. **#1095 (MERGED) draws
10 mL from a vial into a syringe** — a different step (pulling the plunger to a volume); this task never moves the
plunger. #1013 scrubs an IV hub for 15 s; #1003 fits a smooth glass cone; #34/#706/#1711 are bayonet/twist-lock plugs,
not a threaded luer connection; #793 pushes a glass tube into a stopper. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `luer_lock_twist_connect.py` | The `yam_arena` task file: scene, the free syringe and its declared carry, the seat/twist/lock mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/adapter.usda` | The welded, visual-only bench fixture, 20 mm vial, adapter skirt and female luer-lock port with its bore and thread ridges |
| `assets/syringe.usda` | The real, gravity-free 10 mL syringe: 20 mm barrel, plunger, luer-lock collar, 6% male luer tip and a red index mark |
| `build_assets.py` | Writes the two USD files (pxr; boxes and cylinders, collider on the barrel) |
| `test_luer_lock_twist_connect.py` | Offline checks (19): every scoring outcome and cap, the 1 s hold and its restart, real sizes, the YAM reach gate, the 1.5-turn plan and both arms' waypoint reach |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the sources, and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame-by-frame top-view and `data.contact` evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |
| `make_media.py` | Renders the internal top-view `setup.png`, `demo.mp4` and `demo_top.gif` |

## Real product and values

The product class is a **standard luer-lock connection** (ISO 80369-7, formerly ISO 594): a male luer tip with a **6%
(1:20) taper**, surrounded by a **threaded locking collar**, and a female luer-lock port. The 20 mm size is the vial's
neck finish; the adapter carries the female luer port. Seating the tapered tip and rotating the collar develops the
lock; the brief's bench torque is **0.3-0.6 N m** and the lock takes about **1.5 turns** when done deliberately. Real
sizes, masses and sources are in `REFERENCE.md`. No size, mass or torque is scaled down.

**Measured (product class / brief):** 6 mm-class male luer tip, 6% taper; ISO 80369-7 luer-lock; 20 mm vial neck;
20 mm syringe barrel; female luer port on the adapter; 0.3-0.6 N m lock torque; ~1.5 turns.

**Estimated (each in `REFERENCE.md` and in `task.json` fidelity):** 10 mL syringe with a 20 mm x 46 mm barrel, an
11 mm collar and an 8 mm tapered tip; a 20 mm x 48 mm vial and a port lip at z 0.100; barrel mass 12 g; a declared
500-580 degree lock window for the nominal 540 degree (1.5-turn) twist. The torque is stated in setup/fidelity, not
integrated into the dynamics.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The vial adapter's port axis is at **x 0.28, y 0**, lip at **z 0.100** (the 0.10 m work height). The
syringe starts upright at **x 0.28, y 0.070**, barrel centre **z 0.056**, its tip bottom just above the holder. With
the tip seated the barrel centre is at **z 0.141**. Every grasp and work point is inside the 0.30 m forward reach gate
and 0.10-0.16 m above the table; both arms start at READY.

## Mechanism

The syringe is a real gravity-free rigid body. While a gripper's fingers close across its 20 mm barrel the syringe
follows the gripper (a declared carry, the same family device as the cantilever's lid detents); the robot must still
reach, close on, carry, seat and twist it. The seat is real geometry: the tip bottom must be coaxial with the port and
at z 0.141 within 6 mm. The twist is counted from the syringe's own unwrapped rotation about the port axis. A YAM
wrist cannot turn 540 degrees in one grip, so the demo regrips twice and the count carries across the three half-turn
grips — the way a hand regrips a luer collar. Turning while the tip is off the port sets `cross_thread`; releasing to
a clear, finished state before the lock is made sets the loose-connection cap; releasing off the bench sets `dropped`.
The 0.3-0.6 N m torque is the real bench value read with a gauge, not a simulated contact force.

## Demo

Open loop, known geometry, one arm (the left; the right stays home). The left arm: (A) winds its wrist to the twist
start yaw while the syringe is untouched, then picks the syringe up by the barrel; (B) carries it over the adapter and
lowers the male tip into the female port; (C) twists half a turn, releases, winds back, regrips, and repeats twice more
for a cumulative 1.5 turns; (D) lets go and goes home. Every approach is from above with the fingers closing across
the barrel. Regripping is part of the skill, not a shortcut: the twist counter only advances while the tip is seated.

**MuJoCo 1.0 at step 704 of 1,200 (35.20 s after the settle, t=35.70 s in the task log).** Step timeline: picked up
126, tip seated 172, thread engaged (0.5 turn) 372, locked 583; the 1 s hold completes at 704. Final: syringe seated
and free at z 0.8891, cumulative twist +538 degrees (1.49 turns) inside the 500-580 degree lock window, no
cross-thread, both grippers home. Idle 0; picked up 0.25; seated 0.5; engaged 0.75; cross-thread, loose connection or
a drop each cap at 0.5. Two further clean runs and one run with the staged syringe moved 3 mm and tilted 3 degrees
also scored 1.0 at step 704.

## Scoring

| Step | Done when | Points |
|---|---|---|
| syringe picked up | lifted clear of its staging holder | 0.25 |
| tip seated | male tip coaxial in the female port at the seated height | 0.5 |
| thread engaged | cumulative seated twist >= 0.5 turn | 0.75 |
| luer locked | cumulative seated twist ~1.5 turns, held 1 s, syringe intact, both grippers clear and home | **1.0** |

Caps (0.5 ceiling): cross-thread (turning before the tip is seated), a loose connection (clear finished state before
the lock is made), a syringe dropped off the bench. Idle 0.

## Media

`demo.mp4` (Isaac top view only, 640x360, 396,076 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
1,635,867 B) and `setup.png` (640x360, 28,088 B) are in `benchmark-media-luer-lock-twist-connect/`, together with
`dream.mp4` (completed Dream-1 top camera, 320x176, 637,202 B). All PR files are under 1.87 MB; `task.json` media M-U6YEEW.

## Gate status

**MuJoCo 1.0 at step 704; Isaac 1.0 at step 704** (`adhoc_e4234126.json`); 19/19 offline tests pass.
Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed). Fields and consistency pass; `task.json` media M-U6YEEW.
No PR, upload or push was made.
