# Centre an 80 mm magnetic dust boot on a 65 mm spindle nose, press it to the shoulder and let go

A routine machine-shop task. A steel spindle nose (65 mm diameter, with a shoulder at its base) sits at x 0.30, y 0.
An 80 mm magnetic dust boot — oil-resistant NBR elastomer with a moulded-in magnet ring, 65 mm bore, 40 mm tall —
waits in a staging pocket at x 0.25, y -0.06. The robot must pick the boot by its small central boss, bring its bore
**concentrically** over the nose (the embedded magnets must centre, not grab an off-centre bite of the rim), press it
down against the shoulder with a **4-7 N** hand force, release it, and leave it seated with both grippers clear and
home. How close is concentric, how far the press must travel, and what "seated" means are the robot's to work out
from the instruction.

**What is new against the board (fresh query 2026-11-17T21:28:30Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned 2,533 PRs, newest #2534 (461 OPEN, 1,865 CLOSED, 207
MERGED). No live OPEN task has this object + mechanism + outcome. **#2212 (OPEN), "Fit a shaft-end magnetic encoder
and verify its installation"**, the rank-23 near neighbour, is a different task: a D-keyed magnetic *encoder* plus
a sensor bracket with a field check — different object (sensor target, not an elastomer dust seal), different
mechanism (keyed carrier + bracket + catch + field check, not concentric magnet-centring and a 4-7 N shoulder
press) and different outcome (verified encoder, not a seated dust boot). Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `magnetic_dust_boot.py` | The `yam_arena` task file: scene, boot and spindle, alignment/press/seat mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/spindle.usda` | The welded spindle fixture: 65 mm nose, shoulder boss and staging pocket (visual-only) |
| `assets/boot.usda` | The real 80 mm dust boot: elastomer body, 65 mm bore, embedded magnet ring, alignment tick and the small grasp boss (the only collider) |
| `build_assets.py` | Writes both USD files with pxr (boxes and cylinders) |
| `test_magnetic_dust_boot.py` | Offline checks (18): the full scoring ladder, the 1 s hold and its restart, gripper-clear, all three caps, real sizes, warm-start IK reach of every waypoint, demo order and timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its sources, measured vs estimated numbers, and the explicit routine-importance case |
| `PASS_THROUGH_QA.md` | Frame-by-frame top view and `data.contact` evidence that nothing passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query, #2212's difference, and the nearest PRs |
| `make_media.py` | Renders the internal top-view media into `benchmark-media-magnetic-dust-boot/` |

## Real product and values

The product class is an **80 mm magnetic spindle-nose dust boot / dust shoe**: an oil-resistant elastomer (NBR)
body with an embedded magnet ring that centres on a steel machine-tool spindle nose and is pressed against the
shoulder to keep chips and coolant out of the taper. Hutchinson's *Magnetic Rings* catalogue documents the
construction (metal insert bonded to magnetic rubber, NBR ferrite, oil & grease resistant, machine-tool and
high-speed-spindle applications, including an 80 mm OD / 64 mm ID ring); 80 mm magnetic dust-boot/dust-shoe
products from PwnCNC, Onefinity and Printables confirm the class. Sources and the measured/estimated split are in
`REFERENCE.md`.

**Measured (product class):** oil-resistant elastomer body with embedded magnet ring; 80 mm OD ring class;
4-7 N press (brief); machine-tool spindle application.

**Estimated (each in `REFERENCE.md` and `task.json` fidelity):** 80 mm OD x 40 mm boot, 65 mm bore and nose,
magnet ring band, 65 mm nose 45 mm tall on a 100 mm shoulder boss, 90 g boot, 6 mm concentric and 5 mm seat
tolerance, 4 mm overpress limit, 35 mm press stroke. No size, mass or force is scaled down.

## Routine importance (explicit case)

This is a routine, high-population action, not an occasional repair. BLS OEWS lists about **287,000 machinists**
(51-4041) and **422,000 industrial machinery mechanics** (49-9041) — roughly **709,000 workers**, over the
500,000 bar on machine operators/maintenance alone; the broader maintenance pool (49-9071) is about 1.62 million.
A spindle-nose boot is fitted or renewed at every chuck/fixture change, at each setup, after spindle cleaning and
at scheduled service — commonly several times per shift per machine. An off-centre grab, a proud boot or a forced
jam lets chips/coolant into the taper, causes runout or a bound spindle, and costs scrapped parts and downtime.
Sources and detail in `REFERENCE.md`.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The spindle axis is at **x 0.30, y 0**; its shoulder face is at **z 0.050** and its 65 mm nose runs
from **z 0.050 to 0.095**. The boot starts in its pocket at **x 0.25, y -0.06**, centre at **z 0.020**; when seated
its centre is at **z 0.070** and its bottom face on the shoulder. Carry height is z 0.150 (clear of the nose and
inside YAM reach). Both arms start at READY; only the left arm moves in the demo.

## Mechanism

The boot is a real gravity-free rigid body. While the left gripper's pads close across the boot's central grasp boss
(the boot's only collider — an 80 mm body would block a small grip), the boot follows the gripper (a declared carry,
the family's detent device). The concentric test is real geometry: the bore centre must stay within **6 mm** of the
spindle axis. The press is a real closed-grip downward stroke of at least **90 % of a 35 mm stroke**; the seat is the
boot bottom face within **5 mm** of the shoulder, centred. Driving the boot more than **4 mm past** the shoulder is
a forced press (a pass-through of the solid stop). The 4-7 N force is stated in fidelity, not integrated.

## Demo

Open loop, known geometry, one arm (the left; the right stays home). The left arm: (A) picks the boot by its boss,
(B) carries it at z 0.150 to the spindle, descends and centres the bore on the nose, (C) presses through the full
stroke until the bottom seats on the shoulder, (D) releases and goes home. Every approach is from above with the
fingers closing across the boss.

**MuJoCo 1.0 at step 354 of 1,200 (17.70 s).** Step timeline: aligned 179, pressed 228, seated 228, released 253;
the full 1 s hold completes at 354. Final: boot centred on the nose (dxy ~0.002 m) with its bottom on the shoulder,
both grippers home and clear. Worst IK track error ~22 mm during the high carry (the mechanism states are geometric,
so tracking error there does not affect the seat). Idle 0; aligned 0.25; pressed 0.5; seated 0.75; magnets grab
off-centre, boot left unseated or press forced past the shoulder caps at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| boot aligned | bore centre within 6 mm of the spindle axis on the approach | 0.25 |
| press applied | closed-grip downward stroke reaches 90 %+ of the 35 mm stroke, centred | 0.5 |
| boot seated | bottom face within 5 mm of the shoulder, centred | 0.75 |
| released + held | released after the seat; held 1 s, seated and centred, grippers clear and home | **1.0** |

Caps (0.5 ceiling): magnets grab off-centre, boot left unseated, press forced past the shoulder. Idle 0.

## Media

`demo.mp4` (Isaac top view only, 640x360, 239,165 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 5 fps,
1,304,623 B) and `setup.png` (640x360, 28,870 B) are in `benchmark-media-magnetic-dust-boot/`, together with
`dream.mp4` (completed Dream-1 top camera, 320x176). Each PR file is under 1.87 MB; `task.json` media M-44C4FX.

## Gate status

**MuJoCo 1.0 at step 354; Isaac 1.0 at step 354** (`adhoc_14d8794a.json`); 18/18 offline tests pass.
Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed). Fields and consistency pass; `task.json` media M-44C4FX.
No PR, upload or push was made.
