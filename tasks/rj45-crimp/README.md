# Crimp an RJ45 plug onto a CAT6 cable

A benchtop network-technician task. A real CAT6 cable has had its jacket stripped and its eight conductors
straightened in **T568B order** (wire 1 = orange/white, carrying a red index band); a real 8P8C RJ45 plug waits in a
pocket beside a real ratcheting crimper. The robot must pick the cable, align wire 1 with the plug's pin-1 notch and
push all eight wires fully into the plug, carry the wired plug to the crimper seat and seat it fully, squeeze the
ratcheting handle with a real 4-7 N grip until the contact teeth pierce the insulation and the strain-relief jaw grips
the jacket, then pull-test the cable axially with 4-7 N for 1 s. A flipped cable (wrong wire order), a squeeze with the
plug not fully seated, or a squeeze held below 4 N are scored caps. The instruction states the goal; the ordering,
seating, force and verification are the robot's to work out.

**Importance (routine, not occasional).** The US Bureau of Labor Statistics reports **323,600 network and computer
systems administrators** (SOC 15-1244, ~13,400 openings a year) and **903,100 computer support specialists** (SOC
15-1230, ~48,700 openings a year) in 2025 — **over 1.2 million US workers** in these two occupations alone, plus
electricians, low-voltage/structured-cabling installers and non-US workers. Terminating CAT6 cable is recurring core
work of that large workforce (patch panels, wall jacks and equipment-room runs are built and reworked in batches),
not a one-off. Category: **Impact on jobs**. The full case, sources and measured/estimated split are in `REFERENCE.md`.

**What is new against the board (fresh query 2026-11-17T21:53:21Z).**
`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state` returned **2,538 PRs,
newest #2539** (466 OPEN, 1,865 CLOSED, 207 MERGED). No PR in the whole board has this object + mechanism + outcome:
an eight-wire T568B plug termination with a ratcheting squeeze, strain relief and a 1 s axial pull-test. The nearest
crimp PR, **#741 (OPEN), "Add crimp-a-terminal-onto-a-cable-end task"**, pushes one bare wire end into a single
terminal barrel and closes a lever — no eight-wire order, no RJ45 plug seat, no strain relief, no pull-test and no
damage caps. The queue's near neighbour, **#1972 (MERGED), heat-shrinks a sleeve over a crimped cable lug** with a
heat gun — a thermal, two-hand rotation task, different object, mechanism and outcome. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `rj45_crimp.py` | The `yam_arena` task file: scene, the four real-size bodies, the declared carries and the seated lock, the wiring/seat/crimp/pull mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/crimper.usda` | The fixed, visual-only ratcheting crimper: base, seat rail, eight contact teeth, strain-relief jaw, ratchet wheel and stops |
| `assets/handles.usda` | The hinged handles and pawl, posed by the task as the squeeze closes them |
| `assets/plug.usda` | The free 8P8C plug: 11.6 x 11.7 x 6.6 mm body, eight contacts on 1.02 mm pitch, latch, wire channels, pin-1 index notch |
| `assets/cable.usda` | The free stripped CAT6 cable: 6.2 mm jacket, eight 23 AWG conductors on 1.02 mm pitch, red index band on wire 1 |
| `build_assets.py` | Writes the four USD files with pxr (boxes and cylinders; no outside meshes) |
| `test_rj45_crimp.py` | Offline checks (24): every scoring step and cap, the 1 s hold and its restart, real sizes and force mapping, geometry, IK reachability of every waypoint, demo order and timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real products, recognisable features, routine-importance case, sources and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame-by-frame and `data.contact` evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |
| `make_media.py` | Renders the internal top-view media into `benchmark-media-rj45-crimp/` |

## Real product and values

The products are an **8P8C (RJ45) modular plug** and a **ratcheting crimp tool** for CAT6 UTP. Recognisable features,
all modelled: an 11.6 x 11.7 x 6.6 mm plug body with eight gold contacts on the 0.040 in / 1.02 mm pitch, a locking
latch, a rear wire-entry with eight channels and a pin-1 index notch; a 6.2 mm jacket with eight colour-coded 23 AWG
conductors straightened on the same pitch; and a crimper with a plug-seat rail, eight contact teeth above the seat, a
strain-relief jaw that bites the jacket, and a ratcheting handle. The wire order is **T568B** (orange/white, orange,
green/white, blue, blue/white, green, brown/white, brown). Product-class literature gives the plug outline, the 1.02 mm
pitch, the two wiring standards and the contact-piercing + strain-relief mechanism; the brief gives the 4-7 N squeeze
and pull. Measured/given: 8P8C plug, 1.02 mm (0.040 in) contact pitch, 4-7 N forces, the four-step ladder and the three caps. Estimated
(each flagged in `REFERENCE.md`): the exact 11.6 x 11.7 x 6.6 mm outline, 6.2 mm jacket, 23 AWG wire size, 0.35 s
squeeze dwell, 15 deg order window, 12 deg seat-square window and the geometric cap thresholds. No size is scaled down.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The ratcheting crimper is fixed at **x 0.250, y -0.030**, its seat rail centred at **x 0.270,
y -0.030** with the plug centre z 0.018; its handle grip zone is centred at **x 0.190, y -0.130, z 0.050**. The
8P8C plug waits in a pocket at **x 0.220, y 0.060** (tip +x, pin-1 notch +y). The stripped CAT6 cable waits at
**x 0.150, y 0.060** (wire tips +x, wire-1 index band +y). Both arms start at READY. Every grasp point lies about
0.06-0.09 m above the table and inside both arms' working area.

## Mechanism

The plug and cable are real gravity-free rigid bodies; the crimper and its handles are posed visual-only bodies with
no colliders. A closed gripper within the grasp point carries the cable (a declared carry); after the wires are in, the
plug is slaved to the cable with a fixed 6 mm offset, so carrying the cable carries the wired plug. When the plug is
fully in the seat the assembly **locks**, so the squeeze and the pull-test cannot drag it out. Order is checked from
the cable's index direction (wire 1 must land on pin 1; a 180 deg flip is wrong, not an equivalent phase). The crimp
force and pull force are **declared from finger closure** — `force = (0.0939 m - gap) * 86 N/m` — so a 0.024 m gap is
6.0 N and a 0.028 m gap is 5.7 N, both inside the 4-7 N band; the 4 N line is the weak-crimp threshold. Force is
stated, not integrated; the caps are geometric/force state flags, as in the family's other fit tasks.

## Demo

Open loop, known geometry, both arms. The **left** arm picks the cable, aligns wire 1 to pin 1 and inserts all eight
wires, carries the wired plug to the crimper and seats it, then releases and waits; the **right** arm closes on the
handle and squeezes until the ratchet locks; the left arm takes the cable jacket, draws it back axially with a 4-7 N
pull and holds the pull-test. Every approach is from above or laterally along the cable axis.

**MuJoCo 1.0 at step 401 of 1000 (20.05 s).** Step timeline: wires inserted 133, plug seated 226, crimp + strain
relief 293, pull-test begins 362; the 1 s success hold completes at 401. Final: plug seated at x 0.2656 y -0.0282,
cable locked at x 0.2596 y -0.0282, no cap, both arms settled. Checks run: the clean demo and a start pose moved
+3 mm in x/y/z and +3 deg about z both reached 1.0 at step 401; a flipped wire order tripped `wrong_order` (0.5), an
early squeeze tripped `not_seated` (0.5), a half-closed squeeze tripped `weak_crimp` (0.5), and a fully misrouted
flipped cable scored 0.0.

## Scoring

| Step | Done when | Points |
|---|---|---|
| eight wires inserted | wire 1 on pin 1, all eight wires pushed fully into the plug | 0.25 |
| plug seated | wired plug fully in the crimper seat rail, square | 0.5 |
| crimped + strain relief | 4-7 N squeeze held 0.35 s, contacts pierce, jaw grips jacket | 0.75 |
| pull-test holds 1 s | 4-7 N axial pull on the cable held 1 s, no cap, all steps intact | **1.0** |

Caps (0.5 ceiling, block 1.0): `wrong_order` (wire 1 off pin 1), `not_seated` (squeeze before the plug is fully
seated), `weak_crimp` (squeeze held below 4 N). Idle 0. The 1 s hold is timed in the task's `success()`; a break
restarts it and step points stop at 0.75.

## Media

`demo.mp4` (Isaac top view only, 640x360, 286,471 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
1,056,667 B) and `setup.png` (640x360, 28,164 B) are in `benchmark-media-rj45-crimp/`, together with `dream.mp4`
(completed Dream-1 top camera, 320x176, 796,443 B). Each PR file is under 1.87 MB; `task.json` media M-ZVZGFG.

## Gate status

**MuJoCo first gate: 1.0 at step 401, 24/24 offline tests pass, all fields within limits (check_fields ok),
consistency ok, no live twin (TWIN_CHECK).** Isaac demo **1.0 at step 401** (`adhoc_f75e0410.json`); Dream-1 **0.0
at 2,400 steps** (2,401 action rows, completed). No PR, upload or push was made.
