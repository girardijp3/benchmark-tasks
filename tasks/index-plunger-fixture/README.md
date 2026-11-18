# Index a 16 mm spring index plunger fixture

A bench task from a machinist's / fixture builder's day. A 150 mm aluminium index plate sits on a fixed pedestal at
x 0.25, y 0 with eight 7 mm holes in its rim 45 degrees apart and a 24 mm hub on its axis. A 16 mm class hand index
plunger is bracketed on the robot's left (+y) side with its 7 mm pin in the hole at the home stop. The robot has to
pull the knob straight out against the 4-7 N spring to retract the pin, keep it out while it turns the plate one hole
pitch (45 degrees) to the next stop, release the knob so the spring drives the pin into the next hole, and leave the
plate indexed with both grippers home. The instruction states only the goal: what "retract", "next stop" and "release
into the hole" mean, and that a forced turn, an over-turn or a pin dropped between holes is wrong, are the robot's to
work out.

**What is new against the board (fresh query 2026-11-17T21:41:18Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned **2,536 PRs**, newest **#2537** (464 OPEN, 1,865 CLOSED,
207 MERGED) — the board has moved since the analyst's 19:45Z snapshot (2,512 PRs, newest #2513), so the query was rerun
at launch. No live OPEN task has this object + mechanism + outcome. The nearest CLOSED PRs are #332 (change over a
fixture plate against its locators — a different action), #233 (index an off-balance rotary table 90 deg and lock it
with a separate pin), #829 (index a workpiece on a dividing head), #1117 (index a Geneva drive) and #292 (test two
recessed spring plungers with a pin tool) — different objects, mechanisms or outcomes, and all CLOSED. A full search is
in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `index_plunger_fixture.py` | The `yam_arena` task file: scene, the plate and plunger and their declared carry/rotation, the pull/turn/release mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/base.usda` | The fixed pedestal, bearing ring, plunger bracket/bore and home-stop mark |
| `assets/plate.usda` | The 150 mm plate: disc, central hub knob and 8 radial index holes |
| `assets/plunger.usda` | The 16 mm class index plunger: 16 mm knob and 7 mm pin |
| `build_assets.py` | Writes the three USD files (pxr; boxes and cylinders, colliders on the two moving bodies) |
| `test_index_plunger_fixture.py` | Offline checks (18): every scoring outcome and cap, the 1 s hold and restart, gripper-clear, real sizes, all demo waypoints reachable, the plan and its timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the sources, the routine-importance case and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame-by-frame top/close-up and `data.contact` evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |

## Real product and values

The product class is a **16 mm class spring / hand index plunger**: a threaded body holding a spring-loaded, slightly
tapered pin with a knob (ball or hand knob) for retraction, matched to a locating bushing or an index hole. The pin is
held in the next hole by spring force; to move the fixture you pull the knob out (against the spring), turn to the next
stop, and let the pin re-engage. Representative catalogue items are the Carr Lane **CLM-10-HRPK** hand-retractable
plunger (M16 x 1.5 body, 8 mm pin, ~31 mm knob, 2.5-6.7 lbf / ~11-30 N end force) and the Carr Lane index-plunger /
bushing family (12-20 mm pins, 20-30 mm bodies). The brief's **4-7 N** pull band sits at the light end of that family;
the task keeps the brief's 4-7 N and a 16 mm grasp knob per the gate. Sources and the measured/estimated split are in
`REFERENCE.md`.

**Measured (product class / brief):** threaded body class M16 / 16 mm; 8 mm pin class; spring-loaded retractable pin
into a hole/bushing; 4-7 N pull force (brief); one hand pulls, the other rotates the fixture.

**Estimated (each in `REFERENCE.md` and in `task.json` fidelity):** 16 mm knob, 7 mm pin, 150 mm plate with eight
7 mm rim holes at 45 degrees, 24 mm hub, 90 mm pedestal, part masses 600 g / 120 g, 16 mm pull travel. No size, mass or
force is scaled down.

## Routine importance (explicit case)

Indexing a fixture with a spring plunger is a **routine, repeated** action in machining and fixture work — it happens
every stop of every setup or part changeover, not occasionally. A conservative proxy from **US BLS OEWS, May 2024**:
**298,790** machinists (SOC 51-4041), **421,940** industrial machinery mechanics (49-9041) and **56,930** tool and die
makers (51-4111), who fit and use jigs, fixtures and indexing devices — **777,660** workers, above the 500k bar. Honest
caveat: no source counts workers performing this exact pull/turn/release sub-action; the number is an occupation-level
proxy, not a direct measurement. Sources in `REFERENCE.md`.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The plate axis is at **x 0.25, y 0**; the disc is 150 mm across from local z 0.100 to 0.120 with its
8 index holes at radius 0.072 and its hub knob at **z 0.130**. The plunger's 16 mm knob centre is at **x 0.25, y 0.150,
z 0.110** with its pin in the home hole; pulling moves it 16 mm along +y. The left arm grips the knob; the right arm
grips the hub. Every grasp point lies within 0.35 m of an arm base and above the bench; the fixture is fixed.

## Mechanism

The plate and plunger are real gravity-free rigid bodies posed by the task file from YAM gripper measurements: while a
gripper is closed across the knob the plunger follows its TCP along the plunger axis (a declared carry, like the
family's latch detents); while a gripper is closed on the hub the plate yaw follows that wrist's yaw, so rotating the
wrist 45 degrees turns the plate exactly one hole pitch about its centre (declared rotation, as in the cycloidal-disc
task). The robot must still reach, close on, pull, turn and release both. The 4-7 N spring is declared, not integrated;
the pull, the 45 degree turn, the hole alignment and the pin release are real geometry.

Three caps catch the wrong action: **plate overshoot** (turned past one pitch + 12 degrees), **plunger misses hole**
(released with the plate more than 8 degrees off any stop) and **forced pull** (plate turned more than 8 degrees while
the pin is still engaged). Each holds the score at 0.5 and blocks 1.0. The demo triggers none.

## Demo

Open loop, known geometry, two arms working together: the left arm grips the knob and pulls it 16 mm clear, then holds;
the right arm grips the hub, waits for the pull, rotates the wrist 45 degrees and lets go; the left arm returns the pin
to the hole, opens and homes; the right arm homes. **MuJoCo 1.0 at step 356 of 1,000 (17.80 s).** Step timeline: pin
pulled 103, plate at next stop 160, pin released into hole 252; the 1 s hold completes at 356, plate at +44.9 degrees
(hole error -0.1 degree), pull 0, both grippers home and clear. Idle 0; pulled 0.25; rotated 0.5; pin seated 0.75;
overshoot, missed-hole and forced-pull each cap at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| plunger pulled | knob pulled out past 12 mm, pin retracted | 0.25 |
| plate at next stop | plate turned 45 degrees (one hole pitch) while the pin is clear | 0.5 |
| pin released into hole | knob released, pin back in the hole at that stop | 0.75 |
| left indexed + hold | plate left indexed, grippers clear and home, held 1 s | **1.0** |

Caps (0.5 ceiling): plate overshoot, plunger misses hole, forced pull while the pin is engaged. Idle 0.

## Media

`demo.mp4` (Isaac top view only, 640x360, 277,316 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 5 fps,
1,302,922 B) and `setup.png` (640x360, 29,648 B) are in `benchmark-media-index-plunger-fixture/`, together with
`dream.mp4` (completed Dream-1 top camera). Each PR file is under 1.87 MB; `task.json` media M-L36Y28.

## Gate status

**MuJoCo 1.0 at step 356; Isaac 1.0 at step 356** (`adhoc_835e33d4.json`); 18/18 offline tests pass.
Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed). Fields and consistency pass; `task.json` media M-L36Y28.
No PR, upload or push was made.
