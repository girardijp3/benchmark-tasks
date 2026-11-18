# Snap a locking carabiner onto an anchor eye and screw the gate shut

A routine, safety-critical task from a rigger's, arborist's or climber's day. A compact ~60 mm aluminium screw-lock
(SCREW-LOCK) locking carabiner lies flat on a base beside a fixed anchor eye (an eye-bolt post with a ring on top). The
robot opens the gate by pushing it against the post, hooks the post through the loop, lets the spring shut the gate,
grips the knurled screw sleeve with the same arm and turns it about a quarter turn until it is fully locked, then
pull-tests the connection by sliding the eye to the gate end of the loop and holding it. The instruction states only
the goal; that the gate must open before the post can enter, that the eye must be inside the loop before the gate is
shut, and that a quarter turn of the sleeve is what locks it, are the robot's to work out.

## Why this is routine, not occasional (importance case)

BLS/O*NET put **riggers (SOC 49-9096) at 22,530 jobs** and **tree trimmers and pruners, also called arborists (SOC
37-3013), at 55,160 wage-and-salary jobs in May 2025** — 70,600 including the self-employed in the 2025 employment
projections. Construction and extraction occupations employed **6.4 million** workers in May 2025 (BLS), and many
ironworkers, roofers and line workers use locking carabiners daily for fall arrest. Add millions of recreational
climbers and the population doing this work is well past **500,000** workers. For all of them, clipping a locker to an
anchor and checking that the sleeve is fully shut and the link holds is repeated, life-safety work, not a rare trick; a
sleeve left loose or a gate left open can unclip. (Sources: BLS OEWS May 2025 and Employment Projections 2025-35;
BLS O*NET SOC 49-9096 and 37-3013; general product literature for SCREW-LOCK connectors.)

**What is new against the board (fresh live query at build time).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned **2,538 PRs, newest #2539** (466 OPEN, 1,865 CLOSED, 207
MERGED). Only two PR titles contain "carabiner", and **both are CLOSED**: **#591 "Assemble a hanging carabiner chain"**
(three carabiners linked into a chain, no anchor or lock step) and **#2159 "Clip a spring-gate carabiner to a steel
tie-down staple"**. #2159 is the important near neighbour and is **not a live twin**: it uses a **spring-gate**
carabiner and a **tie-down staple**, with the gate merely springing shut; this task uses a **manual screw-lock sleeve**
and an **anchor eye**, and the scored outcome includes **screwing the sleeve fully shut and a pull-test** — a different
mechanism (spring gate vs threaded sleeve) and a different outcome (clipped vs locked-and-proof-pulled). No OPEN task
has this object, mechanism or outcome. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `locking_carabiner.py` | The `yam_arena` task file: scene, real spring-hinged gate, declared screw sleeve, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/carabiner.usda` | The ~60 mm locking carabiner: a free body and a spring-hinged gate with a knurled screw sleeve (OpenUSD); one articulation |
| `assets/anchor.usda` | The fixed anchor eye: base, post and ring (OpenUSD) |
| `build_assets.py` | Writes the two USD files (pxr; boxes, capsules, cylinders) |
| `test_locking_carabiner.py` | 23 offline checks: every rung and cap, the 1 s hold and its restart, real size, the YAM reach gate and the demo's IK |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its recognisable features, sources, and measured vs estimated numbers |
| `PASS_THROUGH_QA.md` | Frame-by-frame top-view and `data.contact` evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |
| `make_media.py` | Renders the internal top-view `setup.png`, `demo.mp4` and `demo_top.gif` |

## Real product and values

The product class is a **manual screw-lock (SCREW-LOCK) locking carabiner** used as fall-protection/PPE: a light-alloy
oval or D body, a spring-closed gate, and a **knurled sleeve that screws down over the gate to lock it**. Petzl's
connector literature describes SCREW-LOCK as a manual locking system with a red band visible when unlocked and EN 362
certification; SingingRock's COLT screw, DMM's Phantom Screwgate and Black Diamond's screwgate lockers are typical
examples at **42-85 g**, **15-25 mm gate opening** and **EN 362 / EN 12275 / UIAA 121**. The brief specifies a **~60 mm**
carabiner, so the model is a compact locker: **60 mm outline, 6 mm wire, 50 g, 34 mm gate**. No size or mass is scaled
down. Sources and every measured vs estimated number are in `REFERENCE.md`.

**Measured (product class / brief):** ~60 mm locking carabiner, light-alloy body, spring gate, manual screw-lock sleeve,
gate-closed / sleeve-locked visual check, pull-test before use, EN 362 / EN 12275 / UIAA 121 class.

**Estimated (each in `REFERENCE.md` and `task.json` fidelity):** 6 mm wire cross-section, 50 g mass, 34 mm gate swinging
60 degrees on a 0.05 N m/rad spring, a 6 mm eye-bolt post, a quarter-turn (75 degree) lock window. The thread is
declared from the gripper's twist, not integrated as a helix.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0. The
anchor eye's post is at **x 0.345, y 0** on a 240 x 200 x 8 mm base centred at x 0.31. The carabiner starts flat with
its spine at **x 0.255, y 0**, loop and gate toward the post. Hooked, the spine sits at about x 0.318 (post mid-loop);
the pull-test slides it back to x 0.299 (post 46 mm down the loop, against the gate). All work points are within
0.38 m forward reach. The right arm does the whole demo; the left stays at READY.

## Mechanism

The carabiner is a real articulated rigid body. The gate is a revolute joint with a spring and a closed stop: pushing
the carabiner's gate against the post swings the gate open (about 42 degrees), the post passes inside the loop, and the
spring snaps the gate shut behind it. Hooking is a geometric test: the post's axis must cross the carabiner's plane
inside the loop polygon, the plane within 45 degrees of horizontal. The screw sleeve's **thread is declared**, the way
the family declares latches and twists: while the right gripper turns above the sleeve with the gate shut, the task
accumulates the wrist's yaw; a quarter turn sets the locked flag. The demo turns 100 degrees (0.28 turns), inside the
75 +/- 15 degree lock window. The pull-test is a real geometric test: the eye must slide to the gate end of the loop and
stay there. A sleeve left loose, a gate left open or a dropped carabiner are scored as caps.

## Demo

Open loop, known geometry, one arm. The right arm: (A) pinches the spine, lifts the carabiner, lines the gate up with
the post, pushes until the post is inside the loop and the gate snaps shut; (B) releases it resting on the post, moves to
the screw sleeve; (C) turns the wrist a quarter turn to lock it and lets go; (D) regrips the spine, slides the eye to the
gate end of the loop and holds the pull-test for 1 s, then lets go and goes home.

**MuJoCo 1.0 at step 597 of 1,600 (29.85 s).** Rungs: gate opened 205, eye through the loop 206, gate closed 240, sleeve
locked + pull-test 556; the 1 s hold completes at 597. Final state: sleeve 0.28 turns LOCKED, eye at -46 mm in the loop,
gate closed, no caps. Idle 0; gate open 0.25; hooked 0.5; gate closed 0.75; sleeve locked + pull-test 1 s = 1.0. Two
further clean runs also scored 1.0 at step 597. The 23 offline tests pass.

## Scoring

| Rung | Done when | Points |
|---|---|---|
| gate opened | the gate swung at least 12 degrees open at any time | 0.25 |
| eye through the loop | the post's axis crosses the carabiner's plane inside the loop | 0.5 |
| gate closed | hooked, with the gate shut behind the post | 0.75 |
| sleeve locked + pull-test | sleeve locked, eye slid to the gate end of the loop, held 1 s | **1.0** |

Caps (0.5 ceiling): releasing a threaded carabiner with the gate open, or starting the pull-test before the sleeve is
locked. A carabiner off the table scores 0 for the whole run. Idle 0. Full score needs all four rungs held together for
1 s, timed in the task's `success()`; a lapse restarts the second.

## Media

`demo.mp4` (Isaac top view only, 640x360, 348,203 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
1,196,625 B) and `setup.png` (640x360, 27,873 B) are in `benchmark-media-locking-carabiner/`, together with `dream.mp4`
(completed Dream-1 top camera, 320x176, 203,259 B, 930 frames). Each PR file is under 1.87 MB; `task.json` media M-MB6RQF.

## Gate status

**MuJoCo first gate: 1.0 at step 597, 23/23 offline tests pass, all task.json fields within limits, live twin check
clean.** Isaac demo **1.0 at step 640** (`adhoc_4d08203e.json`); Dream-1 **0.0 at 2,400 steps** (2,401 action rows,
completed). No PR, upload or push was made.
