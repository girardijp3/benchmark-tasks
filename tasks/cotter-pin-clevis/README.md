# Secure a clevis joint with a real 3 mm cotter pin

A benchtop maintenance-task: a real 3 mm ISO 1234 steel split (cotter) pin lies straight in a white holder beside a
fork clevis whose upright clevis pin carries a 3.2 mm cross-hole. The robot must lift the cotter, align its wire axis
with the hole, push it fully through, bend its two legs apart to opposite sides with a finger, pull the red tab outward
to test that the joint holds, release and go home. A straight cotter shakes out and the clevis pin walks out of the
linkage; pulling with the pin not through, pulling with the legs not spread, or dropping the cotter are scored caps.
The instruction states only the goal; the alignment, insertion, leg spreading and pull test are the robot's to work out.

**Importance (routine, not occasional).** Maintenance and repair workers hold about **1.5 million US jobs**, and
industrial machinery mechanics, machinery maintenance workers and millwrights held **547,300 US jobs in 2025**
(~51,900 openings a year; SOC 49-9041/49-9043/49-9044), with heavy-equipment, farm and aircraft mechanics adding
more. Securing clevis linkages with cotter pins is recurring preventive/corrective maintenance on brakes, controls and
linkages: a safety-critical failure if the pin backs out. Category: **Impact on jobs**. The full case, sources and
measured/estimated split are in `REFERENCE.md`.

**What is new against the board (fresh query 2026-11-17T21:53:56Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state` returned **2,536 PRs, newest #2537** (464 OPEN, 1,865 CLOSED,
207 MERGED). The two near cotter-pin PRs are **#994 (OPEN, labelled "Not accepted" — impossible 10 mm / 0.01 N m leg
physics, no revision since 2026-11-07)** and **#2060 (CLOSED, "Not accepted: final" — failed the 500,000-worker
importance bar, reviewer invited a different new task)**. Both were rejected by review; neither has an accepted
implementation. This submission fixes both rejection reasons with a real 3 mm pin and a 500,000+ worker case, and adds
an alignment rung and a pull-test outcome absent from both. Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `cotter_pin_clevis.py` | The `yam_arena` task file: scene, the free cotter (eye+shank and two legs), the align/push/bend/pull mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/clevis.usda` | The welded fixture: stand, fork prongs, rod end, 10 mm clevis pin with a 3.2 mm cross-hole, red pull tab and white cotter holder (visual-only, no colliders) |
| `assets/cotter.usda`, `assets/leg_long.usda`, `assets/leg_short.usda` | The real 3 mm split pin: eye loop + 12 mm shank, and its 36 mm green long leg and 28 mm blue short leg, each a free gravity-free body |
| `build_assets.py` | Writes the four USD files with pxr (boxes, cylinders and a generated eye ring; no outside meshes) |
| `test_cotter_pin_clevis.py` | Offline checks (21): every scoring rung and cap, the 1 s hold and its restart, real sizes, leg-bend and pull geometry, all demo waypoints reachable, the plan and its timing |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, recognisable features, 500k+ importance case, sources and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Frame-by-frame and `data.contact` evidence that no part passes through a solid, plus the in-sim cap checks |
| `TWIN_CHECK.md` | The fresh live-twin query and the two rejected near cotter-pin PRs |
| `make_media.py` | Renders the internal top-view media into `benchmark-media-cotter-pin-clevis/` |

## Real product and values

The product is a **split pin / cotter pin to ISO 1234** (also called a cotter key): a soft low-carbon-steel wire looped
into an eye at one end, with two legs that are pushed through a cross-hole in a clevis pin and then bent apart to
retain it. Recognisable features: an eye loop, a straight shank, two legs of unequal length (the extended-prong form
makes the long leg pushable first), a bright zinc finish and a cross-drilled clevis pin. The pin is a replaceable
fastener; a bent-then-straightened cotter is scrapped on safety work (FAA AC 43.13-1B ch. 7 and farm/heavy-equipment
service manuals). Sizes: **3 mm nominal** split pin, **2.8 mm wire**, **3.2 mm clearance cross-hole**, **10 mm clevis
pin**, 4-7 N push. Measured/given: those sizes, the split-pin product class and the safety-failure mode. Estimated
(each flagged in `REFERENCE.md`): 12 mm shank, 36/28 mm legs, 1.6 g, the pull-tab geometry, the 55 deg bend gate and
the 10 deg axis window. No size is scaled down.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The fixture is fixed at **x 0.250, y -0.055**; its cross-hole axis runs along +x at **z 0.8025
world**, its near face is at x 0.245 and its red pull tab is at **x 0.308**. The straight cotter waits in the white
holder at **x 0.250, y 0.045**, centre z 0.7575 world, wire along +x, green long leg on the +y side. The correct
inserted centre is **x 0.2445, y -0.055, z 0.8025**, wire axis along +x, eye band against the pin's near face. Both
arms start at READY. Every grasp point lies about 0.76-0.85 m world (0.01-0.11 m above the table) and within 0.40 m of
an arm base.

## Mechanism

The cotter is three real gravity-free rigid bodies. While a closed gripper is within 16 mm of the eye/shank grasp point
it carries the cotter (a declared carry, the family's device); the robot must still reach, close, align, push and carry
it. Each leg bends about its root as a closed finger sweeps its tip outward, and latches at the angle reached, like soft
steel that stays bent. The cross-hole, wire axis, seat band and colour-coded leg tips are real geometry. The scorer
reads the mechanism flags: aligned (axis within 10 deg over the hole), through (centre within 6 mm of the hole in
x/y/z), spread (both legs at least 55 deg to opposite sides), pull (an 8 mm outward drag at the tab). The 4-7 N push and
pull are **declared, not integrated**; the fixture is visual-only with no colliders, so the caps are geometric state
flags as in the family's other fit tasks.

## Demo

Open loop, known geometry, one arm (the left; the right stays home). The left arm: (A) grips the straight cotter in the
holder and lifts it; (B) carries it over the cross-hole and aligns its axis; (C) pushes it through and releases it;
(D) closes on each leg's tip in turn and sweeps it outward to the spread mark; (E) grips the red tab, pulls it outward,
holds, releases and lifts away; (F) goes home. Every approach is from above.

**MuJoCo 1.0 at step 688 of 900 (34.40 s after the settle).** Step timeline: aligned 136, through 183, spread 448,
pull-test holds 549; the 1 s hold completes at 688. Final: cotter centre x 0.2449, y -0.0553, z 0.8038 (within
0.6 mm of the hole), axis 0.2 deg, leg bends +57.9/+55.2 deg to opposite sides, no cap, both grippers home; worst
IK error 0.09 mm. Idle 0; aligned 0.25; through 0.5; spread 0.75; not-through, not-spread and dropped each cap at 0.5.
Three in-sim bad demos were run: a pull with the legs straight latched `not_spread` and scored **0.5**; a pull with the
cotter not inserted latched `not_through`; an installed cotter carried off the bench latched `dropped` and scored
**0.5**. Neither reached 1.0.

## Scoring

| Step | Done when | Points |
|---|---|---|
| aligned with hole | held over the hole, wire axis within 10 deg of the cross-hole axis | 0.25 |
| cotter through | centre within 6 mm of the hole in x/y/z, axis within 10 deg | 0.5 |
| legs spread | both legs at least 55 deg to opposite sides of the shank | 0.75 |
| pull-test holds | 8 mm outward drag, joint still seated and spread, held 1 s, caps clear, grippers home | **1.0** |

Caps (0.5 ceiling): pull with the pin not through, pull with the legs not spread, cotter dropped off the bench. Idle 0.
The 1 s hold is timed in the task's `success()`; a break restarts it and step points stop at 0.75.

## Media

`demo.mp4` (640x360, top view only, 141 KB, < 1.87 MB), `demo_top.gif` (480x270, 625 KB, < 1.87 MB) and `setup.png`
(640x360) are written by `make_media.py` into `benchmark-media-cotter-pin-clevis/`. The film is the scripted MuJoCo
demo; no 1.0 Dream run was made by the builder.

## Gate status

**MuJoCo first gate: 1.0 at step 688, 21/21 offline tests pass, all fields within limits (check_fields ok),
consistency ok, no live accepted twin (TWIN_CHECK — both near cotter-pin PRs were rejected by review).** The builder
Isaac demo 1.0 at step 688 (`adhoc_db58460c.json`); Dream-1 0.0 at 2,400 steps (2,401 action rows). No PR,
upload or push was made.
