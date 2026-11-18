# Hone a 200 mm chef's knife on a 300 mm honing steel

A prep-station task from a chef's day. A real 200 mm chef's knife lies on a wooden rest to the robot's right; a real
300 mm honing steel is clamped horizontally in a bench holder, its rod heel (guard) pointing toward the robot. The
robot takes the knife by its handle, sets the blade at about **20 degrees** to the rod at the steel heel, sweeps the
edge from heel to tip with a light touch, alternates to the other side and sweeps again, then parks the knife back on
its rest and holds still. The instruction states only the goal: how to find the angle, how light the pressure must be,
what "alternating sides" means and where the knife belongs are the robot's to work out.

**What is new against the board (fresh query 2026-11-17T21:51:47Z).** `gh pr list --repo robocurve/benchmark-tasks
--state all --limit 3000 --json number,title,state,url` returned **2,538 PRs**, newest **#2539** (466 OPEN, 1,865
CLOSED, 207 MERGED). The only live knife-sharpening task is **#1421 OPEN** "Sharpen a kitchen knife on a whetstone at
15 to 20 degrees, both sides, then lay it down edge away" — a different tool (abrasive stone vs honing steel), a
different outcome (grinding a new bevel vs realigning the edge with no metal removed) and a different end state (lay
down edge away vs park on a rest). No live OPEN task has this object + mechanism + outcome. Full search in
`TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `hone_kitchen_knife.py` | The `yam_arena` task: scene, declared carry, angle/pressure/sweep geometry, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/knife.usda` | The real 200 mm chef's knife (205 g; convex blade, bright ground bevel, three-rivet handle, bolster) |
| `assets/steel.usda` | The 300 mm honing steel (grooved rod, knurled handle) in its bench clamp and V-rest, welded, no colliders |
| `assets/rest.usda` | The wooden two-cradle knife rest the knife starts on and is parked on |
| `build_assets.py` | Writes the three USD files (pxr + scipy convex hull) |
| `test_hone_kitchen_knife.py` | 18 offline checks: every scoring step and cap, the 1 s hold and its restart, gripper-clear, real sizes, reach, demo order/timing |
| `task.json` | The task's details for the board (all fields; `check_fields.py` passes) |
| `REFERENCE.md` | The real products, their recognisable features, the sources and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Top-view frames and `data.contact` evidence that nothing passes through a solid |
| `TWIN_CHECK.md` | The fresh live query and the whetstone task's differences |

## Real product and values

The products are a **200 mm chef's knife** (a standard 8 in cook's knife: broad curved stainless blade, thin ground
edge, full tang through a three-rivet dark handle, steel bolster, ~205 g) and a **300 mm honing steel** (a hard
chromium-plated rod, smooth or finely grooved, ~200 mm exposed rod and ~100 mm knurled handle). Honing realigns a
rolled edge before prep; it does not grind metal away like a whetstone. Chefs and food-service staff do it many times
a shift (US cooks/chefs/food-prep, SOC 35-2011/2012/2014, 500k+ workers). Sources and the measured/estimated split
are in `REFERENCE.md`.

**Measured (product class):** 200 mm blade; 300 mm steel; ~20° presentation; light pressure; no metal removal.

**Estimated (each in `REFERENCE.md` and in `task.json` fidelity):** knife 205 g, 55 mm heel blade, 2.5 mm spine,
130 x 22 mm handle; steel 16 mm rod diameter, 100 mm handle; 1.5 mm light edge overlap (0-3 mm light band, >5 mm
heavy); 180 mm heel-to-tip travel; YAM jaw 20 mm. Nothing is scaled down.

## Layout

Positions are metres from the arm-base midpoint, x forward, y to the robot's left, table top z 0.7475. The steel's rod
heel (guard) is at **x 0.250, y -0.150**; its rod axis runs +X to x 0.450 at centre z 0.7755, radius 8 mm (top
surface z 0.7835), with the 100 mm handle in a bench clamp. The chef's knife starts on a wooden rest at **x 0.330,
y -0.410**, origin at the blade heel, blade along +X, edge 28 mm below the origin. The right arm uses the knife;
the left stays at READY. A correct touch has the blade yawed 20° to the rod with the edge overlapping the rod top
by 1.5 mm. Every grasp point lies 0.07-0.12 m above the table and within 0.40 m of the right arm base.

## Mechanism

The knife is a real, gravity-free rigid body. While the right gripper's fingers close across the handle's grasp point
the knife follows the gripper (a declared carry, the family's usual device); the robot must still reach, close on,
orient, sweep and park it. The steel, its clamp/V-rest and the knife rest are one welded, collider-free fixture.
Contact is a **real geometric measurement** of the ground edge against the rod's top surface: the blade's yaw gives
the angle (scored 15-25°, cap outside 10-30°), the edge's depth below the rod top gives the pressure (0-3 mm light,
>5 mm cap), and the blade-heel x from 0.250 to 0.430 gives the 180 mm heel-to-tip travel. The two sides are the two
20° presentations (yaw +20° then -20°). No force sensor or metal removal is simulated.

## Demo

Open loop, known geometry, one arm (the right; the left stays home). The right arm: (A) grips the knife by its handle,
lifts it off the rest; (B) carries it to the steel heel and sets the blade at 20° with a light touch; (C) sweeps the
edge heel-to-tip along the rod; (D) lifts, presents the other side at -20° and sweeps again; (E) parks the knife flat
on its rest, opens the gripper and goes home. Every approach is from above with the fingers closing across the handle.

**MuJoCo 1.0 at step 585 of 2,400 (29.25 s after the settle, t=29.75 s in the task log).** Step timeline: angle set
162, heel-to-tip sweep 242, both sides + clean edge 396, knife parked 483; the 1 s hold completes at 585. Final: blade
angle 19.4-20.8° on the rod, edge overlap 0.8-2.9 mm (target 1.5 mm), both sweeps 182/181 mm of 180 mm, knife flat on
the rest, both grippers home and clear, no cap. Idle 0; angle 0.25; sweep 0.5; both sides 0.75; parked + hold 1.0.
Wrong angle, heavy press or a dropped knife each cap at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| angle set | knife grasped, blade at ~20° on the steel heel with a light touch | 0.25 |
| heel-to-tip sweep | edge travels >=170 of 180 mm along the rod, light pressure, angle held | 0.5 |
| both sides, clean edge | the same sweep completed on the other side | 0.75 |
| knife parked + hold | knife flat on its rest, grippers home, all still for 1 s | **1.0** |

Caps (0.5 ceiling): blade >10° off 20° on the rod, edge >5 mm into the rod, knife dropped off the bench. Idle 0.
The real trial checks the angle with a protractor, the touch by feel and the parked knife on its rest; a stopwatch
times only the 1 s hold.

## Media

`demo.mp4` (Isaac top view only, 640x360, 454,602 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 4 fps,
1,629,489 B) and `setup.png` (640x360, 28,763 B) are in `benchmark-media-hone-kitchen-knife/`, together with
`dream.mp4` (completed Dream-1 top camera, 320x176, 606,141 B). Each PR file is under 1.87 MB; `task.json` media M-UUWHN3.

## Gate status

**MuJoCo 1.0 at step 585; Isaac 1.0 at step 585** (`adhoc_cde64e66.json`); 18/18 offline tests pass, static IK
reaches every waypoint (min 14° joint margin). Dream-1: **0.0 at 2,400 steps** (2,401 action rows, completed).
Fields and consistency pass; `task.json` media M-UUWHN3. No PR, upload or push was made.
