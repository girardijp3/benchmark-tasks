# PASS_THROUGH_QA — hone-kitchen-knife

No gripper, finger, forearm or held knife passes through a solid in the demo. The honing steel, its bench clamp/V-rest
and the wooden knife rest are one welded, **visual-only** body with **no colliders at all**, so there is no solid for
a finger or the knife to pass through; the only colliders are the knife (blade + handle) and the two robot grippers.
The contact table below is MuJoCo (top camera plus `data.contact`, every control step). The same scripted demo was
also checked with the offline IK/reach tools; both arms reach every waypoint (minimum joint margin 14 deg).

## 1. Frames checked (top view, 0/10/25/45/60/80/95/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 0 | 200 mm knife flat on its wooden rest, 300 mm steel clamped horizontally, both grippers at READY |
| 10 | 58 | right pads close on the 22 mm handle from above; no overlap with the rest or the steel |
| 25 | 146 | knife lifted clear, rotating toward the steel heel; blade above the rod, no penetration |
| 45 | 263 | side A sweep near the steel tip: blade yawed ~20 deg, edge sitting on the rod's top surface |
| 60 | 351 | knife lifted and carried across to side B above the steel; edge well clear of the rod |
| 80 | 468 | side B sweep finished; knife carried back toward its rest, still held by the handle |
| 95 | 556 | knife parked flat on the rest, pads open and lifting away; score 1.0 holds |
| 100 | 585 | scored state: knife parked, both grippers home and clear, 1 s hold complete |

The carried knife stays on its declared track: it follows the pads from rest to rod and back with no jump, the blade
holds ~20 deg yaw while the edge overlaps the rod by 1.5 mm, and the tip of the blade never dips below the rod's top
surface. No forearm, finger or held knife crosses the clamp, the V-rest or the knife rest in any frame.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving the knife or the pads |
|---:|---|---|
| 60 | approach | none — the knife is still on the rest and the pads are above it |
| 150 | grip the handle | `knife/HandleCore--right/tip_left` and `--right/tip_right` — intended pad-on-handle grip |
| 180 | angle set at the heel | same two intended handle-pad pairs; blade edge on the rod by declared geometry |
| 230 | side A sweep | same two intended handle-pad pairs; no steel or clamp contact (no colliders) |
| 400 | side B sweep complete | same two intended handle-pad pairs; knife still carried cleanly |
| 480 | park / release | none — the pads have opened and the knife is on the rest |
| 520-584 | 1 s hold, home | none — no gripper-part or part-solid contact |

Every contact is the intended pad-on-handle grip. The steel, clamp, V-rest and knife rest have no colliders, so no
unintended solid penetration is possible and the edge-to-rod touch is measured geometrically (declared pressure).
The only importer note is that the blade's convex mesh is collided as a thin convex shell (0.3-4 mm, never through
the wall behind it); all fit and contact surfaces are primitives or convex hulls at true size.

## 3. Scoring gates and caps

The scorer latches `wrong_angle` when the blade is more than 10 deg off 20 while on the rod, `heavy_press` when the
edge overlaps the rod by more than 5 mm, and `dropped` when the knife is released off the rest. Each holds the score
at 0.5 and blocks 1.0. In the demo the angle stays 19.4-20.8 deg on the rod, the overlap stays 0.8-2.9 mm (target
1.5 mm), and the knife is released only on the rest, so no cap fires. The final park pose is flat (blade up component
> 0.90), the grippers are home, and the whole state is still for 1 s.

## 4. Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 454,602 B (< 1.87 MB), 655 frames with the final 1.0 frame held 3 s.
- `demo_top.gif`: 480x270, 4 fps, 1,629,489 B (< 1.87 MB), same top view.
- `setup.png`: 640x360, 28,763 B, Isaac top camera at reset.
- No close-up, cutaway or composite is in the PR media; none is needed to hide a wall in the arm path.


## Isaac run

The same scripted demo reached **1.0 at step 585 on Isaac** (`adhoc_cde64e66.json`): the knife was taken from its
wooden rest, held at 19.4-20.8 deg to the steel, swept heel-to-tip on both sides with 0.8-2.9 mm light overlap, then
parked flat on the rest with both grippers home and the whole state still for 1 s; no wrong-angle, heavy-press,
missed-sweep or drop fault. The steel/clamp/rest assembly stays a welded visual-only body with no collider on Isaac;
the knife and grippers keep their colliders. No task-local scoring or force change was needed. It was filmed with
the top camera and two render-only close-ups (`film_rod`, `film_knife`) on a copy of the task; no arm, finger or held
knife is seen crossing the steel, clamp, rest or table.
