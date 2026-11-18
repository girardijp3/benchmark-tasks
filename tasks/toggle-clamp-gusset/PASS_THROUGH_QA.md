# PASS_THROUGH_QA — toggle-clamp-gusset

No part passes through a solid in the demo. The fixture, locator nest, clamp bases and staging tray are one
**welded, visual-only** body with no colliders at all, so there is no fixture solid for a part or finger to pass
through; the colliders are the gusset, the two clamp levers and the two robot grippers. The evidence below is MuJoCo
(top camera plus `data.contact` at every control step). The same scripted demo is the one submitted; its top-view
media is `benchmark-media-toggle-clamp-gusset/demo.mp4`, `demo_top.gif` and `setup.png`.

## 1. Frames checked (top view, 0/13/27/49/54/81/97/100 %)

| % | step | what it shows |
|---:|---:|---|
| 0 | 1 | gusset upright in the staging tray, both red clamp handles open, locator nest bare, both arms at READY |
| 13 | 80 | left pads close on the top edge of the gusset upright; no overlap with the fixture |
| 27 | 163 | gusset seated square on the two locator pins and back wall, score 0.25; clamps still open |
| 49 | 300 | right pads close on the clamp A handle; gusset stays seated square |
| 54 | 331 | clamp A flipped over-centre, its jaw pad over the gusset foot, score 0.5 |
| 81 | 499 | clamp B flipped over-centre too, score 0.75; gusset still square on the nest |
| 97 | 595 | the clamped assembly has held 1 s, both grippers clear and home |
| 100 | 614 | scored state: 1.0, gusset square, both levers locked over-centre, grippers home |

The carried gusset follows the left pads from the tray to the nest with no jump, stays upright (foot along x,
upright along z) at the seat, and the two levers swing on their pivots from open to over-centre. No forearm,
finger or held part crosses the fixture or the body in any frame. The final frame of `demo.mp4` holds the 1.0 state.

## 2. Contact evidence (`data.contact`, MuJoCo)

| step | phase | contacts involving the free parts |
|---:|---|---|
| 1 | reset | none |
| 205 | gusset seated, clamps open | none |
| 300 | right arm grips clamp A | `clamp_b/Handle1 -- clamp_a/JawPad5` (4 pairs, dist -0.00007 to -0.00010 m) — a sub-0.1 mm graze as the carried lever passes its neighbour; no penetration of a solid, and the levers are written to their pivots every step |
| 340 | clamp A flipped | `gusset/Upright2 -- clamp_a/LinkArm3`, `gusset/Upright2 -- clamp_a/JawPad5` — the real jaw-on-bracket contact |
| 510 | clamp B closing | `gusset/Upright2 -- clamp_a/LinkArm3`, `gusset/Upright2 -- clamp_a/JawPad5`; two transient broad-phase `table -- gusset/Upright2` pairs (dist -0.0015 / -0.0025 m) |
| 614 | scored state | `gusset/Upright2 -- clamp_a/JawPad5`, `gusset/Upright2 -- clamp_b/JawPad5` — both jaws holding the bracket; **no gripper-part contact** |

Every durable contact is either an intended pad-on-part grip or a clamp jaw pressing the bracket. On the one
transient `table -- gusset/Upright2` pair at step 510: the gusset body is written at z 0.7923 every control step
(its MuJoCo free-joint qpos reads `[0.280, 0.000, 0.7923, ...]` at that step) and the table top is at 0.7475, a
44.8 mm gap, so this is a broad-phase artifact while the jaw pushes near the bracket — the frame review shows no
penetration and the authored pose is unchanged before and after it. The fixture itself has no collider, so no
solid penetration is possible there. The graze at step 300 is likewise sub-0.1 mm and does not move either lever.

## Pinch / mis-clamp protection

The scorer has explicit caps: flipping a clamp while the gusset is not seated sets `gusset_shift`; releasing a
clamp with its lever yaw between 0.25 and 0.85 of the over-centre angle sets `clamp_not_over`; releasing any part
off the bench sets `dropped`. Each holds the score at 0.5 and blocks 1.0. The demo triggers none of them: final
clamp twists are 70 and 69 degrees (threshold 55 degrees), the gusset is seated before either flip, and both
parts are released on the bench.

## Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 415,200 B (< 1.87 MB), 684 frames with the final 1.0 frame held 3 s.
- `demo_top.gif`: 480x270, 4 fps, 1,442,451 B (< 1.87 MB), same top view.
- `setup.png`: 640x360, 29,196 B, Isaac top camera at reset.
- No close-up or cutaway in the PR media.

## Gate status

**MuJoCo 1.0 at step 614 of 1,200** (31.20 s in the task log): gusset seated square at step 163, clamp A over-centre
at 331, clamp B at 499, full assembly held 1 s at 595, success at 614. 19/19 offline tests pass; fields and
consistency pass. Isaac demo 1.0 at step 614 (`adhoc_d16440a9.json`); Dream-1 result in `task.json`. No PR,
upload or push was made.


## Isaac run

The scripted demo reached **1.0 at step 614 on Isaac** (`adhoc_d16440a9.json`): the 210 g gusset seated square on the
locator nest, both sprung toggle levers flipped 70 deg over-centre (the declared stand-in for the cam), and the
assembly stayed clamped square with both grippers home for the 1 s hold; no gusset off-square, clamp released early
or dropped-part fault. The fixture stays welded and collider-free on Isaac; the gusset, levers and grippers keep
their colliders. No task-local scoring or force change was needed. It was filmed with the top camera and two
render-only close-ups (`film_clamps`, `film_gusset`) on a copy of the task; no arm, finger or held part is seen
crossing the fixture, tray or table.
