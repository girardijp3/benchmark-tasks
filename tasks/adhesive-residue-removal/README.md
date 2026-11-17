# Remove the leftover price-label adhesive from a returned pan's base with a 91% IPA wipe

**Dream-1 video provenance.** The completed Dream-1 run is `adhoc_2a3d7d65.json` (2,400 steps, score 0.0, policy never picks a pad). Its original top-camera frame store was deleted during VM cleanup, so `dream.mp4` is a replay of that run's exact 2,400 recorded actions (`actions/20261117_041726_15aef825/scene-0-e0.jsonl`, same task, `object_jitter=0`, seed 0; replay log `adhoc...dream-replay_ca53fdbe.json`, 2,400 steps, 0.0). It is not the scripted demo and not the MuJoCo film. A literal original-frames video would need a fresh Dream run.

**Status: PASS.** MuJoCo demo: **1.0 at step 1,446** (72.3 s after the settle); Isaac: **1.0 at step 1,446** with the real PhysX pad-pan contact force (3.0-3.3 N dwell, 2.0-3.2 N wipes); Dream-1: **0.0 at 2,400 steps** (the policy never picks a pad); pytest: **13 passed**. Media is top-view only in `benchmark-media-adhesive-residue-removal/`. No push, no upload.

On the returns bench: a real Calphalon Premier hard-anodized 12-inch pan (30.5 cm, 1.63 kg) upside down on a
3 mm silicone mat; a real Avery 5163 80 x 40 mm label was pulled off its base and lies on the waste tray. The left
hand grips the stainless handle (the pan is free, not clamped). The right hand lays a Chemtronics Chempad
(91% IPA) on the 80 x 40 mm adhesive patch, waits 20 s, wipes with a second pad, then wipes with a fresh white
third pad and leaves the base clean and dry. The real returns-desk step is a wipe for light residue; the pad is
the tool.

The residue is a declared 128-cell material-removal layer (16 x 8 cells of 5 mm). A cell is removed only by a
pad in real contact with the pan base: the live scorer reads the normal force and contact points from MuJoCo's
contact solver and the rubbing distance from the pad's motion in the pan's frame. A pad 7 mm above the base
removes nothing; a pad moving beside the patch removes nothing. The dwell makes the cells soft; the wipe removes
them by contact force and rubbing distance, not by a timer.

| File | What it is |
|---|---|
| `adhesive_residue_removal.py` | The `yam_arena` task file: the scene, the free pan, the 128-cell state, the pad wipe model, scoring and the scripted `demo()` |
| `assets/pan_calphalon.usda` | The 12-inch Calphalon pan: flared anodized bowl, 64 mm base, rim contact ring, stainless handle, rivets, hanging hole |
| `assets/pad_holder.usda` | A Chemtronics Chempad (76 x 102 mm, 3 mm felt) on an 80 x 106 mm backing foot with a 30 mm round grip bar |
| `assets/bench.usda` | The mat, three-pad tool tray, waste tray with the peeled label, foil packets, 5 mm grid overlay, 10x loupe and 30-degree raking LED |
| `assets/residue.usda` | 128 visual 5 mm cells on stiff slides; the free root follows the pan and a clean cell hides inside the base |
| `build_assets.py` | Writes the four USD files with pxr; rounded prisms, lathes, true masses |
| `test_adhesive_residue_removal.py` | 13 offline checks: idle, hover, dry wipe, pad placed/wet 0.25, dwell/soften, contact-only wipe, force below minimum, dirty-pad redeposit, paste outside, gritty-pad scratch, 60/90/98 ladder, 1 s hold, nudged pan |
| `benchmark-media-adhesive-residue-removal/demo.mp4` | The 1.0 demo, top camera only (640 x 360 H.264, 20 fps) |
| `benchmark-media-adhesive-residue-removal/setup.png` | The setup, top camera (640 x 360) |
| `benchmark-media-adhesive-residue-removal/demo_top.gif` | The top-view 1.0 demo as a GIF |
| `REFERENCE.md` | Products, sources and every real vs sim number |
| `REPORT.md` | The acceptance case, test/demo numbers, pass-through check and twin check |

## Run

```bash
cd benchmark-submission/tasks/adhesive-residue-removal
PYTHONPATH= ../../../local_sim/.venv/bin/python -m pytest -q test_adhesive_residue_removal.py

cd ../../../local_sim
PYTHONPATH= .venv/bin/python -m mjkit.run \
  ../benchmark-submission/tasks/adhesive-residue-removal/adhesive_residue_removal.py --steps 1800 --trace
```

`pytest`: **13 passed**. MuJoCo demo: **1.0 at step 1,446**; 128/128 cells clean by 64.4 s; the surface dry by
71.9 s; the 1 s hold completes at 72.3 s; the left hand still holds the handle and the right arm is home.

## Scoring

The 5 mm cell grid is the real before/after photo; a cell is clean at residue <= 0.01. The pad must be flat
(<= 20 degrees), at 0.5 N or more, with the pan held, to place and wet the patch: that is **0.25**. The ladder is
**0.5 at 60%**, **0.75 at 90%** with the fresh white final pad, and **1.0 at 98%** (126/128) plus a fresh white final pad (not used for the
dwell, load <= 0.05 g, used for the final wipe, not the dwell), a dry base (surface IPA <= 0.05), no scratch and no residue outside
the 80 x 40 mm patch, with the state held 1 s and the tool off the pan. Idle is 0. A dirty pad (load above 0.05 g)
redeposits a 0.12 film and caps the result; residue outside the patch is 0; a gritty pad that rubs latches a
scratch and scores 0. The real checks are the phone photo through the 5 mm grid, the white final Chempad, the
10x loupe and the 30-degree raking light. There is no gloss meter. The 98% threshold is not lowered.

## Media and pass-through

The final media is top-view only: `benchmark-media-adhesive-residue-removal/demo.mp4`, `demo_top.gif` and `setup.png`. **Pass-through check (PASS_THROUGH_QA):** frames at 0, 10, 25, 45, 60, 80, 95 and 99% were checked; the scene is open (an inverted pan on a mat, tools from above); no gripper, pad or held part intersects a visible solid; the left gripper meets only the handle and the right gripper only the pad holders; no wall, cover, shelf or enclosure is in the arm path. **Cutaways: none.**

## Real build

A Calphalon Premier 12-inch hard-anodized pan, a 3 mm silicone mat, an Avery 5163 label cut to 80 x 40 mm, three
Chemtronics Chempad CP400 pads (91% IPA), a 5 mm grid overlay, a 10x loupe and a 30-degree raking LED. Apply the
label with a card jig and a 2 kg roller, age 24 h at 23 C / 50% RH, peel at 180 degrees at 300 mm/s; put the
facestock on the waste tray. Lay pad 1 on the residue for 30 s, wipe with pad 2, then wipe with fresh pad 3; the
final pad must stay white. Photograph before/after through the grid; check the dry base and the raking light.

## Twin check

No clean twin on the board (through #2086): #2019 is griddle char removal; #1619 is a
heat-gun paint strip; #1154 frees a 3D print from a build plate; #1925 cleans an SC ferrule; #2289 applies a label.
None is a pressure-sensitive label-adhesive wipe on a hard-anodized pan base with a 5 mm photo grid, a white final
pad and a dry/scratch latch.
