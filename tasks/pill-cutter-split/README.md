# Split a scored tablet in a pill cutter and put one half in each of two medicine cups

Household medication preparation: an older adult or a family carer lays a scored tablet in the V of a cheap plastic
pill cutter, presses the lid down so the blade cuts it on the score line, and puts the two halves into separate cups.
Splitting a scored tablet to take half a prescribed dose is a familiar kitchen-table task for people managing their
own or a relative's prescriptions; the cut is the hard part, because the blade meets the tablet at **12-48 N** and a
light touch stalls on it instead of cutting. (The task is **Societal relatability**, not a jobs population count.)

- The tablet goes into the cradle with its score line at the blade.
- The lid is pulled shut by its fin, which cuts the tablet only after the blade's real resistance is overcome; the lid
  is opened again by the fin.
- One half goes in each cup, and the halves must be balanced (each 40-60% of the tablet).

| File | What it is |
|---|---|
| `pill_cutter_split.py` | The `yam_arena` task file: scene, force-gated blade cut, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/pill_cutter.usda` | Splitter: fixed base with the V cradle, lid on a revolute hinge (one articulation), blade shroud and rubber fin grip |
| `assets/tablet.usda`, `assets/half_tablet_7p5.usda` ... `half_tablet_12p5.usda` | The whole tablet and the five half lengths the cut can leave |
| `assets/pill_dish.usda`, `assets/cup_tray.usda` | The dish with a centring pocket and the tray with two cups (fixtures) |
| `build_assets.py` | Writes the USD files with pxr (boxes, prisms, cylinders) |
| `test_pill_cutter_split.py` | Offline checks: load, every scoring outcome, the force gate, the blade-edge cut, the 2.5 mm unequal-halves case, the shroud, the 1 s timing, reach (no GPU, no network) |
| `REAL_TRIAL.md` | The inert 20 mm scored tablet/surrogate, batch force check, half weighing, pad shield and residue rule |
| `pad_shield.scad` | The 0.6 mm PETG U-clip for the shared gripper pads |
| `task.json` | The task's details for the board |

## Assets

Created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and
kilograms, Z up.

- **Pill cutter** (`pill_cutter.usda`): a typical plastic splitter. Base 80 x 40 x 11 mm with a V cradle along y (20 mm wide at
  the rim, 5.5 mm deep). Lid 80 x 40 mm with a 12 mm clearance under its top plate, side rails, a blade over the cradle's
  middle and a **24 mm tall, 6 mm thick rubber-covered pull fin** standing near its free edge. The fin is one 14 x 6 x
  24 mm box with static/dynamic friction 2.5/2.0 (a tacky silicone overmold): the demo's 8 mm squeeze and 14-15 N a
  finger are unchanged, and the pinch carries the blade's 8.24 N at the fin.
  Hinge on the robot's side at the rim: revolute about y, 0 shut, 95 degrees open (stop), 0.6 N m s/rad damping and a
  **20 N m/rad drive** used for the blade brake. Base 22 g, lid 12 g, 30 g in all. The base is the articulation's fixed root.
- **Blade and shroud:** the blade's visual is 16 x 0.8 x 19 mm; its cutting edge is **7 mm below the lid plane** (table
  z 4 mm with the lid shut, 4 mm below a seated tablet's underside). A **0.6 mm PETG shroud** encloses the blade's upper
  7 mm (lid-frame z 5-12 mm, table z 16-23 mm with the lid shut): two side plates at y +/-0.7...1.3 mm (0.3 mm off the
  blade faces), a top plate and two end plates. Its lower edge is 0.5 mm above a seated tablet's 15.5 mm top. The lower
  12 mm (the cutting edge) is exposed only inside the V cradle, below the 11 mm rim, where the pads never go. A pad
  meets the shroud, not the blade. The pass-through `BladeCollider` covers the same upper 7 mm; the lower edge is left to
  the declared cut.
- **Tablet** (`tablet.usda`): 20 x 9 x 7.5 mm, 1.8 g, a visual score line across the middle. **Halves:** 7.5, 8.75, 10,
  10 (a second body), 11.25 and 12.5 mm long, masses proportional to length (0.675, 0.7875, 0.9, 0.9, 1.0125 and
  1.125 g). A 2.5 mm placement offset therefore leaves real 7.5/12.5 mm bodies, not two 10 mm stand-ins.
- **Dish** (`pill_dish.usda`): a 40 x 26 x 3 mm plate with an oblong pocket (11 x 21.5 mm, 2 mm deep, 2 mm chamfer).
  **Tray** (`cup_tray.usda`): a 56 x 120 x 4 mm tray holding two 30 mL medicine cups (38 mm inside, 30 mm tall) 60 mm
  apart.
- **The cut is force-gated and geometric.** While the blade is in the tablet (lid **16.5 to 5.5 degrees**) the hinge
  drive is a brake: the target is set `blade_resistance_n(lid)/20 rad` open of the current angle, so the drive reacts
  with a peak **0.56 N m = 8.24 N at the fin = 14.0 N at the blade** (the real 40 mm blade / 68 mm fin 1.7:1 lever),
  falling to **30% (2.47 N at the fin / 4.2 N at the blade)** as the score cracks. A closing force below the peak
  stalls the lid on the tablet and cuts nothing; the scripted demo's applied force at the fin is logged every second
  and reaches about **9.0 N** at engagement, above the 8.24 N peak. The cut is declared only when the **blade's actual
  cutting edge**, computed from the lid pose, has passed the tablet's **underside**; the blade plane (splitter y = 0)
  and the tablet's pose give the real split: a tablet 2.5 mm along the cradle leaves 7.5 and 12.5 mm halves
  (0.675/1.125 g), and the cut is scored at 2.5 mm off the score line, so step 4 fails. The half bodies are placed at
  the actual cut, 5 mm apart.
  **Measured basis.** UW-Madison BME design report, RERC FINAL, Table 1
  (https://bmedesign.engr.wisc.edu/projects/f07/pill_cap/file/view/7d95bd6b-2a10-43ae-a018-34a0a2f14d2b/RERC_FINAL.pdf):
  **1.25-5 lbf (5.6-22.2 N) at the handle, 2.71-10.8 lbf (12-48 N) at the pill**; average 7.37 lbf (32.8 N) at the pill. The
  task models **14.0 N at the blade / 8.24 N at the fin**, the low end of that range (the surrogate's batch acceptance
  is 14.0 +/- 1.5 N at the blade, inside 12-48 N). The force is not lowered below the measured 12 N; a 40 N tablet
  model is not cut by the same demo (MuJoCo: lid stalls, 0.2). A separate 47 N tablet hardness (nebivolol 5 mg,
  3.13 mm, [PMC8720793](https://pmc.ncbi.nlm.nih.gov/articles/PMC8720793/)) is the top of the range.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pill_cutter_split.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pill-dream

# Scripted demo (left arm), 2,200 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pill_cutter_split.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2200 --store-frames --log-dir ~/yam-eval/pill-demo
```

The job log prints `[pill_cutter_split] t=...  steps done: ...` when a step is done or undone, a line when the blade
engages and when the edge passes the tablet underside (`blade edge passed ... peak F_fin 8.24 N ... cut into ...`), and
a trace every second: the lid's angle, the blade force at the fin and edge height, the tablet's place against the cradle,
the cup each half is in.

**The demo** (Isaac times: seat 10.8 s, blade engaged 41.55 s, cut 45.60 s, lid open 72.10 s, both cups 101.75 s,
1.0 at step 2,045; MuJoCo: cut 45.05 s, 1.0 at step 2,045 / 102.25 s; run limit 2,400). Left arm, open loop: opens the pads to 29 mm, pinches the tablet across
its width in the dish pocket with a gentle 2.5 N a finger, sets it in the cradle; pulls the lid's fin from 95 to 50
degrees with the 14-15 N a finger pinch, slows the last 50 degrees to 2 s per 8 degrees, takes the fin again for the
last stretch and pushes through the 8.24 N peak (the applied force is logged); the cut is at lid 5.6 degrees, 0.0 mm
off the score line, 0.896/0.904 g halves. It then pinches the fin again to open the lid to its 95 degree stop, lifts each
half across its width and lets go inside its cup. The pause before the cut is the brake: the arm has to push.

## Instruction

It says the goal (split the tablet, one half in each cup) and leaves out which hand, where things are and how.

## Scoring

Four steps, **0.2 each**; steps 1 and 2 once reached, steps 3 and 4 as things stand.

| Step | Done when |
|---|---|
| 1. Tablet in the cradle at the blade | its middle within 2.5 mm of the blade and the cradle's line, length along the cradle within 8 degrees, level within 8 degrees |
| 2. Lid shut on it: tablet cut | the lid reached shut (within 3 degrees) and the blade's actual edge passed the tablet's underside |
| 3. Lid opened again | after the cut, the lid is past 80 degrees |
| 4. One half in each cup, balanced, grippers clear | one half in the orange cup and the other in the blue one; the cut plane within **1 mm** of the score line; each half's actual mass **40-60%** of 1.8 g (0.720-1.080 g); no pad within 3 cm of either half or the fin |

- **1.0 (success):** every full-success condition held together, without a break, for 1 s: all four steps, no violation,
  and each half within 2 mm and 2 degrees of where it was when that second began. The second is timed inside
  `success()` and any lapse restarts it. The step points stop at 0.8, also while the second is being timed.
- **0.6** for a 2.5 mm placement offset: the cut still happens (the blade plane is at the splitter's centre) but leaves
  7.5 and 12.5 mm, 0.675/1.125 g halves and is 2.5 mm off the score line, so step 4 is false; steps 1-3 are 0.6. A
  tablet at 1.1 mm off the score line also fails step 4 (the 1 mm cut-line check).
- **0** if the tablet or a half falls off the table, or the state is not finite.
- **Guards:** nothing is cut unless the blade edge passes the tablet's underside with the whole tablet in the cradle;
  closing the lid on an empty or badly seated cradle scores nothing for the cut; both halves in one cup does not count; a
  lid left shut after the cut does not get step 3; a hand left at a half or the fin does not get step 4. The 40-60%
  mass check and the 1 mm score-line check match the real-trial check in `REAL_TRIAL.md`.

## Real trial

`REAL_TRIAL.md` gives a **small-pack, readymade 20 mm scored tablet** — **Windmill High Potency Calcium 600 mg**,
60 tablets, UPC 035046000646, **$5.19**, 20 mm, scored, 1.7-2.0 g; measure its cut force and use it only if 12-17 N —
and a **few-minute fast-set-stone surrogate**: **DAP Plaster of Paris**, 4 lb, about **$6**, sets **20-30 min with no
24 h drying**; an 8-cavity PETG mould with a 0.8 mm score ridge makes a batch of eight in one 25-30 min pour
(**3-4 min per tablet**), 20 x 9 x 7.5 mm, 1.80 +/- 0.10 g. Both avoid a 1,000-tablet order and the old 45 min + 24 h
cure. The sheet also has the **0-50 N push-pull gauge** batch acceptance (**8.2 +/- 0.9 N at the fin / 14.0 +/- 1.5 N
at the blade**, Shore D 55-75 cross-check), the **0.01 g half weighing** (each 40-60% of the pre-cut mass) with the
**10x +/-1 mm score-line check**, the **0.6 mm PETG pad U-clips** (`pad_shield.scad`, Prusament PETG + 3M VHB 5952)
that leave the inner grip face and both scored squeezes unchanged, and the **91% IPA wipe and 10x inspection** after
every trial.

## Reset

Open the lid to its 95 degree stop, lay the tablet in the dish pocket with its length left-right, stand the tray on its
spot. The tablet has about 1 mm of play each way in the dish pocket; the demo's pick is centred on the pocket and its
29 mm opening and gentle pinch take it within 0.3 mm of the cradle centre (the MuJoCo cut is 0.0 mm off the score
line). After a real trial, wipe the pads, blade shroud and cradle with 91% IPA and inspect (REAL_TRIAL.md).

## Pass-through check

Checked frames: top camera at 0, 10, 25, 45, 60, 80, 95 and 100% of `demo.mp4`, and the left wrist camera at the dish
pinch, cradle set-down, fin pull and both cup drops. No gripper, finger, forearm or held part intersects a visible solid.
The splitter's blade is visible inside the lid; its upper 7 mm is inside the 0.6 mm PETG shroud (side plates 0.3 mm off
the blade, top and end plates), so a pad meets the shroud; the lower 12 mm cutting edge is inside the V cradle below the
11 mm rim. The blade-slot line and the tablet's score line are flush markings, not solids; the rubber feet are visual
pads under the base, outside the arm path. Cutaways: none.

## Status

- MuJoCo: **1.0 at step 2,045** (102.25 s; 2,200-step scripted runner, 2,400 run limit). Cut at lid 5.6 degrees,
  blade edge 7.9 mm, halves 0.896/0.904 g, 0.0 mm off the score line.
- Isaac: **1.0 at step 2,045** (101.75 s; cut at 45.60 s, lid open 72.10 s, both cups 101.75 s; 2,200-step
  scripted runner, 2,400 run limit). The log's cut line: `blade edge passed ... at lid 5.6 deg; peak F_fin 8.24 N ...
  cut into 10.0/10.0 mm halves, 0.902/0.898 g, cut 0.0 mm off the score line`.
- Dream-1: **0.0 at 2,400 steps** (completed, hit the step limit; **2,401 action rows** in
  `actions/20261117_033314_ad02b01d/scene-0-e0.jsonl`, model `dreamlabs/dream-1-beta`, log `adhoc_44b767ed.json`; the
  top-camera `dream.mp4` is rendered from the same run). The model made real actions (the arms moved) but never took
  the tablet; a `pool_full` or no-action run is not counted, and this run is not one.
- pytest: **45 passed** (the previous outcomes plus the force gate, the profile, the blade-edge cut, the 2.5 mm
  unequal-halves case, the balanced-halves step, the shroud, the fin width and the two squeezes).
- Starts (MuJoCo): nominal **1.0**, cut 0.0 mm off the score line; tablet start **+3 mm in x** **1.0** (cut 0.1 mm
  off); **+3 mm in y** **0.6** (cut 1.6 mm off, 8.8/11.2 mm, 0.752/1.048 g) — the open-loop demo cannot correct a
  large start offset, and the 1 mm cut-line check says so rather than hiding it.
- Tablet source (practicality revision): **Windmill High Potency Calcium 600 mg**, 60 tablets, UPC
  035046000646, **$5.19** (20 mm, scored; use only if its measured cut force is 12-17 N), or the **DAP Plaster of
  Paris** surrogate (4 lb, about **$6**, sets 20-30 min, **no 24 h**; an 8-cavity batch in 25-30 min = **3-4 min per
  tablet**; 20 x 9 x 7.5 mm, 1.80 +/- 0.10 g, 14.0 N batch check). The sim tablet is unchanged (20 x 9 x 7.5 mm,
  1.8 g, 14.0 N), so **no new media** is needed.
- **Weak/strong gate checks (MuJoCo):** the same demo with a 40 N tablet model (23.5 N at the fin, above the arm's
  pinch) stalls the lid at its 14-16 degree engagement and cuts nothing: score 0.2. The task's 14.0 N model cuts.
- Media: `benchmark-media-pill-cutter-split/` - `demo.mp4` (top camera, 3 s held final frame with SCORE 1.0, the PR
  media), `demo_top.gif`, `setup.png`; `demo_review.mp4` is the four-view internal composite. Uploaded as **M-T4GAZH** (setup screenshot, Dream-1 run and scripted demo on the task board under this ID). The sim tablet is unchanged, so this practicality revision adds no media.
