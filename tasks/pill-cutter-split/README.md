# Split a scored tablet in a pill cutter and put one half in each of two medicine cups

A one-arm task on a care-home med pass and in a pharmacy's unit-dose packing. Pharmacy technicians (SOC 29-2052, about 0.46M)
split scored tablets when packing unit-dose cards; nursing assistants (SOC 31-1131, about 1.4M) split them at the med cart. The
scored tablet goes in the V cradle of a pill cutter, the lid is shut so the blade splits it along the score line, the lid is opened
again and the halves are set out for two doses. Here a 20 x 9 x 7.5 mm caplet (1.8 g) lies in a pill dish; an 80 x 40 mm pill
splitter with its lid open (95 degrees, past upright, so it stays open) stands beside it; a dosing tray with two 30 mL medicine cups
(orange and blue) stands beyond.

- The caplet goes into the cradle with its score line at the blade.
- The lid is shut by its pull fin, which cuts the caplet, and opened again by the fin.
- One half goes in each cup and the hand moves clear.

| File | What it is |
|---|---|
| `pill_cutter_split.py` | The `yam_arena` task file: scene, the cut event, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/pill_cutter.usda` | Splitter: fixed base with the V cradle, lid on a revolute hinge (one articulation) |
| `assets/tablet.usda`, `assets/half_tablet.usda` | The whole caplet and one half (rigid bodies) |
| `assets/pill_dish.usda`, `assets/cup_tray.usda` | The dish with a centring pocket and the tray with two cups (fixtures) |
| `build_assets.py` | Writes the USD files with pxr (boxes, prisms, cylinders) |
| `test_pill_cutter_split.py` | Offline checks: the file loads, every scoring outcome, the 1 s timing, the cradle test, reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

Created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and
kilograms, Z up.

- **Pill cutter** (`pill_cutter.usda`): a typical plastic splitter. Base 80 x 40 x 11 mm with a V cradle along y (20 mm wide at the
  rim, 5.5 mm deep) built from two convex prisms. Lid 80 x 40 mm with a 12 mm clearance under its top plate, side rails that rest on
  the base, a blade over the cradle's middle and a 24 x 14 x 6 mm pull fin standing near its free edge. Hinge on the
  robot's side at the rim height: revolute about y, 0 shut, 95 degrees open (stop; past upright, so gravity holds it there),
  0.6 N m s/rad of friction (a damped drive), lid 12 g, base 22 g, 30 g in all (typical of a plastic splitter). The base is the
  articulation's fixed root. The blade's visual is 16 x 0.8 x 18 mm; a **pass-through collider** covers its upper 7 mm (lid-frame
  z 5 to 12 mm, table z 16 to 23 mm with the lid shut), above the caplet's 15.5 mm top in the cradle, so the arm cannot pass
  through the blade and the collider cannot push the caplet or the halves at the cut. The lower 11 mm is the cutting edge that
  enters the caplet at the cut; it is left to the declared cut (the caplet is replaced, not crushed).
- **Caplet** (`tablet.usda`): 20 x 9 x 7.5 mm, 1.8 g, a score line across the middle. **Half** (`half_tablet.usda`): 10 x 9 x 7.5
  mm, 0.9 g. A 1000 mg caplet is typically 19-22 mm long.
- **Dish** (`pill_dish.usda`): a 40 x 26 x 3 mm plate with an oblong pocket (11 x 21.5 mm, 2 mm deep, 2 mm chamfer) so the caplet
  rests centred, like a pill tray's cell. **Tray** (`cup_tray.usda`): a 56 x 120 x 4 mm tray holding two 30 mL medicine cups (38 mm
  inside, 30 mm tall, 14 wall segments each), 60 mm apart.
- **The cut** is declared task code: every control step, if the lid is within 3 degrees of shut and the whole caplet lies in the
  cradle (middle within 2.5 mm of the blade and the cradle's line, at its seat height, length along the cradle within 8
  degrees, level within 8 degrees), the caplet is replaced by its two halves, 5 mm apart on the cradle's walls (the blade
  wedges them apart), and the whole caplet is moved out of the way. The two halves wait out of the way until then.
  **Blade cut force, sim vs real.** The blade is not simulated and the sim scores no cut force. Real, measured on a standard
  hand-held pill cutter [UW-Madison BME design report, RERC FINAL, Table 1](https://bmedesign.engr.wisc.edu/projects/f07/pill_cap/file/view/7d95bd6b-2a10-43ae-a018-34a0a2f14d2b/RERC_FINAL.pdf): **1.25-5 lbf (5.6-22.2 N) applied at the
  handle, giving 2.71-10.8 lbf (12-48 N) at the pill**; the average was 7.37 lbf (32.8 N) at the pill. The demo's fin grip
  is about 14-15 N (3.1-3.4 lbf) a finger, inside that measured applied range; through this splitter's 1.7:1 lever (fin
  68 mm, blade 40 mm) that is about 24-26 N at the pill, inside the measured 12-48 N. The sim's cut is gated on the lid
  angle and the caplet seated, not on the blade force; the cut force is stated here and is not reduced. A separate 9.1 mm
  scored tablet hardness of 47 N (nebivolol 5 mg, 3.13 mm thick, [PMC8720793](https://pmc.ncbi.nlm.nih.gov/articles/PMC8720793/)) is consistent with the top of that range.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pill_cutter_split.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pill-dream

# Scripted demo (left arm), 1,900 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pill_cutter_split.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1900 --store-frames --log-dir ~/yam-eval/pill-demo
```

The job log prints `[pill_cutter_split] t=...  steps done: ...` when a step is done or undone, a line when the lid shuts on the
caplet (`cut into two halves`), and a trace every second: the lid's angle, the caplet's place against the cradle, the cup each half is in.

**The demo** (times on the Isaac station: seat 10.8 s, cut 37.9 s, lid open 64.4 s, both cups 93.4 s, 1.0 at 94.95 s).
Left arm, open loop: opens the pads to 29 mm, pinches the caplet across its width in the dish pocket with a gentle 2.5 N a
finger (a 20 N pinch ejected the 1.8 g caplet up the pocket chamfer on PhysX; the pad tips sit 4.1 mm above its underside, which
leaves 0.55 mm to the finger carriage and 1.1 mm to the 11 mm cradle rim), lifts it and sets it in the cradle; goes up over the
lid and pinches the lid's fin (fingers closing along y, about 14-15 N a finger) to carry the lid down from 95 to 50 degrees,
lets go and takes the fin again with a different wrist angle to bring it shut; the caplet is cut. It pinches the fin again to
swing the lid up to 50 degrees, again with a second grip to the 95 degree stop. It then lifts each half in turn, across its
width with the same gentle pinch, over a medicine cup and lets go inside.

## Instruction

It says the goal (split the tablet, one half in each cup) and leaves out which hand, where things are and how.

## Scoring

Four steps, **0.2 each**; steps 1 and 2 once reached, steps 3 and 4 as things stand.

| Step | Done when |
|---|---|
| 1. Caplet in the cradle at the blade | its middle within 2.5 mm of the blade and the cradle's line, length along the cradle within 8 degrees, level within 8 degrees |
| 2. Lid shut on it: caplet cut | the lid reached shut (within 3 degrees) with the whole caplet in that place (the cut event) |
| 3. Lid opened again | after the cut, the lid is past 80 degrees |
| 4. One half in each cup, grippers clear | one half in the orange cup and the other in the blue one (inside the wall, below the rim), no pad within 3 cm of either half or of the lid's fin |

- **1.0 (success):** every full-success condition held together, without a break, for 1 s: all four steps, no violation, and each
  half within 2 mm and 2 degrees of where it was when that second began. The second is timed inside `success()` and any lapse
  restarts it. The step points stop at 0.8, also while the second is being timed. The tests check no 1.0 at
  0.8 s, 1.0 at 1 s, a lapse restarting it, and no 1.0 while a step is missing or a hand is near. The arms' return is not scored.
- **0** if the caplet or a half falls off the table, or the state is not finite.
- **Guards:** nothing is cut unless the caplet really lies in the cradle at the blade when the lid shuts, so closing the lid on an
  empty or badly seated cradle scores nothing for the cut; both halves in one cup does not count; a lid left shut after the cut
  does not get step 3; a hand left at a half or the fin does not get step 4.
- **Real station:** the halves would be checked in each cup (a weighing, or a look into the cups), and the cut by the line
  across the score.

## Reset

Open the lid to its 95 degree stop, lay the caplet in the dish pocket with its length left-right, stand the tray on its spot.
The caplet has about 1 mm of play each way in the dish pocket; the demo's pick is centred on the pocket and its 29 mm opening
and gentle pinch still take it 3 mm / 3 degrees off nominal (four such starts pass in MuJoCo).

## Pass-through check

Checked frames: top camera at 0, 10, 25, 45, 60, 80, 95 and 100% of `demo.mp4` (0, 9.5, 23.6, 42.5, 56.7,
75.6, 89.8 and 94.5 s), and the left wrist camera at 5, 10, 35, 37, 64, 93 and 94 s (the dish pinch, cradle set-down, fin
pull and both cup drops). No gripper, finger, forearm or held part intersects a visible solid.
The splitter's blade is visible inside the lid and its upper 7 mm has a collider above the caplet; the lower cutting edge is
the declared cut. The blade-slot line and the caplet's score line are flush markings, not solids; the rubber feet are visual
pads under the base, outside the arm path. Cutaways: none.

## Status

- MuJoCo: 1.0 at step 1,889 (94.45 s; 1,900-step scripted runner, 2,400 run limit).
- Isaac: 1.0 at step 1,889 (94.45 s simulated, 94.95 s in the task log; 1,900-step scripted runner, 2,400 run limit).
- pytest: 37 passed (idle, no pinch, caplet in the dish, caplet in the cradle un-pinched, shut lid with no cut, lid left shut,
  both halves in one cup, near gripper, off-table, the 1 s timing and lapse, the gentle pinch, the blade collider, the pad
  penetration latch).
- Nudges (MuJoCo): 4 starts with the caplet 3 mm / 3 degrees off nominal (x +/-, y +/-), 1.0 each; 4 more at the 1 mm
  pocket-play edges with 3 degrees of yaw, 1.0 each.
- Dream-1 score: 0.0 (2,400 steps, hit the step limit; the Dream service returned `pool_full` for the whole run and sent no actions).
- Media: `benchmark-media-pill-cutter-split/` - `demo.mp4` (top camera, 3 s held final frame with SCORE 1.0, the PR media), `demo_top.gif`, `setup.png`;
  `demo_review.mp4` is the four-view internal composite (top, left wrist, top crop on the cradle and a scoring panel).
