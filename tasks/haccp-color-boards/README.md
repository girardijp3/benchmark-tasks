# Colour-coded prep: raw pork, chicken and cod each on their own board, sanitising between foods

A bimanual food-safety task. Three colour-coded HDPE cutting boards lie in a row (yellow, red, blue), a stainless pan
holds three raw cuts (a chicken thigh, a pork loin chop and a cod fillet) and a tub of sanitiser stands
at the right. Each cut belongs on one board: red for raw meat, yellow for raw poultry, blue for raw fish (the US
food-service colour code).
- The left arm sets the chicken thigh on the yellow board.
- The right arm sets the pork chop on the red (middle) board, sanitises its pads, then sets the cod on the blue board.
- Both go back, leaving each cut lying flat on its own board.

The skill is not the carrying, it is keeping raw foods apart. Cross-contamination is a penalty that lasts the whole
run: a cut that touches the wrong board for a single control step, a gripper that goes from one raw food to another
without being sanitised, or unsanitised pads touching another food's board all score 0 however the scene looks
afterwards. The cuts are rigid stand-ins for soft meat (the robot cannot handle soft meat), but they are irregular
shapes of true size and mass with rounded edges, not blocks.

| File | What it is |
|---|---|
| `haccp_boards.py` | The `yam_arena` task file: scene, scoring (`steps_done`, `success`, `score`, the cross-contamination checks) and the scripted `demo()` |
| `assets/board_red.usda`, `board_yellow.usda`, `board_blue.usda` | The three cutting boards (OpenUSD, fixed to the bench) |
| `assets/prep_pan.usda` | The stainless gastronorm pan the cuts start in (fixed) |
| `assets/sanitiser_tub.usda` | The sanitiser dip tub (fixed; the solution is a visual slab) |
| `assets/raw_thigh.usda`, `raw_pork.usda`, `raw_cod.usda` | The three raw cuts (one rigid body each, convex collider) |
| `test_haccp_boards.py` | Offline checks: the file loads, every scoring outcome, persistence of contamination, the sanitiser rule, the 1 s full-success timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All assets: created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Units are metres and kilograms, Z up. Sizes and their sources:

- **Cutting boards** (3 x 0.9 kg HDPE, fixed): 229 x 305 x 12.7 mm (9 x 12 x 1/2 in), a stock size of NSF-listed
  colour-coded boards in food-service catalogues. Red, yellow and blue top faces, a slightly darker lower edge band
  and a faint juice-groove outline (visual only). Friction (static/dynamic) 0.35/0.3. The colour code is the US
  food-service one (red raw meat, yellow raw poultry, blue raw fish and seafood, e.g. as taught in ServSafe-style
  courses); other countries differ (in the UK yellow is cooked meat), so the task says "HACCP colours" and the setup
  gives the sizes, not the code.
- **Prep pan** (fixed): stainless gastronorm 1/2 pan, 265 x 325 x 20 mm (EN 631-1 size, 20 mm depth), 1.5 mm sheet.
  Floor and four wall colliders. Friction 0.4/0.3.
- **Sanitiser tub** (fixed): 150 x 200 x 70 mm polypropylene tub (typical food-service dip tub, a typical size, not a
  specific product), 3 mm walls, solution 40 mm deep (a visual slab with no collider: the pads enter it freely).
- **Chicken thigh**: boneless, skinless, 100 x 67 x 24 mm, 114 g. A rounded teardrop outline, flat underside,
  near-vertical wall with a round-over, domed top, dark red-brown.
- **Pork loin chop**: boneless, 112 x 69 x 22 mm, 131 g, a rounded trapezoid with a pale fat edge.
- **Cod loin fillet**: skinless, 170 x 58 x 28 mm, 193 g, white flesh tapering to a thin tail.
  The sizes are those of 4 to 7 oz retail portions (typical values from retail portion guides; no single product was
  measured). **They are rigid**: real raw meat is soft and deforms in the grip; here each cut keeps its shape, and
  the README, the fidelity note and the video say so.
- **Cut physics**: one rigid body each, mass from the real density of raw meat and fish, 1.06 g/cm3 x the hull's
  volume (107, 124 and 182 cm3). Collider: one convex hull of 61 vertices (the playbook's 64-vertex limit). Friction
  0.6/0.5 (wet raw meat on HDPE and steel is about 0.4 to 0.6), restitution 0. The visual mesh is finer than the hull
  and agrees with it to under 1 mm.

**Layout** (m, x forward, y to the robot's left, from the midpoint between the arm bases): boards centred at x 0.40,
y 0.32 (yellow), 0 (red), -0.32 (blue), 15 mm apart; pan at x 0.145; thigh at y 0.105, cod at y 0, pork at y -0.105
(their lengths along x); sanitiser tub at x 0.16, y -0.38. The robot's reach (0.6 m) decides the layout: the middle
board is the farthest point either arm reaches, so the pork is set 30 mm nearer the robot than the board's centre.

## Scoring (what the code does)

Four steps of 0.2, judged as things stand, never more than 0.8 until full success:

| Step | Condition |
|---|---|
| 1 | The pork lies on the red board: its lowest point within 1.5 mm of the top face, its whole underside over the board, tilted less than 10 degrees from flat |
| 2 | The chicken thigh lies on the yellow board, the same |
| 3 | The cod lies on the blue board, the same |
| 4 | Steps 1 to 3 and no gripper's pads within 3 cm of any cut (a clear release) |

**Full success (1.0, ends the run)**: all four steps have held together, without a break, for 1 s, timed in the task
file, with each cut within 2 mm and 2 degrees of where it was when that second began; any lapse restarts the second.
Where the arms are is not scored. `SUCCESS_HOLD_S` is 0 (the timing is the task's own).

**Cross-contamination (0 for the whole run)**: checked at every control step, kept for the run, never undone by
moving the food back:
1. a cut touching a board that is not its own: any of 2,500 points sampled over its collider's surface (and its
   vertices) on that board's top face (within 1 mm) or lower, inside the board's outline;
2. a gripper whose pads held one kind of raw food gripping a different kind before they were sanitised (a food is
   held when its pads' span reaches the cut's bounds, both fingers press with at least 0.5 N and the pads are at
   least 3 mm apart);
3. unsanitised pads that held a raw food touching a board that is not that food's (a pad tip within 1.5 mm of the
   top, the pads reaching half their opening to each side).

Pads are **sanitised** when they are held open (20 mm apart or more) with the tips between 2 mm above the tub's floor
and 10 mm under the solution's surface, inside the tub, holding nothing, for 2 s without a break. A cut that falls
off the table also scores 0. A state that is not finite scores 0.

These rules read poses and finger states the robot's own actions produce; the task code moves nothing, holds nothing,
and decides no success by itself. What the task does **not** model: soft meat, juices, pathogen transfer, the real
10 to 60 s sanitiser contact time (2 s here, to fit the run), or the two pads as separate surfaces (they count as one).

## Run it

On the YAM Isaac Lab-Arena station, from this folder, with the Arena environment active. The instruction is
`task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=haccp_boards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/haccp-dream

# Scripted demo (both arms), 820 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=haccp_boards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 820 --store-frames --log-dir ~/yam-eval/haccp-demo

# Offline checks (no GPU)
python -m pytest -q test_haccp_boards.py
```

The scripted demo plays `demo()` (open loop: it reads no state). It prints a trace line each second and every change of
the step timeline, `[haccp_boards] t=...`, and every cross-contamination event and sanitising.

## The demo's timeline

Both arms grip straight down with the fingers closing across a cut's width, the pads 4 mm above the pan's floor,
squeezing 8 mm inside the cut's width (about 20 N a finger).

| Time (s) | Left arm | Right arm |
|---|---|---|
| 0 to 5.7 | over the thigh, down beside it, pads round it, grip, hold | the same on the pork |
| 5.7 to 11.2 | lift, carry it to the yellow board, lower it to 1.5 mm above the board | the same, to the red board |
| 11.2 to 12.8 | let go, lift away, go home | let go, lift away |
| 12.8 to 15.3 | home | over the sanitiser tub |
| 15.3 to 20.8 | home | pads open into the solution, held 2.8 s, out |
| 20.8 to 33.6 | home | the same carry with the cod, to the blue board |
| after | | go home (not scored) |

On Isaac (L40) the same demo runs unchanged: first two steps at t 11.4 s, the third at 32.2 s, success conditions met
at 34.6 s, 1.0 after the 1 s hold.

## Instruction

"Lay out the raw pork chop, chicken thigh and cod fillet on the cutting boards matching their HACCP colours, and keep any raw food from contaminating another." It states the goal and what must not happen, not the colour code, the
sanitising or the order. Hand roles are not given: either arm may do any of it.

## Reset

Put the three cuts back in the pan in a row (thigh at y 0.105, cod at 0, pork at -0.105, each lying flat with its
length along x); boards in their places and dry; the tub stays filled; both grippers start clean (the scorer starts
each episode with clean pads and no events).

## Differences from a real kitchen (stated, not hidden)

- The cuts are rigid. A real cut deforms, sags and slips in a grip; a rigid one is easier to hold. The shapes, sizes
  and masses are real, the edges are rounded, and the pinch is across a width of 58 to 69 mm of a 94 mm gripper.
- Contamination is read from poses. In a real trial a person or a swab would judge contact; here the scorer's rule is
  the same as the trial's (any contact with the wrong board counts, and it cannot be undone).
- The sanitiser is a visual slab with a timer, not a fluid.
- Colour recognition (which board is for which food) is the policy's to know: nothing in the scene labels them.
