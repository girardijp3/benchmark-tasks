# Refit a washed baffle filter in a range hood

A kitchen-cleaning task. A freshly washed stainless baffle filter lies on the counter and its hood waits with a bottom trough and a spring latch. The robot has to refit it the way a kitchen worker does: carry the 1.2 kg slab by its two handles, stand its bottom edge in the trough with the top swung out, pull the latch up against its spring so the top can swing in, swing the filter flat to the panel and let the latch drop over its top edge. The instruction states only the goal; the carry, the order and which hand does what are the robot's to work out.

The skill is a heavy slab handled by two hands (more than one arm should hold), a lead-in into a trough, then a spring latch worked by one hand while the other holds the slab: with the latch down, its hook stops the top edge from swinging in at about 12 degrees, so the latch has to be pulled up against its spring and held while the filter goes in, then let go.

**Why it matters.** Dishwashers (SOC 35-9021, about 0.5 million in the US), restaurant cooks (35-2014, about 1.3 million) and kitchen cleaners refit hood filters at the nightly or weekly close of every commercial kitchen: the baffle filters go through the dish machine and each has to be stood back in its trough and latched before the next service, because a missing or loose filter lets grease into the duct, a fire risk. The job is awkward for one person and a good fit for a robot at closing time: a 1.2 kg wet slab handled with two hands while one hand also works a sprung latch.

**Station, stated plainly.** The filter bank here sits at counter height and leans back 30 degrees above horizontal so the table-mounted arms reach it. In a real hood the bank is overhead, at 45 degrees or more from horizontal, and the filter faces down: there gravity swings the filter's top out and the latch is what holds it up. Here gravity holds the filter on the panel, so the latch is scored as the retention step (it has to be pulled the full 16.5 mm against its spring for the top edge to pass, then let down over that edge).

| File | What it is |
|---|---|
| `range_hood_baffle_filter.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/hood.usda` | The hood panel (OpenUSD): fixed-root articulation, trough, a spring latch plate on a prismatic joint with a spring drive, handle and visible coil |
| `assets/filter.usda` | The baffle filter (OpenUSD): one free rigid body, collider slab, visual frame, chevron baffles and two drop handles |
| `build_assets.py` | Writes the USD files with pxr (boxes, prisms and meshes; the closed 96-sided round-part helpers are inlined) |
| `test_range_hood_baffle_filter.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Hood panel** (fixed root, one articulation): a 580 x 314 mm back plate leaning back 60 degrees from vertical, a trough (floor, a front lip 18 mm high 49.5 mm from the panel, and a 45 degree lead-in plate) and a latch plate (a 50 x 40 x 3 mm hook plate 2.5 mm in front of the filter's face, an ear and a T handle) on a prismatic joint along the panel with a spring drive (150 N/m, 3 N preload, 16.5 mm travel). The colliders are those boxes, invisible; the visible parts are separate meshes: a brushed stainless panel with folded hems, side guide channels, a formed gutter with a rolled lip and end caps, an upper panel with folded side walls and a top canopy, riveted trim, a brass hook plate with rounded corners, rounded T handle and rivets, guides, bracket, a plunger rod (96-sided) and a coil (24-sided wire). The coil is a static mesh and does not compress when the latch moves.
- **Filter** (free rigid body, 1.18 kg): a 406.4 x 254 x 44.45 mm solid collider (invisible); the visible part is a sheet-metal channel frame (web, flanges, rolled beads, riveted) with two rows of interlocked chevron baffles (12 and 11) so the solid slab looks like a real baffle filter; the baffles are visual only. Two drop handles (90 mm crossbar, 15 x 10 mm, 22 mm out) on the face at y +-0.16, with rounded visual posts, bar and washers over the same collider boxes.
- Sizes: the filter is a Component Hardware FR51-1016 class 10 x 16 x 1.75 in stainless baffle filter (2.6 lb in the retailer listing); the handles follow AllPoints 263888 (drop handles; sizes estimated). The trough, lip and hook sizes are estimates: no hood maker publishes them.
- **Latch spring (estimate, stated plainly):** 5.5 N at full travel (150 N/m, 3 N preload, 16.5 mm). No hood-latch force is published, so this is an estimate inside the cited TE-CO spring-plunger light-force band of about 3 to 10 N, to be checked with a force gauge on a real latch. A first version at 8.5 N was beyond what the arm could deliver while holding the latch plate up, so it was lowered to 5.5 N; the success logic is unchanged (the plate must still be pulled the full travel against its spring and let down over the top edge). In the demo the right hand is commanded 30 mm past the plate's stop (PULL_EXTRA) so that the arm's stiffness delivers that force; the latch itself only moves 16.5 mm.

## Layout

The panel's trough corner is at x 0.27, 0.02 m above the table; the filter lies at x 0.115. Both arms grip the handle crossbars with the pads closing up the panel's slope (fingers pointing into the filter's face); the right hand takes the latch's T handle beyond the filter's right end.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=range_hood_baffle_filter.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/range-hood-baffle-filter-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=range_hood_baffle_filter.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/range-hood-baffle-filter-demo
```

The log prints `[hoodfil] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. Both arms take the filter by its two handles (about 20 N a finger), lift it 60 mm, carry it over the trough with the top swung out 35 degrees and stand its bottom edge in. They swing the top in to 25 degrees; the right hand lets go, takes the latch's T handle and pulls it up its 16.5 mm against the spring (5.5 N, the arm is commanded 30 mm past the stop); the left hand swings the filter flat to the panel; the right hand lets the latch down and it drops over the filter's top edge; both hands let go and back away out of the face. Success at step 795 (39.75 s).

## Scoring

| Step | Done when |
|---|---|
| 1 | the filter's bottom edge is in the trough (both bottom edges between panel and lip, within 45 mm of the floor, centred within 30 mm) |
| 2 | with step 1, the filter lies flat on the panel and floor (centre within 3 mm of the face offset, 6 mm along the panel, 30 mm along y; within 3 degrees) |
| 3 | with step 2, the latch plate is back down within 1.5 mm of its spring stop |
| 4 | with step 3, no gripper holds the filter and no pad is within 3 cm of it |

The latch plate cannot be passed by the top edge while it is down (the hook plate stops the swing at about 12 degrees), so step 2 needs the latch pulled and held.

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success():
  all four steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the filter dropped or off the table); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: a filter laid on the panel without the trough scores nothing, a filter propped against the hook scores 0.2 (step 1 only), the latch down alone is not enough, and a filter released before the latch drops leaves the ladder at 0.4 (all tests). In MuJoCo, doing nothing, the left arm alone and the right arm alone all score 0, and a demo with the right hand not pulling the latch scores below 1.
- **Real station (instruments):** a straightedge across the filter face against the panel (flat within 3 mm); the latch hook seen over the filter's top edge and a tug on the filter's handles (it must not swing out); a force gauge on the latch handle (the spring force over its travel); a ruler for the 3 cm hands-clear distance and a phone video for the 1 s of stillness.

## Reset

Lay the filter flat on the counter at x 0.115 with its handles up and the latch plate down at its spring stop. Nothing is held.

## Fidelity notes

Full text of the task.json fields, shortened there to meet the length limits:

Fidelity: Real part: Component Hardware FR51-1016 class 10 x 16 x 1.75 in baffle filter, 2.6 lb (listing); drop handles per AllPoints 263888 (sizes estimated). Estimates: latch spring (3 N preload, 150 N/m, 5.5 N at full travel: no hood-latch force is published, so this is an estimate inside the TE-CO spring-plunger light-force band of 3 to 10 N; a first 8.5 N version was beyond the arm and was lowered), trough, lip, hook overlap, panel lean. Declared: solid slab with visual baffles. Steel on steel 0.30/0.25; pads 0.6/0.5. Station, stated: the filter bank sits at counter height and leans back 30 degrees above horizontal so the table-mounted arms reach it. In a real hood the bank is overhead, at 45 degrees or more from horizontal, and the filter faces down, so there gravity swings its top out and the latch is what holds it up; here gravity holds the filter on the panel and the latch is scored as the retention step (pulled the full travel against its spring, then down over the top edge).

Context: Dishwashers (SOC 35-9021, about 0.5 million employed in the US), restaurant cooks (35-2014, about 1.3 million) and janitors and building cleaners (37-2011, about 2 million; BLS OEWS, approximate) take grease filters out of kitchen hoods, wash them and refit them as a routine closing or weekly duty. A refit has two hard parts: the filter is a heavy, awkward slab (a 10 x 16 in stainless baffle filter weighs 2.6 lb, about 1.2 kg, more than one robot arm should hold), and it has to be stood in the bottom channel with its top swung out, then swung in past a spring latch that must be pulled out of the way and let go to drop over the filter's top edge. Most hood manuals describe this sequence (insert in the track, pull the spring release handle, push the filter in, release the handle). The setting is the nightly close of a commercial kitchen: the hood's baffle filters go through the dish machine and every one has to be refitted and latched before the next service, since a missing or loose filter lets grease into the duct, a fire risk.

## Status

Dream-1 score: 0.0 (1600 steps, hit the step limit). Demo: 1.0 at step 795 (39.8 s). Media M-QTEAZN.
