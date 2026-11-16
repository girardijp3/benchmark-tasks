# Pull the middle blade fuse out of a fuse block with a tweezer puller held crosswise to it, and put it in a tray

An automotive service technician's task (MAXI blade fuses are automotive; O*NET 49-3023, about 0.8 million US jobs). A fuse block lies on the table with three blade fuses standing in it, and a plastic tweezer-type fuse puller lies in front of it. The robot has to take the middle fuse out with the puller and put it in a tray. The instruction states only the goal; that the puller has to be taken across the fuse, its jaws squeezed shut on the fuse's two faces and the fuse drawn straight up against the clips, are the robot's to work out.

The skill is a tool used on a part: picking up a spring-opened tool by its two arms with the grip squeezing it (squeezing is what closes the jaws, so the grip has to be light while carrying and firm to clamp), sliding its jaws between the neighbouring fuses (14 mm open against an 8.5 mm fuse, a flared lead-in), clamping and pulling a fuse out against about 4 N of clip friction (real est. 10-30 N), then releasing it in a tray. The puller is carried roughly horizontally, perpendicular to the fuse's vertical axis, its jaws tipped down toward the fuse by about 5 degrees, and it must be set down and let go afterwards.

| File | What it is |
|---|---|
| `fuse_puller_crosswise.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/fuse_block.usda` | The fuse block with three pockets, each with two spring clips on prismatic joints, and the tray (OpenUSD); one articulation with a fixed root |
| `assets/fuse.usda`, `fuse_50.usda`, `fuse_20.usda` | The 30 A middle fuse and the two neighbouring fuses (50 A, 20 A) (OpenUSD); each one free rigid body |
| `assets/puller.usda` | The tweezer puller: head and two arms on revolute joints with springs (OpenUSD); one articulation with a free root |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; boxes, capsules, spheres) |
| `test_fuse_puller_crosswise.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Real values** (the sim uses 4 N; the real figures below are estimates, not catalogue data): a MAXI fuse seated in a block's terminals takes roughly 10 to 30 N to pull (estimated from the blade-to-terminal grip; none was measured here), so the sim's 4 N is a deliberately light, spring-clip-only retention that a 30 g puller's 20 N jaw grip can overcome; the puller spring is the sim's 3 N m/rad (about 6 N at the pads) against a few newtons at the tips of real plastic pullers (estimate).
- **Looks** (visual meshes only, the colliders are the simple boxes below, invisible): ATO/MAXI blade fuses in the ISO colour code (20 A yellow, 30 A green, 50 A red) with chamfered housings, a window with the metal link and the amp rating printed on both faces (`assets/textures/fuse_*.png`); a dark ABS fuse block with chamfered corners, mounting ears and screws and bronze clip teeth; a red moulded-plastic puller with grip ribs, a pivot rivet and steel jaw tips; a pressed stainless tray.
- **Fuse block** (one articulation, fixed root): a 52 x 84 x 20 mm base, three 30.6 x 9.5 x 12 mm pockets 28 mm apart, each with two spring clips (teeth 8 x 3 x 8 mm on prismatic joints `ClipAP` ... `ClipCM`, stiffness 2,700 N/m, 1.5 mm of interference, about 4 N a side), and a tray (54 mm square inside, 3 mm floor, 10 mm rim).
- **Fuse** (free body, 8 g): a 29.2 x 8.5 x 34 mm block; its lower 12 mm sits in the pocket.
- **Puller** (one articulation, free root, 30 g): a head block and two arms (3 x 12 x 100 mm) hinged about Z 21 mm apart (joints `ArmP`, `ArmM`, spring 3 N m/rad preloaded against the open stops, limits 5 degrees), 28 mm long jaw plates standing 2 mm inside the arms (14 mm apart open) with a flared tip.

**Building it for a real trial.** A MAXI blade fuse (29.2 x 8.5 mm), a block or a printed block with a 30.6 x 9.5 mm pocket 12 mm deep and two small leaf-spring clips; a plastic tweezer-type fuse puller about 100 mm long whose arms spring open with about 6 N at the grip point (3 N m/rad at the hinge, the sim's value; a printed puller with a soft 0.5 N m/rad spring let the puller slip in the light grip in simulation, so it would need a stiffer one or a firmer grip); a small tray. The outer two fuses stay in their own clips.

## Layout

The block's middle is at x 0.38, y -0.12; the tray at x 0.33, y -0.01; the puller's hinge line at x 0.16, y -0.12 (where the demo also lays it back down). The right arm does the whole demo.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fuse_puller_crosswise.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/fuse-puller-crosswise-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fuse_puller_crosswise.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 720 --store-frames --log-dir ~/yam-eval/fuse-puller-crosswise-demo
```

The log prints `[fusepull] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm, pointing down, takes the puller by the front part of its two arms with a light grip (the pads 24 mm apart on the arms' outer faces, covering the arms from 41 to 79 mm from the hinge line, beside the jaw plates: the YAM gripper's fingers are about 100 mm long and the puller's head is wider than the arms, so it cannot take them further back; the arms' spring presses them), lifts it, tips the jaws down 5 degrees toward the fuse (10 degrees made the arms close unevenly on the station and the clamp test lapsed), carries it forward until the jaws straddle the fuse (coming in 14 mm high to clear the block's edge, then dropping either side of the fuse), squeezes the arms shut (the jaws clamp the fuse, about 20 N at the pads), lifts the fuse 40 mm straight up out of the clips, carries it over the tray, lowers it and lets the arms open so the fuse drops in. It then lifts the open puller straight up off the fuse, carries it back beside the block, lays it flat on the table where it started, lets go and goes home, leaving the fuse lying free in the tray. Success at step 642 (32.1 s).

## Scoring

| Step | Done when |
|---|---|
| 1 | the puller's jaws were closed on the fuse (arms 1.2 to 3.2 degrees, the fuse between the jaw plates) |
| 2 | the fuse is out of its pocket and was taken out by the puller (not lifted or moved 3 mm before the jaws had closed on it, and the jaws closed on it when it first left the pocket) |
| 3 | with step 2, the fuse rests in the tray |
| 4 | with step 3, the jaws are open, the puller lies on the table (within 20 degrees of flat) and no gripper holds it, and neither a gripper pad nor any part of the puller (head, arms, jaw plates) is within 3 cm of the fuse |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success():
  all steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the fuse or the puller is lost off the table, or either neighbouring fuse, free in its own clips, is moved more than 3 mm from its pocket); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). A puller left around the fuse in the tray, still held, in the air or within 3 cm of the fuse does not finish step 4. Shortcuts: a fuse knocked or pinched out by the gripper without the jaws on it limits the score for the whole run (steps 2 to 4 never count, even if the puller grips it later); jaws closed on nothing or beside the fuse are not clamped (all tests).
- **Real station:** judged from video: the puller's jaws on the fuse's faces, the fuse leaving the block and landing in the tray, the other two fuses untouched in their clips.

## Reset

Stand the fuse in the block's middle pocket, lay the puller on the table at x 0.16, y -0.12 with its jaws toward +x (yaw 0), jaws open; nothing else is moved.

## Fidelity notes

Full text of the task.json fields, shortened there to meet the length limits:

Fidelity: Rigid plastic and metal boxes and prisms at true size (MAXI blade fuse, 29.2 x 8.5 mm); friction 0.50/0.45 (fuse), 0.90/0.80 (puller jaws), 0.50/0.45 (clips). All three fuses are free bodies held by their own pair of spring teeth in the block (4.05 N each, friction retention about 4 N); the fuses' blades and the block's terminals are not modelled (the lower 12 mm of each fuse sits in its pocket). Real values (estimates, not catalogue data): a MAXI fuse seated in terminals takes roughly 10 to 30 N to pull, so the 4 N clip retention is deliberately light (spring clips only, no terminals); a real plastic puller's arms spring open with a few newtons at the tips, the sim's 3 N m/rad is about 6 N at the pads (a printed 0.5 N m/rad spring slipped in the light grip in simulation).

## Status

Dream-1 score: 0.0 (2400 steps, hit the step limit). Demo: 1.0 at step 642 (32.1 s). Media M-3DSA9N.
