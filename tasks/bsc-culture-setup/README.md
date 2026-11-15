# Lay out a biosafety cabinet's work surface for a cell-culture session

A biosafety-cabinet work zone (slotted front and rear grilles, back wall, raised glass sash) holds three items in a jumble on the front
grille: a 250 mL media bottle, a pipette tip box and a waste pan. The robot lays them out the way cabinet practice requires: well inside the front grille,
rear grille kept clear, clean items on one side and the waste on the other, 3 cm between items, then lets go of everything. The instruction does not
spell out any of these rules.

| File | What it is |
|---|---|
| `bsc_culture_setup.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`), scripted `demo()` |
| `assets/cabinet.usda` | The cabinet zone (fixed): mat, grilles, sash and hood (visual), back wall (collider) |
| `assets/media_bottle.usda`, `tip_box.usda`, `waste_pan.usda` | The items (rigid bodies) |
| `test_bsc_culture_setup.py` | Offline checks: loads, every scoring outcome, the 1 s timing, reach |
| `task.json` | The task's details for the board |

## Assets

Created for this task by its authors, licensed CC-BY 4.0. Metres and kilograms, Z up. Typical lab-consumable sizes (not catalogue parts):
media bottle 70 mm across, 140 mm tall with a 42 mm cap, 0.30 kg full; tip box 127 x 83 x 55 mm with a clear lid, 0.12 kg; waste pan 80 x 130 x 60 mm with 4 mm walls, 0.25 kg
(the 80 mm width fits the gripper's 94 mm opening). One box or cylinder collider per body (the pan: five boxes), friction 0.6/0.5.
Cabinet: 0.30 x 0.60 m mat, 100 mm front grille and 50 mm rear grille strips with slots (visual), glass back wall at x 0.55 (collider). The sash is drawn raised to a 380 mm
opening and has no collider: the arms' ready pose passes through any lower sash, so the sash cannot be modelled as an obstacle (stated in the fidelity note).
The usable depth is 15 cm (x 0.35 to 0.50) because of the robot's reach, much less than a real cabinet's.

## Run it

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=bsc_culture_setup.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/bsc-demo
```

**The demo.** Open loop. Left arm: the media bottle (fingers across 70 mm), 3 cm lift, back to x 0.425, y 0.06; then the tip box (fingers across 83 mm) to y 0.22. Right arm, at the same time:
the waste pan (fingers across 80 mm) to y -0.12. Each lets go, lifts, backs out toward the front. Local test: 1.0 at step 564, also with every item's start moved 3 mm and turned 3 degrees in three directions.

## Scoring

Four steps, 0.2 each: bottle, tip box, waste pan each upright with the whole footprint at x 0.35 to 0.50 on its side of the middle (clean y > 0, waste y < 0); then all three, at least 3 cm apart, with no gripper within 3 cm.
1.0 only after every condition has held for 1 s, timed in the task file (`SUCCESS_HOLD_S = 0`), items within 2 mm / 2 degrees; step points stop at 0.8. 0 if an item tips over 45 degrees or leaves the table.
