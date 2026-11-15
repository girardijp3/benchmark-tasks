# Replace the cracked cover on a robot's electronics bay: slide it off its posts into the scrap tray, then lock the new one on

A robot-maintenance task. The top cover of a small mobile robot's electronics bay is cracked. It hangs on three headed
posts through keyholes: it has to be pushed 10 mm along its slots until the posts stand in the keyholes' big holes, and
then lifted off. The robot puts the cracked cover in the scrap tray, takes the new cover from the parts rack, lowers it
onto the posts so the big holes go over the heads, and slides it back 10 mm so the posts reach the ends of the slots,
where the heads trap the plate. The instruction names the result (new cover fitted so that it is locked on, cracked
one in the tray) and nothing about how.

The skill is a two-stage swap with a slide lock: the cover is removed and replaced through the same keyholes, so the
sequence and the direction of the slide have to be right, and the new cover must be aligned over three posts (a
cover set down on the heads, or left at the unlock position, does not count).

| File | What it is |
|---|---|
| `robot_cover_swap.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/cover_cracked.usda`, `assets/cover_new.usda` | The two covers (OpenUSD, box colliders, one rigid body each) |
| `assets/bay_deck.usda` | The deck with its three posts on the robot's body (fixed) |
| `assets/parts_rack.usda`, `assets/scrap_tray.usda` | The rack the new cover lies on, and the scrap tray on its riser (fixed) |
| `tools/build_assets.py` | Writes all the USD files |
| `test_robot_cover_swap.py` | Offline checks: every scoring outcome, the 1 s timing, the arm's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values for a small mobile robot's service cover, not one catalogue entry.

- **Covers** (free, ABS, 100 g each, friction 0.45 / 0.35): 150 x 100 mm, a 3 mm plate on a 20 mm skirt of 3 mm walls,
  a 40 x 12 x 14 mm finger rib on top of the plate; the plate has three keyholes, each a 22 x 18 mm hole for a post's
  head and an 8 mm wide slot running 13 mm from the hole's middle toward the robot; the cracked cover (grey) has crack
  lines and the new cover (white) a green label, drawn as thin visual geometry. The plate is cut into 16 boxes around
  the keyholes so the holes and slots are exact.
- **Deck** (fixed): a 190 x 140 mm, 12 mm thick plate on a body block, its top face 60 mm above the table. Three posts
  (5 mm shaft, 12 mm head, 2.5 mm thick) at (-50, -30), (-50, +30), (+50, 0) mm from the deck's middle; the head sits
  1 mm above the plate. 0.5 mm of clearance sits between a shaft and the end of its slot, 1.5 mm each side in the
  8 mm slot.
- **Rack** (fixed): 170 x 120 x 30 mm. **Scrap tray** (fixed): 174 x 124 mm inside (12 mm of margin round a cover), 35
  mm walls, a 3 mm floor, on a 30 mm riser.

How the lock works: with the cover's middle at the deck's middle (locked), each post stands at the end of its slot and
its 12 mm head overlaps the plate by 2 mm on each side of the 8 mm slot (and by 5 mm beyond the hole's edge), so the cover cannot be lifted. Sliding the cover 10 mm
away from the robot puts the posts in the holes' middles (holes 22 x 18 mm: a head fits through when the cover is
within 5 mm in x and 3 mm in y of that unlocked pose); the cover then lifts straight off.

## Layout

Deck at x 0.34, y -0.10 (top face 60 mm up); rack at x 0.20, y -0.30 (30 mm tall); tray at x 0.22, y +0.07 (floor 33 mm
up). Only the right arm is used.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=robot_cover_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/cover-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=robot_cover_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/cover-demo
```

The log prints `[cover_swap] t=...  steps done: ...` on every change and a trace each second (each cover's offset
from the deck or tray, and whether a gripper holds it).

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm points its gripper straight down onto a
cover's rib (pad tips 2 mm above the plate, fingers closing across the 12 mm rib along y, about 20 N a finger). The
cracked cover: lifted 0.7 mm, pushed 10 mm away from the robot, lifted 30 mm and 50 mm, carried over the tray, lowered
into it, let go. The new cover: lifted off the rack, carried over the posts (aimed 2.5 mm past the holes' middles
because the arm sags that much short of its target there), lowered until the skirt rests on the deck, eased up 0.7
mm, pulled 12 mm toward the robot (the posts stop it at the slots' ends after 10), let go, lifted away; home. Success at
about step 562 (28 s).

## Scoring

The cracked cover counts as off the bay only after a gripper has held it, the new cover as locked only after a gripper
has held it.

| Step | Done when |
|---|---|
| 1 | the cracked cover's middle is at least 0.17 m from the deck's middle |
| 2 | step 1 and the cracked cover lies in the tray (middle within 11 mm of the tray's, lower edge 2 mm below to 12 mm above the floor, within 6 deg of flat) |
| 3 | step 2 and the new cover is locked (middle within 2 mm in x and 2.5 mm in y of the deck's, skirt on the deck's top face within -1 to +1.5 mm, within 4 deg of square, 2.5 deg of flat) |
| 4 | step 3 and no gripper pad within 3 cm of either cover |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success(): all four steps and both covers within 2 mm and 2 deg of where they were when that second began. A lapse
  restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** only if a cover falls off the table.
- Doing nothing scores 0 (run). Shortcuts (tests): a cover pushed or dropped without being held scores nothing; the new
  cover left at the unlock position, resting on the post heads, crooked, 3 mm off in y, or never held does not
  complete step 3; the cracked cover on the tray wall, on the table or on the rack does not complete step 2; closed
  fingers or a gripper beside a cover do not hold it. The new cover cannot lock while the cracked one is on the posts
  (the posts and plate collide).
- **Real station:** judged from video: the new cover seated on its posts with the heads over the plate, the cover not
  liftable by hand (a 5 N lift test), the cracked cover in the tray.

## Reset

Lower the cracked cover onto the posts with its holes over the heads and slide it 10 mm toward the robot until the
posts reach the slot ends; lay the new cover on the rack on its mark; empty the tray.

## Status

Dream-1 score: 0.0. Demo: 1.0 (success at step 562, Isaac). Media M-KJ6ST5.
