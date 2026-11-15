# Snap a fallen cable clip back onto a robot arm link at its painted band

A robot-maintenance task. A robot arm link, a 25 mm carbon-fibre tube, is held in a repair jig on the table with a 30
mm band painted on it. The nylon cable clip that belongs at the band has fallen off and stands on the table. The robot
has to pick the clip up and press it straight down onto the tube at the band: the tube spreads the clip's two spring
jaws, and they snap shut behind it. The clip is left on the link. The instruction says only that the clip goes back on
at the painted band; how to hold it and how to push is the robot's job.

The skill is a snap fit with a part held in the gripper: carry a small clip over a tube, get it square and over the
band, and press it through the spring jaws without letting it slip in the grip or tilt (a tilted clip jams on the
tube).

| File | What it is |
|---|---|
| `arm_link_cable_clip.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/cable_clip.usda` | The clip (OpenUSD): an articulation of a ring with its loop and two spring jaws |
| `assets/link_jig.usda` | The link in its jig (OpenUSD): tube, two housings, two saddles, the painted band; fixed |
| `tools/build_assets.py` | Writes both USD files |
| `test_arm_link_cable_clip.py` | Offline checks: every scoring outcome, the 1 s timing, the arm's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the part class (a 25 mm CFRP tube is a common arm-link size; nylon cable
clips for tubes come in 16-32 mm bores), not one catalogue entry.

- **Link jig** (fixed): a carbon-fibre tube 25 mm across and 300 mm long, axis along y, 60 mm above the table; at each
  end an aluminium joint housing 52 mm across and 40 mm long in a saddle. A 30 mm band is painted yellow on the tube
  (visual only), centred on the middle. Friction 0.3 / 0.25.
- **Clip** (free articulation, 40 g: ring 24 g, each jaw 8 g, orange nylon, friction 0.3 / 0.25):
  - the ring is 24 mm wide and 38 mm across (bore 27 mm, the tube leaves 1 mm all round), 5.5 mm wall, with a cable-tie
    loop on top (14 mm x 17 mm, slot drawn only), 47 mm tall from the jaw lips to the top of the loop;
  - two jaws hang from the ring at 25 degrees above its equator and curve down to lips 16 mm apart, 9 mm narrower than
    the tube; each is a hinge with a 0.1 N m/rad spring (limit 25 degrees open, 0 at rest) and light damping;
    resting, the lips sit 0.8 mm inside the tube's surface; a jaw held open by the tube turns about 2.2 degrees, so the
    clip grips the tube with about 0.3 N a jaw;
  - the ring and jaws are made of box colliders (tangent boxes round the bore), so the contact is exact in both
    simulators.
  - It stands on the table on the two jaw lips, opening down, loop up.

The press needs about 1-1.5 N straight down (the jaws open about 14 degrees at the tube's widest part), well inside
the robot's pressing force; a tilted or dragged clip needs several times more because it jams.

## Layout

The tube's axis runs along y through x 0.30, y -0.08 at z 0.060 above the table; the clip stands at x 0.20, y -0.20.
Only the right arm is used.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arm_link_cable_clip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/clip-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=arm_link_cable_clip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/clip-demo
```

The log prints `[cable_clip] t=...  steps done: ...` on every change and a trace each second (the clip's offset from
the link's axis, its tilt and the two jaw angles).

**The demo.** Open loop: fixed waypoints, no simulator reads. The right arm points its gripper straight down onto the
clip's loop, pads closing across the clip's 24 mm width (along the link) with the pad tips 20 mm above the bore's
centre, so the fingers stay above the tube; closes (about 20 N a finger), lifts 50 mm, carries the clip over the
band, lowers it onto the tube in 3 s until the ring top rests on it (commanded 1 mm lower), holds, opens, lifts away
and goes home. Success at about step 275 (13.75 s).

## Scoring

Steps 2 to 4 count only after a gripper has held the clip off the table.

| Step | Done when |
|---|---|
| 1 | a gripper presses the clip with both fingers (at least 0.5 N, pads at least 12 mm apart, pads inside the clip's outline) and its bore centre is at least 1 cm above its starting height (counts once reached) |
| 2 | the link's axis within 2 mm of the bore's centre, the clip's axis within 8 degrees of the link's and its middle within 6 mm of the band's middle |
| 3 | step 2 and each jaw within 6 degrees of rest |
| 4 | step 3 and no gripper pad within 3 cm of the clip |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success(): all four steps, the clip within 2 mm and 2 deg of where it was when that second began (each jaw within 2
  deg). A lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** only if the clip leaves the table area (falls off).
- Doing nothing scores 0 (run). Shortcuts: the clip dropped or pushed onto the link without being held scores 0 (steps
  2 to 4 need the earlier hold; test); pads that touch but do not press, or closed fingers, do not count as a hold
  (tests); a clip lying on top of the tube, half-pressed, off the band, crooked or beside the link does not complete
  step 2, and a jaw still spread does not complete step 3 (tests).
- **Real station:** judged from video: the clip seated on the tube at the painted band, its jaws closed round the tube,
  the gripper clear. A light straight-up tug with a spring scale (about 0.7 N: more than the clip's 0.4 N weight, less than the
  1-1.5 N it takes to press it through the jaws) tells a snapped clip from one resting on top.

## Reset

Pull the clip off the link and stand it on the table at its mark on the lips of its jaws, loop up, width along y.

## Status

Dream-1 score: 0.0 (arms moved to the clip but never lifted it onto the link). Demo: 1.0 (success at step 275, Isaac).
