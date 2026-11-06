# Put a screwdriver in its foam-lined case and close the lid until the clasp catches

A bimanual tool-kitting task. A hard plastic screwdriver case lies on the table, its lid standing open; its foam
insert has a cutout shaped like the screwdriver. The screwdriver lies on the bench beside it.
- One hand picks the screwdriver up by its handle, turns it to lie along the case and lays it in the cutout, handle
  in the handle pocket.
- The other pushes the lid over and presses it shut until the clasp catches. Both arms go back.

The screwdriver's round handle has nothing flat for a gripper to grip but its two grip panels, and lying on the bench
it is pitched 6° (it rests on its handle and the end of its shaft), so it has to be levelled on the way. The cutout
leaves 1.5 mm all round, the fingers holding it fit only in the finger notch across the handle pocket, and the lid
stays shut only once its hook has ridden over the clasp's ramp.

| File | What it is |
|---|---|
| `screwdriver_case.py` | The `yam_arena` task file: scene, the open-lid start, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/screwdriver_case.usda` | The case (OpenUSD), one articulation: base with its foam insert, lid on a damped hinge, clasp slider |
| `assets/screwdriver_task.usda` | The SimReady screwdriver referenced unchanged, with this task's colliders (copied next to the download at spawn) |
| `assets/textures/` | Plastic grain maps and the lid's label (no brand) |
| `test_screwdriver_case.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Case** (`assets/screwdriver_case.usda`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Shell:** 216 x 80 x 47 mm, 2.5 mm polypropylene walls (blue, matte grain), four rubber feet; base 31 mm tall to
  the parting line, lid 16 mm. 158 g in all (base 98 g with its foam, lid 58 g, clasp slider 1.8 g), from the meshes'
  volumes at PP 0.905 g/cm³ and EVA foam 50 kg/m³. Sizes are those of common single-tool hard cases for 200 mm hand
  tools (210–230 x 70–90 x 40–55 mm, 2–3 mm PP walls, a rear pin hinge, one front snap latch).
- **Foam insert:** closed-cell EVA, 211 x 75 x 25.5 mm, charcoal, its top 2 mm below the rim. The cutout (light grey,
  "shadow foam") follows the screwdriver: at every point along it, the screwdriver's radius plus 1.5 mm, round below
  its axis and straight-sided above. A finger notch (22 mm along, 66 mm across, 13 mm deep) crosses the handle pocket.
  The insert collides as a signed distance field (1024 cells along its length, about 0.2 mm).
- **Lid:** hinged at the back on a damped (friction) hinge without a spring, 0.03 N·m·s/rad, with a 100° open stop.
  Open, its centre of mass is 18 mm behind the hinge, so it rests on the stop; pushed past about 75° it settles shut
  under its own weight in about 1.5 s.
- **Clasp:** a slider in a housing on the front face (3 mm travel, 200 N/m return spring, 0.2 N preload) and a hook
  hanging from the lid's front edge. The slider's tongue has a 45° ramp on top: pressing the lid shut makes the hook's
  lip push the slider out about 1.7 mm and snap under it. Under its own weight the lid stops on the ramp about 3° open,
  unlatched. Pull the slider out about 2 mm to open the case.

**Screwdriver:** NVIDIA's SimReady `handTool_screwdriver_phillips_ph2_a01` (Hugging Face dataset
`nvidia/simready-assets`, revision `7879e0e5…`, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)): a Phillips
PH2 screwdriver, 188 mm long, handle 34 mm across, 80 g. It is downloaded once at spawn (about 45 MB, SHA-checked)
into `~/.cache/simready` (`SIMREADY_CACHE` overrides; `HF_TOKEN` is used if set), and this task's
`screwdriver_task.usda` is copied next to it.
- The SimReady model collides as one signed-distance-field mesh of its whole visual mesh: its round handle rolls on
  a table and gives flat gripper pads no face to hold square.
- `screwdriver_task.usda` switches that collider off and adds five convex 12-sided sections (a flat face every 30°,
  one face down) that enclose the visual mesh: the domed butt, a straight grip zone 34 mm between its side faces
  (where a gripper holds it), the neck, the collar and the shaft with its tip. SimReady's mass and inertia are kept.

The case and the colliders are built by a headless Blender script (not part of this folder) that reads the SimReady
mesh, fits the colliders and cuts the foam around it.

**The open lid at reset.** Isaac Lab starts an articulation's joints at 0 (lid shut) and sets every drive's target to
0. A reset event in the task file opens the lid onto its stop, puts the clasp slider home and sets its spring's
target back to the USD's (the 0.2 N preload).

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=screwdriver_case.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/case-dream

# Scripted demo (both arms), 760 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=screwdriver_case.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 760 --store-frames --log-dir ~/yam-eval/case-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With `-E object_jitter=0`
  the layout is fixed and no seed is needed. The job log prints a `[screwdriver_case] t=...  steps done: ...` line
  whenever a step is done or undone, and a trace line every second (how far the screwdriver is from its seat, the
  lid's angle, the clasp slider, how far the case has moved).
- **The demo.** It reaches 1.0 at step 667 (33.4 s): screwdriver picked up at 6.0 s, in its cutout at 13.9 s, lid
  closed (settled on the clasp's ramp, 3° open) at 26.9 s, latched at 28.7 s, both arms home by 32.8 s. A trial takes
  about 6 minutes on an L40S, including startup and the first download of the screwdriver.
  - The right arm comes down over the handle, its fingers closing across the grip zone's two flat side faces with
    their tips 8 mm below the handle's axis, and lifts it 9 cm.
  - It turns the screwdriver in place, a quarter turn and 6° up so it lies level along the case (its tip swings past
    the case's right end, clear of the open lid), carries it over the cutout and lowers it straight down, the fingers
    entering the finger notch, and opens them 5 mm a side with the screwdriver 1.5 mm above its seat. It drops in
    (within 1.1 mm of its seat) and the arm goes home.
  - The left arm, pointing 30° forward of straight down (a straight-down wrist can't reach behind the open lid),
    comes in behind the lid; the outer face of its closed front finger meets the lid's top edge and tips the lid
    toward the robot to 55°. It lets go and the lid settles onto the clasp's ramp under its own weight (about 2 s).
  - Pointing straight down, the closed fingertips press the lid's top 7 mm from its front face, over the clasp, 2.5 mm
    below shut: the hook pushes the slider out and it snaps back over the lip. The left arm goes home.
- **Results.** Dream-1: 0.0 (it never picked the screwdriver up). Scripted demo: 1.0 (all four steps, success at
  step 667), three runs out of three.
- **Videos:** `inspect-robots video ~/yam-eval/case-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_screwdriver_case.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order: the right picks the screwdriver up and lays it in its cutout handle
first into the handle pocket, the left closes the lid and presses it until the clasp catches, and both arms go back.
It leaves out where things are, which the model finds from the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Screwdriver picked up | a gripper holds the screwdriver with its lowest point at least 2 cm above the table | once reached |
| 2. Screwdriver in its cutout | the screwdriver's axis, at the handle's middle and on the shaft, is within 3 mm of where it lies seated in the cutout, handle in the handle pocket, and no gripper holds it | at the end |
| 3. Lid closed | with step 2 done, the lid is within 5° of shut | at the end |
| 4. Lid latched | with step 2 done, the lid is shut (within 1°) and the clasp slider is back home (within 0.3 mm): the clasp has caught it | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  the case within 3 cm and 10° of where it started, and the case and the screwdriver still (under 1 cm/s and
  0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps, if the case or the screwdriver has fallen off the table (centre more than
  2 cm below its top), the case has tipped over (more than 20°), or the state is not finite.
- **The right way round:** a screwdriver lying in the case backwards can't sit in the cutout (its handle doesn't fit
  the shaft's channel) and never counts as in it, so the lid over it doesn't count either. Step 2 counts only once a
  gripper has picked the screwdriver up; a lid closed over an empty case earns nothing.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the screwdriver's bounding
  box (5 mm slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`,
  `_held`).
- **On a real station** the same steps are judged from the video: the screwdriver lying down in its cutout (below the
  foam's top face, handle in the handle pocket), the lid down, and the clasp caught (the lid doesn't lift when
  released; the slider is flush), with a ruler for the case's position.

## Reset

- Pull the clasp's slider out about 2 mm and open the lid onto its stop.
- Take the screwdriver out and lay it on its mark on the table, tip pointing away from the robot.
- Put the case back on its mark.
