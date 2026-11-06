# Load a filament spool into a dryer and latch the lid

A bimanual loading task from a 3D-printing workshop. A single-spool filament dryer stands on the table with its smoked
clear hood (the lid) fully open; next to it, a 1 kg spool of orange PLA stands on its rims in a printed stand.
- One hand lifts the spool off the stand, carries it over the dryer's front wall and sets it down on the dryer's two
  rollers, then lets go.
- The other hand swings the open lid over and presses it shut until its catch clicks. Both arms go back.

The spool weighs 1.18 kg and is held only by its two cardboard flanges; it must come down centred on both rollers,
its axis along them, with 16.5 mm to spare on each side. The lid is hinged at the back and rests open on its stop; it
stays shut only once its hook has ridden over the catch's sprung tongue.

| File | What it is |
|---|---|
| `filament_dryer.py` | The `yam_arena` task file: scene, the open-lid start, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/filament_dryer.usda` | The filament dryer (OpenUSD), one articulation: base, hood on a friction hinge, two free-spinning rollers, the catch's sprung slider |
| `assets/filament_spool_1kg.usda` | The 1 kg spool of 1.75 mm PLA on a cardboard spool, one rigid body |
| `assets/spool_stand.usda` | The printed spool stand, one rigid body |
| `assets/textures/` | The dryer's display, buttons and rating plate, the spool's label, kraft cardboard (no brand) |
| `test_filament_dryer.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Dryer, spool and stand** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up. Generic designs, no
brand or logo.

- **Filament dryer:** 262 x 130 x 240 mm, 1.2 kg, sized like common single-spool dryers (one popular model lists
  265 x 118 x 274 mm and about 1.2 kg, for spools up to 210 x 85 mm).
  - The base is a graphite ABS shell on rubber feet, 160 mm tall, with a 230 x 100 mm cavity, a control panel
    (display and three buttons) on the end facing the robot and a rating plate on the other end.
  - Two white POM rollers (22 mm, 130 mm apart) spin freely on axles along the cavity; a 200 mm spool rests on both
    with its centre 120 mm up.
  - The hood is smoked clear polycarbonate, 80 mm deep, 0.25 kg, hinged along the base's back edge on a friction
    hinge (damping only). It rests fully open (100°) on its stop by its own weight and, pushed past about 50° from
    shut, falls shut by itself in about a second. A push-fit filament outlet sits on its end.
  - Why the base is that tall: a deep hood hinged at the back sweeps the space just behind its front skirt as it
    swings. With the rim at 160 mm the spool's top stays clear of that sweep; with a 121 mm rim and a 119 mm hood the
    hood jammed on the spool's flange at 60°, and the arm pushing on it shoved the whole dryer.
  - The catch, near the end facing the robot: a tab with a hook hangs from the hood's front edge into a slot in the
    base's front wall. A slider in the wall, on a 200 N/m return spring (0.2 N preload), holds its tongue 1.1 mm
    over the hook; the hook's 45° chamfer pushes it back as the hood is pressed shut and the tongue snaps over the
    hook within 0.3° of shut.
- **Spool:** 200 mm across, 67 mm wide, 53 mm hub hole, kraft-cardboard flanges and core, 1.0 kg of orange 1.75 mm
  PLA wound to 174 mm, its end clipped through a hole in the flange; 1.18 kg in all. It collides as three 32-sided
  prisms (the two flanges and the winding).
- **Stand:** a printed PLA cradle, a 150 x 90 mm plate with two round-topped rails 110 mm apart; the spool stands on
  its rims, axis level, its centre 113 mm above the table. 80 g, loose on the table.

All colliders are exact boxes and capsules or small convex prisms (at most 64 points), never hulls of detailed meshes.

**The open lid at reset.** Isaac Lab starts an articulation's joints at 0 (lid shut) and sets every drive's target to
0. A reset event in the task file opens the lid onto its stop, puts the catch slider home and sets its spring's target
back to the USD's (-1 mm: the preload).

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=filament_dryer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dryer-dream

# Scripted demo (both arms), 700 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=filament_dryer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/dryer-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With `-E object_jitter=0`
  the layout is fixed and no seed is needed. The job log prints a `[filament_dryer] t=...  steps done: ...` line
  whenever a step is done or undone, and the state every second: how far the spool is from its seat on the rollers,
  how far its axis is turned, its speed and height, the lid's angle, the catch slider's travel and how far the
  dryer and the stand have moved.
- **The demo.** It reaches 1.0 at step 639 (32.0 s), the same in three runs in a row: spool taken off the stand
  at 6.9 s, on the rollers at 14.8 s, lid swung over at 23.1 s, shut and caught at 25.6 s, both arms home by
  31.5 s. The spool ends 0.3 mm from its seat along the dryer, 2.9 mm off the middle across it, its axis within
  0.1°; the dryer doesn't move. A trial takes about 5 minutes on an L40S, including startup.
  - The right arm points forward and 38° down, its fingers closing across the spool's width on the outsides of the
    two flanges at its top, the contact patch right over the spool's axis so it hangs straight. Pointing down that
    far keeps the gripper's housing (54 mm and more behind the fingertips' middle, as wide as the spool) 15 mm clear
    of the spool's rim; pointing level, it would hit it. The arm takes the spool's weight, lifts it 16 cm, carries it
    sideways over the dryer's front wall (1.5 cm clear) and lowers it onto the rollers, lets the rollers take the
    weight, opens the fingers only to 76 mm (the open lid is beside them), lifts out and goes home.
  - The arms hold position through soft joint drives (80 N m/rad at the shoulder, 10 at the wrist, as on the real
    YAM) with gravity compensation for their own links only, so the 1.18 kg spool would pull the right gripper about
    6 cm down. While the spool is in the hand, the demo raises the right arm's joint targets by the sag its weight
    causes (each joint's torque from the spool's weight over its drive's stiffness), faded in as it takes the weight
    and out once the rollers have it (`payload_sag`).
  - The left arm points forward, level, fingers closed, at the lid's near end, over the catch (the gripper's
    housing stays off the lid). The side of its fingers follows the open lid's top face near its free edge and pushes
    it over, the gripper rolling with the face; past about 50° the lid falls shut on its own and the fingers follow it
    down, press it 8 mm (about 2 N through the soft wrist) so the catch snaps over the hook, lift away and go home.
- **Videos:** `inspect-robots video ~/yam-eval/dryer-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_filament_dryer.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order: the right lifts the spool off its stand, sets it on the dryer's rollers
and lets go; then the left swings the lid over and presses it shut until it clicks; both arms go back. It leaves out
where things are, which the model finds from the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Spool taken off the stand | a gripper holds the spool lifted clear of the stand (its lowest point 1 cm above the rails) | once reached |
| 2. Spool on the rollers | the spool rests on both rollers: its centre within 6 mm of its seat along the dryer and 4 mm in height, within 12 mm of the middle across it, its axis along the dryer's within 8°, and no gripper holds it | at the end |
| 3. Lid swung over | with step 2 done, a gripper has pushed the lid 40° toward shut (open less than 60°) | once reached |
| 4. Lid shut and caught | with step 2 done, the lid is shut (within 2°) and the catch holds it (its slider back home) | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  the dryer within 3 cm and 10° of where it started, and the dryer and the spool still (under 1 cm/s and 0.5 rad/s)
  for 1 s.
- **0** at the end, whatever the steps, if the dryer or the spool has fallen off the table (more than 2 cm below its
  top), or the state is not finite.
- **An empty dryer doesn't count:** closing the lid without the spool on the rollers earns nothing (steps 3 and 4 need
  step 2).
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
  "A gripper pushes the lid": a gripper's pads within 2 cm of the lid's outer top face, over the lid.
- **On a real station** the same steps are judged from the video: the spool sitting level in the middle of the
  rollers, the lid down flat and not lifting when the dryer is tipped toward the camera, a ruler for the dryer's
  position.

## Reset

- Open the lid fully, onto its stop.
- Take the spool out of the dryer and stand it back in its stand, axis across the table.
- Put the dryer and the stand back on their marks.
