# Open the cabinet, take the multimeter out and then close it

A bimanual fetching task. A small birch-plywood cabinet stands on the table, its door shut, with a handheld
multimeter standing inside.
- One hand pulls the door open by its bar handle, following the door's arc, and holds it open.
- The other hand reaches in, takes the multimeter out and stands it on the bench beside the cabinet, then goes back.
- The first hand pushes the door shut until the catch holds it, lets go and goes back.

The door swings on a hinge, so the hand on the handle has to turn with it along an arc instead of pulling straight.
The catch takes a firm pull to open and a firm push to close. The multimeter is 49 mm thick, which leaves 22 mm a
side in the gripper's 94 mm opening, and it stands 30 mm inside the door frame under a shelf.

| File | What it is |
|---|---|
| `cabinet_item.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/bench_cabinet.usda` | The cabinet (OpenUSD), one articulation: carcass, door on its hinge, catch |
| `assets/multimeter.usda` | The multimeter, one rigid body |
| `assets/textures/` | Birch grain and ply-edge maps, the door's label, the meter's LCD and front panel (no brand) |
| `test_cabinet_item.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Cabinet and multimeter** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Cabinet:**
  - a bench-top storage cabinet, 220 x 250 x 350 mm on four 6 mm rubber feet, in 12 mm birch plywood (680 kg/m³)
    with a 9 mm hardboard back, 3.13 kg without its door;
  - inside, 196 mm wide and 241 mm deep, with a shelf whose underside is 222 mm above the floor;
  - an overlay door, 219 x 346 x 15 mm (0.95 kg with its hardware). It hangs on two barrel hinges at its left edge
    and swings in front of the carcass up to 120°. The hinge is damped (0.5 N·m·s/rad, no spring), so the door stays
    where it is left;
  - a steel bar pull, 12 mm thick, 168 mm long, on 128 mm hole centres with 35 mm projection, vertical, near the
    door's right edge at mid-height;
  - a catch under the top panel and a strike on the door. PhysX has no magnetism, so the catch holds by contact:
    a 6 mm rod on the strike rides under the catch body's rounded 6 mm lip (a compliant contact, 3500 N/m).
    - Shut, the lip presses the door against the hinge's stop.
    - It takes about 6 N at the handle to pull the door past the lip (measured in the simulator). Typical
      small magnetic cabinet catches hold 20–40 N; this one is lighter so a two-finger grip on the bar opens it.
    - Past 1.1° the lip pushes the door further open, so the door rests either caught (0°) or at 2.5° or more,
      never half-caught.
- **Multimeter:** a generic handheld 6000-count digital multimeter in a yellow rubber boot, 186 x 89 x 49 mm,
  400 g: the size and mass of a common bench meter of this class (UNI-T UT61E+: 186 x 89 x 49 mm, 400 g). It has
  an LCD, a rotary dial, four buttons, four input jacks and a tilt stand.
  - It collides as its own chamfered box (24 points), so it stands still on its end.
  - Rubber friction 0.9 / 0.8, and a torsional-friction patch (8 mm) for the pads' grip on the boot.
- **Colliders:** exact boxes for every panel, foot and handle post, and capsules for the bar and the two catch rods.
- **Articulation details:**
  - PhysX never collides two links joined by a joint, so the door can't touch the carcass. Its stop is the hinge's
    lower limit.
  - The catch is its own link, fixed to the carcass, so that the door's strike can touch it. Self-collisions are on.
  - A positive hinge angle opens the door.

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cabinet_item.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cabinet-dream

# Scripted demo (both arms), 760 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cabinet_item.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 760 --store-frames --log-dir ~/yam-eval/cabinet-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With `-E object_jitter=0`
  the layout is fixed and no seed is needed. The job log prints a `[cabinet_item] t=...  steps done: ...` line
  whenever a step is done or undone, with the door's angle, where the multimeter is and how far the cabinet has moved.
- **The demo.** It reaches 1.0 at step 631 (31.6 s), the same in three runs in a row: door opened by its handle at
  8.3 s, multimeter taken out at 16.1 s, on the bench at 21.0 s, door shut and caught at 27.5 s, both arms home by
  30.5 s. A trial takes about 4 minutes on an L40S, including startup.
  - The left arm grips the bar from the front, pointing level at the door 10° off square, its fingers closing
    around the bar. It pulls the door open 80° along its arc: the gripper moves on a circle about the hinge and
    turns with the door (`Waypoint.pivot`). The first 2.5° take the door past the catch.
  - The right arm waits at home until the door is open. It reaches in level, fingers closing across the
    multimeter's 49 mm thickness at 150 mm up (about 30 N a finger), and lifts it about 1 cm. It draws the meter
    straight out, carries it right, past the cabinet's front corner, and stands it on the bench beside the cabinet.
    It lets go, rises off the meter and goes home.
    - The arm sags about 20 mm under the 400 g meter (its wrist motors are soft), so its lifts are commanded
      higher than needed. The meter tilts up to 7° in the grip and straightens as it is set down.
  - The left arm pushes the door back along the same arc and 0.5° past shut: the strike rides under the catch's
    lip and snaps in behind it. The left arm lets go, backs off and goes home.
- **Videos:** `inspect-robots video ~/yam-eval/cabinet-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_cabinet_item.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and in what order:
- the left opens the door by its handle;
- the right takes the multimeter out, sets it on the bench to the right of the cabinet and goes back;
- the left shuts the door until the catch holds, lets go and goes back.

It leaves out where things are, which the model finds from the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Door opened by its handle | the door is open at least 45° while a gripper holds its bar handle | once reached |
| 2. Multimeter taken out | a gripper holds the multimeter and it is wholly outside the cabinet's carcass | once reached |
| 3. Multimeter on the bench | the multimeter is outside the cabinet, resting on the table (its lowest point within 4 mm of it), and no gripper holds it | at the end |
| 4. Door shut and caught | with step 2 done, the door is shut past the catch (under 0.8°), no gripper holds the handle, and the cabinet stands where it stood (within 2 cm and 5°) | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and the cabinet and the multimeter still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps:
  - the cabinet or the multimeter has fallen off the table (its origin more than 2 cm below the table top);
  - the cabinet is tipped more than 20°;
  - the state is not finite.
- **Guards:**
  - a door swung open without a gripper on its handle never earns step 1;
  - a multimeter knocked out of the cabinet without a gripper holding it never earns step 2;
  - a door shut before the multimeter was taken out earns nothing;
  - a door held shut by hand isn't caught.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box
  (5 mm slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video, with a protractor for the door's angle and a
  ruler for the cabinet's position.

## Reset

- Stand the multimeter back on its end inside the cabinet at its mark, 30 mm in from the front, display toward the
  hinge side.
- Shut the door until the catch holds it.
- Put the cabinet back on its mark.
