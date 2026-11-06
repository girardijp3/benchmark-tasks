# Hold an SO-101 arm's 3D-printed shoulder bracket up with one hand and push a servo motor into its pocket with the other

A bimanual assembly task from building a robot arm. The shoulder bracket of an SO-101 arm (its 3D-printed "rotation
pitch" part with the motor holder screwed on) stands on the table with its servo pocket empty; an STS3215 servo stands
next to it on its rear end.
- The left hand picks the bracket up by its motor holder and holds it up.
- The right hand picks the servo up, turns it a quarter turn so its horn faces the robot's left (the side the upper
  arm goes on), and pushes it into the pocket until it sits on the bottom. It lets go and goes back.
- The left hand sets the bracket down standing, lets go and goes back.

The pocket leaves 0.25 mm a side and is 19 mm deep, and the bracket hangs in a gripper: the servo has to come down
square and in line with a part that is not fixed. It fits two ways round; only one puts the horn where the upper arm
goes.

| File | What it is |
|---|---|
| `servo_mount.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/so101_shoulder_bracket.usda` | The SO-101 shoulder bracket with its motor holder (OpenUSD), one rigid body: the servo's pocket as exact boxes |
| `assets/sts3215_servo.usda` | The STS3215 servo with its disc horn, one rigid body with exact box colliders |
| `assets/textures/` | The servo's side label (generic, no brand) |
| `assets/LICENSE-SO-ARM100.txt` | The Apache-2.0 licence of the SO-ARM100 project, whose meshes the two USD files use |
| `test_servo_mount.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Meshes:** the bracket, its motor holder and the STS3215 servo come from the SO-ARM100 project ("Standard Open
SO-100 & SO-101 Arms", [github.com/TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), commit
`a758567`), licensed [Apache-2.0](assets/LICENSE-SO-ARM100.txt): its `Simulation/SO101` meshes, placed together as in
its `so101_new_calib.urdf`. Changed for this task: converted to OpenUSD, decimated, materials, colliders and a label
added.

**Everything else** (colliders, materials, label, build script): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Shoulder bracket:**
  - the SO-101's rotation-pitch part with its motor holder, yellow PLA, 84 mm tall, 45 g (the URDF's shoulder link,
    100 g, less its servo); it stands on its lower fork arm;
  - the pocket is exact boxes: 25.3 x 35.2 mm inside (0.25 mm a side around the servo), 19 mm deep, with a 1.5 mm
    lead-in at the rim (the printed rim is rounded); its walls reach out to the holder ring's real outer faces, which
    the left hand pinches (52.4 mm across). The printed CAD fits the servo with 0–0.2 mm; real ones are pressed in;
  - the rest collides as three flat-bottomed convex pieces made from the part's own cross-sections (its foot with the
    lower fork arm, its column, its top with the upper fork arm), so it stands without rocking.
- **Servo:**
  - an STS3215 serial bus servo, case 45.4 x 24.8 x 34.7 mm (datasheet 45.2 x 24.7 x 35 mm), 55 g, a 20 mm disc horn
    on its output shaft near one end, a label on one side;
  - it collides as three boxes (the case) and two 16-sided prisms (horn, bearing boss): PhysX cooks convex hulls of
    millimetre parts up to 0.3 mm off, more than the fit;
  - it stands on its rear end; it goes into the pocket that end first, horn on the outside.

Nothing is downloaded at run time.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=servo_mount.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/servo-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=servo_mount.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/servo-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With `-E object_jitter=0`
  the layout is fixed and no seed is needed. The job log prints a `[servo_mount] t=...  steps done: ...` line
  whenever a step is done or undone (with how high the servo's rear end is above the pocket's floor, how far off the
  pocket's line, how tilted, and how far its horn is turned from the right side), and a trace line every second.
- **The demo.** It reaches 1.0 at step 489 (24.5 s): bracket picked up at 6.0 s, servo in the pocket the right way
  round at 13.45 s and fully home at 13.5 s, bracket set down at 20.25 s, both arms home by 24.5 s. A trial takes about
  4 minutes on an L40S, including startup.
  - The left arm comes at the bracket from behind, pointing forward and 10° down, and pinches the motor holder's ring
    across its long sides (52.4 mm, 30 N a finger), the pads' middle on the pocket's axis. It lifts the bracket 5 cm and
    holds it there.
  - The right arm grips the standing servo from above across its sides, its pads' lowest corners 24 mm above the
    servo's rear end (5 mm over the rim once seated). It leans 30° about its closing axis: straight down, the right
    wrist doesn't reach this high. It lifts the servo over the held bracket, turning it a quarter turn on the way so
    the horn faces left, and lowers it in line with the pocket.
  - It pushes the servo in line toward 11 mm deep (it binds after 2 mm) and lets go. Freed, the servo
    lines up with the walls and drops to the floor. The closed fingertips then press its top 2 mm past seated, and the
    right arm goes home.
  - The left arm sets the bracket down where it stood, lets go, backs off and goes home.
  - Held in the air, the bracket hangs 2 mm lower than where the hand was sent and 0.8° tilted. Pushed, it tilts about
    1° the other way and slides 4 mm down in the grip. That is more than the 0.25 mm play but well inside the 1.5 mm
    lead-in.
- **Videos:** `inspect-robots video ~/yam-eval/servo-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_servo_mount.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order: the left picks the bracket up by its motor holder and holds it, the right
picks the servo up, turns its horn to the left and pushes it fully into the pocket, lets go and goes back, then the
left sets the bracket down and goes back. It leaves out where things are, which the model finds from the cameras.
"Horn facing your left" is the one thing the cameras can't tell: the pocket takes the servo either way round.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Bracket picked up | a gripper holds the bracket with its lowest point at least 1 cm above the table | once reached |
| 2. Servo in the pocket the right way round | its rear end at least 5 mm down in the pocket, within 1.5 mm of the pocket's centre line, in line with it within 5°, horn toward the bracket's upper-arm side within 30° | at the end |
| 3. Pressed fully home | step 2, and its rear end within 1 mm of the pocket's floor, in line within 2° | at the end |
| 4. Set down | step 2, the bracket standing on the table (its underside within 3 mm of it, upright within 10°), no gripper holding the bracket or the servo | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and the bracket and the servo still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps, if either part has fallen off the table (centre more than 2 cm below its
  top), or the state is not finite.
- **Steps 2 to 4 count only after a gripper picked the servo up:** a servo that ends up in the pocket some other way
  earns nothing.
- **Relative to the bracket:** the servo's place and its horn are judged in the bracket's own frame, wherever the
  bracket is, so a bracket set down turned still counts if the servo is in it the right way round.
- **The wrong way round:** a servo in the pocket with its horn on the other side never counts as in the pocket.
- **"A gripper holds" in simulation** is read from the simulator: both finger pads are on the part (its box, 5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
  The pads, not the gripper's axis: while the left hand pinches the holder's ring, its axis runs through the pocket.
- **On a real station** the same steps are judged from the video, with a ruler for the servo's height over the rim.

## Reset

- Pull the servo out of the bracket's pocket and stand it on its rear end at its mark, horn away from the robot.
- Stand the bracket at its mark, the motor holder's flat back toward the robot.
