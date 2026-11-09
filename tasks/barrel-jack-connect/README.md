# Plug an LED dimmer's connector plug into a wall adapter's connector

A bimanual connector task. A 12 V wall adapter is plugged into a wall socket behind the bench; its cord runs down
onto the bench and ends in a female 5.5 x 2.1 mm in-line barrel jack. An LED strip dimmer stuck to the bench has an
input cord ending in the matching male plug.
- One hand picks the jack up and holds it.
- The other picks the plug up by its overmold, lines it up with the jack's bore and pushes it fully in, lets go and
  goes back.
- Then the first hand lays the joined connectors on the bench, lets go and goes back.

The barrel is 5.5 mm across and the bore 5.9 mm, with spring contacts that centre it: the plug has to come in on
the bore's line, within about two degrees, or it jams at the mouth. Both connectors hang on loose cords, so neither stays put by itself.

| File | What it is |
|---|---|
| `barrel_jack.py` | The `yam_arena` task file: scene, the cords' starting layout, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/adapter_jack_cable.usda` | The wall adapter, its 350 mm cord and the in-line jack: one articulation |
| `assets/controller_plug_cable.usda` | The LED strip dimmer, its 290 mm input cord and the barrel plug: one articulation |
| `assets/wall_panel.usda` | The painted back panel the socket is in |
| `assets/textures/` | The adapter's rating label and the dimmer's top label (no brand) |
| `test_barrel_jack.py` | Offline checks: the file loads, the layout, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Adapter, dimmer, connectors, cords and back panel** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up.

- **Wall adapter:** a 12 V DC 1 A Europlug wall adapter, 36 x 46 x 70 mm black ABS with a rating label, plugged into
  the socket (fixed). Its 3.5 mm two-core PVC cord leaves its bottom through a strain relief.
- **In-line jack:** a female 5.5 x 2.1 mm barrel jack, 10.5 mm across and 42 mm long with its strain relief, 7 g:
  a 9.8 mm deep bore with a 45 deg x 0.6 mm lead-in, a split contact sleeve and a 2.0 mm centre pin.
- **Barrel plug:** a male 5.5 x 2.1 mm plug as the CUI/Same Sky PP3-002A (5.5 mm OD, 2.1 mm ID, 9.5 mm mating
  length), with a 9 mm, 20 mm long overmold and a strain relief, 4 g.
- **LED strip dimmer:** a single-colour 12-24 V dimmer, 60 x 34 x 18 mm black ABS with a knob and a top label, 35 g,
  stuck to the bench by its adhesive pad (fixed). Its input cord ends in the plug; its short output lead ends in a
  two-pin connector (not used).
- **Cords:** 12.5 mm capsule links (0.275 g, 22 g/m) on spherical joints with springs: bending 0.012 N m/rad, twist
  0.0088 N m/rad, +/-50 deg and +/-20 deg limits. The adapter's cord is 350 mm (28 links) and the dimmer's 290 mm
  (23 links); real ones are 1.5 m and longer. A PhysX articulation holds at most 64 links, and most of a long cord
  would only lie still on the bench: these are long enough to reach where the plug goes in, with slack.
- **Back panel:** an 18 mm white-painted board, 800 x 400 mm, standing on the bench 56 cm in front of the arms.

**Wall socket:** SimReady `equipment_powerOutlet_wallMount_b01` (a 60 x 60 mm Schuko socket with a flip lid) from the
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) dataset, CC-BY 4.0, revision
`7879e0e50423290300ac20ecd6e727325f680b60`. The task file downloads it at spawn (about 36 MB, checked against the
dataset's SHA-256 sums) into `~/.cache/simready` (or `$SIMREADY_CACHE`); it isn't in this folder. It is spawned
as part of the fixed panel, its lid flipped up onto the adapter, without its own physics.

**How the fit is simulated (PhysX):**

1. **Exact primitives.** PhysX cooks convex colliders of millimetre-sized parts up to 0.3 mm off, so the fit is
   made of its exact shapes:
   - the jack's bore is twelve boxes, a 12-sided tube 5.9 mm across inside (0.2 mm a side around the barrel). With
     0.1 mm a side, a demo run jammed the plug 2.4 mm short at 3.6 deg of tilt, a slant the fingers' friction allows;
   - twelve more boxes form the 45 deg lead-in, and twelve the front face;
   - the plug's barrel is a 5.5 mm capsule with a ring of sixteen 0.5 mm capsules at its tip. That gives the real
     barrel's small rounded edge, where a single capsule's 2.75 mm round nose would centre the plug far more easily
     than a real one;
   - the jack's 2.0 mm centre pin is visual only.
2. **Springy contacts.** The jack's split contact sleeve is three rounded bars 5 mm inside the bore, at 120 deg,
   reaching 0.32 mm past the bore's wall into the barrel's path (0.12 mm into a centred barrel). They have a
   compliant contact (5000 N/m) and centre the barrel. The plug pulls out at 0.6-1 N in simulation (in a rail test
   it held 0.6 N and slid out at 1 N), against 3-30 N for a real jack (datasheets give 0.3-3 kgf insertion and
   withdrawal forces).
3. **The cords' starting shape.** Arena resets objects' poses but not their joint angles, and the simulator starts
   joints at 0. So the task file lays both cords out when they spawn, and a reset event writes the same joint angles
   at each episode's start. The cords' bending springs rest in that shape (their targets are written into the USD
   at spawn), like cords that have taken a set, so they lie still: in a spawn run no cord part moved faster than
   1 mm/s.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word. The first run downloads
the wall socket (about 36 MB) from Hugging Face.

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=barrel_jack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/barrel-jack-dream

# Scripted demo (both arms), 700 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=barrel_jack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/barrel-jack-demo
```

- **Scores:** Dream-1 0.0 (it never picked up the jack or the plug); the scripted demo 1.0 (success at step
  607), three runs out of three. Media ID M-A8RTXT.
- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With
  `-E object_jitter=0` the layout is fixed and no seed is needed.
  - The job log prints a `[barrel_jack] t=...  steps done: ...` line whenever a step is done or undone, with how far
    the plug is from fully in, how far off the bore's line and how tilted.
  - Every half second it also prints a `trace` line: both connectors' positions and the fastest part of either cord.
- **The demo.** It reaches 1.0 at step 607 (30.4 s): plug lifted at 6.0 s, jack lifted at 6.2 s, plugged in at
  14.7 s, the joined connectors laid down and let go at 25.5 s, both arms home by 30.4 s. A trial takes about 4
  minutes on an L40S, including startup.
  - The left arm grips the jack straight down across its middle and lifts it 5 cm, its mouth toward the right.
  - The right arm grips the plug's overmold straight down, lifts it, and turns its hand 70 deg about the plug's own
    axis: the gripper then points forward and down, its fingers above and below the plug, so its wrist stays in front
    of the plug, clear of the left wrist above the jack. The plug keeps level all the while.
  - The right arm lines the plug up 3 cm short of the jack's mouth, then 12 mm short, and pushes it in along the bore
    until its overmold meets the jack. It opens to 24 mm, backs off, moves aside and goes home.
  - The left arm lays the joined connectors back where the jack lay, lets go and goes home.
  - The right hand's aim is turned 1.8 deg up and 1.1 deg away from the robot, as measured in simulation: the plug
    sits that much off in the fingers.
- **Videos:** `inspect-robots video ~/yam-eval/barrel-jack-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- **Offline checks:** `python -m pytest -q test_barrel_jack.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order, including how the task ends: the plug pushed fully in, the joined
connectors laid on the bench, and both arms back. It leaves out where things are, which the model finds from the
cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. Jack lifted | a gripper holds the jack with its lowest point at least 2 cm above the bench | once reached |
| 2. Plug lifted | a gripper holds the plug with its lowest point at least 2 cm above the bench | once reached |
| 3. Plugged in by hand | the plug is fully in the jack (its overmold within 1 mm of the jack's mouth, within 0.5 mm of the bore's centre line, its axis within 5 deg of the bore's), and a gripper held the plug when it went in | at the end |
| 4. Joined connectors laid down | with step 3 done, both connectors lie on the bench (lowest points within 3 mm of it) and no gripper holds either | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and both connectors still (under 1 cm/s) for 1 s.
- **0** at the end, whatever the steps, if a connector has fallen off the bench (its centre more than 2 cm below
  the bench top) or the state is not finite.
- **Not by hand:** a plug that ever went fully in with no gripper on it never earns steps 3 and 4 in that run. That
  covers a plug pulled in by its cord, or the jack shoved onto a plug lying on the bench.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video, with a ruler for heights and gaps. A plug is fully
  in when no metal barrel shows between its overmold and the jack.

## Reset

- Pull the plug out of the jack.
- Lay the adapter's cord back from the wall along the bench toward the robot, turning right into the jack. The jack
  lies with its mouth to the robot's right.
- Lay the dimmer's cord toward the robot, turning left into the plug. The plug lies with its barrel to the robot's
  left.
- The adapter stays in the wall socket and the dimmer stays stuck to the bench.
