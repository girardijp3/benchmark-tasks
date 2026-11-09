# Fit the PH2 bit into an electric screwdriver's hex bit holder

A bimanual tool-change task. A compact 4 V pen screwdriver stands nose up in its charging dock, its magnetic bit
holder empty; next to it, an aluminium bit block holds four 25 mm 1/4-inch hex bits: PH1, PH2, SL5 and T20.
- One hand holds the screwdriver steady in its dock.
- The other picks the PH2 bit out of the block, turns it so its hexagon lines up with the holder's hex socket, and
  pushes it in until it seats. Both hands let go and both arms go back.

The socket is 6.45 mm across flats on the bit's 6.35 mm hexagon: 0.05 mm a side. The bit goes in only within about
1.6° of the socket's hexagon and 0.3° of its axis (the screwdriver can lean a little in its dock to help), and the
two hexagons start 25° apart, so the hand has to turn the bit on the way. The right bit has to be told apart from three
others of the same size: by its red ring, its cross tip and the size printed on the block.

| File | What it is |
|---|---|
| `bit_insert.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/screwdriver_4v.usda` | The screwdriver (OpenUSD), with its hex socket, lead-in and retaining ball |
| `assets/screwdriver_dock.usda` | Its charging dock (spawned fixed to the table) |
| `assets/bit_block.usda` | The bit block, its sizes printed on it |
| `assets/bit_ph1.usda`, `bit_ph2.usda`, `bit_sl5.usda`, `bit_t20.usda` | The four bits |
| `assets/textures/` | The block's size labels and the screwdriver's label (no brand) |
| `test_bit_insert.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Screwdriver, dock, bit block and bits** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up. Dimensions follow
ISO 1173 (1/4-inch hex insert bits, form C6.3, 25 mm), ISO 10664 (T20) and the class of 3.6–4 V Li-ion pen
screwdrivers (25–30 mm across, 140–200 mm long, 200–300 g).

- **Screwdriver:** 27 mm across, 145 mm long, 230 g; teal-grey body with a black ribbed rubber grip, a rocker switch,
  charge LEDs and an LED ring around the chuck.
  - Its nose holds a 1/4-inch hex magnetic bit holder: a hex socket 6.45 mm across flats, 10.6 mm deep, with a
    0.6 mm × 45° lead-in.
  - The magnet is modelled as a 1 mm retaining ball in one flat with a springy (compliant) contact, pressing a seated
    bit against the opposite flat.
  - A key rib on its battery end fits a keyway in the dock, so the socket's hexagon always faces the same way (within
    about 0.55°).
- **Dock:** a round black base 76 mm across with a cup 27.3 mm across (0.15 mm clearance), fixed to the table. The
  screwdriver can lean about 0.6° in it.
- **Bit block:** anodised aluminium, 84 × 24 × 16 mm, 78 g, on a rubber pad, loose on the table. Four hex holes 6.70 mm
  across flats and 10 mm deep, 18 mm apart, each size printed in front of its bit with a colour bar.
- **Bits:** 25 mm, 6.35 mm across flats, 5.5–5.9 g: PH1 (green ring), PH2 (red ring), SL5 (yellow ring), T20 (blue
  ring), each with its own tip.

**Colliders for a 0.05 mm fit.** Exact primitives wherever something fits or rests:
- the socket is six boxes on its hexagon, six tilted boxes for its lead-in, a box for its floor and a sphere for the
  retaining ball;
- each bit's hex shank is three crossed boxes: a regular hexagon is exactly the union of the three rectangles spanning
  its opposite flats. Three smaller ones make its end chamfer as a 0.3 mm step;
- the screwdriver's body is three capsules (exactly round in the dock's cup and in the hand), on a flat foot;
- the dock's cup is 18 boxes, the block's holes six boxes each.

A first version had each bit's shank as an exact 18-point convex prism. In a standalone PhysX test (the bit pushed
down a rail into the socket) it stopped 1 mm in even when perfectly lined up: PhysX cooks small convex meshes a little
larger, more than the fit's 0.05 mm a side. The three-box shank goes fully in within 1° of the hexagon and stops at
2°, as the geometry says.

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=bit_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/bit-dream

# Scripted demo (both arms), 620 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=bit_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 620 --store-frames --log-dir ~/yam-eval/bit-demo
```

- **Score.** The run prints `task_score` at the end: 0.25 per step done, 1.0 on success. With `-E object_jitter=0`
  the layout is fixed and no seed is needed. The job log prints a `[bit_insert] t=...  steps done: ...` line whenever
  a step is done or undone, with how far the PH2 bit is in the socket, off its axis, tilted and turned against its
  hexagon, and how the screwdriver stands.
- **The demo.** It reaches 1.0 at step 486 (24.3 s): PH2 bit taken out at 5.9 s, started in the holder at 14.6 s
  and seated 0.05 s later, pressed home, both hands off with everything in place at 19.9 s, both arms home by 24.3 s.
  - The left arm grips the screwdriver's rubber grip from its own side, pointing 40° below level, fingers closing
    level across it (about 10 N a finger), and holds it.
  - The right arm, pointing straight down, closes its fingers gently (about 7 N a finger) across two flats of the PH2
    bit near its top: the flat pads line the bit's hexagon up with the fingers. It lifts the bit out and carries it
    over the socket, turning the wrist 25° on the way so the bit's hexagon matches the socket's.
  - It lowers the bit to 2 mm above the mouth, settles, then starts it 1 mm in, into the lead-in, and lets go: the
    bit slides down to the magnet. (Pushing the pinched bit all the way in tilts it a few degrees in the fingers, and
    the hexagon jams; it fits only within 0.3°.) The closed fingertips then press on the bit's tip to push it home.
  - The right arm goes home; then the left lets go, backs off and goes home.
  - The scripted hand aims at where the bit actually is: three small corrections measured in simulation (the hand's
    offset at the socket, about 1.5 mm; its turn, 0.7°; the bit's lean in the fingers, 1.3°) are in the task file
    (`INSERT_FIX`, `YAW_FIX`, `TILT_FIX`).
- **Videos:** `inspect-robots video ~/yam-eval/bit-demo/<log>.json` writes one MP4 per camera next to the frames.
- **Offline checks:** `python -m pytest -q test_bit_insert.py`.
- **Prompt:** add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It says which hand does what and the order: the left holds the screwdriver steady, the right takes the PH2 bit, lines
its hexagon up with the holder and pushes it in, then both let go and go back. It names the bit by its size only, as
printed on the block; the colour rings are not mentioned. It leaves out where things are, which the model finds from
the cameras.

## Scoring

Four steps, **0.25 each** (`STEPS` in the task file).

| Step | Done when | Judged |
|---|---|---|
| 1. PH2 bit taken out | a gripper holds the PH2 bit lifted clear of the block (its lowest point above the block's top) | once reached |
| 2. PH2 bit started in the holder | its end at least 2 mm into the socket, within 1 mm and 5° of the socket's axis | at the end |
| 3. PH2 bit seated | at least 9 mm in (fully home is 10.6 mm), within 0.5 mm and 2° of the axis, its hexagon within 3° of the socket's | at the end |
| 4. Let go, everything in place | with step 3 done: no gripper holds the bit or the screwdriver, the screwdriver stands in its dock (within 1.5 mm and 3°), and the PH1, SL5 and T20 bits stand in their own holes | at the end |

- **1.0 (success, ends the run):** all four steps, both arms back where they started (within 3 cm) holding nothing,
  and the screwdriver and the bit in it still (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0** at the end, whatever the steps, if the screwdriver, the block or any bit has fallen off the table (centre more
  than 2 cm below its top), or the state is not finite.
- **The right bit:** steps 2 and 3 count only for the PH2 bit, and only once a gripper has taken it out of the block.
  Another bit pushed into the holder earns nothing, and a bit pulled out of its hole and not put back fails step 4.
- **"A gripper holds" in simulation** is read from the simulator: the finger pads reach the part's bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`).
- **On a real station** the same steps are judged from the video and by hand: the bit's colour ring shows just above
  the chuck when it is seated, and a seated bit doesn't wobble.

## Reset

- Pull the bit out of the screwdriver's holder and stand it back in its hole in the block (PH2, second from the
  robot's left), hex end down.
- Stand the screwdriver back in its dock, rib in the keyway.
- Put the block back on its mark.
