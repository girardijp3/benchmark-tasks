# Swap a hair clipper's number 2 guide comb for the number 4 and clip it on

A one-job hairdressing task with two hands. A hair clipper lies loose on the table with a number 2 guide comb clipped
on its blade; to its right a tray holds guides 1, 3, 4, 5 and 6 and one empty slot.
- The left hand holds the clipper still: it is loose on the table (about 1.4 N of friction), and both pulling a comb off
  and pushing one on take more than that, so an unheld clipper just slides away.
- The right hand pulls the number 2 off the blade, puts it in the empty slot, picks the number 4 (the number is
  raised on each comb's roof; the combs differ in height too) and pushes it onto the blade until its rear clip drops
  behind the catch ridge.
- Both hands let go and the parts stay still.

| File | What it is |
|---|---|
| `clipper_guard_swap.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/clipper.usda` | The clipper (OpenUSD), a rigid body with a catch ridge on its blade collar |
| `assets/guard_1.usda` ... `guard_6.usda` | The six numbered guide combs, each one articulation: comb plus a hinged rear clip |
| `assets/tray.usda` | The guide tray (a fixture) |
| `test_clipper_guard_swap.py` | Offline checks: the file loads, every scoring outcome and shortcut, both arms' reach |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are
metres and kilograms, Z up. Colliders are boxes, with a few small convex prisms for the lead-ins and the ridge.

- **Clipper** (280 g, typical of a compact corded Wahl-type clipper; 160 mm long): housing 93 x 42 x 36 mm, blade
  collar 22 x 56 x 13 mm, blade head 45 x 46 x 9 mm (standard 46 mm blade width). The catch ridge on the collar's top
  rises 2.2 mm over 3 mm (front slope) and falls over 1.8 mm (rear face). It lies on its flat back.
- **Guides** (14-17 g, ABS-class plastic at 1050 kg/m3, solid): a 54 x 53 mm comb. Cutting lengths follow the usual
  numbering of 1/8 inch per number: 3.2, 6.4, 9.5, 12.7, 15.9, 19.1 mm for numbers 1 to 6; the roof stands that far
  above the blade's top face, so the guides are 16.2 to 32.1 mm high. Clearances on the blade: 1 mm above and below,
  1.5 mm each side, with lead-in chamfers (the rear opening is 3.5 mm wider each side than the blade; the floor's rear
  edge and the bridge's rear edge are ramped). Numbers are raised 0.6 mm white digits, 10 mm high (visual only).
  Teeth are visual: the colliders are solid slabs.
- **The clip:** each guide's rear clip is a 1.5 mm arm on a hinge (spring 0.4 N·m/rad, preloaded 2 degrees, hook lift
  limit 12 degrees) whose hook rides up the ridge's front slope and drops behind it when the comb's rear edge reaches
  the collar's face (about 1.7 mm of play). It is real contact, designed for about 3 N to push on and about 5 N to
  pull off. The sizes of the clip are typical values: real clips are moulded flex arms.
- **Tray:** 238 x 150 x 2 mm base, 4 mm rim, six slots with printed outlines; five guides stand in it.
- Friction: ABS 0.40 / 0.35, steel 0.40 / 0.35, rubber tray 0.90 / 0.80.

## Run it

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clipper_guard_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/clipper-demo
```

The log prints a `[clipper_guard] t=...  steps done: ...` line when a step is done, plus a trace every second.

**The demo.** On Isaac (L40): number 2 guide off at 9.65 s, number 4 lifted from the tray by 23.15 s, on the blade at 32.15 s, success conditions met at 38.95 s and 1.0 at step 789. Open loop, no readings of the simulator.
- Left arm: grips the housing across its width near the rear end (gripper leaning 25 degrees, wrist away from the
  right arm), holds until the right hand has let go, lets go, lifts and moves away.
- Right arm: grips the number 2's side walls, pulls it 85 mm off the blade, lifts it, puts it in the empty slot and lets
  go; grips the number 4, lifts it 25 mm, carries it toward the clipper and across to the blade's line, sets it on the
  table with its mouth at the blade's tip, then slides it on over 4 s (commanded 8 mm past the seat), lets go and moves
  away.

## Instruction

It states the goal and leaves out how, which hand, the tray, where things are and the rule that the other guides stay.

## Scoring

Four steps, **0.2 each**; only full success scores 1.0. Steps 1 to 3 count once reached; step 4 is judged as things stand.

| Step | Done when |
|---|---|
| 1. Number 2 off the clipper | its rear edge is at least 50 mm past the collar's face |
| 2. Number 4 lifted | a gripper holds it (both fingers press, pads apart) and it is at least 12 mm above its start |
| 3. Number 4 on the blade | after a gripper has held it: at least 15 mm of blade inside it, within 4 mm sideways, 3 mm in height and 6 degrees |
| 4. Number 4 clipped on | after steps 1 and 2: rear edge against the collar (-0.8 to +2.5 mm), square (3 mm, 3 degrees, 4 degrees tilt), clip arm down (within 1.5 degrees), and no other guide on the blade |

- **1.0 (success):** all four steps, the other four guides in the tray where they were (15 mm, 15 degrees), no gripper
  pinching and every gripper's pads at least 3 cm from the clipper and all guides, and the clipper and every guide
  within 2 mm and 2 degrees for 1 s. The 1 s is timed in the task file (`SUCCESS_HOLD_S = 0`). Arms home is not scored.
- **Guards:** a wrong number clipped on, the number 2 left on the blade, a guide dropped on the blade without a hand,
  seated with the hook up or 6 mm short, a hand still pinching or hovering within 3 cm, or a disturbed tray guide gives
  no full score. Doing nothing scores 0. With the left arm idle the demo scores 0.2: the clipper is carried away.
- **0** if the clipper or any guide falls off the table or the state is not finite.
- **Real world:** a seated guide must resist a pull of about 5 N and a ruler shows its rear edge flush with the collar.

## Reset

Lay the clipper on its back at its mark with the number 2 pushed fully on, and the other guides in their slots, teeth
toward -y; the guides may be shuffled between slots between trials.

## Status

Dream-1 score: 0.0 (both arms moved up from their bases and hovered; the number 2 guide stayed on the clipper and nothing was picked from the tray, in the 1,200-step run limit). Demo: 1.0 (number 2 off at 9.65 s, number 4 lifted by 23.15 s, on the blade at 32.15 s, clipped on and success at step 789). Media: M-BTZ59Z.
