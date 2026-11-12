# Put the battery back in a flashlight the right way round and screw its cap on

A bimanual repair step: one hand holds a small 18650 flashlight standing head-down with its tail open, while the other
picks up the lithium-ion cell lying on the table, turns it positive nub down, slides it into a bore with 0.4 mm of
play, then picks up the tail cap and screws it on. The cap's 4-start thread is modelled as real collision geometry,
so the cap has to be turned (a little over one turn) to go on.

| File | What it is |
|---|---|
| `flashlight_insertion.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/flashlight_18650.usdc` | The flashlight (OpenUSD): body, tail cap and battery as three rigid bodies, threads included |
| `assets/flashlight_{body,cap,battery}.usda` | One rigid body each, referencing the flashlight file, with the task's friction settings |
| `assets/textures/battery_18650_wrap.png` | The battery's wrap label (no brand) |
| `test_flashlight_insertion.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Flashlight** (`assets/`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). 110 mm long, 34 mm head, 22 mm body tube, 26 x 21 mm tail
cap, black anodised aluminium; units metres and kilograms, Z up.

- Three rigid bodies: the body (70 g: tube, head, LED, reflector, lens, bezel and the male thread), the tail cap
  (18 g: cap, female thread, switch button) and an 18 x 65 mm 18650 cell (47 g) with a raised positive nub.
- The tail-cap thread is 4-start trapezoidal, 2.5 mm pitch, 10 mm lead, right-hand: turned clockwise (seen from the
  tail) the cap goes on, and a collar on the body stops it at the closed position. Thread, tube, head and cap collide
  as signed distance fields (SDF), so the threads really engage; the cell collides as its convex hull.
- The bore is 18.8 mm, the cell 18 mm.
- The part files set the friction: the cap's outer shell and the body tube grip like rubber (1.5 static, 1.3
  dynamic) and stand in for knurled aluminium and compliant finger pads; the threads keep aluminium's 0.35 / 0.30;
  the cell's PVC wrap is 0.30 / 0.25 and takes the lower of the two frictions in any contact, so it slides down the
  bore under its own weight.

No other assets: nothing is downloaded.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flashlight_insertion.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/insert-dream

# Scripted demo (both arms), 1,800 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flashlight_insertion.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/insert-demo
```

- The run prints `task_score` at the end (0.2 per step done (at most 0.8), 1.0 only on success). With `-E object_jitter=0` the layout
  is fixed and no seed is needed. The job log prints a `[flashlight_insertion] t=...  steps done: ...` line whenever a
  step is done or undone, with how far the cap has turned.
- The demo has all four steps done at step 1,083 (54 s) and reaches 1.0, ending the run, at step 1,126 (56.3 s),
  once the three parts have stayed in place for the hold: battery picked up at 8.4 s, in
  positive end first at 21.8 s, cap on the thread at 39.9 s, screwed down at 54.2 s after 464 deg of turning. The
  left arm grips the body from the side, lifts it 3 cm, carries it 5 cm toward the right arm and stands it back on
  the table, holding it there. The right arm picks the cell up from above, tilted toward the nub, tips it 90 deg
  nub-down, lines it up over the bore, slides it about 1 cm in and lets it drop to its seat; then it picks up the
  cap, sets it on the thread and screws it on clockwise in grips of 200, 200 and 100 deg, sinking with the thread.
  A trial takes about 4 minutes on an L40S, including startup.
- Videos: `inspect-robots video ~/yam-eval/insert-demo/<log>.json` writes one MP4 per camera next to the frames.
- Offline checks: `python -m pytest -q test_flashlight_insertion.py`.
- Add `--no-prompt` to skip the operator's success question at the end of a run that doesn't succeed.

## Instruction

It gives the model what to do with each hand and in which order, and the one thing the cameras can't settle: which
end of the cell goes first. It leaves out how to screw the cap on and where things are, which the model works out
from the cameras.

## Scoring

Four steps, **0.2 each** (`STEPS` in the task file); only full success scores 1.0. Positions are centres in metres; positions along the
flashlight's axis are relative to the body's centre. Assembled, the cell's centre is 11 mm and the cap's 47 mm from
the body's, toward the tail; the cell's flat end is then 2 mm inside the tube's rim.

| Step | Done when | Judged |
|---|---|---|
| 1. Battery picked up | a gripper holds the cell with its lowest point at least 2 cm above the table | once reached |
| 2. Battery in, positive end first | the cell is in the tube (centre within 3 mm of the axis, axis within 10° of it), its nub toward the LED head and its flat end no higher than the rim | at the end |
| 3. Cap on the thread | with step 2 done, the cap sits on the tube's end (centre within 1.5 mm of the axis, within 17.5 mm of its seated place) after turning at least 90° clockwise on the thread | at the end |
| 4. Cap screwed down | with step 2 done, the cap is within 1 mm of its seated place (tight against the collar) after turning at least 300° | at the end |

- **1.0 (success, ends the run):** all four steps, and the three parts in place for 1 s: each within 2 mm and 2° of
  where it was 1 s before (`settled`), as a person would judge "nothing is moving" from the video. It is judged from
  the poses, not the simulator's instantaneous speeds: a cap screwed tight keeps a contact jitter of tens of mm/s while
  it stays within 0.2 mm and 0.5°.
- **All four steps without full success score 0.8:** the step points stop at 0.8, so a run that finishes
  the steps but doesn't meet the rest of the success conditions above never scores 1.0.
- **0** at the end, whatever the steps, if the cell is in the tube the wrong way round (nub toward the cap), any part
  has fallen off the table (centre more than 2 cm below its top) or the state is not finite, or the cap sits on the
  collar without having turned 300° (forced down rather than screwed: real threads don't allow it).
- The cap steps count only over a cell put in the right way: capping an empty or wrongly loaded flashlight earns
  nothing for them. Which arm does what is in the instruction, not the scoring.
- In simulation, "a gripper holds the cell" is read from the simulator: the finger pads reach its bounding box (5 mm
  slack), both fingers press with at least 0.5 N, and the pads are at least 3 mm apart (`GripperState`, `_held`). The
  cap's turn on the thread is tracked from the two parts' orientations and starts over whenever the cap leaves the
  tube's end. On a real station the same steps are judged from the video, with a ruler for the heights.

## Reset

Unscrew the cap and set it open end down at its mark; take the cell out and lay it on its side at its mark, nub pointing
away from the robot; stand the flashlight head-down at its mark.
