# Close a swollen sliding window by its brittle plastic tab, without bending the tab

A household and garden task. A wooden sliding-sash window has swollen and sticks in its track; its only handle is a brittle plastic pull tab on the sash's front face. The robot has to close the window. The instruction states only the goal and the two conditions (tab only, no bending); that the tab must be gripped near its root and the sash drawn steadily, is the robot's to work out.

The skill is a precise, gentle grasp of a 10 mm tab 70 mm above the table, then a steady pull of about 6 N against the track's dry friction through 150 mm, stopping when the sash meets the jamb: the arm lags the load by about 3 cm, a pull that overshoots the stop loads the tab, and a grip near the tip bends it.

| File | What it is |
|---|---|
| `sticky_sash_window.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/window_frame.usda` | The window frame, sill, lane walls, header, jambs and fixed pane (OpenUSD); fixed |
| `assets/sash.usda` | The sash and its pull tab on a spring hinge (OpenUSD); one articulation with a free root |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; boxes, capsules, spheres) |
| `test_sticky_sash_window.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Window frame** (fixed): a 40 mm sill with 6 mm lane walls either side of each lane, a header with lane walls, two 50 mm jambs (the left one, +y, is the sash's end stop) and a fixed 4 mm pane in the back lane. Pine friction 0.40.
- **Sash** (one articulation, free root): pine stiles 38 mm, bottom rail 55 mm, top rail 38 mm, a 4 mm glass pane, 250 x 270 x 20 mm, 1.5 kg, friction 0.40 on the sill. A plastic tab (36 x 10 x 20 mm, 5 g, friction 0.5) on a revolute joint `Handle` (axis Z): spring 0.8 N m/rad, damping 0.004 N m s/rad, limits plus or minus 25 degrees.

**Building it for a real trial.** Pine sill 40 x 90 x 580 mm; lane walls 6 x 12 mm strips glued 22 mm apart; sash from 20 x 38 mm pine with a 55 mm bottom rail, 4 mm glass in a rebate; adjust the drag with the sill's finish until a spring balance on the tab reads 5.5 to 6.5 N at break-away. Tab: a 3D-printed 36 x 10 x 20 mm tab on a flexure or a screw hinge; replace it when it snaps.

## Layout

The window's middle is at x 0.36, y 0.02 on the table; the left jamb's face (the stop) is at y 0.26. The sash starts with its middle 150 mm from the stop and the tab at y 0.065, x 0.35. The left arm does the whole demo.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sticky_sash_window.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sticky-sash-window-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sticky_sash_window.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 350 --store-frames --log-dir ~/yam-eval/sticky-sash-window-demo
```

The log prints `[sash] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The left arm, pointing ahead and 25 degrees down, pads astride the tab 14 mm out from its hinge (SQUEEZE: fingers 2 mm inside the tab's width), grips it, pulls 180 mm along the sill over 5 s (the arm lags the sash by about 27 mm under the 6 N drag, so the sash arrives at the stop), holds, lets go, backs off. Success at about step 268 (13 s).

## Scoring

| Step | Done when |
|---|---|
| 1 | the sash is 10 mm along from its start |
| 2 | the sash is 75 mm along |
| 3 | closed: left edge within 3 mm of the stop, in its lane, tab not bent |
| 4 | with step 3, no gripper pad within 3 cm of the tab and none holds it |

- **1.0** only when every full-success condition has held together for 1 s, timed inside the task's success():
  all steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (a tab bent past 10 degrees, a sash moved by anything but a grip on the tab (or pushed by a second gripper's pads while the first holds the tab), a sash out of its lane, a sash off the table); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: a push on the sash's frame or glass moves it with no grip on the tab, which is a rule event; gripping the tab at its tip bends it past 10 degrees (MuJoCo: 0); a sash left short, or lifted out of the lane, does not close.
- **Real station:** judged from video: the sash lies shut against the jamb in its lane with the tab straight.

## Reset

Place the sash on the sill in the front lane with its middle at x 0.36, y -0.015 (150 mm from the jamb), the tab straight; nothing else is moved.

## Status

Dream-1 score: 0.0 (2400 steps, hit the step limit). Demo: 1.0 at step 267 (13.3 s). Media M-DTJVZE.
