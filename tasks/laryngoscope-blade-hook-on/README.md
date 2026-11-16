# Hook a laryngoscope blade onto its handle and click it into the lit position

A nurse's or paramedic's task. A laryngoscope for the crash cart comes in two parts: a round battery handle and a blade. The robot has to put them together the way the shift check does: slide the blade's hook slot onto the handle's hinge bar, swing the blade out through a quarter turn until it clicks into its locked, lit position, and leave the lit scope on the tray. The instruction states only the goal; how the handle is steadied, where the blade is taken and how the swing is split are the robot's to work out.

**Why it matters.** Every crash cart and every ambulance carries a laryngoscope that a nurse or paramedic has to check, typically each shift or each day (registered nurses, O*NET 29-1141.00, about 3.2 million in the US; EMTs and paramedics, 29-2042.00 / 29-2043.00, about 0.27 million). A blade that is not fully clicked in gives no light, and that is otherwise found out only during an emergency intubation. A robot that runs these routine equipment checks frees nurse time on every ward; the check needs a small mechanism worked by hand (hook, swing, click) and a light seen to come on.

The skill is a mechanism worked by hand: taking a 130 mm blade by its middle, sliding a 5 mm wide hook slot (with a 3 mm lead-in) onto a 4.5 mm bar to within a few mm, and swinging it about that bar through 90 degrees. The wrist cannot turn through all of it in one grasp, so the hand has to let go half way (the hinge holds the blade), turn the wrist round and take a fresh grip.

| File | What it is |
|---|---|
| `laryngoscope_blade_hook_on.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/handle.usda` | The laryngoscope handle (OpenUSD): 28 x 150 mm grip, head bridge and 4.5 mm hinge bar; free rigid body |
| `assets/blade.usda` | The Macintosh 3 style blade (OpenUSD): hook slot with lead-in; articulation (free root) with a visual lamp marker |
| `assets/tray.usda` | The crash-cart tray (OpenUSD): 260 x 400 x 3 mm plate; fixed |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; convex colliders: boxes, cylinders and prisms; visible parts are separate meshes: knurled chrome grip, tapered channel blade with flange and tip lamp, dark tray with a rim) |
| `test_laryngoscope_blade_hook_on.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Handle** (free, 0.25 kg): a medium C-cell fibre-optic handle of the Welch Allyn 60813 class (ISO 7376 green standard): a 28 mm x 150 mm grip (the 150 mm is a reseller's 6 in listing, the 28 mm and 0.25 kg are estimates), a head bridge 21-27 mm above the tray and a vertical 4.5 mm (4.505 mm) hinge bar under its outer end: the bar diameter is taken from ISO 7376:2009 Figure 2 (handle hook-on fitting of the green fibre-illuminated system, 4.505 +-0.065 mm; the 2020 edition's Table 1 gives the same 4.505 +-0.065 mm for light-guide blades; the conventional system's bar is 3.95 +-0.05 mm). The bridge height above the tray and its length are estimates.
- **Blade** (articulation, free root, 0.08 kg): a Macintosh 3 style blade, 130 mm long, 20 mm high and 25 mm wide at the base, with a 5 mm hook slot 6.5 mm deep and a 3 mm 45 degree lead-in on each side (estimates: the standard's blade-side drawing (Figure 3 of the 2020 edition) is not in the public previews, so the slot width, depth and lead-in are our estimates, chosen to clear the standard's 4.57 mm largest bar by 0.2 mm a side). A visual lamp marker at the tip slides out when the lamp is lit.
- **Tray** (fixed): 260 x 400 x 3 mm steel plate (estimate of a crash-cart drawer tray).

## Declared effects and estimates (fidelity)

- **Hinge: declared.** Once the bar is 3 mm or more inside the slot (within 1 mm of its centre line, 4 mm of its height, within 10 degrees of in line) the blade's root pose is written every step so it turns only about the bar, with zero velocity; a real J-hook does this by contact. The friction that holds a released blade at its angle is not modelled.
- **Detent and lamp: declared.** At 87 degrees or more the blade is held at 90 degrees, the lamp is lit and the lamp marker slides out 4.5 mm; the lamp lights only in the detent, and the video shows that marker. Pulling the blade back 4 degrees puts it out. The detent torque and the swing's reaction on the handle are not modelled.
- **Measured from ISO 7376:** the hinge bar diameter (4.505 mm). **Estimates:** slot width, depth and lead-in, the 130 mm blade, the 28 x 150 mm handle, the tray, the 87 degree click.
- **Click torque:** not modelled and not measured on a real handle (estimate 0.1-0.3 N m; a torque gauge on a real Welch Allyn class handle would give it).
- **Blade shape:** the visible blade is drawn straight (a tapered channel with a flange and a tip lamp); a real Macintosh 3 blade is curved along its length. Its hook-on base, which is what meets the bar and is scored, is the same.
- **Not part of this task:** a real check usually ends by folding the blade closed again so the battery does not drain; here the task ends at the light check, with the lit scope released on the tray.
- **One hand or two:** the handle lies on the tray, so one hand can do the whole fit (the right arm alone scores 1.0 on Isaac); the other may steady the handle.

Video note: the demo video's three panes are render-only camera views of the same run (a tray view, a view of the hinge, and a view of the blade tip lamp, which sits in a dark socket until the click and then rises and glows); the physics and the scoring are the task's own. The visible meshes are separate from the colliders, which are the same boxes, cylinders and prisms as before.

## Layout

The bar is at x 0.31, y 0.038. The left arm takes the handle (gripper pointing down, pads closing along x) at y 0.17; the right takes the blade (pointing down) 60 mm from its base face.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=laryngoscope_blade_hook_on.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/laryngoscope-blade-hook-on-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=laryngoscope_blade_hook_on.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/laryngoscope-blade-hook-on-demo
```

The log prints `[scope] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. The left arm takes the handle from above by its base end and holds it. The right arm takes the blade by its middle (15 N a finger), slides its hook slot 49 mm onto the bar and swings it out 45 degrees (the hand moves on an arc about the bar, the wrist turns with it). It lets go (the hinge holds), lifts, turns the wrist round by 180 degrees, takes a fresh grip and swings the blade the other 45 degrees until it clicks. It lets go, lifts away and goes home; then the left arm lets go and goes home. Success at step 542 (27.1 s) on Isaac.

## Scoring

| Step | Done when |
|---|---|
| 1 | the blade is hooked on the bar (bar 3 mm or more in the slot, flat, in line when it went on) |
| 2 | the blade is swung 45 degrees or more |
| 3 | the blade is clicked into the detent (87 degrees or more, held at 90), lamp lit |
| 4 | with step 3: both parts lie on the tray, nothing held, no pad within 3 cm of either part |

Hooking: the bar must be 3 mm or more inside the 5 mm slot within 1 mm of its centre line and 4 mm of its height, with the blade within 10 degrees of in line and flat (under 5 degrees). Once hooked, the blade turns only about the bar.

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file (in `success()`):
  all four steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the handle or the blade off the table); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: a blade laid on the tray beside the handle, a blade not in line when it meets the bar, one too high or off the bar's centre line, one pushed past the bar, and a blade swung by hand but not hooked score nothing; a blade released short of the detent gives no lamp (all tests). Doing nothing scores 0 (Isaac too); the left arm alone scores 0; the right arm alone scores 1.0 (Isaac, step 533), because the left only steadies the handle.
- **Real station (instruments):** the lamp seen lit at the blade tip; a protractor on the blade against the handle (90 degrees within 3, so the blade sits in its detent); a gentle pull straight back on the blade (it stays hooked); a ruler for the 3 cm hands-clear distance and a phone video for the 1 s of stillness. A torque gauge on the click gives the real detent torque.

## Reset

Lay the handle and the blade on the tray as in Setup, blade in line with the handle, 45 mm short of the bar, nothing held, lamp out.

## Fidelity notes

Full text of the task.json fields, shortened there to meet the length limits:

Fidelity: Declared effects: the hinge (once the bar is in the slot the blade's pose is written to turn only about it) and the detent (at 87 deg+ the blade is held at 90 deg and the lamp lights, shown by a sliding marker). Detent torque (real click not measured; estimate 0.1-0.3 N m), swing reaction and hinge friction are not modelled. Bar diameter 4.505 mm is from ISO 7376 Fig. 2; slot, lead-in, handle, tray and click angle are estimates. The visible blade is drawn straight (a tapered channel with a flange); a real Macintosh 3 blade is curved along its length, while its hook-on base, the part that is scored, is the same. The task ends at the light check: folding the blade closed again afterwards, as a check usually ends, is not part of it.

## Status

Dream-1 score: 0.0 (1600 steps, hit the step limit). Demo: 1.0 at step 542 (27.1 s). Media M-2WSUQ6.
