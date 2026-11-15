# Fill two blood tubes from a needle holder in a training arm without moving the needle

A bimanual phlebotomy task. An IV/venipuncture training arm lies across the table, palm up, its hand on the robot's
right. A 21 G multi-sample needle is already in its vein at the elbow crease, on a single-use tube holder that rises
20 degrees off the skin, its open end toward the hand. Two empty 13 x 100 mm evacuated tubes (clear plastic) lie on a
tube tray on the right.

- Each tube is taken from the tray, its stopper end put into the holder and pushed onto the needle's back end until
  the needle is through the stopper; it is kept on while it fills (6 mL in 8.6 s, at the draw rate measured with 21 G
  needles), pulled off and laid back on the tray. The blood is shown: a dark-red column rises in the tube from its
  stopper while the needle is through it, and stays when the tube comes off.
- The holder must not move: the needle pushed in or pulled out 5 mm, or levered 5 degrees, would go through or out of
  the vein, and the run scores 0.

Why both hands: pushing the tube on folds back the needle's rubber sleeve, a spring of 1.2 to 2.4 N that pushes the
tube back off the needle unless it is held on (a known effect: closures are designed against "push-off" from the
holder, see Sources). The needle only slides and levers in the tissue, so the push, and the hold for the whole fill,
drive the holder and the needle deeper unless the other hand holds the holder steady. In a trial with the holding hand
idle, the first push drove the needle 12 mm into the arm (score 0). A tube let go once it is on is pushed back off the
needle and does not fill (in a trial). A robot arm is soft at the wrist (about 4 mm per newton here), so the holding
hand only keeps the needle still if it rests on the patient's arm, as a phlebotomist's hand does.

| File | What it is |
|---|---|
| `vacutainer_tube_push.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/training_arm.usda` | The IV training arm (a fixture): silicone skin, the vein marked, convex-hull colliders |
| `assets/needle_holder.usda` | The needle holder on its needle: an articulation fixed at the skin entry point (tissue pivot and slide, the needle's sprung rubber sleeve) |
| `assets/blood_tube.usda` | The 13 x 100 mm clear tube with its rubber stopper, one rigid body (used twice) |
| `assets/blood_column_48.usda`, `_24`, `_12`, `_06` | Pieces of the blood column shown in a tube as it fills (48, 24, 12 and 6 mm long): visual only, no collider, no gravity |
| `assets/tube_tray.usda` | The tube tray with V-rests (a fixture) |
| `test_vacutainer_tube_push.py` | Offline checks: the file loads, every scoring outcome, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All of them were created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/),
built by a headless Blender script (not part of this folder) that writes the USD with its physics. Units are metres
and kilograms, Z up. Every part is at its real size.

- **Needle holder** (`needle_holder.usda`, 6.5 g): a universal single-use holder for 13 and 16 mm tubes, clear
  polypropylene, 52.3 mm long, body 20 mm across, round base flange 30 mm across (the dimensions of the Globe
  Scientific 1201/1202 holder's data sheet); bore 17 mm with a 1.5 mm wall and a 2 mm lead-in at its mouth (typical: it
  takes 16 mm tubes). On its nose a green 21 G multi-sample needle hub; the needle's patient end runs 20 mm to the skin
  and on into the vein; its back end, 0.82 mm and 16 mm long, points back into the bore under a rubber sleeve
  (2.4 mm). A tube is fully on with its stopper 44 mm from the skin entry point, against the hub's rear boss.
  - The needle in the tissue: a pivot at the entry point (the tissue resists levering it: 2 N m/rad) and a slide along
    the needle (tissue drag 30 N s/m and a 100 N/m spring), as typical values for a needle in a silicone training arm.
  - The sleeve: a sprung slide, 8 mm of travel, 1.2 N where it is free and 2.4 N folded (typical).
- **Blood tube** (`blood_tube.usda`, 7 g empty): 13 x 100 mm clear PET evacuated tube (0.9 mm wall, 11.2 mm bore),
  6 mL draw, red conventional rubber stopper, a narrow paper label with a red band (no brand, 90 degrees of its round)
  on the side that lies down in the tray; the rest of the tube is clear. The stopper's top has an 8 mm well, its floor a
  45-degree cone down to a 2.4 mm diaphragm 7.1 mm under the top; its plug reaches 13 mm into the tube.
- **Blood column** (`blood_column_*.usda`): dark-red cylinders 11.2 mm across (the tube's whole bore), 48, 24,
  12 and 6 mm long, visual only (no collider, no gravity; their 1 g mass is a placeholder: nothing touches them). Each
  tube has one 48, 24 and 12 mm piece and two 6 mm pieces; they wait under the table, out of sight.
- **Training arm** (`training_arm.usda`, fixed): an adult IV/venipuncture arm about 575 mm long, upper arm to fingertips,
  about 100 mm across and 75 mm high at the elbow crease (the size of the 3B Scientific P51 IV arm trainer, 22.6 x 5.1
  x 4.1 in), silicone skin (friction 1.0/0.9), the median cubital vein marked in blue.
- **Tube tray** (`tube_tray.usda`, fixed): blue polypropylene, 140 x 160 mm with 12 mm walls, three lanes 40 mm apart,
  each with two V-rests that hold a 13 mm tube level, 25 mm up; a tube's closed end overhangs the rests (typical).
- **Physics:** colliders are boxes, cylinders, a capsule (the needle), a sphere and small convex hulls (the stopper,
  the holder's flange, the arm), none over 28 points. Frictions (static/dynamic): PET 0.4/0.35, rubber 0.8/0.7,
  polypropylene 0.4/0.3, steel 0.4/0.3, silicone skin 1.0/0.9; restitution 0. The holder's articulation is solved
  with 128 position iterations (`SOLVER_ITERATIONS`; PhysX's default is 4, the arms use 16): with 32, the 0.3 g
  sleeve, pushed by its 1.2 to 2 N spring, sank 3 mm into the stopper's diaphragm hole and caught there, so pulling the
  tube off dragged the needle out of the arm. With 128 it rests on the stopper's cone, where the geometry puts it.

## The drawn blood (declared task code)

Nothing in the simulator flows. The task shows the blood the real tube would draw:
- While a tube is pierced (the condition scored as "on the needle", below), blood is drawn at 0.70 mL/s, the average
  rate measured for 5 mL evacuated tubes on 21 G needles (5 mL in 7.15 s; Fujii 2013, see Sources), until the tube
  holds its 6 mL draw (8.6 s). A tube pulled off early keeps what it drew (its stopper reseals and its vacuum keeps
  the rest); pushed on again it draws the rest, but it only counts as filled after 8.6 s on the needle without a
  break.
- The blood is a dark-red column in the tube's bore from the stopper's plug, 10.2 mm long per mL (61 mm for 6 mL),
  made of the tube's pieces stacked from the stopper (overlapping; a column under 6 mm long has the rest of its piece
  hidden in the plug). A step event (every control step, after the physics) moves each piece to its place in the tube
  with the tube's velocity there, so it rides along with the tube until the next step; pieces not needed wait under the
  table. The column stays along the tube's axis from the stopper (it does not slosh).
- The blood's mass (6.3 g for 6 mL) is not added to the tube; the pieces touch nothing and move nothing.
- The tube is clear PET with a narrow label on its underside, and the holder is clear polypropylene, so the column
  shows. The top camera sees it once the tubes are back in the tray (two red bars, about 11 x 2 pixels at 320 x 176),
  not while the tube is on the needle (both arms are over the holder then); the demo video adds two filming-only
  close-ups (the tube on the needle, the tray) with each tube's fill.

## Run it

On the YAM Isaac Lab-Arena station, from this folder, with the Arena environment active. The instruction is
`task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=vacutainer_tube_push.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/vacutainer-dream

# Scripted demo (both arms), 1,450 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=vacutainer_tube_push.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1450 --store-frames --log-dir ~/yam-eval/vacutainer-demo
```

The job log prints a `[vacutainer_tube_push] t=...  steps done: ...` line when a step is done or undone, plus a trace
every second: how far the needle has been pushed in or levered and how far its sleeve is folded back, and for each
tube how far its stopper is along the holder, its offset and tilt from the needle's axis, and whether it is pierced,
filled (and how much blood it has drawn) and in the tray.

**The demo.** Open loop: a fixed waypoint script that never reads a simulated pose. It reaches 1.0 at step 1316
(66.3 s in the task's log): tube 1 is filled at 21.6 s and back in the tray at 31.4 s, tube 2 is filled at 52.7 s and
back at 62.6 s (all four steps), then the left hand lets go of the holder and lifts off, and every success condition
has held for 1 s at 66.3 s. A trial takes about 12 minutes on an L40S (1,450 steps), including startup.
- Left arm: comes down beside the holder from above, leaned 55 degrees toward the hand so its wrist stays over the
  upper arm, clear of the right arm; rests its open fingertips on the skin either side of the holder, pressing lightly,
  closes on the holder's body and keeps it there until both tubes are back; lets go, lifts off, goes home.
- Right arm, for each tube: from above, closes across the tube 30 mm from its closed end, lifts it off the rests, turns
  it in line with the holder (pointing in from the near side, 50 degrees forward), slides the stopper end into the
  bore up to the needle, pushes it fully on and holds it there for 9.2 s while it fills, pulls it off, lays it back
  on its rests and lets go. The push and the hold are commanded as the pose that presses with 4 N and 3 N along the needle, worked out
  from the arm's joint stiffness (the arm gives under load and its wrist would tilt the tube); the script reads no
  simulator state.
- The needle moves at most about 0.5 mm in the tissue during the demo (0.53 mm out and 0.27 degrees on the station).
- Measured offsets the demo compensates: the right hand holds the tube with its axis on the TCP (`TUBE_IN_PADS` 0);
  held 3 mm out toward the fingertips, the tube slid 5 mm in the pads under the push and the fingers closed round
  its far side. The physics is deterministic, so these are fixed numbers.

## Instruction

It states the goal (fill both tubes from the needle that is already in the arm, without moving the needle, and lay
them back in the tray) and leaves out which hand does what and how to keep the needle still.

## Scoring

Four steps, **0.2 each**. The fill steps count once reached; the tray steps as things stand.

| Step | Done when |
|---|---|
| 1. A tube filled | a tube was on the needle (its axis within 1 mm and 6 degrees of the needle's) with its stopper's face at most 46 mm from the skin entry point (the needle's tip through the diaphragm) for 8.6 s without a break: its 6 mL drawn (the red column full) |
| 2. A filled tube back in the tray | a filled tube lies on the tray (its middle inside the walls, level within 15 degrees, resting on the rests), off the needle, with no gripper's pads within 3 cm of it |
| 3. Both tubes filled | as step 1, for both tubes |
| 4. Both filled tubes back in the tray | as step 2, for both tubes |

- **1.0 (success):** all four steps, no gripper's pads within 3 cm of either tube or of the holder, no violation, and
  the tubes and the holder unchanged (within 2 mm and 2 degrees), all held without a break for 1 s, timed in the task
  file from the first control step they all hold (any lapse starts the second again; `SUCCESS_HOLD_S` is 0). All four
  steps without that score 0.8.
- **0** if at any time the needle was pushed into or pulled out of the arm more than 5 mm, or levered more than 5
  degrees, if a tube falls off the table, or if the state is not finite.
- **Guards:** a tube pushed on and pulled off before 8.6 s does not fill (a break starts the 8.6 s again; the tube
  keeps the blood it drew, shown part-full); a tube on the
  needle but not through the diaphragm, or off the needle's axis, does not fill; an unfilled tube laid in the tray
  scores nothing; a tube held over the tray, standing, or left beside it is not back; the same tube filled twice is not
  both tubes.
- **Real station:** the training arm's simulated-blood supply fills each tube, checked on a photo; a pen mark on the
  holder's body against a ruler taped on the skin shows how far the needle moved (video); the tubes on the tray and the
  release on a photo and the video.
- **Safety (real station):** only the training arm, never a person; the needle stays fixed in the training arm and is
  removed and discarded into a sharps container by staff after the run; the back needle is under its sleeve inside the
  holder.

## Reset

Lay each tube back on its lane's rests (lanes 1 and 2 from the robot's left), its stopper toward the training arm;
pull the needle out of the training arm and put it back in the same puncture, 20 degrees up, the holder's open end
toward the hand, the sleeve unfolded; empty the tubes (or use fresh ones).

## Sources

- Holder: Globe Scientific, Disposable Needle Holder, Item 1201/1202, technical data sheet (body 20 mm, base 30 mm,
  height 52.3 mm, polypropylene; fits 13 and 16 mm tubes).
- Tubes: 13 x 100 mm evacuated blood-collection tubes (for example BD Vacutainer Plus serum tubes, 13 x 100 mm,
  6.0 mL).
- Training arm: 3B Scientific P51 IV injection arm trainer, 22.6 x 5.1 x 4.1 in.
- Needle angle: WHO guidelines on drawing blood (2010): enter the vein at an angle of 30 degrees or less.
- Sleeve push-back: US patent 5,494,170 (combination stopper-shield closure): tabs on a closure are there "to prevent
  push-off of the closure from the needle holder that may be caused by the force of a needle sleeve".
- Draw rate: C. Fujii, "Vacuum-venipuncture skills: time required and importance of tube order", Vascular Health and
  Risk Management 9:457-464 (2013): with straight 21 G needles, 5 mL tubes filled in 7.15 s (SD 1.80 s) on average.
- Occupations: O*NET 31-9092.00 Medical Assistants and 31-9097.00 Phlebotomists; BLS OEWS May 2023.

## Status

Dream-1 score: 0.0 (both arms stayed near their bases; the right gripper moved over the tray's near edge but never
moved a tube, and neither touched the holder). Demo: 1.0 at step 1316. Media: M-EEWGSZ.
