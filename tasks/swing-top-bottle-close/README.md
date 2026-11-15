# Close a swing-top bottle: stopper on, wire bail snapped down over centre

A bimanual hold-and-close task. An open swing-top (flip-top) glass bottle stands free on the table: its wire bail
lever hangs down along the bottle's right side and the porcelain stopper dangles from its wire link beside the
shoulder.
- The left hand holds the bottle while the stopper is carried and set on the mouth.
- The right hand puts the stopper on the mouth (the lever swings up with it through the link), then pushes the lever
  down: its crank swings under the pivot, stretching the link and the gasket, past centre, and the closure snaps onto
  its stop and stays shut.
- Both hands let go: the stopper stays pressed on the mouth, the bottle upright where it stood.

The skill is an over-centre wire closure on a light bottle (300 g, 65 mm across) that tips over if it is moved while
the stopper is dragged over the mouth. The clamp on the gasket is the carbonated contents' pressure, so the closure
must hold about 100 N.

| File | What it is |
|---|---|
| `swing_top_bottle.py` | The `yam_arena` task file: scene, the open start, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/swing_top_bottle.usda` | The bottle (OpenUSD), one articulation, free on the table: glass bottle, bail lever, wire link, stopper |
| `test_swing_top_bottle.py` | Offline checks: the file loads, every scoring outcome, the 1 s full-success timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

**Swing-top bottle** (`assets/swing_top_bottle.usda`): created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up. Sizes are those of
common 330-500 ml swing-top beer and lemonade bottles (65-75 mm across, a 26-30 mm mouth bead, 200-400 g of glass)
with porcelain stoppers 25-30 mm across on rubber gaskets; the wire is thickened from the usual 2-3 mm to 6 mm.

- **Bottle** (the articulation's root, free on the table, 218 g): green glass, 190 mm tall, 65 mm across; the mouth's
  bead is 30 mm across with a 16 mm bore and a 3 mm x 45 deg lead-in. A steel wire yoke round the neck, 162 mm up,
  carries the lever's two pivot ears. A cream paper label (no markings).
- **Lever** (bail, 35 g): two 6 mm steel arms, 70 mm from the pivot to a red plastic thumb paddle (20 x 58 x 12 mm;
  its two ends along the arm, 20 mm apart, are the grip faces). A 12 mm crank on each arm, leading it by 55 deg,
  carries the link's hooks. Stop at 15 deg past centre.
- **Link** (20 g): a 6 mm wire loop, 50.7 mm from the cranks to a crossbar through the stopper's head. Its give (the
  gasket compressing and the wire stretching) is one prismatic spring along it: 100 N/mm, no shortening, up to 8 mm.
- **Stopper** (22 g): white porcelain head 28 mm across with a red rubber gasket, a 13 mm plug 6 mm long (1.5 mm a
  side in the bore, 2 mm nose chamfer) and a 16 mm knob on top. It turns on the crossbar, which runs through the
  hole across its head with no sideways play: the stopper hangs upright and seats only where the mouth puts it.
- **The closure:** with the stopper on the mouth the lever is at 0 deg and the link is just taut. Pushing the lever
  down 30.8 deg brings the crank straight under the pivot (centre): the link is then stretched 1.32 mm. The lever's
  stop is 15 deg further (45.8 deg), where the link is stretched 1.00 mm: a **100 N clamp** on the gasket.
- **Physics:** one articulation rooted at the glass body, self-collisions on, nothing fixed to the world; the task
  file gives it 128 solver position iterations, which the stiff seal spring needs to stretch as designed while the
  stopper sits on the mouth. Colliders are convex hulls of 64 points or fewer: the body, the shoulder (two pieces),
  the neck, the mouth's bead (12 pieces round the bore), the paddle, the lever's arms and cranks, the link's legs, the
  stopper's head, plug and knob. The stopper seats on the mouth by contact. Frictions (static/dynamic): glass 0.5/0.4,
  steel 0.45/0.35, porcelain 0.6/0.5, rubber 0.9/0.8, paddle 0.9/0.8; restitution 0.

**Closure force, derived (no bottle was measured).** The clamp must exceed the pressure the contents push the stopper
up with. Bottle-conditioned beer and lemonade are carbonated to about 2.5 to 3 volumes of CO2 at 20 C, which is about
2.5 to 3 bar gauge (standard carbonation tables). Over the 16 mm bore (pi x 8 mm squared, 201 mm2) that is
2.5 bar x 201 mm2 = 50 N, and 3 bar gives 60 N. The closed clamp is set at 100 N, about twice that load. The gasket
stiffness then follows: 100 N/mm, which gives 100 N at the closed stop's 1.0 mm of gasket compression (a 3 mm
rubber gasket compressed about a third). The paddle push follows from the linkage geometry: the paddle force is the
link's tension times the crank's arm across the link, divided by the 70 mm lever. With the same geometry the peak
paddle push is about 3.5 N, at about 13 deg from the seated position: the lever ratio is at most 12 mm / 70 mm, so
even a 100 N clamp needs under 17 N at the paddle. The YAM fingers can push that without difficulty.

**The open start.** Isaac Lab starts an articulation's joints at 0 (here: the stopper on the mouth, the link just
taut). A reset event in the task file opens it: the lever down on its stop along the bottle's right side, the stopper
hanging from the link beside the shoulder. The lever and the stopper's hinge have damping only, so their drive targets
don't matter; the link's spring has a 0 target (unstretched), which is what Isaac Lab sets.

## Run it

On the YAM Isaac Lab-Arena station, from this folder, with the Arena environment active. The instruction is
`task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=swing_top_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/swingtop-dream

# Scripted demo (both arms), 680 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=swing_top_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 680 --store-frames --log-dir ~/yam-eval/swingtop-demo
```

The job log prints a `[swing_top_bottle] t=...  steps done: ...` line when a step is done or undone, plus a trace
every second: the bottle's tilt and how far it has moved, whether it is held, the stopper's offset from seated, the
lever's angle and the link's stretch.

**The demo.** Open loop: a fixed waypoint script that never reads a simulated pose. It reaches 1.0 at step 587
(29.9 s): the bottle is held at 4.8 s, the stopper is on the mouth at 14.3 s, the lever is past centre at 23.4 s (the
link stretched 1.3 mm at centre and 1.1 mm on the stop), and both hands are clear at 28.9 s (step 567), when every
success condition first holds; 1 s later the run succeeds. A trial takes about 8 minutes on an RTX A4000 or 4 on an
L40S (680 steps), including startup.
- Left arm: from the bottle's left, pointing across it 15 deg below horizontal, closes on the body's front and back
  70 mm up and holds the bottle while the stopper is carried and set; then lets go and goes home.
- Right arm: leaning 45 deg forward, comes down on the hanging stopper's knob, the fingers closing across it (they square
  the stopper up); lifts it up beside the neck, over the mouth and down onto it, the lever swinging up with it; lets go
  and lifts away. It then takes the paddle between its fingers, pointing at the paddle's face (leaning 45 deg), and
  pushes the lever down along its arc onto its stop, turning with it; lets go, backs off and goes home.
- Measured offsets the demo compensates: the right wrist gives under the lever's pull while it carries the stopper,
  so the knob is aimed 14 mm above the carry path (`CARRY_AIM`), enough for the stopper's head to pass over the mouth's
  bead. The physics is deterministic, so this is a fixed number.

## Instruction

It says the goal in one sentence: the stopper on the mouth and the lever snapped down. It leaves out where things are
and how to move.

## Scoring

Four steps, **0.2 each**, judged as things stand at the end, each only with the bottle upright (within 5 deg) and within
2 cm of where it stood.

| Step | Done when |
|---|---|
| 1. Bottle held | a gripper has held the bottle by its glass (its pads through the glass, both fingers pressing) |
| 2. Stopper on the mouth | the stopper sits on the mouth: its hinge within 2.5 mm of the bore's axis and within -2/+1.5 mm of its seated height, its axis within 6 deg of the bottle's |
| 3. Lever pushed down past centre | with step 2, the lever is at least 5 deg past centre and the link stretched at least 0.5 mm (the stopper pulled down on the mouth, half the 100 N clamp at the stop) |
| 4. Let go | with step 3, no gripper is within 3 cm of the bottle or its closure |

- **1.0 (success, ends the run):** every full-success condition has held together, without a break, for 1 s: all four
  steps (so both hands have let go and are clear), the bottle never knocked over and on the table, and the bottle,
  the lever and the stopper each within 2 mm and 2 deg of where they were when that second began. The arms' return is
  not scored.
- **0.2 per step, at most 0.8:** otherwise, including while that second is being timed.
- The 1 s is timed in the task file itself: every control step, `_Progress` notes when the full-success conditions
  first held together (`met_since`) and the poses then, and any lapse (a step undone, a violation, the bottle, the
  lever or the stopper moved more than 2 mm or turned more than 2 deg from those poses) clears it, so the second
  starts over. `success()` and `score()` return 1.0 only once it is complete; `SUCCESS_HOLD_S` is 0, so the run ends
  on that same step rather than one second later. The tests check that: no 1.0 after 0.95 s and 1.0 at 1 s; a lapse
  (a gripper back within 3 cm, the lever moved 3 deg) restarts the second; and 1.0 is never reached over 3 s while any
  one condition is missing (bottle never held, stopper off the mouth, lever short of centre or never crossed, either
  hand still on, a gripper within 3 cm, the bottle moved, leaning, knocked over earlier or off the table, the bottle,
  lever or stopper still moving).
- **0** if the bottle has been knocked over (tilted more than 45 deg) at any time, is off the table, or the state is not
  finite.
- **Guards:** the open lever already rests on the stop, so the lever only counts after it has gone over centre with the
  stopper on; pulling the stopper onto the mouth with the lever left on its stop never crosses centre and scores no
  lever step. Lifting the lever back up past 10 deg short of centre, or the stopper coming off, cancels the crossing.
  Holding the lever or the stopper is not holding the bottle.
- **Why two hands (physics, not a rule):** with the left hand idle, carrying the stopper over drags the bottle and it
  falls over: score 0. The hold is needed while the stopper is carried and set. Once the stopper is on the mouth, the
  lever push is a force inside the bottle, lever, link and stopper, and the bottle does not need to be held for it:
  the left hand may let go before the push (a simulated run scores 1.0 that way).
- **Real station:** the closure is checked after the run by turning the bottle over a tray: a closed stopper holds
  water. The bottle is empty during the run.

## Reset

Pull the lever up over centre (the stopper lifts off), let the lever down onto its stop along the bottle's side and
let the stopper hang beside the shoulder. Stand the bottle back on its spot, the lever toward the right arm.

## Status

Dream-1 score: 0.0 (both arms stayed folded near their bases and never reached the bottle). Demo: 1.0 at step 587.
Media: M-4TXNGK.
