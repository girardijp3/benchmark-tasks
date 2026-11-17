# Light a dinner candle with a safety match struck on its box, then put the match in the ashtray

A household task most adults have done. A safety-match box lies flat on the table, one safety match lies loose beside it, and a 25 cm dinner candle stands in a low brass candle holder next to a ceramic ashtray. The robot has to get the candle alight with the match and tidy up: the match is struck on the box's striker strip, its flame is brought to the wick and held there until the wick catches, and the burning match goes in the ashtray before the flame creeps down the stick to the fingers.

The skill is a quick pressed strike of a 2 mm stick against a box that has to be held free in the other hand (the strip is on a long narrow side and the box would be pushed away on the table), then a timed flame transfer. One arm cannot do it (one arm alone scores 0). The score is judged from what the camera sees: a match flame, the flame at the wick, a candle flame that stays lit, and the used match in the ashtray. No force reading on the hand-held box is needed or scored. For real trials the pads carry clip-on stainless/ceramic heat shields (section below); the task stays match-on-gripper, not tool-on-match.

**Why it matters.** Lighting candles with a match is something most adults do at home, and it is also a daily pre-service routine in restaurants and banquet rooms, where every table's candle is lit before the doors open. It is a short, real test of a fast pressed strike on a hand-held box, a flame carried to a small target and a burning item put away safely in time.

| File | What it is |
|---|---|
| `match_light_candle.py` | The `yam_arena` task file: scene, the declared strike / flames / burn-down rules, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/matchbox.usda` | The match box: a free articulation, the box plus its striker panel on a 2 mm spring joint (a compliant contact sensor, not a force threshold) |
| `assets/match.usda` | One safety match (free rigid body) |
| `assets/candle.usda`, `assets/holder.usda`, `assets/ashtray.usda` | The 250 mm candle, the brass candle holder and the ceramic ashtray (free rigid bodies) |
| `assets/flame_match.usda`, `assets/flame_candle.usda` | The two flames (visual only, kinematic; the task moves them) |
| `build_assets.py` | Writes the USD files in `assets/` (pxr; closed 96-sided lathe meshes for the round visible parts, bevelled hulls for the box, convex-hull colliders) |
| `test_match_light_candle.py` | Offline checks: the strike gate, wick, burn-down and fire rules, every scoring outcome, the 1 s timing and both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres, kilograms, Z up. Real product classes (typical sizes; none is a branded catalogue drawing): a Swan Vestas / Bryant & May class safety-match box, a 10 inch taper candle of 7/8 inch (22 mm) base, a low brass candle holder, a round ceramic ashtray.

- **Match box** (12 g with its matches, estimate: box 6 g plus about 50 matches at 0.12 g): 52 x 36 x 16 mm, navy sleeve with a white-and-gold oval label and a red band on both big faces, cream tray end plates, and a speckled brown striker strip (48 x 12 x 0.5 mm, feathered ends) on each long narrow side. The strip on the -y side sits on a panel that moves 2 mm into a 2.5 mm pocket against a 4000 N/m spring: its deflection tells the task that the head is pressed on the strip. The sim requires 0.1 mm of contact, not a force threshold.
- **Match**: 47 mm long, 2.1 mm square splint, 3.2 mm red head, 2 g (real about 0.12 g, see below).
- **Candle**: ivory 22 mm at the base tapering to 19.8 mm over 250 mm, 80 g (estimate: 0.9 g/cm3 wax), a cotton wick 8 mm proud of a melted dish. It stands in the socket with 0.4 mm clearance each side.
- **Candle holder**: polished brass, 91 mm drip dish, a cup 33 mm across with a rolled rim and a bead ring, the socket 22.8 mm across and 25 mm deep with a 3 mm 45 degree lead-in, rim 30 mm up, 0.35 kg, free. It is low because the wick tip is then 263 mm above the table; the arms' working height is about 0.3 m.
- **Ashtray**: white glazed ceramic with a blue band, 104 mm across, 24 mm tall, a bowl 84 mm across, three cigarette rests, 0.20 kg, free.
- **Flames**: a 20 mm match flame and a 28 mm candle flame (an emissive amber teardrop with a blue core and a hot centre), moved by the task.

## Declared effects (and what is stated, not simulated)

Fire is not simulated; the task file's `_fx_step` declares it, gated on what the robot does:

- **Strike.** The head's centre must touch the strip (within 1 mm to 3 mm of the panel face, inside the strip's width and length), the panel pushed in by at least 0.1 mm (a contact gate, not a force threshold), and the head must slide over it at 0.3 m/s or more (its movement relative to the box, from one control step to the next) for 15 mm or more with every step meeting all three. At 20 Hz a step at 0.3 m/s is already 15 mm, so one full step does it. The match is then lit. Anything slower or in the air does nothing; the stroke restarts on any step that fails. A real trial does not measure the press at all: it scores the flame the strike produces.
- **Burn-down.** Lit, the flame front moves from the head toward the base end at 2 mm/s (estimate: the whole stick burns about 23 s). The flame (a vertical teardrop, 3 mm below to 12 mm above the burning point) is drawn at the front by moving a kinematic flame object.
- **Wick.** The wick tip within 5 mm of the flame body for 2 s (continuous) lights it; the candle flame is then drawn at the tip. Step 2 counts 0.5 s of that.
- **Burn.** If the front reaches the near edge of a gripper's pads that are closed on the stick (with the flame body's upward reach included), the fingers are burnt: score capped at 0.5, 1.0 lost.
- **Scorch.** If any point of a pad comes within 4 mm of the match flame body, the score is capped at 0.5. In a real trial the 2.2 mm steel-and-ceramic shield sits in that gap, so this is the simulator's stand-in for "the shield is touching the flame".
- **Touch.** If the match's stick or a gripper's pads touch the candle's wax body below its top, the score is capped at 0.5. Only the match flame may touch the wick; the flame, the stick, the pads and the shields stay off the wax.
- **Fire.** A lit match lying on the table outside the ashtray, or a candle or candle holder tilted more than 15 degrees (or the candle off the table): score 0 for the rest of the run.

Stated simplifications and changes to the real values: the match weighs 2 g, not about 0.12 g (estimate): 0.12 g was tried on Isaac with the final demo and the pads do not pick the stick up (score 0), so 2 g stays (heavier than real, never lighter); the splint's friction on the pads is set to 1.2 (a rubber pad on dry wood is 0.8-1.2; the YAM pads pinch a 2 mm stick with only a few newtons); the head-on-strip friction is 0.2, not the real 0.4 (tried on Isaac with the final demo, see below); the striker panel is a spring-loaded contact sensor at 4000 N/m (real about 3000 N/m, estimate); the burn rate is an estimate; the match lies loose on the table instead of being taken from the box and the box's drawer is not opened; the candle holder is low (30 mm rim, real candle holders are 30-60 mm); the match is pinched 12 mm from its base end, not at the very end (a real hand holds it nearer the end): at 8 and 11 mm the strike drag turned the stick out of the pinch on Isaac, at 12 mm and deeper it held.

**Ignition in the simulator and on video.** The sim gate is a strike motion, not a force reading: the head must be in contact with the strip on two consecutive 50 ms samples and must run at 0.3 m/s or more over it on the second, covering 15 mm or more (a single touching sample is not a strike; a pytest proves it). The 0.3 m/s and 15 mm are estimates for a Swan Vestas class box and match (a hand strikes at roughly 0.3-0.5 m/s), set low enough that a real strike passes. A real trial scores the outcome instead: the match flame seen at the head after the draw, on video. No load cell, kitchen scale or other force measurement is part of the task; the hand-held box never has to be weighed or instrumented.

**Real values against the sim.** Striker panel: real cardboard side with a glued abrasive strip, estimated about 3000 N/m under a finger-size load; the sim panel is 4000 N/m, used only to detect contact. The 2 g match: a contact-solver limit in both simulators, not a physical choice. A 0.12 g stick between finger pads has an effective contact mass so small that the soft contacts (MuJoCo) and the solver's depenetration and friction iteration (PhysX) give a grip that creeps and ejects the stick under side loads; at 2 g the grip holds. Head-on-strip friction: the real value is about 0.4. Tried at 0.4 on Isaac with the final demo (head and strip both 0.4, so PhysX's average is 0.4): the press builds, but the drag pulls the stick out of the weak pinch at the draw and there is no strike (score 0); MuJoCo also needed 0.2. The demo uses 0.2: head 0.2 and strip 0.2 give 0.2 effective (PhysX averages the two); splint 1.2 and pad 0.5 give 0.85 effective (raised from about 0.5-0.6 for a dry splint on rubber, because the pinch is weak). The 2 g match: heavier than the real 0.12 g, so it softens nothing; 0.12 g was tried on Isaac and is not picked up.

## Heat protection for the gripper pads (real trials)

The simulator does not model heat. A real trial fits both grippers with clip-on pad shields; the task stays **match-on-gripper** (no hemostat or other tool changes it). The shield is passive PPE: the pad still pinches the match, and the bare rubber gripping face still touches the 2.1 mm splint.

- **Outer shell:** 0.6 mm (0.024 in) 304 stainless steel sheet, [McMaster-Carr 8983K111](https://www.mcmaster.com/8983K111) (6 x 6 in, rated to 1500 F / 815 C). Cut two 22 x 26 mm blanks per gripper. Fold each into a U: a 22 x 20 mm back plate over the pad's outer face, two 3 mm side tabs, and 1 mm folds over the pad's end face and its head-facing edge; the gripping face stays bare. The folds are flush with the pad's outline, so they do not reach into the gap between the pads or toward the match head.
- **Thermal break:** 1.6 mm (1/16 in) binderless ceramic fiber sheet, [McMaster-Carr 93285K31](https://www.mcmaster.com/93285K31) (2300 F / 1260 C, 6 in wide). Cut a 20 x 24 mm piece under each steel back plate and thin strips under the side tabs. No adhesive on the pad; the U-clip's own spring tension holds the sandwich on.
- **How the pads stay clear:** the gripping face (the side that touches the match) is bare rubber over its full area. The 2.2 mm steel-plus-ceramic sandwich is only on the outer face, sides and edges, so it adds nothing between the pads. The gripper opening command is unchanged; the pads still close on the 2.1 mm splint 12 mm from its base end, and the pinch force and friction on the match are unchanged. The shield weighs about 2.5 g and sits behind the grip.
- **Distances:** the pad tip is about 13 mm from the match's base end, so at ignition the shield is about 32 mm from the burning head. The front moves toward the base at 2 mm/s. In the MuJoCo demo the closest pad point to the flame body is **7.4 mm**, about 6 s after ignition while the match is turned upright between the wick and the ashtray; at the wick it is 16.5 mm and at release about 11.7 mm. The shield's head-facing fold is flush, so the steel is about 7 mm from the flame at the closest point and the rubber is 2.2 mm behind it. In a real trial keep the match tilted at 45 degrees through that transfer instead of copying the demo's upright turn; the shield then stays farther from the flame. If a shield ever looks like it will touch the flame, drop the match into the ashtray early and end the trial.
- **Before a trial:** close the pads on a 2.1 mm drill shank; each shield's head-facing fold must be at least 4 mm clear of the shank and the bare pad faces must touch it. Hang a 50 g weight from the pinched match; it must not slip. Check the ceramic paper is dry and uncracked and the clip is seated.
- **After a trial:** read the rubber under each shield with an IR thermometer (for example a Fluke 62 MAX+) or a K-type thermocouple: **45 C or less**. A 50 C non-reversible temperature label on each pad must be unchanged. Inspect for scorch marks, soot or sticky rubber on the pad and shield. If any check fails, stop, replace the pad and shield, and do not run again until the clearance is restored. The steel shell may be warm; the rubber behind the ceramic is what must stay cool.

## Real-trial fire safety

The flame is declared in the sim; a real trial with a real match must have: a non-flammable tray (ceramic or metal) under the whole workspace and a ceramic ashtray with a little water or sand in it; a fire extinguisher or a bowl of water within reach and a person at hand; the robot never left unattended while a match or candle is lit; the room ventilated and cleared of paper, cloth and spirits; matches kept in the box when not in use; and the run stopped (arms to rest) on any lit match leaving the ashtray or any tipped candle. The heat shields must be fitted and checked as above before the first trial; the match is released into the ashtray within about 8 s of ignition, before the flame reaches the pads; and a trial is stopped if the flame touches a shield, the candle body or anything other than the wick.

## Layout

Box x 0.27, y 0.17 (flat, long side along x); match centre x 0.20, y -0.20 (along y, head toward -y); candle holder x 0.44, y -0.075 (candle seated); ashtray x 0.33, y -0.11 (in front of the candle holder, 0.115 m from it).

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=match_light_candle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 580 --store-frames --log-dir ~/yam-eval/match-light-candle-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=match_light_candle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 580 --store-frames --log-dir ~/yam-eval/match-light-candle-demo
```

The log prints `[match] t=...  STRUCK: ...` with the strip push, speed and stroke at ignition, `the wick has caught`, `RULE BROKEN: ...`, `steps done: ...` on every change and a trace each second.

**Run limit: 580 steps (29 seconds at 20 Hz).** The demo reaches 1.0 at step 431 on Isaac and in MuJoCo (21.55 s); the limit is that plus about 30 percent.

**The demo.** Open loop. The left arm picks the box by its ends (closing along x), lifts it and turns its wrist to hold it on edge with the striker strip facing up, 0.17 m above the table at x 0.30. The right arm picks the match 12 mm from its base end, turns its wrist to hold it upright head down, lowers the head on the strip and presses it by commanding 27 mm below the strip (the arm is soft in a vertical press; the panel push is about 0.6 mm); the left arm then draws the box 44 mm under the head in 0.075 s (0.33-0.34 m/s measured, one or two control steps). The left arm waits clear; the right lifts, carries the flame to the wick, tilts the match 45 degrees so the burnt end hangs clear of the candle, holds the flame beside the wick (2 mm off it) for 2.35 s, moving the match 4 mm along the stick halfway because the burning point creeps up it, backs away straight over the ashtray, tilts the match 45 degrees (head toward the robot) over the bowl so that it falls flat, lets go (about 8 s after ignition) and goes home by a path clear of the candle. The pads stay at least 7.4 mm from the flame body; in a real trial the shields described above are in that gap. The left arm then puts the box back down and goes home.

## Scoring

| Step | Done when |
|---|---|
| 1 (0.25) | a match flame is seen on video after the head is drawn across the striker strip |
| 2 (0.25) | the match flame is held at the wick for 0.5 s (video) |
| 3 (0.25) | the candle flame is established and still burns 2 s after the match is drawn away (video) |
| 4 | the used match lies in the bowl (both ends inside the 42 mm bowl radius, under the rim), the candle is lit, within 4 mm of the candle holder's socket axis, on the socket floor and within 6 degrees of upright, the box lies on the table within 15 degrees, no gripper holds the match or box, no pad is within 3 cm of either, and neither the match stick, a pad nor a shield has touched the candle body, wick or flame (only the match flame may touch the wick, not the wax) |

- **1.0** only when step 4 has held together with step 3 for 1 s, timed inside the task's success(): no rule event, and the five movable parts within 2 mm and 2 degrees of where they were when that second began. A lapse restarts the second.
- **0.25 per step, at most 0.75** otherwise (steps 1 to 3), also while the second is being timed. A burn, a pad within 4 mm of the flame (a scorch) or a touch of the candle body caps it at 0.5 and rules out 1.0.
- **0** after a fire (lit match on the table outside the ashtray, candle or holder tilted over 15 degrees); kept for the whole run.
- Doing nothing scores 0.
- **Real station (instruments):** a phone camera for the top view and a second close-up camera for the head and wick; the match flame at the strike, the flame at the wick, and the candle flame still standing after the match is withdrawn are the three video observations. No force measurement is needed: do not put a kitchen scale under the hand-held box. A stopwatch times the 2 s, the used match is seen inside the ashtray's bowl, a 15 degree check of the candle is made against a square, a ruler gives the 3 cm hands-clear distance, the video gives the 1 s of stillness, and an IR thermometer or thermocouple reads the pad under each heat shield (45 C or less) after the run.

## Checks

- **Isaac (L40, scripted demo):** score 1.0 at step 431 (21.55 s), the same step as in MuJoCo. At the strike: strip push 0.62 mm, speed 0.34 m/s, 17 mm of strip; the wick caught after 2.0 s in the flame (t = 18.05 s); no rule event. No force term is used or scored. Doing nothing scores 0; the left arm alone 0; the right arm alone 0 (no strike). Head friction 0.4 and a 0.12 g match both fail there (see the stated simplifications). The strike needed retuning on Isaac: pinch 12 mm from the base end, press 27 mm, a 0.075 s draw; the ashtray sits where the right arm reaches it (static IK error under 2 mm along the whole demo, tested).
- **MuJoCo (local stand-in):** demo 1.0 at step 431, no rule events; the closest pad-to-flame-body distance is 7.4 mm. Start offsets: match across its length (x) +-3 mm, along its length (y) +2 mm, box x/y +3 mm, candle holder +3 mm x and y, ashtray +-3 mm x and +3 mm y: all 1.0 (steps 431-432; the demo is open loop). Match y -3 mm gives 0.75 (the pinch lands deeper and the flame reaches the pads first: the demo has no slack there). Idle 0; the left arm alone 0; the right arm alone 0.
- `python -m pytest -q test_match_light_candle.py`: 19 passed (the bare-table and moved-ashtray exploit, the two-sample contact gate, the strike gate, wick, burn, scorch, candle touch, fire, ladder, 1 s timing, reach of both arms along the demo).

## Reset

Lay the box flat, lay the match along y with its head toward -y, stand the candle in the candle holder, put the ashtray down. Fit and check the heat shields before the first real trial. Nothing is lit.

## Fidelity notes

Full text of the task.json fields, shortened there to meet the length limits:

Fidelity: Match true size but 2 g (real ~0.12 g; 0.12 g tried on Isaac: the pads do not pick it up; heavier than real softens nothing); splint friction 1.2 (PhysX effective 0.85). Ignition, flames and burn-down (2 mm/s, est.) are declared effects; the sim strike gate is strip contact plus 0.3 m/s over 15 mm, with no force term (real trials score the flame on video). A pad within 4 mm of the flame is a scorch. Strip on a 4000 N/m panel (real ~3000, est.), a contact sensor. Head-strip friction 0.2, not the real ~0.4 (0.4 tried on Isaac: the drag pulls the stick out of the pinch, no strike). The match is pinched 12 mm from its base end (a hand holds it nearer the end; at 8-11 mm the strike drag turned it out of the pinch on Isaac). Candle 250 mm, 80 g (est.) in a low brass candle holder with a 30 mm rim (arm reach; real 30-60 mm). Heat is not simulated: real trials fit 0.6 mm 304 stainless over 1.6 mm ceramic fiber outside the gripping face, pinch unchanged. The match lies loose; the box's matches are not modelled.

## Status

Dream-1 score: 0.0 (580 steps, hit the step limit). Demo: 1.0 at step 431 (21.55 s). Media M-VY4P4W.

Revision after review: real-trial scoring is the ignition and flame seen on video (no force term on the hand-held box), and real trials use the pad heat shields above. The demo trajectory and score are unchanged, so the media is unchanged.
