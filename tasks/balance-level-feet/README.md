# Level an analytical balance on its two thumb-wheel feet until the bubble is centred

A GLP bench task. A Shimadzu AUX220 analytical balance (220 g x 0.1 mg, 217 x 356 x 338 mm, 7.0 kg) has just been moved on the lab bench and stands tilted: its two front level screws are at different heights, the single fixed centre-rear foot is the third support, and the circular bubble at the front is 3.2 mm off centre. The robot has to turn each large knurled front wheel, in small strokes, until the body is level in both axes and the bubble sits in the red circle, then let go and leave it still. The instruction states only the goal. That each wheel moves both axes, that the left foot is 3.9 mm too long and the right 2.2 mm, and that the gripper has to let go before the 1 s hold are the robot's to work out from the bubble and the body.

**What is new against the board.** #621 merged weighs a sample on a balance (a different step); #1969 merged shims a door; #2202 open shims a sleeper. No board task levels an instrument. The mechanism here is a real two-axis screw-jack: two front thumb wheels, one fixed rear foot and a free 7 kg body. Each wheel is a screw jack; the body tilts in both axes as either foot moves; the state is the body's real pitch and roll, not a scripted animation. A different colour, size or start is not the difference.

| File | What it is |
|---|---|
| `balance_level_feet.py` | The `yam_arena` task file: scene, the free three-foot body, the declared screw and bubble, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/balance_aux220.usda` | The AUX220: base, key panel and LCD, glass draft shield, 80 mm pan, circular level, rear foot, and the two front feet on slide joints with their knurled wheels on revolute joints (free articulation root `Base`) |
| `assets/level_bubble.usda` | The 7 mm green bubble (gravity off, no collision; a visual indicator moved by the task) |
| `build_assets.py` | Writes the two USD files (pxr; rounded prisms and 96-sided lathes; colliders are primitives or convex hulls) |
| `test_balance_level_feet.py` | Offline checks (16): every scoring outcome, idle, one foot only, the 0.5/0.1 degree windows, the bubble, the 1 s hold and its restart, over-tighten/off-thread damage, nudged starts, both arms' reach at the nominal and nudged wheel poses, the plan and its timing, the asset's joints |
| `task.json` | The task's details for the board |
| `REFERENCE.md` | The real product, its 5-10 recognisable features, and every measured vs estimated number |

## Real product and values

The **Shimadzu AUX220** is a 220 g x 0.1 mg analytical balance, 217 W x 356 D x 338 H mm, 7.0 kg, with an 80 mm stainless pan (Shimadzu AU series specification page). The instruction manual (AUW/AUX series, section 4.3) says: "There are two level screws on the front of this balance. The screws grow longer when turned in clockwise direction from above and they grow shorter when turned counter-clockwise. Adjust the level screws until the bubble in the level indicator is in the middle of the red circle." The third support is a single fixed foot at the centre rear (manual drawing; the WIND UW/UX manual describes the same three-point arrangement explicitly). The level screws are the "large level screws" of the brochure; the spare part is Shimadzu 321-62884, but its thread and size are not published.

**Measured (published):** 220 g / 0.1 mg, 217 x 356 x 338 mm, 7.0 kg, 80 mm pan, two front level screws and one fixed centre-rear foot, level indicator at the front, clockwise (from above) extends a screw.

**Estimated (each one is in `REFERENCE.md` and `task.json`'s fidelity):** base 60 mm, draft-shield assembly 236 mm (226 mm glass + 10 mm floor), foot extension 40 mm, front feet at x -168, y +-85 mm, rear foot at x +150 mm, level wheel 34 mm dia x 26 mm, thread 1.75 mm/rev, 22 mm travel, about 0.05 N m low torque, body COM 95 mm, body/foot/wheel masses 6.70/0.12/0.03 kg, 30 mm vial with 4 mm/deg and a 0.5 mm centre. The 0.05 N m is the low real torque named in the brief; the 1.75 mm pitch is an estimate for the large screw. The task does not reduce the mass or the level window.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0. The balance base is centred at x 0.528, y 0, 40 mm above the table when level. The body is 217 x 356 x 298 mm on 40 mm feet (338 mm overall). The two front level wheels are at x 0.360, y +-0.085; the wheels are 34 mm in diameter and 26 mm tall, their centres at z 0.7715 m when the feet are at 40 mm. The fixed rear foot is at x 0.678, y 0. The circular level is at the body's front right (local -160, -73 mm; 0.0784 m above the base underside). The initial front extensions are 43.90 mm (left) and 42.20 mm (right) against the 40.00 mm rear foot, so the body is pitched 0.55 degrees nose-up and rolled 0.55 degrees left-up; the bubble is 3.2 mm from the vial's centre. The 7.0 kg body is free on the three feet.

## Mechanism

The balance is one articulation with a free root body (`Base`). The rear foot is part of `Base`; the two front feet are `FootL` and `FootR` on vertical prismatic joints, and each carries `WheelL`/`WheelR` on a revolute joint. The base, front panel and shield floor are colliders; the glass is visual. The two front pads are 12 mm rubber spheres; the rear foot is a 15 mm sphere. The body settles on its three contacts and is free to pitch and roll.

The screw is declared, not modelled in contact. While a gripper holds a wheel (both pads on the knurled rim, opening 34 mm, about 20 N each), the wheel's angle follows the hand's yaw; the wheel's foot slide follows it at **1.75 mm a revolution** (EST). The body's tilt follows from the three support heights, not from a scripted pose. A wheel turned counter-clockwise (seen from above) lowers its foot; a wheel turned clockwise raises it. The rear foot is fixed at 40.00 mm, so the level state is both front extensions at 40.00 mm.

The bubble is a separate gravity-free visual body. Its position on the vial is computed from the body's real tilt at 4 mm of travel per degree (EST for a 30 mm circular level, 30 arcmin per 2 mm). The score never reads the bubble: it reads the body's pitch and roll from its root quaternion. The bubble is the real-world check the operator uses.

## Demo

Open loop, known geometry. The left arm approaches the left wheel at yaw 0, turns the open pads to -80 degrees at the wheel, settles and pinches the 34 mm wheel, then turns it in six 120-degree arcs and an 82.3-degree trim (802.3 degrees), lowering the left foot from 43.90 to 40.00 mm. Between arcs the pads ease off to 46 mm (6 mm clear each side), the wrist resets at the wheel and the pads pinch again; a 12 mm draw-back/re-approach re-centres the TCP before each pinch. The right arm waits at READY, then turns the right wheel in five 90-degree arcs (450 degrees; the 2.6-degree remainder leaves a 0.013 mm error) at yaw -40 to +50. Both go home, and the task's `success()` times the 1 s hold. The wheels' total turns are `(43.90 - 40.00)/1.75 = 802.3` and `(42.20 - 40.00)/1.75 = 452.6` degrees.

Demo: **1.0 at step 1,946 (97.3 s after the settle) on Isaac, twice, and 1.0 at 1,946 in the film run; MuJoCo 1.0 at 1,947 twice.** Final pitch -0.02 degrees, roll +0.06 degrees, bubble 0.23 mm, left/right feet 40.00/39.84 mm. The left-only intermediate state is pitch +0.20 degrees, roll -0.73 degrees: one axis out, score **0.25**. The wheel angles at the end are +764.9 and +450.7 degrees (the grip gate records the hand yaw, so a few degrees per arc are lost; the level window absorbs it).

## Scoring

| Step | Done when | Points |
|---|---|---|
| one axis | one of pitch/roll within 0.5 degrees | 0.25 |
| both axes | both within 0.5 degrees | 0.5 |
| level | both within 0.1 degrees and the bubble centre within 0.5 mm | 0.75 |
| full | the level state held for 1 s with both grippers clear (no pad within 10 cm of a wheel or 3 cm of the body, no grip force) and the body within 2 mm / 0.5° of where the second began | **1.0** |

- **0** otherwise, including doing nothing. The initial 0.55/0.55 degrees and 3.2 mm bubble is 0; holding still at the start does not score.
- **Damage latch:** a front foot below 30 mm is off its thread; above 52 mm it is over-tightened. Either latches damage, caps the score at 0.5 and makes 1.0 impossible. A foot off the bench at the scored state is 0 (the level state requires all three feet on the bench); the body over 5 degrees or the balance off the table is 0.
- The level state is per-axis 0.1 degrees and the bubble is 0.5 mm; at 0.1 degrees on both axes the bubble is 0.57 mm, so the bubble is the slightly tighter real check. The ladder is 0/0.25/0.5/0.75/1.0.
- **Real station:** the bubble by eye; both axes with a digital inclinometer on the pan (0.01 degree); the balance's own bubble remains the visual check; each foot's extension from its thread turns (1.75 mm a revolution, EST) or a caliper; the wheel torque with a small torque gauge (about 0.05 N m, low). The video shows the 1 s hold with both hands clear.

## Checks

```bash
cd benchmark-submission/tasks/balance-level-feet
PYTHONPATH= ../../../local_sim/.venv/bin/python -m pytest -q test_balance_level_feet.py
cd ../../../local_sim
PYTHONPATH= .venv/bin/python -m mjkit.run ../benchmark-submission/tasks/balance-level-feet/balance_level_feet.py --steps 2400 --trace
```

- pytest: **16 passed**.
- Demo: **1.0 at step 1,947 in MuJoCo (twice)**; the left-only state 0.25; idle 0; over-tightened foot 0.5; off-thread foot 0.5; the 0.5-degree window 0.5; the level window 0.75; the 1 s hold 1.0; nudge starts 0 or 0.25 at the start and 0.75 when level.
- Both arms reach the wheel at the nominal and +-6 mm / +-4 mm nudged poses across the plan's yaw range (left -80..+40, right -40..+50 and -45/0/+45), within 4 mm and 4 degrees.
- **Run limit: 2,400 steps (120 s at 20 Hz).** The demo needs about 1,947 steps (97.4 s), plus the 3 s policy hold.
- **Isaac (2026-11-16):** the original 90-degree strokes at yaw -45..+45 drove the wrist joint to its +-120-degree limit, and after 2-4 strokes the arm could not re-find the wheel (0.25). The demo now uses 120-degree left arcs (-80..+40) and 90-degree right arcs (-40..+50), both inside the reliable yaw range, and resets the wrist in place with the pads eased off to 46 mm and a 12 mm draw-back instead of the 45 mm retreat. Isaac demo **1.0 at step 1,946 twice** and **1.0 at 1,946 in the film run**. The 0.5/0.1-degree level window, the 1.75 mm/rev pitch, the 0.05 N m torque and the 30-52 mm damage limits are unchanged.
- **Pass-through check:** frames at 10/45/80/100 % and the left-wheel reach; no gripper, pad or forearm intersects the balance body or the bench. The right-wheel reach is behind the right forearm in the front close-up; the top view shows the gripper under the base edge, outside the body, and the pads' TCP stayed 0.9-14 mm from the wheel. No cutaways.
- **Media:** `benchmark-media-balance-level-feet/demo.mp4` (top view only, 640 x 360, 97.8 s + a 3 s held last frame, 1.36 MB), `demo_top.gif` (1.77 MB), `setup.png` (0.41 MB), `dream.mp4` (0.53 MB); `demo_review.mp4` is internal only.

## Reset

Set the left front foot to 43.90 mm and the right to 42.20 mm, both wheel angles 0, the rear foot 40.00 mm. Place the base at x 0.528, y 0, 44 mm above the table, level, and let it settle onto the three feet; the body tilts to 0.55/0.55 degrees and the bubble is 3.2 mm off centre. Put the bubble on the vial's glass and the arms at READY. Wipe the feet and the bench before the trial.

## Status

Dream-1: **0.0** (2,400 steps, hit the step limit; `benchmark-media-balance-level-feet/dream.mp4`). MuJoCo demo: 1.0 at step 1,947, twice; pytest: 16 passed. Isaac demo: 1.0 at step 1,946 twice and 1.0 at 1,946 in the film run. Media in `benchmark-media-balance-level-feet` (top view only for the PR); `demo_review.mp4` is internal only; nothing uploaded.
