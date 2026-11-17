# Plug a 40 mm DAC plug into a switch cage

A data-centre bench task. A short 1U-style switch sits on a riser with two DAC/SFP port cages facing the robot:
the near (target) port and a neighbouring decoy. A cable-management comb stands over the target port, and a
40 mm DAC plug waits tip-first in a cradle to the left. The robot picks the plug, aligns it with the keyed cage,
pushes it in until the spring latch clicks, seats the strain-relief boot between the comb teeth, then tugs the
boot back to prove the latch holds before letting go. The instruction states only the goal — which port, that the
push must be straight, that the boot belongs in the comb, and what the tug means are the robot's to work out.

**What is new against the board (fresh query 2026-11-17).** `gh pr list --repo robocurve/benchmark-tasks --state all
--limit 3000 --json number,title,state,body,url,closedAt,mergedAt,labels` returned **2,523 PRs, newest #2524**
(451 OPEN, 1,865 CLOSED, 207 MERGED). No OPEN task has this object + mechanism + outcome. The adjacent
closed-final connector tasks are #1516 (fibre SFP into a NIC — no latch/click/boot/tug, "Impact on jobs"), #2143
(unlatch and *remove* a QSFP28 from a switch — the opposite action) and #14 (dress three cables into a comb — a
different action). Full search in `TWIN_CHECK.md`.

| File | What it is |
|---|---|
| `dac_plug_switch_cage.py` | The `yam_arena` task file: scene, the fixed switch and free plug, the align/latch/boot/tug mechanism, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/switch.usda` | The switch chassis, riser, two port cages with latch notch, the comb and the plug cradle |
| `assets/plug.usda` | The real 40 mm DAC plug (keyed body, gold tongue, latch tab, rubber boot) |
| `build_assets.py` | Writes the two USD files (pxr; boxes only) |
| `test_dac_plug_switch_cage.py` | Offline checks (20): every scoring outcome and cap, the 1 s hold and its restart, gripper-clear, real sizes, YAM reach/press gate, all demo waypoints reachable, plan and timing |
| `make_media.py` | Renders the straight-down top-view `setup.png`, `demo.mp4` and `demo_top.gif` |
| `task.json` | The task details for the board |
| `REFERENCE.md` | The real product, its recognisable features, the sources and every measured vs estimated number |
| `PASS_THROUGH_QA.md` | Contact-probe and frame-by-frame top/close-up evidence that no part passes through a solid |
| `TWIN_CHECK.md` | The fresh live-twin query and the nearest PRs |

## Real product and values

The product class is a **40 mm DAC (direct-attach copper) plug** for a switch port — an SFP/SFP+-style metal
module with a gold contact tongue, a top spring latch tab and a moulded strain-relief boot. The switch has metal
port cages and a cable-management comb. Public SFP/SFP+ specs give a 13.7 x 8.6 mm module in a 14.0 x 8.95 mm
cage, 34-40 N maximum insertion force and >= 90 N latch retention. The task uses a 13.5 x 8.6 mm plug in a
14.5 x 9.5 mm cage, a 4-7 N push, a 12 mm boot in a 14 mm comb slot, all at true size. Sources and the
measured/estimated split are in `REFERENCE.md`.

**Measured (product class):** SFP module 13.7 x 8.6 mm; cage inside 14.0 x 8.95 mm; cage overall 15.0 x 9.8 mm;
insertion 0-40 N (SFP+ 34 N); latch retention >= 90 N; DAC cable 4.5-6 mm; push straight in until it clicks.

**Estimated (each in `REFERENCE.md` and `task.json` fidelity):** plug body length 40 mm and width 13.5 mm; cage
mouth 14.5 x 9.5 x 45 mm; boot flange 12 mm; comb slot 14 mm; 4-7 N push over a 36 mm stroke; latch click at
36 mm depth, boot seated at 40 mm; a 4 mm tug; plug mass 20 g. No size, mass or force is scaled down.

## Layout

Positions are in metres from the midpoint between the arm bases, x forward, y to the robot's left, table top z 0
(world z 0.7475). The switch face is at **x 0.310**; its two port cages are at **y +0.015 (target)** and
**y -0.015 (decoy)**, centres at **z 0.100**. Each cage mouth is 14.5 x 9.5 mm and 45 mm deep, with a gold latch
notch on the target's top wall. A two-tooth comb at x 0.312 forms a 14 mm slot over the target port. The plug
starts tip-first in a cradle at **x 0.250, y 0.060**, centre z 0.100. The switch and cradle are fixed; both arms
start at READY. The work point (x 0.31, y 0.015, h 0.10) is reached by the left YAM arm with ~0 mm IK error; the
queue's suggested h 0.20 was unreachable (22-38 mm error) and a low real bench riser is used instead.

## Mechanism

The plug is a real gravity-free rigid body. While a gripper's fingers close across its grasp point the plug
follows the gripper (a declared carry, the same family device as the cantilever box's latches); the robot must
still reach, close on, carry, align and insert it. Orientation is preserved tip-first/latch-up. The latch and
boot are declared mechanism states checked against the plug's real pose: the tab clicks when the tip reaches
36 mm depth in the target cage, the boot seats when the plug is 40 mm deep and square in the comb, and once
seated the latch **pins** the plug so a 4 mm pull cannot back it out. The two port cages and the comb are real
geometry; inserting into the decoy, forcing in off-axis, missing the latch, leaving the boot out or a failed tug
each cap the score. The spring force and click are declared states, not a simulated spring; there is no cable or
electrical signal (see `REFERENCE.md`).

## Demo

Open loop, known geometry, one arm (the left; the right stays home). The left arm: (A) picks the plug from its
cradle; (B) carries it to the near port and holds it aligned with the keyed cage; (C) pushes in until the latch
clicks; (D) gives a final straight push to seat the boot in the comb, releases and lifts away; (E) grips the boot
tail behind the cage, draws it back 4 mm to tug-test the latch, releases and goes home. Every approach is from
above with the fingers closing across the part.

**MuJoCo 1.0 at step 477 of 900 (23.85 s after the settle).** Step timeline: aligned 128, latched 191, boot seated
227, tug holds 329; the full 1 s hold completes at 477. Final plug depth 39.2 mm, keyed, no cap, both grippers
home. Idle 0; aligned 0.25; latched 0.5; boot seated 0.75; tug held + 1 s 1.0; wrong port, forced plug, latch
miss, boot not seated and failed tug each cap at 0.5.

## Scoring

| Step | Done when | Points |
|---|---|---|
| aligned at target port | plug tip at the near cage mouth, keyed, within tolerance | 0.25 |
| latched / click | pushed to latch depth in the target cage, tab clicks | 0.50 |
| boot seated in comb | plug square at full depth, boot between the comb teeth | 0.75 |
| tug holds | 4 mm tug leaves the plug latched, +1 s hold, grippers clear and home | **1.0** |

Caps (0.5 ceiling): wrong port, forced plug, latch miss, boot not seated, failed tug. Idle 0.

## Media

`demo.mp4` (640x360, straight-down top view, < 1.87 MB), `demo_top.gif` (< 1.87 MB) and `setup.png` are written
by `make_media.py` into `benchmark-media-dac-plug-switch-cage/`. The film is the scripted MuJoCo demo; no Dream
run was made by the builder.

## Gate status

**MuJoCo first gate: 1.0 at step 477, 20/20 offline tests pass, all `task.json` fields within limits (keywords
162 <= 200).** Isaac demo 1.0 at step 477 (`adhoc_a35a5b48.json`); Dream-1 0.0 at 2,400 steps (2,401 action rows). No PR, upload
or push was made.
