# PASS_THROUGH_QA — stick-vac bin eject

No arm, pad, finger, forearm or held part passes through a solid in the demo. The table below is the MuJoCo
`data.contact` audit (top camera plus wrist close-ups). The same scripted demo reached **1.0 at step 520 on Isaac**
(`adhoc_e3f66aa5.json`) with the same real 6 N button, side-wall bin grip and declared bottom-flap dump; the Isaac
evidence is the top camera and two render-only close-ups (`film_bin`, `film_button`) saved on a filming copy.

## 1. Frames checked (top view, 0/10/25/45/60/80/95/100 %)

| % | frame | what it shows |
|---:|---:|---|
| 0 | 0 | vac and seated clear bin, arms at READY, no overlap |
| 10 | 52 | right pads close on the red release button, fingers straddling its 14 mm cross-knob; no finger inside the body |
| 25 | 131 | button released, right arm crossing to the top carry point; bin still seated |
| 45 | 235 | right pads pressing the bin walls at its centre of mass, bin pulling forward off the cradle |
| 60 | 314 | bin held high over the waste bin, bottom flap open, dust falling into the waste bin |
| 80 | 418 | empty bin being carried back, flap shut, right pads on the bin walls, left arm home |
| 95 | 497 | bin lowered to the cradle, right hand released and lifting straight clear; bin level on its shelf |
| 100 | 523 | scored state: bin seated, button out/clicked, dust in the waste bin, vac on four feet, both hands clear |

Wrist close-ups (internal, not in PR media): 4.0 s — pads on the red button; 10.5 s — pads on the bin walls;
14.5 s — flap open with dust falling; 21.5 s — empty bin back at the cradle. No forearm or held part crosses the
motor housing, the cradle walls, the waste-bin rim or the bin walls in any frame beyond the intended grips below.

## 2. Contact evidence (`data.contact`, MuJoCo)

Pairs are sorted body-local geom names with contact counts. Only the intended task contacts and resting/loaded
contacts occur; no arm–solid penetration pair appears.

| step | t (s) | phase | robot–task contacts |
|---:|---:|---|---|
| 40 | 2.0 | approach button | none |
| 80 | 4.5 | button press | `EjectKnob`–`right/*` fingertip pads (4-7 each) — the intended grip |
| 120 | 6.5 | button released | none |
| 200 | 10.5 | bin pull | `WallY1`/`WallY2`–`right/*` fingertip pads (3-4 each) — intended bin grip |
| 240 | 12.5 | bin lift | same two intended bin-wall grips; no cradle or body contact |
| 280 | 14.5 | dumping | same two intended grips; dust resting in the waste bin |
| 300 | 15.5 | hold over waste | same two intended grips; flap fully open |
| 340 | 17.5 | carry back | same two intended grips; flap shut |
| 420 | 21.5 | lower to cradle | same two intended grips; bin settling onto the shelf |
| 460 | 23.5 | hands clearing | none |
| 500 | 25.5 | final hold | none |
| 523 | 26.65 | scored state | **none** |

All other contacts at every step are the physical resting/loaded pairs only:
- `vac/Body` feet–`world` — the four rubber feet on the bench;
- `bin` walls/flap–`vac/CradleShelf` — the seated bin on its shelf;
- six `debris_*`–`bin/FlapPlate`/`bin/Wall*` before the dump, then `debris_*`–`waste/Floor` after it;
- the red button's prismatic drive holding it at its stop.

No gripper, finger or forearm touches the motor housing, the bin's top collar or the waste-bin rim at any step beyond
the two intended grips (button knob, bin walls). The final contact list has no arm–task pair. The bin sits level at the
scored state (tilt under the 8 degree window, offset under 12 mm) and the vac's tilt and travel are far inside the
stability window.

## 3. Pinch / cap protection

The scorer has no separate pinch pair because the only moving gap is the bin's bottom flap, which is opened by a
declared rule only while the bin is held over the waste bin — there is no finger-sized closing gap. The caps that are
scored are: dropped bin (0.25), dust left or unlatched refit (0.75), forced pull/latch (0.5) and tipped/moved/off-table
vac (0). The demo triggers none of them; the unit tests drive each one explicitly.

## Isaac run

Isaac demo: **1.0 at step 520** (`adhoc_e3f66aa5.json`): the 6 N eject button was pressed past the 5 mm trigger, the
bin was gripped by its side walls, lifted clear, carried over the waste bin and its bottom flap opened by the
declared rule, then refitted and latched with the unit steady and both grippers clear, 1 s still. No drop, forced
pull, missing dust or tip/move fault. The top and close-up frames were reviewed: no arm, finger, forearm or bin
passes through the vac body, cradle, waste bin or table. No task-local scoring or force change was needed.

## 4. Media and cutaways

- `demo.mp4`: Isaac top camera only, 640x360, 429,755 bytes (< 1.87 MB), 590 frames with the final 1.0 frame held
  3 s. No close-up or cutaway in the PR media.
- `demo_top.gif`: 480x270, 5 fps, 1,411,428 bytes (< 1.87 MB), same top view.
- `setup.png`: 640x360, 31,892 bytes, Isaac top camera at reset.

Cutaways: **none**. No wall is hidden; the top camera looks straight down at the vac, bin and waste bin, and the wrist
close-ups used for this check are internal evidence only.
