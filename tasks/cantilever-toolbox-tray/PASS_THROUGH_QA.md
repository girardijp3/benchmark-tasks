# PASS_THROUGH_QA — cantilever-toolbox-tray

No arm, pad, finger, forearm or held part passes through a solid in the demo. The per-step contact audit below is
MuJoCo (top camera plus wrist-camera close-ups, `data.contact` every control step). The same scripted demo was run
on Isaac and also reached **1.0 at step 359**; the Isaac evidence is the top-camera frames and two render-only
latch close-up cameras saved while the run played (`~/film_out/cantilever-toolbox-tray`). Source:
`toolbox_cantilever.py`, `benchmark-media-cantilever-toolbox-tray/demo.mp4` (Isaac top view).

## 1. Frames checked (top view, 0/10/25/45/60/80/95/100 %)

| % | frame | what it shows |
|---:|---:|---|
| 0 | 0 | closed box, arms at the two front latch knobs, no overlap |
| 10 | 35 | both arms approach the latch knobs from the front, outside the box |
| 25 | 89 | pads straddle the two latch knobs, plates swinging out; no finger inside the box |
| 45 | 161 | left arm moves from the released latch to the wire lid handle above the lid |
| 60 | 215 | left pads on the 8 mm handle bar, above the lid top; right arm already home at the side |
| 80 | 287 | lid lifted, both trays swinging out level; grippers above/outside the solids |
| 95 | 341 | trays fully open to y +-0.31; left hand releasing, right hand home; no overlap with the open trays |
| 100 | 358 | scored state: box on four feet, trays open, tools in their trays, both hands clear |

Close-ups (wrist cameras): 4.0 s — pads on the two latch knobs; 10.0 s — pads on the handle bar; 12.0 s — mid lift
with the bar still captured; 17.0 s — right wrist looks at the open right tray and screwdriver, no penetration.
No forearm or held part crosses a tray wall, the lid skirt or the base in any frame.

## Isaac run and film

The Isaac demo reached **1.0 at step 359** (`adhoc_e4e52215.json`): both latches pulled to **-25 deg**, the lid
lifted **115.0 mm**, the arms swung to **-130/+135 deg**, the tools stayed in their trays and no damage, pinch or
forced latch fired. The run was filmed on a copy of the task with two render-only close-up cameras (the task logic,
physics and scoring are unchanged): `film_latch` looks at the front of the two latch knobs and `film_latch_side`
looks across the left latch pad and the lid handle. The top view and both close-ups were reviewed at the eight
top-view phases in the table above and at t 4.0 s (pads on the two knobs), 10.0 s (left pads on the wire handle),
12.0 s (mid lift) and 17.0 s (open trays). No arm, finger, forearm or held tool is seen crossing a tray wall, the
lid skirt or the box wall, and no tool appears to float. Isaac does not expose the MuJoCo-style per-step contact
pair log; the numeric contact table below is from MuJoCo.

## 2. Contact evidence (`data.contact`, MuJoCo)

Pairs are sorted body names with contact counts. Only the intended task contacts and the resting/loaded contacts occur.

| step | t (s) | phase | robot-box contacts |
|---:|---:|---|---|
| 40 | 2.00 | latch approach | none |
| 80 | 4.00 | latch pull | `left/tip_left`–`toolbox/LatchL` (4), `left/tip_right`–`toolbox/LatchL` (3), `right/tip_left`–`toolbox/LatchR` (4), `right/tip_right`–`toolbox/LatchR` (4) — the intended knob grips |
| 110 | 5.50 | latches released | none |
| 200 | 10.00 | handle grip | `left/tip_left`–`toolbox/Top` (4), `left/tip_right`–`toolbox/Top` (4) — the intended wire-handle grip (the bar is part of the lid body `Top`) |
| 240 | 12.00 | mid lift | same two intended handle-bar contacts; no tray or base contact |
| 280 | 14.00 | trays open | none |
| 340 | 17.00 | hands clearing | none |
| 359 | 17.95 | scored state | **none** |

All other contacts at every step are the physical resting/loaded pairs only:
- `toolbox/Base`–`world` (16) — the four rubber feet on the bench;
- `hammer`–`toolbox/Base` (3) — the hammer in the upper fixed tray;
- `screwdriver`–`toolbox/WingR` (2–3) — the screwdriver in the right moving tray;
- `toolbox/WingL`–`wrench` (2–6) — the wrench in the left moving tray.

No gripper, finger or forearm touches the box walls, the tray walls, the lid skirt or the table edge at any step beyond
the four intended knob/handle contacts above. No tool appears floating or outside its tray; the tools stay in their
tray frames throughout the lift (final lift 114.2 mm, tray level error < 0.1 deg).

## 3. Pinch protection

The scorer has an explicit `pinched` latch: any pad point inside the box envelope (x +-0.100, y +-0.220, z 0.100-0.200
in box-local coords) while the lid/trays are moving sets the latch, caps the score at 0.5 and blocks 1.0. The demo never
triggers it (both arms stay at the front knobs and above the lid). Combined with the contact list, no finger is ever
in a closing gap.

## 4. Media and cutaways

- `demo.mp4`: top camera only, 640x360, 21.45 s, 429 frames (369 filmed including the 0.5 s settle, then the final
  1.0 frame held 3 s), 352,903 bytes (< 1.87 MB). No close-up or cutaway views in the PR media.
- `demo_top.gif`: 480x270, 6 fps, 129 frames, 1,672,925 bytes (< 1.87 MB), same top view.
- `setup.png`: 640x360, 31,036 bytes (< 1.87 MB), Isaac top camera at reset.

Cutaways: **none**. No wall is hidden; the top camera looks straight down at the open box. The two latch close-up
cameras were added only to a filming copy of the task (render-only, no physics or scoring change); they are
internal evidence for this check and are not in the PR media.
