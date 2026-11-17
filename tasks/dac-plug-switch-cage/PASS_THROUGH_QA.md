# Pass-through QA — no part passes through a solid

The only moving part is the DAC plug. It is a gravity-free rigid body whose pose is written every control step by
the declared mechanism (the same declared-carry device the family uses for detents and loose parts), so its
trajectory is analytic, not contact-driven. This note checks that the analytic path never puts the plug through
the switch, the cage walls or the comb, and that the rendered frames agree.

## Geometry clearance (analytic, from `build_assets.py` / `dac_plug_switch_cage.py`)

| Interface | Plug part | Solid | Clearance |
|---|---|---|---|
| Body in cage (width) | 13.5 mm body | 14.5 mm mouth | 0.5 mm per side |
| Body in cage (height) | 8.6 mm body | 9.5 mm mouth | 0.45 mm top/bottom |
| Boot in comb | 12 mm boot flange | 14 mm comb slot (two 2 mm teeth 18 mm apart) | 1 mm per side |
| Tip vs cage back | tip at world x 0.350 at full seat | cage floor at x 0.355 | 5 mm of travel left |
| Keyed orientation | local +x tip, +z latch-up throughout | key rib / latch notch | plug never rolled or flipped |

The plug goes in on a straight +x line at constant y 0.015, z (TOP+0.100). With the cross-sections above, every
point of that path clears the cage and comb; the only way to fault is the scored mechanism states (wrong port,
forced off-axis, missed latch, boot out of comb, failed tug).

## Contact probe (MuJoCo, every milestone of the 1.0 demo)

All `d.contact` pairs were scanned at steps 1, 60, 128, 160, 191, 210, 227, 260, 329, 360 and 477, keeping any
pair whose body name contains `plug`:

| Step | Phase | Plug–solid contacts |
|---:|---|---|
| 1 | staged in cradle | none |
| 60 | descending onto plug | none |
| 128 | aligned at target port (score 0.25) | none |
| 160 | holding alignment | none |
| 191 | latched / click (0.50) | none |
| 210 | seated boot push | none |
| 227 | boot in comb (0.75) | none |
| 260 | released, gripper lifting away | none |
| 329 | tug registered (1.00 path) | none |
| 360 | holding the tug | none |
| 477 | full success | none |

No contact involving the plug at any sampled step — the plug is carried above the bench and enters a cage whose
walls are visual geometry (the static switch has no colliders, as in the family's fixture convention). There is
therefore no contact-driven penetration and no contact chatter to corrupt the score.

## Frame-by-frame visual check (top view)

`benchmark-media-dac-plug-switch-cage/frames/` holds the straight-down top-view render every 3rd control step;
`top_frames/` holds the un-annotated crop. The frames were inspected at the four scored transitions:

- **frame_0043 / step 128** (aligned, 0.25): plug tip at the near cage mouth, cable side clear, both cage mouths
  visible; the plug is over the target (y +0.015), not the decoy (y -0.015).
- **frame_0065 / step 192** (latched, 0.50): plug body swallowed by the cage, boot still proud of the comb; no
  overlap with the neighbouring port or the comb teeth.
- **frame_0077 / step 227** (boot seated, 0.75): plug flush in the cage, boot flange between the comb teeth.
- **frame_0113 / step 336** (tug, 1.00 path): gripper on the boot tail behind the cage, plug still flush — the
  latch holds and the plug does not slide out.

No frame shows the plug crossing a cage wall, the comb teeth, the cradle or the neighbouring port. `setup.png`
and `demo_top.gif` use the same straight-down camera, so the recorded media matches this check.

## Result

No pass-through. The plug's analytic path clears every solid by at least 0.45 mm (body/cage) and 1 mm (boot/comb),
the contact probe finds no plug–solid contacts at any milestone, and the top-view frames agree.


## Isaac run

The same scripted demo reached **1.0 at step 477 on Isaac** (`adhoc_a35a5b48.json`): the plug was carried to the
correct target port (y +0.015), seated to 39.5 mm depth with the boot between the comb teeth, the latch held the
pull test and the grippers cleared; no wrong-port, forced or drop fault. The task-local Isaac change was
`disable_reset_pose()` on the switch and plug (the task writes their root poses itself; Arena's float64 reset pose
is rejected by PhysX's transform kernel). The declared carry, geometry, ports and scoring are unchanged. The run
was filmed with the top camera and two render-only close-ups (`film_ports`, `film_boot`) on a copy of the task;
no gripper, finger, forearm or plug is seen crossing a cage wall, comb tooth or the neighbouring port. Media:
`demo.mp4` (Isaac top view, 640x360, 295,038 B, final 1.0 frame held 3 s), `demo_top.gif` (480x270, 1,729,470 B),
`setup.png` (28,798 B).
