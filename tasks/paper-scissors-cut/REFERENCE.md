# REFERENCE — paper-scissors-cut

## Sources and real values

| Item | Value | Source / status |
|---|---|---|
| Fiskars 8 in office scissors | 203 mm overall, ~60 g, stainless blades, orange loop handles | Fiskars 01-004761 class product data (EST: 90 mm blade from rivet, 10 mm handle thickness) |
| Hinge return spring | 0.4 N m/rad, damping 0.02 N m s/rad, 3 N m drive limit | EST from a light office scissor; authored as a USD angular drive (degree units converted in `build_assets.py`) |
| A4 paper | 210 x 297 mm, 80 gsm, 0.10 mm, 4.99 g | ISO 216 / standard 80 gsm; 80 g/m2 x 0.06237 m2 = 4.99 g |
| Offcut strip | 100 x 210 mm, 1.68 g | 100 mm measured from the long edge of the A4 sheet |
| Cut line | 210 mm long, y = -0.150, x = 0.360–0.570 | Measured from the sheet layout; red rule and end ticks in the USD |
| Tray | 404 x 264 mm opening, 6 mm floor, 12.5 mm rim, floor z 0.7535 | EST, a common A4 letter tray; sized for the 100 x 210 mm strip |
| Riser / mats | 20 mm riser + 1 mm green mat (top 0.7685); 30 mm rest + 1 mm grey mat (top 0.7785) | EST fixtures so the blade clears the mat and the scissors have a marked home |
| Hinge open/close | 10.1 degrees at the line start, ~1.3 degrees during the sweep | Measured from the final MuJoCo run |
| Cut front / deviation | 211.97 mm (tip past the 210 mm line); 0.02 mm max deviation | Measured from the final MuJoCo run at step 545 |
| Offcut landing | (0.465, -0.200, 0.75354) m, inside the tray, below rim 0.7600 | Measured from the final MuJoCo run |
| Scissor rest | rivet (0.32925, -0.46459, 0.78498), blades 18.1 degrees open, speed ~1e-7 m/s | Measured from the final MuJoCo run |
| Kept sheet | (0.465, -0.0515, 0.76854), shift 0.0 mm, turn 0.0 degrees | Measured from the final MuJoCo run |

Trial cost: a pair of office scissors (~$12–20), A4 paper (~$0.02/sheet), a letter tray (~$5), two risers/mats
(~$10): well under $50.

## Mechanism model

- **Scissors**: `assets/scissors.usda` is a free two-body articulation. `ScissorA` (root, 32 g) and `ScissorB`
  (child, 28 g) are joined by the `Shear` Z revolute at the rivet; the USD drive is the return spring. Each link has
  a convex blade wedge, a neck, an elliptical loop handle, a flat grip plate and an outward grip lip. The grip lip
  is the pad contact face at pickup. The 10 mm loop thickness lets the parallel pads clamp the handles.
- **Grasp**: while the right pads are on the handles (`_on_handles`: contact > 1 N, pad centre within 20 mm of the
  nominal handle midpoint, opening < 85 mm), `_MECH.grasp_locked` engages. The interval event `_scissor_servo` then
  writes the scissor **root pose** every control step from the arm's **commanded** TCP (`action_manager` joint targets
  -> `ik.tcp_pose`), offset 74 mm forward / 13.5 mm down, with the bisector along world +X and the root yaw set to
  `-hinge/2`. Opening the hand past 86 mm near the rest mat releases the servo and restores the open hinge target.
- **Hinge actuation**: the same event writes the `Shear` target from the actual right gripper opening:
  `HINGE_OPEN * clip((opening - 0.050) / 0.0439, 0, 1)`. A 94 mm opening is fully open; 50 mm is shut.
- **Paper bond**: `paper` and `offcut` are two free bodies. While `_MECH.bonded`, `_paper_pin` writes the offcut pose
  from the kept sheet's pose plus the 148.5 mm relative translation and zeroes its velocity; the offcut is spawned
  with gravity disabled so it cannot sag between corrections. This is the intact-fibre seam. When the blade tip has
  swept past `LINE_END` with the blades closed, on the line and with the sheet unmoved, `_release_offcut` stops the
  servo, enables gravity (MuJoCo: `body_gravcomp = 0`) and gives the strip a 0.15 m/s downward velocity to land in
  the tray. The sheet is **not** pre-cut: before the cut the strip cannot move independently; a seam deviation during
  the cut latches the `bond_error` cap.
- **Cut geometry**: `extra_states` reads both link poses, forms the two 90 mm blade tips, the bisector and the tip
  midpoint. A cut frame requires `blade_open <= 0.10 rad`, tip and rivet within 3 mm of the paper plane, the rivet
  within 1.5 mm of the line and inside the x range. The cut front is the max tip x; the line deviation is the max
  lateral error of tip or rivet. The strip is released only after the front passes `LINE_END - 0.8 mm`.
- **Scoring**: the ladder (0 / 0.25 / 0.5 / 0.75 / 1.0) and the caps are in `paper_scissors_cut.py`. A 1 s pose hold
  is inside `success()`; `SUCCESS_HOLD_S = 0` so the framework does not add a second hold.

## Calibration

The arm's actual TCP trails its command by 1–2 mm during the sweep. Servoing the tool to the **commanded** TCP makes
the blade run on the printed line while the pads remain visually on the handles (a compliant-grasp abstraction).
The rest and cut TCP heights (13.5 mm above the paper, 18.5 mm above the rest mat) place the pad surfaces 1 mm above
the grip lips so the lips carry the tool. The mat edge is 50 mm short of the cut line so neither the pads nor the
upper blade scrape it. The tray's 6 mm floor stops the 0.1 mm strip from tunnelling. The left press point is 7 mm
inside the kept sheet's +Y edge, reachable by the left arm in a straight-down grip.

## Pass-through / contact filtering

`local_sim/mjkit/sidecars/paper_scissors_cut.py` adds excludes for:

- scissors bodies vs the paper and offcut — the paper is a rigid bond, not a tearable mesh; the blade sweeps through
  the seam plane and the bond release, not contact, is the cut (the same abstraction as the guillotine task's blade
  with no collider). The blade still collides with the tray and mat, so a dip into the tray is measured.
- right fingertips vs the paper and offcut — the pads pass just under the sheet at the line without dragging it.
- the two paper bodies vs each other — the 0.1 mm seam gap would otherwise rub.
- the left `tip_right` pad vs the paper — the closed left pads must not pinch the 0.1 mm sheet between their faces.

On Isaac the equivalent filtering should be authored as collision groups / filtered pairs. See
`PASS_THROUGH_QA.md` for the measured contact audit.

## Limitations (read before the Isaac finisher)

1. **Servo grasp**: the tool pose is written from the arm's commanded TCP, not from contact forces. The pads do touch
   the handles (contact audit), but the 1–2 mm arm error is absorbed by the servo. On Isaac the finisher may prefer a
   high-friction grip and should re-check that the blade still tracks the printed line within 1 mm.
2. **Left hand / sheet contact**: in MuJoCo the left fingertips are positioned over the sheet and `pressed` is scored
   geometrically, but the YAM pad mesh does not produce a resolved contact on the 0.10 mm sheet at that pose; the sheet
   is held in place by the mat and the filtered right pads. A real trial presses the sheet with a finger; on Isaac
   the finisher should verify (or add a small compliant presser pad / clip so that) the left hand physically holds the
   sheet. This is a fidelity gap, not a scoring relaxation.
3. **Rigid paper / bond**: the sheet cannot wrinkle or tear progressively; "uncrumpled" is scored as no shift/tilt,
   and a tear as a seam pull error or a sheet shift. A deformable sheet is out of scope for this benchmark's task-file
   format.
4. **No photometric/cutter render**: the cut is the bond release driven by the measured blade sweep; no mesh fracture
   and no visible kerf opens until the strip drops. There is no cutaway and all bodies are drawn whole.
5. **Isaac/Dream not run**: only MuJoCo was available in this build; the finisher must run the Isaac embodiment and
   Dream-1, then publish. No push, upload or PR was made here.

## Files checked

- `submission_tools/check_fields.py paper-scissors-cut`: all fields ok.
- `paper_scissors_cut.py` is the only task file; `build_assets.py`, `test_*.py`, `make_media.py` and the three `.md`
  files are the build/test/docs files in the folder (as in the other tasks here).
