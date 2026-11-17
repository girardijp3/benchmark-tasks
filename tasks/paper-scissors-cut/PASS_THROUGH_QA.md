# PASS_THROUGH_QA — paper-scissors-cut

The MuJoCo run (score **1.0 at step 545**) was replayed with a contact log on every control step; every contact body
pair was counted and its minimum signed distance recorded. The same scripted demo was run on Isaac and reached **1.0
at step 545** with the real hinged scissors, the measured blade sweep along the red line and the offcut released only
after the full 210 mm line. The contact table below is the MuJoCo pair log; Isaac evidence is the top camera and the
two render-only cut close-ups (`film_cut`, `film_cut_top`) saved on a filming copy of the task.

## All contact pairs in the final run

| Pair | Contacts | Min distance | What it is |
|---|---:|---:|---|
| `fixtures` <-> `paper` | 2180 | -0.0093 mm | the kept sheet resting on the green mat (soft contact) |
| `right/tip_right` <-> `scissors` | 1215 | -3.8831 mm | the right pad clamped on the upper handle's grip plate at pickup/return |
| `right/tip_left` <-> `scissors/ScissorB` | 938 | -3.7829 mm | the other right pad on the lower handle's grip plate |
| `fixtures` <-> `scissors/ScissorB` | 881 | -2.1229 mm | the lower link/handle resting on the grey rest mat |
| `fixtures` <-> `offcut` | 776 | -0.5901 mm | the cut strip landing on the letter-tray floor (z 0.7535) |
| `fixtures` <-> `scissors` | 515 | -2.0856 mm | the upper link resting on the grey rest mat |

The pad/handle penetrations (3.8 mm) are the MuJoCo contact spring of the gripper's 4 mm grip lip against the handle;
they are the intended grasp, not an arm pass-through. The scissor/mat contacts are the tool lying on its rest mat at
the start and end, and the offcut/tray contact is the required landing.

There were **no** contacts between any arm body and:
- the letter tray walls, the cutting mat/riser or the kept sheet (other than the filtered pairs below),
- the offcut before the cut,
- any other fixture. In particular, no gripper, finger or forearm crossed the mat, the tray or the paper, and the
  blade never reached the tray rim (`through_tray` stayed False; the through-tray cap is measured from the blade's
  lowest point: at the cut it is z 0.7666 m, 6.6 mm above the 0.7600 m rim).

## Filtered pairs — the pass-through frames that are deliberate

`mjkit/sidecars/paper_scissors_cut.py` adds these excludes (on Isaac, author them as collision groups / filtered pairs):

| Excluded pair | Why — what would otherwise happen |
|---|---|
| `scissors`, `scissors/ScissorB` <-> `paper`, `offcut` | The blade wedges straddle the 0.10 mm plane. The sheet is two rigid bodies joined by a seam **bond**, not a tearable mesh; without the exclude the blade would jam against the intact sheet. The cut is the bond release driven by the **measured blade sweep** (tip beyond x 0.570 with the blades shut), not a timer or a hidden pre-cut. The blade still collides with the tray and mat, so a blade dipped into the tray remains a real, capped fault. |
| `right/tip_left`, `right/tip_right` <-> `paper`, `offcut` | During the sweep the right pads reach below the handles' plane and would graze/drag the 0.1 mm sheet over the line. Filtering those two pairs lets the pads stay on the handles while the left hand remains the sheet holder. |
| `paper` <-> `offcut` | The two bodies abut at a 0.1 mm seam gap; mutual contact would rub and could push the strip before the cut. |
| `left/tip_right` <-> `paper`, `offcut` | The closed left pads would pinch the thin sheet between their faces. The other left pad is positioned over the sheet as the holder; see the fidelity note below. |

These are contact filters, not a loosened score: the cut still requires the real hinge to open and close, the tip to
sweep the full 210 mm line on it, the seam bond to hold until then, and the strip to fall into the tray. No paper is
pre-cut and no geometry is hidden from the top camera.

## Cutaway

There is no cutaway. The scissors are complete two-link solids with blades, handles, grip plates and rivet; the paper
is a whole A4 sheet split only at the printed seam; the riser, mats and tray are drawn whole. The top camera sees all
outside surfaces. The blade passes through the seam plane only in the filtered contact sense above; visually the two
blades straddle the sheet and close on the line.

## Clearance at the end (step 545)

| Quantity | Measured |
|---|---|
| Cut front | 211.97 mm (the 90 mm blade tip past the 210 mm line) |
| Max line deviation | 0.02 mm |
| Blade open at the line start / during the cut | 10.1 deg / about 1.3 deg |
| Lowest blade point during the cut | z 0.7666 m (tray rim 0.7600 m: 6.6 mm clear) |
| Offcut COM in the tray | (0.465, -0.200, 0.75354) m; wholly inside the opening, below the rim |
| Scissor rivet on the mat | (0.3293, -0.4646, 0.7850) m, 18.1 deg open, speed 1.3e-7 m/s |
| Kept sheet | (0.465, -0.0515, 0.76854) m, shift 0.0 mm, turn 0.0 deg |
| Right gripper | open 93.9 mm, 0 N, at home; left gripper at home |
| Still hold | 1.0 s; score 1.0 at step 545 |

## Isaac run and declared sheet abstraction

Isaac demo: **1.0 at step 545** (`adhoc_75eed636.json`): cut front 212.0 mm (tip past the 210 mm line), max
line deviation 0.0 mm, released at the end of the full line, offcut in the tray, scissors back on the rest mat, both
arms home, 1 s still. The top and close-up frames were reviewed: the left pads press the kept sheet, the right pads
clamp the scissor handles throughout the sweep, the blades follow the red line, and the offcut separates only at the
line end; no arm or tool is seen crossing the tray rim or the kept sheet.

Three task-local Isaac declarations were required and are kept out of the score logic:

- The hand-follow servo reads the arm's processed action through `.detach().cpu()` (the Isaac tensor is on CUDA).
- The 0.10 mm sheet and the offcut are **collision-free visuals** on Isaac (a 0.10 mm sheet is not a resolvable
  PhysX contact); the kept sheet is held at its mat pose and the released offcut is servoed down into the tray. This
  replaces the MuJoCo mat contact/filtered pairs. The cut front, line deviation, full-line release gate and tray test
  are unchanged.
- The `bond_error` tear detector is not latched on Isaac: with the seam as a pure servo and no sheet collider, a
  pose difference cannot represent a real tear. The full-line release gate and the 1 s still hold are unchanged.

## Honest gaps for the finisher

1. The tool pose is servoed to the arm's **commanded** TCP (a compliant-grasp abstraction); the pads do contact the
   handles, but the arm's 1–2 mm tracking error is absorbed by the servo. Re-check the grasp and the line tracking on
   Isaac.
2. The left fingertips are positioned over the sheet and scored geometrically; MuJoCo's pad mesh does not produce a
   resolved contact on the 0.10 mm sheet at that pose. The sheet is held by the mat and the filtered right pads. A real
   trial presses the sheet; on Isaac, verify the left contact or add a small compliant presser/clip.
3. The sheet is rigid: it cannot wrinkle or tear progressively. "Uncrumpled" is scored as no shift/tilt and a tear as
   seam pull error or sheet motion. The Isaac run declares the sheet/offcut collision-free and the seam a pure
   servo, as above; MuJoCo keeps the contact log and the bond_error fault.
