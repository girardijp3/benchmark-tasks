# REFERENCE — toggle-clamp-gusset

## What the real object is

A **100 mm steel angle-bracket gusset** (a 90 degree L bracket, 100 mm legs) located on a bench fixture and held
square by **two hold-down toggle clamps**. Recognisable features: a horizontal **foot** and a vertical **upright**
meeting at a 90 degree inner corner; a fixture with a **locator nest** (a back wall the upright leans on, an end
stop and two locator pins through the foot); and two lever clamps whose red handles are pushed **over-centre** so a
rubber jaw pad presses the foot down. Each lever takes a **4-7 N** hand force to snap past its toggle point. The job
is a routine fixturing step in fabrication and welding shops: drop the part on the locators, throw both clamps,
then tack, drill or grind it while it cannot move. A part off its pins, a lever left short of over-centre, or a
dropped part means a crooked weldment and a rework or scrap.

Sources for the object class: general catalogue/retail listings for 100 mm steel angle brackets and hold-down
toggle clamps (L-bracket legs, clamp lever and rubber hold-down pad, over-centre locking, hand-force band). The
brief supplies the 4-7 N per clamp and the YAM gate (work height h 0.15, reach 0.40, 100 mm gusset). No single
manufacturer drawing is public, so the individual dimensions below are estimates within that envelope, each
flagged.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Gusset | 100 mm steel angle bracket, 90 degree legs | product class / brief |
| Clamps | two hold-down toggle clamps, lever + rubber jaw pad | product class / brief |
| Clamp hand force | 4-7 N each | brief |
| Fixture | bench nest: back wall, end stop, two locator pins | product class (fixturing practice) |
| YAM station gate | h 0.15, reach 0.40, 100 mm gusset | brief |
| Outcome | gusset seated square and locked by both clamps | brief |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Gusset foot | 100 x 30 x 4 mm | typical 100 mm angle bracket; 30 mm flange, 4 mm sheet |
| Gusset upright | 100 x 4 x 100 mm, plus two small web ribs | same bracket; makes the 100 mm vertical leg |
| Gusset mass | 210 g | steel sheet, 100 mm legs (EST) |
| Toggle clamp lever | ~90 mm overall, 45 mm link arm, 16 x 14 mm rubber pad | common small hold-down clamp (EST) |
| Clamp mass | 90 g each | steel lever + pad (EST) |
| Fixture plate | 160 x 240 x 45 mm, aluminium, top at local z 0.045 | bench holder for a two-hand fixturing task |
| Locator pins | 6 mm dia, at x FX +-0.020, y +0.012 | two-pin locating of the foot |
| Clamp pivots | x 0.245 / 0.315, y -0.037, world z 0.080 | 70 mm apart; open levers do not touch |
| Over-centre flip | 70 deg yaw of the lever; latch threshold 55 deg | declared short-throw stand-in for the real lever arc |
| Seating tolerance | 7 mm in x/y/z; square = foot along x and upright along z (cos > 0.90) | forgiving but observable |
| Staging tray | gusset upright at x 0.28, y +0.10, z 0.045 | within reach, clear of the nest |
| Clamp hand force in sim | declared (4-7 N band) | stated in setup/fidelity, not integrated as a contact force |

## Deliberate simplifications (honest scope)

* **Declared carry.** While a gripper is closed across a part's grasp point the part follows the gripper. This is
  the same "declared mechanism" device the family uses (the check valve's poppet/spring/cap, the cantilever's lid
  coupling). The robot must still reach, close on, carry, orient and release both the gusset and each clamp; the
  locator nest, squareness test and both over-centre flips are real geometry and geometry-derived state.
* **Over-centre as a yaw.** A real hold-down toggle clamp's handle sweeps through a vertical plane and snaps past
  a knee. The sim represents the snapped state by a **70 degree yaw of the lever about its pivot**, detected from
  the gripper's rotation (with a 55 degree latch threshold, an 8.6 degree demo margin). This is a declared
  short-throw stand-in, comparable to the family's 60 degree thread twist: the handle visibly swings, the jaw pad
  sweeps over the gusset foot, and a release short of the threshold sets the `clamp_not_over` cap. It is not a
  simulated knee-joint contact.
* **4-7 N stated, not integrated.** The per-clamp hand force is declared in the brief, setup and fidelity. The
  snap is scored geometrically; no force sensor or contact-force threshold is simulated.
* **Fixture welded and collider-free.** The fixture, locator nest, clamp bases and staging tray are one welded
  visual-only body with no colliders, exactly like the check valve's brass body. The only colliders are the gusset,
  the two clamp levers and the two grippers, so there is no fixture solid for a part to pass through. The jaw's
  press on the foot is represented by the `at_pivot + over-centre` latch rather than a contact force.
* **No tacking or drilling.** The task is the fixturing step only: make the part square and clamp it. Nothing is
  welded or cut in the sim.

## What is NOT simplified

Real product size throughout (no scaling); a real L-shaped gusset that must sit foot-down and upright against the
back wall; a real two-pin locator nest; two separate clamp levers that must each be reached, carried and flipped
past centre; a gusset stage; three failure caps (shifted gusset, clamp not over-centre, dropped part); the 1 s hold;
and both grippers clear and home. Idle and every cap are scored.

## Importance case (explicit, routine not occasional)

This is **routine production work**, not a rare one-off. In fabrication and welding shops a bracket is located and
clamped many times per shift before each tack, drill or grind; workers rotate through stations partly because the
step is so repetitive.

Official US counts (BLS Occupational Outlook Handbook, 2025 employment, pages visited 2026-11-17):

| Occupation | SOC | 2025 jobs | Source |
|---|---|---:|---|
| Welders, cutters, solderers, and brazers | 51-4121 | **437,700** | https://www.bls.gov/ooh/production/welders-cutters-solderers-and-brazers.htm |
| Sheet metal workers | 47-2211 | **131,100** | https://www.bls.gov/ooh/construction-and-extraction/sheet-metal-workers.htm |
| **Combined** | — | **568,800** | sum of the two published figures |

That is **over 500,000 workers** in the two occupations that most directly do this fixturing step.

**Honest limits of the case.** The 568,800 is a US-only BLS OOH employment total for two broad occupations, not a
count of people who perform *this exact* gusset-and-toggle-clamp motion; no public source measures that sub-task's
frequency directly. The "routine, not occasional" character is reasoned from the OOH duty descriptions (repeated
fitting, fastening and fixturing; BLS notes fabrication tasks may be "highly automated and repetitive" and that
sheet-metal workers rotate stations to avoid repetitive-strain injury), not from a measured per-worker task count.
The claim is therefore an explicit, sourced importance argument with its uncertainty stated — not a measured
frequency. The importance case does not affect the mechanics or the score.
