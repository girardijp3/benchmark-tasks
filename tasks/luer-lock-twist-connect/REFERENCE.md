# REFERENCE — luer-lock-twist-connect

## What the real object is

A **luer-lock connection**: the standard small medical fluid-fitting system defined by **ISO 80369-7** (formerly ISO
594). A **male luer** is a tapered nozzle with a **6% (1:20) taper**; a **luer-lock** male fitting adds a threaded
collar (or shell) around that nozzle. A **female luer-lock port** has the matching tapered socket and an external or
internal thread. The connector is made by pushing the male tip into the female socket and rotating the collar until
it is seated and locked; doing it square and fully seated is what prevents a cross-thread or a loose, leaking joint.

The task uses a **10 mL luer-lock syringe** (a 20 mm barrel, finger flanges, plunger, a threaded collar around the
tip and a ~6 mm-class tapered male tip) and a **20 mm vial adapter** (a 20 mm-neck drug vial with a clip-on adapter
whose top carries the female luer-lock port). The 20 mm measurement is the vial neck finish used by injection vials
(ISO 8362-1); it sets the adapter skirt size, not the luer thread. Seating the tip and twisting the collar until
locked is the daily act of reconstituting a drug, drawing a vaccine or starting an IV admixture.

Sources: **ISO 80369-7** (luer connectors, 6% taper and lock geometry); general catalogue/retail listings for 10 mL
luer-lock syringes and 20 mm vial adapters; the brief's stated **0.3-0.6 N m** lock torque and **~1.5 turns**. **BLS
Occupational Outlook Handbook, 2025** (Registered Nurses, SOC 29-1141, 3,465,400 jobs; Pharmacy Technicians, SOC
29-2052, 471,200 jobs) for the routine-importance case. No single manufacturer's drawing is public, so the internal
dimensions below are estimates within the stated envelope; each is flagged.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Luer system | ISO 80369-7, 6% (1:20) taper | standard |
| Male tip class | ~6 mm, tapered | product class / brief |
| Lock | threaded collar, twist to lock | product class |
| Lock torque | 0.3-0.6 N m | brief |
| Lock twist | ~1.5 turns when done deliberately | brief |
| Vial neck / adapter | 20 mm neck finish | ISO 8362-1 / product class |
| Syringe barrel | 20 mm | brief |
| Failure modes | cross-thread, loose connection | product class |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Syringe | 10 mL, 20 mm OD x 46 mm barrel, 11 mm collar, 8 mm tapered tip, plunger to z +0.057 | typical 10 mL luer-lock syringe; fits a two-jaw grip on the barrel |
| Male tip bore / tip OD | 3.2 mm bore / ~5.6 mm tip | the 6 mm-class taper at its narrow end |
| Vial | 20 mm OD x 48 mm glass, 20 mm crimp and stopper | standard 20 mm injection vial |
| Adapter | 20 mm skirt, female port lip at local z 0.100 (the 0.10 m work height) | clip-on vial adapter with a female luer port |
| Barrel mass | 12 g | representative empty 10 mL polypropylene syringe |
| Seated barrel centre | local z 0.141 (centre = port lip 0.100 + tip drop 0.041) | tip bottom meets the female port lip |
| Staging holder | syringe at x 0.28, y 0.070, centre z 0.056 | within 0.30 m reach, clear of the vial |
| Declared twist | 540 deg nominal (1.5 turns), 500-580 deg window; three 180 deg regrips | a YAM wrist cannot turn 540 deg in one grip |
| Cross-thread / drop | turning > 25 deg off the seat; release outside x 0.10-0.40, \|y\| <= 0.18, z >= TOP - 0.01 | scorer's safety caps |
| Torque in sim | declared (0.3-0.6 N m band) | stated in setup/fidelity, not integrated into contact forces |

## Deliberate simplifications (honest scope)

* **Declared carry.** While a gripper is closed across the barrel, the syringe follows the gripper. This is the same
  declared-mechanism device the family uses (lid detents, ball-valve stem torque): the robot must still reach, close,
  orient, carry, seat and twist, and the seat, cumulative twist and lock are real geometry in the scorer. It avoids a
  round syringe rolling out of a parallel jaw.
* **Torque and thread.** The 0.3-0.6 N m lock torque is the real bench value read with a gauge. The twist is counted
  from the syringe's rotation while seated with a 500-580 degree lock window; it is not a simulated helix with
  contact forces. A real hand also regrips a collar to reach 1.5 turns; the demo does the same in three half-turn
  grips and the count carries across regrasps.
* **Vial and fluid.** The vial, adapter and female port are welded, visual-only geometry with no fluid and no collider,
  as in the family's other bench fixtures. The lock outcome is decided by the real seat and twist state.
* **Fixture fixed.** The vial is clamped so the skill is a one-handed seat-and-twist, not "hold the vial with one hand".

## What is NOT simplified

Real product size throughout (6 mm-class tip, 20 mm barrel and vial neck, no scaling); real 12 g syringe mass; the
male tip must be coaxial in the female port at the seated height before any twist counts; the lock needs a cumulative
~1.5 turns; a cross-thread, a loose connection and a dropped syringe are all scored; the gripper must do the work and
end clear and home; and full score needs a 1 s still hold. Idle and every failure cap are scored.
