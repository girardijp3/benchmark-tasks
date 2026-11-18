# REFERENCE — cotter-pin-clevis

## What the real object is

A **split pin (cotter pin / cotter key)** to **ISO 1234**: a soft low-carbon-steel, zinc-plated wire looped into an eye
at one end, with two parallel legs at the other. It is pushed through a cross-hole drilled near the end of a **clevis
pin** — the plain pin that joins the two prongs of a fork clevis to a rod end — and its legs are then bent apart so the
pin cannot back out. It is one of the oldest and most widely used mechanical retainers, found on tractor and trailer
hitches, brake and throttle linkages, gate valves, agricultural implements and aircraft control linkages.

Recognisable features, all modelled here:

* **An eye loop** at the head (a flat ring in the wire's plane), which stops the pin passing fully through the hole and
  gives a finger something to close on. Modelled as a generated ring on the cotter body.
* **A straight shank** where the two legs are still together, then two legs of **unequal length** (the extended-prong
  form, one long and one short): the long leg can be pushed out from inside first, then the short leg from the side the
  long leg has left. Modelled as a 36 mm green-tipped leg and a 28 mm blue-tipped leg on a 12 mm shank.
* **A cross-drilled clevis pin** in a fork clevis with a rod end. The washer and the pin head are part of the same
  welded fixture. The 3.2 mm cross-hole runs across the 10 mm pin; the 2.8 mm wire slides through it with clearance.
* **A pull test**: the finished joint is pulled to try to back the cotter out. A straight or wrongly bent cotter walks
  out; a cotter with both legs spread to opposite sides holds. This is the service check modelled as the last rung.
* **One-use rule**: on safety work a cotter pin that has been bent and then straightened is scrapped, not reused
  (FAA AC 43.13-1B chapter 7; farm and heavy-equipment service manuals). The scorer therefore penalises a joint that
  is pulled before the legs are spread, rather than letting a straight pin count as secure.

## Importance case (routine, 500k+ workers)

This is **routine work of a large maintenance occupation**, not an occasional gadget task:

* The US Bureau of Labor Statistics, Occupational Outlook Handbook (visited 17 Nov 2026), reports about **1.5 million
  jobs** for *Maintenance and Repair Workers, General* (SOC 49-9071), who "repair or replace ... machinery and
  equipment" and "perform routine preventive maintenance." Clevis-and-cotter linkages are exactly the kind of
  fastener they service on conveyors, doors, gates, lifts and plant equipment.
* The BLS OOH (last modified 27 Aug 2026) reports **547,300 jobs in 2025** for *Industrial Machinery Mechanics,
  Machinery Maintenance Workers, and Millwrights* (446,900 + 59,700 + 40,700), with about **51,900 openings a year**
  and 14% projected growth to 2035 (SOC 49-9041, 49-9043, 49-9044):
  https://www.bls.gov/ooh/installation-maintenance-and-repair/industrial-machinery-mechanics-and-maintenance-workers-and-millwrights.htm
* O*NET adds *Farm Equipment Mechanics and Service Technicians* (49-3041), *Mobile Heavy Equipment Mechanics* (49-3042)
  and *Aircraft Mechanics and Service Technicians* (49-3011, roughly 120,000 US jobs). Cotter pins retain the clevis
  pins on tractor hitches, hydraulic linkages, landing-gear and control linkages — safety-critical joints that are
  checked on schedule.
* Cotter replacement is a scheduled preventive/corrective task done repeatedly across a fleet, so the work recurs.
  The queue's cost/prep estimate is **< $10, 1 min**: a 3 mm split pin, a clevis pin with a cross-hole and a small
  force gauge. This is why the category is **Impact on jobs** with a routine, safety-critical maintenance case.

## Sources for the product / mechanism

* **ISO 1234:2011, *Pin cotters and cotter pins*** — nominal sizes and the split-pin form (eye, two legs, extended
  prong); the 3 mm nominal class.
* **FAA AC 43.13-1B, *Acceptable Methods, Techniques, and Practices — Aircraft Alterations and Repairs*, ch. 7** —
  cotter pins are bent to safety a joint; a bent-then-straightened cotter is not reused.
* **Farm and heavy-equipment service manuals** (e.g. John Deere, CAT) — "install a new cotter pin and spread the legs"
  on clevis-pin hitch and linkage joints.
* **BLS OOH**, *Maintenance and Repair Workers, General* and *Industrial Machinery Mechanics, Machinery Maintenance
  Workers, and Millwrights* (worker counts above); **O*NET** SOC 49-3041, 49-3042, 49-3011 for the equipment mechanics.
* **ISO 2341 / ISO 1234 hardware catalogues** — clevis pins with a cross-drilled hole and matching split pins; cross-hole
  clearance a fraction of a millimetre over the wire.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Fastener class | ISO 1234 split / cotter pin, low-carbon steel, zinc plated | product class |
| Nominal size | 3 mm (brief); 2 mm in the rejected #2060, 10 mm in the rejected #994 | brief / board |
| Wire diameter | 2.8 mm (a few tenths under the nominal size, so it slides the hole) | product class / estimated |
| Cross-hole clearance | 3.2 mm | estimated from the 2.8 mm wire |
| Clevis pin | 10 mm shank, cross-drilled | product class |
| Push / pull force | 4-7 N declared (brief) | brief |
| Failure modes | straight pin backs out; legs must be spread; pulled early = cap | brief / FAA AC 43.13-1B |
| Importance | ~1.5 M maintenance & repair workers; 547,300 machinery mechanics/millwrights | US BLS OOH |
| Cost / prep | < $10, 1 min | queue row |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Shank / leg lengths | 12 mm / 36 mm and 28 mm | plausible extended-prong 3 mm split pin; stated, not scaled |
| Mass | 1.6 g (0.8 g body + 0.4 g per leg) | plausible mass for a 2.8 mm steel wire of that length |
| Alignment window | axis within 10 deg | a visible tug aligns a 2.8 mm wire with a 3.2 mm hole |
| Insertion gate | centre within 6 mm of the hole in x/y/z | enough for the eye band to seat at the near face |
| Spread gate | each leg at least 55 deg, opposite sides | a real cotter is opened ~30-60 deg per leg to retain |
| Pull test | 8 mm outward drag at the tab | a deliberate pull that a straight cotter cannot resist |
| Hover / cap geometry | 5 mm above the hole; 6 mm cotter reach, 12 mm finger reach, 45 mm pad clearance | stated gates for the declared carry |
| Declared carry / bend, no fixture colliders | closed gripper carries and bends; fixture visual-only | family device; skill is align / push / spread / pull |

## Deliberate simplifications (honest scope)

* **Declared carry.** A closed gripper within 16 mm of the eye/shank grasp point carries the cotter, and a closed finger
  at a leg tip bends it; the robot must still reach, close, align, push, carry, release, pull and go home. This avoids
  modelling a 1.6 g wire slipping between parallel jaws. The bend latches, like plastic deformation of soft steel; no
  fatigue cracking or springback is simulated.
* **Force stated, not integrated.** The 4-7 N push and pull are written into the setup/fidelity fields; the simulator
  reads geometry and flags, not contact force.
* **Fixture visual-only.** The clevis, pin, cross-hole, rod end, pull tab and holder have no colliders. "The joint
  holds" and the three caps are geometric state flags, not simulated shear or friction.
* **One arm, fixed fixture.** The clevis is welded to a bench plate, so this is a one-hand manipulation task.

## What is NOT simplified

Real 2.8 mm wire and 3.2 mm cross-hole (no scaling); a real gravity-free three-body split pin with green and blue leg
tips; the eye band, axis and leg bend are real geometry; the robot must align, push through, spread both legs and
pull-test before the score; a 1 s hold is required; and all three failure caps are scored. Idle scores 0.
