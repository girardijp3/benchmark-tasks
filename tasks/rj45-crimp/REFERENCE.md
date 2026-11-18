# REFERENCE — rj45-crimp

## What the real objects are

**The plug.** A crimp-type **8P8C modular plug** ("RJ45" in Ethernet use): a transparent polycarbonate body with eight
flat gold-plated phosphor-bronze contacts parallel to the body, a locking latch, eight rear channels that accept the
conductors, and a strain-relief region that the crimp tool deforms over the cable jacket. Its shape and dimensions are
standardised for telephony by **ANSI/TIA-1096-A / ISO 8877** and for data by **IEC 60603-7-1..7** (up to 600 MHz for
Cat6-class links); wiring practice is **ANSI/TIA-568** (T568A / T568B pin-outs). Recognisable features, all modelled:
eight contacts at **0.040 in = 1.02 mm pitch** (L-Com TDS8PC5 drawing; the modular-connector reference gives the same
1.02 mm contact pitch and a 0.260 in = 6.60 mm insertion height), a **0.460 in = 11.68 mm body width** (same drawing),
a latch, a rear wire entry with eight channels, and a pin-1 index notch. The brief supplies no plug dimensions, so the
**11.6 mm contact-body length** is a stated modelling estimate (datasheets quote 16.76 mm overall for a shielded plug
with strain relief and up to ~22.5 mm for booted plugs); nothing is scaled down for the robot.

**The cable.** CAT6 UTP: a **6.2 mm (0.24 in) jacket** around four twisted pairs of **23 AWG** conductors, each with
about **1.02-1.07 mm** insulated diameter (L-Com gives 0.042 in = 1.07 mm maximum conductor insulation). The four
pairs are untwisted, straightened and ordered for the chosen wiring standard before insertion. T568B order, pin 1 to 8,
is white/orange, orange, white/green, blue, white/blue, green, white/brown, brown; wire 1 carries the red index band in
the model so the robot can see whether it has registered pin 1. 6.2 mm and 23 AWG are the product class; the 10 mm
strip length, 1.05 mm modelled wire OD and the red band are stated modelling choices.

**The tool.** A **ratcheting modular-crimp tool**: a die-cast head with a plug-seat rail, eight contact teeth that drive
into the conductors to displace the insulation and meet the plug contacts, a strain-relief jaw that crimps onto the
cable jacket, a ratchet wheel and pawl that will not release until the handles are fully closed, and two handles. The
model reproduces these features as real geometry: seat rail, eight teeth at 1.02 mm pitch, jaw, ratchet and hinged
handles. The **4-7 N squeeze** is given by the brief; a real ratcheting crimp tool needs considerably more hand force,
so the brief's lower figure is honoured as the declared work range rather than claiming a tool catalogue value.

**Why the sequence matters.** The manufacturer termination procedure is: strip the jacket, arrange the conductors in
the chosen standard, insert them fully until the jacket enters the strain-relief region, seat the plug in the correct
die, and squeeze until the ratchet releases — then verify. Wires out of order fail continuity (a split pair breaks
gigabit links); a plug only partly in the die bends contacts and misses the jacket; a weak squeeze leaves the contacts
un-pierced so an axial pull slides the conductors out of the plug. Those are exactly the three scored caps.

## Importance case (routine, 500k+ workers)

Terminating CAT6 is routine work of a very large workforce, not an occasional chore:

* The US Bureau of Labor Statistics, **Occupational Outlook Handbook (2025 data, visited 17 Nov 2026)**, reports
  **323,600 jobs** for *Network and Computer Systems Administrators* (SOC 15-1244; ~13,400 openings a year) and
  **903,100 jobs** for *Computer Support Specialists* (SOC 15-1230; ~48,700 openings a year) — **over 1.2 million US
  workers** across these two occupations alone
  (https://www.bls.gov/ooh/computer-and-information-technology/network-and-computer-systems-administrators.htm and
  https://www.bls.gov/ooh/computer-and-information-technology/computer-support-specialists.htm). BLS's Employment
  Projections table (2025-35) gives the same 2025 base of 323.6k and 903.1k thousands.
  *Computer network support specialists* (15-1231) add 152,500 jobs. Add structured-cabling installers, electricians
  and non-US workers and the exposed population is far larger than 500,000.
* BLS describes administrators as workers who "**install, configure, and maintain organizations' computer networks and
  systems**" and support specialists as workers who "**maintain computer networks**". Building and reworking patch
  panels, wall jacks and equipment-room cable runs is a recurring, batched duty for these occupations, so a
  termination is one of many done in a shift, not a one-off gadget.
* The queue row tags this "Impact on jobs" with a routine-termination case; that category is retained here with the
  worker counts and an explicit statement that the work is repeated. The brief's measured forces (4-7 N squeeze and
  pull) and a plug/crimper from any cable supplier make a cheap real trial.

## Sources for the product / mechanism

* **IEC 60603-7** parts 7-1/7-2/7-4/7-5/7-7, **ANSI/TIA-1096-A**, **ISO 8877**: the 8P8C modular interface and its
  dimensions; IEC adds the Cat5e/Cat6 high-frequency performance requirements.
* **ANSI/TIA-568** (and ISO/IEC 11801): the T568A / T568B conductor ordering used for Ethernet on 8P8C; T568B is
  modelled.
* **L-Com TDS8PC5** shielded 8x8 plug drawing: 0.040 in / 1.02 mm contact pitch ("7X .040"), 0.460 in / 11.68 mm body,
  0.260 in / 6.60 mm insertion height, 16.76 mm overall with strain relief, 0.042 in / 1.07 mm max conductor
  insulation, TIA-1096 conformity.
* **Wikipedia, "Modular connector"**: 8P8C plug 11.68 mm wide, 6.60 mm insertion height, 1.02 mm contact pitch, eight
  equally spaced contacts; the plug/jack spring interface and the RJ45-vs-8P8C naming distinction.
* **Farnell / L-Com / Digi-Key modular-plug datasheets**: 23-26 AWG solid or stranded conductors, gold-plated contacts,
  polycarbonate housings, and the crimp termination procedure.
* Manufacturer termination guides (generic): strip, order, insert fully, seat in the correct die, crimp until the
  ratchet releases, then verify continuity / pull.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Plug type | crimp 8P8C modular ("RJ45") plug | IEC 60603-7 / TIA-1096-A |
| Contact pitch | 1.02 mm (0.040 in) | L-Com TDS8PC5 / modular-connector reference |
| Plug body width / insertion height | 11.68 mm / 6.60 mm | L-Com TDS8PC5 |
| Cable | CAT6 UTP, 6.2 mm jacket, 23 AWG conductors | product class / brief |
| Wire order | T568B (wire 1 = white/orange) | TIA-568 |
| Squeeze and pull forces | 4-7 N | brief |
| Mechanism | strip → order → insert → seat → squeeze (contacts pierce + strain relief) → pull-test | brief / maker procedure |
| Failure caps | wrong wire order, plug not seated, weak crimp (contact not pierced) | brief |
| Occupation size | 323,600 admins + 903,100 support specialists (2025) | US BLS OOH / Employment Projections |
| Cost / prep | cheap plug + crimper + cable + small force gauge | queue row |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Plug contact-body length | 11.6 mm | stated modelling length; published overall lengths with boot are 16.8-22.5 mm |
| Modelled wire OD / strip length | 1.05 mm / 10 mm | inside the 1.07 mm max insulation; enough to reach the contacts |
| Red index band on wire 1 | visual marker | lets the robot and camera read the T568B registration |
| Crimp seat centre / handle grip pose | (0.270, -0.030, z 0.018) / (0.190, -0.130, z 0.050) | declared fixture geometry inside both arms' reach |
| Order window / seat-square window | 15 deg / 12 deg | enough to demand deliberate alignment, inside the printed-index resolution |
| Insert / seat / handle-reach tolerances | 8 mm / 5 mm / 45 mm | absorb the arm's steady-state pose error while still requiring a real approach |
| Squeeze dwell / weak-crimp dwell | 0.35 s / 0.60 s | enough for the scorer to register a deliberate squeeze or a sustained weak one |
| Declared force mapping | `(0.0939 m - finger gap) * 86 N/m` | two finger joints, full open 0.0939 m; 0.024 m = 6.0 N, 0.028 m = 5.7 N |
| Pull distance / 1 s hold | 12 mm draw-back / 1 s | a deliberate axial verification, timed in `success()` |
| Declared carry / no-collider fixture | closed gripper carries the cable; crimper and handles visual-only | family device; the skill is order + seat + force, not friction |

## Deliberate simplifications (honest scope)

* **Declared carry.** A closed gripper within the grasp point carries the cable; the wired plug is slaved to it with a
  fixed offset. The robot must still reach, close, align, insert, carry and seat. This avoids modelling eight 1 mm wires
  slipping between parallel jaws.
* **Seated lock.** Once the plug is fully in the seat, the assembly is frozen so the squeeze and pull cannot drag it
  out. This stands in for the die gripping the plug and the strain relief holding the jacket.
* **Force stated, not integrated.** The 4-7 N squeeze and pull are read from finger closure, not from contact forces.
  The crimper, its teeth, jaw and handles have no colliders; the caps are geometric/force state flags, as in the
  family's other fit tasks.
* **No electrical model.** Continuity is not simulated; wire order is read from the cable's index geometry and the
  pin-1 notch, and the pull-test is a force + draw-back + dwell test.
* **One hand for the plug, two for the job.** The left arm wires and seats, the right arm squeezes, the left arm pulls;
  the handles animate with the squeeze but are not individually grasped.

## What is NOT simplified

Real 8P8C plug width/pitch/height and CAT6 jacket size (no scaling down); a real gravity-free rigid plug and cable in
their own USD files with the eight colour-coded conductors and the pin-1 index; the crimper seat, contact teeth,
strain-relief jaw and ratchet; the robot must achieve all four ladder steps in order with the three caps live; the 4-7 N
forces are checked against a stated mapping; and the final 1 s pull hold is timed in `success()`. Idle scores 0.

## Near-board item that is explicitly different

Queue neighbour **#1972 (MERGED), "Heat-shrink a sleeve over a crimped cable lug: one hand turns the cable, the other
works a heat gun along it"**, shares the words *crimped cable* but is a **thermal, exposure-over-time** task with a
745 g heat gun, eight stations, rotation, a 3 mm nozzle clearance and a 2 s hold. It has no RJ45 plug, no eight-wire
order, no ratchet squeeze and no pull-test. It is documented in `TWIN_CHECK.md` and is not a twin of this task.
