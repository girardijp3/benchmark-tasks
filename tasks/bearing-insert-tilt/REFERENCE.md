# REFERENCE — bearing-insert-tilt

## What the real object is

A **spherical-OD (self-aligning) bearing insert** for a pressed/cast pillow-block or flange housing — the familiar
"bearing in a housing" unit on conveyors, food-processing lines, agricultural machines, hoists and robot joints. The
Schaeffler/SKF/NTN/FYH catalogues call it a *radial insert ball bearing with a spherical outside surface* (or a
spherical plain insert); the enclosing standard is **ISO 9628:2019** for insert bearings and **ISO 3228:2013** for the
matching housings, with the UC/UCP, SY/P2B and similar designation families.

Recognisable features (all modelled here):

* **Convex spherical outer ring** on the bearing, whose radius matches a **concave spherical bore in the housing**.
  This is the self-aligning mechanism: the outer surface and housing bore are spherical, so the unit compensates for
  shaft/installation misalignment. FYH quotes an allowable aligning angle of about **3 deg** for standard ball units
  (1 deg with covers); GGB's self-aligning housings quote **+/- 5 deg**. The brief's "tilt to align, no jamming" is this
  geometry: enter beyond the aligning cone and the rim jams instead of the spherical surfaces seating.
* **Two lubrication holes 180 deg apart in one plane of the outer ring** (Schaeffler), matched to **one grease
  fitting / port in the housing**. To keep the unit relubricatable after replacement, the bearing's grease hole must be
  rotated into registration with the housing port — the "rotate to align" step. The two holes make half-turn phases
  equivalent.
* **A wide inner ring with set-screw / eccentric-collar locking** and seals (modelled here by the bore sleeve and
  index dot; the set screw is not actuated).
* **Replaceability**: NTN states "the bearing used in the NTN bearing unit is replaceable. In the event of bearing
  failure, a new bearing can be fitted to the existing housing." Timken notes the one-piece UCP housing "makes bearing
  replacement easy." So seating a new insert into an existing housing is the real maintenance operation.

**Important honest scope note.** The brief gives a *20 mm spherical-OD* bearing and a pressed housing. The 20 mm is the
**spherical outer diameter** of a small insert (a mini housed-bearing class), not the 47 mm OD of a UC204. The 8 deg
inclined housing axis stands in for the sloped mounting surface (conveyor frames, inclined guards) that makes a tilt
genuinely necessary; the 60 deg port azimuth stands in for a grease fitting that arrives at an arbitrary clocking.
Both are declared geometry within the self-aligning product class, not a claim about one specific catalogue drawing.

## Importance case (routine, 500k+ workers)

This is **routine work of a large maintenance occupation**, not an occasional chore:

* The US Bureau of Labor Statistics, Occupational Outlook Handbook (last modified 27 Aug 2026, visited 30 Oct 2026),
  reports **547,300 jobs in 2025** for *Industrial Machinery Mechanics, Machinery Maintenance Workers, and Millwrights*
  (446,900 + 59,700 + 40,700), with **about 51,900 openings a year** and 14% projected growth to 2035
  (https://www.bls.gov/ooh/installation-maintenance-and-repair/industrial-machinery-mechanics-and-maintenance-workers-and-millwrights.htm).
  Source SOC codes: 49-9041, 49-9043, 49-9044. This single US aggregated occupation already exceeds 500,000 workers;
  add general maintenance/repair workers, robotics technicians and non-US workers and the exposed population is far
  larger.
* BLS lists the occupation's duties as "**Repair or replace broken or malfunctioning components**", "**Clean and
  lubricate equipment or machinery**", "adjust and calibrate", and "perform tests ... to make sure that the machine is
  running smoothly", and notes machinery maintenance workers "must follow machine specifications and adhere to
  **maintenance schedules**." BLS also notes automated conveyors need "**regular care and maintenance**" of "conveyor
  belts, motors, and rollers" — exactly the equipment that uses spherical-OD insert bearings in pillow blocks.
* Bearing replacement is a scheduled preventive/corrective task, done repeatedly across a plant's conveyor and drive
  fleet, so the work is recurring rather than occasional. The queue's cost/prep estimate is **< $20, 1 min**, a cheap
  real-trial route: a small spherical-OD insert and pressed housing from a bearing supplier, a hand press or arithmetic
  force gauge, and a protractor.

This is why the category is **Impact on jobs** with an explicit routine-maintenance case, rather than a one-off gadget
story. (The queue tagged this idea "robot maint." with a weaker importance score; this reference makes the broader,
routine maintenance-technician case explicitly, with the worker count above.)

## Sources for the product / mechanism

* Schaeffler medias, *Radial insert ball bearings*: "Spherical outside surface for the compensation of misalignments",
  "two lubrication holes in one plane of the outer ring offset by 180 deg", matched housings with a spherical bore.
* SKF / SKDIN, *Radial Insert Self Aligning Ball Bearings*: "sphered (spherical) outer surface ... automatically
  compensate for static misalignment when fitted into a compatible housing", UC series, conveyors and food processing.
* FYH, *Ball Bearing Units* catalogue: "self-align inside the housing because of the spherical shape of the outer
  diameter of the bearing insert and the concave shape of the inner diameter of the housing"; allowable aligning angle
  3 deg (standard) / 1 deg (with covers); grease hole and grease fitting shown.
* NTN, *Bearing Units Technical Data*: spherical outer ring / spherical housing bore self-align; replaceable bearing;
  dowel-pin seat in the housing.
* Timken, *UCP 200 Pillow Block Mounted Bearings*: wide-inner-ring ball bearings with self-aligning spherical outside
  diameters, one-piece housing for easy replacement.
* GGB, *UNI Self-aligning Pillow Block Bearing Housing*: self-aligning spheric adjustable to +/- 5 deg.
* ANDE Bearing, *Pillow Block Bearings: Types, Codes & How to Specify*: insert-bearing units use a wide-inner-ring ball
  insert per **ISO 9628:2019** and a housing per **ISO 3228:2013**, self-aligning spherical outer surface, set-screw
  locking.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Bearing class | radial insert ball bearing with spherical outside surface | product class / ISO 9628:2019 |
| Spherical OD | 20 mm (radius 10 mm) | brief |
| Seat | matched concave spherical bore in a pressed housing | product class / brief |
| Self-aligning | spherical surfaces compensate misalignment; ~3-5 deg aligning angle | FYH / GGB catalogues |
| Grease features | two outer-ring lubrication holes 180 deg apart; one housing port | Schaeffler / FYH |
| Fit force | 4-7 N light press | brief |
| Failure modes | jam on steep entry, scoring of the bore, forced press | brief |
| Cost / prep | < $20, 1 min | queue row |
| Occupation size | 547,300 US jobs (2025), ~51,900 openings/yr | US BLS OOH (see above) |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Bore / width / mass | 8 mm / 7 mm / 30 g | plausible small spherical-OD insert; stated mass, not scaled |
| Seat inclination | 8 deg from vertical | stands in for a sloped conveyor mounting; within the aligning class |
| Grease-port azimuth | 60 deg about the seat axis from the default index | arbitrary clocking of the housing fitting |
| Seat radius / clearance | 10.3 mm / 0.3 mm radial clearance | slightly larger than the 10 mm ball for a sliding spherical fit |
| Alignment windows | tilt 4 deg, index 15 deg (mod 180) | within the ~3-5 deg aligning class; two-hole symmetry |
| Jam / score / force thresholds | 6 deg entry tilt, 15 deg misregistered press, 4 mm below flush | stated geometric caps, not measured tolerances |
| Press dwell | 0.35 s | enough for the scorer to register a deliberate light press |
| Declared carry / no-housing-collider | closed gripper carries the bearing; housing visual-only | family device; the skill is orientation + press, not friction |

## Deliberate simplifications (honest scope)

* **Declared carry.** A closed gripper within 16 mm of the bearing's grasp point carries it; the robot must still reach,
  close, tilt, rotate, carry and seat it. This avoids modelling a 30 g steel ring slipping between parallel jaws.
* **Force stated, not integrated.** The 4-7 N press is written into the setup/fidelity fields and detected as a closed
  gripper dwelling 0.35 s at the flush seat; the simulator reads geometry and flags, not contact force.
* **Housing visual-only.** The pressed housing (base, boss, spherical seat, rim, grease port, pocket) has no colliders.
  "No jamming or scoring" is captured by geometric caps (`jammed`, `scored`, `forced`), not by simulated contact damage.
* **No gear train / shaft.** The wide inner ring, set screw and shaft are not simulated; the task is the insert-to-housing
  fit and port registration, which is the service step under test.
* **One arm, fixed fixture.** The housing is welded to a bench plate so this is a one-hand manipulation task.

## What is NOT simplified

Real 20 mm spherical OD and matched 10.3 mm spherical seat (no scaling); a real gravity-free rigid bearing with a true
30 g mass and a cylindrical collider; real spherical geometry, seat inclination, grease port and index hole; the robot
must tilt and rotate the bearing to the correct orientation and press it flush before the score; a 1 s hold is
required; and all three failure caps (jam, score, forced press) are scored. Idle scores 0.
