# REFERENCE — index-plunger-fixture

## What the real object is

A **16 mm class spring / hand index plunger** used to lock a rotary fixture plate at one of a ring of index holes.
Recognisable features: a threaded body; a spring-loaded, slightly tapered **pin** that seats in an index hole or a
separate locating bushing; a **knob** (ball knob or hand knob) that is pulled to retract the pin against the spring; and
sometimes a setscrew notch or a 90 degree lock to hold the pin retracted. To index the fixture: pull the knob out, turn
the plate to the next hole, release the knob and let the spring drive the pin home. The taper eases alignment; the
spring (here 4-7 N) keeps the pin seated against vibration.

Sources: Carr Lane Manufacturing product pages and catalogue for index plungers and hand-retractable plungers
(`carrlane.com/product/spring-loaded-devices/index-plungers` and
`.../hand-retractable-plungers/...`), plus the general product class (Kipp / MISUMI / Jergens detent and index
plungers). The exact force/torque envelope and the 16 mm knob come from the brief/gate. No single manufacturer's
drawing of this exact fixture is public, so the fixture dimensions below are estimates within the stated envelope;
each one is flagged.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Plunger size class | 16 mm (M16 thread class hand-retractable / index plunger) | product class; brief/gate |
| Operation | pull knob to retract pin, rotate to next stop, release to seat pin | product class |
| Pull / spring force | 4-7 N | brief |
| Mechanism parts | threaded body, spring, retractable pin, knob, index hole / bushing | product class |
| Index ring | multiple holes, one per stop, equal angular pitch | product class |

Catalogue cross-check (not the exact model): Carr Lane **CLM-10-HRPK-L** hand-retractable plunger, M16 x 1.5 thread,
8 mm plunger diameter, F knob diameter 31 mm, initial/final end force 2.5 / 6.7 lbf (about 11 / 30 N); Carr Lane
index plungers in 12 mm and 20 mm pin sizes (20 / 30 mm bodies). The brief's 4-7 N band is at the light end of that
range, consistent with a lighter spring in the same family.

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Knob diameter | 16 mm | brief/gate grasp feature (full-size M16 hand knobs are larger, ~21-31 mm) |
| Pin diameter | 7 mm | fits the 8 mm class plunger family with light clearance |
| Plate diameter x thickness | 150 mm x 20 mm | a small rotary index / fixture table with an 8-hole rim ring |
| Index holes | 8 x 7 mm radial holes at 45 degrees, radius 0.072 m | equal divisions; next stop is one pitch (45 deg) |
| Central hub knob | 24 mm dia x 20 mm tall | a grip feature on the rotation axis; turning it turns the plate about its centre |
| Pull travel | 16 mm | enough for the pin tip to clear the plate rim before the turn |
| Pedestal | 150 x 150 x 90 mm | bench fixture that holds the plate and plunger bracket |
| Part masses | plate 600 g, plunger 120 g | aluminium plate, steel plunger (EST) |
| Stop / align / overshoot tolerances | 8 deg at the stop and at release; 12 deg overshoot | gives a firm detent-like window without hiding a missed stop |
| Spring force in sim | declared 4-7 N band | stated in setup/fidelity, not integrated as a contact force |

## Routine importance (explicit case, 500k+ workers)

Indexing a fixture with a spring plunger is a **routine, repeated** manipulation in machining, fixture building and
machine maintenance: the operator pulls the pin, turns the index plate to the next station and releases it, once per
station or per part changeover — not an occasional or one-off action. A conservative occupation-level proxy from the
**US Bureau of Labor Statistics, Occupational Employment and Wage Statistics (OEWS), May 2024**:

| Occupation | SOC | Employment (May 2024) |
|---|---|---:|
| Machinists | 51-4041 | 298,790 |
| Industrial machinery mechanics | 49-9041 | 421,940 |
| Tool and die makers | 51-4111 | 56,930 |
| **Sum (proxy)** | | **777,660** |

Sources: BLS OEWS May 2024 national data (`https://www.bls.gov/oes/tables.htm`, table 1 and the OEWS profiles at
`data.bls.gov/oesprofile`), the BLS Economics Daily "A look at manufacturing occupations for National Manufacturing
Day" (29 Sep 2025), and the BLS news release `ocwage_04022025.pdf`. Tool and die makers are defined by BLS as fitting
and assembling "dies, cutting tools, jigs, fixtures, gauges" — the fixture-building occupation. Industrial machinery
mechanics repair and adjust production machinery that uses such fixtures.

**Honest caveat:** these are occupation totals, not a count of workers who perform this exact pull/turn/release
sub-action; no public source measures that specific number. The proxy establishes that well over 500,000 workers are
in occupations where this is routine work; it is not a direct measurement of the sub-action's frequency.

## Deliberate simplifications (honest scope)

* **Declared carry and rotation.** While a gripper is closed across the knob, the plunger translates axially with that
  gripper; while a gripper is closed on the hub, the plate yaw follows that wrist's yaw. This is the same declared
  mechanism device the family uses (the cantilever's lid coupling, the ball valve's viscous stem torque, the
  cycloidal disc's wrist-driven phase). The robot must still reach, close on, pull, turn and release both parts, and
  the pull distance, the 45 degree turn, the hole alignment and the release are real geometry.
* **Spring force.** The 4-7 N pull force is declared in setup/fidelity, not integrated as a contact force; the
  mechanism detects pull distance and the order of pull / turn / release.
* **Holes are visual.** The eight rim holes and the pin are real geometry but the seat is not a simulated contact fit;
  alignment is measured as the plate angle to the nearest stop. This avoids relying on solver depenetration to decide
  whether a 7 mm pin dropped into a 7 mm hole.
* **Fixture fixed.** The pedestal is clamped to the bench so this is a two-hand index task, not a "hold the fixture"
  task.

## What is NOT simplified

Real product size throughout (no scaling); a separate plate and plunger with true masses and a real ring of eight
holes; the pin must be pulled clear before the turn; the plate must turn exactly one pitch; the pin must be released at
a stop, not between holes; every grasp is made by a YAM arm; the three failure caps are scored; and the final indexed
state must hold for 1 s with both grippers clear and home.
