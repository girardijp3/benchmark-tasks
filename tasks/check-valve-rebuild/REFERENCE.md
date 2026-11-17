# REFERENCE — check-valve-rebuild

## What the real object is

A **20 mm brass spring check valve** (nominal 3/4 in / DN20). Recognisable features: a hex or round brass body with
a female-threaded inlet and outlet; an internal machined **seat**; a **poppet/disc** with a soft elastomer sealing
face and a guide stem; a **compression spring**; and a **threaded bonnet/cap** that closes the body and sets the
spring preload. It permits flow from inlet to outlet (pressure lifts the poppet off the seat against the spring) and
blocks reverse flow (pressure pushes the poppet onto the seat and seals). Rebuilding one is a common plumbing and
maintenance job: remove the cap, replace the worn poppet and spring in the correct order and orientation, thread the
cap back, and flow-test it.

Sources: general catalogue/retail listings for 20 mm brass spring check valves (body material, nominal size, seat,
poppet, spring, threaded cap, one-way operation) and the brief's stated force/torque envelope (4-7 N spring,
0.3-0.6 N m cap). No single manufacturer's internal drawing is public, so the internal dimensions below are
estimates within the stated envelope; each one is flagged.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Nominal port / body | 20 mm brass spring check valve | product class |
| Operation | one-way; poppet lifts for forward, seals for reverse | product class |
| Spring force | 4-7 N | brief |
| Cap torque | 0.3-0.6 N m | brief |
| Parts | seat, poppet/disc, spring, threaded cap, body | product class |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Body diameter x height | 28 mm x 40 mm | typical 20 mm brass body; fits the 4-7 N internals |
| Cap across flats x height | 26 mm x 14 mm | hex bonnet for a 20 mm valve; generous parallel-jaw grip |
| Poppet disc dia x thickness | 16 mm x 3 mm | seats in a 12 mm bore with a 2 mm land each side |
| Poppet seal face | 14 mm dia elastomer | the real sealing annulus, on the underside |
| Poppet guide stem | 5 mm dia x 14 mm | locates the spring coaxially |
| Spring | 14 mm OD x 24 mm free, ~6 mm compressed | gives the 4-7 N band at the cap |
| Seat bore / ledge | 12 mm / z 0.062 | the sealing land the poppet meets |
| Fixture | 90 x 90 x 50 mm aluminium | bench holder, fixes the valve for a two-handed rebuild |
| Part masses | cap 30 g, poppet 8 g, spring 4 g | brass cap, steel disc, steel spring (EST) |
| Staging pockets / cap peg | poppet y 0.045, spring y 0.070, cap peg y -0.045 | within reach, clear of the body |
| Cap thread | 60 degrees effective twist, 6 mm button press | a declared short thread stand-in (a real cap turns further); left wrist reaches 70 deg |
| Spring force / cap torque in sim | declared (4-7 N / 0.3-0.6 N m band) | the force/torque is stated in setup/fidelity, not integrated into dynamics |

## Deliberate simplifications (honest scope)

* **Declared carry.** While a gripper is closed across a part's grasp point, the part follows the gripper. This is
  the same "declared mechanism" device the family uses (e.g. the cantilever's lid-to-arm coupling, the ball valve's
  viscous stem torque): the robot must still reach, close on, orient, carry and place each part, and the seat, spring,
  cap thread and two test buttons are real geometry. It avoids a spring coil slipping through a parallel jaw, which
  is not the skill under test.
* **Threads and torque.** The cap thread is a declared 60 degree twist with a 20 degree window; the 0.3-0.6 N m
  cap torque and 4-7 N spring force are stated in the brief/fidelity rather than simulated as contact forces.
* **Flow test.** The two test buttons stand in for applying pressure at the inlet/outlet ports; no fluid is
  simulated. The one-way outcome is decided by the real seat/seal geometry and the poppet's orientation and position.
* **Fixture fixed.** The valve is clamped in a bench fixture so the rebuild is a two-hand manipulation task, not a
  "hold the body with one hand" task.

## What is NOT simplified

Real product size throughout (no scaling); real separate seat, poppet, spring and body with true masses; the poppet
must be seated upright on the seat; the spring must be present; the cap must be twisted and seated, not forced; every
part must be reached and grasped by a YAM arm; and the one-way flow outcome is checked against the seat/seal state
with a 1 s hold. Idle and all five failure caps are scored.
