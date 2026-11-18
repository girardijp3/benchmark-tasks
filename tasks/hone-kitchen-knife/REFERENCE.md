# REFERENCE — hone-kitchen-knife

## What the real objects are

A **200 mm chef's knife** (the named product size; a standard 8 in / 200 mm cook's knife): a broad stainless blade
with a gently curved belly, a thin ground edge, a full tang through a dark three-rivet handle and a bolster at the
heel. Typical mass 180-230 g; blade height at the heel ~50-60 mm, blade thickness ~2-2.5 mm, handle ~120-135 mm.

A **300 mm honing steel** (the named product size; also called a sharpening steel or butcher's steel): a hard
chromium-plated steel rod, smooth or with fine circumferential grooves, of overall length 300 mm, with a ~200 mm
exposed rod and ~100 mm knurled handle. It is held by its handle, often horizontally pointing away from the body
or clamped in a bench holder, and the knife edge is drawn along it at roughly 20°. Honing **realigns** the rolled
edge; it does not grind metal away the way a whetstone does. Chefs hone before prep, often many times per shift.

Sources: general catalogue/retail listings for 200 mm chef's knives and 300 mm honing steels (overall lengths, rod
diameter, handle construction, one-way duty of a honing steel vs a whetstone) plus the brief's stated angle (~20°),
light pressure and routine frequency. No single manufacturer's internal drawing is public, so the dimensions below
are estimates within the stated envelope, each flagged.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Chef's knife blade | 200 mm | named product class |
| Honing steel overall | 300 mm | named product class |
| Honing angle | ~20 degrees | brief |
| Contact | light pressure | brief |
| Duty | before prep, many times per shift | brief (chefs / food service) |
| Occupation scale | cooks, chefs, food prep (SOC 35-2011/2012/2014), 500k+ | BLS OEWS |
| Honing vs sharpening | realigns edge, no metal removed | product class |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Knife mass | 205 g | typical 200 mm chef's knife |
| Blade height at heel | 55 mm, tapering to a point | typical 200 mm chef's profile |
| Blade thickness | 2.5 mm at the spine, ~1 mm at the edge | stainless chef's knife stock |
| Handle | 130 mm long, 22 mm thick, three rivets, steel bolster | standard full-tang handle |
| Honing steel rod | 200 mm exposed, 16 mm diameter (8 mm radius) | 300 mm overall less a 100 mm handle |
| Rod surface | fine circumferential grooves, dark honing band | common commercial grooved steel |
| Steel holder | aluminium clamp under the handle + V-rest under the rod tip | common bench-holder accessory |
| Knife rest | two-cradle wooden rest, 250 mm base | kitchen knife rest |
| Edge contact | 1.5 mm light overlap; 0-3 mm light band, >5 mm heavy | declared contact (no force sensor) |
| Sweep | 180 mm of heel-to-tip travel along the rod; 170 mm counts full | rod length less grasp clearance |
| Grip | 20 mm jaw opening on a 22 mm handle | YAM parallel jaw |

## Deliberate simplifications (honest scope)

* **Steel clamped, not hand-held.** A chef usually holds the steel in the non-dominant hand. Here it is fixed in a
  real bench holder/clamp so the task is single-arm knife manipulation; the angle, sweep, light pressure,
  alternating sides and park outcome are unchanged. The steel and its clamp are one welded, collider-free fixture.
* **Declared carry.** While the gripper is closed on the handle's grasp point, the knife follows the gripper (the
  family's usual declared mechanism). The robot must still reach, close on, orient, sweep and park the knife.
* **Declared pressure.** The rod has no collider; contact is measured as the real geometric overlap of the ground
  edge with the rod's top surface (1.5 mm in the demo). Light (0-3 mm) scores; >5 mm caps at 0.5. No force sensor
  or fluid/metal removal is simulated, as on the family's other declared-force tasks.
* **Two sides = two presentations.** The 20° presentation is made once on each side of the steel (blade yaw +20°
  then -20°); the second sweep is the "alternate sides" half.

## What is NOT simplified

Real product size throughout (no scaling); separate real knife, steel, holder and rest with a real knife mass; the
blade must be yawed to ~20° to score; the edge must travel at least 170 mm along the rod with ≤3 mm overlap on each
side; the knife must be parked on the rest with both grippers home; idle and all three failure caps are scored; and
the whole scored state must hold for 1 s. A stopwatch times only the hold in a real trial.
