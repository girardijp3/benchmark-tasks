# Reference: STANLEY 1-94-738 18 in. metal cantilever tool box

## Product and sources

- **Product:** STANLEY 1-94-738 18 in. Metal Cantilever Tool Box (yellow/black), five compartments, metal latches, two carry handles.
- **Manufacturer page:** https://asia.stanleytools.global/product/1-94-738/stanley-18-metal-cantilever-tool-box — "Cantilever box has 3 or 5 Compartments ... provided with 2 handles"; specifications: capacity 40 L, latch material metal, product weight 4.8 kg, weight capacity 35 kg, one internal tote tray, metal handle.
- **Retail listings:** Klium (https://www.klium.com/en/stanley-1-94-738-tool-case-metal-cantilever-5-drawers-38451) and Toolnation (https://www.toolnation.com/stanley-1-94-738-tool-case-metal-cantilever-5-drawers.html) give the outside as 20.8 x 20.8 x 45 cm (460 x 208 x 208 mm). Amazon lists "Double latch for greater strength; tray swing out for easy tool access; double handle" (https://www.amazon.com/Stanley-1-94-738-Metal-Tray-Multicolor/dp/B005IW9J3Y).
- **Product photos:** https://asia.stanleytools.global/ASIA/PRODUCT/IMAGES/HIRES/1-94-738_1.jpg (closed, carry bails up, silver straps on the end) and `_2.jpg` (open, the trays fanned out, the left tray lower and the right higher, the wire top-tray handle).
- **Occupations:** U.S. Bureau of Labor Statistics, OEWS May 2024: Maintenance and Repair Workers, General (SOC 49-9071, about 1.5 million jobs, https://www.bls.gov/oes/current/oes499071.htm) and Automotive Service Technicians and Mechanics (SOC 49-3023, https://www.bls.gov/oes/current/oes493023.htm). Mechanics and maintenance technicians open a cantilever box on the bench for tools and parts many times a day.
- **Latch force:** the 4-7 N brief for the latch pull at the knob. The real spring is not published; the sim's 0.25 N m/rad over the 35 mm plate is 7.1 N at the knob (EST).

## Recognisable features modelled

1. 460 x 208 x 208 mm steel body, 4.8 kg, five compartments: a large black base tray, two fixed nested trays inside it, and two yellow moving trays.
2. Two yellow moving trays on silver parallelogram straps with rivets: the left tray sits lower, the right one 40 mm higher, and both fan out level when the lid is lifted.
3. The yellow lid with a black wire tray handle (a 60 mm grip bar) and two black carry bails folded down at the ends.
4. Two metal draw latches on the front corners: a 35 mm steel plate with a 10 mm knob, its hook under a keeper on the lid.
5. The black base with a yellow STANLEY label plate on the front, and four black rubber feet.
6. Shallow 2 mm tray walls; the 150 mm adjustable wrench in the left tray, the 120 mm screwdriver in the right, the 300 mm claw hammer in the upper fixed tray.
7. The open state: the trays spread to y +-0.31 m, the lid at 310 mm, the base and the fixed trays visible between them.
8. The real open sequence: release both latches, then lift the lid's tray handle; the trays fan out in one motion.
9. The two carry bails and the wire handle are distinct: the bails are for carrying, the wire handle is for opening.

## Published values vs estimates

| Quantity | Value in the task | Status / source |
|---|---|---|
| Body W x D x H | 460 x 208 x 208 mm | published (Stanley/retailers; the Stanley page lists 450 x 208 x 208 with the length as "height") |
| Net weight | 4.8 kg | published (Stanley) |
| Compartments | 5 | published (Stanley) |
| Metal latches | 2 | published ("metal latch", "double latch") |
| Carry handles | 2 bails | published ("2 handles") |
| Capacity / load | 40 L / 35 kg | published (Stanley) |
| Base height | 105 mm | EST from the product photo |
| Lid thickness / mass | 13 mm / 0.40 kg | EST |
| Box mass split | base 3.20, arms 0.30, wings 0.80, lid 0.40, latches 0.04 = 4.74 kg | EST split of the 4.8 kg |
| Moving trays | 160 x 150 x 30 mm, 2 mm walls | EST; the real compartment sizes are not published |
| Fixed nested trays | 420 x 180 x 25 mm | EST |
| Right tray height | 40 mm above the left | EST from the open product photo |
| Arm length / pivot | 149 mm; y +-0.225, z 0.107; -130/+135 deg open | EST from the photo and the 208 mm box |
| Latch plate / knob | 35 mm plate, 10 mm knob, 12 mm pull, 25 deg swing | EST from the photo |
| Latch detent | 0.25 N m/rad = 7.1 N at the knob | EST from the 4-7 N brief |
| Lid lift / detent | 115 mm; holds open above 100 mm | EST; the open photo shows the trays clear of the lid |
| Tools | 150 mm wrench 0.35 kg; 120 mm screwdriver 0.15 kg; 300 mm hammer 0.50 kg | real sizes; typical masses |
| Tray positions (closed) | left 100-130 mm, right 140-170 mm above the bench | task layout |
| Open tray positions | left y +0.161..+0.311 at 225-255 mm; right y -0.331..-0.181 at 257-287 mm | task layout |
| Run limit | 2,400 steps (120 s at 20 Hz) | the Dream evaluation budget; the demo needs 359 steps |

## What was simplified, and why it is still honest

- **The lid-to-arm coupling is declared.** The real trays are linked to the lid by the parallelogram straps; the sim drives the arm joints from the lid's lift at the measured ratio (130/135 degrees over 110 mm), while the arms, trays and level joints are real bodies. This is the same kind of declared mechanism as the balance task's screw (the wheel's turn drives the foot at the real pitch); the arm and tray geometry, masses and hinges are real.
- **One physical arm chain per side, with two visual straps.** The real parallelogram has two parallel straps on each side. The sim uses one revolute arm per side (0.149 m) plus a second revolute joint at the tray that cancels the arm's roll; the second strap is visual. The tray stays level, and the visual open/closed poses match the product photo. The leveling joint stands in for the second parallel arm.
- **The two fixed nested trays are fixed to the base.** They are the other two of the five compartments (the model's base + two fixed + two moving = five); the real trays may be removable, but the task's scored trays are the two moving ones and the tools in them.
- **The latch spring is a declared detent.** The real latch's spring force is unpublished. The sim uses a 0.25 N m/rad revolute drive (about 7 N at the knob) and latches open once the plate passes -10 degrees; the plate, knob and hook are real bodies and the hook physically blocks the lid's keeper until it swings out. A lid forced up with a latch closed sets the `forced` cap.
- **The tools are simple primitives.** The wrench is a bar and a head; the screwdriver a handle and a blade; the hammer a handle and a head. Their real lengths and typical masses are used; the trays are shallow and the tools are free bodies. The 35 kg published load is not tested; the task carries 1.0 kg of tools.
- **Steel shells are 2 mm.** The real trays are thinner; 2 mm boxes keep the shallow tray collisions stable at 240 Hz. The box's 4.8 kg, the 460 x 208 x 208 mm size and the five-tray layout are not reduced. No mass, size or force is lightened to make the task pass.
- **The glass/wood?** None: this box is all steel and rubber; no visual-only part hides a collider. The lid's wire handle and the carry bails are visual/collider as appropriate; the handle bar is a collider for the grip.
- **The top view is a real top camera; no cutaways.** The wrist cameras at the latch and handle show the pads on the knobs/bar and the box surfaces; no gripper or forearm passes through a solid.
