# Reference: Shimadzu AUX220 analytical balance

## Product and sources

- **Product:** Shimadzu AUX220, AUX series analytical balance, 220 g x 0.1 mg, 80 mm pan, external-calibration (AUX) model.
- **Specification page:** Shimadzu "AU Series — Specs" (https://www.shimadzu.com/an/products/balances/analytical-balances/au-series/spec.html): capacity 220 g, minimum display 0.1 mg, pan 80 mm dia, dimensions approx. 217 W x 356 D x 338 H mm, weight approx. 7.0 kg. The brochure C054-E049G gives 220 x 330 x 310 mm / 7 kg for the same family; the current specification page's 217 x 356 x 338 mm is used (the larger, current cabinet).
- **Instruction manual:** Shimadzu AUW-D/AUW/AUX/AUY series instruction manual (AUWAUX_E.book / the AUX-220 manual, e.g. https://www.petrame.com/wp-content/uploads/2017/07/shimadzu-analytical-balance-aux-Manual.pdf and https://jpbowlin.com/wp-content/uploads/2016/02/auw-manual.pdf). Section 4.3, "Adjust the horizontal level": "There are two level screws on the front of this balance. The screws grow longer when turned in clockwise direction from above and they grow shorter when turned counter-clockwise. Adjust the level screws until the bubble in the level indicator is in the middle of the red circle." Section 2.1 and the rear-view drawing show the level screws at the front corners and the single fixed foot at the centre rear; the WIND UW/UX manual states the same three-point arrangement explicitly: "a single fixed foot at centre rear ... and two level screws on left and right at the front".
- **Parts list:** the level screw is Shimadzu part 321-62884 (A-2 Standard Accessories and Maintenance Parts List); the manual does not publish its thread, diameter or travel.
- **Shimadzu's own leveling notes:** "Level adjustment, particularly important in installing an analytical balance, can be performed with ease using an easy-to-view level gauge at the front of the balance and large level screws" (C054-E049G brochure). The AUX series has a conventional circular bubble, not the motorized leveling of the higher models.

## Recognisable features modelled

1. 217 x 356 mm footprint, 338 mm overall height, 7.0 kg: a light-grey housing on a dark die-cast base, with rounded corners and a chamfered top edge.
2. A large glass draft shield (three sliding doors: front, left, right) with a dark frame, corner posts, top rails and a top handle/vent boss.
3. An 80 mm stainless weighing pan on a pan support inside the chamber, visible through the glass.
4. A front key panel with a 50 x 17 mm dark LCD with four pale-green digit groups and six rounded keys.
5. A 30 mm circular level at the front right: black bezel, clear glass well, fine ring and a red 5 mm centre circle; the green 7 mm bubble.
6. Two large knurled front level screws (34 mm across, 26 mm tall, 24 flutes, a red index mark) under the front corners; each has a stainless shaft and a black rubber pad.
7. One fixed 30 mm rubber foot at the centre rear, under the base; the three-point support.
8. A dark label/legend plate on the front, a power socket and a plain model plate on the back.
9. The front feet protrude 7 mm ahead of the base's front face and the base has a 40 mm front relief above them (the level screws are at the very front corners, as in the manual drawing).

## Published values vs estimates

| Quantity | Value in the task | Status / source |
|---|---|---|
| Capacity / readability | 220 g / 0.1 mg | published (Shimadzu AU specs) |
| Body W x D x H | 217 x 356 x 338 mm | published (Shimadzu AU specs) |
| Net weight | 7.0 kg | published (Shimadzu AU specs) |
| Pan diameter | 80 mm | published |
| Leveling | two front level screws + one fixed centre-rear foot | published (manual section 4.3 and drawing) |
| Level screw direction | clockwise (from above) extends the screw and raises that corner | published (manual 4.3) |
| Level screw size | 34 mm dia x 26 mm tall wheel | **EST** from the manual's "large level screws" and the drawing; part 321-62884 |
| Level screw thread | 1.75 mm per revolution | **EST** (M12 x 1.75 class; Shimadzu does not publish it) |
| Level screw travel | 22 mm (30 to 52 mm extension from the base underside) | **EST** from the drawing's exposed screw length |
| Level screw torque | about 0.05 N m (low) | **EST** / the task brief's real low torque; the thread is self-locking under the ~23 N front-foot load |
| Base height / draft-shield assembly | 60 mm / 236 mm (226 mm glass + 10 mm floor) | **EST** from the 338 mm overall and the drawing proportions |
| Foot extension (nominal) | 40 mm | **EST** from the drawing; the published 338 mm includes the feet |
| Foot positions | front x -168 mm, y +-85 mm; rear x +150 mm | **EST** from the drawing's footprint |
| Body COM | 95 mm above the base underside | **EST** (base heavy, glass shield light); inertia from a 217 x 356 x 298 mm box |
| Body / foot / wheel mass | 6.70 / 0.12 / 0.03 kg (total 7.00) | **EST** split of the published 7.0 kg |
| Level vial | 30 mm dia, 4 mm of bubble travel per degree, 5 mm red centre | **EST** from a typical 30 mm circular level (30 arcmin per 2 mm, radius ~229 mm) |
| Bubble centre window | 0.5 mm (0.125 deg) | **EST** from the vial's 5 mm centre circle and the 0.1 deg full window |
| Initial tilt | pitch 0.55 deg, roll 0.55 deg | task start (from the front extensions 43.90 / 42.20 mm) |
| Bench | the station table (0.7475 m top) | the real lab bench |

## What was simplified, and why it is still honest

- **The screw is declared, not cut in contact.** The real level screw is a 1.75 mm/rev thread in the base. In the sim the wheel is a real revolute link and the foot a real prismatic link; while a gripper holds the wheel (a 20 N pinch, the pads 34 mm apart), the wheel's angle follows the hand's yaw and the foot's slide follows at 1.75 mm a revolution. The body is free on the three contacts. A 1.75 mm pitch is fine enough that the thread's helix is not resolved by the solver; the declared joint keeps the exact real relation. The wheel cannot be turned without gripping it, and the foot's reaction is carried by the free body.
- **The rubber pad is a 12 mm sphere.** A real 24 mm rubber pad conforms to the bench; a point contact is statically determinate and keeps the three feet from fighting. The rear foot is a 15 mm sphere. The body's tilt is read from its real quaternion, not from the pad geometry.
- **The bubble is a visual body.** It is a separate gravity-free, collision-free 7 mm disc placed on the vial's glass from the body's real tilt (4 mm/deg). The score never reads the bubble; it reads the body's pitch and roll. The bubble is the real-world check on the video and on the bench.
- **The glass draft shield is visual only.** The robot works at the front feet, below the base; the shield is not touched. The base, front panel and shield floor are colliders. No visual hides a collider gap.
- **The scale is true size.** The body, pan, feet, wheels, vial and the 7 kg mass are the real product's; the only reductions are the contact simplifications above and the declared screw. Nothing that decides the task is lightened or loosened.
