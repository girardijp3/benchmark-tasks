# Reference: Dyson V6 Cord-free stick vacuum — dust bin release and empty

## Product and sources

- **Product:** Dyson V6 Cord-free handstick vacuum, with a 0.4 L clear bagless bin and a red bin-release button.
- **Dyson support specification** (support.dyson.co.nz, `.../dyson-v6/sv03-rv4-dyson-v6-cordfree/specifications`): Bin volume **0.4 litres**; weight **2.04 kg**; dimensions **211 x 1180 x 208 mm (H x W x D)**; 2 Tier Radial cyclones; 28 AW / 100 AW boost; washable lifetime filter.
- **Manual listing** (manua.ls, Dyson V6 Cord-Free Extra): "bagless dust container ... dry dirt **capacity of 0.4 liters**."
- **Emptying procedure** (gizmocleaning.com, "How to Properly Empty Your Dyson Stick Vacuum V6"): locate the **red "bin release" button**, press it so the dustbin separates from the body, hold the bin over a trash bin, press the **bottom release** lever so dust falls out, then reattach **until it clicks**.
- **Occupations:** U.S. BLS OEWS May 2024 — Maids and Housekeeping Cleaners (SOC 37-2012, about 900,000 jobs) and Janitors and Cleaners (SOC 37-2011, about 2.2 million). Emptying a stick vac's bin is a routine part of the work.

## Recognisable features modelled

1. The pale cordless motor/body module on the bench, with a violet motor cap and a short wand and floor head behind it.
2. The clear 0.4 L dust bin seated on the front of the body, with a red marking and a top carry bar.
3. The **red spring bin-release button** on a thin bracket at the front top of the body.
4. The **hinged bottom dump flap** that opens to drop the dust.
5. A household waste bin next to the vac.
6. Six loose pieces of debris (dust grit, a chip and twig fragments) inside the bin.
7. The real sequence: press the button, pull the bin out, empty it into the waste bin, refit and click it latched.

## Published values vs estimates

| Quantity | Value in the task | Status / source |
|---|---|---|
| Whole vacuum | 211 x 1180 x 208 mm, 2.04 kg | published (Dyson support) |
| Bin capacity | 0.4 L | published (Dyson support / manual) |
| Button colour / action | red, press to release | published (manual / emptying guide) |
| Module on the bench | 120 x 150 x 195 mm, 2.04 kg | EST: the upper body module only; the wand/head are props |
| Clear bin | 70 x 70 x 85 mm (0.42 L), 0.14 kg | EST from the 0.4 L capacity; walls 4 mm |
| Button press | 10 mm travel, 600 N/m = 6 N at the stop | EST; 6 N is inside the 4-7 N brief |
| Bottom flap | hinge at the +x bottom rim, -100 deg open | EST from the "bottom release" guide |
| Debris | 6 pieces, about 0.06 kg | EST |
| Waste bin | 240 x 240 x 35 mm | household bin class, EST |
| Reach / height | button at x 0.292, z 0.215; bin seat at x 0.29 | task layout, inside the 0.3 m height gate |
| Run limit | 1,200 steps (60 s at 20 Hz) | task |

## What was simplified, and why it is still honest

- **Only the upper module is simulated.** The wand and floor head are fixed visual props behind the body; they are not part of the release/empty sequence. The published 2.04 kg is carried by the simulated module, not reduced.
- **The bottom tab is a declared mechanism.** The real bin needs its bottom red lever pressed. The sim declares the flap to open whenever the held bin is over the waste bin and closes it as soon as the dust has landed. The flap, its hinge, the bin's free articulation and the six debris are real bodies; the declared switch stands in for the finger press, as other tasks declare couplings and detents.
- **No interlock pin.** A flat cradle shelf and a forward pull hold the bin at the start; the button must first be pressed past 5 mm or the pull sets the `forced` cap. This avoids the pin-in-hole jam that blocked the earlier canister task.
- **The bin seat is loose.** The final pose is scored within 12 mm and 8 degrees, not a microscopic tolerance, because a real bin clicks home against a gasket rather than a machined fit.
- **Dust is discrete pieces.** Six light rigid bodies stand in for fine dust; they fall through the real flap into the real waste bin and are scored by their final poses.
- **Primitives and 2-4 mm shells.** The body, bin and waste bin are boxes and cylinders with thin walls; no mass, size or force is lightened to make the demo pass. The top-view camera is a real top camera; no cutaways or hidden geometry.
