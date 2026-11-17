# Reference: the real products in thermal-paste-removal

The scene is a PC-repair bench with a true-size AM4-class desktop CPU in its socket, a real 91% IPA lint-free wipe and
a real grey thermal-paste patch on the IHS. This file records the catalogue/measurement facts, the 5-10 recognisable
features modelled, and every estimate. Sources were checked on 2026-11-16; the sim values are in `task.json` fidelity
and the README.

## AMD AM4 Ryzen-class CPU (the IHS and package)

- **Package:** 40 x 40 mm, 1331 PGA pins, Socket AM4; the socket's cooler mounting holes are 54 x 90 mm. (Wikipedia,
  Socket AM4: "Processor dimensions 40 mm x 40 mm", "Contacts 1331", "PGA"; Socket Geek AM4: "Package size 40 x 40
  mm", "Contacts 1,331", "Cooler mounting holes 54 x 90 mm".)
- **IHS:** 37.5 x 37.5 mm square, nickel-plated copper, about 3.6 mm thick; the top of the IHS sits **7.35 mm above the
  motherboard PCB** on a Ryzen 5 1600 in a Gigabyte AX370 Gaming K5 (a caliper measurement posted by Nobu,
  [H]ardForum thread "AM4 Socket Dimensions Help Needed", 2017-04-24; the same thread gives the Intel LGA115x stack
  7.781 +/- 0.335 mm for comparison). The IHS is a simple flat lid on AM4, unlike the notched AM5 lid (Igor's Lab,
  "Myths and facts about thermally conductive pastes", 2025; "AMD Ryzen 7000 heatspreader and cooling analysis", 2022).
- **Recognisable features modelled:** the square nickel lid with a 0.15 mm top chamfer; the thin copper foot ring; the
  laser-etched centre area (no readable text); the pin-1 triangle at one corner; the dark-green substrate, 1.25 mm
  wider than the lid on each side, with a gold edge trace; the tiny SMDs at the substrate corners; the 1331 pins on
  the underside (drawn but hidden in the socket).
- **Socket:** a black 54 x 54 mm AM4 frame (a class estimate; the 54 x 90 mm cooler pattern is published), a 40.2 mm
  opening, a steel load plate around the substrate and a steel lever along the +x side with a plastic grip, the four
  cooler holes with brass inserts and a white pin-1 silk triangle. The pins are in the socket under the CPU; the
  visible no-touch zone is the frame, load plate, lever and the substrate ring.

## Motherboard and bench

- **ATX board:** 305 x 244 mm, 1.6 mm PCB, on nine 6.5 mm brass standoffs (ATX form factor; 6.5 mm is the common
  standoff height). Dark-green solder mask, white silkscreen, four DDR4 DIMM slots, a 24-pin ATX header, two finned
  VRM heatsinks, a chipset heatsink, two PCIe slots, an M.2 slot and screw, six 6 mm electrolytic capacitors, a CPU
  fan header, the rear I/O shield, traces and a pin-1 mark. The bench: a 500 x 440 x 3 mm dark anti-static mat, a
  80 x 80 x 3 mm blue wipe tray with a foil packet, a grey waste tray, a 4 g paste tube, the CPU's box, a Phillips #2
  screwdriver, a folded-wipe packet and an anti-static bag.

## Thermal paste

- **Product class:** Arctic MX-4 (4 g tube, density 2.5 g/cm3; the maker's product page). A usual desktop application
  is 0.3-0.5 g (SunbeamTech, "How much thermal paste on CPU"; Delidded, "How much thermal paste on a CPU"). Most of it
  leaves with the cooler; the residual film on the IHS after lifting a cooler is a 30 x 20 mm oval, about 0.05 mm
  thick and **0.08 g** in this task (an estimate: 30 x 20 x 0.05 mm = 0.030 cm3 at 2.5 g/cm3 = 0.075 g, rounded to
  0.08 g). The colour is a dark grey (0.345, 0.340, 0.320 in the render).
- The real job's check is the after-photo/loupe: a uniform, slightly reflective nickel lid with no grey streaks or
  film. The sim's 122-cell coverage layer is a stand-in for that area check.

## 91% IPA lint-free wipe

- **Wipe:** a Kimtech W4 dry cleanroom wipe, 12 x 12 in (33330), polypropylene, low lint for ISO Class 4; or the
  Kimtech 782100 21 x 20 cm 2-ply lint-free wipe. The task folds one to **50 x 50 mm, 2 mm thick**, with a
  **24 x 24 mm** contact face standing 0.8 mm proud of the flange. A folded Kimwipe is the usual tool in the thermal
  paste guides: "91% isopropyl alcohol with Kimtech Kimwipes... unlike most paper towels, they do not leave behind lint
  particles" (Nathan Cheek, MacBook Pro thermal paste, 2019); "Lift the bulk with a dry lint-free wipe, then clean what
  is left with 90 to 99 percent isopropyl alcohol on a fresh wipe... let the surface dry" (Delidded, "Cleaning thermal
  paste with isopropyl alcohol", 2026).
- **IPA:** 91% isopropyl alcohol (USP), 1.5 mL on the folded wipe. The 91% grade is the task's; 90-99% is the guides'
  recommendation. 91% IPA flashes off in seconds at room temperature; the real trial lets the lid dry 2-5 minutes
  before fresh paste.
- **Applicator:** the only non-catalogue part. A 40 x 40 x 6 mm dark-blue backing and a 20 x 20 x 26 mm grip bar,
  25 g (estimate), hold the folded wipe for the parallel-jaw gripper. The hand holds the wipe directly in the real job;
  the wipe, its 24 x 24 mm working face, the 1.5 mL of 91% IPA and the white-pad check are unchanged. The holder keeps
  the fingers 12 mm above the IHS and outside the socket.
- **Recognisable features modelled:** white nonwoven with a blue edge line, a centre crease, a woven emboss and a
  slightly thicker contact face; the dark-blue grip bar with three grip ribs; the foil tray; the waste tray.

## Estimates and reduced/declared values

- **Socket 54 x 54 mm** body (the 54 x 90 mm cooler pattern is published; the body is a class estimate). IHS 3.6 mm
  thick (the published AM5 lid is 3.42 mm; the AM4 lid is a similar nickel-copper lid; the 7.35 mm stack is the
  measured value used). The substrate and SMD sizes are class estimates.
- **Paste residual 0.08 g** from a 0.3-0.5 g application; 2.5 g/cm3 density; 0.05 mm film. The 122 x 2 mm cells are a
  coverage measure, not 122 physical blobs.
- **Wipe 2.5 g + holder 25 g = 27.5 g**; 1.5 mL IPA. The 50 x 50 mm fold and the 24 x 24 mm contact face are chosen for
  the 37.5 mm IHS, not a catalogue sheet size. The wipe's 240 s drying time, the 1.5 s IPA film half-life, the 0.10
  dryness threshold, the dry rub's lint (which a wet pass lifts), the 0.12 g paste capacity, the 0.30 redeposit
  fraction, the 6 N wet/2 N dry scratch forces, the 25 degree lean and the 0.06 removal rate are declared sim estimates; the real checks are the loupe, the white final
  wipe, the 2-5 min dry time and the technician's light touch.
- **The contact-driven part is not estimated:** the pad's normal force, contact footprint and motion are read from the
  solver every control step. The 0.12 N contact gate and the 0.2 mm rubbing gate are the model's thresholds, not
  scripted actions.

## Real-world build (for a real trial)

A real AM4 CPU in a motherboard/socket on standoffs on an anti-static mat, with the cooler lifted off and the old
paste left as a 30 x 20 mm grey patch; a Kimtech W4 folded to 50 x 50 mm with 1.5 mL of 91% IPA; a waste tray; a
loupe and a raking light. Wipe in small circles/overlapping passes, keep the wipe within the IHS footprint and off the
socket frame, load plate and lever; lift the wipe and let the IPA flash off 2-5 minutes; inspect the lid for grey
streaks and scratches, and the socket/pins for paste. Replace the wipe if its face is loaded. The sim's scoring is the
same ladder: wet wipe, 60 %, 95 %, and a clean/dry/lint-free/socket-clean/scratch-free held state.
