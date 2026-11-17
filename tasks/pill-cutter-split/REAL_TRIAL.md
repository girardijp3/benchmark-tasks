# Real trial: tablet, force check, half check, pad shield and residue

This file is part of the task's reproducibility. It gives a readily prepared inert scored tablet (or a source for a
custom placebo), a batch acceptance that measures the 14.0 N cutting force, the observable half check, and the physical
pad shield and residue rule.

## 1. The 20 mm scored tablet

**Preferred: an inert custom scored placebo.** A placebo manufacturer can press a 20.0 x 9.0 x 7.5 mm oblong,
functionally scored, white tablet from microcrystalline cellulose/lactose (e.g. Nutricraft Labs, "Custom Placebo
Tablets", 5-22 mm, oval/oblong, scoring lines; minimum 1,000, 2-4 weeks; or Zeebo Research custom supply). Ask for
**20.0 x 9.0 x 7.5 mm, one 0.8 mm deep score across the 9 mm width at mid-length, 1.80 g, no active ingredient**.
A commercial 20 mm scored tablet is not standard, so the surrogate below is the day-to-day option; both are inert.

**Repeatable surrogate (12-48 N, matches the sim's 20 x 9 x 7.5 mm, 1.8 g, 14.0 N).**

- **Material:** USG No. 1 Pottery Plaster (calcium sulfate hemihydrate), 100 g powder to 35 g deionised water
  (35:100 by mass). A weaker batch (outside the window below) is made with 38 g water; a stronger one with 32 g.
  Do not use resin or polymer clay: their cut force is outside the cited tablet range.
- **Mould:** two-part 3D-printed PETG mould (0.12 mm layers, 4 perimeters, 30% infill, 0.2 mm clearance between the
  halves). Cavity **20.0 x 9.0 x 7.5 mm**, 0.5 mm corner radii; a **0.8 mm deep x 1.2 mm wide score ridge** across
  the 9 mm width at the middle of the 20 mm length. Release with PTFE mould spray.
- **Cure:** mix 60 s, pour, tap the mould 20 times on the bench to raise bubbles, cure **45 min** in the mould at
  23 +/- 2 C, demould, air-dry **24 h** at 23 +/- 2 C and 50 +/- 10% RH. Sand the two 9 x 7.5 mm ends flat with 400
  grit; brush the score groove clean. The dry tablet is 1.80 +/- 0.10 g (density about 1.33 g/cm3).
- **Batch acceptance (5 tablets):** calipers 20.0 +/- 0.2 x 9.0 +/- 0.2 x 7.5 +/- 0.2 mm; mass 1.80 +/- 0.10 g on a
  0.01 g scale. Cut one tablet in the same pill cutter with a **0-50 N push-pull gauge** (e.g. Imada Z2-50) on the
  fin, or a 0-50 N load cell under the cutter: record the **peak at the fin** and require **8.2 +/- 0.9 N**, which is
  **14.0 +/- 1.5 N at the blade** through the 1.7:1 lever, inside the cited **12-48 N at the pill / 5.6-22.2 N at the
  handle** (UW-Madison BME design report, RERC FINAL, Table 1; average 32.8 N at the pill). Hardness cross-check:
  **Shore D 55-75** on the flat face (Checkline DD-224 or equivalent) or a 10 mm cube crush at 14 +/- 3 N. Reject the
  batch and adjust the water by 3 g if the peak is outside 7.3-9.1 N at the fin. Store the tablets dry; they are
  friable and single-use.
- **Sim's declared tablet:** 20 x 9 x 7.5 mm box, 1.8 g, homogeneous, score line visual only; the cut is a declared
  blade-plane split, not a fracture model. The hinge brake's peak is 0.56 N m = **8.24 N at the fin / 14.0 N at the
  blade**, falling to 30% as the score cracks, so the simulated cut needs the same peak as the surrogate.

**Safety:** the tablet is inert plaster/placebo, not medication. Wear nitrile gloves and eye protection when cutting;
the blade is sharp.

## 2. Real half check (numeric and observable)

1. Weigh the whole tablet before the trial on a **0.01 g scale**; record m0 (1.80 g nominal).
2. After the cut, weigh each half. **Pass only if 0.40 m0 <= each half <= 0.60 m0** (0.720-1.080 g for 1.80 g).
   A 2.5 mm placement offset gives 0.675 / 1.125 g and **fails**; this is the same 40-60% rule the sim's step 4 uses.
3. With a **10x loupe or the top-camera zoom**, check that the cut face follows the score line within **+/- 1 mm**
   along the 20 mm length (measure from the groove to the cut face at both ends). More than 1 mm **fails**; the sim
   uses the same 1 mm cut-line tolerance.
4. Log the two masses and the two cut-line offsets with the trial. Both checks are on every trial; a failed half
   check means step 4 is not earned (the sim's step 4 is false for the same numbers).

## 3. Pad shield (shared grippers)

The robot's finger pads are shared across tasks; a 0.6 mm PETG U-clip covers the **outer and side faces** of each pad
near the tip, leaving the inner grip face exposed. It is a real part, not a visual: it is printed from **Prusament PETG**
(1.75 mm, 240 C nozzle, 0.2 mm layers, 3 perimeters, 40% infill) from `pad_shield.scad`, 24 mm long x 14 mm wide x 6 mm
high, 0.6 mm walls, open on the grip side; press-fit, or fix with **3M VHB 5952** tape on the outer face. The blade is
also inside a 0.6 mm PETG shroud on the cutter (the upper 7 mm; only the cutting edge below the cradle rim is exposed),
so the pad's inner face is **at least 0.6 mm (shroud) + 0.6 mm (clip) = 1.2 mm** from the blade. The clips add 0.6 mm
only to the outer envelope: the **fin 8 mm squeeze** (6 mm fin) and the **caplet 1 mm squeeze** (9 mm caplet), and the
pad's inner strike face, are unchanged. Check with calipers that the clip's inner opening is 0.0 mm behind the pad's
inner face (the face is flush and exposed) and that the clip's outer envelope is 0.6 mm; a trial with a clipped pad
grips the fin at the same 6 mm and the caplet at the same 9 mm as the sim.

## 4. Residue rule

After **every** trial, wipe the pad faces, the blade shroud, the V cradle and the fin with a fresh **91% isopropyl
alcohol (IPA)** wipe; inspect them at **10x** for white plaster/tablet residue; replace the wipe; do not reuse it. If
residue remains, wipe again and re-inspect before the next trial. The sim cannot model residue; this rule is part of
the real station procedure and the reset.
