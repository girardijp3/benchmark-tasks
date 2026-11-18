# REFERENCE — magnetic-dust-boot

## What the real object is

A **magnetic dust boot / magnetic spindle-nose dust seal**: an oil-resistant elastomer (typically nitrile/NBR)
cap with a **moulded-in magnet ring** that centres itself on a steel machine-tool spindle nose and is pressed
axially against the nose shoulder. It keeps chips, grit and coolant out of the spindle taper/bore when the chuck
or a fixture is off, and it is removed and refitted whenever the nose is cleaned or the setup changes. The
recognisable features are the flexible black elastomer body, the visible magnet ring at the bore, and the way the
ring snaps concentrically onto the steel nose; a sloppy approach lets the magnet grab an **off-centre** bite of
the rim, a shallow push leaves the boot **proud of the shoulder**, and a hard jamb drives it **past the seat**.

There is no single public manufacturer drawing for an "80 mm boot on a 65 mm nose", so the exact body/ring
dimensions below are stated estimates inside a real product envelope; each is flagged.

## Sources (product class, true sizes)

| Source | What it establishes |
|---|---|
| Hutchinson Precision Sealing Systems, *Magnetic Rings* catalogue — https://precisionsealingsystems.hutchinson.com/wp-content/uploads/2023/02/Magnetic_Rings_Catalogue_EN-3.pdf | Magnet rings are a **metal insert bonded to a magnetic rubber (elastomer) compound**; NBR ferrite compound gives **oil & grease / fluid resistance**; typical applications include **machine tools and high-speed spindles**; size chart includes an **80 mm OD / 64 mm ID** ring (ref CLA80) and an 80 mm / 67 mm ring (CSA80) — essentially the boot size here. |
| PwnCNC, *Dust Boot v12* — https://pwncnc.com/collections/all/products/dust-boot-v12 | 80 mm spindle collar dust boot with a **removable magnet-held** brush track — the 80 mm magnetic dust-boot class. |
| Onefinity, *Suck It SM dust shoe (80 mm)* — https://www.onefinitycnc.com/product-page/suckit-sm-dust-boot-atc-compatible | 80 mm spindle dust shoe with **pre-installed magnets**, removed for tool changes — routine fit/remove cycle. |
| Printables, *80 mm CNC spindle magnetic dust shoe* (models 466000 / 1258373 / 656416) | Magnet-held 80 mm spindle dust boot/shoe; magnets press-fit into the body. |
| Brief (this task) | **4-7 N** press force; YAM reach gate 0.35 m; work height 0.20 m; no off-centre grab. |

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Product class | 80 mm magnetic dust boot / spindle-nose seal | product listings |
| Construction | oil-resistant elastomer with an embedded magnet ring | Hutchinson catalogue, NBR ferrite |
| Magnet ring | moulded/bonded metal insert + magnetic rubber compound | Hutchinson catalogue |
| 80 mm ring OD | 80 mm OD / 64 mm ID reference ring exists | Hutchinson CLA80 |
| Press force | 4-7 N | brief |
| Work height / reach | shoulder 0.05 m, boot reach 0.35 m | station YAM gate |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Boot body | 80 mm OD x 40 mm tall, 65 mm bore | fits a 65 mm nose with a small magnet land; real product size, no scaling |
| Embedded magnet ring | ~6 mm tall band at the lower inner rim | the band that centres on the steel nose |
| Spindle nose | 65 mm diameter, 45 mm tall, on a 100 mm shoulder boss | representative machine-tool spindle nose presenting the seat at 0.05 m |
| Boot mass | 90 g | oil-resistant elastomer body + magnet ring (EST) |
| Staging pocket | boot at x 0.25, y -0.06 | within reach, clear of the spindle |
| Alignment / seat / overpress | 6 mm / 5 mm / 4 mm | bore centred within 6 mm, bottom within 5 mm of the shoulder, >4 mm past it is forced |
| Press stroke | 35 mm from approach to seat | a full concentric press onto the nose |

## Routine-importance case (explicit)

This is a **routine**, high-population action, not an occasional repair:

* **Population (BLS OEWS, May 2024 / 2025):** Machinists **51-4041 ≈ 287,050** (OEWS national profile; 249,790 in
  manufacturing alone, BLS Manufacturing Day 2024); Industrial machinery mechanics **49-9041 ≈ 421,940** (all
  industries, May 2024 OEWS; 446.9k in the 2025 employment matrix). Together **≈ 709,000 workers**, comfortably
  over the 500,000 bar on machine operators/maintenance alone. Broader maintenance pool: Maintenance and repair
  workers, general **49-9071 ≈ 1.62 million** (2025 employment matrix); CNC tool operators/programmers **51-9160**
  add more. Sources: https://www.bls.gov/oes/tables.htm , https://data.bls.gov/oesprofile/ ,
  https://www.bls.gov/opub/ted/2025/a-look-at-manufacturing-occupations-for-national-manufacturing-day.htm .
* **Frequency:** a spindle-nose boot is fitted or renewed at every chuck/fixture change, at each job setup, after
  spindle cleaning, and at scheduled service — commonly several times per shift per machine. That is a routine
  shop-floor cycle, not a rare one-off.
* **Consequence of error:** an off-centre magnet grab, a boot left proud of the shoulder, or a forced jam past
  the seat lets chips/coolant into the taper, causes runout or a bound spindle, scraps parts and costs downtime.

## Deliberate simplifications (honest scope)

* **Declared carry.** While a gripper is closed across the boot's grasp point, the boot follows the gripper (the
  family's declared-detent device). The robot must still reach, close on, carry, centre, press, seat and release;
  the concentric alignment, press stroke, seat and all three caps are real geometry.
* **Force.** The 4-7 N press force is stated in fidelity rather than integrated as a contact force (the family's
  convention for stated spring forces and torques). The press requires a closed grip and a full stroke depth, not
  a bare touch.
* **No colliders on the spindle.** The fixture is one welded, visual-only body, as in the family's fixtures, so no
  finger or boot can pass through a solid; the only collider is the boot. The seat, concentricity and overpress
  tests are geometric. A "forced press" is precisely driving the boot through the shoulder stop, which is capped.
* **Magnetic attraction** is represented by the declared carry plus the geometric concentricity test; field
  strength and pull-off force are not simulated (the skill is alignment and seating, not magnetics).

## What is NOT simplified

Real product size throughout (no scaling): 80 mm boot OD, 65 mm bore and nose, 40 mm boot height, 90 g mass,
4-7 N press. The boot must be centred within 6 mm, pressed through 90% of its stroke, seated within 5 mm of the
shoulder, released, and left seated and centred with both grippers clear and home for a full 1 s. Idle and all
three failure caps (magnets grab off-centre, boot left unseated, press forced >4 mm past the shoulder) are scored.
The robot grips a small central boss on the boot (the only collider), so a closed grip can take the 80 mm boot;
the elastomer body itself is visual-only, like the spindle fixture.

## Notes for the Isaac finisher

The scripted demo reaches MuJoCo 1.0 at the step recorded in `task.json` / `README.md`; worst IK track error is
reported there. All success tests are geometric on the mechanism state, so no contact re-tuning is needed. If
Isaac tracking of the final press is marginal, shorten the press waypoint; do not lower the 6 mm alignment or
seat tolerances, since those are what make "no off-centre grab" real. Re-run the live twin check before any PR
(the board is moving ~2 PRs/day).
