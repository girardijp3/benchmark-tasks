# REFERENCE — panel-push-clip

## What the real object is

An **8 mm nylon panel push-clip (push-type retainer / push-in rivet)** used to fasten automotive and appliance trim
panels. Recognisable features: a round **cap/head about 20 mm across**, an **8 mm stem** that fits an 8 mm hole, and
**spring barbs near the tip** that spread as the clip is pushed through the panel and then spring back behind it to
retain it. The cap ends up flush with the panel. It is fitted by automotive service technicians, body repairers and
appliance repairers; a quick pull confirms the barbs are holding. Correct fit means the stem goes in square, the press
is firm but not forced, the cap is flush, and the clip resists a pull.

Sources (public catalogue / retail specifications, accessed 2026-11-17):

* **Lawson Products KT11397** — "Front Fender Push-Type Retainer for Nissan, Nylon, Black, Fits 8mm Hole, 20mm Head,
  9mm Stem Length" (catalogue page 26-161; OEM 01553-09321).
  <https://www.lawsonproducts.com/products/front-fender-push-type-retainer-nylon-20mm-kt11397>
* **Auveco 17216** — "Push-Type Retainer 20mm Head Diameter 20mm Length", Hole Size 8 mm, black nylon, GM/Ford/Chrysler
  OEM cross-references. <https://www.auveco.com/push-type-retainer-20mm-head-dia-20mm-length-17216>
* **Lawson Products 1680277** — "Interior Panel Trim Retainer, white nylon ... 8mm hole size, 20mm bolt size, 16mm Top
  Head Diameter, 21mm Overall Length, Stem Length 16mm", panel range 0.75-1.7 mm.
  <https://www.lawsonproducts.com/products/interior-panel-trim-retainer-1680277>
* **Hi-Line SR5901** — "5/16\" (8mm) Hole Size, 20mm Flange, 18mm Length, Push Retainer", black nylon.
  <https://hi-line.com/5-16-8mm-hole-size-20mm-flange-18mm-length-push-retainer>
* **Disco 1861PK / 40K618** — Nylon Retainer, 8 mm hole / 8 mm shank, 20 mm body length, push-in rivet.
  <https://eu.raptorsupplies.com/pd/disco/1861pk>

The brief supplies the force envelope (**4-7 N press**) and the product class (8 mm push-clip). No single
manufacturer's internal drawing is public, so the individual dimensions below are stated estimates, each flagged;
none is scaled down from the real product. The near-twin PR **#1986** cites the same class ("Fender Splash Shield
Retainer ... Fits 8mm Hole, 20mm Head", Lawson P53958), confirming the object class.

## Measured / given by the catalogues and the brief

| Quantity | Value | Source |
|---|---|---|
| Object class | 8 mm nylon panel push-clip / push-type retainer | product class + brief |
| Hole fit | 8 mm | catalogues (Lawson, Auveco, Hi-Line, Disco) |
| Cap / head diameter | 20 mm | catalogues (Lawson KT11397, Auveco 17216, Hi-Line SR5901) |
| Stem / shank diameter | 8 mm | Disco 1861PK (8 mm rivet/shank) |
| Stem / body length | ~18-21 mm | catalogues (Hi-Line 18, Auveco 20, Lawson 16/21, Disco 20) |
| Material | black/white nylon | catalogues |
| Press force | 4-7 N until the barbs seat and the cap is flush | brief |
| Retention check | quick pull-test | procedure / brief |
| Work height | single-hand fit at <= 0.30 m above the bench | station gate |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Clip mass | 2 g | a real nylon retainer is about 2 g; catalogues list pack weights only |
| Cap thickness | 3 mm | typical moulded head thickness for a 20 mm cap |
| Stem length used | 18 mm | within the catalogue 16-21 mm range |
| Barbs | 4 flared wedges to ~13 mm across | typical spring-barb retention feature; not a single drawing |
| Panel | 120 x 100 x 3 mm flat plate, 8 mm hole | typical trim-panel patch within YAM reach |
| Seat pose | cap-top z 0.003 m, cap underside flush with panel top | 3 mm cap thickness |
| Press travel | K_CLIP 350 N/m; 4 N = 11.43 mm, 7 N = 20 mm | a compliant stand-in giving the 4-7 N band |
| Pull travel | K_PULL 350 N/m; demo pulls 16 mm = 5.6 N | a firm retention pull inside 4-7 N |
| Seat / align tolerances | 4 mm x/y, 2.5 mm z, 8 deg tilt; align 10 mm / 12 deg | within a real 8 mm hole plus gripper repeatability |

## Deliberate simplifications (honest scope)

* **Declared carry.** While the gripper is closed across the cap, the clip follows the gripper. The clip, panel and
  tray have no colliders (as in the family's welded fixtures), so the gripper can close at the cap and the clip can
  pass through the hole without a rigid-body contact solve. The robot must still reach, close on, align, press and
  pull the clip; a soft nylon barb slipping through a parallel jaw is not the skill under test.
* **Locked once seated.** Once the barbs seat, the clip is posed at the seat fixed to the panel and no longer follows
  the gripper, so an upward pull is real gripper TCP travel against a fixed clip. This models barb retention without
  integrating a flexible-barb contact force. A pull beyond 7 N is a "not seated / extracted" cap.
* **Forces.** Insertion and pull forces are a declared 350 N/m compliant travel measured from the gripper TCP's real
  displacement in MuJoCo/Isaac (4 N = 11.43 mm, 7 N = 20 mm). They are not contact forces. The demo presses 5.4-5.95 N
  and pulls 5.0 N, both inside 4-7 N.
* **Single arm.** The panel is fixed, so only the right arm is used; the left stays at READY. This is a station-realism
  choice, not a scoring shortcut — all four manipulation steps are still performed by the robot.
* **One clip per trial.** Real work uses many clips per panel; this task scores one full align/press/seat/pull cycle.

## What is NOT simplified

Real product size and mass throughout: an 8 mm stem, a 20 mm cap, ~18 mm stem length, 2 g gravity-free clip, flared
barbs and a matching 8 mm panel hole; the clip must be reached, grasped, aligned over the hole, pressed to a measured
5.4-5.95 N with a flush cap, then given a measured 5.0 N pull held for 1 s. Idle and all four failure caps
(cross-insert, left unseated, dropped, forced fit) are scored. Full credit also requires both grippers clear and home.
