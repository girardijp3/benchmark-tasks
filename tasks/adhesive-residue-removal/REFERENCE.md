# Reference: returned hard-anodized pan, price-label adhesive wipe

## Products and sources

| Item | Real product | Published size / mass | Source |
|---|---|---|---|
| Pan | Calphalon Premier Hard-Anodized Nonstick 12-Inch Frying Pan with Lid, RBT1392 | 12 in / 30.5 cm across; 3.6 lb / 1.63 kg pan (5.5 lb with lid); hard-anodized aluminium exterior; stainless handle | https://www.calphalon.com/cookware/cookware-by-material/nonstick-hard-anodized-cookware/premier-hard-anodized-nonstick-12-inch-frying-pan-with-lid/SP_2590418.html |
| Label | Avery 5163 TrueBlock Shipping Labels, Ultrahold permanent adhesive | 2 x 4 in / 50.8 x 101.6 mm stock; the task uses one 80 x 40 mm piece cut from it | https://www.avery.com/products/labels/5163 |
| Solvent pad | Chemtronics Chempad CP400, 91% IPA / 9% deionized water, lint-free felt | 3 x 4 in / 7.6 x 10.2 cm | https://www.chemtronics.com/chempad ; https://www.chemtronics.com/content/msds/TDS_CP400.pdf |
| Adhesive removal | IPA swells pressure-sensitive acrylic adhesive | 30 s dwell in the real protocol; the wipe lifts the softened film | https://group.nagase.com/nagasechemtex/e1/en/advanced-concept-materials/dismantlable-adhesive/adhesive-removal-methods/ |
| Hard anodize | Hard anodized aluminium is Mohs 9; raw aluminium is 2.5 | The finish is damaged by a trapped hard grit, not by the soft felt | https://www.anoplate.com/news-and-events/how-hard-is-hardcoat-anodize-hardness-types-applications/ |
| Employment | Stockers and order fillers 2,764,800; retail salespersons 3,936,700 (2024) | BLS National Employment Matrix | https://data.bls.gov/projections/occupationProj |

## Recognisable features modelled

Pan: 30.5 cm flared rim; dark charcoal hard-anodized bowl; flat 24 cm base with a shallow step ring;
stainless strap handle with two rivets and a hanging hole; the pan is upside down on a 3 mm silicone mat.
Chempad: white 76 x 102 mm lint-free pad, 3 mm thick, on an opened foil packet; three pads on the tray, one for
the dwell, one working wipe, one fresh white final wipe. Label: white 80 x 40 mm facestock with an Avery-blue
TrueBlock strip, peeled and lying on the waste tray. The 5 mm grid overlay, 10x loupe and 30-degree raking LED
are on the bench.

## Sim vs real

| Number | Real | Sim | Note |
|---|---|---|---|
| Pan diameter | 0.305 m | 0.305 m | published |
| Pan mass | 1.63 kg | 1.63 kg | published |
| Pan height | 0.064 m | 0.064 m | EST; the base face is 64 mm above the rim plane |
| Base diameter | 0.24 m | 0.24 m | EST |
| Handle | 0.20 m | 0.20 m long, 25 x 15 mm | EST |
| Chempad | 76 x 102 mm, ~1-2 mm felt | 76 x 102 x 3 mm, compliant 900 N/m, wet felt friction 0.55 / 0.45 | thickness and wet friction are EST; measure the pad on the pan |
| Residue patch | 80 x 40 mm film, ~0.02 mm | 128 cells of 5 mm, 0.05 mm visual, no collider | declared material-removal layer |
| Wipe force | 0.5-1 N press by hand; at least 0.5 N to wet | 0.5 N minimum; live 1.6-3.6 N | the live pad press is measured from MuJoCo contact |
| Dwell | 30 s | 20 s | EST; the real stopwatch decides |
| Wipe removal | soft adhesive lifts on the felt | 0.20 of residue per mm of rubbing on a soft cell; 0.08 wet; 0.02 dry | EST; calibrate on a bench coupon |
| Surface dry | IPA evaporates in seconds | 6 s after the pad lifts | EST |
| Soft life | the swollen film stays soft while wet | 45 s | EST |
| Pad load | a loaded pad redeposits | >0.05 g redeposits a 0.12 film | EST; weigh the pad before/after |
| Damage | a trapped hard grit gives a 2 mm bright line | a gritty pad latches a scratch | real check: 10x loupe + 30-degree raking light |
| Mat friction | 1.2 | 1.2 | the pan is free, not clamped |

Where a number is EST, there is no published value: the source names the method, and the real bench measurement
is required before a real trial. The pan's published size and mass are the only numbers taken directly from a
manufacturer.
