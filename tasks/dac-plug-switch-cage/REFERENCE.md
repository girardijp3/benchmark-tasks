# Reference — real product, sources and measured vs estimated numbers

## Product

A short **DAC (direct-attach copper) cable** used to link a server/NIC to a top-of-rack switch. Its plug is an
SFP/SFP+-style metal module: a keyed rectangular body with gold contact tongue at the tip, a spring latch tab on
top that clicks into the port cage, and a moulded strain-relief **boot** where the cable leaves the plug. The
switch has rows of port **cages** (metal sockets) and a cable-management **comb** above them to keep each boot
seated and the cable bend controlled. Patching procedure: align the plug tip-first with the port, push straight
in until the latch clicks, seat the boot in the comb, tug gently to confirm retention, let go. A plug in the
wrong port, forced in off-axis, unclicked, with the boot out of the comb, or that pulls back is a link fault or a
cable that works loose. Category: data-centre / AI-cluster technician work (BLS OEWS SOC 15-1232, 15-1244).

## Sources

- **SFP MSA (SFF-8074 / INF-8074), "SFP Transceiver Package Dimensions" and "SFP Cage Assembly Dimensions":**
  transceiver width 13.7 ± 0.1 mm, height 8.6 ± 0.1 mm (front), 8.5 mm (rear); cage inside width 14.0 ± 0.1 mm
  (min 13.9), inside height 8.95 ± 0.15 mm, cage overall 15.0 mm wide / 9.8 mm high; latch opening near the cage
  front. Insertion force 0-40 N, extraction 0-11.5 N, module retention 90-170 N, cage latch strength 180 N.
  <http://schelto.com/SFP/SFP%20MSA.pdf>
- **SFP+ / zSFP+ cage and DAC cable connector spec (SFF-8431-class):** SFP+ module-to-cage insertion 34 N max
  (45 N with heatsink/clip), extraction 12.5 N max, latch strength 91 N min, cable rotational pull 33 N min.
  <https://file.huaqiu.com/web2/M00/AD/59/pYYBAGSPlaeAKaBxAAMU-pruCF4463.pdf>
- **DAC product data sheet (10G SFP+ passive copper twinax):** cable jacket 4.5-6 mm, bend radius 5x diameter,
  PVC jacket — the boot must keep the cable clear of the cage edge.
  <https://www.optcore.net/wp-content/uploads/2018/03/SFP-10G-DAC-PxM_DS.pdf>
- **Vendor installation guide:** "align the DAC module with the I/O port ... gently push in the module straight
  until it clicks to fully seat it in the port"; remove by pulling the tab.
  <https://sc1.checkpoint.com/documents/Appliances/FRU_Transceivers/Content/Topics-FRU-Transceivers-and-DAC/Transceivers-and-DAC-Cables.htm>
- **Queue source for this task (BUILD_QUEUE.md rank 6):** "40 mm DAC/USL-style plug; switch port cage; latch; boot
  into comb; 4-7 N", YAM gate h 0.20 / reach 0.40 / 40 mm plug, plug at x 0.35 z 0.20, align/push/latch/boot,
  first-MuJoCo gate at x 0.35 z 0.20.

## Measured from the product class (public spec / catalogue)

| Quantity | Value | Source |
|---|---|---|
| Plug body width x height | 13.7 x 8.6 mm (front) | SFP MSA Table 1 |
| Cage inside width x height | 14.0 x 8.95 mm | SFP MSA Table 4 |
| Cage overall width x height | 15.0 x 9.8 mm | SFP MSA Table 4 |
| Insertion force envelope | 0-40 N (SFP), 34 N max (SFP+) | SFP MSA / SFP+ spec |
| Latch retention | >= 90 N (module), >= 91 N (cage latch) | SFP MSA / SFP+ spec |
| DAC cable diameter | 4.5-6 mm | DAC data sheet |

## Estimated / task-model values (stated in `task.json` fidelity and in the task file)

| Quantity | Value | Why |
|---|---|---|
| Plug body length | 40 mm | The queue's "40 mm DAC/USL-style plug"; the part of a DAC plug that enters the cage. |
| Plug body width x height | 13.5 x 8.6 mm | Just inside the MSA cage (14.0 x 8.95), following the MSA module width/height. |
| Cage mouth width x height x depth | 14.5 x 9.5 x 45 mm | MSA inside size plus a small lead-in; 45 mm cage depth from MSA overall 48.8 mm. |
| Boot flange width | 12 mm | Slightly wider than the 13.5 mm body's mating cable strain relief; fits the 14 mm comb slot. |
| Comb slot | 14 mm | Two 2 mm teeth 18 mm apart, centred on the port; clear the 12 mm boot. |
| Insertion push | 4-7 N | The queue's force range (inside the 0-34/40 N real envelope). |
| Latch depth / boot depth | 36 mm / 40 mm click | Tab reaches the cage notch at 36 mm; boot flange square in the comb at 40 mm. |
| Tug pull-back | 4 mm | A short retention check; real spec uses >= 33 N cable pull. |
| Plug mass | 20 g | A DAC plug module (no long cable modelled). |
| Work height / reach | port at h 0.10 m, x 0.31 m | YAM left arm reaches (0.31, 0.015, 0.10) with ~0 mm error; the queue's h 0.20 could not be reached (22-38 mm IK error at z 0.20). The gate is "h <= 0.20, reach 0.40"; 0.10 m is a low, real bench-riser layout. |

Nothing in the model is scaled down; the only simplifications are that the spring force/click and the latch's
holding force are declared mechanism states rather than a simulated spring, the DAC cable is represented by the
plug's boot tail (no flexible cable), and there is no electrical signal. The static switch is visual geometry
(the family convention: only the free plug carries a collider); the cage, latch notch, comb and boot are real
modelled shapes and the fit/retention checks are analytic against those exact dimensions.
