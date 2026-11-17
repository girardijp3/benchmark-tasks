# Real bench fixture for the light-curtain test

This is the buildable version of the two bench fixtures described in the README's **Real-world check**. They are
training/test aids that guard nothing: no machine is connected and no real safety curtain is opened, altered or
jumpered. One fixture is the sound curtain; the other has one receiver channel's link removed (the documented open
channel). Everything runs from a single 5 V supply.

The simulated station and this fixture use the same numbers: 8 beams at 20 mm pitch, 57 to 197 mm above the table
(CH0 = 57 mm ... CH7 = 197 mm), 10 mm lenses (30 mm resolution with a 30 mm rod), 400 mm between the lens faces,
150 mm protected height (52 to 202 mm), two 40 x 40 mm posts.

## 1. Principle: how a link makes one dead channel

Each receiver is a 3-pin IR module: **output low while its 38 kHz beam is clear, output high while the beam is
blocked**. Each receiver output goes through its own labelled 2-pin link to an 8-channel comparator board. The
comparators have open-collector outputs tied to one shared **BUS**:

- **Link fitted:** a blocked receiver drives its comparator input high; the comparator sinks the BUS low. The lamp
  driver then lights **red**. A clear receiver leaves the BUS high.
- **Link removed:** a 10 kΩ resistor pulls that comparator input low, so the comparator can never sink the BUS. The
  channel is ignored by the lamp, although its emitter and receiver still work.
- The BUS is low if **any enabled** channel is blocked, and high only when **every enabled** channel is clear. So the
  lamp is red while any working beam is interrupted and green only when no working beam is.
- Remove one link (say CH3, 117 mm) and sweep a 30 mm rod. The rod covers CH3 alone when its axis is within about
  9.5 mm of 117 mm: the neighbouring beams at 97 and 137 mm are 20 mm away, so at the edge of that band they are
  10.5 mm from the rod axis — just outside the 10 mm that keeps a 10 mm lens fully inside the 30 mm rod. In that
  19 mm band the lamp stays **green**. At every other height the rod covers an enabled beam (or two), and the
  lamp is **red**. This is the same not-red band the simulation scores, at whatever height the drawn channel is.
- With all eight links fitted the lamp is red at every height in the field. That is the sound fixture.

The lamp is hardware: it works with the Arduino Nano unplugged. The Nano is only an optional channel monitor.

## 2. Bill of materials

Quantities are **per fixture**; the bench has two fixtures. The Total column is for both fixtures. Prices are
approximate single-unit USD (Nov 2026) and are only a guide. Anything marked **sub** may be substituted.

| # | Item | Qty/fixture | Total | Part / example | Substitutes and notes | Low-cost source | ~USD |
|---|---|---|---|---|---|---|---|
| 1 | IR emitter, 940 nm | 8 | 16 | Vishay **TSAL6200** (5 mm, 100 mA) | **sub** TSAL6100 (10°, narrower) or TSAL6400 (25°, wider; its beam fills the 10 mm lens better); generic 5 mm 940 nm IR LED | Mouser 782-TSAL6200, Digi-Key 751-1204-ND, LCSC | 8 |
| 2 | IR receiver for light barriers, 38 kHz | 8 | 16 | Vishay **TSSP4038** (3-pin, fixed gain, continuous carrier) | **sub** TSSP58038; **do not use TSOP4838** — its AGC suppresses a continuous 38 kHz carrier | Mouser 782-TSSP4038, Digi-Key TSSP4038-ND, LCSC C143753 | 21 |
| 2b | 10 mm lenses | 16 | 32 | 10 mm diameter, about 10 mm focal length plano-convex lens, acrylic/PMMA (8 emitter + 8 receiver per fixture) | **sub** a 10 mm ball lens or a molded 10 mm LED lens; a bare 10 mm hole is not enough — a wide-angle LED would spill past the receiver and crosstalk. A 10 mm opal diffuser in front of a 5 mm LED also fills the lens; without one the effective beam is about 6 mm and the green band about 16 mm, still inside the 25 mm lane tolerance | AliExpress/LCSC/Edmund Optics | 8 |
| 3 | 38 kHz oscillator | 1 | 2 | NE555P timer | **sub** TLC555, LMC555, or a 38 kHz oscillator module | Mouser 595-NE555P, Digi-Key 296-1411-5-ND | 1 |
| 4 | Emitter driver | 1 | 2 | 2N2222A NPN, TO-92 | **sub** PN2222, BC337, or one ULN2803 channel | Any electronics supplier | 1 |
| 5 | 8-channel comparator board | 1 | 2 | **2x LM339N** quad comparators on 5 x 7 cm perfboard, plus 8 input dividers | **sub** 2x LM2901N; **sub** 2x TLV1704 (rail-to-rail input, omit the dividers and use a 2.5 V reference); a ready-made 8-channel LM339 module only if its inputs accept 0–5 V and its outputs are open-collector | Mouser 595-LM339N, Digi-Key 296-1393-5-ND | 3 |
| 6 | Lamp driver | 1 | 2 | 74HC14 hex Schmitt inverter | **sub** 74HCT14, 74HC04 (no Schmitt), or two 2N3904 drivers | Mouser 595-SN74HC14N, Digi-Key 296-1563-5-ND | 1 |
| 7 | Lamp LEDs | 1 red + 1 green | 2 + 2 | 5 mm red and green LEDs, 2 V, 20 mA | **sub** a 3-pin common-cathode bi-colour LED (common cathode to GND; red and green anodes through 1 kΩ to the two driver outputs) | Any electronics supplier | 1 |
| 8 | Removable fault links | 8 | 16 | 2-pin 2.54 mm male header + 2.54 mm shunt jumper, labelled CH0...CH7 | **sub** 2-pin 5.08 mm terminal block + short wire loop; **sub** 2-pin JST-XH plug with a loop | Any electronics supplier | 3 |
| 9 | Resistors | — | — | 8x 150 Ω 1/4 W (emitters); 8x 100 Ω (TSSP supply); 16x 10 kΩ 1% (input dividers); 1x 10 kΩ (BUS pull-up); 2x 10 kΩ + 1x 3.3 kΩ (1.24 V reference); 1x 1 kΩ + 1x 1.4 kΩ (555 R1/R2); 1x 1 kΩ (Q1 base); 1x 10 kΩ (Q1 base pull-down); 2x 1 kΩ (LEDs) | A 1/4 W resistor kit covers these | Any electronics supplier | 3 |
| 10 | Capacitors | — | — | 1x 10 nF (555 timing); 1x 100 nF (555 pin 5); 1x 100 µF/10 V (emitter rail); 8x 100 nF (TSSP decoupling); 8x 100 nF (comparator inputs); 1x 10 µF + 1x 100 nF (reference); 2x 100 nF (comparator/lamp supply) | A ceramic/electrolytic kit covers these | Any electronics supplier | 3 |
| 11 | 5 V supply | 1 shared | 1 | 5 V 2 A regulated adapter, 5.5 x 2.1 mm barrel, centre positive | **sub** Mean Well RS-15-5 (5 V, 3 A), or a USB 5 V 2 A charger with a USB breakout; **not** a 12 V supply | Any electronics supplier | 10 |
| 12 | Fuse and power input | 1 shared | 1 | 1 A resettable polyfuse + barrel-jack-to-screw-terminal adapter | **sub** 1 A blade fuse in an inline holder | Any electronics supplier | 3 |
| 13 | Perfboard / enclosure | 1 | 2 | 7 x 9 cm perfboard and a small plastic project box | **sub** stripboard; keep the 5 V and BUS wiring short | Any electronics supplier | 8 |
| 14 | Cable and connectors | — | — | 10-wire ribbon for the receiver (8 OUT + 5 V + GND), 3-wire for the emitter, 3-wire for the lamp, 2-pin power; IDC or JST connectors; heat-shrink | **sub** two 5-wire ribbons; label both ends | Any electronics supplier | 10 |
| 15 | 3D-printed posts | 2 | 4 | 40 x 40 x 230 mm PETG/PLA posts, 2 mm wall, 8 lens bores of 10 mm at 20 mm pitch (57...197 mm), 10 mm lens seats, removable back cover | **sub** 40 x 40 mm aluminium extrusion or 40 mm square PVC tube with drilled 10 mm lens seats; print a 10 mm lens holder if the bore is ragged | Local 3D printer | 6 |
| 16 | Foot plates | 2 | 4 | 90 x 90 x 12 mm plates, 4x M4 slots for alignment | **sub** 12 mm plywood or steel plate; clamp to the table | Local 3D printer / hardware | 6 |
| 17 | Test rod | 1 | 2 | 30 mm OD x 300 mm, 1.5 mm wall aluminium tube, capped ends, yellow/black bands, black grip band in the middle; about 110 g | **sub** SICK test rod 30 mm, P/N **2022602** (250 mm); a 30 mm wooden dowel is not equivalent in mass | SICK distributor, metal supplier | 16 |
| 18 | Tag | 1 | 1 | 76 x 127 x 5 mm red HDPE "DO NOT USE" tag | **sub** 5 mm PVC or acrylic sheet; round the edges | Local plastics supplier | 2 |
| 19 | Tag holders and rack | 3 | 3 | 100 x 28 mm block, slot 8 x 90 mm, 24 mm deep, flared to 20 mm at the top (two holders + one rack) | **sub** 3D-printed holder from the task's `tag_holder.usda` dimensions | Local 3D printer | 3 |
| 20 | Arduino Nano (optional) | 1 | 2 | Arduino Nano clone, 5 V, for the channel monitor only | **sub** ESP32 DevKit (3.3 V: use 10 kΩ/20 kΩ dividers on the monitor inputs); **or omit** — the lamp is hardware | Any electronics supplier | 10 |
| 21 | Hardware and wire | — | — | M3 x 10 mm and M4 x 16 mm screws, washers, nuts, 22 AWG stranded wire, tape, cable ties | — | Hardware store | 8 |

**Rough total:** about USD 120–150 for two fixtures with the discrete emitter/receiver build, or about USD 190–220
with the Adafruit break-beam alternative below. The 3D printing time is the main labour.

### Off-the-shelf alternative (no 555, no emitter wiring)

Use **8x Adafruit 2168 IR break-beam sensor (5 mm LEDs)** per fixture instead of items 1, 2, 3 and 4 (keep the
10 mm lenses, item 2b). Each 2168 is a matched
emitter/receiver pair, 3.3–5 V, 50 cm range, with an open-collector receiver output. Power both halves from 5 V, add a
10 kΩ pull-up from the receiver output to 5 V (high = beam blocked), and take that output through the **same** 2-pin
link into the **same** LM339 comparator board. Fit the 5 mm LEDs behind the same 10 mm lenses; add a 10 mm opal
diffuser if you want the full 10 mm beam (without it the green band is about 15 mm, still inside the 25 mm lane
tolerance). The rest of the fixture is unchanged. Adafruit 2168 is USD 5.95 each (16 pairs = USD 95); a generic 5 mm
IR break-beam pair is about USD 1.50–2 each (16 pairs = USD 24–32). This route needs no 555 and no emitter driver, but
the 5 mm receiver is not modulated, so keep the bench out of direct sunlight and strong incandescent light.

A real safety light curtain (for example a SICK deTec4 30 mm/300 mm set) is **not** a substitute for this
fault-injection fixture: its channels cannot be individually disabled without altering a safety device. Use a real
curtain only as a reference for the test-rod procedure, never as the faulty fixture.

## 3. Wiring

### 3.1 Emitter post (one per fixture)

```
+5 V ──┬── 150 Ω ──|>|── TSAL6200 CH0 ──┐
       ├── 150 Ω ──|>|── TSAL6200 CH1 ──┤
       │        ... 8 branches ...       ├── Q1 collector (2N2222A)
       └── 150 Ω ──|>|── TSAL6200 CH7 ──┘
                                          Q1 emitter ── GND
NE555 pin 3 ── 1 kΩ ── Q1 base
                  └── 10 kΩ ── GND
```

The `|>|` arrow is the LED: anode to the 150 Ω resistor, cathode to the common Q1 collector. The 555 astable is:

```
+5 V ── R1 1 kΩ ── pin 7 ── R2 1.4 kΩ ── pin 6 ── C1 10 nF ── GND
pin 2 tied to pin 6
pin 4 and pin 8 to +5 V; pin 1 to GND; pin 5 ── 100 nF ── GND
pin 3 = 38 kHz output (about 37.9 kHz with these values, 63% duty)
```

Add a 100 µF/10 V electrolytic and a 100 nF ceramic across the +5 V rail at the emitter post. Each emitter runs at
about 23 mA (8 x 23 = 184 mA), well inside the TSAL6200 100 mA rating and the 2N2222A 800 mA rating. The emitter
cable is 3 wires: +5 V, switched cathode (Q1 collector), GND. Keep the 150 Ω resistors in the emitter post so the
cable carries only the common current.

If using the Adafruit 2168 alternative, each emitter is powered from +5 V and GND and draws about 20 mA; no 555 and no
2N2222A.

### 3.2 Receiver post (one per fixture)

Eight TSSP4038 modules, one behind each 10 mm lens, pins 1 = OUT, 2 = GND, 3 = Vs. Each module gets a 100 Ω
series resistor in its Vs lead and a 100 nF capacitor from Vs to GND. Run a 10-wire ribbon: +5 V, GND, and OUT0...OUT7.
Label the receiver end and the main-board end CH0...CH7, bottom to top.

### 3.3 Main board: links, comparator OR and lamp

Only one channel is drawn; channels 0...7 are identical.

```
TSSP4038 CHk
  pin 3 Vs ── 100 Ω ── +5 V
  pin 2 GND ─────────────────────────── GND
  pin 1 OUT ── Jk ── 10 kΩ ── node A ── LM339 INk−
                               │
                             10 kΩ
                               │
                              GND
  node A ── 100 nF ── GND

LM339 INk+ ── 1.24 V reference:
               +5 V ── 10 kΩ ── node ── 3.3 kΩ ── GND
               node ── 10 µF ── GND; node ── 100 nF ── GND

LM339 OUTk (open collector) ── BUS ── 10 kΩ ── +5 V
BUS ── 74HC14 pin 1
       pin 2 (RED)   ── 3-wire lamp cable ── 1 kΩ ── red LED   ── GND
       pin 3 tied to pin 2
       pin 4 (GREEN) ── 3-wire lamp cable ── 1 kΩ ── green LED ── GND
```

- **Jk fitted** = channel enabled. **Jk removed** = channel disabled; the 10 kΩ from node A to GND holds the
  comparator input at 0 V, below the 1.24 V reference, so the comparator output stays off and cannot pull the BUS low.
- The 10 kΩ/10 kΩ divider at each input brings the receiver's 5 V high down to about 2.25–2.5 V, inside the LM339
  common-mode range (0 to Vcc − 1.5 V at 5 V). The 1.24 V reference is the threshold. A receiver high gives
  2.25–2.5 V (blocked); a receiver low gives about 0 V (clear).
- All eight LM339 outputs are wired together at the BUS. Any one enabled channel that is blocked pulls the BUS low.
  The 10 kΩ pull-up returns the BUS high only when every enabled channel is clear.
- The 74HC14 inverts the BUS twice: BUS low (blocked) → pin 2 high → red LED on, pin 4 low → green off; BUS high
  (clear) → pin 2 low → red off, pin 4 high → green on. Use 1 kΩ LED resistors (about 3 mA); if the lamp is dim,
  replace them with 470 Ω or add a 2N3904 driver.
- 74HC14 pins 14 = +5 V, 7 = GND, with a 100 nF decoupling capacitor. The lamp cable is 3 wires: GND, RED,
  GREEN; the two 1 kΩ resistors and the LEDs live in the receiver top cap.

LM339 pinout (both packages): V+ = pin 3, GND = pin 12; comparator 1 OUT = pin 2, IN− = pin 4, IN+ = pin 5;
comparator 2 OUT = pin 1, IN− = pin 6, IN+ = pin 7; comparator 3 OUT = pin 14, IN− = pin 8, IN+ = pin 9;
comparator 4 OUT = pin 13, IN− = pin 10, IN+ = pin 11. Use two packages for the eight channels.

### 3.4 Optional Arduino Nano monitor

Tap each TSSP4038 output **before** its link (at the receiver end of Jk) through a 1 kΩ series resistor to Nano
D2...D9. The Nano's 5 V pin is fed from the same regulated 5 V rail, or the Nano is powered by USB with the external
5 V disconnected — never both at once. A short sketch prints the eight states and is only a setup aid:

```cpp
void setup() { Serial.begin(115200); for (int p = 2; p <= 9; p++) pinMode(p, INPUT); }
void loop() {
  for (int p = 2; p <= 9; p++) { Serial.print(digitalRead(p)); Serial.print(p < 9 ? " " : "\n"); }
  delay(100);
}
```

`1` = beam blocked (receiver output high), `0` = beam clear. The monitor keeps working when a link is removed, which
is how you prove that the receiver still works while the lamp ignores it. The lamp does not depend on this sketch.

## 4. Assembly

1. **Print the posts and foot plates.** 40 x 40 x 230 mm posts, 2 mm walls, 8 lens bores of 10 mm at 20 mm pitch,
   centres at 57, 77, 97, 117, 137, 157, 177 and 197 mm above the table; removable back cover; cable entry at the
   bottom. The field edges are 52 and 202 mm (150 mm protected height). Print or drill the 90 x 90 x 12 mm foot plates
   with 4x M4 slots. Glue a 10 mm plano-convex lens into each bore, convex side outward, and leave the LED/TSSP at the
   focal point about 10 mm behind it. Deburr every bore; a ragged lens seat changes the effective beam.
2. **Emitter post.** Fit the eight TSAL6200s behind the lenses, at the focal point, anode toward the 150 Ω resistor,
   cathode to the common bar. Solder the eight 150 Ω resistors to a small stripboard, add the 100 µF/100 nF, and bring
   out the 3-wire cable. Check each LED with a phone camera held at arm's length (IR appears as a faint purple/white
   spot) before closing the post; do not hold the emitter against the lens.
3. **Receiver post.** Fit the eight TSSP4038s behind the lenses, at the focal point, lens centred, pins 1/2/3 as marked.
   Solder a 100 Ω resistor in each Vs lead and a 100 nF from Vs to GND. Bring out the 10-wire ribbon and label
   CH0...CH7.
4. **Main board.** Build the 555 and 2N2222 emitter driver, the eight link headers, the two LM339s with their input
   dividers and 1.24 V reference, the BUS pull-up, and the 74HC14 lamp driver. Keep the BUS and the comparator inputs
   short; add 100 nF at each comparator input and at each IC supply. Mount the board in the project box and label every
   connector and link.
5. **Lamp.** Fit the red and green LEDs in the receiver top cap with their 1 kΩ resistors, wired to the 3-wire lamp
   cable. The lamp must be visible from the tester's normal position in front of the field.
6. **Mechanics.** Bolt each post to its foot plate. Set the two posts with **400 mm between the lens faces**
   (the post centres are then about 440 mm apart), square to the table and parallel to each other. Clamp the foot
   plates; do not rely on friction. Check with a 400 mm spacer and a plumb line. The 10 mm lenses give about ±5 mm of
   alignment tolerance; adjust the foot slots until every channel is clear.
7. **Power and static test.** Connect the 5 V supply through the 1 A fuse. Fit all eight links on both fixtures.
   Both lamps must be green with no rod. Pass a card (or a finger) through each beam in turn: the lamp must go red and
   return green. If a channel does not switch, check its lens alignment, its 100 Ω/100 nF, its link and its
   comparator divider.
8. **Optional monitor.** Connect the Nano and confirm that each channel changes 0/1 as the beam is blocked. Remove
   one link: the monitor still shows that channel changing, but the lamp no longer reacts to it. Refit the link.
9. **Trial furniture.** Mark the three lanes on the table at −150, 0 and +150 mm from the field's middle (25 mm
   tolerance), and mark the field bottom/top at 52/202 mm on the posts. Put the two tag holders 15 cm in front of the
   fields, beside them, and the tag rack between the fixtures. Make the 30 x 300 mm rod and the 76 x 127 x 5 mm tag.
10. **Final check.** Both lamps green, all links fitted, rods on their saddles, tag in the rack, holders empty. Then
    make the draw (section 5) and start the trial.

## 5. Fault draw and reset

**Draw (before each trial, by a second person; the tester does not see it).**

- Flip a coin: **heads = LEFT fixture, tails = RIGHT fixture** (left/right as labelled on the posts, i.e. the
  robot's left/right in the simulation).
- Roll an eight-sided die: **1 = CH0 (57 mm), 2 = CH1 (77 mm), ..., 8 = CH7 (197 mm)**. If only a d6 is available,
  roll it and flip a coin: heads uses 1–6; tails with a 1 or 2 uses 7 or 8; tails with 3–6 is a reroll. That gives
  each of the eight channels the same chance. A phone random-number app with a recorded result is also acceptable.
- Record on a slip: date/time, coin, die, fixture, channel, height = 57 + 20 x channel mm, setter's initials. Keep the
  slip out of the tester's view until the trial is scored.
- Fit all eight links on both receiver boards. On the drawn fixture only, remove the drawn channel's link. Leave the
  other seven fitted. The sound fixture keeps all eight.
- Power-cycle and check: both lamps green with no rod; the drawn channel's receiver still toggles on the Nano monitor
  (if fitted), but the lamp does not react to it. If the lamp reacts, the link is not the one you think it is; recheck
  the labels.

**Reset after the trial.**

1. Power off. Refit the removed link so both fixtures have all eight links.
2. Put each rod back on its saddles, the tag in the rack, and leave both holders empty.
3. Power on and confirm both lamps green. The next trial starts with a new coin flip and die roll; never reuse the
   previous draw.

## 6. Real-trial checklist

- [ ] 5 V 2 A supply, 1 A fuse, correct polarity; no exposed mains wiring on the bench.
- [ ] Both lamps green with no rod; all eight links fitted on both fixtures.
- [ ] Rods on their saddles, tag in the rack, both holders empty.
- [ ] Lanes marked at −150, 0, +150 mm; field edges at 52/202 mm; stopwatch and record sheet ready.
- [ ] Draw made by the second person and recorded: fixture + channel + height.
- [ ] Drawn link removed on the drawn fixture only; other seven links fitted; sound fixture has all eight.
- [ ] Sound fixture: lamp red at every height in all three lanes (no green anywhere).
- [ ] Faulty fixture: lamp green only near the drawn channel's height, in all three lanes; red everywhere else.
- [ ] Each lane swept tip first, rod level, hand outside the field, one continuous run of **at least 3 s** for the
  150 mm lane (at most 50 mm/s). The rod's lowest and highest positions reach the bottom and top edges.
- [ ] Tag placed in the faulty fixture's holder only, after that fixture's whole field was covered; sound holder empty.
- [ ] Both rods out of both fields; hands/grippers clear of the tag and rods; final lamp state recorded.
- [ ] Result recorded; links refitted, rods on saddles, tag in rack, holders empty.

If the sound fixture shows green anywhere, stop: the fixture is misaligned or faulty, and the trial does not count.
If the faulty fixture shows green away from the drawn height, or red at the drawn height, check the link labels,
the channel map, the lenses and the 1.24 V reference before repeating.

## 7. Safety (low voltage only)

- The whole fixture is **5 V DC**. Use a certified 5 V 2 A adapter or a bench supply set to 5.0 V with a 1 A current
  limit. Do not connect mains wiring to the fixture. Keep the supply off wet surfaces.
- Fit the 1 A fuse in the +5 V lead; check polarity before the first power-up. Unplug before rewiring or changing a
  link.
- The emitters are 940 nm IR LEDs. The beam is invisible; do not stare into an emitter or a lens at close range and
  do not view it through a magnifier or other optical instrument. A phone camera at arm's length is acceptable for a
  quick emitter check. Treat the emitter as a bright source even though it looks dim.
- This is a training/test aid, **not a safety device**. It guards nothing. Never connect it to a machine's safety
  circuit, never use it to bypass or "prove" a real curtain, and never alter a real safety curtain.
- Deburr the 3D-printed posts and foot plates. Cap both ends of the aluminium test rod; do not swing it. Clamp the
  posts so they cannot fall. Keep the tag's edges rounded.
- If using a USB cable for the Nano, use either the USB 5 V or the external 5 V, not both. Keep the 5 V and ground
  common to all boards.
