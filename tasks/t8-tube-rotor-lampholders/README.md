# Relamp a T8 batten: fit the tube in both rotor lampholders

A building-maintenance task. A new T8 tube lies on the bench and a wall-mounted batten waits with two rotor lampholders. The robot has to fit it the way a maintenance worker does: both pin pairs have to enter both holders at once, and the tube then has to be turned a quarter turn until both rotors click. The instruction states only the goal; that the tube is carried level to the batten, pushed in, and turned, and how the hands share it, are the robot's to work out.

The skill is a long part fitted into two fixtures at once (a 590 mm tube, 12.7 mm bi-pins (2.47 mm) entering 3.4 mm notches through a 30 degree lead-in), then a quarter turn of the whole tube by two hands together, against a click detent. One hand cannot do it: the tube has to be held at both ends so the two pin pairs line up.

**Why it matters.** General maintenance and repair workers (SOC 49-9071, about 1.5 million in the US) relamp linear batten fittings as routine upkeep of warehouses, car parks, schools and plant rooms, where a crew changes hundreds of tubes a shift; LED replacement tubes keep the same G13 bi-pin caps and fit the same holders, so the fitting job stays. In this cell the batten is mounted on an upright board at bench height, as on a workshop test board; on site the same batten is on a wall or ceiling.

**How this differs from merged #175** ("Fit a two-pin lamp tube into its sockets and turn it to lock"). Both fit a two-pin tube at both ends and turn it a quarter turn. In #175 an 18 in tube is lowered from above down the slots of two fixed sockets (gravity seats the pins) and a spring plunger in each of the tube's own end caps clicks into its socket. Here the holders are the G13 rotor lampholders of a real T8 batten (BJB 26.422 class, 90 degree rotor): the 590 mm tube has to be carried level in the air at the holders' height and pushed sideways so both pin pairs enter the front notches and the rotor slots at once (IEC G13 pins at their 2.47 mm maximum in 3.2 mm slots, nothing for gravity to seat), and it is the holders' rotors that turn with the pins to their stops; the tube lights only when both rotors are at their stops, and the pins are retained behind the holder walls.

| File | What it is |
|---|---|
| `t8_tube_rotor_lampholders.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/batten.usda` | The batten with its two rotor lampholders (OpenUSD): fixed-root articulation, each rotor on a revolute joint about the tube's axis |
| `assets/t8_tube.usda` | The T8 tube (OpenUSD): free articulation with four visual strips on prismatic joints that show when it is lit |
| `assets/cradles.usda` | The two bench trays with end stops (OpenUSD): fixed |
| `build_assets.py` | Writes the USD files |
| `test_t8_tube_rotor_lampholders.py` | Offline checks: every scoring outcome, the 1 s timing, both arms' reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All created for this task by its authors, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Metres,
kilograms, Z up. Sizes are typical values of the real product class, not one catalogue entry.

- **Batten** (fixed root): a 680 x 80 mm pan with a 760 x 190 mm back board, 53 mm behind the tube's axis, and two lampholders. Each holder is a housing (a ring of boxes round a 10 mm pocket, with the front open as a notch 3.4 mm wide and two 30 degree lead-in plates opening to 13.4 mm) and a rotor (two 1.7 mm walls 3.2 mm apart and a floor, 9.4 mm radius, on a revolute joint about the tube's axis, drive stiffness 0.1 N m per 45 degrees). The housing block itself is a visual shell; the 14 mm thick holders sit 298 mm either side of the middle.
- **Looks** (visual only, colliders unchanged): the tube is white phosphor glass with aluminium end caps, brass pins, dark groove rings and a printed rating label; the batten is a white pan with screw heads and folded edges on a light board; housings are white with light rotors.
- **T8 tube** (free articulation, 0.13 kg): 26 mm across, 590 mm cap to cap, two bi-pins on each end (2.47 mm, the IEC 60061 G13 maximum, 12.7 mm apart, 7.5 mm out of the caps), glass friction 0.6 / 0.5. Four 8 mm wide visual strips on prismatic joints slide out 3.5 mm when the tube is lit, as bright yellow bars standing about 2.3 mm proud of the glass (hidden inside it, below the glass surface, otherwise; no collider).
- **Trays** (fixed): two 40 mm wide flat trays between 8 mm side walls, with end stops 1.5 mm outside the caps below the pins' height.
- Fidelity, stated plainly:
  - **Real parts.** G13 pin spacing is 12.7 mm and the pin diameter lies between 2.35 mm and 2.47 mm (IEC 60061-1 sheet for G13; the assets use the 2.47 mm worst case, which leaves 0.365 mm a side in the 3.2 mm slot). The pin length could not be checked against the standard in open sources: 7.5 mm is an estimate. BJB's public G13 pages (rotor lampholders such as 26.422.1313, 26.308.1125) list a 90 degree rotor and, for the spring type, 2 mm of lamp length compensation (3 mm for a pair); they do not say whether the lamp enters from the front or the top.
  - **Simplified.** The holders are front-entry (the pins push in along +x through an open notch with a lead-in) with a fixed length and no leaf spring: the 2 mm length compensation of BJB's spring type is omitted (the tube sits 3 mm clear of each holder's inner face instead). A real top-entry holder would need the tube lowered, not pushed; this task does not model that.
  - **Estimates, not measurements.** Slot 3.2 mm, notch 3.4 mm, 30 degree lead-in, pocket 10 mm, rotor wall length, pin length, and the click torque (0.1 N m peak, to be measured with a torque gauge on a real holder).
  - **Declared detent and lamp.** The click is a drive that pulls a rotor on from 45 to 90 degrees (and one short of 45 back to 0); it is not a modelled spring-pawl. The lamp is four visual strips that slide out 3.5 mm (bright yellow, visible from outside) only while both pin pairs are in and both rotors are at 90 +-5 degrees; there is no electrical model. The strips show in the video.

## Layout

The holders' axis is at x 0.42, 0.115 m above the table; the trays at x 0.33. Both arms grip the tube at y +-0.20 with the pads closing along x (straight down), then tilt together about the tube's axis.

## Run it

On the YAM Isaac Lab-Arena station, from this folder. The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=t8_tube_rotor_lampholders.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/t8-tube-rotor-lampholders-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=t8_tube_rotor_lampholders.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 760 --store-frames --log-dir ~/yam-eval/t8-tube-rotor-lampholders-demo
```

The log prints `[t8lamp] t=...  steps done: ...` on every change and a trace each second.

**The demo.** Open loop: fixed waypoints, no simulator reads. Both arms take the tube from the trays by pads closing along x (about 20 N a finger), lift it 16 mm, carry it to 40 mm in front of the notches, lower it to the holders' height and ease the grip to a light hold (about 4 N a finger). They push it 40 mm in along +x: the pins ride down the lead-in and into the slots. The grip firms up again (about 20 N), and both wrists tilt together from 0 to -92 degrees about the tube's axis in 9 s, which turns both rotors to their stops; the click pulls them on at 45 degrees. After a short hold they let go and back away along -x. The tube hangs about 2.5 mm low in the pockets. Success at step 627 (31.35 s).

## Scoring

| Step | Done when |
|---|---|
| 1 | both pin pairs are in both holders (axis within 3.5 mm of the holders' at both ends, within 2.9 mm of the middle along y, level within 3 degrees) |
| 2 | with step 1, both rotors have turned 45 degrees or more |
| 3 | with step 2, both rotors at 90 degrees within 5: the tube is lit |
| 4 | with step 3, no gripper holds the tube and no pad is within 3 cm of it |

The lamp is lit only while the tube is in both holders and both rotors are at their stops.

- **1.0** only when every full-success condition has held together for 1 s, timed in the task file:
  all four steps, no rule broken, and the parts within 2 mm and 2 degrees of where they were when that second began. A
  lapse restarts the second. The arms' return is not scored.
- **0.2 per step, at most 0.8** otherwise.
- **0** after any rule event (the tube dropped or off the table); the event is kept for the whole run, however it is put right.
- Doing nothing scores 0 (run). Shortcuts: turning the rotors without the tube in them is not possible (the tube turns them), a tube lying in front of the holders scores nothing, a rotor short of its stop leaves the tube dark, and a tube released before the quarter turn leaves the step ladder at 0.2 (all tests). Left alone, either arm alone scores 0 in MuJoCo.
- **Real station (instruments):** the tube lit (power the batten); a protractor or the rotor's own markings on both holders (at the 90 degree stop); a tug forward on the tube (it does not come out of the notches); a ruler for the 3 cm hands-clear distance and a phone video for the 1 s of stillness.

Isaac checks: idle 0 and either arm alone 0 (300 steps each); the scripted demo scores the same 1.0 at step 627. In the demo video the lampholder housing shell, its back plate and the rotor floor are hidden (render-only, in the filming copy; collision and physics unchanged) and one camera looks end-on at a holder, so the pins, the rotor slot and the rotor turning to its stop can be seen; the ring and lead-in colliders are what the pins meet. The yellow bars on the tube appear only when the tube is lit, and the video ends on the 1.0 score.

## Reset

Lay the tube in its trays with its pin line horizontal and its caps between the end stops; both rotors at 0 degrees, the lamp off. Nothing is held.

## Fidelity notes

Full text of the task.json fields, shortened there to meet the length limits:

Fidelity: Declared: the rotor click (a drive pulls a rotor past 45 deg on to 90 and one short of it back to 0; peak 0.1 N m, an estimate) and the lamp (four strips show only with both pin pairs in and both rotors at 90 +-5 deg). Holder slot, notch, lead-in and pin length (7.5 mm) are estimates (G13 pin spacing 12.7 mm and pin diameter 2.35 to 2.47 mm are IEC 60061; the pins use the 2.47 mm maximum; pin length was not verified). Holders are front-entry with a fixed length; BJB's leaf-spring length compensation (2 mm) is omitted, and BJB's pages do not say whether their rotor holders are front- or top-entry. The lamp is four visual strips, not an electrical model. Plastic on brass friction 0.30/0.25; tube on pads 0.6/0.5.

Context: Maintenance and repair workers, general (SOC 49-9071, about 1.5 million employed in the US, BLS OEWS) relamp linear batten fittings as routine upkeep of warehouses, car parks, schools and plant rooms, where crews change hundreds of tubes a shift; LED replacement tubes keep the same G13 bi-pin caps, so the job stays. In a T8 batten each end of the tube sits in a G13 rotor lampholder: both bi-pin ends are pushed into the holders' front notches at once, with the 590 mm tube held level in the air at the holders' height, and the whole tube is turned a quarter turn so both rotors turn with the pins to their 90 degree stop, which closes the contact; the tube is then retained and lit. One hand cannot do it: both ends have to enter at once. Here the batten is mounted on an upright board at bench height, as on a workshop test board; on site it is on a wall or ceiling. Unlike merged #175 (an 18 in tube lowered down the slots of two fixed sockets and locked by spring plungers in its own caps, gravity seating the pins), here the pins enter sideways into the holders' rotors, which turn and light the tube only when both reach their stops.

## Status

Dream-1 score: 0.0 (1600 steps, hit the step limit). Demo: 1.0 at step 627 (31.4 s). Media M-97BDXU.
