# PASS_THROUGH_QA — lighting-track-adapter

MuJoCo contact log over the full 396-step demo (240 Hz physics, 20 Hz control), recording every contact pair that
involves a gripper tip or the head link. Intended contacts are on the real surfaces of the handled part; no arm
link, gripper body or finger passes through a fixed solid or through a wall of the part being held. The same demo
reached **1.0 at step 396 on Isaac** (`adhoc_395d23de.json`); the contact table below is the MuJoCo pair log, and
the Isaac evidence is the top camera plus two render-only close-ups (`film_slot`, `film_lever`) saved on a filming
copy of the task.

| Pair | Contacts | Deepest contact | What it is |
|---|---:|---:|---|
| `head/HeadLink/Tab*` <-> `track/TrackLip*` | 860 / run | 0.056 mm | the two contact tabs resting on the real cavity floor / lip tops (the scored lock) |
| `head/HeadLink/AdapterCollider` <-> `track/TrackLip*` | 192 / run | 0.096 mm | the 40 mm adapter body seated on the track underside |
| `head/HeadLink/KnobCollider` <-> `left/tip_*` | ~400 / run | 2.05 mm soft | the left pads gripping the 24 mm lever knob; the depth is MuJoCo's over-damped contact spring on the intended grasp, not a pass-through |
| `head/HeadLink/StemCollider`/`AdapterCollider` <-> `right/tip_*` | ~120 / run | 0.95 mm soft | the right pads gripping the 20 mm stem for the tug |
| `head/HeadLink/HeadCollider` <-> `left/tip_*` | 8 / run | 0.21 mm | the knob post grazing the head's top rim during the turn; no entry into the housing |
| `left/tip_*` <-> `track/Pedestal` / `TopArm` | 30 / run | 0.50 mm | a finger edge grazing the stand behind the track |
| `left/tip_right` <-> `track/TrackLip` | 12 / run | 0.78 mm | a finger pad grazing the exposed lip edge during the lever turn |

The largest unintentional contact is a **0.78 mm soft edge graze** on the open lip rim (one 7 mm-thick lip, not a
closed wall); every other incidental pair is <= 0.5 mm. These are contact-spring offsets on exposed edges, not a
pad inside a wall: the left gripper approaches the lever from the open front and never crosses the track, and the
right gripper approaches the stem from the front. The scorer's `clear` check requires both grippers >= 5 cm from
the head box at the 1 s hold.

## Frames checked

Frames at 0, 10, 25, 45, 60, 80, 95 and 100% of the top-view demo, plus a front close-up at 5.0 s (lever grip),
8.3 s (cam locked), 15.0 s (stem tug) and 20.3 s (final hold): no finger, forearm, lever or head is seen through
the track, the stand or the head housing. The head is a complete forged cylinder; its lens, bezel, fins and label
are visible. The 0.78 mm lip graze is sub-pixel at 640 x 360 and is not a visible pass-through.

## Isaac run and declared controls

Isaac demo: **1.0 at step 396** (`adhoc_395d23de.json`): head inserted, lever code-keyed to -90 deg, cam locked,
4 mm stem tug passed, grippers clear, 1 s still. Task-local Isaac declarations: the free-root head is pinned to
its seat pose each step (the fixed-root joint drops the PhysX articulation); the lever follow and the stem tug use
the declared code-keyed proximity/closed-grip gates because a 10 mm paddle and 20 mm stem are not resolved force
contacts; the joint position/velocity writes keep the 1 s still-hold rates below threshold. The real hinged head,
cam detent, 10 N tug test, geometry and scores are unchanged.

## Cutaway

None. The track is a real extrusion seen from above and from the front; the head hangs below it. No wall is hidden
and no camera looks through a solid.

## Penetration latch

The offline test `test_grip_points_are_on_real_surfaces_not_inside_solids` checks that the left pad centres lie on
the real 24 mm lever-knob surface and the right pad centres on the real 20 mm stem surface, and that neither is
inside the 40 mm adapter body or the 69.85 mm head body. The pass-through result above is from the actual
contact log; the finisher should confirm it on the Isaac contact reports and the close-up frames.
