# REFERENCE — cervical-collar-shells

## What the real object is

A **two-piece rigid cervical collar** (the Philadelphia / Aspen / Miami class of rigid neck brace). Recognisable
features: a rigid **front shell** with a chin cup that slides under the jaw, a rigid **back shell** with an
occipital rim that wraps behind the neck, soft foam liners, and **side tabs** on each shell that overlap at the
sides of the neck and are pressed together until they click/lock. It is applied by EMTs, paramedics, nurses and
first-aid responders to limit neck movement after a suspected spinal injury. Correct fit means the chin sits in
the front cup, the back shell is centred behind the neck, both tabs are fully seated at a firm hand press (not
slammed), and the collar is neither rotated nor pressing on the neck.

Sources: public catalogue/retail and clinical-training descriptions of two-piece rigid cervical collars (front
chin-support shell, back occipital shell, side tabs, foam liner, one-size-fits-most ~300 mm width); general
first-aid/EMT procedure descriptions of two-person or two-hand collar application. The brief supplies the
force envelope (4-7 N per tab) and the mass/size class (~300 mm, 0.3 kg). No single manufacturer's internal
drawing is public, so the individual shell dimensions below are stated estimates, each flagged; none is scaled
down from the real product.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Object class | two-piece rigid cervical collar on a care-training manikin | product class |
| Front shell | rigid chin-support shell with chin cup + foam liner | product class |
| Back shell | rigid occipital-support shell with rim + foam liner | product class |
| Side tabs | two 30 mm tabs, pressed to lock the shells | brief |
| Tab press | 4-7 N until seated | brief |
| Overall width / mass | ~300 mm across the tabs; ~0.3 kg total | brief |
| Work height | two-hand fit at <= 0.3 m above the bench | brief / station gate |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Front / back shell mass | 0.14 / 0.16 kg | split of the stated 0.3 kg total, back shell slightly larger |
| Shell width / height | ~110 mm wide x ~65 mm tall | typical adult rigid collar panel; 30 mm tabs at the sides |
| Shell thickness | 6-10 mm rigid plastic + 3 mm foam liner | typical rigid-collar construction |
| Manikin neck | radius 42 mm, local z 0.040-0.160 | adult training manikin; collar seats at z 0.120 |
| Manikin head | radius 55 mm sphere, chin at local z 0.160 | simplified training head |
| Tab press travel | 4 N = 5.7 mm, 7 N = 10 mm (K_TAB 700 N/m) | a compliant tab stand-in giving the 4-7 N band |
| Shell seat poses | front x 0.285, back x 0.215, both y 0, z 0.120 | under chin / behind neck, within YAM reach |
| Manikin / shells | fixed welded manikin; gravity-free shells, no colliders | declared carry, as in the family's welded fixtures |

## Deliberate simplifications (honest scope)

* **Declared carry.** While a gripper is closed across a shell's grasp point, the shell follows the gripper. The
  robot must still reach, close on, orient, carry and seat each shell. The shells and manikin have no colliders,
  so the gripper can close at the grasp point; this is the same "declared mechanism" device used by the family's
  welded fixtures and detents. It avoids a soft foam edge slipping through a parallel jaw, which is not the skill
  under test.
* **Tab force.** The 4-7 N press is a declared 700 N/m compliant travel measured from the gripper TCP's real
  displacement in MuJoCo/Isaac: 5.7 mm (4 N) just seats a tab, 10 mm (7 N) is the forced-fit limit, and the demo
  presses 8 mm (5.6 N). The force is not integrated as a contact force.
* **Manikin fixed.** The manikin is a welded, visual-only body with no colliders, so the fit is a two-hand
  manipulation task rather than a "hold the patient's head" task. "Neck moves" is a declared cap fired when a
  shell is knocked off the bench or jammed at over 20 degrees.
* **Two arms work one at a time.** They physically collide if both reach the manikin's neck together; the scripted
  demo sequences them (left seats its shell and goes home, then right, then each presses its tab). This is a
  station-realism choice, not a scoring shortcut: both hands are used and both end home.

## What is NOT simplified

Real product size and mass throughout; two distinct rigid shells with true 0.14/0.16 kg masses, a real chin cup
and occipital rim, 30 mm tabs and foam liners; each shell must be reached, grasped, oriented and seated on the
manikin's neck; each tab must be pressed to a measured 8 mm (5.6 N) in the 4-7 N band; the scored state requires
both shells seated, both tabs engaged, no cap, both grippers clear and home, and a 1 s hold. Idle and all three
failure caps (neck moves, tab miss, forced fit) are scored.
