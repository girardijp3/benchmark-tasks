# REFERENCE — locking-carabiner

## What the real object is

A **locking carabiner** (a "locker") is a load-bearing connector used as personal protective equipment. Its body is a
light-alloy oval or D loop; one side is a **spring-closed gate** hinged at one end and latched at the nose. A **manual
screw-lock sleeve** — a knurled collar around the gate — is screwed down after the gate is closed; it blocks the gate
from opening. Petzl's connector guide describes SCREW-LOCK as a manual locking system whose red band is visible only
when unlocked, recommended for harsh environments and connections left in place. Professional connectors are certified
to **EN 362** (fall protection), climbing ones to **EN 12275 / UIAA 121**. The act of clipping one to an anchor — open
the gate, hook it, close the gate, screw the sleeve fully shut, visually check and pull-test — is standard rigging,
arborist and climbing practice.

The task models a **compact ~60 mm locking carabiner** (the brief's size) and a fixed **anchor eye** (an eye-bolt post
with a ring on top) that the carabiner hooks over.

Sources: Petzl *Connectors* technical catalogue (SCREW-LOCK system, red unlocked-warning band, EN 362); SingingRock
COLT screw and OXY screw product sheets (48-74 g, 18-21 mm gate opening, EN 362 / EN 12275 / UIAA 121); DMM Phantom
Screwgate (42 g, 94 x 54 x 100 mm, 15 mm gate opening, EN 362 / EN 12275 / UIAA 121); Black Diamond RockLock / LiteForge
screwgate sheets (45-85 g, 15-25 mm gate opening, EN 362 class); SKYLOTEC screw-lock karabiner guidance (the sleeve is
tightened after attachment, the connection is visually checked and proof-loaded). **BLS OEWS May 2025** and **BLS
Employment Projections 2025-35** for the routine-importance case (riggers 22,530; tree trimmers and pruners 55,160 /
70,600; construction and extraction 6.4 million). No single manufacturer's drawing is public for a generic 60 mm
locker, so the internal dimensions below are estimates within the stated envelope; each is flagged.

## Measured / given by the product class and the brief

| Quantity | Value | Source |
|---|---|---|
| Carabiner class | locking carabiner, spring gate, manual screw-lock sleeve | product class |
| Carabiner size | ~60 mm | brief |
| Certifications (class) | EN 362 / EN 12275 / UIAA 121 | Petzl, SingingRock, DMM, BD catalogues |
| Locking system | screw sleeve, tightened after clipping, visually checked | Petzl / SKYLOTEC guidance |
| Gate | spring-closed, opens to clip, must be shut and locked | product class |
| Proof check | visual sleeve check + pull-test before use | SKYLOTEC / rigging practice |
| Post / anchor | eye-bolt (stem + ring), fixed | product class / brief |
| Workforce | riggers 22,530; arborists 55,160-70,600; construction 6.4 M | BLS OEWS / EP 2025 |

## Estimated (stated, not scaled down)

| Quantity | Value | Why |
|---|---|---|
| Outline / wire | ~60 mm outline, 6 mm wire (3 mm radius) | a compact locker; real lockers are 42-85 g with 5-7 mm stock |
| Mass | 50 g (45 g body + 5 g gate and sleeve) | between DMM Phantom (42 g) and SingingRock COLT (48 g) |
| Gate | 34 mm bar, 60 degree inward swing, 0.05 N m/rad spring, closed stop at the nose | a spring gate light enough for a 5-15 N push; DMM gate opening 15 mm class |
| Sleeve | knurled collar on the gate near the hinge, gold/yellow | Petzl's unlocked-warning colour family; visual only |
| Lock twist | 75 +/- 15 degree window; demo turns 100 deg | a deliberate quarter turn of a screw sleeve |
| Anchor eye | 240 x 200 x 8 mm base, 6 mm post 40 mm above the table, 24 mm ring | a rigid eye-bolt stand within the arm's reach |
| Layout | post at x 0.345, y 0; carabiner spine starts at x 0.255 | within 0.38 m forward reach |
| Hooked geometry | post axis inside the loop polygon, plane within 45 deg of horizontal | a real pose test the scorer reads |
| Pull-test | post slid to 46 mm down the 54 mm loop, gate shut, held 1 s | the proof-load stand-in (no force integration) |
| Thread in sim | declared from wrist yaw while above the sleeve | like the family's declared latches/twists |

## Deliberate simplifications (honest scope)

* **Declared sleeve thread.** The knurled sleeve is modelled on the gate link in the USD (it is visible in the setup and
  the demo). Its *rotation* is not a simulated helix: while the right gripper turns within 30 mm of the sleeve with the
  gate shut, the task accumulates the wrist's yaw; a quarter turn sets the `locked` flag the scorer reads. The demo
  hovers 18 mm above the sleeve so the declared turn does not drag the clipped carabiner around the post. This is the
  same declared-mechanism family device used for lid detents, valve stem torque and luer twists. A real hand grips the
  sleeve; the robot must still reach, orient, turn and then proof-pull.
* **No force integration for the proof load.** The pull-test is a geometric state (eye at the gate end of the loop, gate
  shut, sleeve locked, held 1 s), not a simulated load. Real riggers check by eye and by hand pull; the scorer checks
  the same captured state.
* **The spring gate is real.** The gate is a revolute joint with a spring and a closed stop, and the post opens it by
  real contact; the hook is a real geometric test. Only the sleeve thread is declared.
* **Anchor fixed.** The eye-bolt is bolted to a base, so the skill is one-handed clip-and-lock, not "hold the anchor
  with one hand".
* **Visual-only knurl.** The sleeve's notches and the anchor's ring are visual geometry; only the carabiner's body,
  gate and the gripper pads carry colliders.

## What is NOT simplified

Real product size throughout (60 mm outline, 6 mm wire, 50 g, 34 mm gate, 6 mm eye-bolt, no scaling); the gate really
opens on the post and really springs shut; the eye must be inside the loop before the gate counts as closed; the sleeve
needs a declared quarter turn; the pull-test needs the eye at the gate end of the loop; a gate left open, a loose sleeve
and a dropped carabiner are all scored; the gripper must do the work; and full score needs a 1 s still hold. Idle and
every cap are scored.
