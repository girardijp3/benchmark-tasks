# TWIN_CHECK — flat-pack-cam-lock

## Fresh live query (builder, 2026-11-17T21:58:40Z)

```
gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 \
  --json number,title,state
```

Returned **2,538 PRs, newest #2539**: **466 OPEN, 1,865 CLOSED, 207 MERGED**. This is a newer board than the
BUILD_QUEUE snapshot, so the query was re-run at build time as the queue policy requires. The full result was searched
for the object + mechanism + outcome; a different tool alone is not a different task. A second search over furniture /
cabinet / shelf / drawer / dowel / assembly titles returned 109 PRs, all reviewed below and in the table.

## Object + mechanism + outcome of this task

**Object:** one real 15 mm eccentric furniture cam and one mating M6 connecting bolt with two 16 mm panels.
**Mechanism:** the robot drops the cam into a 15 mm bore, inserts the bolt square into an 8 mm hole in the mating panel,
engages a carried screwdriver in the cam's cross recess and turns the cam ~180 deg; the eccentric draws the panels from
a 3 mm gap to flush. The 0.3-0.6 N m torque is declared.
**Outcome:** a flush, undamaged joint held 1 s with both grippers home; a cam turned while unseated, a cross-threaded
bolt or panels forced flush are scored caps.

## Search result

No PR in the whole 2,538-row board has this object + mechanism + outcome. In particular there is **no OPEN task** that
*places* a single 15 mm cam and its bolt, then uses a *carried* screwdriver for a half turn to draw two panels flush
with cam-not-seated / cross-threaded / forced-flush caps. The live-twin search is **clear**.

## The near twin — #1520 (OPEN, three times not accepted)

**#1520 "Add task: Join a shelf to a cabinet side with cam locks: cams open to the edge, push the shelf on, turn them
tight"** is the closest PR and is documented here rather than treated as a block. Its review history is three
"Not accepted" decisions (TK-NVNXRJ, TK-85Q8WJ, TK-BC5NRR), the last a REVISE whose feedback asked for compatible
bolt-head/hole sizes, realistic cross-recess engagement and removal of artificial screwdriver support. It is still OPEN.
It differs from this task in every scored respect:

| Aspect | #1520 (near twin) | This task |
|---|---|---|
| Joint count | **two** cams and two bolts joining a shelf to an upright cabinet side, bimanual | **one** cam and one bolt, one arm |
| What the robot places | cams are already in the shelf; bolts already in the side; the robot only opens cams, pushes the shelf on, turns and returns tools | the robot **drops the cam into its pocket** and **inserts the bolt into the mating panel** — both are scored ladder steps |
| Screwdriver | screwdrivers stand in holders; each hand turns a cam in two quarter turns with a regrip; a tip within 2 mm turns a cam (the last review's "artificial support" and "proximity turning" objections) | one screwdriver is **carried by the gripper throughout** the turn, tip in the recess; no regrip detached from the cam, no proximity turn of an unsupported tool |
| Torque | no torque band; a 10 N breakout force gate was removed after review | a declared **0.3-0.6 N m** hand-torque band, held 0.35 s |
| Failure model | board breakout / 9 mm short-stop / geometry gates that failed three reviews | three geometric caps tied to the scored steps: cam not seated, bolt cross-threaded, panels forced flush |
| Domain | Societal relevance, after furniture-specific occupations were found insufficient for the jobs threshold | **Impact on jobs**, with an aggregated 500k+ case (1.8M assemblers, 890k carpenters, 2.94M hand laborers/material movers) |

The same real product family (15 mm Minifix-type cams and M6 bolts) appears in both, which is why this is a near twin;
the object count, the robot's placement actions, the tool interaction, the torque and the caps make it a different
task, and this task avoids the specific flaws for which #1520 was revised three times.

## Nearest furniture-assembly PRs (and why each is a different task)

| PR | State | Title | Difference |
|---:|---|---|---|
| **#1520** | OPEN | Join a shelf to a cabinet side with cam locks | Near twin: two pre-installed cams, screwdrivers in holders, force/short-stop gates. See table above. |
| **#2003** | OPEN | Join two boards on three glued dowels, pressing them together evenly | Glue and three dowels, not a cam-and-bolt joint; no screwdriver turn or torque. |
| **#1902** | MERGED | Roll a cabinet shelf wider than its face-frame opening through it, level it inside, and lower it onto its four pins | Shelf insertion on pins through a face frame; no cam, bolt or tightening turn. |
| **#1946** | CLOSED | Install two hook-in gondola brackets at a marked height and seat the steel shelf | Hook-in brackets and shelving; no eccentric cam or bolt draw. |
| **#2407** | CLOSED | Add cabinet_hinge task | Door hinge, not a carcass cam joint. |
| **#2258** | CLOSED | Square up a racked cabinet face frame on the bench until its diagonals match | Squaring a frame, not joining panels with a cam. |
| **#1130** | CLOSED | Add laminate-drawer-front: pull the spacer dowels and bond the laminate to the drawer front | Dowel alignment and bonding, not a cam-and-bolt draw. |
| **#940 / #180** | CLOSED | Clamp a blank with an eccentric cam | An eccentric *clamp* on a fixture, not a furniture cam joining two panels; workholding, REJECT. |
| **#82** | CLOSED | Add reverse-turn-lock task | A key-operated cam lock on a box with a slide lid; no furniture joint, bolt draw or panel flush, REJECT. |
| **#2068** | OPEN | Put a screwdriver away in a tool chest drawer while the drawer beside it stands open | Storing a screwdriver, not using one to turn a cam. |

## Blocked/excluded memory checked

The queue's excluded list and the existing `benchmark-submission/tasks/*/BLOCKED.md` family were reviewed. None blocks a
single-joint 15 mm cam-and-bolt tightening task. The repeated non-acceptance of #1520 was read in full and its three
construct-validity objections (bolt geometry, head/hole compatibility, artificial screwdriver support) are each
addressed above by design and in `REFERENCE.md`. No builder `BLOCKED.md` applies here.

**Verdict:** no live OPEN twin, no builder block; the task is clear to build. #1520 is OPEN and a documented near twin,
not a duplicate: it is a different object count, a different robot action set, a different tool interaction and a
different failure model.
