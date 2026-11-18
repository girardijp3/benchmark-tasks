# Live twin check: panel-push-clip

**Query at launch: 2026-11-17T22:00:58Z**, fresh
`gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000 --json number,title,state`.
Returned **2,538 PRs**, newest **#2539**: **466 OPEN, 1,865 CLOSED, 207 MERGED**. This is a fresh query, later than the
analyst's 19:45Z snapshot in `submission_tools/filter/BUILD_QUEUE.md` (2,512 PRs, newest #2513), so it includes all
newer PRs. The board token from the query is saved at `/tmp/opencode/panel_push_clip/prs.json`.

## Title searches (this board snapshot)

| Pattern | Hits | Notes |
|---|---:|---|
| push-clip family (`push.?clip`, `snap.?clip`, `panel.?clip`, `trim.?clip`, `retaining.?clip`, `push.?rivet`, `push.?fit`) | 2 | #1654 CLOSED "Push-fit an elbow onto a copper and a plastic pipe...", #662 MERGED "Release a push-fit fitting and pull the pipe out" — both plumbing push-fit, not panel clips |
| `clip` (whole word) | 31 | #1986 (nearest, below), #1378, #2150, #2356, #2492, #2445, #2421, #2279, #1905, #2189, #2179, #1797, #1720, #1136, #1123, #15, ... |
| `pin` (whole word) | 65 | #30 MERGED "Pick the pin that fits the hole and seat it", #1891 OPEN header pins, #2010 CLOSED locating pins, #2060 CLOSED clevis+cotter, #2059 CLOSED heatsink-on-carrier-pin, ... |
| `barb` / `mounting hole` / `flush` | 24 | #1892 MERGED hose-clamp-barb, #1378 CLOSED trim-panel clip press, #1212 CLOSED heat-set insert flush, #1209 CLOSED clip a lead flush, #1099 CLOSED push hose on valve barb, ... |

## The flagged near twin — #1986 (read in full)

**#1986 CLOSED (not merged), 2026-11-16T20:51:34Z** — "Refit a splash-shield panel with a push-in clip: hold its hole
over the body's and push the clip home through both". Adds `tasks/trim-clip/`. This is the PR another builder flagged;
I fetched and read its `task.json` and `README.md` from head sha `e8ecd21d`. It is the closest title on the board and
uses the **same real product class** (an 8 mm-hole / 20 mm-head nylon push retainer, Lawson P53958). It is nevertheless
a **different task**:

| | #1986 `trim-clip` (CLOSED) | this task `panel-push-clip` |
|---|---|---|
| object handling | **bimanual**: one arm slides a **loose 8 x 20 cm splash-shield panel** on a flange and holds its hole aligned while the other inserts the clip | **single arm**: the **panel is fixed**; only the clip is manipulated |
| hole / catch | 11.4 mm **square** holes; the barbs are **two sprung steel jaws in the hole** that the clip's collar spreads | an **8 mm round** hole; the **barbs are on the clip** and are declared to spring behind the panel |
| force | no measured press force; "push the head home" | a **4-7 N press** gate measured from the gripper TCP, 4-7 N throughout |
| pull-test | none | a **4-7 N retention pull held 1 s** against a clip locked in the panel |
| scoring ladder | lined-up 0.25 / home 0.5 / still 1 s = 1.0 (0.9 on timeout); 0 otherwise | aligned 0.25 / pressed 0.5 / barbs seated 0.75 / pull-test held 1 s = 1.0 |
| caps | none | cross-insert / left unseated / dropped / forced fit, each 0.5 |

Different object (loose panel vs fixed panel + clip), different mechanism (jaws in the hole vs barbs on the clip),
different outcome (no force gate or pull-test vs a measured press and a 1 s retention pull). #1986 is also CLOSED, not
merged. **Different task.**

## Other near PRs and why they are not this task

* **#1378 CLOSED** "Back up a springy trim panel while pressing a clip home" — bimanual: one arm backs a springy panel
  with a fork paddle while the other presses a detent clip. Different object handling (a panel to back up), no force
  gate and no pull-test. **Different.**
* **#2150 CLOSED** "Snap a fallen cable clip back onto a robot arm link at its painted band" — a cable clip on a
  carbon-fibre link at a marked band, not a push retainer in a panel mounting hole. **Different.**
* **#1892 MERGED** "hose-clamp-barb" — a hose barb, not a panel push-clip. **Different.**
* **#30 MERGED** "Pick the pin that fits the hole and seat it" — chooses among pins by hole fit; a pin, not a nylon
  push-clip, and no align/press/seat/pull ladder. **Different.**
* **#1891 OPEN** "Seat a robot controller's add-on board on its two rows of header pins without bending any pin" —
  a board on many header pins; no push-clip, mounting hole or pull-test. **Different.**
* **#2010 CLOSED** "Fit locating pins to a fixture and load a bracket onto them so it is located" — locating pins and
  a bracket. **Different.**
* **#2356 OPEN** "Clip a car air freshener onto a dashboard vent vane..." and the other `clip` hits (#2492 air-filter
  cover, #2445 squeegee rubber, #2421 urinary bag, #2279 clipboard, #1905 shower valve U-clip, #2189 table fan,
  #2179 sun visor, #1797 cable clip, #1720 binder clip, #1136 picture frame, #1123 power-cord retention clip) — all
  different clip objects and mechanisms; none is an 8 mm push-clip aligned, force-pressed, seated and pull-tested in a
  fixed panel hole. **Different.**

## Result

**Clear to build.** No live OPEN twin has this object + mechanism + outcome. The flagged near neighbour #1986 is
CLOSED (not merged) and is a bimanual loose-panel refit with square holes, jaws in the hole, no force gate and no
pull-test; the other flagged pin/pr-clip PRs are all different objects and mechanisms. A different clip, panel or
fastening method is not the same task.
