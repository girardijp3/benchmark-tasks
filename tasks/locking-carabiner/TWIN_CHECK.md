# Live twin check: locking-carabiner

**Query at build time:** fresh `gh pr list --repo robocurve/benchmark-tasks --state all --limit 3000
--json number,title,state`. Returned **2,538 PRs**, newest **#2539** (466 OPEN, 1,865 CLOSED, 207 MERGED). The
analyst's `NEW_CANDIDATES.txt` / board snapshot was slightly older, so this is a genuinely fresh query against the
live board, not a cached list. Every line below was searched against this snapshot.

## Title searches (this board snapshot)

| Pattern | Hits |
|---:|---:|
| `carabin` | **2 — both CLOSED** (#591, #2159) |
| `locking carabiner` | **0** |
| `screw gate` / `screw-gate` | **0** |
| `anchor eye` | **0** |
| `gate` | 14 (garden gates, blast gates, flap quadrants, #2159) |
| `sleeve` | 8 (sampler, heat-shrink, bayonet sleeves, clothing) |
| `proof` | 5 (voltage-prover, anchor-plate proof-pull, child-proofing) |
| `clip` | 36 (filters, squeegees, vent clips, clipboards) |
| `rigg` / `climb` / `arbor` | no climbing/rigging carabiner task |

No OPEN PR title contains "carabiner", "locking carabiner", "screw gate" or "anchor eye". No OPEN task combines a
locking carabiner, a screw-lock sleeve and a pull-test outcome.

## The two carabiner PRs — both CLOSED, neither a live twin

* **#591 CLOSED — "Assemble a hanging carabiner chain".** The robot clips **three carabiners together** into a chain
  hanging from a stand, attaching the first to the stand and each remaining one through the one before; partial credit
  per link, gates closed and the robot letting go. It is a **chain-assembly** task with no anchor eye, no locking sleeve
  and no pull-test. Different object arrangement, mechanism and outcome. Its state is CLOSED, so it is not live on the
  board.
* **#2159 CLOSED — "Clip a spring-gate carabiner to a steel tie-down staple" (board TK-CCP2D4, media M-CKTDFJ).**
  This is the closest near neighbour and is **explicitly not a twin**:
  - **Mechanism:** a **spring-gate** carabiner. The staple's leg pushes the gate open and the **spring snaps it shut**.
    There is no screw sleeve and no turning step at all.
  - **Anchor:** a **steel U-shaped tie-down staple** (two legs and a bar), not an **anchor eye** (eye-bolt post + ring).
  - **Outcome:** the carabiner is **clipped** (linked, gate closed, robot clear). This task's outcome is **locked and
    proof-pulled**: the screw sleeve must be fully shut (a quarter turn) and the connection must hold a 1 s pull-test.
  - **Scoring:** #2159 scores lifted / leg through loop / gate closed / grippers clear; this task scores gate opened /
    eye through loop / gate closed / **sleeve locked + pull-test held 1 s**, with distinct **gate-not-closed** and
    **sleeve-not-locked** caps.
  - **State:** CLOSED, not live.

The two tasks are related products but a different locking mechanism (spring gate vs manual screw-lock sleeve), a
different anchor, and a different final skill and scored outcome. Documenting this distinction is the point of this
section.

## Other near neighbours and why they are not this task

* **#1766 MERGED** "Squeeze locking pliers on a tube past their lock point" — locking pliers, not a carabiner/sleeve.
* **#1867 OPEN** "Mount an e-stop button ... thread its locking ring on behind" — an e-stop locking ring, no carabiner.
* **#1711 CLOSED** / **#1489 CLOSED** — removing spring-loaded / multistage **bayonet sleeves**; removal, not screwing a
  carabiner sleeve shut.
* **#1972 MERGED** "Heat-shrink a sleeve over a crimped cable lug" — a heat gun, no carabiner or anchor.
* **#2053 OPEN** "Prove a stick-mounted voltage detector on its proof tester" — a proof test, but electrical, not a
  carabiner pull-test.
* **#1379 CLOSED** "Proof-pull an anchor plate to its test load" — pulling an anchor plate with a load cell, not
  locking a carabiner to an eye.
* **#1285 CLOSED** "Ratchet a tie-down strap taut round a crate" — a strap ratchet, not a carabiner.

## Result

**No live OPEN task with the same object (compact locking carabiner + anchor eye), mechanism (open gate, hook the eye,
close the gate, screw the locking sleeve a quarter turn) and outcome (sleeve fully locked, pull-test held 1 s).** No
CLOSED or MERGED exact-final twin exists either: #591 is a chain assembly and #2159 is a spring-gate clip to a staple,
both documented above and both CLOSED. **Clear to build — proceed.**
