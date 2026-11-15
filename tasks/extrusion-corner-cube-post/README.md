# Seat an aluminium post in the open socket of a frame's corner cube until it sits on the socket floor

A one-arm frame-building task. Test rigs, robot cells, 3D-printer frames and compute-rack carts are assembled from 20 x 20
mm T-slot aluminium extrusion (the "2020" profile). Corner cube connectors join three profiles at a corner: each profile end
goes into a socket of the cube. Here the cube already holds two rail stubs (a corner of a frame lying on the table) and its
third socket points up. A 100 mm post (43 g) stands in a rack beside it.

The socket is 22 x 22 mm and 22 mm deep for the 20 mm profile (1 mm a side, with a 4 mm lead-in chamfer at the mouth). The
post has to be carried over the cube plumb and centred, lowered into the socket and driven fully down onto its floor. Left
on the rim, leaning on a wall, or standing in the socket's mouth it is not seated.

- The left hand lifts the post out of the rack by its upper end, carries it over the cube and lowers it into the socket until
  it is seated, then lets go.
- The post must stay seated and still.

| File | What it is |
|---|---|
| `extrusion_corner_cube.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `steps_done`) and the scripted `demo()` |
| `assets/corner_cube.usda` | The corner cube with its two rail stubs (OpenUSD), a fixture |
| `assets/post_rack.usda` | The post rack (a fixture) |
| `assets/post.usda` | The 100 mm post, one rigid body |
| `build_assets.py` | Writes the three USD files with pxr (boxes and small convex prisms) |
| `test_extrusion_corner_cube.py` | Offline checks: the file loads, every scoring outcome, the 1 s timing, the arm's reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

All three assets were created for this task by its authors, licensed
[CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Units are metres and kilograms, Z up. The profile is the common
2020 section (20 x 20 mm, about 0.43 kg/m, so 43 g for 100 mm).

- **Corner cube** (`corner_cube.usda`, fixed): a 34 mm cube with a 22 x 22 mm socket open upward (22 mm deep, floor 12 mm above
  the table, a 4 mm chamfer at the mouth) and two side sockets holding 70 mm stubs of 20 x 20 mm rail (along +x and -y), all
  one fixed body. The sockets are wider than a real cube's (1 mm a side instead of a few tenths) so the fit is a matter of
  alignment, not of a press. A real cube is locked by a set screw, not modelled.
- **Rack** (`post_rack.usda`, fixed): a 50 x 50 x 30 mm block with a 22 x 22 mm, 18 mm deep pocket, 3 mm chamfer.
- **Post** (`post.usda`, 43 g): a solid 20 x 20 x 100 mm bar, the four slots and the end painted as dark strips (no hollow
  core).
- **Physics:** colliders are boxes and small convex prisms; aluminium friction 0.35/0.3 (static/dynamic), plastic 0.45/0.35;
  restitution 0. No joints.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active. The instruction is `task.json`'s, word for
word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=extrusion_corner_cube.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cube-dream

# Scripted demo (left arm), 420 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=extrusion_corner_cube.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 420 --store-frames --log-dir ~/yam-eval/cube-demo
```

The job log prints `[extrusion_corner_cube] t=...  steps done: ...` when a step is done or undone, and a trace every second:
the post's lower end's offset from the socket's axis, its height against the floor, its tilt and whether it is held.

**The demo** (Isaac L40 times: post lifted 6.7 s, in the socket 11.9 s, seated 13.0 s, let go 16.0 s, held still to 17.0 s). Left arm, open loop: comes down on the post from above, fingers closing across
its 20 mm width (along y), pads on its upper 35 mm; grips and lifts it out of the rack, carries it over the cube with its lower
end 8 mm over the cube's top, lowers it 23 mm into the socket until it sits on the floor, lets go, lifts off and goes home.

## Instruction

It says what to do (stand the post in the open socket, fully down on its floor) and to let go, and leaves out which hand,
where things are and how to move.

## Scoring

Four steps, **0.2 each**; step 1 once reached, steps 2 to 4 as things stand, each only with the one before.

| Step | Done when |
|---|---|
| 1. Post lifted out of the rack | a gripper has held it (pads through the post's box, both fingers pressing) at least 2 cm above where it stood |
| 2. Post inside the socket | with step 1, its lower end at least 8 mm below the cube's top (inside the straight walls) and within 2 mm of the socket's axis, the post within 8 degrees of vertical |
| 3. Post seated on the floor | with step 2, its lower end within 1.5 mm of the socket's floor, centred within 1.5 mm, within 3 degrees of vertical |
| 4. Post let go | with step 3, no gripper holds it or has its pads within 3 cm of it |

- **1.0 (success):** every full-success condition held together, without a break, for 1 s: all four steps, no violation, and
  the post within 2 mm and 2 degrees of where it was when that second began. The second is timed inside the task's success() and any lapse restarts it. The step points stop at 0.8, also while the second is being timed. The
  tests check no 1.0 at 0.9 s, 1.0 at 1 s, a lapse restarting it, and no 1.0 while any one condition is missing. The arm's
  return is not scored.
- **0** if the post falls off the table or over (more than 45 degrees), the cube is moved more than 5 mm, or the state is not
  finite.
- **Guards:** a post put in the socket without a gripper lifting it from the rack scores nothing; a post resting on the rim,
  off-axis or leaning is not in the socket; a post held in the socket but not let go scores 0.6 at most.
- **Real station:** the post is checked by a light push on its top (it stays in the socket) and a plumb check.

## Reset

Lift the post out of the socket and stand it in the rack's pocket.

## Status

Dream-1 score: 0.0. Demo: 1.0 (success at step 340, Isaac). Media M-LS7BDV.
