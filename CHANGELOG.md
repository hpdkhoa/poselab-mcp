# Changelog

## 0.3.0

New:

* `anatomy`: each arm against the human arm's limits. The elbow stays at least 2 cm below the shoulder. The elbow is
  a hinge bent 5 to 150 degrees. The wrist stays within 30 degrees of the forearm's line and inside its joint range
  (flexion 80, extension 70, radial 20, ulnar 30). Each finger joint curls only toward the palm within its range
  (knuckle -30 to 100, middle joint -5 to 110, end joint -10 to 90) and stays in the finger's own plane. The reply
  lists every rule a pose breaks. A rig's `limits` and `finger_limits` override the values.
* `arm_ranges`: the research behind the limits. For each joint of the arm it gives the AAOS normal range, the range
  daily tasks use, the value `anatomy` checks, and why the two differ where they do. It cites AAOS, Morrey 1981,
  Palmer 1985, Ryu 1991, Hume 1990, Bain 2015 and Soucie 2011. `docs/ANATOMY.md` has the same tables. Standard
  values come from the AAOS 1965 table as reprinted in Greene and Heckman 1994; the docs show where Eaton's table
  differs (10 to 25 degrees for the fingers and thumb). The 30 degree wrist rule and the knuckle's 40 degree side bend
  are marked as working values with no source. The docs say why the finger defaults stay looser than AAOS.
* The thumb in `anatomy`: its knuckle (MCP) bends -10 to 60 and its end joint (IP) -15 to 90 across the palm,
  each close to its hinge's plane; its base joint (CMC) spreads at most 80 degrees from the index metacarpal and sits
  at most 20 behind the palm. The hinge ranges take Eaton's hyperextension and the AAOS flexion plus the fingers' 10
  degrees of slack; the base joint's two limits are working values. `fix_clip` bends the knuckle and end joint back
  into range; the base joint is reported, not mended. A rig's `thumb_limits` overrides the values.
* `grip`: how each hand touches the rifle. A hand on the rifle must face it with the palm, not the back of the hand.
  The rifle must stay out of the back of the hand and of each finger (0.1 cm), and the palm side may press in up to
  1 cm. `clearance` with palms and fingers left out, the usual way to allow a grip's contact, never showed a hand on
  the wrong side. On the sample rig a hand turned 180 degrees about its forearm fails it; the idle grip passes.
* `solve` takes `anatomy` and `grip` goals, and `scan_clip` `anatomy` and `grip` checks. Each value is the count of
  broken rules.
* `fix_clip` mends `anatomy`: it swings the elbow under the shoulder and inside its bend, eased over the neighbouring
  frames. Then it turns the wrist back inside its limits and each finger joint back inside its range. It never turns
  a wrist so far that the rifle goes into the back of the hand.

Fixes:

* `fix_clip` with a `clearance` check that needed an elbow swing failed with `NameError: smooth` (0.2.0 to 0.2.2).
  The easing helper was missing.
* Found while building `grip`, before release: the palm's direction pointed out of the back of the hand, so
  `anatomy` read wrist flexion as extension and checked it against the wrong limit. The wrist's bend is also now split
  on two perpendicular axes; the old split read the sample's right wrist as 34 degrees ulnar when it is 57.

Notes:

* The sample rig's idle grip breaks the wrist rule (44 degrees left, 58 right). The rig is unchanged, so the
  benchmark numbers still hold. `fix_clip` with an `anatomy` check brings the left wrist to 29 degrees. It leaves the
  right at 58: every turn that mends it puts the rifle inside the back of the palm.
* `examples/edge_cases.py` covers the new tools, goals and checks. `examples/selftest.py` calls `anatomy` and `grip`.

## 0.2.2

Fixes:

* Bad input now gets a clear message, never a raw Python error. Before, these failed with `IndexError`, `KeyError`
  or `TypeError`:
  * a point, pivot or pole that is not three numbers (`move_gun`, `reach`, `where`, `distance`);
  * `faces_eye` on an `[x, y, z]` point with no normal;
  * `solve` with a bad `[min, max]` range, `samples` of 0, or a `maximize` index past the last goal;
  * a clip check without what its type needs (`hold` without a side, `contact` without `a` and `b`);
  * `render` with no views.
* Inputs that gave a wrong answer without a word are now refused: an unknown name in `solve`'s dofs (it was dropped),
  a zero `faces_eye` normal, a `visible` radius of 0 or less, `record_clip` with fps of 0 or less or a key before
  0 s, a `during` that is not `[from_s, to_s]`, and a `pop` check with an empty bone list.
* `pose_clip` at a time outside the clip shows the first or last frame and says so. Before, a loaded FBX clip read
  frames outside its range, and the reply gave the time asked for, not the one shown.
* The `record_clip` description no longer names `scan_clip` and `fix_clip`. A client that offers `record_clip`
  without them was pointed at tools it does not have.
* `scan_clip` and `fix_clip`: a `pop` check takes `"gun"` for the gun bone, as every frame description calls it. An
  unknown bone name now gets an error that lists the rig's bones, not a raw `KeyError`.
* An unknown point name gets an error that lists the rig's points and the other kinds of point.
* `examples/edge_cases.py` calls every tool with bad input and fails on any raw Python error. PUBLISHING.md lists it
  with the checks to run before a release.

Benchmark:

* `agent_bridge.py` runs the tasks with coding agents, with no API key.
* Results (benchmarks/README.md): with Claude Code agents, 3 trials per task, measured agents passed 15 of 15 and
  vision agents 11 of 15. The run data is in `benchmarks/runs/2026-10-08-claude-code/`.
* The `turn_only` prompt names the `stock` pivot and allows any roll. An agent showed that a turn about the `bore`
  pivot does show the port, so the old prompt had no single right answer.
* The clip grader checks that the clip exists before it scans, so a missing clip no longer logs a server error.

## 0.2.1

* Tool errors now reach the client with their message. Before, a missing Blender showed only "Error executing tool
  load_rig". Now the client reads "Blender not found: set POSELAB_BLENDER ...".
* The benchmark checks for Blender and for Anthropic credentials before it starts. The credential check is a free
  token count, so a run with no key stops at once and records nothing.

## 0.2.0

Motion: Pose Lab now checks and mends whole clips, not only poses.

* `record_clip`: a clip from poses keyed at times, eased from key to key.
* `load_clip`: a clip from a `.pose.json`, an FBX or a BVH file, with a bone name map.
* `scan_clip`: every frame against checks, with the worst value, when, and the time spans that fail. New checks:
  `contact`, `hold` (a hand's drift on the rifle) and `pop` (sudden jumps).
* `fix_clip`: mends pops, holds, contacts and clearance, and reports the scan before and after. It also lists the
  frames where a hand must be somewhere its arm cannot reach.
* `save_clip`: writes `.pose.json` and FBX into the output folder only.

Fixes:

* Bone scale no longer drifts during long solves. Rounding in the arm IK built up over hundreds of moves and
  stretched the parts on the bones (seen in 0.1.0 renders after `solve`). The drift also changed measurements: the
  0.1.0 README said a move met every goal on my game's AK rig. With the fix, no sample of 400 meets all four.
* `solve` reports how many samples met every goal together (`all_met_in_samples`).
* The sample rig's arms are now adult length (30 cm and 28 cm). The left arm was fully stretched at idle, so any move
  of the rifle pulled its hand off the handguard.

## 0.1.0

First release: 17 tools for measured poses, the solver, renders, and the built-in sample rig.
