# Changelog

## 0.4.0

The forearm's roll, physically. ToangTown's arm clips (2026-10-10) showed what 0.3.1's twist rule could not: it
measured the hand against the idle grip, so a wrong idle grip passed, and `move_gun`, `reach` and `fix_clip` moved a
hand's whole roll into the forearm bone, which wrings the skin at the elbow on a rig with twist bones.

* `anatomy` reports `forearm_rotation_deg`: positive for supination, negative for pronation, from thumb up, measured
  against the elbow's hinge plane. Past 80 either way (AAOS) is a rule (`pronation_deg`, `supination_deg`). Turning
  the forearm and hand 20 degrees moves the reading 20 (`examples/anatomy_faults.py`).
* On a rig with forearm twist bones it reports `forearm_roll_deg` and `forearm_links_deg`: the roll of each link (the
  elbow's end, each twist bone, the wrist) against the skin's bind. A link past 50 degrees is a rule
  (`forearm_link_max_deg`, a working value). Rigs without twist bones keep the 0.3.1 wrist twist rule.
* `move_gun`, `reach` and `fix_clip` spread a placed hand's roll along the twist bones (a tenth at the elbow's end,
  `forearm_elbow_share`) instead of putting it all in the forearm bone. The hand and the elbow do not move.
* A rifle's own FBX animation (a trigger, a magazine) no longer plays, and its bones start at rest: it followed
  whatever scene frame a clip's sampling left. The rifle's parts move only by `move_part`.
* Inside or outside the rifle is decided by seven rays in uneven directions, the majority deciding (it was three). On
  a mesh that is not watertight (a trigger, a guard), one grazing ray flipped the answer for a point moved 0.0001 cm:
  the same finger read 1.46 cm inside or 0.04 cm off from one frame to the next.

Changes you will see: the sample's left idle hand reads `forearm supinated 133`. Its idle grip was already outside the
wrist rule.

## 0.3.1

A real game rig tested the checks: ToangTown's M24 magazine hold, 8 poses with 7 known faults
(`benchmarks/runs/2026-10-09-toangtown-m24`). 0.3.0 caught 5 and read clipping at about 40% of its depth. 0.3.1
catches all 7 with the depths exact.

Fixes:

* `clearance` and `grip` measured to the nearest corner point of the rifle's mesh. A hand 1.68 cm inside a low-poly
  magazine read 0.68 cm: a flat face has no vertex inside it. They now measure to the surface (the closest point on
  its faces), inside or out by ray parity. Each takes about 0.01 s on the sample.
* `anatomy` hid a finger kinked sideways at its middle joint: it measured against a plane fitted through the
  fingertip, which follows the kink. The middle joint is now measured against its own hinge, the end joint against the
  middle joint's bending plane.
* `anatomy` had no check of the wrist's twist. A forearm turned 154 degrees under a still hand passed. It now reports
  `wrist_twist_deg` (the hand's twist about the forearm's line, beyond the idle grip's) and fails it past 30.
  `move_gun`, `reach` and `fix_clip` move a placed hand's twist into the forearm, so their poses read none.
* `grip` called a hand holding with one fingertip on the rifle and the palm 3 to 5 cm off. It now reports `touching`
  and `holding` apart, with `palm_gap_cm`: holding needs the palm's surface within `palm_within_cm` (1.5). With
  `hold` true, a hand that does not hold breaks a rule.
* A `rigs.json` or `.pose.json` saved with a UTF-8 byte order mark (Windows PowerShell's default) failed to load.
* `anatomy` failed a wrist reported at its 20 degree limit (20.3 before rounding). Every rule now reads the value as
  reported: angles to the degree, the elbow's height to 0.1 cm.

Changes you will see:

* The sample's idle grip now shows its real contacts: the left thumb 2.7 cm inside the handguard, the right palm
  1.2 cm into the pistol grip, the trigger finger's back 0.8 cm inside the guard. `grip` fails it. The sample is
  unchanged.
* The benchmark's `fingertip_contact` mark moved to [2.6, 33.0, -1.5]. At the old mark the other fingertips sat
  1.5 cm inside the handguard, so the task had no clean solution. Oracle 5 of 5, control 0 of 5.
* `examples/motion_test.py` now finds 0.93 cm of clearance in the faulty clip (0.0 before) and mends it.
* New: `examples/anatomy_faults.py` (`blender -b -P`) puts each fault on the sample and fails if a check misses it.

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
