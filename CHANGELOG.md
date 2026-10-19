# Changelog

## Unreleased

A hand holds any object by measured contact (2026-10-18 and 19). The goals and every decision behind this work are in
docs/HOLD_DECISIONS.md.

* **The object reader** (`holdlab.py`): `read_object` finds a part's runs a hand can wrap (an axis, sections and their
  girth, the inner end) from 2 mm surface samples, splitting each section into its pieces (a grip and the guard bar in
  front of it); the openings through the object (a trigger guard, a handle's eye), with the edge an extended finger lies
  along; the web point under an overhang (a beavertail, a hammer's head). `suggest_marks` proposes hold and no-touch
  marks for a person to confirm. Each part is read in its own bone's frame, so a rigged part that moves is read where
  it is. `examples/hold_reader_tests.py`: known answers on a cylinder, a pistol-shaped block and a box.
* **The contact model, the core** (`contactlab.py`): the object's surface labelled by region (back, front, left,
  right, tang, opening edge, no-touch parts) from its samples, not its polygons; the hand's skin labelled by part (palm,
  web, each digit segment's pad, side and back). `contact_map` and `contact_pairs` give every pair's contact area,
  press area and depth and facing, hand on hand included; `contact_pictures` draws the object's regions, the hand's
  parts, the collision heat (see-through) and the contact map.
* **Purposes on contact pairs:** power wrap (palm and finger pads on their regions, web, cover over the hand's span),
  thumb in line, thumb wrap (a round handle), index extended along an opening's edge, support (palm on the open side,
  fingers over the other hand), no-touch, press past soft tissue's give, comfort. `hold_gate` passes a hand only when
  it does its job; the search scores with the same purposes (one ruler).
* **The search** (`hold_object`): the whole arm (hand place and turn, elbow swing, the shoulder girdle via `shoulder`,
  clavicle forward and up within Neumann's ranges), the thumb's base; the palm settled onto the object by signed skin
  distance; `conform_digits` closes the fingers on the drawn skin in numpy, the knuckle leading, a touching segment
  stopping the joints behind it; a pull toward each unmet contact; starts by the hand's own frame (`seed_hold`), ranked
  by how far past its limits the arm is (`arm_excess`). A try takes 0.36 to 0.44 s (it took 0.7 to 2.7 s).
* **Soft tissue give:** a press counts only past the palm's 3 mm, a finger or thumb pad's 1.5 mm, 0.5 mm elsewhere.
* `examples/hold_bench.py`: the hold benchmark; a 3.5 cm handle leaned 20 degrees passes the gate (palm 9.3 cm2,
  finger pads 8.3, web 0.8, thumb 1.7, cover 41 percent, press 0.34 cm2).
* **Hard rules** (never traded by the search, each fails the gate): no clipping, tested from both sides (skin inside the
  object past its tissue's give, and the object's surface inside the flesh past it, so a thin part through a finger
  between its vertices is caught; `inside_flesh`), no hand inside the other past the skin's give; a holding finger's
  joints (`hold_digit_rule`, HOLD_DIGIT_LIMITS: no backward bend, no twist past a grasp's). `examples/clip_tests.py`:
  known answers (a thin ring through a coarse finger).
* **The target solver** (`target_hold`, no chance in it): the purposes name targets, the hand is fitted onto them by
  weighted least squares, the body eases past a limit, the fingers close by the skin and are laid down by a fixed step
  pattern, the thumb steps out of the object, and a fixed-step polish tunes the whole hold on the gate's own score.
  It stops at the first round that passes, keeps the best, and reports each unmet purpose with its numbers. The
  cylinder passes with a score of 0, the same on every run.
* **The walls** (`wall_faults`, `_walled`, `_walls_hold`): a real body's limits and physics are never scored or
  traded. Every step of the target solver, its fits, its polish and the per-frame repair is undone when it breaks a
  wall or goes further past one; holds are ranked walls first; a hold past a wall is reported as "no hold inside the
  walls" and never handed out. `hold_object` now runs the target solver; the random search and the random per-frame
  repair are removed. Not yet run.
* **Distance grids** (`sdflab.py`): each part's signed distance on a 0.2 cm grid in its own frame, built once and kept
  on disk; `sdf_query` gives distance, nearest part and outward normal for many points at once (20000 in 0.02 s).
  The contact read, the palm settle, the closing and the relax use it. `examples/sdf_tests.py`: known answers and the
  measured error near the surface (99 of 100 points under 0.022 cm); the press rule ignores sinking under
  SDF_TOL_CM (0.03 cm) past the give.
* Snapshots keep the shoulder girdle's angles with the bones.
* `close_hand(mend=False)` closes without turning the wrist (the old mend rotated a settled hand into the object).
* `render(centre=...)` aims the outside views at a world point.
* Removed: the coverage-ring and capsule versions of the power wrap, support and no-touch purposes (one ruler left).

The hand work trimmed to one score (2026-10-18). Five searches each scored a hold their own way and undid one
another's work; a known good hold adapted in small moves replaces searching from nothing.

* `hold_score`: the one score every hold search and every repair uses (contact rules, presses into the gun and the
  other hand, the arm's ease and rules, the digits' rules, reach).
* `adapt_hold`: a reference hold already on the hand (a pack's hand, a dataset's grasp) adapted to this gun, started
  from the hand's own bends, kept within a span and a turn of its start; how far it went says whether the reference
  fits. `es_hold` gains the elbow's swing as a dimension (the whole arm moves), `from_pose`, `span_cm`, `turn_deg`,
  `keep_w` and `hand_only`.
* `put_reference`: a reference hold put on the hand: digit bends, a rigid fit to the wrist, knuckles and thumb base,
  the thumb's tip aimed.
* `repair_hand`: one frame mended by the same score (the hand and the elbow move, the digits stay). It does the work
  of Gun Kit's four repair passes.
* `skin_presses`: the drawn skin against the gun (signed, ray parity) and the other hand, vertex by vertex; the
  capsules test a slightly thinner body than the picture.
* `view_report`: what the eye sees of a hold: the sights' visible share, the arms' cover, each hand shown.
* `examples/geometry_tests.py`: known answers for the capsule, segment and signed-distance measures (13 checks).
* Removed: `hold_chain` and `thumb_search` (no callers left once the trim landed).

Every hand in an action has a target, from a review by eye of the repair pilot (2026-10-15): a repaired AK charge
pose broke no rule, but its right hand pulled nothing and three of its fingers curled in the air.

* `purpose`: each hand gets a target (a rig point or [x, y, z], an act, and the part's size around the point). hold:
  a pinch (grip's grasp rule, kept to the part) or a wrap (the palm on the part and two fingers hooked on it). pull: a
  pinch or a finger hooked on the part. press: a digit on it. rest: the hand on it. reach: no contact yet. A hand with
  no target breaks a rule, and so does a finger curled more than 120 degrees over its three joints with its pad more
  than 2 cm from the rifle, in every act. The grasp rule (`_grasp`) can now keep to the rifle's surface near a point.
* A rig's `point_parts` ties a rig point to a moving part: a charging knob rides with the carrier.

## 0.7.0

A posing tool both an agent with Pose Lab's checks and an agent with renders alone can use, for the repair pilot in
the paper "Measure, don't judge" (2026-10-14).

* `turn_bone`: turns one bone by some degrees about an axis through its head, in a named frame; the bones below it
  follow. It bends a finger joint, turns a wrist or turns an arm at the shoulder. It refuses the rifle's bone (use
  `move_gun`). The reply gives the bone, the axis, the degrees and the frame, and no positions: an earlier version
  returned the bone's head, and an agent with renders alone read joint positions from turns of zero degrees.
* PoseFault repair (`benchmarks/posefault/repair_tasks.py`): `agent_bridge.py` serves `pf:` tasks, one PoseFault pose
  each, under two conditions (vision, measured), and grades the final pose by `measure.py`'s rules. A task may name
  its rig. In the pilot (30 poses from three ToangTown bases), agents with the checks repaired 19 of 27 faulty poses
  and agents with renders alone 1 of 27; both left the 3 clean poses alone.

## 0.6.0

Whether a hand really holds a part, from the paper "Measure, don't judge" (2026-10-14): a review by eye rejected
ToangTown's M24 magazine hold because the thumb does not wrap the magazine, and no rule saw it.

* `grip` takes `grasp`: a hand meant to hold a part must pinch it between the thumb and a finger (an antipodal grasp).
  A digit touches when its skin is within `contact_cm` (0.3) of the rifle; the thumb and a touching finger grasp
  when their directions from the rifle's surface are at least 180 - 2 atan(`friction`) degrees apart (127 at the
  default 0.5, skin on metal or polymer). Every reply now gives each hand's `grasping`, `grasp_opposition_deg` (the
  widest thumb to finger angle) and `digits_touching`. On the M24 hold the thumb and three fingers touch the
  magazine 52 degrees apart: not a grasp.
* PoseFault: bases whose hand is meant to hold a part get the grasp rule and two faults, `thumb_off` and
  `fingers_open` (30, 45, 60 degrees); a new base, ToangTown's AK empty reload at 2.4 s, right hand on the grip.
  `generate.py --bases` adds a base to a set without making the others again.

## 0.5.0

Tools for fixing a hold for good, from ToangTown's weapon audit (2026-10-13): on its AK-47 the left palm was on the
gun, but the forearm and sleeve filled the lower left of the view; on 13 guns that borrow another gun's arms the left
hand missed the handguard.

* `view_cover`: the share of the player's view the arms cover (all, the lower left third, the bottom middle third),
  the arms rendered alone from the eye as a mask, back faces culled as a game culls them. ToangTown's game probe
  measures the same in the game (within about 0.06).
* A rig whose idle FBX lacks bones the mesh is weighted to (an export that leaves out Manny's corrective bones, such
  as upperarm_out_l or clavicle_scap_l) no longer leaves those vertices at rest: each orphaned group's weights go to
  its nearest ancestor the idle armature has. `describe` lists them as `orphan_groups_adopted`. Before this, the
  AKS-74U arms kept a shoulder inside the eye, and `view_cover` read the AK-47's lower left as 1.0 against 0.57 in
  the game.
* `retarget_clip` takes `gun_move` (move_gun's arguments, applied to every frame first, the other hand keeping its
  hold) and `pole` (the elbow's direction, as `solve_grip`'s best gives it). Without the pole, every frame of a
  retargeted clip lost the solved elbow and broke the forearm's supination limit.
* `move_gun` keeps each kept hand's elbow pointing the way it pointed. The arm IK's default pole bent a run clip's
  right wrist past its limit in every frame.
* A finger whose back is inside the rifle after a hand moves opens about its own hinge until it is clear
  (`solve_grip` and `retarget_clip`). `solve_grip`'s rows list `grip_bad`, the grip rules a candidate broke.
* `hand_shown`: how much of a hand the player sees (its own vertices in the view and first along their line of
  sight); `view_cover` reports `hand_l_shown`, and `solve_grip`'s `view_budget` takes `hand_l_shown_min`, so a hold
  that clears the view cannot do it by losing the support hand. Within about 0.04 of ToangTown's game probe.
* `shift_hand`: a hand moved frame by frame by given offsets in the gun frame (a hand that holds a part whose track
  changed follows its new path), its turn, elbow direction and finger curl kept, then mended.
* `solve_grip`: searches a hand's hold. The palm goes on a mark, moved along the barrel; the elbow points toward each
  of several poles; the hand rolls about the barrel (the forearm pronates or supinates with it). Each candidate settles
  the palm onto the rifle's surface, then counts only if the palm holds the rifle (`grip`), nothing else clips
  (declared contacts left out; the palm's own press is `grip`'s to judge), every arm and finger rule holds, and the view
  stays within a budget. On the built-in sample it finds a hold the idle grip misses (the idle forearm is supinated
  126 degrees, past 80): 2 cm forward, rolled 30 degrees, the elbow lower.
* `retarget_clip`: a clip from another rig played with one hand held on this gun's own hold by arm IK, fingers mended;
  a span where the hand leaves the gun (a reload) is kept as it is.
* `check_set`: every check on a clip in one summary: the worst clearance and when, the frames that break an arm or
  finger rule, the frames a hand is not holding.
* `examples/tools_050.py` checks the four on the sample.

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
