# The hold tool: goals and decisions

This file holds the owner's goals for the hand-hold tool and every decision taken on the way, with its date and its
reason. Read it before changing the hold code, and check each change against it. Last updated 2026-10-22.

## The goal

1. **Hold any object.** The tool makes a hand hold an object the way a person would. A pistol or a rifle is one
   object among many: a hammer, a mug, a bat, a box. Nothing in the hold code may be written for one gun or one
   kind of gun.
2. **Fully automatic.** Nobody poses a hand by hand ("I want this project to be automatic workflow, not me
   posing for 30 minutes"). No per-object numbers typed by a person or by Claude.
3. **The computer sees the object through numbers.** The hold is judged by measures on the object's geometry and the
   drawn skin, never by a picture and never by copying a hand from a pack. Pictures are for people to check the
   numbers.
4. **For an FPS game.** The result is hand clips that look right from the player's eye and pass the game's Gate.

## The owner's rules (the hold must obey them)

- **Real anatomy first.** Human bone and body sizes (ANSUR II), joint ranges (AAOS, Neumann), no irregular sizes. A
  hold may miss a touch; it may never bend a body past its range.
- **The whole arm moves.** When a finger is at its limit, the wrist moves, and the shoulder and arm count in the
  range. A support arm brings the shoulder forward (the clavicle tool).
- **No part kept fixed** because it once worked. Every hand is re-solved by measurement.
- **Purpose, not pose.** Each hand has a job: hold the grip, support it for stability, hold any object, touch here,
  never touch there. Rules are written as purposes against the object's measured features, not as coordinates.
- **The firing hand on a pistol:** high on the grip, the web under the tang (beavertail); palm and fingers covering
  as much of the grip as the hand can; the thumb in line with the index finger's direction, its pad pressing a little
  into the grip; the trigger finger straight along the trigger guard's line (not up at the chamber, not on the trigger
  except in the fire frames). No gap on the grip, for recoil control.
- **The support hand** fills the side the firing hand leaves open and holds over the firing fingers; its thumb lies
  forward along the firing thumb.
- **No-touch:** no skin on moving parts (slide, hammer), the trigger outside the fire frames, the magazine release.
- **A hand off its part fails.** A gate that passes a hand that is not doing its job is a wrong gate.
- **Thicker or thinner skins** of the same object must still hold without clipping (the skin check).
- **Never clipping, never twisted fingers** (hard rules, 2026-10-19: "in real life that will not happen"). No skin
  inside the object past its tissue's give, no object inside the flesh past it (tested from both sides, so a thin
  part through a finger is caught), no hand inside the other past the skin's give. A holding finger never bends a
  joint backward (knuckle at most 5 degrees, middle joint 0, end joint 5) or twists out of its plane past a grasp's
  (knuckle 20, middle 8, end 10 degrees). A broken hard rule fails the gate and costs the search as much as a broken
  body rule: it is never traded for a gain elsewhere.

## Decisions

| Date | Decision | Why |
|---|---|---|
| 2026-10-17 | Contact measured by coordinates, not pictures (contact rules, wrap, closure) | The owner: looking at pictures is a limit |
| 2026-10-17 | ANSUR II body size check; slim arms by bone edits applied at load | The arms looked too big; sizes must be human |
| 2026-10-18 | Trim to one score; a reference hold per grip style, adapted | Five searches each scored differently and undid each other |
| 2026-10-18 | The G17 pack (Fab Standard License) may be read; its numbers stay in the game project | Licence: no redistribution on its own |
| 2026-10-18 | **Reversed:** the G17 is a comparison only, never a rule; the tool reads the object itself | The owner: build vision through numbers, not memory of a pack hand |
| 2026-10-18 | Hold any object; marks: suggested from geometry and confirmed by a person, or from a rig's facts (guns) | The owner chose both ways |
| 2026-10-18 | OCR: Tesseract (already installed), OCR Lab as its own MCP (github.com/hpdkhoa/ocrlab-mcp) | The owner asked for OCR as a new MCP |
| 2026-10-18 | Object reader: runs a hand can wrap, openings, the web point under an overhang, from 2 mm surface samples | General: a grip, a handle, a mug's handle |
| 2026-10-18 | Purposes from the GRASP taxonomy (power wrap, support, pinch, press) plus no-touch and comfort | General vocabulary for any object |
| 2026-10-18 | Shoulder girdle tool (clavicle forward, up; Neumann ranges) in the search | The owner: a support arm brings the shoulder forward |
| 2026-10-18 | **The contact model is the core of the tool** (not a benchmark side step): labelled object surface, labelled hand skin, contact pairs, collision as area and depth, the contact map | The owner corrected the framing |
| 2026-10-18 | One ruler: the purposes, the search's score and the gate all read the contact pairs | A search scoring differently from the gate finds holds the gate fails |
| 2026-10-18 | Work rules: a benchmark with known answers, one change per run, keep the best, solver proven on a cylinder before the gun, report only measured results | Fixes kept making the hold worse without a baseline |
| 2026-10-19 | The search: palm settled by signed skin distance; fingers closed by contact without the old wrist mend, then relaxed by the skin; a pull toward unmet contacts; seeds by the hand's frame; seeds ranked by how far past its limits the arm is | Each fault found on the cylinder benchmark |
| 2026-10-19 | Conforming finger closure on the skin (numpy, one read): the knuckle leads, a touching segment stops its own joint and those behind it; replaces the capsule closing and the skin relax | The fingers stopped at their first touch and hovered; a try went from 0.72 to 0.38 s |
| 2026-10-19 | Soft tissue give: a press counts only past the tissue's give (palm 0.3 cm, finger and thumb pads, web and palm edge 0.15, elsewhere 0.05); the palm settle and the closing may sink into it | Rigid meshes do not squash; a real grip's contact area comes from flattened palm and pads. The purposes' area thresholds were not changed |
| 2026-10-19 | A power wrap's cover is the share of the run's surface covered over the stretch the hand spans, not over the whole run | A long handle is not held less for being long (a 30 cm bat handle could never reach 30% over its whole length) |
| 2026-10-19 | A clip frame where a hand rests on its home hold is not repaired: it is the hold, searched and gated | The per-frame repair still scored with the old capsule ruler and moved held hands off the gun (two rulers again). Open: move the repair onto the contact ruler |
| 2026-10-19 | A hand placed first is not charged for the other hand in its rest pose; the index line's up runs from the run's outer end to its inner end | Scoring faults found on the M1911 seed |
| 2026-10-19 | Openings ignore parts much smaller than the held part (a trigger inside its guard); the edge an extended finger lies along is the opening's top band (15 percent of its height up the run), fitted straight | The trigger split the guard hole into slots and the edge came out along the slot behind it (80 degrees to the bore) |
| 2026-10-19 | The gun's stance is searched: a grid of places ahead and up, judged by both hands' best starts (arm excess first, then the purposes) | Both shoulders sat at their limits: the style's guessed stance was far for these arms |
| 2026-10-19 | The per-frame repair moves onto the contact ruler (`repair_frame`: skin presses past the give, the arm's excess, a pull back to the motion) | Closes the open item: no second ruler on moving frames |
| 2026-10-19 | In the score the grip's own purpose (power wrap, support) counts three times its refinements (thumb and index lines); the stance grid reaches 8 cm down | The search traded the palm on the grip (10 to 0.4 cm2) for a better thumb line; the stance pick sat at the grid's edge |
| 2026-10-19 | Hard rules: no clipping either way (skin in the object, the object in the flesh, a hand in the other, past the give) and holding-digit limits (no backward bend, no twist past a grasp's); known answers in examples/clip_tests.py | Run B had a support hand's fingers bent backward and the firing index through the guard's ring; the old checks passed both (vertices only, free-hand ranges) |
| 2026-10-19 | No random search (owner): a target solver to replace it, fallback to be chosen | The search guessed; results swung run to run |
| 2026-10-19 | The fallback when a target is unmet, with no chance in it: 1 the purposes in priority order (hard rules, the grip, its refinements); 2 the body moves in a set order (wrist, elbow, shoulder girdle, stance); 3 a fixed list of starts; 4 an honest report of what is unmet and why; 5 a refinement loosened only with the owner's yes, per purpose | The owner: no random; agreed 2026-10-19 |
| 2026-10-19 | No GPU ray tracing for now (a GTX 1060 3GB has no RT cores; Blender gives OptiX and CUDA only to Cycles); the object as per-part signed distance grids instead (`sdflab.py`, 0.2 cm cells, built once, kept on disk) | A ray cast costs 0.9 us on the CPU; the cost was Python loops, one query per vertex. The grid answers 20000 points with normals in 0.02 s |
| 2026-10-19 | The grid's error near the surface is measured (examples/sdf_tests.py: median 0.002 cm, 99 of 100 points under 0.022, worst 0.056), and the press rule does not count skin sunk less than SDF_TOL_CM (0.03 cm) past its give | A rule cannot ask for more than its ruler can tell: 0.15 cm2 sunk 0.004 cm failed the hard rule |
| 2026-10-19 | **The target solver** (`target_hold`) replaces the random search: each purpose names where its hand part goes (the palm's half by the knuckles onto the run, its facing into the surface, the web onto its point or the nearest surface, the knuckle line along the run in the start's way); the hand fitted onto them by weighted least squares (Kabsch), a fit the arm cannot take refused; the palm settled; the body eased past a limit (elbow, then girdle); the fingers closed by the skin, then laid down by a fixed step pattern (base closing while the end opens); the thumb's base and bends stepped out of the object; last a fixed-step polish of the whole hold on the gate's own score (the hand around and along the run, moved and turned, the elbow, the girdle, each joint) | The owner: the search must look toward the holding part, not guess. The cylinder passes with a score of 0, the same answer on every run |
| 2026-10-19 | A power wrap seats the handle across the distal palm (the palm's half by the knuckles), not the palm's middle | Seated at the middle only the fingertips reached (pads 3.4 of 4 cm2); a real power grip lies by the knuckle crease |
| 2026-10-19 | **The walls** (owner): a real body's limits and physics are walls, never scored, never weighed, never traded. `wall_faults` reads every wall a hand breaks (the arm's rules, the fingers' and thumb's ranges, a holding digit's limits, clipping both ways and hand into hand). Every step of every fit, the polish and the per-frame repair goes through it: a step that breaks a wall, or goes further past one, is undone; a step that leaves the walls less broken is kept first. A round must end inside its start's walls or it is undone. Holds are compared in a fixed order (walls, then the gate, then the score). A hold past a wall is reported "no hold inside the walls" and never counts as found; Gun Kit checks every hold whatever found it, and a clip with any frame past a wall is not written, only reported. The old random search and the random per-frame repair are gone (they priced the rules) | The target solver priced a broken thumb rule at 50 against clipping at 830 and swung the thumb base to 158 degrees (limit 80); it let the hand leave the gun to escape a clip. The owner: "the rule and the body anatomy is set to keep a real human body not the thing you can score and break, clipping is against physics" |
| 2026-10-19 | When a result breaks a wall: stop and tell the owner; no long runs tuning through it; no default changed to a solver that has not beaten the old one on the real case without the owner's yes | Four 20 to 30 minute gun runs were tuned through broken rules before asking |
| 2026-10-22 | **The skin as real tissue (Option A)**: a finger segment's pad gives by a sourced depth at the hand's grip force (finger_give.json: Wu 2014 pressure per segment, Perez-Gonzalez 2013 probe travel, broad contact by Dzidek 2017's ratio 0.376, below 1 N Dzidek's area exponent 0.37); the thumb, palm, web and finger sides and backs keep skin_give.json | The owner: "I choose Option A"; "1 use b, 2 use Dzidek, 3 do the fingers and leave the rest" |
| 2026-10-22 | Grip force: 5 N at rest, up to 40 N firing, 20 N if 40 N clips (contactlab.set_grip; Gun Kit grip_rest_N, grip_fire_N, grip_fire_fallback_N) | The owner: "when not firing use 5N, fight the recoil can go to 40N, but when test if clipping too much then change to 20N" |
| 2026-10-22 | The fingers close until each skin point has sunk its own give (chain_close give=; conform_digits tissue=True for the four fingers); past the give by more than the ruler's tolerance (SDF_TOL_CM) is pressed. The old fixed press of 0.12 cm and target 0 stay for the thumb | Option A: "the hand is allowed into the gun by the measured give, then the skin is reshaped" |
| 2026-10-22 | Gun Kit: a fire grip per passed home hold (the wrapping fingers closed at 40 N, else 20 N, every wall read; its press morph press_<side>_fire); fire clips (the style's trigger clips) use it on home frames; both hands' press morphs keyed; the morph FBX written after all presses exist | The owner's grip rule; the support hand's press was never keyed |
| 2026-10-22 | A gun's grip read along its magazine well's line from the facts (`run_along`), not searched | The owner: "use 1". On the rebuilt M1911 a 0.64 cm step at the grip's back broke the searched run; a run tilted 8 degrees sideways won |
| 2026-10-22 | The web point is the crook on the grip's middle line (reach read in a 0.3 cm strip there) | The owner's rule names it an exact point; the farthest sample across a 2.4 cm band landed 0.79 cm left on the rounded tang |
| 2026-10-22 | Line B: straight side rays from the firing side against the frame's own polygons (no trigger), at the ring's top edge, moved down the face until the line's middle is at the ring's centre height | The owner: "B line not in the middle of trigger ring". It was 0.62 cm high, its ends inside the gun (the palm's 37 degree face direction slipped into the opening; the trigger was hit) |
| 2026-10-22 | **Touch points first** (`check_touch_points`, first in `direct_hold`): each touch point carries its definition; a miss stops the hold before any hand is placed | The owner: "keep the rule from point touch in global memory ... and implement it into the code with argument is the touch points" |
| 2026-10-22 | Line B runs parallel to the bore, on the frame's right face, its middle at the trigger ring's middle height (Gun Kit line_b_dir "bore", the default) | The owner: "Line B's direction: parallel to the bore". Along the ring's sloped top edge (25 degrees on the rebuilt guard) no hand reaches it: 5 cm off |
| 2026-10-22 | Step 2 one by one (direct_hold only="web_index"): 2a the web on A (a move), 2b the index pad line along line B, the roll set so the pad faces the frame (the index on the gun's right side) | The owner: "render web and index finger right first, follow the rule one by one"; "index finger is on the right side of the gun" |
| 2026-10-22 | Step 2 is one calculation in place_on_touch (no extra steps): the web fold on A; the straight index pad line turned parallel to line B (tip forward) about A; then turned about line B's direction through A as near line B as the hand allows (outside, not pressing). The square pad-facing, the web slide along the crook and the index curl were tried and removed | The owner: "not a new step, the B line must be calculated to place the hand directly in the A point and index finger along the B line"; "the index can be straight in B line position without the need of touch B point, just keep it close is enough for index finger no need to press index finger". The rigid open hand has the web 1.75 cm from its index pad line; the crook is 1.16 cm from line B |
| 2026-10-19 | All decisions and goals kept in this file, to read and revalidate later | The owner asked |
| 2026-10-19 | If the search is slow, make it faster so more retries fit | The owner asked |

## How to revalidate

- Reader known answers: `blender -b --factory-startup -P examples/hold_reader_tests.py` (a cylinder, a pistol block,
  a box).
- Geometry known answers: `blender -b --factory-startup -P examples/geometry_tests.py`.
- Distance grid known answers and its measured error: `blender -b --factory-startup -P examples/sdf_tests.py`.
- Clipping known answers: `blender -b --factory-startup -P examples/clip_tests.py`.
- The hold benchmark (the arms are yours): `blender -b --factory-startup -P examples/hold_bench.py -- --arms-folder
  <folder> --arms Arms.fbx --idle Arms_Idle.fbx --eye 0 -2.3 162.6 --bone-edits bone_edits.json --out <folder>`.
  It writes bench_cylinder.json (every number) and the pictures (hand, heat, contact).
- Gun Kit on a gun: the hands step with the grip style's purposes; `contact_view` draws the contact map of a saved
  clip frame.

## State (2026-10-19)

- Built: OCR Lab; the object reader; mark suggestions; purposes; the shoulder tool; the contact model and its
  pictures; the gate on contact pairs; the whole-arm search with the fixes above; the cylinder benchmark.
- The cylinder (3.5 cm, leaned 20 degrees) PASSES the gate (2026-10-19): palm 9.3 cm2, finger pads 8.3, web 0.83,
  thumb 1.67, cover 41% over the hand's span, press 0.34 cm2 at 0.06 cm, wrist 14, forearm 33; the third of three
  starts passed; a try 0.36 s.
- The target solver on the cylinder (2026-10-19): PASSES with a score of 0, the same numbers on two runs. Palm 10.95
  cm2, finger pads 4.19, web 0.40, thumb 3.75, cover 31.6%, nothing pressed or pierced, no digit fault, wrist 21
  (comfort 22), forearm 34, elbow 52, girdle forward 10 and down 6. The third of three fixed starts passed (the first
  two: palm short, or a pinky tip twisted 16). 36 s for the passing start, 116 s for all three; the contact read
  0.08 s.
- Later the same day: the fit goes only as far as the arm takes it (the whole move, else 3/4, 1/2, 1/4, the body
  easing first), the palm faces the run's axis from a target on the run's wall, and the thumb and finger fits count
  the object inside the flesh too. The cylinder still passes; a solve takes 138 s now (36 s before the two-way test
  in the fits: a full contact read per step, to speed up).
- The M1911 idle on the target solver (2026-10-19, four runs of 21 to 32 min): it fails. Best support hand (run 3):
  palm 7 cm2 on the panel, 4.4 cm2 over the firing fingers, nothing pressed; but its index end joint twisted 85.
  Firing hand (run 3): thumb in line (8.6 deg), index 8 deg off the guard line, but the frame inside the thumb's base
  (32 cm2, 1.3 cm). Run 4: the thumb base spreads 158 deg from the index (limit 80): the thumb fit's base turns are
  not held to the thumb's range. Open: hold the thumb fit to the base's range; the support index twist; the firing
  palm (0 to 0.2 cm2) never seats on the backstrap; the gun runs are slow (most of it the stance grid and the starts).
- The walls are built (2026-10-19) and not yet run: the owner asked for no test run until the speed plan is chosen.
- The contact areas the purposes ask for are first estimates; set them from holds the owner accepts.
- The M1911 idle on the new search (2026-10-19): a two-hand grip that looks right (firing hand high, fingers round
  the front strap, index along the frame over the guard; support hand over the firing fingers, both thumbs forward
  on the support side). The gate still fails on details. Firing hand: web 0/0.3 cm2, cover 22%/30%, index 34 deg
  off the guard line (15), thumb 19 deg off the index (15), thumb pad 0/0.8. Support: palm 3.5/4 cm2, panel cover
  15%/20%, thumb 30 deg off. Both shoulders at their limits (support back 20, both down 8): the stance may be far.
  A try 0.38 to 0.44 s on the gun; the idle run 18 minutes.
- Later runs (2026-10-19, after the stance search, the guard edge fix, the repair on the contact ruler, the grip
  purpose weighted x3): the firing hand keeps a high grip (palm 10 cm2, pads 6.4 + 2.8), its index within 17 deg and
  its thumb within 18 deg of their lines (limit 15), web 0, cover 20%; the support hand swings between runs (run A:
  palm 3.5 cm2 on the panel, fingers over the firing fingers; run B: under the grip, 3.9 cm2 pressed into the firing
  hand). Results swing run to run: the search is random and the area thresholds are estimates.
- Needed from the owner: pick the run whose hold looks right (or say what is wrong in each), so the areas the
  purposes ask for can be set from a hold the owner accepts, not guessed.
- Not yet: the other clips, the working holds
  (magazine, slide) on purposes, the skin check baseline, the game check loop.

## State (2026-10-22)

- The M1911A1 was rebuilt from its drawings (Blueprint Lab). On the rebuilt gun the firing hand's hold stops at step 2
  (placed): 606 cm2 of skin inside the gun, 1.28 cm at most; the wrist 17 degrees bent. This was so before the tissue
  change (the 09:26 run) and is so after it. No clips are made until it passes.
- After the touch-point fixes (2026-10-22): all touch points pass their definitions; web 0.10 cm, index 0.10 cm from
  line B, wrist 14; step 2 still breaks the palm's wall (the open flat palm 1.28 cm in the 3.25 cm wide grip, 570 cm2).
  Waiting on the owner: (a) turn about A-B, (b) palm wall read after the palm cups, (c) palm out to touch.
- The tissue give is built and tested on known answers (contact_solve_tests 12-13, clip, sdf, direct); it acts at
  step 4 (the fingers' closing) and in the walls, press shape and clips.

## 2026-10-23: web on A and index on line B are a hard rule

The owner: "I said do not bring the hand out of point A and line B, why you missed it all the time? Hard rule no
skip." Every step of the hold keeps the web fold on A and the straight index on line B; a step that moves either
off is a broken wall: undone, stopped, reported. Never traded for another goal (the palm, the forearm, the height).
The owner also set: "we start with the aim position first, then the idle position is just the aim position with a
pull"; and the experiment "keep the hand, the wrist and the finger, move the hand up so index finger is point
straight to B line direction with the B line and the straight direction to the ground make a 90 degree angle, then
bring the gun up so B line match the index finger line."
