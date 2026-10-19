# The hold tool: goals and decisions

This file holds the owner's goals for the hand-hold tool and every decision taken on the way, with its date and its
reason. Read it before changing the hold code, and check each change against it. Last updated 2026-10-19.

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
| 2026-10-19 | All decisions and goals kept in this file, to read and revalidate later | The owner asked |
| 2026-10-19 | If the search is slow, make it faster so more retries fit | The owner asked |

## How to revalidate

- Reader known answers: `blender -b --factory-startup -P examples/hold_reader_tests.py` (a cylinder, a pistol block,
  a box).
- Geometry known answers: `blender -b --factory-startup -P examples/geometry_tests.py`.
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
