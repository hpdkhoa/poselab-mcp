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
- The cylinder: a real wrap (palm on the side, fingers round, thumb over), the body inside every range, but the gate
  fails: the fingers hover 0.2 to 1 cm off the handle (they stop at their first capsule touch).
- The contact areas the purposes ask for are first estimates; set them from holds the owner accepts.
- Not yet: conforming finger closure, the M1911 idle on the new search, the other clips, the working holds
  (magazine, slide) on purposes, the skin check baseline, the game check loop.
