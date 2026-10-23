# Pistol grip from the Sharp Shooting images: a machine plan (v0.1, 2026-10-23)

Source: two images, "Pistol Grip: Backstrap Placement" and "Pistol Grip: Wrist Mechanics" (Sharp Shooting, right handed shooter).
The owner asked (2026-10-23): "make the plan so a machine can have the plan to execute the render for a hand and a gun".

This plan is not built. It lists what a run would do, in order, with every target's kind and every number's source.
The existing hard rule stays under it: the web fold on A and the straight index on line B at every step (HOLD_DECISIONS.md, 2026-10-23).
The images say nothing about the index finger, so line B comes from the owner's rule, not from the images.

## 0. Inputs

| Input | What it is | Source |
|---|---|---|
| gun mesh | the gun as rendered, bore axis, slide top, backstrap, beavertail underside, both grip panels, front strap, trigger guard ring | the game model |
| hand mesh, firing | the posed skin mesh, with web fold, palm, index pad, finger segments, wrist, forearm | the rig |
| hand mesh, support | the same, left hand | the rig |
| measured cache | skin give, palm thickness, web to index pad distance, all measured from the meshes | numbers cache (never guessed) |

## 1. Touch points (checked first, before any hand is placed: `check_touch_points`)

Each row names the kind. A line or a surface is never turned into one exact point.

| Id | Hand | Hand part | Gun target | Kind | The image's words | Status |
|---|---|---|---|---|---|---|
| A | firing | web fold (thumb to index) | the crook under the beavertail, on the grip's middle line | point | "Grip the back strap, straight back, deep into the webbing between the thumb and index finger. No gap!" | existing (web point) |
| H | firing | top edge of the hand (web and index base) | underside of the beavertail | surface | "Grip your gun as high as possible under the beaver tail" | new |
| S1 | firing | palm face | firing side grip panel, facing into it | surface | "No gap" (image 1), "fill any empty spaces" (image 2) | existing (palm face) |
| B | firing | straight index pad line | line B, parallel to the bore, on the frame's right face, middle at the trigger ring's middle height | line | not in the images | existing owner rule |
| F | firing | forearm axis | the bore axis, seen from above | line | "Your forearm should align with the gun" | new |
| S2 | support | palm face | the open part of the support side grip panel, the part the firing fingers do not cover | surface | "Fill any empty spaces to maximize hands' contact with the frame" | new |
| K | support | the fingers' segments | over the firing hand's fingers | surface (hand on hand) | "Wrap the fingers of the support hand around, hold the gun with both hands" | new |
| W | both | wrist axis | forearm axis and slide top line on one line, seen from the side | line | "Wrist locked straight" | new |

A touch point that misses its definition (wrong height, wrong side, off the middle line, off the surface) stops the run. It is reported, never placed on.

## 2. Steps, in order

Every step: walls read before and after (anatomy, no clipping, no hand into hand). A step that breaks a wall is undone, the run stops, the owner is asked. Every step keeps A and B.

| Step | Image sentence | Machine action | Check after the step |
|---|---|---|---|
| 1 | "Grip the back strap, straight back, deep into the webbing ... No gap!" | Place the firing web fold on A. Index pad line along B (existing `place_on_touch`). | web to A distance; index to B distance; both within the cache's skin give |
| 2 | "Grip your gun as high as possible under the beaver tail" | Slide the firing hand up the grip's axis until the hand's top edge meets the beavertail underside (H), stopping at the skin's give. This slide is along the backstrap only, A stays on the crook. | gap between hand top edge and beavertail underside; must be 0 within the give, never negative |
| 3 | "Your forearm should align with the gun" | Set the forearm so its axis is parallel to the bore seen from above (F). Done by the arm steps (elbow, shoulder), not by moving the hand on the gun. | angle between forearm axis and bore, top view |
| 4 | "Wrist locked straight" | Set the wrist so forearm axis and slide top line are one line seen from the side (W). | wrist bend angle, side view; angle between forearm axis and slide top |
| 5 | "Align your open support hand at about a 45 degree angle with the slide" | Open support hand. Its finger direction at 45 degrees to the slide top line, fingers down and forward, palm facing the support panel. | angle between support finger line and slide top |
| 6 | "Wrap the fingers of the support hand around, hold the gun with both hands" | Close the support fingers over the firing fingers (K) by calculated moves (chain_close), each move inside the walls. | support finger skin to firing finger skin, within the give; no hand into hand |
| 7 | "Fill any empty spaces to maximize hands' contact with the frame" | Press the support palm onto the open part of the support panel (S2), facing into it, stopping at the give. | contact area on S2; open gap area on the panel left uncovered |
| 8 | "A locked wrist helps control recoil" | Re-read W for both wrists after step 7. | both wrist bends, side view |

Step 1 is the existing hold's step. Steps 2 to 8 are the images' additions. Step 2 moves the hand on the gun after placement; this is the one place the images ask for it (the "as high as possible" rule). It needs the owner's yes before it is built, because the direct hold plan calls a hand move after Step A a fault.

## 3. Numbers

| Number | Value | Source |
|---|---|---|
| support hand to slide angle | 45 degrees | image 2, "about a 45 degree angle" |
| wrist bend, firing and support | 0 degrees (straight) | image 2, "wrist locked straight" |
| forearm to bore angle, top view | 0 degrees | image 1, "align with the gun" |
| gap at A, H, S1, S2 | 0 | image 1, "no gap"; image 2, "fill any empty spaces" |
| beavertail underside height on this gun | open | to measure on the gun mesh |
| skin give at the web, hand top edge, palms | open | numbers cache (measured) |
| tolerance for "about 45" | open | the owner decides |
| tolerance for "straight" wrist | open | the owner decides; the anatomy wall is 30 degrees, the image asks for straight |

## 4. Walls (never scored, never traded)

- Anatomy: every joint inside its range (ANATOMY.md); the wrist within 30 degrees of the forearm.
- No clipping: no hand skin inside the gun, no gun inside the flesh past the give.
- No hand into hand past the give.
- A and B kept at every step (owner, 2026-10-23).

## 5. Output of a run

- The pose, with every touch point's measured distance to its target, A first.
- Pictures: top view (F), right side view (W, B), left side view (S2, K), rear view (A, H), each labelled with the step and the key numbers. Before and after pictures for steps 2 and 7.
- A list of any touch point that stopped the run, with its definition and where it really sat.

## 6. Open items for the owner

1. Step 2 slides the hand on the grip after placement. Yes or no?
2. Tolerances for "about 45 degrees" and "straight".
3. The images give no thumb placement for either hand. Keep the existing thumb step, or leave the thumbs out?
4. The images give no finger spacing on the front strap. Keep the existing finger closing (Step D)?
