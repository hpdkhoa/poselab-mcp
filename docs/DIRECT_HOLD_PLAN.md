# The direct hold: plan, factors and equations (v0.11.0, rebuilt 2026-10-20)

## The owner's words (the method; built as stated)

- "why not place the hand directly on the gun at the position you got from the document/research paper, then from
  there you adjust the wrist, the arm, the elbow and the shoulder?"
- "i think you should look at the points on the hand that touch the part of the gun, like the grip the trigger ring
  and other touchable parts."
- "treats all the problems as math and physics to optimize, calculate everything before start drawing"
- "it need to move with the direction opposite the gun if clipping or into the gun if not grip and the distance
  should be calculate, not blind moving"
- The walls (anatomy, no clipping) are never scored, traded or skipped: checked at every step.
- Each run takes 10 minutes at most.
- The ready hold's place is solved, the muzzle not tipped down.

The first build (v0.10.0) did not follow the first sentence: it started the hand from an older helper (seed_hold)
and then moved the hand on the gun (a sliding palm settle, a turn onto the forearm). Those are removed. A step that
moves the hand on the gun after Step A is a fault.

## Step 0. The touch points (from the shooting manual and the gun's marks)

FM 3-23.35's two-hand grip: the firing hand's web high on the backstrap under the grip safety, the backstrap down
the palm, the trigger finger along the frame (ready: off the trigger), the other fingers round the front strap; the
support palm on the other panel, its fingers over the firing fingers. In the gun body's frame, so they ride the gun:

| Hand | Hand point | Gun point | Weight |
|---|---|---|---|
| firing | web | the web point (the crook under the grip safety) | 2 |
| firing | the web-to-wrist line | along the grip's axis, downward (the backstrap down the palm) | 2 |
| firing | palm facing | into the firing side's panel (a direction only) | 3 |
| firing | index middle joint | the frame's side by the trigger guard's top edge | 1 |
| support | palm middle | the support mark on the other panel, facing into it | 3 |
| support | the knuckles' line | along the grip's axis, the index's toward the top | 1 |

The trigger guard ring is met by the fingers' closing (Step D): the open hand's middle finger reaches it only when
it curls. The palm's middle onto the facts' grip mark was tried first and dropped: the hand's web-to-palm distance
is 3.2 cm, the gun's web point to that mark 6.7 cm, so no hand meets both (pair_geometry in the report). The
web-to-wrist line stands in for the backstrap line; the wrist is not the heel of the hand, so this line is an
approximation (open item).

## Step A. The hand directly on the gun (by formula, no pose set)

The fingers open (the rig's bind pose). The hand, rigid, is moved by x = (t, w) (shift, turn about c):

    min  sum_k w_k |p_k + t + w x (p_k - c) - T_k|^2
    with n_i . (t + w x (p_i - c)) >= -give_i + 0.03 cm - d_i      every body skin point i (palm, palm side, web)
         -n_h . (t + w x (q_j - c)) >= depth_j - give_j + 0.03 cm   every gun point j inside the body's flesh
         (support hand: the body skin out of the firing hand past the skin's give)

A constrained least squares: with H = J'J = L L' and x0 the free fit, y = L'(x - x0), the walls read
(A L'^-1) y >= b - A x0, solved by Hildreth's method (contactsolve.constrained_fit). It is linear in x, so it is
solved again on the moved points. The hand is then fixed on the gun.

## Steps B and C. The wrist, the arm, the elbow and the shoulder (by formula)

For every gun place in the box (the firing hand only: 5 cm right and left, 3 cm forward and back, 5 cm up and down,
1 cm apart; 847 places), every girdle setting (30) and every elbow swing (91, 2 degrees apart), the elbow is on the
circle of the two-bone arm and every arm wall is worked out with the walls' own formulas (bodysolve.arm_walls):
elbow under the shoulder, elbow bend, wrist bend and its four ranges, forearm rotation, the shoulder's ranges. Only
bodies inside every wall are kept; they are ranked by comfort, which holds no wall:

    cost = (wrist / 10)^2 + (forearm roll / 30)^2 + (girdle share)^2 + ((elbow - f_pref) / 20)^2 + (gun moved cm / 10)^2

f_pref is Step B's reach equation D*^2 = l1^2 + l2^2 + 2 l1 l2 cos(f_pref) as a preference. The best is posed and
read by the real walls (the forearm's links and the wrist's twist have no formula); a body the real walls refuse is
replaced by the next, at most 12. On the M1911 the formulas and the posed rig agree within 1 degree.

## The valid-start rule

After Steps A to C: the arm inside every wall, the hand's body (palm, palm side, palm back, web, forearm) not in the
gun or the other hand past its give, the arm 1 cm short of full reach. The fingers are judged after Step D.

## Step D. The fingers (calculated moves, every step inside the walls)

A point p on a segment turning by s about the hinge through c along a moves at dp/ds = a x (p - c), its distance to
the gun changes at dd/ds = n(p) . (a x (p - c)), and the Newton step s = (target - d) / (dd/ds) closes it onto the
surface or opens it out (contactsolve.chain_close; the gun's points inside the flesh ride as inner points). Each of
close, thumb, relax and close again is undone if it breaks a wall. The thumb's line fit is still a fixed step search
(open item).

## Step E. The inside-the-hand check, only the moved part

Built; 0 differences against the full check on 368 checked calls.

## Long guns: the preferred length of pull (formula built, no gun can use it yet)

    G(LOP) = pocket + bore * LOP - up * grip_drop,   |G(LOP) - S| = D*,   held to the stock's range

directmath.preferred_lop, tested in examples/direct_tests.py. No long gun's facts have a butt plate or pocket mark
yet, so no run uses it.

## Time

Hard stop at 10 minutes; the run plans its work to 9.

## Planned next (v0.12, agreed 2026-10-21, not built yet)

The owner's words: "leave the support hand out of equation, after done firing hand then come to support hand so only
one hand at a time"; "never pull out the touch point, never rotate the palm direction out of the object"; "choice 1
but count the finger need to touch A-to-B line ... must not clipping or crossed the wall by default calculation";
the thumb "need to touch the left side of the object or opposite side of index finger to complete a grab"; the
support hand: "the pattern is the same, index finger in one side and thumb is in another side".

One template for both hands, one hand at a time (the support hand only after the firing hand passes):

1. Place, exact: the center touch point (a point) and the line touch point (a line), one calculation.
2. Roll about the center-to-line axis: the only move; both touches stay exact. For each roll angle (1 degree
   steps, the hand's fixed shape, no posing), work out:
   a. the fingers' angles for the hold, calculated onto their touch places (option 1), inside their AAOS and
      holding ranges, no skin past its give;
   b. the thumb's 4 angles onto the opposite side (pad on the surface within its give, facing in), inside its ranges;
   c. the grab: the thumb's side and the fingers' side face opposite ways (n_thumb . n_fingers < 0);
   d. no palm, web or touching-finger skin past its give, the touching finger kept on its line, the palm facing in.
   Keep the angles that meet all; take the one with the palm squarest to its panel and the digits nearest relaxed.
   None: stop and report which condition fails, by how much, at the nearest angle. No pulling away.
3. Then the body (Steps B, C), the start check, and the pose read by the real walls.

Targets (kind, source):

| Hand | Part | Target | Kind |
|---|---|---|---|
| firing | web | the crook under the grip safety | point (center) |
| firing | index middle joint, index straight | the frame's right side by the guard's top edge | line |
| firing | middle, ring, little | wrap the front strap under the guard | surface |
| firing | thumb pad | the frame's left side, by the slide stop | line or area |
| firing | palm | into the right panel | facing |
| support | palm | the left panel's open part (the support mark) | point (center) |
| support | thumb pad | forward along the frame's left side, under the firing thumb | line |
| support | fingers | over the firing fingers, round the front to the right side | surface (the firing hand's skin) |
| support | palm | into the left panel | facing |

Grab check: firing hand, index (right) against thumb (left); support hand, thumb (left) against fingers (right, over
the firing fingers).

## Measured, not guessed (v0.12, 2026-10-21)

The owner's rule: "in 3D the target need to be the surface or the skin of the real final object, not guessing not
scale from bone ... you have the tools to get exact numbers". Built so far:

| Number | Before | Now |
|---|---|---|
| The web's touch point | a point between two bone heads, a guessed 0.9 cm under the skin | the skin fold, measured by a ray (that bone point is 2.71 cm under the skin) |
| The index's touch point | the middle joint's bone head plus a 0.75 cm capsule radius | the pad skin at the middle joint, measured by a ray |
| The support palm's touch point | a bone midpoint plus the fitted palm radius | the palm skin at its middle, measured by a ray |
| Each fingertip's length | 0.8 of the middle segment | the farthest tip skin along the end segment, measured |
| The gun's touch points | a step back along the distance grid's gradient (missed by 0.89 cm in the crook) | the gun mesh's nearest point |
| The frame line by the guard | pushed 3 cm sideways, then snapped | a ray from outside the firing side, its first hit |
| A gap that decides a wall | the distance grid (error 0.03 cm) | the mesh's exact distance for points within 0.06 cm of the wall |

The numbers are kept next to the code: poselab_mcp/data/measured/hands_<rig hash>.json (in the carrying bone's
frame, with the source files' names and hashes) and weaponlab/gunkit/data/measured/<gun>.json.

Still guessed (no mesh can measure them; each needs a published source): the skin's give (palm 0.3, palm side and
web and pads 0.15, else 0.05 cm), the glove's thickness (0.15 cm, "a guess"), the working arm and finger limits
marked "working value" in ranges.py, a holding finger's limits, the purpose and comfort numbers marked "estimate",
the ready place in the view and the preferred elbow flexion (20). Solver settings (step sizes, tolerances) are
choices, listed in the audit, not body facts.

## The distance grids, checked against the mesh (grid v3, 2026-10-21)

examples/sdf_sign_check.py reads 6000 points within 1 cm of the M1911's surface by the grids and by the exact check
(each closed shell on its own, inside by a six-direction ray-parity majority, the union the least distance). The
M1911 is 21 closed shells, some inside others (0 open edges); counting crossings over all of them at once read a point
inside two shells as outside.

| Grid | Wrong side | Largest difference |
|---|---|---|
| v1: either of two ray directions, all shells at once | 15% (hand points near the grip) | 2.26 cm |
| v2: six votes, all shells at once | 2.6% | 1.01 cm |
| v3: six votes, shell by shell, the union the least | 0.55% (28 of 5095) | 0.112 cm |

v3 does not pass (the goal is 0 wrong sides): its remaining errors are within 0.112 cm of the surface. So every gap
that decides a wall and reads within 0.15 cm of its limit is measured on the mesh itself (holdlab.exact_gaps,
contactlab.contact_pairs).

## v0.14 plan: the owner's grip order (2026-10-23, not built yet)

Source: the owner's chart "Pistol grip: backstrap placement, right handed shooter" (Sharp Shooting) and two side
photos (a Beretta 92 and an M1911, the index straight along the frame, the wrist up, the forearm behind the gun).

| The owner's words | The design | Target kind (source) |
|---|---|---|
| "1. With right hand (dominate hand). grip the back strap- straight back, deep into the webbing between the thumb & index finger. no gap." | The open hand placed in one calculation: the web fold on A, pressed to touching within the skin's give (no gap); the knuckle line (index knuckle to little knuckle) parallel to the backstrap line (the grip run read along the well; the owner's answer "Knuckle line along backstrap", photo 3); turned about that line so the palm faces the left panel (the purpose rule). Walls of the web and the palm read. | A: a point (the crook's middle line). Backstrap: a line. Panel: a surface. (chart, top view) |
| "2. Close the fingers and keep the forearm align with the gun" + answer "Stays straight on line B" + "first move the wirst up" (answer: "Same: done in step 2") | The palm cups; middle, ring, little wrap by the grip's size; the thumb curls to the left side (close_grab). The index: middle and end joints straight; its base knuckle turned so the pad line lies along line B, close, not pressing. The arm solved so the forearm lies in the gun's middle vertical plane (top view: the chart's dashed line), its tilt to the bore the least the wrist walls allow (wrist up). Walls of every part read. | Line B: a line (owner's red line). The gun's middle plane: a surface (chart). |
| "3. Grip the gun as high as possible under the beaver tail" + answer "Slide up to the highest no-clip point" | The closed hand (and the arm with it) slid up along the backstrap line to the highest place where no part clips past its give; A becomes that place. The arm re-solved to keep the forearm aligned. Walls read. | Beavertail underside: a surface (chart, photo 3). |

Answered 2026-10-23: the index base knuckle may turn inside step 2 ("Allow, inside step 2 closing"; its range a
wall); straight back = the knuckle line along the backstrap ("Knuckle line along backstrap"; my first reading, the
hand's long line, was wrong by photo 3).
