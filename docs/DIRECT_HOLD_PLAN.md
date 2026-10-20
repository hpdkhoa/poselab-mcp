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
