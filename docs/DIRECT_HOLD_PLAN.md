# The direct hold: plan, factors and equations

Agreed with the owner on 2026-10-20. For a gun, the hand's place on it is known from the shooting manuals, so
the hand is placed there directly and the body is solved backward from it. No search over hand turns or stance
places. An object with no marks still uses the target solver.

Every value below is a number the tool prints, so each step can be checked.

## Names

- S: the shoulder (upper arm head). E: the eye. W: the wrist (hand bone head).
- l1: upper arm length (shoulder to elbow). l2: forearm length (elbow to wrist). Both read from the arms.
- f: elbow flexion (0 = straight arm).

## Step 0. The marks (once per gun, saved)

The gun facts give points: the grip, the support, the trigger, the sights. The direct hold also needs:

| Mark | How it is found | Saved as |
|---|---|---|
| Grip axis | the object reader's run nearest the grip mark | `grip_axis` (point, direction) |
| Index line | the trigger guard opening's top edge | `guard_line` (two points) |
| Web point | the crook under the frame's overhang (`web_point`) | `web_point` |
| Firing palm face | the grip style's firing contact face (FM 3-23.35 2-3) | `firing_face` |
| Support palm face | the grip style's support contact face (FM 3-23.35 2-2): the gun's support side | `support_face` |
| Long guns, later | butt plate centre, cheek rest, length of pull range, handguard area | not in any facts file yet |

The facts' `normals.support` points down; the support palm faces the gun's side, so `support_face` comes from
the grip style. The marks are saved to `hold_marks.json` with a hash of the gun's mesh; a changed gun recomputes.

## Step A. The hand from the marks

The firing hand is turned so its palm faces into the grip from `firing_face` and its knuckles run along the grip
axis toward the top, then moved so its web sits just under the web point (`seed_hold` with that face and no turn).
The fingers start open from the rig's bind pose (`open_fingers`), each finger in its own plane; the index starts
straight. The idle pose's fingers can sit swung out of their plane, so they are not the start.

The palm is then settled by a calculated shift (`settle_palm`), not by fixed steps. With d_i the signed distance
of palm point i to the gun (inside below 0) and n_i the gun's outward normal there, the shift x solves

    min |x|^2   with   n_i . x >= -give_i + 0.05 cm - d_i   for every palm point
                       n_k . x <=  gap - d_k                 for the nearest palm point k

(a small quadratic program, Hildreth's method, `contactsolve.min_shift`). The support hand adds rows that keep
every skin point out of the firing hand past the skin's give (`rest_on_other`). The step is linear, so the skin is
read again and the shift solved again, at most 4 rounds.

## Step B. The gun's distance from the body

The preferred reach from the preferred elbow flexion f_pref (law of cosines):

    D*^2 = l1^2 + l2^2 + 2 l1 l2 cos(f_pref)

The gun moves along the view's forward line by d. The hand rides with it (Step A at each d). |W(d) - S| grows
with d, so bisection finds d with |W(d) - S| = D* exactly; d is kept inside [-15, 15] cm of the style's stance.

f_pref is a grip style value. For the two-hand pistol: 20 degrees ("arms out, elbows a little bent", FM 3-23.35);
an estimate until a measured value replaces it. Long guns, later: the butt on the shoulder pocket and the length of
pull solved the same way, |G(LOP) - S| = D*, clamped to the stock's range.

## Step C. The arm

The hand stays where Step A put it. The elbow's swing about the shoulder-wrist line (phi) and the shoulder girdle
(forward, up) are set by a fixed step search that reads only joint angles (microseconds each):

    J = (wrist bend / 10)^2 + (forearm roll / 30)^2 + (girdle share)^2

For the firing hand only, the gun's distance along the sight line is a move too, at most 3 cm either way from
Step B. Along the sight line the aim does not change. The support hand is not on the gun yet. These moves are on
joint angles, where no distance field gives a direction, so they stay a fixed step search (4, 2, 1 degrees).

Walls (never crossed): the AAOS ranges, the wrist within 30 degrees of the forearm line, the elbow 5 to 150
degrees, the elbow 2 cm under the shoulder, the girdle's working range.

## The valid-start rule

After Steps A to C, before any finger work, the start is valid only when all hold (changed 2026-10-20: the fingers
are open at the start and Step D closes them, so they are judged after Step D by every wall):

1. The arm inside every AAOS wall: wrist flexion 80, extension 70, radial 20, ulnar 30, forearm roll 80 each way,
   elbow 5 to 150, the shoulder's ranges.
2. The working rules: wrist within 30 degrees of the forearm line, elbow 2 cm under the shoulder, girdle forward 15,
   back 20, up 30, down 8.
3. The hand's body does not clip: no palm, palm side, web or forearm skin past its give (palm 0.3 cm, palm side
   and web 0.15 cm), nothing of the gun inside that flesh, at most 0.05 cm2.
4. Reachable: |W - S| <= l1 + l2 - 1 cm.

An invalid start is reported with each wall it breaks and by how much. It is not repaired and no hold is made.

## Step D. The fingers

Each wrapping finger closes until its skin touches the gun, by calculated angles (`contactsolve.chain_close`),
not fixed steps. A point p on a segment that turns by s about the hinge through c along the axis a moves at
dp/ds = a x (p - c), so its distance to the gun changes at

    dd/ds = n(p) . (a x (p - c))

and the angle that brings it to the target distance t is the Newton step s = (t - d) / (dd/ds). The knuckle leads
(the joints turn at rates 1, 0.5, 0.35). Each round s is the first touch over every point coming closer, cut at
the joints' room and at 15 degrees (the step is linear, so a long one is read again). A segment that touches
stops its own joint and those behind it. A segment pressed past 0.12 cm opens by the Newton step back to half of
that, its joints down to a holding finger's least bend. The relax step opens a pressing joint by the same
formula. The straight index and the thumb are turned onto their lines by the fixed step search inside the walls:
those aims are lines, not distances, so no field gives the direction.

The knuckle's side swing is read against the finger's own metacarpal bone (it was read against the middle
finger's line, so an open pinky read 29 degrees; against its own bone it reads 5).

## Step E. The fast inside-the-hand check (built 2026-10-20)

The test "is the gun inside the hand's flesh" keeps its last answer per hand with the skin it was read on. After
a move only the gun's samples near skin that moved are tested again: those within a 2 cm grid cell, grown by one
cell, of a moved vertex at its old place or its new one. A point in the flesh is within 1.5 cm of a skin vertex,
so every sample whose answer can change is in that set. When the gun moves, all are tested. With
POSELAB_PIERCE_CHECK=1 every call also runs the full test and counts the calls where the two differ.

## Step F. The grip height (a check, all guns)

Recoil turns the gun up with moment M = F_recoil * h, h the bore's height above the hand's support. A higher grip
gives a smaller h. The report prints h.

## Time budget

Every run finishes in 10 minutes. Estimate for the direct flow: load 0.5 min, gun read 0.5 min, Steps A to C both
hands under 10 s, Step D about 20 s, clips 1 min, checks and render 0.5 min.
