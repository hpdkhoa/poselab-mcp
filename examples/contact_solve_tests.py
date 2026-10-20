"""Known answers for the calculated contact moves (contactsolve): plain Python, no Blender.

    python examples/contact_solve_tests.py
Prints CONTACT SOLVE ok, or the failures and exits 1."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import contactsolve as CS  # noqa: E402

fails = []


def check(label, ok, got=None):
    if not ok:
        fails.append("%s (got %s)" % (label, got))


# 1. One joint over a floor: a straight segment along +x from the hinge at the origin, the hinge along +y (turning
# + about +y takes +x toward -z), a floor h below. The tip (length l) touches at s = asin(h / l).
h, l = 0.02, 0.05
floor = lambda X: ((X[:, 2] + h) * 100.0, np.tile([0.0, 0.0, 1.0], (len(X), 1)))   # noqa: E731
seg = np.array([[l * k / 10.0, 0.0, 0.0] for k in range(1, 11)])
closed, touching, rounds, log = CS.chain_close({"01": seg}, {"01": np.zeros(3)}, {"01": np.array([0.0, 1.0, 0.0])},
                                               {"01": 90.0}, {"01": 0.0}, {"01": 1.0}, floor, touch_cm=0.01)
want = math.degrees(math.asin(h / l))
check("one joint closes to asin(h / l)", abs(closed["01"] - want) < 0.2, (closed, want))
check("one joint touches", "01" in touching, touching)
check("one joint takes few rounds", rounds <= 6, rounds)

# 2. A floor out of reach: the joint closes to its room and stops
closed, touching, rounds, _ = CS.chain_close({"01": seg}, {"01": np.zeros(3)}, {"01": np.array([0.0, 1.0, 0.0])},
                                             {"01": 30.0}, {"01": 0.0}, {"01": 1.0},
                                             lambda X: ((X[:, 2] + 0.2) * 100.0, np.tile([0.0, 0.0, 1.0], (len(X), 1))))
check("out of reach closes to its room", abs(closed["01"] - 30.0) < 1e-6 and not touching, (closed, touching))

# 3. Pressed in at the start: the floor 0.5 cm above the segment's tip line. The joint opens by the Newton step
# until the deepest point is at -press / 2
press = 0.12
sunk = lambda X: ((X[:, 2] + 0.0) * 100.0 + (X[:, 0] / l) * -0.5, np.tile([0.0, 0.0, 1.0], (len(X), 1)))  # noqa: E731
closed, touching, rounds, log = CS.chain_close({"01": seg}, {"01": np.zeros(3)}, {"01": np.array([0.0, 1.0, 0.0])},
                                               {"01": 90.0}, {"01": 20.0}, {"01": 1.0}, sunk, press_cm=press)
check("a pressed segment opens (closed below 0)", closed["01"] < 0.0, closed)
X = CS.rodrigues(seg, np.zeros(3), np.array([0.0, 1.0, 0.0]), math.radians(closed["01"]))
dmin = float(sunk(X)[0].min())
check("after opening no point is past the press", dmin >= -press - 0.02, dmin)

# 4. Three joints around a round handle (radius 1.75 cm, axis along y through (0, 0, -2.6 cm)): the finger lies along
# +x over it, each segment 2.5 cm. Every segment ends touching or at its room; none presses past press_cm.
R, cz = 0.0175, -0.026
cyl = lambda X: ((np.hypot(X[:, 0], X[:, 2] - cz) - R) * 100.0,  # noqa: E731
                 np.c_[X[:, 0], np.zeros(len(X)), X[:, 2] - cz] / np.maximum(np.hypot(X[:, 0], X[:, 2] - cz), 1e-12)[:, None])
L1 = 0.025
x0 = -0.025
pts = {j: np.array([[x0 + L1 * (k + t / 6.0), 0.0, -0.0075] for t in range(1, 7)]) for k, j in enumerate(("01", "02", "03"))}
heads = {j: np.array([x0 + L1 * k, 0.0, 0.0]) for k, j in enumerate(("01", "02", "03"))}
hinges = {j: np.array([0.0, 1.0, 0.0]) for j in ("01", "02", "03")}
closed, touching, rounds, log = CS.chain_close(pts, heads, hinges, {"01": 90.0, "02": 100.0, "03": 80.0},
                                               {"01": 5.0, "02": 0.0, "03": 5.0}, {"01": 1.0, "02": 0.5, "03": 0.35},
                                               cyl, touch_cm=0.05, press_cm=press)
P2 = {j: v.copy() for j, v in pts.items()}
H2 = {j: v.copy() for j, v in heads.items()}
order = ("01", "02", "03")
for j in order:
    a = math.radians(closed[j])
    for k in order[order.index(j):]:
        P2[k] = CS.rodrigues(P2[k], H2[j], hinges[j], a)
        if k != j:
            H2[k] = CS.rodrigues(H2[k][None, :], H2[j], hinges[j], a)[0]
dd = {k: float(cyl(P2[k])[0].min()) for k in order}
check("a wrap presses nowhere past press_cm", min(dd.values()) >= -press - 0.02, dd)
room = {"01": 90.0, "02": 100.0, "03": 80.0}
check("every segment touches or its joint is at its room",
      all(k in touching or abs(closed[k] - room[k]) < 1e-3 for k in order), (touching, dd, closed))
check("the touching segments are on the surface", all(dd[k] <= 0.05 + 1e-6 for k in touching), dd)
check("a wrap takes at most 24 rounds", rounds <= 24, rounds)

# 5. The smallest shift: points 1 cm inside a floor (normal +z) need a 1 cm shift up, nothing sideways
Nr = np.tile([0.0, 0.0, 1.0], (5, 1))
x, viol = CS.min_shift(Nr, np.full(5, 0.01))
check("min shift out of a floor", np.allclose(x, [0, 0, 0.01], atol=1e-6) and viol < 1e-6, (x, viol))
# a corner: 1 cm into a floor (+z) and 0.5 cm into a wall (+x): the shift is (0.005, 0, 0.01)
Nr = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
x, viol = CS.min_shift(Nr, np.array([0.01, 0.005, -0.02]))
check("min shift out of a corner", np.allclose(x, [0.005, 0, 0.01], atol=1e-6), x)
# rows that do not bind leave the shift at 0
x, viol = CS.min_shift(Nr, np.full(3, -0.01))
check("free rows give no shift", np.allclose(x, 0.0), x)

# 6. A turn as well as a shift: a bar pressed down at one end and up at the other (no shift helps: the needs pull
# opposite ways); with the turn's columns ((p - c) x n) / Lr the smallest move is a turn about y alone
Lr = 0.05
pA, pB = np.array([0.05, 0.0, 0.0]), np.array([-0.05, 0.0, 0.0])
nA, nB = np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])
rows6 = np.array([np.r_[nA, np.cross(pA, nA) / Lr], np.r_[nB, np.cross(pB, nB) / Lr]])
x6, viol6 = CS.min_shift(rows6, np.array([0.01, 0.01]))
check("a pinch needs a turn: the turn meets it", viol6 < 1e-6 and np.allclose(x6[:3], 0.0, atol=1e-7), (x6, viol6))
check("the turn is about y by 0.2 rad", abs(x6[4] / Lr + 0.2) < 1e-4, x6)
x3, viol3 = CS.min_shift(rows6[:, :3], np.array([0.01, 0.01]))
check("a shift alone cannot meet a pinch", viol3 > 0.009, (x3, viol3))

# 7. The turn about an axis that lines a direction up with another: h along x, f in the y-z plane at 30 degrees
# from y, g along z: the best turn is 90 degrees, leaving the 30 degree tilt out of the plane
psi, left = CS.align_turn([1, 0, 0], [0, math.cos(math.radians(30)), math.sin(math.radians(30))], [0, 0, 1])
check("align turn is 90 degrees", abs(math.degrees(psi) - 90.0) < 1e-6, math.degrees(psi))
check("align leaves the 30 degree tilt", abs(left - 30.0) < 1e-6, left)

if fails:
    print("CONTACT SOLVE failed: " + "; ".join(fails))
    sys.exit(1)
print("CONTACT SOLVE ok")
