"""Known answers for the direct hold's equations (directmath): plain Python, no Blender.

    python examples/direct_tests.py
Prints DIRECT ok, or the failures and exits 1."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import directmath as DM  # noqa: E402

fails = []


def check(label, ok, got=None):
    if not ok:
        fails.append("%s (got %s)" % (label, got))


l1, l2 = 0.30, 0.27
check("a straight arm reaches l1 + l2", abs(DM.reach_for_elbow(l1, l2, 0.0) - 0.57) < 1e-9, DM.reach_for_elbow(l1, l2, 0.0))
check("a right-angle elbow reaches sqrt(l1^2 + l2^2)", abs(DM.reach_for_elbow(l1, l2, 90.0) - math.hypot(l1, l2)) < 1e-9)
for f in (5.0, 20.0, 45.0, 90.0, 140.0):
    d = DM.reach_for_elbow(l1, l2, f)
    back = DM.elbow_for_reach(l1, l2, d)
    check("elbow from reach inverts reach from elbow at %g" % f, abs(back - f) < 1e-6, back)
check("past the arm's length is unreachable", DM.elbow_for_reach(l1, l2, 0.60) is None)
# bisection on a growing function: x^2 + x on [0, 10] reaching 12 at x = 3
x, fx, n = DM.bisect(lambda v: v * v + v, 12.0, 0.0, 10.0, tol=1e-9)
check("bisection finds x = 3", abs(x - 3.0) < 1e-6, x)
check("bisection takes about log2(range / tol) steps", n <= 40, n)
x, fx, n = DM.bisect(lambda v: v, 20.0, 0.0, 10.0)
check("a target past the range returns the near end", x == 10.0 and n == 0, (x, n))

# the preferred length of pull: shoulder at the origin, the pocket 4 cm in front of it, the bore straight ahead, the
# grip 5 cm under the bore. |G - S| = D* with G = (0, 0.04 + LOP, -0.05): LOP = sqrt(D*^2 - 0.05^2) - 0.04
S0, P0, B0, U0 = [0.0, 0.0, 0.0], [0.0, 0.04, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]
Ds = DM.reach_for_elbow(0.30, 0.27, 60.0)
lop, used, cut = DM.preferred_lop(S0, P0, B0, U0, 0.05, 0.30, 0.27, 60.0, lop_range=(0.0, 1.0))
check("the preferred length of pull meets the reach", abs(lop - (math.sqrt(Ds * Ds - 0.05 ** 2) - 0.04)) < 1e-4, lop)
lop, used, cut = DM.preferred_lop(S0, P0, B0, U0, 0.05, 0.30, 0.27, 60.0, lop_range=(0.30, 0.36))
check("the stock's range holds the length of pull", cut and abs(used - 0.36) < 1e-9 if lop > 0.36 else not cut, (lop, used, cut))

if fails:
    print("DIRECT failed: " + "; ".join(fails))
    sys.exit(1)
print("DIRECT ok")
