"""Known answers for the thumb solved by formula (digitsolve): plain Python, no Blender.

    python examples/digit_solve_tests.py
Prints DIGIT SOLVE ok, or the failures and exits 1."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import digitsolve as DS  # noqa: E402

fails = []


def check(label, ok, got=None):
    if not ok:
        fails.append("%s (got %s)" % (label, got))


# a straight thumb along +x at height 5 cm, its pad facing down (-z), the palm facing down; the object's surface the
# plane z = 2 cm, the object below it. The thumb must tip down at its base and bend to put its pad on the plane.
plane = lambda X: ((X[:, 2] - 0.02) * 100.0, np.tile([0.0, 0.0, 1.0], (len(X), 1)))   # noqa: E731
seg = {"01": np.array([[0.01 * k, 0.0, 0.05] for k in range(1, 4)]),
       "02": np.array([[0.04 + 0.01 * k, 0.0, 0.05] for k in range(0, 3)]),
       "03": np.array([[0.07 + 0.005 * k, 0.0, 0.05] for k in range(0, 4)])}
d = {"p01": [0.0, 0.0, 0.05], "p02": [0.04, 0.0, 0.05], "p03": [0.07, 0.0, 0.05], "pn": [0.0, 0.0, -1.0],
     "index_meta": [1.0, 0.0, 0.0], "h02": [0.0, 1.0, 0.0], "h03": [0.0, 1.0, 0.0], "bend02": 0.0, "bend03": 0.0,
     "seg": seg, "seg_give": {j: np.full(len(v), 0.05) for j, v in seg.items()},
     "pad": np.array([[0.075, 0.0, 0.045], [0.085, 0.0, 0.045]]), "pad_n": [0.0, 0.0, -1.0], "give_pad": 0.15}
lim = {"spread_max": 80.0, "palmar_min": -20.0, "b02": (-5.0, 60.0), "b03": (-15.0, 90.0)}
band = ([0.0, 0.0, 0.02], [1.0, 0.0, 0.0], 1.5)

best, counts = DS.thumb_solve(d, plane, lim, band, n_index=[0.0, 0.0, -1.0], t2s=np.arange(0, 61, 5), t3s=np.arange(0, 91, 10))
check("a thumb pose is found", best is not None, counts)
if best:
    check("its pad touches the plane within its give", -0.15 <= best["pad_gap_cm"] <= 0.1, best)
    check("its pad is on the target line", best["pad_to_line_cm"] <= 1.5, best)
    check("the base stays inside its spread", best["spread_deg"] <= 80.5, best)
# the grab: the index's side facing the same way as the thumb's (not opposite) keeps no pose
best2, counts2 = DS.thumb_solve(d, plane, lim, band, n_index=[0.0, 0.0, 1.0], t2s=np.arange(0, 61, 5), t3s=np.arange(0, 91, 10))
check("no grab when the index's side is not opposite", best2 is None and counts2["not opposite the index"] > 0, counts2)
# the walls: a spread limit of 5 degrees with the index's line 60 degrees off the thumb refuses every base turn
d3 = dict(d, index_meta=[0.5, 0.866, 0.0])
best3, counts3 = DS.thumb_solve(d3, plane, dict(lim, spread_max=5.0), band, n_index=[0.0, 0.0, -1.0],
                                t2s=np.arange(0, 61, 5), t3s=np.arange(0, 91, 10))
check("the base's spread wall refuses the pose", best3 is None and counts3["base walls"] > 0, counts3)

if fails:
    print("DIGIT SOLVE failed: " + "; ".join(fails))
    sys.exit(1)
print("DIGIT SOLVE ok")
