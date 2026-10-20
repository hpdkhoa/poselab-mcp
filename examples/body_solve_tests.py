"""Known answers for the body solved by formula (bodysolve): plain Python, no Blender.

    python examples/body_solve_tests.py
Prints BODY SOLVE ok, or the failures and exits 1."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import bodysolve as BS  # noqa: E402

fails = []


def check(label, ok, got=None):
    if not ok:
        fails.append("%s (got %s)" % (label, got))


L = {"wrist_max_deg": 30.0, "wrist_flexion_deg": 80.0, "wrist_extension_deg": 70.0, "wrist_radial_deg": 20.0,
     "wrist_ulnar_deg": 30.0, "elbow_bend_max_deg": 150.0, "elbow_bend_min_deg": 5.0, "elbow_under_shoulder_cm": 2.0,
     "pronation_deg": 80.0, "supination_deg": 80.0, "shoulder_flexion_deg": 180.0, "shoulder_extension_deg": 60.0,
     "shoulder_cross_body_deg": 45.0, "shoulder_behind_deg": 45.0, "shoulder_internal_deg": 70.0,
     "shoulder_external_deg": 90.0}
right, fwd, up = np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
l1, l2 = 0.30, 0.27

# 1. The elbow circle: every elbow is l1 from the shoulder and l2 from the wrist; phi = 0 hangs lowest
S, W = np.zeros(3), np.array([0.0, 0.40, -0.10])
E, ok = BS.elbow_circle(S, W, l1, l2, -up, np.arange(-90, 91, 10))
check("the circle is reachable", ok)
check("every elbow is l1 from the shoulder", np.allclose(np.linalg.norm(E - S, axis=1), l1), np.linalg.norm(E - S, axis=1))
check("every elbow is l2 from the wrist", np.allclose(np.linalg.norm(W - E, axis=1), l2))
check("phi 0 hangs lowest", int(np.argmin(E[:, 2])) == 9, E[:, 2])
check("past the arm's length there is no circle", BS.elbow_circle(S, np.array([0, 0.6, 0]), l1, l2, -up, [0])[1] is False)

# 2. The walls read as lab._arm_report reads them: the upper arm hanging, the forearm straight ahead (elbow at 90),
# the hand straight on, the thumb up: elbow 30 cm under the shoulder, wrist 0, no shoulder flexion, no rotation
E1 = np.array([[0.0, 0.0, -0.30]])
W1 = np.array([0.0, 0.27, -0.30])
K1 = W1 + np.array([0.0, 0.08, 0.0])
Ik, Pk = K1 + np.array([0, 0, 0.02]), K1 + np.array([0, 0, -0.02])     # the thumb's side up: index above the little
vals, past = BS.arm_walls(S, E1, W1, K1, Ik, Pk, "r", L, right, fwd, up)
check("the elbow is bent 90", abs(vals["elbow_bend_deg"][0] - 90.0) < 1e-6, vals["elbow_bend_deg"])
check("the elbow is 30 cm under the shoulder", abs(vals["elbow_under_cm"][0] - 30.0) < 1e-6)
check("the wrist is straight", abs(vals["wrist_deg"][0]) < 1e-6)
check("the shoulder hangs (flexion 0)", abs(vals["shoulder_flexion_deg"][0]) < 1e-6, vals["shoulder_flexion_deg"])
check("inside every wall", all(v[0] <= 0.0 for v in past.values()), {k: v[0] for k, v in past.items() if v[0] > 0})

# 3. The hand bent 40 degrees toward the thumb (up): the wrist bend is 40, past its 30 by 10, and radial 40, past 20
K2 = W1 + 0.08 * np.array([0.0, math.cos(math.radians(40)), math.sin(math.radians(40))])
vals, past = BS.arm_walls(S, E1, W1, K2, Ik + (K2 - K1), Pk + (K2 - K1), "r", L, right, fwd, up)
check("the wrist is bent 40", abs(vals["wrist_deg"][0] - 40.0) < 1e-6, vals["wrist_deg"])
check("10 past the wrist's 30", abs(past["wrist bent"][0] - 10.0) < 1e-6, past["wrist bent"])
check("radial 40 is 20 past", abs(abs(vals["wrist_radial_deg"][0]) - 40.0) < 1e-6, vals["wrist_radial_deg"])

# 4. The solve: one gun place, one girdle, the wrist at a ready reach (elbow about 25 degrees); the candidates are
# inside every wall and best first
W = np.array([0.0, 0.52, -0.10])
cands, n_all = BS.solve_body([(np.zeros(3), W, W + np.array([0, 0.08, 0]), W + np.array([0, 0.08, 0.02]),
                               W + np.array([0, 0.08, -0.02]))], [("rest", S, 0.0)], l1, l2, "r", L, right, fwd, 20.0)
check("the solve tried every swing", n_all == 91, n_all)
check("some candidates are inside the walls", len(cands) > 0, len(cands))
check("best first", all(cands[i]["cost"] <= cands[i + 1]["cost"] for i in range(len(cands) - 1)))
for c in cands[:5]:
    _v, p = BS.arm_walls(S, c["elbow"][None, :], W, W + np.array([0, 0.08, 0]), W + np.array([0, 0.08, 0.02]),
                         W + np.array([0, 0.08, -0.02]), "r", L, right, fwd, up)
    check("a kept candidate is inside every wall", all(v[0] <= 0.5 for v in p.values()), c)

if fails:
    print("BODY SOLVE failed: " + "; ".join(fails))
    sys.exit(1)
print("BODY SOLVE ok")
