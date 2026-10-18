"""Known-answer tests for the geometry every hold score stands on: a capsule's skin to a point, two segments'
closest distance, and the signed distance to a mesh (inside is negative, found by ray parity). They run in seconds,
before any clip, so a broken measure shows here and not as a hold that looks wrong.

    blender -b --factory-startup -P examples/geometry_tests.py
Prints GEOMETRY ok, or the failures and exits 1."""
import os
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import lab as LAB  # noqa: E402

Lab = LAB.Lab
fails = []


def near(label, got, want, tol):
    if got is None or abs(got - want) > tol:
        fails.append("%s: got %s, want %s (+-%s)" % (label, got, want, tol))


V = Vector
# a capsule along x, 10 cm long, radius 1 cm
a, b, r = V((0, 0, 0)), V((0.1, 0, 0)), 0.01
near("capsule skin to a point 3 cm above its middle", Lab._seg_point_cm(a, b, r, V((0.05, 0, 0.03))), 2.0, 1e-5)
near("capsule skin to a point past its end", Lab._seg_point_cm(a, b, r, V((0.13, 0, 0.04))), 4.0, 1e-5)
near("a point inside the capsule reads below 0", Lab._seg_point_cm(a, b, r, V((0.05, 0, 0.004))), -0.6, 1e-5)
near("parallel segments 2 cm apart", Lab._seg_seg(a, b, V((0, 0.02, 0)), V((0.1, 0.02, 0))), 0.02, 1e-7)
near("crossing segments", Lab._seg_seg(a, b, V((0.05, -0.05, 0)), V((0.05, 0.05, 0))), 0.0, 1e-7)
near("skew segments 3 cm apart", Lab._seg_seg(a, b, V((0.05, -0.05, 0.03)), V((0.05, 0.05, 0.03))), 0.03, 1e-7)
near("a point segment", Lab._seg_seg(V((0, 0, 0)), V((0, 0, 0)), V((0, 0.05, 0)), V((0, 0.05, 0))), 0.05, 1e-7)

# a 6 cm cube centred at the origin, as a mesh's surface
h = 0.03
vs = [V((x, y, z)) for x in (-h, h) for y in (-h, h) for z in (-h, h)]
faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
S = Lab.Surface(BVHTree.FromPolygons(vs, faces))
_q, d = S.nearest(V((0.0, 0.0, 0.04)))
near("1 cm above the top face", d * 100.0, 1.0, 1e-5)
_q, d = S.nearest(V((0.0, 0.0, 0.01)))
near("2 cm inside under the top face reads -2", d * 100.0, -2.0, 1e-5)
_q, d = S.nearest(V((0.029, 0.0, 0.0)))
near("1 mm inside a side face", d * 100.0, -0.1, 1e-5)
_q, d = S.nearest(V((0.031, 0.031, 0.0)))
near("past an edge, outside", d * 100.0, (2 ** 0.5) * 0.1, 1e-5)
if not S.inside(V((0.0, 0.0, 0.0))):
    fails.append("the cube's centre is not inside")
if S.inside(V((0.0, 0.0, 0.05))):
    fails.append("a point above the cube reads inside")

if fails:
    print("GEOMETRY failed:\n" + "\n".join(fails))
    sys.exit(1)
print("GEOMETRY ok (%d checks)" % 13)
