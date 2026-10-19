"""Known answers for the distance grids (sdflab): a cylinder 3.5 cm across and 12 cm long, its signed distance, its
outward normal, and the grid kept on disk read back the same.

    blender -b --factory-startup -P examples/sdf_tests.py
Prints SDF ok, or the failures and exits 1."""
import math
import os
import sys
import tempfile
import time

import bmesh
import bpy
import numpy as np
from mathutils import Matrix

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import sdflab as SD  # noqa: E402

fails = []


def check(label, ok, got=None):
    if not ok:
        fails.append("%s (got %s)" % (label, got))


bpy.ops.wm.read_factory_settings(use_empty=True)
bm = bmesh.new()
bmesh.ops.create_cone(bm, cap_ends=True, segments=48, radius1=0.0175, radius2=0.0175, depth=0.12)
bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
me = bpy.data.meshes.new("c")
bm.to_mesh(me)
bm.free()
ob = bpy.data.objects.new("c", me)
bpy.context.scene.collection.objects.link(ob)
ob.vertex_groups.new(name="Root").add(list(range(len(me.vertices))), 1.0, "REPLACE")
# the part's frame: moved and turned, so the world to frame step is tested too
F = Matrix.Translation((0.1, 0.2, 0.3)) @ Matrix.Rotation(math.radians(30.0), 4, "X")
ob.matrix_world = F
bpy.context.view_layer.update()


class Stub(SD.SdfMixin):
    rifle_mesh = ob
    out_dir = tempfile.mkdtemp()

    def _obj_frame(self, part):
        return F


s = Stub()
t0 = time.time()
s.object_sdf()
build_s = time.time() - t0
# points in the part's frame (cm), the distance wanted (cm) and the normal wanted
# (the axis is a ridge of the distance: a grid rounds its peak, by up to half a cell's diagonal, so it gets 0.15 cm)
cases = [((0, 0, 0), -1.75, None), ((3.0, 0, 0), 1.25, (1, 0, 0)), ((0, -2.5, 0), 0.75, (0, -1, 0)),
         ((1.5, 0, 0), -0.25, (1, 0, 0)), ((0, 0, 7.0), 1.0, (0, 0, 1)), ((0, 0, -5.5), -0.5, (0, 0, -1)),
         ((1.0, 1.0, 2.0), math.hypot(1, 1) - 1.75, (0.7071, 0.7071, 0))]
P = np.array([list(F @ __import__("mathutils").Vector([c / 100.0 for c in p])) for p, _d, _n in cases])
D, which, N = s.sdf_query(P)
R3 = np.array(F.to_3x3())
for (p, d, n), dg, ng in zip(cases, D, N):
    check("distance at %s" % (p,), abs(dg - d) < (0.15 if p == (0, 0, 0) else 0.08), round(float(dg), 3))
    if n is not None:
        nw = R3 @ np.array(n, dtype=float)
        check("normal at %s" % (p,), float(np.dot(ng, nw)) > 0.95, [round(float(v), 2) for v in ng])
check("the nearest part named", all(w == "Root" for w in which), set(which))
# read back from disk: the same numbers
s2 = Stub()
s2.out_dir = s.out_dir
t1 = time.time()
D2 = s2.sdf_query(P, want_normal=False)[0]
check("the grid read back from disk gives the same distances", np.allclose(D, D2), (D, D2))
check("reading it back is quick (under 1 s)", time.time() - t1 < 1.0, round(time.time() - t1, 2))
# a query for many points at once is cheap
Q = np.random.default_rng(0).uniform(-0.05, 0.05, (20000, 3)) + P[0]
t2 = time.time()
s.sdf_query(Q)
q_s = time.time() - t2
check("20000 points with normals in under 0.5 s", q_s < 0.5, round(q_s, 3))
# the grid's error near the surface (within 0.5 cm, where contact is judged) against the exact signed distance
from mathutils import Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402
bvh = BVHTree.FromPolygons([ob.matrix_world @ v.co for v in me.vertices], [tuple(p_.vertices) for p_ in me.polygons])
rng = np.random.default_rng(1)
loc_ = rng.uniform([-2.3, -2.3, -6.3], [2.3, 2.3, 6.3], (6000, 3))
W = np.array([list(F @ Vector(list(v / 100.0))) for v in loc_])
exact = []
for w in W:
    l_, n_, _i, d_ = bvh.find_nearest(Vector(w))
    r_ = math.hypot(*(F.inverted() @ Vector(w))[:2])
    zin = abs((F.inverted() @ Vector(w))[2]) < 0.06
    exact.append((-d_ if (r_ < 0.0175 * math.cos(math.pi / 48) and zin) else d_) * 100.0)
exact = np.array(exact)
near_s = np.abs(exact) < 0.5
Dg = s.sdf_query(W[near_s], want_normal=False)[0]
err_ = np.abs(Dg - exact[near_s])
print("SDF error within 0.5 cm of the surface: median %.3f, 99th %.3f, max %.3f cm" % (
    float(np.median(err_)), float(np.percentile(err_, 99)), float(err_.max())))
check("the grid's error near the surface, 99 of 100 points under SDF_TOL_CM", float(np.percentile(err_, 99)) <= SD.SDF_TOL_CM,
      round(float(np.percentile(err_, 99)), 3))
print("SDF build %.1f s, 20000 points %.3f s" % (build_s, q_s))
if fails:
    print("SDF failed: " + "; ".join(fails))
    sys.exit(1)
print("SDF ok")
