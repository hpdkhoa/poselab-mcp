"""Known answers for the clipping rule (contactlab): skin inside an object is found from the skin's side, and an
object inside the flesh is found from the object's side, so a thin part passing clean through a finger between its
skin vertices (a trigger guard's ring) is still caught.

    blender -b --factory-startup -P examples/clip_tests.py
Prints CLIP ok, or the failures and exits 1."""
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import contactlab as CL  # noqa: E402

fails = []


def check(label, ok, got=None):
    if not ok:
        fails.append("%s (got %s)" % (label, got))


def box_bvh(centre, size, segs=1):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(centre) @ Matrix.Diagonal((size[0], size[1], size[2], 1.0)))
    if segs > 1:
        bmesh.ops.subdivide_edges(bm, edges=bm.edges[:], cuts=segs - 1, use_grid_fill=True)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    vs = [v.co.copy() for v in bm.verts]
    fs = [[v.index for v in f.verts] for f in bm.faces]
    bm.free()
    return BVHTree.FromPolygons(vs, fs), vs


# a "finger": a closed box 1.6 x 1.6 x 6 cm, its skin vertices only at its corners (a coarse mesh)
finger, fverts = box_bvh(Vector((0, 0, 0)), (0.016, 0.016, 0.06))
# a thin "ring" 3 mm thick passing through the finger's middle: none of the finger's vertices lie inside the ring
ring, rverts = box_bvh(Vector((0, 0, 0)), (0.04, 0.04, 0.003), segs=8)
vertex_inside = [v for v in fverts if ring.find_nearest(v)[0] is not None and (v - ring.find_nearest(v)[0]).dot(ring.find_nearest(v)[1]) < 0]
check("the coarse finger's vertices miss the ring (the old test's blind spot)", not vertex_inside, len(vertex_inside))
# the ring's own surface samples inside the finger's flesh, found from the ring's side
hits = [CL.ContactMixin.inside_flesh(v, finger) for v in rverts]
deep = max((h[0] for h in hits if h is not None), default=0.0)
check("the ring is found inside the finger's flesh", any(h is not None for h in hits), sum(1 for h in hits if h))
check("its depth reads past a pad's give (0.15 cm)", deep > CL.SKIN_GIVE["pad"], round(deep, 2))
# a ring beside the finger, 1 cm off: nothing inside
ring2, rverts2 = box_bvh(Vector((0.03, 0, 0)), (0.02, 0.02, 0.003), segs=8)
hits2 = [CL.ContactMixin.inside_flesh(v, finger) for v in rverts2]
check("a ring beside the finger reads outside", not any(hits2), sum(1 for h in hits2 if h))
# a point just under the finger's skin (1 mm) reads 0.1 cm deep
h3 = CL.ContactMixin.inside_flesh(Vector((0.007, 0, 0)), finger)
check("a point 1 mm under the skin reads 0.1 cm", h3 is not None and abs(h3[0] - 0.1) < 0.01, h3)
# the holding-digit limits are tighter than a free hand's
check("a holding knuckle may not bend back past 5 degrees", CL.HOLD_DIGIT_LIMITS["01"][0] >= -5.0, CL.HOLD_DIGIT_LIMITS["01"])
check("a holding middle joint may not bend back at all", CL.HOLD_DIGIT_LIMITS["02"][0] >= 0.0, CL.HOLD_DIGIT_LIMITS["02"])

if fails:
    print("CLIP failed: " + "; ".join(fails))
    sys.exit(1)
print("CLIP ok")
