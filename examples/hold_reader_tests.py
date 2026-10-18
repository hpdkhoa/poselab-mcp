"""Known answers for the object reader (holdlab.py): shapes built in code, read the same way any object is.

1. A cylinder 3.5 cm across and 12 cm long: one run a hand can wrap, 3.5 cm across, along the cylinder's axis.
2. A block shaped like a pistol (a grip leaning 18 degrees, a frame on top of it, a guard ring in front): the grip's
   run, leaning 18 degrees, its inner end at the top; an opening (the guard) with its upper edge under the frame.
3. A box 9 cm square: nothing a hand can wrap.

    blender -b --factory-startup -P examples/hold_reader_tests.py
Prints READER ok, or the failures and exits 1."""
import math
import os
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "poselab_mcp", "blender"))
import holdlab as HL  # noqa: E402

fails = []


def check(label, ok, got=None):
    if not ok:
        fails.append("%s (got %s)" % (label, got))


class Obj(HL.HoldMixin):
    """Just enough of a Lab for the reader: the object to read."""
    def __init__(self, ob):
        self.rifle_mesh = ob
        self.rifle_arm = None

    def _need(self):
        pass


def fresh():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def mesh_from(bm, name, group="body"):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    vg = ob.vertex_groups.new(name=group)
    vg.add(list(range(len(me.vertices))), 1.0, "REPLACE")
    bpy.context.view_layer.update()
    return ob


def box(bm, centre, size, rot=None):
    m = Matrix.Translation(centre) @ (rot or Matrix.Identity(4)) @ Matrix.Diagonal((size[0], size[1], size[2], 1.0))
    bmesh.ops.create_cube(bm, size=1.0, matrix=m)


# 1. the cylinder
fresh()
bm = bmesh.new()
bmesh.ops.create_cone(bm, cap_ends=True, segments=32, radius1=0.0175, radius2=0.0175, depth=0.12)
r = Obj(mesh_from(bm, "cyl")).read_object(hand_length_cm=19.0)
runs = r["parts"]["body"]["runs"]
check("cylinder: a run", len(runs) >= 1, len(runs))
if runs:
    check("cylinder: 3.5 cm across", abs(runs[0]["diameter_cm"] - 3.5) <= 0.2, runs[0]["diameter_cm"])
    check("cylinder: along its axis", abs(runs[0]["axis"][2]) >= math.cos(math.radians(9)), runs[0]["axis"])
    check("cylinder: most of its length", runs[0]["length_cm"] >= 10.0, runs[0]["length_cm"])

# 2. the pistol-like block (x left, y forward, z up, cm): a grip 3 x 5 cm leaning back 18 degrees, the frame over it,
# a guard ring in front of the grip under the frame
fresh()
bm = bmesh.new()
lean = Matrix.Rotation(math.radians(18.0), 4, "X")         # top tips forward: the grip leans back from the top
box(bm, Vector((0, -0.01, -0.065)), (0.03, 0.05, 0.11), lean)
box(bm, Vector((0, 0.025, -0.005)), (0.03, 0.20, 0.03))      # the frame, its underside at z -2 cm, out past the grip's back (a tang)
box(bm, Vector((0, 0.05, -0.045)), (0.012, 0.05, 0.006))     # the guard's bottom bar, z -4.5 cm, meeting the grip
box(bm, Vector((0, 0.072, -0.0325)), (0.012, 0.006, 0.025))  # the guard's front bar
bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
ob = mesh_from(bm, "pistol", "frame")
L = Obj(ob)
r = L.read_object(hand_length_cm=19.0, inner_toward=[0.0, 0.04, 0.0])
runs = r["parts"]["frame"]["runs"]
check("pistol: a run", len(runs) >= 1, len(runs))
# the frame's bar is a handle too (3 x 3 cm): the grip is the run that leans, as the rig's grip mark would pick it
grips = [r_ for r_ in runs if abs(r_["axis"][2]) > 0.8]
check("pistol: the grip among the runs", grips, [(r_["axis"], r_["length_cm"]) for r_ in runs])
if grips:
    g = grips[0]
    check("pistol: the grip's run below the guard (6 cm or more)", g["length_cm"] >= 6.0, g["length_cm"])
    lean_deg = math.degrees(math.acos(min(1.0, abs(g["axis"][2]))))
    check("pistol: the grip leans 18 degrees", abs(lean_deg - 18.0) <= 6.0, round(lean_deg, 1))
    check("pistol: the grip's girth (3 x 5 cm, about 5 cm across)", 4.2 <= g["diameter_cm"] <= 5.6, g["diameter_cm"])
    check("pistol: inner end at the top", g["inner"][2] > g["outer"][2], (g["inner"], g["outer"]))
    ops = g["openings"]
    check("pistol: the guard's opening", len(ops) >= 1, len(ops))
    if ops:
        top = max(ops[0]["edge"][0][2], ops[0]["edge"][1][2])
        check("pistol: the opening's upper edge under the frame (z -2 cm)", -0.028 <= top <= -0.012, round(top * 100, 2))
        bot = min(ops[0]["edge"][0][2], ops[0]["edge"][1][2])
        check("pistol: the edge runs forward, level", abs(top - bot) <= 0.006, (round(top * 100, 2), round(bot * 100, 2)))
    mk = {"part": "frame", "run": runs.index(g)}
    wp, past = L.web_point(mk, [0.0, -1.0, 0.0])
    check("pistol: the web under the frame on the back (z -2 to -3 cm)", -0.031 <= wp[2] <= -0.017 and wp[1] < 0.0, (round(wp[1] * 100, 2), round(wp[2] * 100, 2), past))
    sug = L.suggest_marks()
    check("pistol: a power wrap suggested on the frame", sug and sug[0]["purpose"] == "power_wrap" and sug[0]["part"] == "frame", sug[:1])

# 3. a 9 cm box: too big to wrap
fresh()
bm = bmesh.new()
box(bm, Vector((0, 0, 0)), (0.09, 0.09, 0.09))
r = Obj(mesh_from(bm, "crate")).read_object(hand_length_cm=19.0)
check("box: nothing to wrap", not r["parts"]["body"]["runs"], [x["diameter_cm"] for x in r["parts"]["body"]["runs"]])

if fails:
    print("READER failed:\n" + "\n".join(fails))
    sys.exit(1)
print("READER ok")
