"""The hold benchmark: one hand holds a built shape with a known answer, by the whole hold stack (the object reader,
the contact model, the purposes, the search, the gate), and every number is written down.

Case "cylinder": a round handle 3.5 cm across and 12 cm long in front of the right hand, its top leaned forward
20 degrees as a hand holds a grip. A power wrap on it must
pass the gate: the palm and the finger pads on the handle, a share of its surface covered, nothing pressed in.

    blender -b --factory-startup -P examples/hold_bench.py -- --arms-folder <folder> --arms Arms.fbx
        --idle Arms_Idle.fbx --eye 0 -2.3 162.6 --out <folder> [--generations 30 --pop 12] [--bone-edits bone_edits.json]
The arms are yours (no arms ship with Pose Lab). Writes <out>/bench_cylinder.json and its contact pictures."""
import argparse
import json
import math
import os
import sys
import time

import bmesh
import bpy
from mathutils import Matrix, Quaternion, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "poselab_mcp", "blender"))
import lab as PL  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--arms-folder", required=True)
ap.add_argument("--arms", required=True)
ap.add_argument("--idle", required=True)
ap.add_argument("--eye", type=float, nargs=3, required=True)
ap.add_argument("--bone-edits", default=None)
ap.add_argument("--out", required=True)
ap.add_argument("--generations", type=int, default=30)
ap.add_argument("--pop", type=int, default=12)
ap.add_argument("--view-cm", type=float, nargs=3, default=(10.0, 28.0, -20.0))
ap.add_argument("--tilt-deg", type=float, default=20.0, help="the handle's top leaned forward, as a hand holds a grip")
a = ap.parse_args(sys.argv[sys.argv.index("--") + 1:])
os.makedirs(a.out, exist_ok=True)

# the handle: a cylinder 3.5 cm across, 12 cm long, on one bone
bpy.ops.wm.read_factory_settings(use_empty=True)
arm_d = bpy.data.armatures.new("Handle")
arm = bpy.data.objects.new("Handle", arm_d)
bpy.context.scene.collection.objects.link(arm)
bpy.context.view_layer.objects.active = arm
bpy.ops.object.mode_set(mode="EDIT")
b = arm_d.edit_bones.new("Root")
b.head, b.tail = (0, 0, 0), (0, 0.05, 0)
bpy.ops.object.mode_set(mode="OBJECT")
bm = bmesh.new()
bmesh.ops.create_cone(bm, cap_ends=True, segments=48, radius1=0.0175, radius2=0.0175, depth=0.12)
bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])            # faces out (a built mesh may come inside out)
me = bpy.data.meshes.new("handle")
bm.to_mesh(me)
ob = bpy.data.objects.new("handle", me)
bpy.context.scene.collection.objects.link(ob)
ob.parent = arm
vg = ob.vertex_groups.new(name="Root")
vg.add(list(range(len(me.vertices))), 1.0, "REPLACE")
mod = ob.modifiers.new("arm", "ARMATURE")
mod.object = arm
fbx = os.path.join(a.out, "bench_cylinder.fbx")
bpy.ops.export_scene.fbx(filepath=fbx, add_leaf_bones=False, bake_anim=False, object_types={"ARMATURE", "MESH"})
bpy.ops.wm.read_factory_settings(use_empty=True)

rig = {"bench": {"title": "Hold benchmark: a cylinder", "folder": a.arms_folder, "arms": a.arms, "idle": a.idle,
                 "gun": fbx, "gun_offset": [0.0, 0.0, 0.0], "eye": list(a.eye), "styled": False,
                 "fit_capsules": True, "thumb_hinge": "ray"}}
if a.bone_edits:
    rig["bench"]["bone_edits"] = a.bone_edits
rigs = os.path.join(a.out, "bench_rigs.json")
json.dump(rig, open(rigs, "w"), indent=1)
L = PL.Lab(rigs, a.out)
L.load_rig("bench")
# the handle leaned: its top forward by tilt_deg about the view's right (a grip, a joystick, a hammer held up)
lean = Quaternion(PL.VIEW_RIGHT, math.radians(-a.tilt_deg))
L.rifle_rest = Matrix.LocRotScale(L._from(list(a.view_cm), "view"), lean, L.rifle_rest.to_scale())
L.pose_idle()
bpy.context.view_layer.update()

out = {"case": "cylinder", "diameter_cm": 3.5, "length_cm": 12.0}
t0 = time.time()
obj = L.read_object(parts=["Root"])
out["reader"] = L.object_summary()
runs = obj["parts"]["Root"]["runs"]
if not runs:
    out["result"] = "the reader found no run"
    json.dump(out, open(os.path.join(a.out, "bench_cylinder.json"), "w"), indent=1, default=str)
    sys.exit(1)
marks = [{"part": "Root", "role": "hold", "purpose": "power_wrap", "run": 0, "hand": "r", "index": "wrap", "thumb": "wrap"}]
out["regions_cm2"] = L.object_regions(marks)
out["read_s"] = round(time.time() - t0, 2)

# the contact read's speed, as the search pays it each try
t1 = time.time()
for _ in range(5):
    L.contact_pairs("r", other=False, cover_regions=["back", "front", "side_a", "side_b", "end"])
out["contact_read_s"] = round((time.time() - t1) / 5.0, 3)

WRAP = {fi: "wrap" for fi in ("index", "middle", "ring", "pinky", "thumb")}
plan = {fi: {"pose": "wrap", "max_curl": [90.0, 100.0, 80.0]} for fi in WRAP if fi != "thumb"}
plan["thumb"] = {"pose": "wrap", "max_curl": [0.0, 55.0, 80.0]}
L.snapshot("save", "_seed")
tries = []
t2 = time.time()
for roll, flip in [(r_, f_) for f_ in (False, True) for r_ in range(0, 360, 30)]:
    L.snapshot("load", "_seed")
    sd = L.seed_hold("r", marks[0], palm_dir=None, roll_deg=float(roll), flip=flip)
    if os.environ.get("BENCH_DEBUG") and roll in (0, 210):
        RW = L._run_world(runs[0])
        ax = RW["axis"]
        pc = L._bw(L._n("hand", "r")).translation.lerp(L._bw(L._n("finger", "r", "middle", "01")).translation, 0.5)
        print("DBG seed", roll, sd, "pc_h", round((pc - RW["inner"]).dot(ax) * 100, 2), "run_len", runs[0]["length_cm"],
              "knuckle_line.ax", round(L._hand_frame("r")[0].dot(ax), 2), "palm.facing.toaxis", round(L._palm_facing("r").dot((RW["sections"][6][0] - pc).normalized()), 2))
    if os.environ.get("BENCH_DEBUG") and roll == 0:
        c0 = L.contact_pairs("r", other=False)
        print("DBG before settle press", c0["press_cm2"], c0["deepest_cm"])
    gap_ = L.settle_palm("r")
    if os.environ.get("BENCH_DEBUG") and roll == 0:
        c1 = L.contact_pairs("r", other=False)
        print("DBG after settle press", c1["press_cm2"], c1["deepest_cm"], gap_)
        import numpy as _np
        P_, N_, A_, _b_, _t_ = L._skin_arrays()
        ix, lb = L._hand_label_arrays("r", P_, N_)
        bvh_, surf_ = L._object_now()[:2]
        cnt = {}
        for i_, l_ in zip(ix, lb):
            if not l_.startswith("palm"):
                continue
            p_ = Vector(P_[i_])
            loc, nrm, _k, d = bvh_.find_nearest(p_, 0.05)
            if loc is None:
                continue
            key = (l_, "normal_in" if (p_ - loc).dot(nrm) < 0 else "normal_out", "parity_in" if surf_.inside(p_) else "parity_out")
            cnt[key] = cnt.get(key, 0) + 1
        print("DBG palm classes", sorted(cnt.items(), key=lambda z: -z[1])[:8])
        shown = 0
        for i_, l_ in zip(ix, lb):
            p_ = Vector(P_[i_])
            loc, nrm, _k, d = bvh_.find_nearest(p_, 0.01)
            if loc is None:
                continue
            nin = (p_ - loc).dot(nrm) < 0.0 and d < 0.02
            if nin and surf_.inside(p_):
                print("DBG pressed", l_, round(d * 100, 2), "loc", [round(v * 100, 1) for v in loc], "p", [round(v * 100, 1) for v in p_])
                shown += 1
                if shown >= 5:
                    break
        print("DBG pressed count shown", shown)
        cyl = L.rifle_mesh
        print("DBG cylinder mesh", cyl.name, len(cyl.data.vertices), "world bbox", [round(v * 100, 1) for v in (cyl.matrix_world @ Vector(cyl.bound_box[0]))], [round(v * 100, 1) for v in (cyl.matrix_world @ Vector(cyl.bound_box[6]))], "scale", [round(s_, 3) for s_ in cyl.matrix_world.to_scale()])
    if os.environ.get("BENCH_DEBUG") and roll in (0, 210):
        pc = L._bw(L._n("hand", "r")).translation.lerp(L._bw(L._n("finger", "r", "middle", "01")).translation, 0.5)
        print("DBG settled", roll, gap_, "pc_h", round((pc - RW["inner"]).dot(ax) * 100, 2))
    for fi in WRAP:
        if fi == "thumb":
            continue
        for j in ("01", "02", "03"):
            L._set_curl("r", fi, j, 0.0, L._limits()[1])
    L.close_hand("r", plan, L._tree(), 0.0, False, mend=False)
    L.relax_fingers("r", tuple(plan))
    sc, why = L._hold_score("r", marks, 0.0, False, None, L._tree())
    if os.environ.get("BENCH_DEBUG") and roll == 0:
        cp_ = L.contact_pairs("r", other=False)
        pr_ = sorted(((k, round(v[1], 2), round(v[2], 2)) for k, v in cp_["pairs"].items() if v[1] > 0), key=lambda z: -z[1])
        print("DBG press", cp_["press_cm2"], cp_["deepest_cm"], pr_[:8], "settle", L.settle_palm("r"))
    if os.environ.get("BENCH_DEBUG"):
        print("DBG score", roll, round(sc, 1), [(p_["purpose"], p_["error"], {k: p_.get(k) for k in ("palm_cm2", "fingers_front_cm2", "cover", "pull_cm", "wrist_deg")}) for p_ in why["purposes"]])
    # a seed that breaks an arm rule (a forearm twisted past its range) is no start: the body first
    rep__, bad_ = L._arm_report("r", L._limits()[0])
    if os.environ.get("BENCH_DEBUG"):
        RW_ = L._run_world(runs[0])
        print("DBG arm", roll, flip, "roll", rep__.get("forearm_rotation_deg"), "wrist", rep__.get("wrist_bend_deg"), "elbow", rep__.get("elbow_bend_deg"),
              "index_up", round((L._bw(L._n("finger", "r", "index", "01")).translation - L._bw(L._n("finger", "r", "pinky", "01")).translation).normalized().z, 2),
              "palm_facing", [round(v, 2) for v in L._palm_facing("r")], "bad", bad_)
    # the starts ranked by how far past its limits the arm is (the body first), then by the score
    ex_ = L.arm_excess("r")
    tries.append((sc, (roll, flip), ex_))
    if (ex_, sc) <= min((t_[2], t_[0]) for t_ in tries):
        L.snapshot("save", "_best")
L.snapshot("load", "_best")
out["seeds"] = sorted([{"roll_deg": r[0], "flip": r[1], "score": round(s_, 1), "arm_excess_deg": e_} for s_, r, e_ in tries],
                     key=lambda x: (x["arm_excess_deg"], x["score"]))
out["seed_s"] = round(time.time() - t2, 1)
t3 = time.time()
out["hold"] = L.hold_object("r", marks, glove_cm=0.0, other=False, generations=a.generations, pop=a.pop, wrap=WRAP)
out["search_s"] = round(time.time() - t3, 1)
out["gate"] = L.hold_gate(marks, sides=("r",), glove_cm=0.0, skin=False)
cm = L.contact_map(("r",), marks=marks)
out["contact"] = {"hands": cm["hands"], "regions": {k: v for k, v in cm["regions"].items() if not k.startswith("other")},
                  "pairs": cm["pairs"][:12]}
centre = L._obj_frame("Root") @ Vector(runs[0]["sections"][len(runs[0]["sections"]) // 2]["centre"])
out["pictures"] = L.contact_pictures(os.path.join(a.out, "bench_cylinder"), centre=list(centre), frame_part="Root",
                                     kinds=("hand", "heat", "contact"), ortho_cm=24.0)
json.dump(out, open(os.path.join(a.out, "bench_cylinder.json"), "w"), indent=1, default=str)
print("BENCH cylinder gate %s, try %.3f s, search %.0f s" % (out["gate"]["pass"], out["hold"]["try_s"], out["search_s"]))
