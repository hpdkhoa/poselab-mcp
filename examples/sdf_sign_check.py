"""The distance grids' inside-or-out against the object's own mesh: points near the surface, each read by the grids
(sdf_query) and by the exact check (shell by shell: each closed shell's nearest face's distance, inside it by ray
parity in six directions, the majority; the union's is the least), and every disagreement counted.

    blender -b --factory-startup -P examples/sdf_sign_check.py -- --rigs <rigs.json> --rig <name> --out <folder>
        [--points 6000] [--seed 1]
Prints the counts and writes <out>/sdf_sign_check.json. Pass: no point more than the grids' tolerance (0.03 cm)
from the surface reads the wrong side."""
import argparse
import json
import os
import sys
import time

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "poselab_mcp", "blender"))
import lab as PL  # noqa: E402
from sdflab import SDF_TOL_CM, SDF_VERSION, shell_signed, shells  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--rigs", required=True)
ap.add_argument("--rig", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--points", type=int, default=6000)
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args(sys.argv[sys.argv.index("--") + 1:])
os.makedirs(a.out, exist_ok=True)
L = PL.Lab(a.rigs, a.out)
L.load_rig(a.rig)
L.pose_idle()
bpy.context.view_layer.update()
t0 = time.time()
L.object_sdf()
build_s = time.time() - t0
vs, ps = L._gun_verts()
bvh = BVHTree.FromPolygons(vs, ps)
V = np.array([list(v) for v in vs])
rng = np.random.default_rng(a.seed)
pick = rng.integers(0, len(V), a.points)
dirs = rng.normal(size=(a.points, 3))
dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
X = V[pick] + dirs * rng.uniform(0.0, 0.01, a.points)[:, None]
shell_bvhs = [BVHTree.FromPolygons(vs, [ps[i] for i in sh]) for sh in shells(ps)]


def exact(p):
    return shell_signed(shell_bvhs, p) * 100.0


E = np.array([exact(p) for p in X])
G = np.asarray(L.sdf_query(X, want_normal=False)[0], dtype=float)
clear = np.abs(E) > SDF_TOL_CM
wrong = clear & (np.sign(E) != np.sign(G))
res = {"grid_version": SDF_VERSION, "points": int(a.points), "beyond_tolerance": int(clear.sum()),
       "wrong_side": int(wrong.sum()), "wrong_side_share": round(float(wrong.sum()) / max(1, int(clear.sum())), 4),
       "max_abs_difference_cm": round(float(np.max(np.abs(E - G))), 3),
       "max_abs_difference_right_side_cm": round(float(np.max(np.abs(E - G)[clear & ~wrong])), 3) if (clear & ~wrong).any() else 0.0,
       "shells": len(shell_bvhs), "grid_build_s": round(build_s, 1), "pass": bool(wrong.sum() == 0)}
json.dump(res, open(os.path.join(a.out, "sdf_sign_check.json"), "w"), indent=1)
print("SDF SIGN CHECK", json.dumps(res))
