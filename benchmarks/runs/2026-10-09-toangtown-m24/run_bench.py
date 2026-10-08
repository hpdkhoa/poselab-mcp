"""The ToangTown M24 magazine-hold benchmark: Pose Lab's anatomy, grip and clearance on 8 poses (one good, seven with a
known fault), timed. The rig is ToangTown's (licensed, not in this repository): fill in rigs.toangtown-m24.json's
folder and point POSELAB_RIGS at it.

    POSELAB_RIGS=.../rigs.toangtown-m24.json POSELAB_BLENDER=... python benchmarks/runs/2026-10-09-toangtown-m24/run_bench.py [out.json]
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from mcp_stdio import PoseLab

TAGS = ["good", "back", "thumb", "twist", "finger", "clip", "elbow", "wrist"]
out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "poselab_now.json")
lab = PoseLab()
t0 = time.time()
lab.data("load_rig", rig="m24")
out = {"load_seconds": round(time.time() - t0, 2), "poses": {}}
for t in TAGS:
    t1 = time.time()
    lab.data("pose_clip", clip=t, seconds=0.0)
    a = lab.data("anatomy", side="l")
    g = lab.data("grip", side="l")
    c = lab.data("clearance", parts="left")
    out["poses"][t] = {"seconds": round(time.time() - t1, 2), "anatomy_bad": a["l"]["bad"], "wrist": a["l"].get("wrist_bend_deg"),
                       "wrist_twist": a["l"].get("wrist_twist_deg"), "grip_ok": g["ok"], "grip": g["l"], "clearance_cm": c["worst_cm"]}
    print("%-6s %.2f s | anatomy %s | grip ok %s holding %s palm %s gap %s | clearance %s cm" % (
        t, out["poses"][t]["seconds"], a["l"]["bad"], g["ok"], g["l"].get("holding"), g["l"].get("palm_faces_rifle"),
        g["l"].get("palm_gap_cm"), c["worst_cm"]))
lab.close()
json.dump(out, open(out_path, "w"), indent=1)
print("load %.2f s; written %s" % (out["load_seconds"], out_path))
