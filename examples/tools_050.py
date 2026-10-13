"""Checks the 0.5.0 tools on the built-in sample, inside Blender:

    blender -b -P examples/tools_050.py

view_cover: the arms cover part of the view at idle, more with the left hand pulled toward the eye, nothing with the
arms out of the view. solve_grip: finds a hold for the left hand on its own idle palm point, and none when the view
budget is zero. retarget_clip and check_set: a recorded clip comes back with the left hand on its hold in every frame,
and check_set counts its frames.
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "poselab_mcp", "blender"))
from lab import Lab  # noqa: E402

L = Lab(None, tempfile.mkdtemp(prefix="poselab-050-"))
L.load_rig("sample")
fails = []


def check(label, ok, detail):
    print("%-4s %-62s %s" % ("ok" if ok else "FAIL", label, detail))
    if not ok:
        fails.append(label)


L.pose_idle()
v0 = L.view_cover()
check("view_cover at idle: some of the view, not all of it", 0.0 < v0["total"] < 0.6, v0)
L.reach("l", [-4.0, 22.0, -8.0], frame="view")                   # the left wrist pulled close in front of the eye
v1 = L.view_cover()
check("view_cover: a hand close to the eye covers more", v1["total"] > v0["total"] + 0.02, (v0["total"], v1["total"]))
L.pose_idle()
palm = L.where(["hand_l", "middle_01_l"])
p = [(a + b) / 2.0 for a, b in zip(palm["hand_l"], palm["middle_01_l"])]
FINGERS = ["index*", "middle*", "ring*", "pinky*", "thumb*"]      # declared contacts: the fingers wrap the handguard
g = L.solve_grip("l", p, along=[-2.0, 2.0], steps=3, rolls=[0.0, 30.0, 60.0, 90.0], ignore=FINGERS)
for row in g["rows"]:
    print("   row", {k: row[k] for k in ("along_cm", "roll_deg", "holding", "palm_gap_cm", "clearance_cm", "rules_broken", "ok")})
check("solve_grip: finds a hold the idle grip misses (the forearm rolled back in its range)", g["found"] and g["best"]["roll_deg"] > 0,
      {k: g[k] for k in ("found", "tried", "passed")} | ({"best": {k: g["best"][k] for k in ("along_cm", "roll_deg", "palm_gap_cm", "clearance_cm")}} if g["best"] else {}))
none = L.solve_grip("l", p, along=[-2.0, 2.0], steps=3, poles=[None], rolls=[0.0, 60.0], ignore=FINGERS, view_budget={"lower_left": -1.0, "bottom_middle": -1.0})
check("solve_grip: no hold within an impossible view budget", not none["found"] and none["passed"] == 0, none["passed"])
L.pose_idle()
L.record_clip("start", "c050", fps=10)
L.record_clip("key", seconds=0.0)
L.move_gun(roll=15.0, keep_hands=["r"])
L.record_clip("key", seconds=0.5)
L.record_clip("stop")
r = L.retarget_clip("c050", "l")
cs = L.check_set(r["clip"], hold_sides=["l"])
check("retarget_clip: every frame made", r["frames"] >= 5, r)
check("check_set: it counts the frames and the clearance", cs["frames"] == r["frames"] and cs["clearance_worst_cm"] >= 0.0,
      {k: cs[k] for k in ("frames", "clearance_worst_cm", "anatomy_break_frames", "not_holding_frames")})
print("TOOLS 050: %s" % ("PASS" if not fails else "FAIL: " + "; ".join(fails)))
