"""Regression test for the checks: the built-in sample with one known fault at a time, each of which a check must
catch, and the idle grip, which the new checks must not newly fail. Runs inside Blender, on the lab itself:

    blender -b -P examples/anatomy_faults.py

The faults come from a real rig's benchmark (benchmarks/runs/2026-10-09-toangtown-m24): a finger kinked sideways at
its middle joint, a hand twisted against its forearm, a hand pushed into the rifle between the corner points of a flat
face, a hand with only a fingertip on the rifle, and a rigs file with a byte order mark.
"""
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "poselab_mcp", "blender"))
from mathutils import Matrix, Quaternion, Vector  # noqa: E402
from lab import Lab, ue  # noqa: E402

out = tempfile.mkdtemp(prefix="poselab-faults-")
L = Lab(None, out)
L.load_rig("sample")
fails = []


def check(label, ok, detail):
    print("%-4s %-58s %s" % ("ok" if ok else "FAIL", label, detail))
    if not ok:
        fails.append(label)


def has(bad, word):
    return any(word in b for b in bad)


# the idle grip: no wrist twist, and the finger and twist rules add nothing new to its known faults (its wrists)
a = L.anatomy()
for s in ("l", "r"):
    check("idle %s: no wrist twist" % s, a[s].get("wrist_twist_deg") == 0, a[s].get("wrist_twist_deg"))
    check("idle %s: no finger or twist rule broken" % s, not any("twist" in b or "finger" in b or "sideways" in b or "out of the finger" in b for b in a[s]["bad"]), a[s]["bad"])

# 1 a finger kinked: the left index's middle joint turned 35 degrees about the palm's normal
L.pose_idle()
f = lambda fi, j: L._bw(L._n("finger", "l", fi, j)).translation
across, k = L._hand_frame("l")
hn, mid = L._bw(L._n("hand", "l")).translation, f("middle", "01")
palm = (across.cross((mid - hn).normalized()) * k).normalized()
L._turn_about(L._n("finger", "l", "index", "02"), palm, 35.0, f("index", "02"))
a = L.anatomy(side="l")
check("finger kinked 35 at the middle joint", any(b.startswith("index") for b in a["l"]["bad"]),
      [b for b in a["l"]["bad"] if b.startswith("index")] or a["l"]["fingers"]["index_03"])

# 2 the hand twisted 150 degrees about its own line against the forearm
L.pose_idle()
hb = L._n("hand", "l")
line = (f("middle", "01") - L._bw(hb).translation).normalized()
L._turn_about(hb, line, 150.0, L._bw(hb).translation)
a = L.anatomy(side="l")
check("hand twisted 150 against the forearm", has(a["l"]["bad"], "twisted"), (a["l"].get("wrist_twist_deg"), a["l"]["bad"][-1:]))

# 3 reach and move_gun roll the forearm themselves: no twist after them
L.pose_idle()
L.move_gun(roll=60.0)
a = L.anatomy()
check("rifle rolled 60, hands kept: no wrist twist", all(a[s].get("wrist_twist_deg", 0) <= 1 for s in ("l", "r")),
      {s: a[s].get("wrist_twist_deg") for s in ("l", "r")})

# 4 a hand pushed into a flat face between its corner points: clearance reads the depth to the surface
L.pose_idle()
tree = L._tree()
hand_l = L._bw(hb).translation
loc, d = tree.nearest(hand_l)
L.reach("l", list(L._to(hand_l + (loc - hand_l).normalized() * (abs(d) + 0.01), "gun")), frame="gun")
c = L.clearance(parts="left")
loc2, d2 = tree.nearest(L._bw(hb).translation)
check("hand 1 cm inside a face: clearance sees it", c["worst_cm"] >= 1.0, (c["worst_cm"], round(d2 * 100, 2)))

# 5 only a fingertip on the rifle, the palm off it: touching, not holding; hold true breaks a rule
L.pose_idle()
tip = L._tip("index", "l")
loc, d = tree.nearest(tip)
away = (L._bw(hb).translation - loc).normalized()
for _ in range(8):     # the hand backed off along the line from the rifle until only the tip is near it
    L.reach("l", list(L._to(L._bw(hb).translation + away * 0.01, "gun")), frame="gun")
    g = L.grip(side="l")["l"]
    if g["palm_gap_cm"] is not None and g["palm_gap_cm"] > 2.0:
        break
L.reach("l", list(L._to(L._bw(hb).translation + (tree.nearest(L._tip("index", "l"))[0] - L._tip("index", "l")), "gun")), frame="gun")
g = L.grip(side="l", hold=True)["l"]
check("fingertip only: touching but not holding", g["touching"] and not g["holding"] and has(g["bad"], "does not hold"),
      {k_: g[k_] for k_ in ("touching", "holding", "palm_gap_cm", "gap_cm")})

# 7 the forearm's rotation (0.4.0): the forearm and hand turned together about the forearm's line read as that much
# more or less supination, the sign by side; past the AAOS range it is a rule
for s in ("l", "r"):
    L.pose_idle()
    r0 = L.anatomy(side=s)[s].get("forearm_rotation_deg")
    lo_, hn_ = L._n("lowerarm", s), L._n("hand", s)
    E_, W_ = L._bw(lo_).translation.copy(), L._bw(hn_).translation.copy()
    ax = (W_ - E_).normalized()
    H_ = L._bw(hn_).copy()
    L._turn_about(hn_, ax, 20.0, W_)
    r1 = L.anatomy(side=s)[s].get("forearm_rotation_deg")
    check("forearm %s turned 20: the rotation moves 20" % s, r0 is not None and r1 is not None and abs(abs(r1 - r0) - 20) <= 2,
          (r0, r1))
    L.pose_idle()
    d_ = 1.0 if s == "r" else -1.0
    want = 100.0 - (r0 or 0.0)          # to 100 degrees of supination
    L._turn_about(hn_, ax, want * d_, W_)
    a = L.anatomy(side=s)
    if a[s].get("forearm_rotation_deg") is not None and a[s]["forearm_rotation_deg"] < 0:
        L.pose_idle()
        L._turn_about(hn_, ax, -want * d_, W_)
        a = L.anatomy(side=s)
    check("forearm %s at 100 supinated: a rule" % s, has(a[s]["bad"], "supinated"), (a[s].get("forearm_rotation_deg"), a[s]["bad"][-1:]))

# 6 a rigs file with a byte order mark loads
path = os.path.join(out, "rigs.json")
with open(path, "w", encoding="utf-8-sig") as fh:
    json.dump({"bom": {"title": "a rigs file saved with a byte order mark"}}, fh)
L2 = Lab(path, out)
check("rigs.json with a byte order mark", "bom" in L2.list_rigs()["rigs"], L2.list_rigs()["rigs"])

print("FAULTS: %d failed" % len(fails))
sys.exit(1 if fails else 0)
