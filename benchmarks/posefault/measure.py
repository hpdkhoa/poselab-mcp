"""PoseFault arm C: Pose Lab alone, no model. Each pose from generate.py goes through anatomy, grip and clearance on
its side; the result says whether Pose Lab flagged the side at all (detection) and whether a flag names the injected
fault (localization). Runs inside Blender:

    blender -b --factory-startup -P benchmarks/posefault/measure.py -- <rigs file or ""> <generate.py output folder>

Writes <folder>/arm_c.jsonl (one line per pose) and prints a table per fault.
"""
import json
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src", "poselab_mcp", "blender"))
from lab import Lab  # noqa: E402

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
RIGS = ARGS[0] if ARGS and ARGS[0] else None
DIR = os.path.realpath(ARGS[1])

# A flag names the fault when its text has one of these words (all lower case).
NAMES = {
    "wrist_bend": ("wrist bent", "wrist past"),
    "wrist_twist": ("wrist twisted", "wrist link rolled", "forearm supinated", "forearm pronated"),
    "back_of_hand": ("back of the hand", "inside the back", "wrist twisted", "wrist link rolled", "forearm supinated", "forearm pronated"),
    "index_side": ("index middle joint twisted", "index end joint twisted", "index knuckle twisted"),
    "middle_side": ("middle middle joint twisted", "middle end joint twisted", "middle knuckle twisted"),
    "index_hyper": ("index middle joint bent",),
    "middle_hyper": ("middle middle joint bent",),
    "palm_clip": ("inside the palm side", "inside the back", "inside the rifle"),
    "elbow_high": ("elbow",),
}


def flags(L, s):
    a = L.anatomy(side=s)
    g = L.grip(side=s)
    c = L.clearance(parts=s, ignore=("index", "middle", "ring", "pinky", "thumb", "palm"))
    out = list(a[s]["bad"]) + list(g[s]["bad"])
    if c["worst_cm"] > 0.1:
        out.append("forearm %.2f cm inside the rifle" % c["worst_cm"])
    return out


def main():
    labels = [json.loads(x) for x in open(os.path.join(DIR, "labels.jsonl"))]
    L = Lab(RIGS, os.path.join(DIR, "measure_out"))
    rows = []
    loaded = None
    for lab_ in labels:
        if lab_["rig"] != loaded:
            L.load_rig(lab_["rig"])
            loaded = lab_["rig"]
        with open(os.path.join(DIR, "poses", lab_["id"] + ".pose.json")) as fh:
            L._apply(json.load(fh)["frames"][0])
        bad = flags(L, lab_["side"])
        named = bool(lab_["faulty"]) and any(w in b.lower() for b in bad for w in NAMES[lab_["fault"]])
        rows.append({"id": lab_["id"], "fault": lab_["fault"], "faulty": lab_["faulty"], "size": lab_["size"],
                     "flagged": bool(bad), "named": named, "bad": bad})
    with open(os.path.join(DIR, "arm_c.jsonl"), "w") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
    by = defaultdict(lambda: [0, 0, 0])
    for r in rows:
        k = r["fault"] if r["faulty"] else ("clean" if r["fault"] is None else "benign:" + r["fault"])
        by[k][0] += 1
        by[k][1] += r["flagged"]
        by[k][2] += r["named"]
    print("POSEFAULT %-16s %5s %8s %6s" % ("fault", "poses", "flagged", "named"))
    for k in sorted(by):
        print("POSEFAULT %-16s %5d %8d %6d" % (k, *by[k]))


main()
