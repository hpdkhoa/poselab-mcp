"""PoseFault: poses with one known fault each, made from clean base poses, for the paper "Measure, don't judge". Runs
inside Blender, on Pose Lab's lab itself:

    blender -b --factory-startup -P benchmarks/posefault/generate.py -- <rigs file or ""> <output folder> [--no-render]

For each base (a rig, a clip or the idle grip, a time, a side), the script first measures the base with Pose Lab. A
side that already breaks a rule is not used as a base: its report goes to bases.jsonl with used false. On each clean
side it writes the base itself (a clean control), a few small changes that stay inside every limit (benign controls)
and one pose per fault and size. The label of each pose comes from what the script did to it, never from a Pose Lab
reading, so the labels do not depend on the tool under test. labels.jsonl holds one line per pose; poses/<id>.pose.json
holds its bones; poses/<id>_<view>.png holds the renders a VLM judge sees.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src", "poselab_mcp", "blender"))
from mathutils import Matrix, Vector  # noqa: E402
from lab import Lab, ue  # noqa: E402

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
RIGS = ARGS[0] if ARGS and ARGS[0] else None
OUT = os.path.realpath(ARGS[1] if len(ARGS) > 1 else os.path.join(HERE, "out"))
RENDER = "--no-render" not in ARGS
VIEWS = ("eye", "left", "right", "top")

# The bases: (rig, clip or "idle", seconds, sides). A rig missing from the rigs file is skipped.
BASES = [
    ("sample", "idle", 0.0, ("l", "r")),
    ("ar", "idle", 0.0, ("l", "r")),
    ("ar", "check", 0.5, ("l", "r")),
    ("ar", "charge", 0.3, ("l", "r")),
    ("ak", "idle", 0.0, ("l", "r")),
    ("ak", "check", 0.5, ("l", "r")),
    ("ak", "charge", 0.3, ("l", "r")),
    ("m24", "good", 0.0, ("l",)),
]

# The faults and their sizes. kind: the rule family a correct finding names (anatomy, grip, clearance). Each size sits
# clearly past the working limit it breaks (wrist 30 off the forearm, twist 30, a middle joint 15 out of plane and 5
# backward, the palm side 1 cm into the rifle), so a base near its limit still ends past it.
FAULTS = [
    ("wrist_bend", "anatomy", (45.0, 60.0, 80.0)),
    ("wrist_twist", "anatomy", (60.0, 100.0, 150.0)),
    ("back_of_hand", "grip", (180.0,)),
    ("index_side", "anatomy", (30.0, 45.0, 60.0)),
    ("middle_side", "anatomy", (30.0, 45.0, 60.0)),
    ("index_hyper", "anatomy", (25.0, 45.0)),
    ("middle_hyper", "anatomy", (25.0, 45.0)),
    ("palm_clip", "clearance", (2.0, 3.0, 4.0)),
    ("elbow_high", "anatomy", (1.0,)),
]
# Small changes inside every limit: a finding on these is a false alarm.
BENIGN = [("wrist_nudge", 8.0), ("index_curl", 8.0), ("hand_back", 0.3)]


def clean_side(L, s):
    a = L.anatomy(side=s)
    g = L.grip(side=s)
    c = L.clearance(parts=s, ignore=("index", "middle", "ring", "pinky", "thumb", "palm"))
    bad = list(a[s]["bad"]) + list(g[s]["bad"]) + (["forearm %.2f cm inside the rifle" % c["worst_cm"]] if c["worst_cm"] > 0.1 else [])
    return bad, {"anatomy": a[s], "grip": g[s], "forearm_clearance": c}


def clean_up(L, s, max_pull_cm=2.0):
    """A base from a game pose that breaks rules: the hand's twist moved into the forearm, the wrist and fingers turned
    back inside their ranges (Pose Lab's own mend), then the hand pulled off the rifle 0.25 cm at a time, at most
    max_pull_cm, until the side passes. Returns (pull in cm, the rules still broken)."""
    L._roll_forearm(s)
    L._mend_hand(s)
    L._place_rifle()
    bad, _ = clean_side(L, s)
    pull = 0.0
    while bad and pull < max_pull_cm - 1e-9:
        inject(L, s, "hand_back", 0.25)
        L._mend_hand(s)
        L._place_rifle()
        pull += 0.25
        bad, _ = clean_side(L, s)
    return pull, bad


def axes(L, s):
    """The hand's frame in world space: wrist, knuckle line (across), long axis, palm normal; the forearm's line."""
    W = L._bw(L._n("hand", s)).translation
    E = L._bw(L._n("lowerarm", s)).translation
    K = L._bw(L._n("finger", s, "middle", "01")).translation
    across, k = L._hand_frame(s)
    long_ = (K - W).normalized()
    palm = across.cross(long_) * k
    return W, E, across, long_, palm.normalized(), (W - E).normalized()


def keep_hand(L, s, H):
    """After an arm move: the hand back at its own world turn, where the arm put it."""
    n = L._n("hand", s)
    L._set_world(n, Matrix.LocRotScale(L._bw(n).translation, H.to_quaternion(), L._unit()))


def finger_turn(L, s, f, axis_kind, deg):
    n = L._n("finger", s, f, "02")
    W, E, across, long_, palm, fore = axes(L, s)
    pivot = L._bw(n).translation
    if axis_kind == "curl":
        # + curls toward the palm: the sign the anatomy check reads (across times -1 on the left hand)
        L._turn_about(n, across * (-1.0 if s == "l" else 1.0), deg, pivot)
    else:
        L._turn_about(n, palm, deg, pivot)


def wrist_deg(L, s):
    """The angle between the forearm's line and the hand's (wrist to middle knuckle), from the bones alone."""
    W, E, across, long_, palm, fore = axes(L, s)
    return math.degrees(fore.angle(long_))


def curl_02(L, s, f):
    """The middle joint's curl, + toward the palm, from the bones alone (the same definition the labels state)."""
    b = lambda j: L._bw(L._n("finger", s, f, j)).translation
    i01 = L._bw(L._n("finger", s, "index", "01")).translation
    p01 = L._bw(L._n("finger", s, "pinky", "01")).translation
    across = (i01 - p01).normalized() * (-1.0 if s == "l" else 1.0)
    a, c = (b("02") - b("01")).normalized(), (b("03") - b("02")).normalized()
    return math.degrees(math.atan2(a.cross(c).dot(across), a.dot(c)))


def turn_wrist(L, s, deg, grow):
    """The hand turned deg about the knuckle line, in the way that bends the wrist more (grow) or less. A benign turn
    (not grow) also takes the way that pushes the hand least into the rifle: on a tight grip one way drives a finger
    into it (the M24 magazine), which would make a benign pose a faulty one."""
    hand = L._n("hand", s)
    W, E, across, long_, palm, fore = axes(L, s)
    M = L._bw(hand).copy()
    tried = []
    for sg in (1.0, -1.0):
        L._turn_about(hand, across, deg * sg, W)
        L._place_rifle()
        c = L._hand_contacts(s, L._tree())
        tried.append((round(c[1] + c[3], 2), wrist_deg(L, s), sg))
        L._set_world(hand, M)
    if grow:
        _d, ang, sg = max(tried, key=lambda t: t[1])
    else:
        _d, ang, sg = min(tried)
    L._turn_about(hand, across, deg * sg, W)


def inject(L, s, name, size):
    hand = L._n("hand", s)
    W, E, across, long_, palm, fore = axes(L, s)
    if name == "wrist_bend":
        turn_wrist(L, s, size, grow=True)
    elif name == "wrist_nudge":
        turn_wrist(L, s, size, grow=False)
    elif name == "wrist_twist":
        L._turn_about(hand, fore, size, W)
    elif name == "back_of_hand":
        L._turn_about(hand, long_, size, W)
    elif name in ("index_side", "middle_side"):
        finger_turn(L, s, name.split("_")[0], "side", size)
    elif name in ("index_hyper", "middle_hyper"):
        # the middle joint set to `size` degrees bent backward, whatever its curl was
        f = name.split("_")[0]
        finger_turn(L, s, f, "curl", -size - curl_02(L, s, f))
    elif name == "index_curl":
        c = curl_02(L, s, "index")
        finger_turn(L, s, "index", "curl", size if c + size <= 100.0 else -size)
    elif name in ("palm_clip", "hand_back"):
        H = L._bw(hand).copy()
        centre = W.lerp(L._bw(L._n("finger", s, "middle", "01")).translation, 0.5)
        co, d = L._tree().nearest(centre)
        u = (co - centre).normalized() if co is not None and d > 1e-6 else palm
        if name == "palm_clip":
            # the palm capsule's skin (radius 1.8 cm) moved to `size` cm past the nearest rifle surface
            move = d - 0.018 + size / 100.0
        else:
            move = -size / 100.0
        # the elbow keeps its own side: the pole on the line from the shoulder-wrist middle through the elbow
        S = L._bw(L._n("upperarm", s)).translation
        L._two_bone(s, W + u * move, pole_world=E + (E - (S + W) / 2.0))
        keep_hand(L, s, H)
    elif name == "elbow_high":
        H = L._bw(hand).copy()
        G = L._gun()
        L._two_bone(s, W, pole_world=G @ ue(Vector((60.0 if s == "l" else -60.0, -10.0, 80.0))))
        keep_hand(L, s, H)
    else:
        raise ValueError(name)
    L._place_rifle()


def save(L, pid, label, labels_fh):
    folder = os.path.join(OUT, "poses")
    fr = L._capture()
    with open(os.path.join(folder, pid + ".pose.json"), "w") as fh:
        json.dump({"fps": 30, "frames": [fr, fr]}, fh)
    if RENDER:
        for img in L.render(views=VIEWS)["images"]:
            os.replace(img["path"], os.path.join(folder, "%s_%s.png" % (pid, img["view"])))
    s = label["side"]
    geometry = {"wrist_deg": round(wrist_deg(L, s), 1), "index_02_curl": round(curl_02(L, s, "index"), 1),
                "middle_02_curl": round(curl_02(L, s, "middle"), 1)}
    labels_fh.write(json.dumps(dict(label, id=pid, geometry=geometry)) + "\n")
    labels_fh.flush()


def main():
    os.makedirs(os.path.join(OUT, "poses"), exist_ok=True)
    L = Lab(RIGS, OUT)
    known = L.list_rigs()["rigs"]
    with open(os.path.join(OUT, "labels.jsonl"), "w") as labels_fh, open(os.path.join(OUT, "bases.jsonl"), "w") as bases_fh:
        for rig, clip, sec, sides in BASES:
            if rig not in known:
                print("POSEFAULT skip %s: not in the rigs file" % rig, flush=True)
                continue
            L.load_rig(rig)
            if clip != "idle" and clip not in L.clips:
                print("POSEFAULT skip %s %s: no such clip" % (rig, clip), flush=True)
                continue
            pose_base = (lambda: L.pose_idle()) if clip == "idle" else (lambda: L.pose_clip(clip, sec))
            pose_base()
            base_frame = L._capture()
            for s in sides:
                L._apply(base_frame)
                game_bad, rep = clean_side(L, s)
                pull, bad = (0.0, []) if not game_bad else clean_up(L, s)
                side_frame = L._capture()
                base_id = "%s-%s-%s" % (rig, clip, s)
                bases_fh.write(json.dumps({"base": base_id, "rig": rig, "clip": clip, "seconds": sec, "side": s,
                                           "game_pose_bad": game_bad, "mended": bool(game_bad), "pull_cm": pull,
                                           "used": not bad, "bad": bad, "game_report": rep}) + "\n")
                bases_fh.flush()
                print("POSEFAULT base %s: game pose %s; %s" % (
                    base_id, "clean" if not game_bad else "breaks %d rules" % len(game_bad),
                    ("used (mended, pulled %.2f cm)" % pull if game_bad else "used") if not bad else "not used: " + "; ".join(bad)),
                    flush=True)
                if bad:
                    continue
                common = {"base": base_id, "rig": rig, "clip": clip, "seconds": sec, "side": s, "mended": bool(game_bad),
                          "pull_cm": pull}
                L._apply(side_frame)
                save(L, base_id + "-clean", dict(common, fault=None, kind=None, size=None, faulty=False), labels_fh)
                for name, size in BENIGN:
                    L._apply(side_frame)
                    inject(L, s, name, size)
                    save(L, "%s-%s" % (base_id, name), dict(common, fault=name, kind=None, size=size, faulty=False), labels_fh)
                for name, kind, sizes in FAULTS:
                    for size in sizes:
                        L._apply(side_frame)
                        inject(L, s, name, size)
                        save(L, "%s-%s-%g" % (base_id, name, size), dict(common, fault=name, kind=kind, size=size, faulty=True), labels_fh)
    print("POSEFAULT done: %s" % OUT, flush=True)


main()
