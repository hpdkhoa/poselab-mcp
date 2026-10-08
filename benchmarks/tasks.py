"""The benchmark's tasks on the built-in sample rig.

Each task has a setup (tool calls that build the starting scene), a prompt, a grader that measures the final scene
with Pose Lab (the model's own claims never count), and an oracle: a scripted solution that proves the task can be
solved (or, for the impossible task, that the answer is right) before any model runs it.
"""
import math

GRIP_R = ["index*_r", "middle*_r", "ring*_r", "pinky*_r", "thumb*_r", "palm_r"]
GRIP_L = ["index*_l", "middle*_l", "ring*_l", "pinky*_l", "thumb*_l", "palm_l"]
GRIPS = GRIP_R + GRIP_L          # a hand round its grip touches the rifle on the idle pose already
PORT_GOALS = [
    {"type": "faces_eye", "point": "port", "min": 0.5},
    {"type": "clearance", "parts": "both", "ignore": GRIPS, "max_cm": 0.0},
    {"type": "visible", "point": "port", "min": 0.6},
    {"type": "on_screen", "point": "port"},
]
CLIP_CHECKS = [
    {"type": "hold", "side": "l", "max_cm": 0.5},
    {"type": "hold", "side": "r", "max_cm": 0.5},
    {"type": "pop", "bones": ["hand_l", "hand_r"], "max_cm_per_s": 250},
    {"type": "clearance", "parts": "both", "ignore": GRIPS, "max_cm": 0.0},
]
CONTACT_TARGET = [2.6, 33.0, 2.0]   # gun frame, cm: on the handguard's left side


def dist(a, b):
    return math.dist(a, b)


def goal_report(lab, goals):
    """Each goal measured on the scene as it stands: [(goal type, met, value)]."""
    out = []
    for g in goals:
        t = g["type"]
        if t == "faces_eye":
            v = lab.data("faces_eye", point=g["point"])["facing"]
            out.append((t, v >= g["min"], v))
        elif t == "visible":
            v = lab.data("visible", point=g["point"])["visible"]
            out.append((t, v >= g["min"], v))
        elif t == "on_screen":
            v = lab.data("screen", point=g["point"]).get("on_screen", False)
            out.append((t, bool(v), v))
        elif t == "clearance":
            v = lab.data("clearance", parts=g["parts"], ignore=g.get("ignore", []))["worst_cm"]
            out.append((t, v <= g.get("max_cm", 0.0), v))
    return out


def hands_and_rifle(lab):
    """Where the hands sit on the rifle (gun frame) and where the rifle is (arms frame)."""
    return {"grip": lab.data("where", names=["hand_l", "hand_r"], frame="gun"),
            "wrists": lab.data("where", names=["hand_l", "hand_r"], frame="arms"),
            "rifle": lab.data("where", names=["muzzle", "stock"], frame="arms")}


def moved(a, b):
    return max(dist(a[k], b[k]) for k in a)


# --- 1. the port to the eye --------------------------------------------------------------------------------------
def setup_port(lab):
    return {"start": hands_and_rifle(lab)}


def grade_port(lab, ctx, submit):
    goals = goal_report(lab, PORT_GOALS)
    drift = moved(ctx["start"]["grip"], hands_and_rifle(lab)["grip"])
    checks = [{"check": t, "met": m, "value": v} for t, m, v in goals] + [{"check": "hands kept their grip (cm)", "met": drift <= 0.5, "value": round(drift, 2)}]
    return all(c["met"] for c in checks), checks


def oracle_port(lab, ctx):
    lab.data("solve", dofs={"roll": [-30, 90], "swing": [-40, 40], "right": [-25, 10], "up": [-30, 5]}, goals=PORT_GOALS, samples=200, maximize=0)
    return {"answer": "done"}


# --- 2. can a turn alone show the port? ---------------------------------------------------------------------------
def setup_turn(lab):
    return {}


def grade_turn(lab, ctx, submit):
    answer = (submit or {}).get("answer", "").strip().lower()
    return answer == "impossible", [{"check": "answer is impossible", "met": answer == "impossible", "value": answer or None}]


def oracle_turn(lab, ctx):
    r = lab.data("solve", dofs={"roll": [-180, 180], "swing": [-40, 40]}, goals=PORT_GOALS, pivot="stock", samples=400)
    return {"answer": "impossible" if r["all_met_in_samples"].startswith("0 ") and not r["all_met"] else "possible"}


# --- 3. a forearm through the rifle -------------------------------------------------------------------------------
def setup_forearm(lab):
    lab.data("move_gun", roll=-60, pitch=-20, keep_hands=["l", "r"])
    worst = lab.data("clearance", parts="both", ignore=GRIPS)["worst_cm"]
    assert worst > 1.0, "the setup no longer clips (%.2f cm)" % worst
    return {"start": hands_and_rifle(lab), "clip_cm": worst}


def grade_forearm(lab, ctx, submit):
    now = hands_and_rifle(lab)
    worst = lab.data("clearance", parts="both", ignore=GRIPS)["worst_cm"]
    checks = [{"check": "no clipping (cm)", "met": worst <= 0.0, "value": worst},
              {"check": "rifle kept still (cm)", "met": moved(ctx["start"]["rifle"], now["rifle"]) <= 0.2, "value": round(moved(ctx["start"]["rifle"], now["rifle"]), 2)},
              {"check": "wrists kept still (cm)", "met": moved(ctx["start"]["wrists"], now["wrists"]) <= 0.3, "value": round(moved(ctx["start"]["wrists"], now["wrists"]), 2)}]
    return all(c["met"] for c in checks), checks


def oracle_forearm(lab, ctx):
    lab.data("snapshot", action="save", name="_oracle")
    for x in (10, 30, 50, 70):
        for y in (-40, -10, 20):
            for z in (60, 100, 140, 180):
                lab.data("snapshot", action="load", name="_oracle")
                lab.data("reach", side="l", target="hand_l", frame="arms", pole=[x, y, z])
                if lab.data("clearance", parts="both", ignore=GRIPS)["worst_cm"] <= 0.0:
                    return {"answer": "done"}
    return {"answer": "done"}


# --- 4. a fingertip on a mark -------------------------------------------------------------------------------------
def setup_contact(lab):
    return {"start": hands_and_rifle(lab)}


def grade_contact(lab, ctx, submit):
    now = hands_and_rifle(lab)
    off = lab.data("distance", a="index_tip_l", b=CONTACT_TARGET, frame="gun")["cm"]
    worst = lab.data("clearance", parts="both", ignore=["index3_l"] + GRIP_R)["worst_cm"]
    checks = [{"check": "fingertip on the mark (cm)", "met": off <= 0.5, "value": off},
              {"check": "nothing else clips (cm)", "met": worst <= 0.2, "value": worst},
              {"check": "rifle kept still (cm)", "met": moved(ctx["start"]["rifle"], now["rifle"]) <= 0.2, "value": round(moved(ctx["start"]["rifle"], now["rifle"]), 2)}]
    return all(c["met"] for c in checks), checks


def oracle_contact(lab, ctx):
    for _ in range(6):
        d = lab.data("distance", a="index_tip_l", b=CONTACT_TARGET, frame="gun")
        w = lab.data("where", names=["hand_l"], frame="gun")["hand_l"]
        lab.data("reach", side="l", target=[w[i] + d["b_minus_a"][i] for i in range(3)], frame="gun")
    return {"answer": "done"}


# --- 5. a clip with faults ----------------------------------------------------------------------------------------
def setup_clip(lab):
    lab.data("record_clip", action="start", clip="roll")
    lab.data("record_clip", action="key", seconds=0.0)
    lab.data("move_gun", roll=75, keep_hands=["l", "r"])
    lab.data("record_clip", action="key", seconds=0.6)
    lab.data("record_clip", action="key", seconds=0.8)
    lab.data("move_gun", up=15, keep_hands=["l", "r"])
    lab.data("record_clip", action="key", seconds=0.8333)
    lab.data("move_gun", up=-15, keep_hands=["l", "r"])
    lab.data("record_clip", action="key", seconds=0.8667)
    lab.data("pose_idle")
    lab.data("record_clip", action="key", seconds=1.4)
    lab.data("record_clip", action="stop")
    scan = lab.data("scan_clip", clip="roll", checks=CLIP_CHECKS)
    assert not scan["all_passed"], "the faulty clip passes its checks"
    lab.data("pose_clip", clip="roll", seconds=0.6)
    rolled = lab.data("where", names=["muzzle", "stock"], frame="arms")
    lab.data("pose_idle")
    return {"frames": scan["frames"], "rolled": rolled}


def grade_clip(lab, ctx, submit):
    try:
        scan = lab.data("scan_clip", clip="roll_ok", checks=CLIP_CHECKS)
    except RuntimeError as e:
        return False, [{"check": "clip roll_ok exists", "met": False, "value": str(e)[:120]}]
    lab.data("pose_clip", clip="roll_ok", seconds=0.6)
    off = moved(ctx["rolled"], lab.data("where", names=["muzzle", "stock"], frame="arms"))
    checks = [{"check": x["check"]["type"] + (" " + x["check"]["side"] if x["check"].get("side") else ""), "met": x["passed"], "value": x["worst"]} for x in scan["checks"]]
    checks += [{"check": "same length (frames)", "met": scan["frames"] >= ctx["frames"], "value": scan["frames"]},
               {"check": "the roll still happens (cm off at 0.6 s)", "met": off <= 2.0, "value": round(off, 2)}]
    return all(c["met"] for c in checks), checks


def oracle_clip(lab, ctx):
    lab.data("fix_clip", clip="roll", checks=CLIP_CHECKS, out="roll_ok")
    return {"answer": "done"}


TASKS = {
    "port_to_eye": {
        "setup": setup_port, "grade": grade_port, "oracle": oracle_port,
        "prompt": ("The rifle's ejection port is on its right side (rig point `port`). Move the rifle so a player could "
                   "look into the port: the port faces the eye at 0.5 or more (1 is square on), at least 60% of it is "
                   "visible from the eye, it is on screen, and no part of the arms is inside the rifle (the fingers and "
                   "palms round their grips may touch it). Both hands must keep their grip on the rifle. Leave the scene "
                   "in that pose and call submit."),
    },
    "turn_only": {
        "setup": setup_turn, "grade": grade_turn, "oracle": oracle_turn,
        "prompt": ("The rifle's ejection port is on its right side (rig point `port`). Question: using only turns of the "
                   "rifle about the `stock` pivot (roll at any angle, and swing up to 40 degrees each way) and no moves, "
                   "can the port face the eye at 0.5 or "
                   "more, with at least 60% of it visible, on screen, and nothing of the arms inside the rifle (fingers "
                   "and palms round their grips may touch)? Answer with submit: answer \"possible\" or \"impossible\"."),
    },
    "forearm_clear": {
        "setup": setup_forearm, "grade": grade_forearm, "oracle": oracle_forearm,
        "prompt": ("In the current pose an arm passes through the rifle. Fix it so no part of the arms is inside the rifle "
                   "(the fingers and palms round their grips may touch it). Do not move the rifle, and keep both wrists "
                   "where they are (within 0.3 cm). Then call submit."),
    },
    "fingertip_contact": {
        "setup": setup_contact, "grade": grade_contact, "oracle": oracle_contact,
        "prompt": ("Put the tip of the left index finger on the point [2.6, 33.0, 2.0] in the gun frame (cm), within 0.5 "
                   "cm. Do not move the rifle. Apart from that fingertip, nothing of the arms may sit more than 0.2 cm "
                   "inside the rifle (the right hand round its grip may touch it). Then call submit."),
    },
    "clip_repair": {
        "setup": setup_clip, "grade": grade_clip, "oracle": oracle_clip,
        "prompt": ("A clip named `roll` is loaded: the rifle rolls and comes back over 1.4 s, at 30 frames a second. It "
                   "has faults: the hands drift off the rifle between its keys, and one frame jumps. Make a clip named "
                   "`roll_ok` with the same motion and length where each hand stays within 0.5 cm of its grip on the "
                   "rifle in every frame, no hand moves faster than 250 cm/s between frames, and nothing of the arms is "
                   "inside the rifle (the fingers and palms round their grips may touch it). Then call submit."),
    },
}

# the tools each condition gets (the harness loads the rig; submit is added by the runner)
VISION = ["describe", "pose_idle", "pose_clip", "move_part", "move_gun", "reach", "snapshot", "record_clip", "render"]
MEASURED = VISION + ["where", "distance", "clearance", "faces_eye", "visible", "screen", "solve", "load_clip", "scan_clip", "fix_clip", "save_clip"]
CONDITIONS = {"vision": VISION, "measured": MEASURED}
