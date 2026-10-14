"""PoseFault repair: an agent gets one PoseFault pose and must find and repair a physical fault on one side, through
agent_bridge.py. Both conditions pose with the same tools (renders, turn_bone, reach, snapshot); the measured one may
also call Pose Lab's checks. The final scene is graded by Pose Lab's rules, as measure.py applies them.

    POSEFAULT_REPAIR=<list.json> python benchmarks/agent_bridge.py serve pf:r01 measured PORT OUT_DIR

list.json: {"poses_dir": <a generate.py output folder>, "episodes": {"r01": "<pose id>", ...}}. The pose ids stay in
that file; the task name the agent's harness sees is the anonymous one.
"""
import json
import os

# the bases whose hand is meant to hold a part (generate.py GRASP_BASES)
GRASP_BASES = {"ak-reload_empty-r", "m24-good-l"}
TIPS = ("thumb", "index", "middle", "ring", "pinky")
PROMPT = ("This is a first-person pose: two arms hold a rifle. The {side} arm and hand may have a physical fault: "
          "something a real human arm and hand could not do, or that puts the body inside the rifle (for example a "
          "wrist bent far off the forearm, a hand twisted against the forearm, the rifle held with the back of the "
          "hand, a finger bent sideways or backward, the palm or fingers sunk into the rifle, the elbow raised above "
          "the shoulder). If you find one, repair it with the smallest change that makes the pose physically possible, "
          "and keep the {side} hand where it is on the rifle. If there is none, change nothing. Do not move the rifle "
          "and leave the other arm as it is. Then call submit with answer: the fault you found and what you changed, "
          "or none.")
# the tools each condition gets; submit is added by the bridge
VISION = ["describe", "render", "turn_bone", "reach", "snapshot"]
MEASURED = VISION + ["anatomy", "grip", "clearance", "where", "distance"]
CONDITIONS = {"vision": VISION, "measured": MEASURED}


def _list():
    with open(os.environ["POSEFAULT_REPAIR"]) as fh:
        return json.load(fh)


def _label(pid, poses_dir):
    for x in open(os.path.join(poses_dir, "labels.jsonl")):
        lab_ = json.loads(x)
        if lab_["id"] == pid:
            return lab_
    raise KeyError(pid)


def _tips(lab, s):
    names = ["%s_tip_%s" % (f, s) for f in TIPS]
    return lab.data("where", names=names, frame="gun")


def _moved(a, b):
    return max(sum((x - y) ** 2 for x, y in zip(a[k], b[k])) ** 0.5 for k in a)


def flags(lab, s, grasp):
    """measure.py's rules on one side: anatomy, grip (the grasp rule on a grasp base), the forearm out of the rifle."""
    a = lab.data("anatomy", side=s)[s]["bad"]
    g = lab.data("grip", side=s, grasp=grasp)[s]["bad"]
    c = lab.data("clearance", parts=s, ignore=["index", "middle", "ring", "pinky", "thumb", "palm"])
    return list(a) + list(g) + (["forearm %.2f cm inside the rifle" % c["worst_cm"]] if c["worst_cm"] > 0.1 else [])


def make(anon):
    L = _list()
    pid = L["episodes"][anon]
    lab_ = _label(pid, L["poses_dir"])
    s = lab_["side"]
    grasp = lab_["base"] in GRASP_BASES

    def setup(lab):
        lab.data("load_clip", clip="start", path=os.path.join(L["poses_dir"], "poses", pid + ".pose.json"))
        lab.data("pose_clip", clip="start", seconds=0.0)
        return {"pid": pid, "side": s, "grasp": grasp, "faulty": lab_["faulty"], "fault": lab_["fault"],
                "start_bad": flags(lab, s, grasp), "start_tips": _tips(lab, s)}

    def grade(lab, ctx, submit):
        bad = flags(lab, s, grasp)
        tips = _tips(lab, s)
        moved = round(_moved(ctx["start_tips"], tips), 2)
        lab.data("load_clip", clip="base", path=os.path.join(L["poses_dir"], "poses", lab_["base"] + "-clean.pose.json"))
        lab.data("pose_clip", clip="base", seconds=0.0)
        from_base = round(_moved(lab.data("where", names=list(tips), frame="gun"), tips), 2)
        checks = [{"check": "rules broken at the end", "met": not bad, "value": bad},
                  {"check": "fingertips moved from the start (cm)", "met": ctx["faulty"] or moved <= 0.5, "value": moved},
                  {"check": "fingertips from the clean base (cm)", "met": True, "value": from_base},
                  {"check": "rules broken at the start", "met": True, "value": ctx["start_bad"]},
                  {"check": "pose", "met": True, "value": {"id": pid, "fault": ctx["fault"], "size": lab_["size"]}}]
        return all(c["met"] for c in checks), checks

    return {"rig": lab_["rig"], "setup": setup, "grade": grade,
            "prompt": PROMPT.format(side="left" if s == "l" else "right")}


class _Tasks(dict):
    def __missing__(self, task):
        if not task.startswith("pf:"):
            raise KeyError(task)
        self[task] = make(task[3:])
        return self[task]


TASKS = _Tasks()
