"""PoseFault arm A: a VLM judges each pose from its renders alone (eye, left, right, top), the way a person reviews a
pose by eye. The answer is graded against the label that generate.py wrote, with the same fault names arm C uses.

    python benchmarks/posefault/judge.py <generate.py output folder> --backend anthropic --model claude-opus-5-5
    python benchmarks/posefault/judge.py <folder> --backend ollama --model qwen2.5vl:7b [--url http://localhost:11434]
    python benchmarks/posefault/judge.py <folder> --grade-only arm_a-<name>.jsonl

Model runs cost money (anthropic) or GPU time (ollama): start with --limit 5. Each run writes
<folder>/arm_a-<backend>-<model>.jsonl, one line per pose with the raw answer, and prints the same table as measure.py.
"""
import argparse
import base64
import json
import os
import re
import sys
import time
import urllib.request
from collections import defaultdict

VIEWS = ("eye", "left", "right", "top")
TYPES = ("wrist_bend", "wrist_twist", "back_of_hand", "finger_sideways", "finger_hyperextended", "hand_in_object",
         "elbow_high", "other")
# the label's fault -> the answer types that name it
NAMED = {
    "wrist_bend": ("wrist_bend",),
    "wrist_twist": ("wrist_twist",),
    "back_of_hand": ("back_of_hand", "wrist_twist"),
    "index_side": ("finger_sideways",),
    "middle_side": ("finger_sideways",),
    "index_hyper": ("finger_hyperextended",),
    "middle_hyper": ("finger_hyperextended",),
    "palm_clip": ("hand_in_object",),
    "elbow_high": ("elbow_high",),
}
PROMPT = """You review a pose for a first-person game: two arms hold a rifle. The four pictures show one pose from the
player's eye, from the left, from the right and from the top. Check the {side_word} arm and hand only.

Find physical faults that a real human arm and hand could not do, or that would put the body inside the rifle:
- wrist_bend: the wrist bent far off the forearm's line (more than about 30 degrees)
- wrist_twist: the hand twisted about the forearm's line against the forearm
- back_of_hand: the rifle held with the back of the hand instead of the palm
- finger_sideways: a finger joint bent sideways out of the finger's plane
- finger_hyperextended: a finger joint bent backward past straight
- hand_in_object: the palm or fingers sunk into the rifle
- elbow_high: the elbow raised above the shoulder
- other: any other physical fault (say what)

Answer with JSON only, no other text:
{{"faulty": true or false, "faults": [{{"type": "<one of the types>", "where": "<body part>", "note": "<short>"}}]}}
An empty list means the pose has no fault."""


def images(folder, pid):
    return [os.path.join(folder, "poses", "%s_%s.png" % (pid, v)) for v in VIEWS]


def ask_anthropic(model, prompt, paths):
    import anthropic  # pip install anthropic
    content = [{"type": "image", "source": {"type": "base64", "media_type": "image/png",
                                            "data": base64.b64encode(open(p, "rb").read()).decode()}} for p in paths]
    content.append({"type": "text", "text": prompt})
    msg = anthropic.Anthropic().messages.create(model=model, max_tokens=1024, messages=[{"role": "user", "content": content}])
    return "".join(b.text for b in msg.content if b.type == "text"), {"in": msg.usage.input_tokens, "out": msg.usage.output_tokens}


def ask_ollama(model, prompt, paths, url):
    body = {"model": model, "stream": False, "options": {"temperature": 0},
            "messages": [{"role": "user", "content": prompt,
                          "images": [base64.b64encode(open(p, "rb").read()).decode() for p in paths]}]}
    req = urllib.request.Request(url.rstrip("/") + "/api/chat", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=600) as r:
        out = json.load(r)
    return out["message"]["content"], {"in": out.get("prompt_eval_count"), "out": out.get("eval_count")}


def parse(text):
    m = re.search(r"\{.*\}", text, re.S)
    try:
        ans = json.loads(m.group(0)) if m else None
    except ValueError:
        ans = None
    if not isinstance(ans, dict):
        return None
    faults = [f for f in ans.get("faults", []) if isinstance(f, dict)]
    return {"faulty": bool(ans.get("faulty")) or bool(faults), "types": [str(f.get("type", "")).strip() for f in faults]}


def grade(labels, answers):
    rows = []
    for lab in labels:
        a = answers.get(lab["id"])
        if a is None:
            continue
        p = a.get("parsed")
        flagged = bool(p and p["faulty"])
        named = bool(lab["faulty"] and p and any(t in NAMED[lab["fault"]] for t in p["types"]))
        rows.append({"id": lab["id"], "fault": lab["fault"], "faulty": lab["faulty"], "flagged": flagged, "named": named,
                     "unparsed": p is None})
    by = defaultdict(lambda: [0, 0, 0])
    for r in rows:
        k = r["fault"] if r["faulty"] else ("clean" if r["fault"] is None else "benign:" + r["fault"])
        by[k][0] += 1
        by[k][1] += r["flagged"]
        by[k][2] += r["named"]
    print("%-22s %5s %8s %6s" % ("fault", "poses", "flagged", "named"))
    for k in sorted(by):
        print("%-22s %5d %8d %6d" % (k, *by[k]))
    print("unparsed answers: %d" % sum(r["unparsed"] for r in rows))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--backend", choices=("anthropic", "ollama"))
    ap.add_argument("--model")
    ap.add_argument("--url", default="http://localhost:11434")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--grade-only")
    a = ap.parse_args()
    labels = [json.loads(x) for x in open(os.path.join(a.folder, "labels.jsonl"))]
    if a.grade_only:
        answers = {json.loads(x)["id"]: json.loads(x) for x in open(os.path.join(a.folder, a.grade_only))}
        grade(labels, answers)
        return
    if not (a.backend and a.model):
        sys.exit("--backend and --model, or --grade-only")
    out_path = os.path.join(a.folder, "arm_a-%s-%s.jsonl" % (a.backend, re.sub(r"[^A-Za-z0-9.]+", "_", a.model)))
    done = {json.loads(x)["id"]: json.loads(x) for x in open(out_path)} if os.path.exists(out_path) else {}
    todo = [l for l in labels if l["id"] not in done][: a.limit or None]
    with open(out_path, "a") as fh:
        for lab in todo:
            prompt = PROMPT.format(side_word="left" if lab["side"] == "l" else "right")
            t0 = time.time()
            if a.backend == "anthropic":
                text, usage = ask_anthropic(a.model, prompt, images(a.folder, lab["id"]))
            else:
                text, usage = ask_ollama(a.model, prompt, images(a.folder, lab["id"]), a.url)
            row = {"id": lab["id"], "answer": text, "parsed": parse(text), "usage": usage, "seconds": round(time.time() - t0, 1),
                   "model": a.model, "backend": a.backend, "date": time.strftime("%Y-%m-%d")}
            fh.write(json.dumps(row) + "\n")
            fh.flush()
            done[lab["id"]] = row
            print("%s %s" % (lab["id"], row["parsed"]), flush=True)
    grade(labels, done)


if __name__ == "__main__":
    main()
