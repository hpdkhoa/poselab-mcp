"""Run benchmark episodes with coding agents (for example Claude Code sub-agents) instead of the API: no API key.

run.py drives a model through the Anthropic API. This file lets an agent that can run shell commands take the same
tasks: a bridge keeps one Pose Lab session open behind a local port, offers only the condition's tools, and grades
the scene with tasks.py when the agent calls submit (or when it reaches the call limit).

    python benchmarks/agent_bridge.py serve TASK CONDITION PORT OUT_DIR     (one episode; stops after grading)
    python benchmarks/agent_bridge.py prompt TASK CONDITION PORT            (the text to give the agent)
    python benchmarks/agent_bridge.py call PORT TOOL '{"arg": 1}'           (what the agent runs; TOOL tools lists them)
    python benchmarks/agent_bridge.py summary RUN_DIR                       (a summary.md over every episode in RUN_DIR)

Differences from run.py: the agent works in its own harness, calls tools through a command and opens renders as
image files. The rule against reading the benchmark's files is an instruction, not a sandbox, so check each
episode's calls.jsonl. No cost is recorded. Use run.py for numbers you publish.
"""
import base64
import json
import os
import socket
import socketserver
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

CAP = 60   # tool calls per episode
SYSTEM = (
    "You pose a first-person rig in Blender through tools: two arms holding a rifle, seen from the player's eye. The "
    "rig is already loaded; call describe first to learn its frames, sign rules and named points. Work only through "
    "the tools. When the task is done, leave the scene in its final state and call submit once; the final scene is "
    "measured and graded after you submit, so a claim in your answer does not count."
)


def serve(task, condition, port, out):
    from mcp_stdio import PoseLab
    from tasks import CONDITIONS, TASKS
    os.makedirs(out, exist_ok=True)
    lab = PoseLab()
    lab.data("load_rig", rig="sample")
    ctx = TASKS[task]["setup"](lab)
    by = {t["name"]: t for t in lab.tools}
    allowed = [n for n in CONDITIONS[condition] if n in by]
    state = {"calls": 0, "done": False, "t0": None, "img": 0}

    def log(rec):
        with open(os.path.join(out, "calls.jsonl"), "a") as fh:
            fh.write(json.dumps(rec) + "\n")

    def finish(submitted):
        passed, checks = TASKS[task]["grade"](lab, ctx, submitted)
        res = {"task": task, "condition": condition, "passed": passed, "submitted": submitted is not None,
               "answer": (submitted or {}).get("answer"), "checks": checks, "tool_calls": state["calls"],
               "seconds": round(time.time() - (state["t0"] or time.time()), 1)}
        with open(os.path.join(out, "result.json"), "w") as fh:
            json.dump(res, fh, indent=1)
        state["done"] = True

    def handle(req):
        if state["done"]:
            return "the episode is over"
        tool, args = req.get("tool"), req.get("args") or {}
        if tool == "tools":
            return json.dumps([{"name": n, "description": by[n].get("description", ""), "input_schema": by[n]["inputSchema"]}
                               for n in allowed] + [{"name": "submit", "description": "Ends the task. Call it once, when "
                               "the scene is in its final state (or with your answer for a question).", "input_schema":
                               {"answer": "for a yes/no question: possible or impossible; otherwise a short note"}}], indent=1)
        state["t0"] = state["t0"] or time.time()
        state["calls"] += 1
        if tool == "submit":
            log({"tool": tool, "args": args})
            finish(dict(args))
            return "submitted"
        if tool not in allowed:
            log({"tool": tool, "args": args, "error": True, "reply": "no such tool"})
            return "ERROR: no such tool"
        r = lab.call(tool, **args)
        parts = []
        for c in r.get("content", []):
            if c["type"] == "text":
                parts.append(c["text"][:20000])
            elif c["type"] == "image":
                state["img"] += 1
                path = os.path.join(out, "render_%02d.png" % state["img"])
                with open(path, "wb") as fh:
                    fh.write(base64.b64decode(c["data"]))
                parts.append("[image saved: %s  (open it to see it)]" % path)
        text = ("ERROR: " if r.get("isError") else "") + ("\n".join(parts) or "(no output)")
        log({"tool": tool, "args": args, "error": bool(r.get("isError")), "reply": text[:2000]})
        if state["calls"] >= CAP:
            finish(None)
            text += "\n[call limit reached: the episode is over and the scene was graded as it is]"
        return text

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            try:
                reply = handle(json.loads(self.rfile.readline()))
            except Exception as e:
                reply = "ERROR: %s" % e
            self.wfile.write(reply.encode())

    srv = socketserver.TCPServer(("127.0.0.1", port), Handler)   # local only
    print("ready", flush=True)
    try:
        while not state["done"]:
            srv.handle_request()
    finally:
        srv.server_close()
        lab.close()


def call(port, tool, args):
    s = socket.create_connection(("127.0.0.1", port))
    s.sendall((json.dumps({"tool": tool, "args": args}) + "\n").encode())
    data = b""
    while True:
        b = s.recv(65536)
        if not b:
            break
        data += b
    sys.stdout.reconfigure(encoding="utf-8")
    print(data.decode())


def prompt(task, condition, port):
    from tasks import TASKS
    cmd = '"%s" "%s" call %d' % (sys.executable.replace("\\", "/"), os.path.abspath(__file__).replace("\\", "/"), port)
    return "\n\n".join([
        "You are taking part in a benchmark episode. " + SYSTEM,
        "TASK: " + TASKS[task]["prompt"],
        "HOW TO CALL TOOLS. Each tool call is one shell command:\n  %s TOOL 'JSON_ARGS'\nStart with TOOL = tools (no "
        "args) to list the tools you may use, with their argument schemas. Example: %s describe '{}'. When a tool returns "
        "an image, the reply gives a PNG path: open it to see it. To finish: %s submit '{\"answer\": \"...\"}'. You have "
        "at most %d tool calls." % (cmd, cmd, cmd, CAP),
        "RULES (this is a fair test): use only that command, and open only the PNG paths the tools give you. Do not "
        "read, list or search any other files or folders (no source code, no benchmark files, no other results), and "
        "do not write files. Do not use any port other than %d." % port,
        "When you are done, report whether you submitted, your final settings, and in 2-3 sentences how you judged "
        "the result.",
    ])


def summary(run_dir):
    rows = []
    for d in sorted(os.listdir(run_dir)):
        p = os.path.join(run_dir, d, "result.json")
        if os.path.exists(p):
            with open(p) as fh:
                rows.append(json.load(fh))
    tasks = list(dict.fromkeys(r["task"] for r in rows))
    conds = list(dict.fromkeys(r["condition"] for r in rows))
    lines = ["# Pose Lab benchmark (agent bridge)", "", "Episodes run by coding agents through agent_bridge.py, graded "
             "by tasks.py. See agent_bridge.py for how this differs from run.py.", "",
             "| Task | " + " | ".join(conds) + " |", "|---|" + "---|" * len(conds)]
    for t in tasks:
        cells = []
        for c in conds:
            rs = [r for r in rows if r["task"] == t and r["condition"] == c]
            cells.append(", ".join("%s (%d calls)" % ("PASS" if r["passed"] else "fail", r["tool_calls"]) for r in rs))
        lines.append("| %s | %s |" % (t, " | ".join(cells)))
    lines.append("| **all** | " + " | ".join("%d / %d" % (sum(r["passed"] for r in rows if r["condition"] == c),
                                                        sum(1 for r in rows if r["condition"] == c)) for c in conds) + " |")
    lines += ["", "Failed checks:", ""]
    for r in rows:
        for ch in r["checks"]:
            if not ch["met"]:
                lines.append("* %s, %s: %s = %s" % (r["task"], r["condition"], ch["check"], ch["value"]))
    with open(os.path.join(run_dir, "summary.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a or a[0] not in ("serve", "prompt", "call", "summary"):
        sys.exit(__doc__)
    if a[0] == "serve":
        serve(a[1], a[2], int(a[3]), a[4])
    elif a[0] == "prompt":
        print(prompt(a[1], a[2], int(a[3])))
    elif a[0] == "call":
        call(int(a[1]), a[2], json.loads(a[3]) if len(a) > 3 else {})
    else:
        summary(a[1])
