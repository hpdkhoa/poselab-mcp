"""Pose Lab's benchmark: does a model pose a first-person rig better with measured answers than with pictures alone?

For each task and condition, a model works through Pose Lab's MCP tools until it calls submit; the harness then
measures the scene with Pose Lab and grades it. Two conditions:
  vision    posing tools and renders only (a model judges from pictures)
  measured  the same plus Pose Lab's measuring, solving, scanning and fixing tools
--oracle runs a scripted solution for every task instead of a model (no API calls): it proves each task can be
solved, and that the graders accept a right answer, before you spend anything on a model. --null submits at once
without changing anything (no API calls): the graders must fail it.

    python benchmarks/run.py --oracle
    python benchmarks/run.py --null
    python benchmarks/run.py --trials 1 --tasks port_to_eye          (a small paid run first: it prints its cost)
    python benchmarks/run.py --trials 3                              (every task, both conditions)

Needs Blender (POSELAB_BLENDER) and, for model runs, Anthropic credentials (ANTHROPIC_API_KEY or `ant auth login`).
"""
import argparse
import base64
import datetime as dt
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mcp_stdio import PoseLab  # noqa: E402
from tasks import CONDITIONS, TASKS  # noqa: E402

# $ per million tokens: input, output, cache read, cache write (5 minutes). Unknown models report no cost.
PRICES = {
    "claude-opus-5-5": (4.00, 20.00, 0.20, 5.00),
    "claude-sonnet-5-5": (2.00, 10.00, 0.20, 2.50),
    "claude-haiku-4-5": (1.00, 5.00, 0.10, 1.25),
}
FALLBACK_MODELS = ("claude-opus-5-5", "claude-sonnet-5-5", "claude-fable-5-1", "claude-opus-5")

SYSTEM = (
    "You pose a first-person rig in Blender through tools: two arms holding a rifle, seen from the player's eye. The "
    "rig is already loaded; call describe first to learn its frames, sign rules and named points. Work only through "
    "the tools. When the task is done, leave the scene in its final state and call submit once; the final scene is "
    "measured and graded after you submit, so a claim in your answer does not count."
)
SUBMIT = {
    "name": "submit",
    "description": "Ends the task. Call it once, when the scene is in its final state (or with your answer for a question).",
    "input_schema": {"type": "object", "properties": {
        "answer": {"type": "string", "description": "for a yes/no question: possible or impossible; otherwise a short note"}},
        "required": ["answer"]},
}


def anthropic_tools(lab, names):
    by = {t["name"]: t for t in lab.tools}
    out = [{"name": n, "description": by[n].get("description", ""), "input_schema": by[n]["inputSchema"]} for n in names if n in by]
    return out + [SUBMIT]


def tool_result_content(r):
    """An MCP tool result as Anthropic tool_result content (text and PNG images)."""
    out = []
    for c in r.get("content", []):
        if c["type"] == "text":
            out.append({"type": "text", "text": c["text"][:20000]})
        elif c["type"] == "image":
            out.append({"type": "image", "source": {"type": "base64", "media_type": c.get("mimeType", "image/png"), "data": c["data"]}})
    return out or [{"type": "text", "text": "(no output)"}]


def cost(model, u):
    p = PRICES.get(model)
    if not p:
        return None
    return round((u["input"] * p[0] + u["output"] * p[1] + u["cache_read"] * p[2] + u["cache_write"] * p[3]) / 1e6, 4)


def run_model(client, args, lab, task, condition):
    """One agent episode; returns (submit input or None, episode stats)."""
    tools = anthropic_tools(lab, CONDITIONS[condition])
    allowed = {t["name"] for t in tools}
    messages = [{"role": "user", "content": TASKS[task]["prompt"]}]
    usage = {"input": 0, "output": 0, "cache_read": 0, "cache_write": 0}
    calls, nudges, stop = 0, 0, None
    extra = {}
    if not args.no_fallbacks and args.model in FALLBACK_MODELS:
        extra = {"betas": ["server-side-fallback-2026-07-01"], "fallbacks": "default"}
    for turn in range(args.max_turns):
        response = client.beta.messages.create(
            model=args.model, max_tokens=16000, system=SYSTEM, tools=tools, messages=messages,
            output_config={"effort": args.effort}, cache_control={"type": "ephemeral"}, **extra)
        u = response.usage
        usage["input"] += u.input_tokens or 0
        usage["output"] += u.output_tokens or 0
        usage["cache_read"] += getattr(u, "cache_read_input_tokens", 0) or 0
        usage["cache_write"] += getattr(u, "cache_creation_input_tokens", 0) or 0
        stop = response.stop_reason
        if stop == "refusal":
            return None, {"turns": turn + 1, "tool_calls": calls, "usage": usage, "stop": "refusal"}
        messages.append({"role": "assistant", "content": response.content})   # append-only: thinking blocks stay valid
        uses = [b for b in response.content if b.type == "tool_use"]
        if not uses:
            if nudges >= 2:
                break
            nudges += 1
            messages.append({"role": "user", "content": "Keep working through the tools, then call submit."})
            continue
        results, submitted = [], None
        for b in uses:
            calls += 1
            if b.name == "submit":
                submitted = dict(b.input)
                results.append({"type": "tool_result", "tool_use_id": b.id, "content": "submitted"})
            elif b.name not in allowed:
                results.append({"type": "tool_result", "tool_use_id": b.id, "content": "no such tool", "is_error": True})
            else:
                r = lab.call(b.name, **dict(b.input))
                results.append({"type": "tool_result", "tool_use_id": b.id, "content": tool_result_content(r), "is_error": bool(r.get("isError"))})
        if submitted is not None:
            return submitted, {"turns": turn + 1, "tool_calls": calls, "usage": usage, "stop": "submit"}
        messages.append({"role": "user", "content": results})
    return None, {"turns": args.max_turns, "tool_calls": calls, "usage": usage, "stop": stop or "max_turns"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="claude-opus-5-5")
    ap.add_argument("--effort", default="high", choices=["low", "medium", "high", "xhigh", "max"])
    ap.add_argument("--tasks", default=",".join(TASKS), help="comma-separated task ids")
    ap.add_argument("--conditions", default="vision,measured")
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--max-turns", type=int, default=30)
    ap.add_argument("--no-fallbacks", action="store_true", help="do not send server-side refusal fallbacks")
    ap.add_argument("--oracle", action="store_true", help="run the scripted solutions instead of a model (no API calls)")
    ap.add_argument("--null", action="store_true", help="submit at once, changing nothing (no API calls): a control the graders must fail")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    args = ap.parse_args()
    tasks = [t for t in args.tasks.split(",") if t]
    scripted = "oracle" if args.oracle else "null" if args.null else None
    conditions = [scripted] if scripted else [c for c in args.conditions.split(",") if c]
    for t in tasks:
        if t not in TASKS:
            sys.exit("no task %s; tasks: %s" % (t, ", ".join(TASKS)))
    try:   # Blender first: without it every task fails with the same error
        from poselab_mcp.worker_client import find_blender
        find_blender()
    except Exception as e:
        sys.exit("Pose Lab cannot start: %s" % e)
    client = None
    if not scripted:
        import anthropic
        client = anthropic.Anthropic()
        try:   # credentials and model name, checked by a free token count before any paid call
            client.messages.count_tokens(model=args.model, messages=[{"role": "user", "content": "ok"}])
        except Exception as e:
            sys.exit("Anthropic API not ready (set ANTHROPIC_API_KEY or run `ant auth login`): %s" % str(e)[:300])
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = os.path.join(args.out, "%s-%s" % (stamp, scripted or args.model))
    os.makedirs(run_dir, exist_ok=True)
    lab = PoseLab()
    rows = []
    try:
        for task in tasks:
            for condition in conditions:
                for trial in range(1 if scripted else args.trials):
                    lab.data("load_rig", rig="sample")
                    ctx = TASKS[task]["setup"](lab)
                    t0 = time.time()
                    if scripted == "oracle":
                        submitted, stats = TASKS[task]["oracle"](lab, ctx), {"turns": 0, "tool_calls": 0, "usage": None, "stop": "oracle"}
                    elif scripted == "null":
                        submitted, stats = {"answer": "done"}, {"turns": 0, "tool_calls": 0, "usage": None, "stop": "null"}
                    else:
                        try:
                            submitted, stats = run_model(client, args, lab, task, condition)
                        except Exception as e:   # one failed episode does not end the run
                            submitted, stats = None, {"turns": 0, "tool_calls": 0, "usage": None, "stop": "error: %s" % str(e)[:200]}
                    passed, checks = TASKS[task]["grade"](lab, ctx, submitted)
                    row = {"task": task, "condition": condition, "trial": trial, "passed": passed, "submitted": submitted is not None,
                           "answer": (submitted or {}).get("answer"), "checks": checks, "seconds": round(time.time() - t0, 1), **stats,
                           "cost_usd": cost(args.model, stats["usage"]) if stats.get("usage") else None}
                    rows.append(row)
                    with open(os.path.join(run_dir, "episodes.jsonl"), "a") as fh:
                        fh.write(json.dumps(row) + "\n")
                    print("%-18s %-9s trial %d  %s  calls %-3d %s  %s" % (task, condition, trial, "PASS" if passed else "fail", row["tool_calls"],
                          "$%.3f" % row["cost_usd"] if row["cost_usd"] is not None else "", row["stop"]), flush=True)
    finally:
        lab.close()
    write_summary(run_dir, args, rows, tasks, conditions)


def scripted(args):
    return "oracle" if args.oracle else "null" if args.null else None


def write_summary(run_dir, args, rows, tasks, conditions):
    lines = ["# Pose Lab benchmark", "",
             "Model: %s, %s" % ({"oracle": "oracle (scripted solutions)", "null": "null (submits at once)"}.get(scripted(args), args.model + ", effort " + args.effort), dt.datetime.now().strftime("%Y-%m-%d %H:%M")), "",
             "| Task | " + " | ".join(conditions) + " |", "|---|" + "---|" * len(conditions)]
    for task in tasks:
        cells = []
        for c in conditions:
            rs = [r for r in rows if r["task"] == task and r["condition"] == c]
            cells.append("%d / %d" % (sum(r["passed"] for r in rs), len(rs)) if rs else "")
        lines.append("| %s | %s |" % (task, " | ".join(cells)))
    lines.append("| **all** | " + " | ".join("%d / %d" % (sum(r["passed"] for r in rows if r["condition"] == c), sum(1 for r in rows if r["condition"] == c)) for c in conditions) + " |")
    if not scripted(args):
        lines += ["", "| Condition | Mean tool calls | Mean seconds | Total cost |", "|---|---|---|---|"]
        for c in conditions:
            rs = [r for r in rows if r["condition"] == c]
            if rs:
                total = sum(r["cost_usd"] or 0 for r in rs)
                lines.append("| %s | %.1f | %.0f | $%.2f |" % (c, sum(r["tool_calls"] for r in rs) / len(rs), sum(r["seconds"] for r in rs) / len(rs), total))
    lines += ["", "Each episode's checks and values are in episodes.jsonl. The graders measure the final scene with Pose Lab."]
    path = os.path.join(run_dir, "summary.md")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))
    print("\nwritten: %s" % run_dir)


if __name__ == "__main__":
    main()
