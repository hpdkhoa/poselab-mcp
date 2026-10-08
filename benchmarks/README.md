# Pose Lab benchmark

Does a model pose a first-person rig better with measured answers than with pictures alone? This benchmark gives a
model five tasks on the built-in sample rig, under two conditions, and grades the final scene with Pose Lab's own
measurements.

## Conditions

| Condition | Tools |
|---|---|
| `vision` | posing tools and renders only: `describe`, `pose_idle`, `pose_clip`, `move_part`, `move_gun`, `reach`, `snapshot`, `record_clip`, `render` |
| `measured` | the same, plus `where`, `distance`, `clearance`, `faces_eye`, `visible`, `screen`, `solve`, `load_clip`, `scan_clip`, `fix_clip`, `save_clip` |

The harness loads the rig and builds each task's starting scene. The model works until it calls `submit`. Then the
harness measures the scene and grades it. A claim in the model's answer never counts, except for the yes/no question.

## Tasks

| Task | What the model must do | Graded on |
|---|---|---|
| `port_to_eye` | Move the rifle so the eye can look into the ejection port. | port facing 0.5 or more, 60% visible, on screen, no clipping beyond the grips, hands kept on their grips |
| `turn_only` | Answer: can turns about the stock alone (no moves) achieve that? | the answer `impossible` |
| `forearm_clear` | An arm passes through the rifle; clear it without moving the rifle or the wrists. | no clipping, rifle and wrists kept still |
| `fingertip_contact` | Put the left index fingertip on a point given in gun coordinates. | the tip within 0.5 cm, nothing else more than 0.2 cm inside the rifle, rifle kept still |
| `clip_repair` | Mend a clip whose hands drift off the rifle and whose one frame jumps. | each hand within 0.5 cm of its grip, no hand faster than 250 cm/s, no clipping, same length and motion |

## Controls (free, no API calls)

* `--oracle` runs a scripted solution for each task. It shows that a solution exists for every task and that the
  graders accept a right answer. Result on 2026-10-08: 5 of 5.
* `--null` submits at once without changing anything. The graders must fail it. Result on 2026-10-08: 0 of 5.

## Results

Run on 2026-10-08 with Claude Code agents through `agent_bridge.py`, not through the API harness `run.py`. Three
trials of every task in each condition, 30 episodes in all. Pose Lab's graders measured every final scene.

| Task | measured | vision |
|---|---|---|
| `port_to_eye` | 3 / 3 | 0 / 3 |
| `turn_only` | 3 / 3 | 3 / 3 |
| `forearm_clear` | 3 / 3 | 3 / 3 |
| `fingertip_contact` | 3 / 3 | 3 / 3 |
| `clip_repair` | 3 / 3 | 2 / 3 |
| **all** | **15 / 15** | **11 / 15** |

| | measured | vision |
|---|---|---|
| Mean tool calls per episode | 17.3 | 34.8 |
| Mean seconds per episode | 45 | 153 |

What the numbers show:

* Measured agents passed every episode. Vision agents failed 4 of 15.
* Every vision fail was on something a picture cannot measure well. In `port_to_eye`, one agent left an arm 2.91 cm
  inside the rifle, and two misjudged how squarely the port faced the eye (0.459 and 0.295 against 0.5). Two of
  those three also pulled a hand off its grip (1.7 and 2.32 cm). In `clip_repair`, one agent's clip let the hands
  drift 1.71 and 1.14 cm off their grips (limit 0.5).
* Vision agents often said so themselves. Several reports say the agent could not see whether a part was a few
  millimetres inside the rifle.
* Vision agents also found ways round the missing measures. In `fingertip_contact`, all three worked out the
  shoulder position and the finger offset from the distances `reach` reports, and all three passed.
* Measured agents used half the tool calls and about a third of the time.

How strong the result is: the one-sided Fisher exact test on 15 of 15 against 11 of 15 gives p = 0.0498. That is
just under the usual 0.05 bar. Most of the difference comes from one task, `port_to_eye`. More trials would make the
size of the gap clearer.

Run details, from `runs/2026-10-08-claude-code/run.json`:

* Claude Code 2.1.289 sub-agents (`general-purpose`), model `claude-opus-5-5` in all 30 transcripts.
* The sub-agent's system prompt comes from Claude Code and is not saved in the transcripts, so its text is not
  recorded. The descriptions of the two Claude Code tools the agents used, Bash and Read, are in
  `claude_code_tools.md`.
* Pose Lab at commit `88b0ccc`.
* Audit: all 30 transcripts were read. Every file the agents opened was a render the tools returned. Two vision
  agents copied their own render with `cp`, and one ran inline Python as a calculator on numbers the tools returned.
  No agent read source code, benchmark files or results.

Known issue in this run: the `record_clip` description named `scan_clip` and `fix_clip`, which the vision condition
does not have. One vision agent noticed. It gives no measurement. 0.2.2 removes those names from the description.

`runs/2026-10-08-claude-code/` holds the summary, the run details and every episode's grade and tool calls. The
renders are left out to keep the repo small.

## Running it

You need Blender (`POSELAB_BLENDER`), the package installed (`pip install -e ".[bench]"`), and Anthropic credentials
(`ANTHROPIC_API_KEY`, or a profile from `ant auth login`).

```
python benchmarks/run.py --oracle
python benchmarks/run.py --null
python benchmarks/run.py --trials 1 --tasks port_to_eye
python benchmarks/run.py --trials 3
```

Model runs cost money. Start with one task and one trial: each episode prints its cost, and the summary adds them up.
The default model is `claude-opus-5-5` at effort `high`; change them with `--model` and `--effort`. On the Claude
models that take it, the harness sends server-side refusal fallbacks (`fallbacks: "default"`); `--no-fallbacks` turns
that off.

Each run writes `benchmarks/results/<time>-<model>/`: `episodes.jsonl` (every episode with each check's value) and
`summary.md` (pass rates per task and condition, mean tool calls, time and cost).

## Without an API key: agent_bridge.py

`agent_bridge.py` lets a coding agent that runs shell commands take the same tasks, for example a Claude Code
sub-agent on a Claude plan. A bridge keeps one Pose Lab session open on a local port and offers only the condition's
tools. It grades the scene with the same graders when the agent calls `submit`, or after 60 tool calls.

```
python benchmarks/agent_bridge.py serve port_to_eye measured 47902 benchmarks/results/agent-run/port_to_eye-measured
python benchmarks/agent_bridge.py prompt port_to_eye measured 47902
python benchmarks/agent_bridge.py summary benchmarks/results/agent-run
```

Start one bridge per episode, each on its own port. Give the text from `prompt` to a fresh agent. Each episode
folder gets `result.json` (the grade), `calls.jsonl` (every tool call) and the renders.

It differs from `run.py`:

* The agent works in its own harness. It calls tools through a command and opens renders as image files.
* The rule against reading the benchmark's files is an instruction, not a sandbox. Check `calls.jsonl` and the
  agent's transcript.
* No cost is recorded.

Use it to look at how agents work on the tasks. Use `run.py` for numbers you publish.

## Limits

* Five tasks on one sample rig. The pass rates say how a model does on these tasks, not on posing in general.
* The `turn_only` answer comes from Pose Lab's own search: no random sample out of 400 met every goal. That is strong
  evidence, not a proof. It holds only for turns about the stock: about the `bore` pivot, a roll of 115 degrees and a
  swing of 37 degrees meet every goal. An agent found that in a trial, which is why the prompt names the pivot.
* The `measured` condition includes the solver and the clip fixer, so it tests a model using Pose Lab, not a model's
  spatial reasoning alone.
