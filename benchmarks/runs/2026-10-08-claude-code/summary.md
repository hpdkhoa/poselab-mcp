# Pose Lab benchmark (agent bridge)

Episodes run by coding agents through agent_bridge.py, graded by tasks.py. See agent_bridge.py for how this differs from run.py.

Run on:

* date: 2026-10-08
* harness: Claude Code 2.1.289 sub-agents (agent type general-purpose, background, non-interactive), through benchmarks/agent_bridge.py; not run through the API harness run.py
* model requested: opus (Claude Code alias)
* model resolved: claude-opus-5-5 in all 30 transcripts (675 model replies)
* sub-agent system prompt: set by Claude Code 2.1.289 for the general-purpose agent type; not saved in the transcripts, so its text is not recorded
* Claude Code tools used: Bash (Pose Lab calls through the bridge) and Read (render PNGs); their descriptions are in claude_code_tools.md
* Pose Lab: commit 88b0ccc (0.2.1 plus the fixes listed under Next in CHANGELOG.md)
* trials: 3 per task and condition, run in 3 rounds of 10 episodes
* call limit: 60 tool calls per episode
* audit: All 30 transcripts read. 120 file reads, all render PNGs the tools returned. Commands: bridge calls (Bash 273, PowerShell 3), sometimes piped through head or grep. Deviations: two vision agents copied their own render PNG with cp (no new information); fingertip_contact vision trial 2 ran inline Python as a calculator to fit the shoulder position from reach distances the tools returned (no file read). No agent read source code, benchmark files or results.

| Task | measured | vision |
|---|---|---|
| clip_repair | 3 / 3: PASS (9 calls), PASS (8 calls), PASS (8 calls) | 2 / 3: PASS (56 calls), fail (56 calls), PASS (58 calls) |
| fingertip_contact | 3 / 3: PASS (9 calls), PASS (9 calls), PASS (11 calls) | 3 / 3: PASS (34 calls), PASS (31 calls), PASS (35 calls) |
| forearm_clear | 3 / 3: PASS (29 calls), PASS (22 calls), PASS (23 calls) | 3 / 3: PASS (44 calls), PASS (28 calls), PASS (26 calls) |
| port_to_eye | 3 / 3: PASS (37 calls), PASS (15 calls), PASS (22 calls) | 0 / 3: fail (23 calls), fail (31 calls), fail (35 calls) |
| turn_only | 3 / 3: PASS (30 calls), PASS (6 calls), PASS (22 calls) | 3 / 3: PASS (11 calls), PASS (30 calls), PASS (24 calls) |
| **all** | 15 / 15 | 11 / 15 |

Failed checks:

* clip_repair, vision, trial 1: hold l = 1.71
* clip_repair, vision, trial 1: hold r = 1.14
* port_to_eye, vision, trial 0: clearance = 2.91
* port_to_eye, vision, trial 0: hands kept their grip (cm) = 1.7
* port_to_eye, vision, trial 1: faces_eye = 0.459
* port_to_eye, vision, trial 1: hands kept their grip (cm) = 2.32
* port_to_eye, vision, trial 2: faces_eye = 0.295
* port_to_eye, vision, trial 2: visible = 0.235
