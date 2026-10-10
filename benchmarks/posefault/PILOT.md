# PoseFault v0 pilot, 2026-10-10

A first pass to check that the pipeline works end to end. The sample is too small to support a claim in a paper.

## The set

`generate.py` made 125 poses from 5 bases: the sample rig's left idle grip, ToangTown's AR-15 check (left hand), its
AK-47 check and charge (right hand) and its M24 magazine hold (left hand). Every base gives 1 clean pose, 3 benign
changes and 21 faulty poses (9 faults at their sizes).

Only the M24 hold passed every rule as the game has it. Pose Lab flagged the other 13 game sides it looked at. The
script mended four of those into bases (`bases.jsonl` keeps each game pose's own report). The game files did not
change.

## Arm C: Pose Lab alone, all 125 poses

| | Poses | Flagged | Named the fault |
|---|---|---|---|
| Faulty | 105 | 105 | 105 |
| Clean | 5 | 0 | |
| Benign | 15 | 1 | |

The one benign flag is the M24 `wrist_nudge`. That base sits at the 20 degree radial limit. The 8 degree nudge goes the
way that keeps the fingers out of the magazine, and that way ends at 21 degrees radial. So by the rule the label is
wrong, not the reading.

## Arm A: a VLM judging renders, 20 poses

The judge was Claude Opus 5.5, run as 4 Claude Code sub-agents with 5 poses each. Each agent read only the four
renders of each pose, under anonymous names (p01 to p20). The prompt is `judge.py`'s.

| | Poses | VLM flagged | VLM named the fault | Pose Lab flagged and named |
|---|---|---|---|---|
| Faulty | 15 | 10 | 3 | 15 |
| Clean | 5 | 1 (false alarm) | | 0 |

It named the fault correctly for `elbow_high` (2 of 2) and one `wrist_bend` (1 of 2). It missed every sideways
finger (0 of 3 flagged), one of two backs of the hand, and the backward finger at 45 degrees. It flagged both palm clips
but called them a bent wrist and a backward finger.

Limits of this pilot: 20 poses, one model and one prompt. Pose Lab certified the bases itself. The renders are
640 x 360 in a grey Workbench look. Answers: `arm_a-claude_code-opus-5-5-pilot.jsonl` in the run folder.
