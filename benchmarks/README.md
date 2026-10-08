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
| `turn_only` | Answer: can turns alone (no moves) achieve that? | the answer `impossible` |
| `forearm_clear` | An arm passes through the rifle; clear it without moving the rifle or the wrists. | no clipping, rifle and wrists kept still |
| `fingertip_contact` | Put the left index fingertip on a point given in gun coordinates. | the tip within 0.5 cm, nothing else more than 0.2 cm inside the rifle, rifle kept still |
| `clip_repair` | Mend a clip whose hands drift off the rifle and whose one frame jumps. | each hand within 0.5 cm of its grip, no hand faster than 250 cm/s, no clipping, same length and motion |

## Controls (free, no API calls)

* `--oracle` runs a scripted solution for each task. It shows that a solution exists for every task and that the
  graders accept a right answer. Result on 2026-10-08: 5 of 5.
* `--null` submits at once without changing anything. The graders must fail it. Result on 2026-10-08: 0 of 5.

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

## Limits

* Five tasks on one sample rig. The pass rates say how a model does on these tasks, not on posing in general.
* The `turn_only` answer comes from Pose Lab's own search: no random sample out of 120 met every goal. That is strong
  evidence, not a proof.
* The `measured` condition includes the solver and the clip fixer, so it tests a model using Pose Lab, not a model's
  spatial reasoning alone.
