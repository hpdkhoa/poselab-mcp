# PoseFault: poses with known faults, for "Measure, don't judge"

PoseFault tests one question: does a VLM that looks at renders find physical pose faults as well as Pose Lab, which
measures them? Each pose carries one fault that a script put in, at a known size, so no person and no tool labels it.

## Files

| File | What it does |
|---|---|
| `generate.py` | Blender script. It takes base poses, keeps each side that passes every rule, injects one fault per pose at each size, and writes the poses, the labels and four renders per pose. |
| `measure.py` | Blender script, arm C. Pose Lab alone checks each pose: flagged (any rule broken on that side) and named (a flag names the injected fault). |
| `judge.py` | Arm A. A VLM sees the four renders and answers in JSON with a fixed list of fault types. Backends: Anthropic API or a local Ollama model. |

## The faults

| Fault | What the script does | Sizes |
|---|---|---|
| `wrist_bend` | turns the hand about the knuckle line, the way that bends the wrist more | 45, 60, 80 degrees |
| `wrist_twist` | turns the hand about the forearm's line | 60, 100, 150 degrees |
| `back_of_hand` | turns the hand over about its own long axis | 180 degrees |
| `index_side`, `middle_side` | bends the middle joint sideways, out of the finger's plane | 30, 45, 60 degrees |
| `index_hyper`, `middle_hyper` | sets the middle joint to this many degrees bent backward | 25, 45 degrees |
| `palm_clip` | moves the palm's skin this far past the rifle's nearest surface (the arm follows, the elbow keeps its side) | 2, 3, 4 cm |
| `elbow_high` | swings the elbow up and out over the shoulder, the hand kept in place | one size |

Controls on every base: the base itself (`clean`) and three small changes inside every limit (`wrist_nudge` 8
degrees, `index_curl` 8 degrees, `hand_back` 0.3 cm off the rifle).

## Bases

A base is a rig, a clip or the idle grip, a time and a side. `generate.py` measures each base first. If the game pose
breaks a rule, the script mends a copy: it moves the hand's twist into the forearm, turns the wrist and fingers back
inside their ranges, and pulls the hand off the rifle 0.25 cm at a time, 2 cm at most. A side that still breaks a rule
is not used. `bases.jsonl` keeps each game pose's own report.

Caution: Pose Lab certifies the bases as clean, so the clean controls cannot show a Pose Lab false alarm on their own.
The benign controls and a check of the base renders by eye cover that.

## Run

```
blender -b --factory-startup -P benchmarks/posefault/generate.py -- <rigs file> <out folder>
blender -b --factory-startup -P benchmarks/posefault/measure.py -- <rigs file> <out folder>
python benchmarks/posefault/judge.py <out folder> --backend anthropic --model claude-opus-5-5 --limit 5
```

The ToangTown rigs (AR-15, AK-47 and M24 holds) are licensed and stay out of this repository. The built-in sample rig
runs with an empty rigs file argument.
