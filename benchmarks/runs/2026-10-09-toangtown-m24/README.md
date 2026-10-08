# ToangTown M24 magazine hold: 8 poses, 7 known faults

A real rig, not the sample. ToangTown's left hand holds an M24A2's box magazine on its FP_AKS74U first-person arms.
The rig files are licensed and not in this repository. `rigs.toangtown-m24.json` shows the entry used.

## The poses

ToangTown's `Tools/blender/pose_m24_maghold.py` made one good hold, then seven copies with one fault each
(`POSE_FAULTS=1`). `faults.json` names them.

| Pose | The fault put in |
|---|---|
| good | none |
| back | the back of the hand on the magazine |
| thumb | the thumb at its rest pose, 1.68 cm inside the magazine |
| twist | the forearm wrung: 154 degrees of twist between forearm and hand |
| finger | the index finger's middle joint bent 35 degrees sideways |
| clip | the palm pushed 1.5 cm into the magazine |
| elbow | the elbow over the shoulder |
| wrist | the wrist bent 55 degrees off the forearm |

## Results with Pose Lab 0.3.0 (`poselab_0.3.0.json`)

| Pose | Pose Lab 0.3.0 | ToangTown's checks (`toangtown_checks.json`) |
|---|---|---|
| good | flags the wrist at a reported 20 radial, its limit | clean |
| back | palm facing -1, not holding; `ok` stays true | palm facing -0.85 |
| thumb | clearance 0.68 cm (true depth 1.68); grip passes it | clip 1.68 cm |
| twist | missed | twist 154 |
| finger | missed: reads 3 to 7 sideways | caught: 48 sideways |
| clip | clearance 0.65 cm (true 1.46); grip fails | clip 1.46 cm |
| elbow | caught | caught |
| wrist | caught | caught |
| **Caught** | **5 of 7**, depth about 40% | 7 of 7, depth about 100% |

Time: `load_rig` 1.7 s, then 0.2 s per pose for `pose_clip`, `anatomy`, `grip` and `clearance`.

## What it found in Pose Lab

1. `clearance` measured to the nearest corner point of the rifle's mesh, not its surface. On a low-poly magazine it read
   0.68 cm where the hand was 1.68 cm in.
2. `anatomy` fitted each finger's plane through its own tip, so the plane followed a sideways kink and hid it.
3. `anatomy` had no check of the twist between forearm and hand.
4. `grip` called a hand "holding" when one fingertip touched, with the palm 3 to 5 cm off.
5. A `rigs.json` saved with a UTF-8 BOM (Windows PowerShell's default) failed to load.
6. A wrist reported at 20 failed its 20 degree limit: the rule compared the unrounded value.

Pose Lab 0.3.1 fixes all six. `poselab_0.3.1.json` holds the same run after the fixes.

## Run it

```
POSELAB_RIGS=<your copy of rigs.toangtown-m24.json> POSELAB_BLENDER=<blender> python benchmarks/runs/2026-10-09-toangtown-m24/run_bench.py out.json
```
