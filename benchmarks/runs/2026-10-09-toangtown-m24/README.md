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

## Results (`poselab_0.3.0.json`, `poselab_0.3.1.json`, `toangtown_checks.json`)

| Pose | Pose Lab 0.3.0 | Pose Lab 0.3.1 | ToangTown's checks |
|---|---|---|---|
| good | flags the wrist at a reported 20 radial, its limit | clean (grip: palm 4.7 cm off, not holding) | clean |
| back | palm facing -1, not holding; `ok` stays true | wrist twisted 179; palm facing -1 | palm facing -0.85 |
| thumb | clearance 0.68 cm (true depth 1.68); grip passes it | clearance 1.68 cm; grip fails | clip 1.68 cm |
| twist | missed | wrist twisted 154 | twist 154 |
| finger | missed: reads 3 to 7 sideways | end joint 48 out of plane | 48 sideways |
| clip | clearance 0.65 cm (true 1.46); grip fails | clearance 1.46 cm; grip fails | clip 1.46 cm |
| elbow | caught | caught | caught |
| wrist | caught | caught | caught |
| **Caught** | **5 of 7**, depth about 40% | **7 of 7**, depth exact | 7 of 7, depth exact |

Time: `load_rig` 1.7 s (0.3.0) and 2.2 s (0.3.1), then about 0.2 and 0.27 s per pose for `pose_clip`, `anatomy`,
`grip` and `clearance`.

0.3.1's `grip` also says the good pose's palm is 4.7 cm off the magazine: the fingers and thumb touch it, the palm
does not. On this rig the palm's skin lies deeper than the 1.8 cm capsule, so the true gap is smaller, but the hold
is fingers and thumb, not palm. That is a finding about the pose, not a false alarm.

## What it found in Pose Lab

1. `clearance` measured to the nearest corner point of the rifle's mesh, not its surface. On a low-poly magazine it read
   0.68 cm where the hand was 1.68 cm in.
2. `anatomy` fitted each finger's plane through its own tip, so the plane followed a sideways kink and hid it.
3. `anatomy` had no check of the twist between forearm and hand.
4. `grip` called a hand "holding" when one fingertip touched, with the palm 3 to 5 cm off.
5. A `rigs.json` saved with a UTF-8 BOM (Windows PowerShell's default) failed to load.
6. A wrist reported at 20 failed its 20 degree limit: the rule compared the unrounded value.

Pose Lab 0.3.1 fixes all six. `examples/anatomy_faults.py` checks each on the built-in sample.

## Run it

```
POSELAB_RIGS=<your copy of rigs.toangtown-m24.json> POSELAB_BLENDER=<blender> python benchmarks/runs/2026-10-09-toangtown-m24/run_bench.py out.json
```
