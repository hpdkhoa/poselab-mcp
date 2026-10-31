# Numbers with meaning: plan

Status: plan only. Nothing built. The M24 work is paused (the W choice is open).

## The goal: the owner's words

The owner chose these three sentences as the goal of this plan (2026-10-31: "do with these words").

| # | Date | The owner's words |
|---|---|---|
| G1 | 2026-10-19 | "in real life that will not happen" (about clipping and twisted fingers; it made both hard rules) |
| G2 | 2026-10-29 | "tell me if I want to make a virtual world like matrix idea where every thing every poly control by numbers, start from the hand and the gun is possible or not?" |
| G3 | 2026-10-31 | "I want to improve the vision of machine by using numbers that have meaning to really appreciate physics in digital world so after the physics and object interact is correct I can start the creation automatically" |

What each one asks of the plan:

| # | Asks for | Where the plan builds it |
|---|---|---|
| G1 | What cannot happen in real life never happens in the result: no clipping, no twisted finger, no joint past its range. These are walls. | Step 2 (walls), Step 4 (a hold that would slip in real life fails), Step 5 (walls hard at every step) |
| G2 | Every thing and every poly controlled by numbers. Start from the hand and the gun. | Step 1 (every poly of the gun belongs to a named part), Step 2 (every part of the hand measured) |
| G3 | Numbers with meaning; physics and object interaction correct first; creation automatic after. | The record rule below, Steps 3 to 5 (interaction and physics), Step 6 (creation, only after) |

Other words of the owner on this plan (2026-10-31): "write the plan document first with my sentence." and "I see
these points sounds good but still don't see how you create, what it will behave and keep this design work
correctly". So each step says how it is created, how it behaves, and how it stays correct.

## The one rule under every step

Every number is a record, never a bare float. A record holds:

| Field | Example |
|---|---|
| value, unit | 1.62 cm |
| meaning | depth of the grip back crook under the stock |
| of what | M24 part `grip_back` |
| kind | point, line or surface |
| source | measured: M24 middle-plane cut, 0.1 cm samples; or a named paper or table |
| check | the range it must sit in, and what happens when it does not |

The code refuses a number without a meaning and a source. This is the owner's rule "In 3D, measure exact numbers"
made into a type.

Why: both M24 failures came from numbers without meaning. In wt1, a ray hit the guard and its "under the stock"
check passed, because the code saw triangles and not parts. In solve3, four contacts were met but the hand was
upside down, because no number said which way the hand wraps.

## Step 1. Meaning on the object

How it is created:
1. Model Lab already names mesh parts in its parts table. A labeler adds a role to each part: `grip_back`,
   `grip_side_l`, `guard`, `trigger_blade`, `top`, `forend`, `bore_axis`.
2. The labeler proposes roles from geometry: the middle-plane cut (closed chains), the bore direction, the trigger
   ring, the face normals.
3. Each role has a written definition. Example: `guard` is a closed loop around the trigger. `grip_back` is behind
   the guard, under the stock top, facing back and down.
4. No person approves the labels. A label is accepted when it passes its definition check (below). The owner
   only reviews the final result (2026-10-31: "I only need to review the final result").
5. Each part gets measured values: its outline, its curve, its normal, its width, and its material with a friction
   value from a named source.

How it behaves: the hold asks "where is `grip_back` on the M24" and gets a surface with its numbers. It never asks
"what does a ray hit".

How it stays correct:
- G2 "every poly": every polygon of the gun belongs to exactly one named part. A polygon with no part stops the run
  and is reported with its place.
- Each label is tested against its definition with numbers. A label that misses its definition stops the run. A
  guard that is not a closed loop around the trigger is not a guard.
- The accepted labels are saved in a cache file per gun. A changed mesh makes the cache stale and labels again.

## Step 2. Meaning on the body

How it is created:
1. Measure the posed skin mesh: segment lengths, pad and palm thickness, the web saddle, finger widths.
2. Take the joint limits from ANATOMY.md and their sources. These are walls.
3. Take skin give from `skin_tissue_sources.json`.
4. Compute the reach envelope: where each fingertip, pad and the web can go inside the walls.

How it behaves: a query such as "can the index pad reach `trigger_blade` with the web on `grip_back`" returns yes
or no with the distance and the joint that limits it. The M24 W2 crook (index 14.4 cm from B, reach 9.84) is
answered before any hand is placed.

How it stays correct:
- Each measured value is compared with an anthropometry source range. A value outside that range stops the run.
- The values sit in a cache file with the mesh they came from.

## Step 3. Contact rules

How it is created: one rule file per hold type (pistol grip, rifle stock grip, forend, magazine). Each rule says:
- which hand part touches which gun part (index pad to `trigger_blade`, web to `grip_back`, thumb pad to `top`
  near A)
- its kind (point, line or surface) and the source of that kind
- the wrap direction: the fingers curl around the gun's side toward the far side, and the thumb closes from the
  other side. This is the missing fifth definition from solve3.
- the owner's sentence it comes from, word for word

How it behaves: the solver reads the rules. It does not invent contacts. A hand part with no rule must have a
written reason to be free, for example "pinky free on a short grip".

How it stays correct:
- Completeness check: every hand part has a rule or a reason.
- Wrap check: the sign of each finger's curl axis against the bore. A hand placed upside down fails this check
  before anything else runs.

## Step 4. Force check (new, the physics)

How it is created: a static balance on the gun.
- Known: the gun's weight at its centre of mass (from the mesh volume and the material densities, or the real
  gun's listed weight), and the recoil push (from `recoil_sources.json` and the recoil paper).
- Unknown: the force at each contact from Step 3.
- Equations: the sum of forces is 0 and the sum of torques is 0. Each contact only pushes (force ≥ 0). Friction
  holds only inside its cone (side force ≤ friction × push). Each finger force stays under a sourced grip limit.
- Solve: a small linear program finds the contact forces, or says no set exists.

How it behaves: the result for each contact is its push, its side force and its margin. The margin says how much
more push the hand can take before the gun slips. A hold where the gun would slip or turn fails with the contact
that gives way and its number.

How it stays correct:
- First, cases with known answers: a block resting on a table, a two-finger pinch, a cylinder in a fist. The
  solver must match the hand-calculated answer.
- Then the M1911A1, the owner's quality bar: it must pass.
- A broken hold (fingers lifted, thumb in the air) must fail.

## Step 5. Solve and gate

How it is created: the existing gate (hold_gate, Gun Kit rule 11) gets two new checks: the Step 3 rules and the
Step 4 force balance. The solver places the hand to meet the Step 3 rules. The walls are hard limits at every step.

How it behaves: every number in the report prints with its record: "web to `grip_back` 0.2 cm (touch: ≤ 0.5),
wrist 18 deg (wall 30), thumb push 12 N (margin 40 %)". The picture marks each one.

How it stays correct: a judged set. The set holds poses the owner has judged:
- good: the M1911A1 hold, the AR-15 clips (POSE_RULES.md)
- bad: M24 solve3 (upside down), M24 wt1 (W on the guard), plan16 (wrist 94)

Every code change runs the whole set. A good pose must pass and a bad pose must fail. If the numbers pass a pose the
owner judged bad, a rule is missing. I bring it to the owner. I do not tune it away. The set grows with every
judgment the owner gives.

## Step 6. Automatic creation

Starts only when Steps 1 to 5 pass on one gun the owner approves.

How it is created:
1. A new gun mesh comes in.
2. Step 1 labels it. Labels that fail their definition stop the run, with numbers.
3. Step 3 picks the hold rules by gun type.
4. Step 5 solves, the gate and the force check run.
5. The pictures go out with markers.
6. The owner reviews the final result. This is the only person step.

For clips, the same flow runs at each key moment on the timeline.

How it behaves: no hand-tuned numbers per gun. A gun that cannot be held under the rules is reported with the rule
that fails and its numbers.

How it stays correct: the judged set from Step 5 runs on every new gun too. Each new gun the owner judges joins it.

## Step 7. Judgments and the gate

The owner's words (2026-10-31): "so how can you prevent the conflict in taste and when scale up that multiple person
use this and the data still can be reuse to get this tool better? and how to eval and create smarter gate with "each
of your judgments turns into a number for next time"?" Then: "add it".

### 7.1 Physics and taste kept apart

| Kind | Example | Who decides | Can it conflict |
|---|---|---|---|
| Wall (G1, real life) | clipping, twisted finger, joint past its range, slip under recoil | numbers only, the same for everyone | no |
| Rule from a mistake | hand upside down, W on the guard | numbers, once a judgment proves the rule | no, it is wrong for everyone |
| Taste | thumb over the top or thumb forward, how high the hand sits | a named style | yes, allowed |

- Taste never overrides a wall.
- Two people who judge the same pose differently make two styles, not one rule. Each person picks a style.
- A judgment becomes a shared rule only when every reviewer agrees on it.

### 7.2 Every judgment is a full record

| Field | Why |
|---|---|
| object, pose (all joint values), code version | the pose can be rebuilt exactly |
| verdict (pass or fail), reviewer, style | taste is kept per person |
| the marked spot and a reason tag ("upside down", "off the grip") | the judgment says what is wrong, not only that it is wrong |
| the measured numbers at that time | what the gate saw |

The pose is rebuilt from the record, so a new measure added later is computed on every old judgment. Old judgments
never go stale, and the data set grows for every user.

### 7.3 A judgment becomes a number

1. The owner marks a fail on a final result.
2. I search the measured numbers for one that separates this pose from all good ones.
3. I propose it as a new check with a threshold and a margin.
4. The new check must still pass every good pose and fail every bad pose in the judged set.
5. The owner approves the new rule before it goes in. A new rule changes the method, so it is the owner's call.

| The owner's judgment | The number that separates it | New check |
|---|---|---|
| solve3: hand upside down | the fingers' curl direction against the bore (good poses positive, solve3 negative) | curl direction positive |
| wt1: W on the guard | the part W sits on is `guard` | W on `grip_back` |

### 7.4 Eval of the gate

| Measure | Target |
|---|---|
| False pass: the gate passes a pose a person failed | 0, the main measure |
| False fail: the gate fails a pose a person passed | low |
| Agreement on held-out judgments (not used to make the rule) | rising over time |
| Agreement between reviewers | shows how much is taste, not physics |

When there are many judgments, a learned scorer can rank poses within a style. It works on taste only, never on
walls.

## What this plan does not claim

- The numbers do not replace the owner's eye yet. Solve3 passed its own numbers and was wrong. The judged set
  measures how close a numbers pass is to an owner pass.
- Values still to find from sources: the friction of glove on wood and on polymer, finger force limits, and the
  stock material densities.

## Choices for the owner

1. Where the new code lives: Pose Lab (body, walls), Model Lab (parts), fakepose (holds), or one new lab for force.
2. Which gun goes first: the M1911A1 (already approved) or the M24 (still open).
3. Which step goes first: Step 1 with the judged set, or Step 4 force on the M1911A1.
