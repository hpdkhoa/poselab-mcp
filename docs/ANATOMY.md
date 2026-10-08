# The arm's joint ranges behind `anatomy`

Pose Lab's `anatomy` tool checks each arm against a human arm's limits. This page lists the research behind each
limit. The `arm_ranges` tool returns the same data to a model, with full citations.

Angles are degrees from the anatomical position: the arm straight at the side, the palm forward.

Three kinds of value appear here:

* **Standard**: the AAOS normal values, the clinical reference for a healthy adult's full range. Every standard value
  here is from the AAOS 1965 table as reprinted in Greene and Heckman 1994. Other published tables differ (see
  [The standard tables differ](#the-standard-tables-differ)).
* **Functional**: what people use in daily tasks, from motion studies. It is smaller than the full range. A range
  ("30 to 130") comes from one study. Where two studies each give one number, the table gives each its own column.
* **Checked**: what `anatomy` tests. Where it differs from the standard, the table says so.

## Shoulder

| Motion | Standard (AAOS) |
|---|---|
| Flexion | 180 |
| Extension | 60 |
| Abduction | 180 |
| Internal rotation | 70 |
| External rotation | 90 |

**Checked:** the elbow stays at least 2 cm below the shoulder (`elbow_under_shoulder_cm`).

This is a working rule, not a joint limit. A shooter keeps the upper arm below horizontal while the hand works the
gun. The shoulder can reach 180 degrees, but a raised elbow on a held rifle looks wrong and tires the arm.

## Elbow

| Motion | Standard (AAOS) | Functional (Morrey 1981) |
|---|---|---|
| Flexion | 150 | 30 to 130 |
| Extension | 0 | |

**Checked:** a hinge bent 5 to 150 degrees (`elbow_bend_min_deg`, `elbow_bend_max_deg`).

150 is the AAOS maximum. The 5 degree minimum is a working rule: people do not hold a rifle's weight on an elbow
locked fully straight. Soucie 2011 gives measured elbow values by age and sex.

## Forearm

| Motion | Standard (AAOS) | Functional (Morrey 1981) |
|---|---|---|
| Pronation | 80 | 50 |
| Supination | 80 | 50 |

**Checked:** no limit of its own. The radius turns over the ulna, so the hand's roll about the forearm's line belongs
to the forearm. The wrist check measures only the bend off that line, never the roll.

## Wrist

| Motion | Standard (AAOS) | Functional (Palmer 1985) | Functional (Ryu 1991) |
|---|---|---|---|
| Flexion | 80 | 5 | 54 |
| Extension | 70 | 30 | 60 |
| Radial deviation | 20 | 10 | 17 |
| Ulnar deviation | 30 | 15 | 40 |

**Checked:** each motion inside its AAOS value (`wrist_flexion_deg`, `wrist_extension_deg`, `wrist_radial_deg`,
`wrist_ulnar_deg`). The total bend off the forearm's line is at most 30 degrees (`wrist_max_deg`).

The 30 degree total is a working rule for this game's look of a hand on a gun. It is not a joint limit, and no study
gives it. It is stricter than daily tasks: Palmer found that daily tasks need about 5 flexion, 30 extension, 10 radial
and 15 ulnar. Ryu found that every task in the study fit within 54 flexion, 60 extension, 17 radial and 40 ulnar. The
sample rig's own idle grip breaks the rule (44 degrees left, 58 right). A rig sets its own value with
`"limits": {"wrist_max_deg": ...}` in `rigs.json`.

## Fingers

| Joint | Motion | Standard (AAOS) | Functional (Bain 2015) | Checked |
|---|---|---|---|---|
| Knuckle (MCP) | flexion | 90 | 19 to 71 | 100 |
| Knuckle (MCP) | hyperextension | 45 | | 30 |
| Middle joint (PIP) | flexion | 100 | 23 to 87 | 110 |
| Middle joint (PIP) | extension | 0 | | 5 past straight |
| End joint (DIP) | flexion | 90 | 10 to 64 | 90 |
| End joint (DIP) | hyperextension | 10 (Eaton lists 0) | | 10 |

Each joint also stays near the finger's own plane: at most 40 degrees off it at the knuckle, 15 at the middle joint and
25 at the end joint. The 40 at the knuckle is a working value with no source. It leaves room for the knuckle's own
spread (abduction), which the AAOS table does not list. The middle and end joints are hinges. Their side bend allows
only for a rig's bone axes.

**Where Pose Lab differs from AAOS:** it allows 10 more flexion at the knuckle and the middle joint, and 15 less
hyperextension at the knuckle.

**Why the defaults stay as they are:**

* The published tables differ by 10 to 25 degrees for the fingers, so no single strict value exists to match.
* The check exists to catch a finger no hand can make: bent backward, twisted, or curled past any table. It does
  not grade a normal grip against one table.
* Stricter defaults would fail grips that pass today in the game the rules come from (ToangTown), and no source
  shows those grips are wrong.

For the strict AAOS 1965 values, put this in the rig's entry in `rigs.json`:

```json
"finger_limits": {"01": [-45, 90, 40], "02": [0, 100, 15], "03": [-10, 90, 25]}
```

Hume 1990 measured the finger joints in daily tasks: the flexion used stayed well inside the AAOS ranges. Bain 2015
repeated the study with the Sollerman hand function test.

## Thumb

| Motion | Standard (AAOS) |
|---|---|
| Base joint (CMC) abduction | 70 (Eaton lists 45 palmar, 60 radial) |
| MCP flexion | 50 |
| IP flexion | 80 |

**Checked:** not yet. The thumb's base joint is a saddle with two axes, so it needs its own check.

## The standard tables differ

Normal range tables do not agree, most of all for the fingers and thumb, where they differ by 10 to 25 degrees:

| Joint | AAOS 1965 (Greene and Heckman 1994) | Eaton |
|---|---|---|
| Knuckle (MCP) flexion | 90 | 90 |
| Knuckle (MCP) hyperextension | 45 | 0 to 45 |
| Middle joint (PIP) flexion | 100 | 100 |
| End joint (DIP) flexion | 90 | 80 |
| End joint (DIP) hyperextension | 10 | 0 |
| Thumb base (CMC) abduction | 70 | 45 palmar, 60 radial |

Pose Lab uses the AAOS 1965 column. The thumb's base abduction varies the most.

## Changing the limits

A rig's `limits` and `finger_limits` entries in `rigs.json` replace any checked value. The keys are those in the
`limits` field of an `anatomy` reply.

## Sources

* **AAOS**: American Academy of Orthopaedic Surgeons. *Joint Motion: Method of Measuring and Recording*. Chicago:
  AAOS, 1965. Updated as Greene WB, Heckman JD (eds). *The Clinical Measurement of Joint Motion*. Rosemont: AAOS,
  1994.
* **Eaton**: Eaton C. Normal range of motion reference values. *The Electronic Textbook of Hand Surgery*.
  [eatonhand.com/nor/nor002.htm](https://www.eatonhand.com/nor/nor002.htm).
* **Soucie 2011**: Soucie JM, Wang C, Forsyth A, et al. Range of motion measurements: reference values and a database
  for comparison studies. *Haemophilia* 2011;17(3):500-507. Data:
  [CDC Joint Range of Motion Study](https://archive.cdc.gov/www_cdc_gov/ncbddd/jointrom/index.html).
* **Morrey 1981**: Morrey BF, Askew LJ, Chao EY. A biomechanical study of normal functional elbow motion. *J Bone
  Joint Surg Am* 1981;63(6):872-877.
* **Palmer 1985**: Palmer AK, Werner FW, Murphy D, Glisson R. Functional wrist motion: a biomechanical study. *J Hand
  Surg Am* 1985;10(1):39-46.
* **Ryu 1991**: Ryu J, Cooney WP, Askew LJ, An KN, Chao EY. Functional ranges of motion of the wrist joint. *J Hand
  Surg Am* 1991;16(3):409-419.
* **Hume 1990**: Hume MC, Gellman H, McKellop H, Brumfield RH. Functional range of motion of the joints of the hand.
  *J Hand Surg Am* 1990;15(2):240-243.
* **Bain 2015**: Bain GI, Polites N, Higgs BG, Heptinstall RJ, McGrath AM. The functional range of motion of the
  finger joints. *J Hand Surg Eur* 2015;40(4):406-411.
