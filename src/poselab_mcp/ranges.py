"""The human arm's joint ranges Pose Lab's anatomy rules rest on, with their sources.

Each joint lists the clinical standard (the AAOS normal values), what people use in daily tasks (from motion
studies), and the value Pose Lab checks. Where Pose Lab's value differs from the standard, the entry says so and why.
Angles are degrees from the joint's neutral (0: the anatomical position, arm straight, palm forward).

"Standard" means the AAOS 1965 table as reprinted in Greene and Heckman 1994. Published normal tables do not agree:
for the fingers and thumb they differ by 10 to 25 degrees (tables_differ), so a standard value here is that one
table's value, not a single agreed number.

"functional" is a range people use, from one study, as [low, high]. "functional_by_study" gives one value from each
study (each the most the study's tasks needed), not a range.
"""

SOURCES = {
    "AAOS": "American Academy of Orthopaedic Surgeons. Joint Motion: Method of Measuring and Recording. Chicago: AAOS, "
            "1965. Updated as Greene WB, Heckman JD (eds). The Clinical Measurement of Joint Motion. Rosemont: AAOS, "
            "1994.",
    "Soucie2011": "Soucie JM, Wang C, Forsyth A, et al. Range of motion measurements: reference values and a database "
                  "for comparison studies. Haemophilia 2011;17(3):500-507. Data: CDC Joint Range of Motion Study, "
                  "https://archive.cdc.gov/www_cdc_gov/ncbddd/jointrom/index.html",
    "Morrey1981": "Morrey BF, Askew LJ, Chao EY. A biomechanical study of normal functional elbow motion. J Bone Joint "
                  "Surg Am 1981;63(6):872-877.",
    "Palmer1985": "Palmer AK, Werner FW, Murphy D, Glisson R. Functional wrist motion: a biomechanical study. J Hand "
                  "Surg Am 1985;10(1):39-46.",
    "Ryu1991": "Ryu J, Cooney WP, Askew LJ, An KN, Chao EY. Functional ranges of motion of the wrist joint. J Hand Surg "
               "Am 1991;16(3):409-419.",
    "Hume1990": "Hume MC, Gellman H, McKellop H, Brumfield RH. Functional range of motion of the joints of the hand. "
                "J Hand Surg Am 1990;15(2):240-243.",
    "Eaton": "Eaton C. Normal range of motion reference values. The Electronic Textbook of Hand Surgery, "
             "https://www.eatonhand.com/nor/nor002.htm",
    "Bain2015": "Bain GI, Polites N, Higgs BG, Heptinstall RJ, McGrath AM. The functional range of motion of the finger "
                "joints. J Hand Surg Eur 2015;40(4):406-411.",
}

TABLES_DIFFER = ("Normal range tables differ for the fingers and thumb. AAOS 1965 (reprinted in Greene and Heckman "
                 "1994): knuckle 90, middle joint 100, end joint 90 with 10 of hyperextension, thumb base abduction 70. "
                 "Eaton: end joint 80 with 0 of hyperextension, thumb base abduction 45 (palmar) or 60 (radial). "
                 "The thumb's base abduction varies the most.")

# Why the finger defaults are not the strict AAOS values (a rig's finger_limits can set them)
FINGER_DEFAULTS = ("The defaults allow 10 more flexion than AAOS at the knuckle and middle joint, and 15 less "
                   "hyperextension at the knuckle. They stay as they are for three reasons. The published tables "
                   "differ by 10 to 25 degrees, so no single strict value exists to match. The check exists to catch a "
                   "finger no hand can make (bent backward, twisted, curled past any table), not to grade a normal "
                   "grip against one table. And stricter defaults would fail grips that pass today in the game the "
                   "rules come from (ToangTown), with no source showing those grips are wrong. Set finger_limits "
                   "{'01': [-45, 90, 40], '02': [0, 100, 15], '03': [-10, 90, 25]} for the strict AAOS 1965 values.")

# joint -> motion -> {"standard": AAOS normal maximum, "functional": the range daily tasks use (and its source),
# "checked": what Pose Lab's anatomy tool tests, "note": why the check differs from the standard, if it does}
RANGES = {
    "shoulder": {
        "flexion": {"standard": 180, "source": "AAOS"},
        "extension": {"standard": 60, "source": "AAOS"},
        "abduction": {"standard": 180, "source": "AAOS"},
        "internal_rotation": {"standard": 70, "source": "AAOS"},
        "external_rotation": {"standard": 90, "source": "AAOS"},
        "checked": "the elbow at least 2 cm below the shoulder (elbow_under_shoulder_cm)",
        "note": "A working rule, not a joint limit: a shooter keeps the upper arm below horizontal while the hand works "
                "the gun. The shoulder can reach 180 degrees of flexion, but a raised elbow on a held rifle looks wrong "
                "and tires the arm.",
    },
    "elbow": {
        "flexion": {"standard": 150, "source": "AAOS", "functional": [30, 130], "functional_source": "Morrey1981"},
        "extension": {"standard": 0, "source": "AAOS"},
        "checked": "a hinge bent 5 to 150 degrees (elbow_bend_min_deg, elbow_bend_max_deg)",
        "note": "150 is the AAOS maximum. The 5 degree minimum is a working rule: an elbow locked fully straight under "
                "a rifle's weight is not a pose people hold. Soucie2011 gives measured values by age and sex.",
    },
    "forearm": {
        "pronation": {"standard": 80, "source": "AAOS", "functional": 50, "functional_source": "Morrey1981"},
        "supination": {"standard": 80, "source": "AAOS", "functional": 50, "functional_source": "Morrey1981"},
        "checked": "not a limit of its own: the hand's roll about the forearm's line belongs here, not to the wrist",
        "note": "The radius turns over the ulna, so a rolled hand is a turned forearm. The wrist check measures only "
                "the bend off the forearm's line, never this roll. The wrist itself has no twist: anatomy fails a hand "
                "twisted more than 30 degrees against its forearm's bone beyond the idle grip (wrist_twist_max_deg, a "
                "working value: a skinned forearm without twist bones wrings past it).",
    },
    "wrist": {
        "flexion": {"standard": 80, "source": "AAOS", "functional_by_study": {"Palmer1985": 5, "Ryu1991": 54}},
        "extension": {"standard": 70, "source": "AAOS", "functional_by_study": {"Palmer1985": 30, "Ryu1991": 60}},
        "radial_deviation": {"standard": 20, "source": "AAOS", "functional_by_study": {"Palmer1985": 10, "Ryu1991": 17}},
        "ulnar_deviation": {"standard": 30, "source": "AAOS", "functional_by_study": {"Palmer1985": 15, "Ryu1991": 40}},
        "checked": "each motion inside its AAOS value (wrist_flexion_deg, wrist_extension_deg, wrist_radial_deg, "
                   "wrist_ulnar_deg), and the total bend off the forearm's line at most 30 degrees (wrist_max_deg)",
        "note": "The 30 degree total is a working rule for this game's look of a hand on a gun, not a joint limit "
                "and not from a study. It is stricter than daily tasks: Palmer1985 found daily tasks need 5 flexion, "
                "30 extension, 10 radial and 15 ulnar; Ryu1991 found every task done within 54 flexion, 60 extension, "
                "17 radial and 40 ulnar. The sample rig's own idle grip breaks it (44 degrees left, 58 right). A rig "
                "sets its own value with limits {'wrist_max_deg': ...}.",
    },
    "finger_mcp": {
        "flexion": {"standard": 90, "source": "AAOS", "functional": [19, 71], "functional_source": "Bain2015"},
        "hyperextension": {"standard": 45, "source": "AAOS"},
        "checked": "the knuckle curls -30 to 100, at most 40 out of the finger's plane (finger_limits '01')",
        "note": "Pose Lab allows 10 more flexion than AAOS, and 15 less hyperextension (why: finger_defaults). For "
                "the strict AAOS values set finger_limits {'01': [-45, 90, 40]}. The 40 degree side bend is a working "
                "value with no source: it leaves room for the knuckle's own spread (abduction), which the AAOS table "
                "does not list.",
    },
    "finger_pip": {
        "flexion": {"standard": 100, "source": "AAOS", "functional": [23, 87], "functional_source": "Bain2015"},
        "extension": {"standard": 0, "source": "AAOS"},
        "checked": "the middle joint curls -5 to 110, at most 15 out of the finger's plane (finger_limits '02')",
        "note": "Pose Lab allows 10 more flexion than AAOS and 5 of slack past straight (why: finger_defaults). For "
                "the strict AAOS values "
                "set finger_limits {'02': [0, 100, 15]}. A hinge has no side bend; 15 allows for a rig's bone axes.",
    },
    "finger_dip": {
        "flexion": {"standard": 90, "source": "AAOS", "functional": [10, 64], "functional_source": "Bain2015"},
        "hyperextension": {"standard": 10, "source": "AAOS", "note": "AAOS 1965 lists 10; Eaton lists 0 (flexion 80)"},
        "checked": "the end joint curls -10 to 90, at most 25 out of the finger's plane (finger_limits '03')",
        "note": "Matches the AAOS 1965 table (other tables differ: tables_differ). The end bone's tip is derived "
                "from the middle bone's line, since a rig keeps no fingertip, so its side bend allows more slack.",
    },
    "thumb": {
        "cmc_abduction": {"standard": 70, "source": "AAOS", "note": "the most varied value: Eaton lists 45 (palmar) "
                          "or 60 (radial)"},
        "mcp_flexion": {"standard": 50, "source": "AAOS"},
        "mcp_hyperextension": {"standard": 10, "source": "Eaton"},
        "ip_flexion": {"standard": 80, "source": "AAOS"},
        "ip_hyperextension": {"standard": 15, "source": "Eaton"},
        "checked": "the knuckle (MCP) bends -10 to 60 and the end joint (IP) -15 to 90, each at most 30 and 25 out of its "
                   "hinge's plane (thumb_limits '02', '03'); the base joint (CMC) spreads at most 80 from the index "
                   "metacarpal (cmc_spread_max_deg) and sits at most 20 behind the palm's plane (cmc_palmar_min_deg)",
        "note": "The hinge ranges take Eaton's hyperextension and the AAOS flexion plus the 10 degrees of slack the "
                "fingers get (why: finger_defaults). The base joint is a saddle with two axes, and no table gives its "
                "range as the angle between two bones, so its two limits are working values with no source: 80 "
                "leaves room above the AAOS 70 for the bones' angle at rest (the sample's thumb rests at 36), and 20 "
                "behind the palm allows the thumb's own extension. The thumb's turn about its own line (opposition) "
                "is not checked.",
    },
}

FUNCTIONAL_HAND = ("Hume1990 measured the finger joints in daily tasks: the flexion used stayed well inside the "
                   "AAOS ranges. Bain2015 repeated it with the Sollerman test: MCP 19 to 71, PIP 23 to 87, DIP 10 "
                   "to 64.")


def arm_ranges():
    return {"angles": "degrees from the anatomical position (arm straight, palm forward)", "joints": RANGES,
            "functional_hand": FUNCTIONAL_HAND, "tables_differ": TABLES_DIFFER, "finger_defaults": FINGER_DEFAULTS,
            "sources": SOURCES,
            "override": "a rig's limits, finger_limits and thumb_limits entries in rigs.json replace the checked values"}
