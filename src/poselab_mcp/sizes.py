"""The human arm and hand's sizes Pose Lab's body_size rules rest on, with their source.

Every range comes from ANSUR II, the 2012 anthropometric survey of U.S. Army personnel (4,082 men, 1,986 women,
measured by hand to the survey's own handbook). The numbers here are computed from the survey's public data files:
the mean, standard deviation and the 1st, 5th, 50th, 95th and 99th percentiles, in centimetres. A rig's measure
is regular when it lies between the 1st and 99th percentile of the character's sex, widened only by the landmark
allowance the entry states.

ANSUR measures between skin landmarks (the acromion on top of the shoulder, the radiale at the top of the radius,
the stylion at the tip of the radial styloid); a rig measures between joint centres. Where the two differ, the
entry gives the allowance and why, so nothing is hidden. ANSUR II has no palm thickness and no finger widths: the
hand's thickness is held by its circumference round the knuckles (a palm twice too thick has a circumference far
past the 99th percentile), and the fingers are not checked by size.
"""

SOURCES = {
    "ANSUR II": "Gordon CC, Blackwell CL, Bradtmiller B, et al. 2012 Anthropometric Survey of U.S. Army Personnel: "
                "Methods and Summary Statistics. Technical Report NATICK/TR-15/007. Natick, MA: U.S. Army Natick "
                "Soldier Research, Development and Engineering Center, 2014. Public data files ANSUR_II_MALE_Public.csv "
                "and ANSUR_II_FEMALE_Public.csv (Penn State OPEN Lab mirror, https://www.openlab.psu.edu/ansur2/).",
}

# cm; each {sex: [mean, sd, p1, p5, p50, p95, p99]}, n 4082 men, 1986 women
_STATS = {
    "acromionradialelength": {"male": [33.52, 1.75, 29.6, 30.7, 33.5, 36.5, 37.7], "female": [31.12, 1.72, 27.1, 28.3, 31.1, 34.0, 35.4]},
    "radialestylionlength": {"male": [26.79, 1.54, 23.5, 24.4, 26.7, 29.5, 30.8], "female": [24.13, 1.52, 20.9, 21.8, 24.1, 26.8, 27.9]},
    "handlength": {"male": [19.33, 0.99, 17.2, 17.7, 19.3, 21.0, 21.8], "female": [18.11, 1.01, 16.0, 16.5, 18.0, 19.9, 20.7]},
    "handbreadth": {"male": [8.83, 0.44, 7.8, 8.1, 8.8, 9.6, 9.9], "female": [7.82, 0.38, 6.9, 7.2, 7.8, 8.5, 8.7]},
    "handcircumference": {"male": [21.23, 1.02, 18.9, 19.5, 21.2, 23.0, 23.7], "female": [18.66, 0.88, 16.7, 17.3, 18.7, 20.2, 20.7]},
    "wristcircumference": {"male": [17.59, 0.90, 15.6, 16.2, 17.6, 19.1, 19.8], "female": [15.48, 0.78, 13.8, 14.2, 15.4, 16.8, 17.4]},
    "forearmcircumferenceflexed": {"male": [31.01, 2.20, 26.3, 27.5, 31.0, 34.8, 36.5], "female": [26.41, 1.85, 22.5, 23.5, 26.3, 29.6, 31.3]},
}

# what body_size measures on a rig, against which survey measure, and the landmark allowance (cm: the rig's value
# may lie that much below the 1st percentile, or above the 99th)
MEASURES = {
    "upper_arm_cm": {"ansur": "acromionradialelength", "below": 4.0, "above": 0.0,
                     "rig": "the shoulder joint's centre to the elbow's (upperarm to lowerarm bone heads)",
                     "why": "the acromion is the top of the shoulder blade, 2 to 4 cm above and outside the shoulder joint's centre; "
                            "the radiale lies about at the elbow joint, so a rig's upper arm reads up to 4 cm shorter"},
    "forearm_cm": {"ansur": "radialestylionlength", "below": 2.0, "above": 0.0,
                   "rig": "the elbow joint's centre to the wrist's (lowerarm to hand bone heads)",
                   "why": "the stylion is the tip of the radial styloid, about 1 to 2 cm past the wrist joint's centre"},
    "hand_length_cm": {"ansur": "handlength", "below": 0.0, "above": 1.5,
                       "rig": "the wrist joint's centre to the farthest skin of the middle finger, along the hand",
                       "why": "the survey starts at the wrist's skin crease, about 1 cm past the wrist joint's centre"},
    "hand_breadth_cm": {"ansur": "handbreadth", "below": 0.0, "above": 0.0,
                        "rig": "the skin's width across the knuckles, index to little finger, thumb left out",
                        "why": "the same landmarks as the survey (metacarpale II to V)"},
    "hand_circumference_cm": {"ansur": "handcircumference", "below": 0.0, "above": 0.0,
                              "rig": "round the skin at the knuckles, thumb left out (a convex girth, as a tape reads it)",
                              "why": "the same landmarks as the survey; it holds the palm's thickness too"},
    "wrist_circumference_cm": {"ansur": "wristcircumference", "below": 0.0, "above": 0.0,
                               "rig": "the least girth of the forearm's skin in its last fifth, at the wrist",
                               "why": "the survey's tape lies just above the styloid processes"},
    "forearm_circumference_cm": {"ansur": "forearmcircumferenceflexed", "below": 0.0, "above": 1.0,
                                 "rig": "the most girth of the forearm's skin in its first half",
                                 "why": "the survey measures a clenched fist's forearm; a relaxed rest pose reads a little less, "
                                        "a sculpted one may read a little more"},
}


def ranges(sex):
    """Each measure's regular range for a sex: {measure: {low, high, p1, p50, p99, mean, sd, survey, why}}."""
    if sex not in ("male", "female"):
        raise ValueError("sex must be male or female")
    out = {}
    for k, m in MEASURES.items():
        mean, sd, p1, p5, p50, p95, p99 = _STATS[m["ansur"]][sex]
        out[k] = {"low": round(p1 - m["below"], 2), "high": round(p99 + m["above"], 2), "p1": p1, "p5": p5, "p50": p50,
                  "p95": p95, "p99": p99, "mean": mean, "sd": sd, "survey": m["ansur"], "rig": m["rig"], "why": m["why"]}
    return out


def human_sizes():
    """The research behind body_size's rules: each measure for men and women, the survey measure it rests on, the
    landmark allowance and why, and the full citation."""
    return {"male": ranges("male"), "female": ranges("female"), "rule": "a measure is regular from the 1st to the 99th "
            "percentile of the character's sex, widened only by the stated landmark allowance",
            "not_covered": ["palm thickness (held by the hand's circumference)", "finger widths and lengths"],
            "sources": SOURCES}
