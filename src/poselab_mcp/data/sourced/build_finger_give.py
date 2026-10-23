"""Builds finger_give.json: how far each finger segment's pad skin gives (cm) at a whole-hand grip force.

Owner's decisions (2026-10-22): "use Wu 2014 handle grip"; "when not firing use 5N, fight the recoil can go to 40N,
but when test if clipping too much then change to 20N"; probe to broad contact: "1 use b" (the measured broad/probe
ratio); below 1 N: "2 use Dzidek"; "3 do the fingers and leave the rest".

For a finger f and its segment s (1 base, 2 middle, 3 tip) at grip G:
    p      = Wu 2014 contact pressure on segment s at grip G (kPa)
    F      = p * A_probe                        force the same pressure puts on the 3.8 mm probe (N)
    D_p(F) = Perez-Gonzalez 2013 skin travel at point(f, s), read between its 1..6 N values (mm)
             below 1 N: D_p(1) * F ** m_g        (Dzidek 2017's contact-area exponent: depth grows as the area)
    give   = r * D_p(F)                         r = broad / probe at the same spot and force (mm -> cm / 10)
Run: python build_finger_give.py  (writes finger_give.json next to it)."""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
A_PROBE_MM2 = math.pi * (3.8 / 2.0) ** 2                       # Perez-Gonzalez 2013: flat indentor 3.8 mm
WU_KPA = {"grip_N": [5, 10, 20, 40],                           # Wu et al. 2014 Table 1 (model, 40 mm handle)
          3: [70, 100, 190, 390], 2: [30, 50, 90, 190], 1: [40, 70, 100, 230]}
# Perez-Gonzalez 2013 Appendix: mean skin travel (mm) at 1..6 N; points 2-5 tip pads, 11-14 middle, 20-23 base
POINTS = {("index", 3): [3.71, 4.25, 4.54, 4.76, 4.93, 5.09], ("middle", 3): [4.04, 4.67, 5.00, 5.23, 5.41, 5.57],
          ("ring", 3): [4.00, 4.69, 5.04, 5.32, 5.52, 5.70], ("pinky", 3): [3.47, 4.01, 4.28, 4.49, 4.66, 4.80],
          ("index", 2): [2.73, 3.44, 3.88, 4.22, 4.50, 4.78], ("middle", 2): [3.24, 4.01, 4.44, 4.75, 4.99, 5.21],
          ("ring", 2): [3.55, 4.46, 4.95, 5.29, 5.53, 5.75], ("pinky", 2): [2.66, 3.30, 3.70, 4.01, 4.28, 4.54],
          ("index", 1): [4.03, 5.06, 5.75, 6.32, 6.81, 7.31], ("middle", 1): [3.71, 4.59, 5.13, 5.60, 5.98, 6.37],
          ("ring", 1): [3.51, 4.36, 4.98, 5.50, 5.97, 6.40], ("pinky", 1): [3.58, 4.46, 5.03, 5.50, 5.89, 6.25]}
POINT_ID = {(f, s): p for f, s, p in [("index", 3, 2), ("middle", 3, 3), ("ring", 3, 4), ("pinky", 3, 5),
                                      ("index", 2, 11), ("middle", 2, 12), ("ring", 2, 13), ("pinky", 2, 14),
                                      ("index", 1, 20), ("middle", 1, 21), ("ring", 1, 22), ("pinky", 1, 23)]}
DZIDEK_DEPTH_MM_AT_2N = 1.6                                    # Dzidek 2017: index pad on flat glass, 2 N
M_G = 0.37                                                     # Dzidek 2017 Table 1: gross area ~ W^0.37 (30 deg)
RATIO = DZIDEK_DEPTH_MM_AT_2N / POINTS[("index", 3)][1]        # broad / probe, index tip at 2 N


def probe_mm(f, s, F):
    D = POINTS[(f, s)]
    if F <= 1.0:
        return D[0] * F ** M_G
    if F >= 6.0:
        return D[-1]
    i = int(F) - 1
    return D[i] + (D[i + 1] - D[i]) * (F - (i + 1))


def build():
    table = {}
    for (f, s) in POINTS:
        row = {}
        for k, G in enumerate(WU_KPA["grip_N"]):
            F = WU_KPA[s][k] * 1e3 * A_PROBE_MM2 * 1e-6
            row[str(G)] = {"probe_N": round(F, 3), "probe_mm": round(probe_mm(f, s, F), 3),
                           "give_cm": round(RATIO * probe_mm(f, s, F) / 10.0, 4)}
        table["%s%d_pad" % (f, s)] = row
    return {"what": "Give of each finger segment's pad skin (cm) at a whole-hand grip force (N). Read by contactlab.skin_give.",
            "grip_N": WU_KPA["grip_N"],
            "give_cm": {lab: [row[str(G)]["give_cm"] for G in WU_KPA["grip_N"]] for lab, row in table.items()},
            "detail": table,
            "equations": {"F": "p * A_probe", "D_p": "Perez-Gonzalez 2013 point table between 1 and 6 N; below 1 N D_p(1) * F^m_g",
                          "give": "r * D_p / 10", "r": "%.4f = 1.6 mm (Dzidek, flat glass, 2 N) / 4.25 mm (Perez-Gonzalez point 2, 2 N)" % RATIO,
                          "A_probe_mm2": round(A_PROBE_MM2, 3), "m_g": M_G},
            "sources": {
                "pressure": "Wu JZ, Dong RG, Warren CM, Welcome DE, McDowell TW. Med Eng Phys 2014;36(7):831-841, Table 1 (PMC4573534)",
                "probe_travel": "Perez-Gonzalez A, Vergara M, Sancho-Bru JL. J Biomech 2013;46(15):2644-2650, Appendix (author pre-print, Universitat Jaume I)",
                "broad_and_low_force": "Dzidek BM et al. J R Soc Interface 2017;14:20160935 (PMC5332579): max displacement about 1.6 mm at 2 N; gross contact area ~ W^0.37",
                "derivation": "below 1 N the depth follows the measured area exponent: a rounded pad pressed flat has depth in step with its contact area (spherical cap: area = 2 pi R depth). Ours, from Dzidek's measured exponent."},
            "points": {"%s%d_pad" % k: v for k, v in POINT_ID.items()},
            "not_covered": "thumb, palm, palm side, web, finger sides and backs keep skin_give.json (owner 2026-10-22: '3 do the fingers and leave the rest')",
            "owner_decisions": ["use Wu 2014 handle grip", "when not firing use 5N, fight the recoil can go to 40N, but when test if clipping too much then change to 20N",
                                "1 use b, 2 use Dzidek, 3 do the fingers and leave the rest"]}


if __name__ == "__main__":
    out = build()
    json.dump(out, open(os.path.join(HERE, "finger_give.json"), "w", encoding="utf-8"), indent=1)
    for lab, v in out["give_cm"].items():
        print(lab, v)
