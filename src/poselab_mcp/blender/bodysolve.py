"""The direct hold's body, solved by formula (v0.11.0, docs/DIRECT_HOLD_PLAN.md Steps B and C). The owner's words:
"place the hand directly on the gun at the position you got from the document/research paper, then from there you
adjust the wrist, the arm, the elbow and the shoulder".

The hand is already fixed on the gun (Step A). For the hand where the gun puts it, the shoulder where the girdle puts
it and the elbow's swing about the shoulder-to-wrist line, every arm wall the walls check reads (lab._arm_report) is
worked out here with the same formulas, in numpy, with no pose set. A pose is kept only when it is inside every wall;
the kept ones are ranked by comfort, which never holds a wall:

    cost = (wrist bend / 10)^2 + (forearm roll / 30)^2 + (girdle share)^2 + ((elbow flexion - f_pref) / 20)^2
           + (gun moved cm / 10)^2

(f_pref, the style's preferred elbow flexion, is Step B's reach equation D*^2 = l1^2 + l2^2 + 2 l1 l2 cos f_pref as a
preference: the gun's place must also keep every wall, so the reach is not forced.) The caller poses the best one and
checks it with the real walls; the formulas here are tested in examples/body_solve_tests.py.

Two walls have no formula here and are read only on the posed arm: the forearm's links (a rig with twist bones) and the
wrist's twist against the idle grip (a rig without)."""
import math

import numpy as np

UP = np.array([0.0, 0.0, 1.0])


def _unit(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def _deg(a, b):
    """The angle between rows of a and b (degrees)."""
    c = np.einsum("...i,...i->...", _unit(a), _unit(b))
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def elbow_circle(S, W, l1, l2, down, phis_deg):
    """The elbow's places for a shoulder S and a wrist W (two-bone arm, lengths l1, l2): on the circle about the S-W
    line, phi = 0 the place nearest `down` (the elbow hanging), turning about S->W. Returns (elbows n x 3, reach
    ok)."""
    d = W - S
    D = float(np.linalg.norm(d))
    if D < 1e-6 or D > l1 + l2 - 1e-4 or D < abs(l1 - l2) + 1e-4:
        return None, False
    a = d / D
    x = (l1 * l1 - l2 * l2 + D * D) / (2.0 * D)
    h = math.sqrt(max(l1 * l1 - x * x, 0.0))
    c = S + a * x
    u = down - a * float(down @ a)
    if np.linalg.norm(u) < 1e-9:
        u = np.cross(a, [1.0, 0.0, 0.0])
    u = u / np.linalg.norm(u)
    v = np.cross(a, u)
    ph = np.radians(np.asarray(phis_deg, dtype=float))
    return c + h * (np.cos(ph)[:, None] * u + np.sin(ph)[:, None] * v), True


def arm_walls(S, E, W, K, Ik, Pk, side, L, right, fwd, up=UP):
    """The arm's values and walls for elbows E (n x 3), as lab._arm_report reads them. S shoulder, W wrist, K the
    middle knuckle, Ik / Pk the index and little finger knuckles (each 3, the hand fixed); right, fwd, up: the view's
    axes. Returns (values {name: n array}, past {wall: n array of degrees or cm past, 0 inside})."""
    E = np.atleast_2d(E)
    n = len(E)
    upper = E - S
    fore = W - E
    hand = np.tile(K - W, (n, 1))
    ts = np.tile(Ik - Pk, (n, 1))
    under = (S - E) @ up * 100.0
    bend = _deg(upper, fore)
    total = _deg(fore, hand)
    sign = -1.0 if side == "l" else 1.0
    palm = np.cross(ts, hand) * sign
    fn = _unit(fore)
    hn = _unit(hand)
    flat = lambda v: v - fn * np.einsum("ij,ij->i", v, fn)[:, None]      # noqa: E731
    e1 = _unit(flat(palm))
    t = flat(ts)
    e2 = _unit(t - e1 * np.einsum("ij,ij->i", t, e1)[:, None])
    c = np.einsum("ij,ij->i", hn, fn)
    off = hn - fn * c[:, None]
    flex = np.degrees(np.arctan2(np.einsum("ij,ij->i", off, e1), c))
    dev = np.degrees(np.arctan2(np.einsum("ij,ij->i", off, e2), c))
    # the forearm's rotation: thumb-side line against the elbow's hinge plane (None near straight: read as 0)
    upv = S - E
    straight = _deg(fore, -upv) < 20.0
    r = _unit(upv - fn * np.einsum("ij,ij->i", upv, fn)[:, None])
    tt = _unit(ts - fn * np.einsum("ij,ij->i", ts, fn)[:, None])
    rot = np.degrees(np.arctan2(np.einsum("ij,ij->i", np.cross(r, tt), fn), np.einsum("ij,ij->i", r, tt)))
    rot = np.where(straight, 0.0, rot if side == "r" else -rot)
    # the shoulder against the view's axes
    lat = right if side == "r" else -right
    u = _unit(upper)
    sf = np.degrees(np.arctan2(u @ fwd, -(u @ up)))
    hzx, hzy = u @ lat, u @ fwd
    wide = np.hypot(hzx, hzy) > 0.5
    cross = np.where(wide, np.degrees(np.arctan2(-hzx, hzy)), 0.0)
    behind = np.where(wide, np.degrees(np.arctan2(-hzy, hzx)), 0.0)
    nearf = np.abs(u @ fwd) < 0.9
    ref = np.where(nearf[:, None], fwd - u * (u @ fwd)[:, None], up - u * (u @ up)[:, None])
    fp = fore - u * np.einsum("ij,ij->i", fore, u)[:, None]
    rr = np.degrees(np.arctan2(np.einsum("ij,ij->i", np.cross(_unit(ref), _unit(fp)), u),
                               np.einsum("ij,ij->i", _unit(ref), _unit(fp))))
    internal = np.where(bend >= 25.0, -rr if side == "r" else rr, 0.0)
    z = np.zeros(n)
    past = {
        "elbow under the shoulder (cm)": np.maximum(z, L["elbow_under_shoulder_cm"] - under),
        "elbow bent": np.maximum(z, bend - L["elbow_bend_max_deg"]),
        "elbow locked straight": np.maximum(z, L["elbow_bend_min_deg"] - bend),
        "wrist bent": np.maximum(z, total - L["wrist_max_deg"]),
        "wrist flexion": np.maximum(z, flex - L["wrist_flexion_deg"]),
        "wrist extension": np.maximum(z, -flex - L["wrist_extension_deg"]),
        "wrist radial": np.maximum(z, dev - L["wrist_radial_deg"]),
        "wrist ulnar": np.maximum(z, -dev - L["wrist_ulnar_deg"]),
        "forearm supinated": np.maximum(z, rot - L["supination_deg"]),
        "forearm pronated": np.maximum(z, -rot - L["pronation_deg"]),
        "shoulder raised": np.maximum(z, sf - L["shoulder_flexion_deg"]),
        "shoulder back": np.maximum(z, -sf - L["shoulder_extension_deg"]),
        "upper arm across the body": np.maximum(z, cross - L["shoulder_cross_body_deg"]),
        "upper arm behind": np.maximum(z, behind - L["shoulder_behind_deg"]),
        "upper arm turned in": np.maximum(z, internal - L["shoulder_internal_deg"]),
        "upper arm turned out": np.maximum(z, -internal - L["shoulder_external_deg"]),
    }
    vals = {"elbow_under_cm": under, "elbow_bend_deg": bend, "wrist_deg": total, "wrist_flexion_deg": flex,
            "wrist_radial_deg": dev, "forearm_rotation_deg": rot, "shoulder_flexion_deg": sf, "shoulder_across_deg": cross,
            "shoulder_internal_deg": internal}
    return vals, past


def solve_body(hand_at, girdles, l1, l2, side, L, right, fwd, f_pref, phis_deg=None, up=UP):
    """Every candidate body for a hand fixed on the gun, by formula. hand_at: [(gun shift cm (3), W, K, Ik, Pk)] the
    hand's points for each gun place; girdles: [(name, S, girdle share)]. Returns the candidates inside every wall,
    best first: dicts with the shift, the girdle, phi, the elbow, the cost and the values; and the count of all."""
    phis = np.arange(-90.0, 90.0 + 1e-9, 2.0) if phis_deg is None else np.asarray(phis_deg, dtype=float)
    down = -up
    keep, n_all = [], 0
    for shift, W, K, Ik, Pk in hand_at:
        gun_cm = float(np.linalg.norm(shift))
        for gname, S, share in girdles:
            E, ok = elbow_circle(S, W, l1, l2, down, phis)
            if not ok:
                n_all += len(phis)
                continue
            vals, past = arm_walls(S, E, W, K, Ik, Pk, side, L, right, fwd, up)
            n_all += len(phis)
            inside = np.all(np.stack([v <= 0.5 for v in past.values()]), axis=0)      # the walls' reading rounds
            if not inside.any():
                continue
            flexion = vals["elbow_bend_deg"]
            cost = ((vals["wrist_deg"] / 10.0) ** 2 + (vals["forearm_rotation_deg"] / 30.0) ** 2 + share ** 2
                    + ((flexion - f_pref) / 20.0) ** 2 + (gun_cm / 10.0) ** 2)
            for i in np.nonzero(inside)[0]:
                keep.append({"shift_cm": [round(float(x), 2) for x in shift], "girdle": gname, "phi_deg": float(phis[i]),
                             "elbow": E[i].copy(), "cost": float(cost[i]),
                             "values": {k: round(float(v[i]), 1) for k, v in vals.items()}})
    keep.sort(key=lambda c: c["cost"])
    return keep, n_all


def nearest_walls(hand_at, girdles, l1, l2, side, L, right, fwd, phis_deg=None, up=UP):
    """When no candidate is inside every wall: the one that is past them least (the sum of what is past), and which
    walls it breaks by how much, to report."""
    phis = np.arange(-90.0, 90.0 + 1e-9, 2.0) if phis_deg is None else np.asarray(phis_deg, dtype=float)
    best = None
    for shift, W, K, Ik, Pk in hand_at:
        for gname, S, _share in girdles:
            E, ok = elbow_circle(S, W, l1, l2, -up, phis)
            if not ok:
                continue
            vals, past = arm_walls(S, E, W, K, Ik, Pk, side, L, right, fwd, up)
            tot = np.sum(np.stack(list(past.values())), axis=0)
            i = int(np.argmin(tot))
            if best is None or tot[i] < best[0]:
                best = (float(tot[i]), {"shift_cm": [round(float(x), 2) for x in shift], "girdle": gname,
                                        "phi_deg": float(phis[i]),
                                        "past": {k: round(float(v[i]), 1) for k, v in past.items() if v[i] > 0.5}})
    return best[1] if best else {"reach": "the arm cannot reach the hand at any gun place"}
