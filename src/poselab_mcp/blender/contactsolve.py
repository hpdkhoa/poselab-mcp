"""Calculated contact moves (v0.10.0): how far and which way a finger joint or a whole hand moves, from the object's
signed distance d and its outward normal n at each skin point. No trial steps: each move is solved from the numbers.
Plain numpy, so it is tested without Blender (examples/contact_solve_tests.py).

A point p on a segment that turns by an angle s about the hinge through c along the unit axis a moves at
    dp/ds = a x (p - c)
so its distance to the object changes at
    dd/ds = n(p) . (a x (p - c))
and the angle that brings it to a target distance t is the Newton step
    s = (t - d) / (dd/ds).
A whole hand moved by x (a shift, 3 numbers) changes each point's distance by n . x; the smallest shift that takes
every point to at least its limit lo solves
    min |x|^2  with  n_i . x >= lo_i - d_i  for every point,
a small quadratic program, solved here by Hildreth's method (one row at a time, exact for this problem)."""
import math

import numpy as np


def rodrigues(X, c, a, ang):
    """Points X (n x 3) turned ang radians about the line through c along the unit axis a."""
    co, si = math.cos(ang), math.sin(ang)
    Y = X - c
    return c + Y * co + np.cross(a, Y) * si + np.outer(Y @ a, a) * (1.0 - co)


def turn_vec(v, a, ang):
    """A direction v turned ang radians about the unit axis a."""
    return rodrigues(v[None, :], np.zeros(3), a, ang)[0]


def min_shift(Nr, need, sweeps=300, tol=1e-6):
    """The smallest shift x (Euclidean) with Nr[i] . x >= need[i] for every row (Hildreth). Rows whose need is far
    below zero never bind and may be left out. Returns (x, the largest violation left)."""
    x = np.zeros(Nr.shape[1])
    if not len(Nr):
        return x, 0.0
    nn = np.einsum("ij,ij->i", Nr, Nr)
    keep = nn > 1e-12
    Nr, need, nn = Nr[keep], need[keep], nn[keep]
    lam = np.zeros(len(Nr))
    for _ in range(sweeps):
        worst = 0.0
        for i in range(len(Nr)):
            g = (need[i] - float(Nr[i] @ x)) / nn[i]
            dl = max(-lam[i], g)
            if dl != 0.0:
                lam[i] += dl
                x = x + dl * Nr[i]
            worst = max(worst, need[i] - float(Nr[i] @ x))
        if worst <= tol:
            break
    viol = float(np.max(need - Nr @ x)) if len(Nr) else 0.0
    return x, max(0.0, viol)


def chain_close(pts, heads, hinges, room_close, room_open, rate, query, touch_cm=0.05, target_cm=0.0,
                press_cm=0.12, step_cap_deg=15.0, most=24, inner=None):
    """A finger's joints closed onto the object by calculated steps. The knuckle leads: every free joint turns at its
    rate (a joint's angle = rate * s), s solved each round as the first touch: for each skin point coming closer
    (dd/ds < 0), s_i = (d_i - target) / (-dd/ds), s = min s_i, cut at the joints' room and at step_cap_deg (the
    step is linear in s, so a long one is checked again). A segment that touches (its nearest point within touch_cm)
    stops its own joint and those behind it; the ones past it keep closing. A segment pressed past press_cm is
    moved out by the Newton step back (the joints behind and at it opened by the angle that brings its deepest
    point to -press_cm / 2) and those joints stop.

    pts: {joint: points riding that segment (n x 3, m)}, the joints in order base to tip; heads: {joint: hinge point};
    hinges: {joint: unit axis, + closes}; room_close / room_open: {joint: degrees it may still close / open};
    rate: {joint: share}; query(X) -> (d cm, n unit outward) for points X.
    A pressed segment that no joint here can move out (no room left to open, or no lever) is left out and logged.

    inner: {joint: (points, d_cm, normals)}: the object's points inside that segment's flesh (an edge between skin
    vertices, which no skin point sees). Each is taken as a point of the finger at that place with distance d_cm
    (below 0 inside past what is allowed) and normal = minus the skin's outward normal there: the finger moving
    by v takes its skin past the object point by -n_skin . v. They ride their segment like its skin, and their
    distance is carried forward linearly from where they started (d = d0 + n . (p - p0)): the flesh has no field
    to read again.
    Returns ({joint: degrees closed, below 0 opened}, touching segments, rounds, the steps' log)."""
    js = [j for j in pts if j in hinges]
    order = list(pts)
    pts = {j: np.array(v, dtype=float).copy() for j, v in pts.items()}
    inner = {j: (np.array(v[0], dtype=float).reshape(-1, 3), np.asarray(v[1], dtype=float), np.array(v[2], dtype=float).reshape(-1, 3))
             for j, v in (inner or {}).items() if len(v[0])}
    ipts = {j: v[0].copy() for j, v in inner.items()}
    hd = {j: np.array(v, dtype=float).copy() for j, v in heads.items()}
    ax = {j: np.array(v, dtype=float) / max(1e-12, np.linalg.norm(v)) for j, v in hinges.items()}
    closed = {j: 0.0 for j in js}
    active = set(js)
    touching = set()
    stuck = set()          # pressed segments no joint here can move out (a palm too deep): reported, left out
    log = []
    n_round = 0

    def ride(j):
        return [k for k in order if order.index(k) >= order.index(j)]

    def apply(moving, s):
        for j in [q for q in js if q in moving]:
            ang = rate[j] * s
            if ang == 0.0:
                continue
            for k in ride(j):
                pts[k] = rodrigues(pts[k], hd[j], ax[j], ang)
                if k in ipts:
                    ipts[k] = rodrigues(ipts[k], hd[j], ax[j], ang)
                if k != j and k in hd:
                    hd[k] = rodrigues(hd[k][None, :], hd[j], ax[j], ang)[0]
                if k != j and k in ax:
                    ax[k] = turn_vec(ax[k], ax[j], ang)
            closed[j] += math.degrees(ang)

    for _ in range(most):
        n_round += 1
        segs = [k for k in order if len(pts[k]) and k not in stuck]
        if not segs:
            break
        X = np.concatenate([pts[k] for k in segs])
        sid = np.concatenate([np.full(len(pts[k]), order.index(k)) for k in segs])
        d, n = query(X)
        d = np.asarray(d, dtype=float)
        n = np.asarray(n, dtype=float)
        iseg = [k for k in segs if k in ipts]
        if iseg:
            Xi = np.concatenate([ipts[k] for k in iseg])
            di = np.concatenate([inner[k][1] + np.einsum("ij,ij->i", inner[k][2], ipts[k] - inner[k][0]) * 100.0 for k in iseg])
            ni = np.concatenate([inner[k][2] for k in iseg])
            X = np.concatenate([X, Xi])
            sid = np.concatenate([sid, np.concatenate([np.full(len(ipts[k]), order.index(k)) for k in iseg])])
            d = np.concatenate([d, di])
            n = np.concatenate([n, ni])
        seg_min = {k: float(d[sid == order.index(k)].min()) for k in segs}
        for k in segs:
            if seg_min[k] <= touch_cm:
                touching.add(k)
                for q in js:
                    if order.index(q) <= order.index(k):
                        active.discard(q)
        deep_k = min(segs, key=lambda k: seg_min[k])
        if seg_min[deep_k] < -press_cm:
            # pressed in: the joints behind and at the deepest segment opened by the Newton step, then stopped
            moving = [q for q in js if order.index(q) <= order.index(deep_k) and room_open[q] + closed[q] > 1e-6]
            if not moving:
                log.append({"round": n_round, "pressed": deep_k, "d_cm": round(seg_min[deep_k], 3), "move": "none left"})
                stuck.add(deep_k)
                continue
            v = np.zeros_like(X)
            for q in moving:
                m_ = np.isin(sid, [order.index(k) for k in ride(q)])
                v[m_] += rate[q] * np.cross(ax[q], X[m_] - hd[q])
            r = np.einsum("ij,ij->i", n, v) * 100.0          # cm per radian of s
            m_ = (sid == order.index(deep_k))
            i = int(np.argmin(np.where(m_, d, np.inf)))
            if abs(r[i]) < 1e-6:
                log.append({"round": n_round, "pressed": deep_k, "d_cm": round(float(d[i]), 3), "move": "no lever"})
                stuck.add(deep_k)
                continue
            s = (-press_cm / 2.0 - d[i]) / r[i]
            lo_s = max(-math.radians(room_open[q] + closed[q]) / rate[q] for q in moving)
            hi_s = min(math.radians(room_close[q] - closed[q]) / rate[q] for q in moving)
            s = min(max(s, lo_s), hi_s)
            apply(moving, s)
            for q in moving:
                active.discard(q)
            log.append({"round": n_round, "pressed": deep_k, "d_cm": round(float(d[i]), 3), "s_deg": round(math.degrees(s), 2),
                        "joints": moving})
            continue
        if not active:
            break
        moving = [q for q in js if q in active]
        v = np.zeros_like(X)
        for q in moving:
            m_ = np.isin(sid, [order.index(k) for k in ride(q)])
            v[m_] += rate[q] * np.cross(ax[q], X[m_] - hd[q])
        r = np.einsum("ij,ij->i", n, v) * 100.0
        coming = (r < -1e-6) & (d > target_cm)
        s_hit = float(np.min((d[coming] - target_cm) / -r[coming])) if coming.any() else math.inf
        s_room = min(math.radians(room_close[q] - closed[q]) / rate[q] for q in moving)
        s_cap = math.radians(step_cap_deg) / max(rate[q] for q in moving)
        s = min(s_hit, s_room, s_cap)
        if s <= 1e-5:
            for q in moving:
                if math.radians(room_close[q] - closed[q]) / rate[q] <= 1e-5:
                    active.discard(q)
            if s_room > 1e-5:
                break                                        # a point already at the target: nothing to close
            continue
        apply(moving, s)
        for q in moving:
            if room_close[q] - closed[q] <= 1e-3:
                active.discard(q)
        log.append({"round": n_round, "s_deg": round(math.degrees(s), 2), "by": "touch" if s == s_hit else ("room" if s == s_room else "cap"),
                    "joints": moving})
    return closed, touching, n_round, log


def wrist_parts(fore, hand, thumb_side, left=False):
    """The wrist's bend as the walls read it (lab._arm_report): the total angle between the forearm (elbow to wrist)
    and the hand (wrist to the middle knuckle), and its split into flexion (+ toward the palm) and radial deviation
    (+ toward the thumb), degrees. thumb_side: the index knuckle minus the little finger's."""
    fore, hand, ts = (np.asarray(v, dtype=float) for v in (fore, hand, thumb_side))
    fn = fore / max(1e-12, np.linalg.norm(fore))
    hn = hand / max(1e-12, np.linalg.norm(hand))
    flat = lambda v: v - fn * float(v @ fn)                         # noqa: E731
    palm = np.cross(ts, hand) * (-1.0 if left else 1.0)
    e1 = flat(palm)
    e1 = e1 / np.linalg.norm(e1) if np.linalg.norm(e1) > 1e-12 else np.zeros(3)
    t = flat(ts)
    e2 = t - e1 * float(t @ e1)
    e2 = e2 / np.linalg.norm(e2) if np.linalg.norm(e2) > 1e-12 else np.zeros(3)
    c = float(hn @ fn)
    off = hn - fn * c
    total = math.degrees(math.acos(max(-1.0, min(1.0, c))))
    return total, math.degrees(math.atan2(float(off @ e1), c)), math.degrees(math.atan2(float(off @ e2), c))


def wrist_past(total, flex, dev, lim):
    """Degrees past the wrist's walls: the total bend past wrist_max_deg and each range part past its limit (the
    largest), 0 inside them all. lim: the arm limits (wrist_max_deg, wrist_flexion_deg, wrist_extension_deg,
    wrist_radial_deg, wrist_ulnar_deg)."""
    return max(0.0, total - lim["wrist_max_deg"], flex - lim["wrist_flexion_deg"], -flex - lim["wrist_extension_deg"],
               dev - lim["wrist_radial_deg"], -dev - lim["wrist_ulnar_deg"])


def rigid_rows(P, c):
    """How points P (n x 3) move for a small rigid move x = (t, w) about c: dP = t + w x (P - c), as n 3 x 6 blocks
    [I, -[P - c]x] stacked (3n x 6)."""
    n = len(P)
    R = np.zeros((3 * n, 6))
    for i, p in enumerate(np.asarray(P, dtype=float) - c):
        R[3 * i:3 * i + 3, :3] = np.eye(3)
        R[3 * i:3 * i + 3, 3:] = np.array([[0.0, p[2], -p[1]], [-p[2], 0.0, p[0]], [p[1], -p[0], 0.0]])
    return R


def constrained_fit(J, r, A, b, damp=1e-9):
    """The move x (6: shift, turn) that best meets the aims J x = r (least squares) while keeping every wall row
    A x >= b. Unconstrained, x0 solves (J'J) x0 = J'r; with H = J'J = L L' the aim is |L'(x - x0)|^2, so with
    y = L'(x - x0) the walls read (A L'^-1) y >= b - A x0 and the smallest y meeting them is Hildreth's (min_shift).
    Returns (x, the largest wall row still unmet, x0)."""
    J = np.asarray(J, dtype=float)
    H = J.T @ J + damp * np.eye(J.shape[1])
    x0 = np.linalg.solve(H, J.T @ np.asarray(r, dtype=float))
    if A is None or not len(A):
        return x0, 0.0, x0
    Lc = np.linalg.cholesky(H)
    Li = np.linalg.inv(Lc.T)
    A = np.asarray(A, dtype=float)
    y, viol = min_shift(A @ Li, np.asarray(b, dtype=float) - A @ x0)
    return x0 + Li @ y, viol, x0
