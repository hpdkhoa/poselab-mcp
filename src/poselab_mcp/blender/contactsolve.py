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
                press_cm=0.12, step_cap_deg=15.0, most=24):
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
    Returns ({joint: degrees closed, below 0 opened}, touching segments, rounds, the steps' log)."""
    js = [j for j in pts if j in hinges]
    order = list(pts)
    pts = {j: np.array(v, dtype=float).copy() for j, v in pts.items()}
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


def align_turn(h, f, g):
    """The turn psi (radians) about the unit axis g that brings the direction h closest to the direction f: h split
    into its part along g (fixed) and across it (h_perp, which turns), h(psi) . f = h_par . f + cos(psi) h_perp . f
    + sin(psi) (g x h_perp) . f, largest at psi = atan2((g x h_perp) . f, h_perp . f). Returns (psi, the angle
    between h(psi) and f in degrees after)."""
    g = np.asarray(g, dtype=float) / max(1e-12, np.linalg.norm(g))
    h = np.asarray(h, dtype=float) / max(1e-12, np.linalg.norm(h))
    f = np.asarray(f, dtype=float) / max(1e-12, np.linalg.norm(f))
    hp = h - g * float(h @ g)
    psi = math.atan2(float(np.cross(g, hp) @ f), float(hp @ f))
    h2 = turn_vec(h, g, psi)
    return psi, math.degrees(math.acos(max(-1.0, min(1.0, float(h2 @ f)))))
