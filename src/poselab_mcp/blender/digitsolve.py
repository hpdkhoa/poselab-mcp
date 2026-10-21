"""The thumb's hold, solved by formula (v0.12.0, docs/DIRECT_HOLD_PLAN.md): the owner's words, "the thumb need to touch
the left side of the object or opposite side of index finger to complete a grab". Plain numpy, tested in
examples/digit_solve_tests.py.

The hand is fixed (Step A), so the thumb's base joint (CMC) is fixed. The thumb has four angles: the base's spread
alpha (about the palm's normal) and lift beta (about the line across the metacarpal and the palm's normal), the
knuckle (MCP, t2) and the end joint (IP, t3), each a turn about its own hinge, carried by the ones before it. Every
pose on the grid is worked out with no pose set:

    pad = the end segment's pad points after R_cmc(alpha, beta), then t2 about the knuckle's hinge, then t3 about the
          end joint's hinge

and kept only when it meets every condition:
    touch:    -give_pad <= min gap(pad) <= 0.1 cm
    facing:   pad normal . surface normal <= -cos 30
    band:     the touching pad point within band_cm of the target line (the frame's side by the slide stop)
    grab:     the surface normal at the thumb . the normal at the index's line < 0 (opposite sides)
    walls:    base spread <= its most, the metacarpal no further behind the palm than its least, each hinge inside its
              range (and a holding thumb's), no thumb skin past its give
The kept pose nearest the thumb as it is (|alpha| + |beta| + |t2| + |t3| least) is the answer."""
import math

import numpy as np


def _rot(axis, ang):
    """Rotation matrices for angles ang (n) about unit axes (n x 3 or 3)."""
    a = np.asarray(axis, dtype=float)
    ang = np.asarray(ang, dtype=float)
    if a.ndim == 1:
        a = np.tile(a, (ang.size, 1))
    a = a / np.maximum(np.linalg.norm(a, axis=1, keepdims=True), 1e-12)
    c, s = np.cos(ang), np.sin(ang)
    x, y, z = a[:, 0], a[:, 1], a[:, 2]
    C = 1.0 - c
    R = np.empty((ang.size, 3, 3))
    R[:, 0, 0] = c + x * x * C
    R[:, 0, 1] = x * y * C - z * s
    R[:, 0, 2] = x * z * C + y * s
    R[:, 1, 0] = y * x * C + z * s
    R[:, 1, 1] = c + y * y * C
    R[:, 1, 2] = y * z * C - x * s
    R[:, 2, 0] = z * x * C - y * s
    R[:, 2, 1] = z * y * C + x * s
    R[:, 2, 2] = c + z * z * C
    return R


def _about(R, c, X):
    """Points X (k x m x 3, or m x 3 shared) turned by R (k x 3 x 3) about centres c (k x 3 or 3)."""
    c = np.asarray(c, dtype=float)
    if X.ndim == 2:
        X = np.broadcast_to(X, (R.shape[0],) + X.shape)
    cc = c[:, None, :] if c.ndim == 2 else c[None, None, :]
    return cc + np.einsum("kij,kmj->kmi", R, X - cc)


def thumb_solve(d, query, lim, band, n_index, alphas=None, betas=None, t2s=None, t3s=None, keep_all=False):
    """The thumb's four angles by formula (the module's equations). d: the thumb as it is, in the world: "p01", "p02",
    "p03" (joint heads), "pn" (the palm's facing), "index_meta" (the index metacarpal's line), "h02", "h03" (the
    hinges, + closes), "bend02", "bend03" (the hinges' bends now, degrees), "seg" {"01", "02", "03": skin points m x 3},
    "seg_give" {same: each point's give, cm}, "pad" (pad points), "pad_n" (the pad's mean normal). query(X) -> (gap cm,
    outward normals). lim: "spread_max", "palmar_min", "b02" (least, most), "b03" (least, most). band: (a point on the
    target line, its direction, its half width cm). n_index: the surface normal at the index's line. Returns
    (best: angles and readings, or None; counts: how many poses failed each condition); with keep_all, also every
    kept pose, cheapest first."""
    alphas = np.arange(-40.0, 40.1, 5.0) if alphas is None else np.asarray(alphas, dtype=float)
    betas = np.arange(-30.0, 45.1, 5.0) if betas is None else np.asarray(betas, dtype=float)
    t2s = np.arange(lim["b02"][0] - d["bend02"], lim["b02"][1] - d["bend02"] + 0.1, 5.0) if t2s is None else np.asarray(t2s, dtype=float)
    t3s = np.arange(lim["b03"][0] - d["bend03"], lim["b03"][1] - d["bend03"] + 0.1, 10.0) if t3s is None else np.asarray(t3s, dtype=float)
    p01, p02, p03 = (np.asarray(d[k], dtype=float) for k in ("p01", "p02", "p03"))
    pn = np.asarray(d["pn"], dtype=float)
    pn = pn / np.linalg.norm(pn)
    imeta = np.asarray(d["index_meta"], dtype=float)
    imeta = imeta / np.linalg.norm(imeta)
    meta0 = p02 - p01
    a_lift = np.cross(meta0, pn)
    a_lift = a_lift / max(1e-12, np.linalg.norm(a_lift))
    T2, T3 = np.meshgrid(t2s, t3s, indexing="ij")
    T2, T3 = T2.ravel(), T3.ravel()
    K = T2.size
    L0, Ld, Lw = (np.asarray(band[0], dtype=float), np.asarray(band[1], dtype=float) / np.linalg.norm(band[1]), float(band[2]))
    n_index = np.asarray(n_index, dtype=float)
    cos30 = math.cos(math.radians(30.0))
    counts = {"base walls": 0, "thumb skin past its give": 0, "pad not touching": 0, "pad not facing in": 0,
              "pad off the target line": 0, "not opposite the index": 0, "kept": 0}
    best = None
    kept_all = []
    seg = {j: np.asarray(d["seg"][j], dtype=float) for j in ("01", "02", "03")}
    give = np.concatenate([np.asarray(d["seg_give"][j], dtype=float) for j in ("01", "02", "03")])
    pad = np.asarray(d["pad"], dtype=float)
    pad_n = np.asarray(d["pad_n"], dtype=float)
    for al in alphas:
        for be in betas:
            R1 = (_rot(a_lift, [math.radians(be)]) @ _rot(pn, [math.radians(al)]))[0]
            q02, q03 = p01 + R1 @ (p02 - p01), p01 + R1 @ (p03 - p01)
            meta = q02 - p01
            spread = math.degrees(math.acos(max(-1.0, min(1.0, float(meta @ imeta) / np.linalg.norm(meta)))))
            palmar = math.degrees(math.asin(max(-1.0, min(1.0, float(meta @ pn) / np.linalg.norm(meta)))))
            if spread > lim["spread_max"] + 0.5 or palmar < lim["palmar_min"] - 0.5:
                counts["base walls"] += K
                continue
            h02, h03 = R1 @ np.asarray(d["h02"], dtype=float), R1 @ np.asarray(d["h03"], dtype=float)
            s01 = p01 + (seg["01"] - p01) @ R1.T
            s02 = p01 + (seg["02"] - p01) @ R1.T
            s03 = p01 + (seg["03"] - p01) @ R1.T
            pd_ = p01 + (pad - p01) @ R1.T
            pnr = R1 @ pad_n
            R2 = _rot(h02, np.radians(T2))
            q03k = q02 + np.einsum("kij,j->ki", R2, q03 - q02)
            h03k = np.einsum("kij,j->ki", R2, h03)
            s02k = _about(R2, q02, s02)
            s03k = _about(R2, q02, s03)
            padk = _about(R2, q02, pd_)
            pnk = np.einsum("kij,j->ki", R2, pnr)
            R3 = _rot(h03k, np.radians(T3))
            s03k = _about(R3, q03k, s03k)
            padk = _about(R3, q03k, padk)
            pnk = np.einsum("kij,kj->ki", R3, pnk)
            skin = np.concatenate([np.broadcast_to(s01, (K,) + s01.shape), s02k, s03k], axis=1)
            Ds, _n = query(skin.reshape(-1, 3))
            Ds = np.asarray(Ds).reshape(K, -1)
            clip = np.any(Ds < -give[None, :] - 0.03, axis=1)
            Dp, Np = query(padk.reshape(-1, 3))
            Dp = np.asarray(Dp).reshape(K, -1)
            Np = np.asarray(Np).reshape(K, -1, 3)
            i_ = np.argmin(Dp, axis=1)
            gmin = Dp[np.arange(K), i_]
            nsurf = Np[np.arange(K), i_]
            ppt = padk[np.arange(K), i_]
            touch = (gmin >= -d.get("give_pad", 0.15)) & (gmin <= 0.1)
            facing = np.einsum("ki,ki->k", pnk / np.maximum(np.linalg.norm(pnk, axis=1, keepdims=True), 1e-12), nsurf) <= -cos30
            off = ppt - L0
            inband = np.linalg.norm(off - np.outer(off @ Ld, Ld), axis=1) * 100.0 <= Lw
            grab = nsurf @ n_index < 0.0
            counts["thumb skin past its give"] += int(clip.sum())
            ok = ~clip
            counts["pad not touching"] += int((ok & ~touch).sum())
            ok &= touch
            counts["pad not facing in"] += int((ok & ~facing).sum())
            ok &= facing
            counts["pad off the target line"] += int((ok & ~inband).sum())
            ok &= inband
            counts["not opposite the index"] += int((ok & ~grab).sum())
            ok &= grab
            counts["kept"] += int(ok.sum())
            for k in np.nonzero(ok)[0]:
                cost = abs(al) + abs(be) + abs(T2[k]) + abs(T3[k])
                pose = {"alpha_deg": float(al), "beta_deg": float(be), "t2_deg": float(T2[k]), "t3_deg": float(T3[k]),
                        "cost": float(cost), "pad_gap_cm": round(float(gmin[k]), 3),
                        "pad_to_line_cm": round(float(np.linalg.norm(off[k] - (off[k] @ Ld) * Ld)) * 100.0, 2),
                        "spread_deg": round(spread, 1), "palmar_deg": round(palmar, 1),
                        "a_lift": a_lift.tolist()}
                if keep_all:
                    kept_all.append(pose)
                if best is None or cost < best["cost"]:
                    best = pose
    if keep_all:
        kept_all.sort(key=lambda p_: p_["cost"])
        return best, counts, kept_all
    return best, counts
