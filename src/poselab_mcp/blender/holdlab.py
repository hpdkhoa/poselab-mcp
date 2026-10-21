"""Holding any object (0.9.0): the object read from its own geometry, marks suggested from it, purposes judged on a
hand, the shoulder girdle, a whole-arm hold search and a gate that fails a hold that does not do its job.

An object is the rig's loaded model (Pose Lab's rifle_mesh, any object: a pistol, a mug, a hammer). Its parts are
its vertex groups (a rigged model's bones), else its connected pieces. The reader measures each part: the runs of it
a hand can wrap (an axis, sections along it with their girth, their free sides, the ends), and the openings beside
them (a trigger guard, a mug's handle). A mark names what a part is for:

    {"part": name, "role": "hold" | "notouch", "purpose": "power_wrap" | "support" | "pinch" | "press",
     "run": index, "palm_dir": [x, y, z] (object frame, optional), "web_end": "inner" | "a" | "b",
     "thumb": "inline" | "opposed", "index": "wrap" | "extended", "index_line": opening index (extended)}

Purposes come from the human grasp taxonomy (Cutkosky 1989; Feix, Romero, Schmiedmayer, Dollar, Kragic 2016, The
GRASP Taxonomy of Human Grasp Types): a power wrap closes palm and fingers round a part; a support hand fills what
the other hand leaves and holds it; a pinch opposes digit pads; a press puts one pad on a face. Each is judged by
numbers on the hand's capsules and on the drawn skin, never by a picture."""
import math
import os

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

from contactlab import CLIP_NOISE_CM2, DIGITS, HOLD_DIGIT_LIMITS, SKIN_GIVE, _lab_key
from deprecation import deprecated

# the shoulder girdle: the sternoclavicular joint's range, a working share of it kept
CLAVICLE = {"protraction_deg": 15.0, "retraction_deg": 20.0, "elevation_deg": 30.0, "depression_deg": 8.0,
            "source": "Neumann, Kinesiology of the Musculoskeletal System (3rd ed., 2017), the sternoclavicular joint: "
                      "elevation about 45, depression about 10, protraction 15 to 20, retraction 15 to 30 degrees; "
                      "two thirds of each kept as the working range"}
# a part a hand can wrap: its girth as a cylinder's diameter
HANDLE = {"min_diameter_cm": 2.0, "max_diameter_cm": 6.0, "best_share_of_hand_length": 0.197, "min_run_cm": 5.0,
          "source": "Kong and Lowe 2005 (Int. J. Industrial Ergonomics 35): the strongest power grip on a cylinder "
                    "of about 19.7 percent of the hand's length across; handles under 2 cm or over 6 cm are pinched "
                    "or palmed, not wrapped"}
# what each purpose asks, measured
PURPOSES = {
    "power_wrap": {"coverage_min": 0.55, "web_gap_max_cm": 1.2, "palm_gap_max_cm": 0.5, "band_cm": 0.4,
                   "why": "palm and fingers closed round the part (the GRASP taxonomy's power wrap), covering as much "
                          "of its girth as the hand can (measured on the drawn skin), the web of the hand high under "
                          "the part's overhang"},
    "thumb_inline": {"angle_max_deg": 15.0, "end_bend_max_deg": 25.0, "gap_cm": [-0.3, 0.15],
                     "why": "the thumb laid in line with the index finger's direction, straight, its pad pressing a "
                            "little into the part (FM 3-23.35 2-1: the thumb alongside, high)"},
    "index_extended": {"angle_max_deg": 15.0, "band_cm": [-0.3, 1.6], "gap_max_cm": 0.6,
                       "why": "the index finger straight along the opening's edge nearest the hand (a pistol's "
                              "trigger finger on the guard's line, off the trigger)"},
    "support": {"open_cover_min": 0.4, "over_share_min": 0.5, "thumb_angle_max_deg": 20.0, "band_cm": 0.4,
                "why": "the support hand fills the girth the other hand leaves open and closes over the other "
                       "hand's fingers, its thumb laid the same way (FM 3-23.35 2-2)"},
    "notouch": {"gap_min_cm": 0.05, "why": "no skin on a part that moves or must stay clear"},
    "comfort": {"wrist_max_deg": 22.0, "roll_max_deg": 60.0, "elbow_deg": [25.0, 135.0], "clavicle_share_max": 0.8,
                "why": "each joint well inside its range, not at its edge (legal is not enough)"},
}


def _np(vs):
    return np.array([[v.x, v.y, v.z] for v in vs], dtype=float)


def _hull2(pts):
    """The convex hull of 2D points (numpy, n x 2), counter-clockwise, and its perimeter."""
    p = sorted(set((round(float(x), 6), round(float(y), 6)) for x, y in pts))
    if len(p) < 3:
        return p, 0.0
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])     # noqa: E731
    lo, hi = [], []
    for q in p:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    for q in reversed(p):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], q) <= 0:
            hi.pop()
        hi.append(q)
    h = lo[:-1] + hi[:-1]
    return h, sum(math.dist(h[i], h[(i + 1) % len(h)]) for i in range(len(h)))


def _blobs(pu, pv, cell=0.006):
    """A section's points split into its separate pieces (a grip and the guard bar in front of it): cells of
    cell metres, touching cells one piece. Returns [(us, vs)]."""
    if len(pu) == 0:
        return []
    iu = np.floor(pu / cell).astype(int)
    iv = np.floor(pv / cell).astype(int)
    cells = {}
    for n, key in enumerate(zip(iu.tolist(), iv.tolist())):
        cells.setdefault(key, []).append(n)
    seen, out = set(), []
    for key in cells:
        if key in seen:
            continue
        stack, idx = [key], []
        seen.add(key)
        while stack:
            a, b = stack.pop()
            idx += cells[(a, b)]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    q = (a + da, b + db)
                    if q in cells and q not in seen:
                        seen.add(q)
                        stack.append(q)
        idx = np.array(idx)
        out.append((pu[idx], pv[idx]))
    return out


def _ring(hull, n=24):
    """n points evenly round a closed 2D polygon."""
    if len(hull) < 3:
        return list(hull)
    segs = [(hull[i], hull[(i + 1) % len(hull)]) for i in range(len(hull))]
    L = [math.dist(a, b) for a, b in segs]
    tot = sum(L)
    out, k, acc = [], 0, 0.0
    for i in range(n):
        s = tot * i / n
        while k < len(segs) - 1 and acc + L[k] < s:
            acc += L[k]
            k += 1
        a, b = segs[k]
        t = 0.0 if L[k] < 1e-12 else (s - acc) / L[k]
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def _dirs(step_deg=10.0):
    """Directions over a half sphere (a line's two ways are one)."""
    out = []
    for el in np.arange(0.0, 90.0 + 1e-9, step_deg):
        n = max(1, int(round(360.0 * math.cos(math.radians(el)) / step_deg)))
        for k in range(n):
            az = 360.0 * k / n
            if el > 89.9 and k:
                break
            e, a = math.radians(el), math.radians(az)
            out.append(np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)]))
    return out


class HoldMixin:
    # --- the object, read from its geometry ------------------------------------------------------------------
    def _obj_frame(self, part=None):
        """A part's own frame as the object is posed now: its bone's (a rigged object moves a part by its bone:
        a gun's frame by its root, its slide by the slide's bone), else the armature object's, else the mesh's."""
        arm = getattr(self, "rifle_arm", None)
        if arm is not None and part and part in arm.pose.bones:
            pb = arm.pose.bones[part]
            return arm.matrix_world @ pb.matrix @ pb.bone.matrix_local.inverted()
        o = arm or self.rifle_mesh
        return o.matrix_world

    def _part_points(self, spacing_cm=0.2, seed=11):
        """Every part's surface sampled about spacing_cm apart, in the object's frame (m): {part: (n x 3 points,
        n x 3 normals)}, with the parts' bones at rest. A part is a vertex group (the heaviest of a polygon's
        vertices), else a connected piece."""
        rng = np.random.default_rng(seed)
        dg = bpy.context.evaluated_depsgraph_get()
        me_ob = self.rifle_mesh
        ev = me_ob.evaluated_get(dg)
        me = ev.to_mesh()
        Mw = me_ob.matrix_world
        V = np.array([list(Mw @ v.co) for v in me.vertices])           # the world, as posed now
        names = {g.index: g.name for g in me_ob.vertex_groups}
        vg = []
        for v in me_ob.data.vertices:
            gm = max(v.groups, key=lambda x: x.weight) if v.groups else None
            vg.append(names.get(gm.group) if gm else None)
        polys = [tuple(p.vertices) for p in me.polygons]
        ev.to_mesh_clear()
        part_of = []
        if any(vg):
            for p in polys:
                gs = [vg[i] or "body" for i in p]
                part_of.append(max(set(gs), key=gs.count))
        else:
            # connected pieces: polygons sharing a vertex are one piece
            parent = list(range(len(V)))

            def find(i):
                while parent[i] != i:
                    parent[i] = parent[parent[i]]
                    i = parent[i]
                return i
            for p in polys:
                for i in p[1:]:
                    ra, rb = find(p[0]), find(i)
                    if ra != rb:
                        parent[rb] = ra
            roots = {}
            for p in polys:
                r = find(p[0])
                roots.setdefault(r, "piece_%d" % len(roots))
                part_of.append(roots[r])
        sp = spacing_cm / 100.0
        acc = {}
        for p, part in zip(polys, part_of):
            for k in range(1, len(p) - 1):
                a, b, c = V[p[0]], V[p[k]], V[p[k + 1]]
                n = np.cross(b - a, c - a)
                area = 0.5 * float(np.linalg.norm(n))
                if area < 1e-12:
                    continue
                n = n / (2.0 * area)
                m = max(1, int(area / (sp * sp)))
                r1, r2 = rng.random(m), rng.random(m)
                s1 = np.sqrt(r1)
                pts = (1 - s1)[:, None] * a + (s1 * (1 - r2))[:, None] * b + (s1 * r2)[:, None] * c
                acc.setdefault(part, ([], []))
                acc[part][0].append(pts)
                acc[part][1].append(np.repeat(n[None, :], m, 0))
        out = {}
        for k, v in acc.items():
            Fi = self._obj_frame(k).inverted()
            R_ = np.array(Fi.to_3x3())
            T_ = np.array(Fi.translation)
            Pw, Nw = np.vstack(v[0]), np.vstack(v[1])
            Pl = Pw @ R_.T + T_
            Nl = Nw @ R_.T
            Nl /= np.maximum(np.linalg.norm(Nl, axis=1, keepdims=True), 1e-12)
            out[k] = (Pl, Nl)                          # each part in its own frame
        return out

    def _runs(self, P, hand_len_cm, step_cm=0.5, dir_step_deg=8.0):
        """The runs of a part a hand can wrap: along a direction, consecutive sections whose girth (as a cylinder's
        diameter) is in the handle range and whose middles follow on from one another. Returns the longest ones,
        best first: {axis, origin, sections [{t, centre, diameter_cm, width_cm, depth_cm, ring (2D)}], length_cm,
        frame (u, v)}."""
        lo_d, hi_d = HANDLE["min_diameter_cm"] / 100.0, HANDLE["max_diameter_cm"] / 100.0
        best_d = HANDLE["best_share_of_hand_length"] * hand_len_cm / 100.0
        st = step_cm / 100.0
        c0 = P.mean(0)
        found = []
        for d in _dirs(dir_step_deg):
            u = np.cross(d, [0.0, 0.0, 1.0] if abs(d[2]) < 0.9 else [1.0, 0.0, 0.0])
            u /= np.linalg.norm(u)
            v = np.cross(d, u)
            t = (P - c0) @ d
            pu, pv = (P - c0) @ u, (P - c0) @ v
            k = np.floor((t - t.min()) / st).astype(int)
            chains, done = [], []
            for b in range(int(k.max()) + 2):
                m = k == b
                blobs = _blobs(pu[m], pv[m]) if m.sum() >= 12 else []
                secs = []
                for bu, bv in blobs:
                    if len(bu) < 12:
                        continue
                    hull, per = _hull2(np.c_[bu, bv])
                    secs.append({"t": float(t.min() + (b + 0.5) * st), "dia": per / math.pi, "cu": float(bu.mean()),
                                 "cv": float(bv.mean()), "hull": hull, "w": float(np.ptp(bu)), "h": float(np.ptp(bv))})
                # each chain goes on with the piece of this section that follows its last one
                nxt, used = [], set()
                for ch in chains:
                    last = ch[-1]
                    cand = [(math.hypot(s_["cu"] - last["cu"], s_["cv"] - last["cv"]), i) for i, s_ in enumerate(secs)
                            if lo_d <= s_["dia"] <= hi_d and abs(s_["dia"] - last["dia"]) <= 0.5 / 100.0]
                    cand = [c for c in cand if c[0] <= 1.2 * st]
                    if cand:
                        _dd, i = min(cand)
                        used.add(i)
                        nxt.append(ch + [secs[i]])
                    else:
                        done.append(ch)
                for i, s_ in enumerate(secs):
                    if i not in used and lo_d <= s_["dia"] <= hi_d:
                        nxt.append([s_])
                chains = nxt
            for run in done + chains:
                if len(run) * step_cm >= HANDLE["min_run_cm"]:
                    found.append((d.copy(), u.copy(), v.copy(), run))
        scored = []
        for d, u, v, run in found:
            L = len(run) * step_cm
            dev = abs(np.mean([s_["dia"] for s_ in run]) - best_d) * 100.0
            # straight runs score higher: the middles' drift sideways over the run
            drift = math.hypot(run[-1]["cu"] - run[0]["cu"], run[-1]["cv"] - run[0]["cv"]) * 100.0
            scored.append((L - 0.8 * dev - 1.5 * drift, d, u, v, run, L))
        scored.sort(key=lambda x: -x[0])
        out = []
        for sc, d, u, v, run, L in scored:
            mid = c0 + d * np.mean([s_["t"] for s_ in run]) + u * np.mean([s_["cu"] for s_ in run]) + v * np.mean([s_["cv"] for s_ in run])
            if any(np.linalg.norm(mid - o["_mid"]) < 0.02 and abs(float(np.dot(d, o["_d"]))) > 0.85 for o in out):
                continue                               # the same run read along a nearly equal direction
            out.append({"_mid": mid, "_d": d, "axis": [float(x) for x in d], "u": [float(x) for x in u],
                        "v": [float(x) for x in v], "origin": [float(x) for x in c0], "length_cm": round(L, 1),
                        "score": round(sc, 2),
                        "diameter_cm": round(float(np.mean([s_["dia"] for s_ in run])) * 100.0, 2),
                        "sections": [{"t": s_["t"], "centre": [float(x) for x in (c0 + d * s_["t"] + u * s_["cu"] + v * s_["cv"])],
                                      "diameter_cm": round(s_["dia"] * 100.0, 2), "width_cm": round(s_["w"] * 100.0, 2),
                                      "depth_cm": round(s_["h"] * 100.0, 2),
                                      "ring": [[float(a), float(b)] for a, b in _ring(s_["hull"], 24)]} for s_ in run]})
            if len(out) >= 3:
                break
        for o in out:
            o.pop("_mid")
            o.pop("_d")
        return out

    def _part_openings(self, name, P, cell_cm=0.3, skip=()):
        """Holes through the object beside a part (a trigger guard, a handle's eye), found once over the whole part:
        a grid of rays across its thinnest direction, against all the object's parts; a cell no ray hits but walled by hits on both sides in both grid directions
        is open; touching open cells are one opening. Returns each opening's cells (object frame, m), middle, area
        and its through direction."""
        # rays against the whole object (another part may close the hole: a slide over a frame's open top), in
        # this part's frame
        bvh = self._part_bvh_local(None, name, skip)
        if bvh is None or len(P) < 60:
            return []
        c = P.mean(0)
        _u, _s, vt = np.linalg.svd(P - c, full_matrices=False)
        a1, a2, thin = vt[0], vt[1], vt[2]
        e1, e2 = (P - c) @ a1, (P - c) @ a2
        step = cell_cm / 100.0
        n1, n2 = int(np.ptp(e1) / step) + 3, int(np.ptp(e2) / step) + 3
        far = float(np.ptp((P - c) @ thin)) + 0.1
        grid = np.zeros((n1, n2), dtype=bool)
        for i in range(n1):
            for j in range(n2):
                q = c + a1 * (e1.min() + (i - 1) * step) + a2 * (e2.min() + (j - 1) * step)
                # the cell's middle and its four corners: a bar thinner than a cell (a guard's bottom) still walls it
                for du, dv in ((0, 0), (-0.45, -0.45), (0.45, -0.45), (-0.45, 0.45), (0.45, 0.45)):
                    qq = q + a1 * (du * step) + a2 * (dv * step)
                    if bvh.ray_cast(Vector(qq - thin * far), Vector(thin), 2 * far)[0] is not None:
                        grid[i, j] = True
                        break
        walled = np.zeros_like(grid)
        for i in range(n1):
            for j in range(n2):
                if not grid[i, j]:
                    walled[i, j] = grid[:i, j].any() and grid[i + 1:, j].any() and grid[i, :j].any() and grid[i, j + 1:].any()
        seen, out = set(), []
        for i in range(n1):
            for j in range(n2):
                if not walled[i, j] or (i, j) in seen:
                    continue
                stack, cells = [(i, j)], []
                seen.add((i, j))
                while stack:
                    x0, y0 = stack.pop()
                    cells.append((x0, y0))
                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        x, y = x0 + dx, y0 + dy
                        if 0 <= x < n1 and 0 <= y < n2 and walled[x, y] and (x, y) not in seen:
                            seen.add((x, y))
                            stack.append((x, y))
                if len(cells) * cell_cm ** 2 < 1.0:     # under a square cm: a seam in the mesh, not a hole
                    continue
                pts = np.array([c + a1 * (e1.min() + (x - 1) * step) + a2 * (e2.min() + (y - 1) * step) for x, y in cells])
                out.append({"cells": pts, "centre": pts.mean(0), "area_cm2": round(len(cells) * cell_cm ** 2, 1), "through": thin})
        return out

    def _attach_openings(self, run, opens, reach_cm=8.0):
        """The part's openings within reach_cm of a run's inner end, each with its edge on the run's inner side
        (toward the rest of the object: a pistol's frame over its guard), where an extended index lies."""
        d = np.array(run["axis"])
        inner, outer = np.array(run["inner"]), np.array(run["outer"])
        inward = d if float((inner - outer) @ d) >= 0 else -d
        got = []
        for o in opens:
            pts = o["cells"]
            if float(np.min(np.linalg.norm(pts - inner, axis=1))) > reach_cm / 100.0:
                continue
            thin = o["through"]
            across = np.cross(thin, inward)
            across /= np.linalg.norm(across)
            # the opening's edge on the inner side: its cells in the top band of its height up the run (the top
            # 15 percent, at least 3 mm), a straight line fitted through them (a guard's top, under the frame)
            h = pts @ inward
            band = max(0.003, 0.15 * float(np.ptp(h)))
            top = pts[h >= h.max() - band]
            c_ = top.mean(0)
            _u, _s, vt = np.linalg.svd(top - c_, full_matrices=False)
            ld = vt[0] - thin * float(vt[0] @ thin)
            ld /= np.linalg.norm(ld)
            t_ = (top - c_) @ ld
            got.append({"centre": [float(x) for x in o["centre"]], "area_cm2": o["area_cm2"], "through": [float(x) for x in thin],
                        "edge": [[float(x) for x in (c_ + ld * t_.min())], [float(x) for x in (c_ + ld * t_.max())]]})
        got.sort(key=lambda g_: float(np.linalg.norm(np.array(g_["centre"]) - inner)))
        return got

    def _part_bvh_local(self, part, frame_part=None, skip=()):
        """A part's polygons as a BVH in frame_part's frame (default its own); part None: the whole object's, less
        the parts in skip."""
        c = getattr(self, "_pbvh", {})
        ck = (part, frame_part, tuple(sorted(skip)))
        if ck in c:
            return c[ck]
        dg = bpy.context.evaluated_depsgraph_get()
        me_ob = self.rifle_mesh
        ev = me_ob.evaluated_get(dg)
        me = ev.to_mesh()
        Fi = self._obj_frame(frame_part or part).inverted() @ me_ob.matrix_world
        V = [Fi @ v.co for v in me.vertices]
        names = {g.index: g.name for g in me_ob.vertex_groups}
        vg = []
        for v in me_ob.data.vertices:
            gm = max(v.groups, key=lambda x: x.weight) if v.groups else None
            vg.append(names.get(gm.group) if gm else None)
        ps = []
        for p in me.polygons:
            gs = [vg[i] or "body" for i in p.vertices]
            own = max(set(gs), key=gs.count)
            if (part is None and own not in skip) or not any(vg) or own == part:
                ps.append(tuple(p.vertices))
        ev.to_mesh_clear()
        bvh = BVHTree.FromPolygons(V, ps) if ps else None
        c[ck] = bvh
        self._pbvh = c
        return bvh

    def read_object(self, parts=None, hand_length_cm=None, inner_toward=None):
        """The object read from its geometry: each part's size, its runs a hand can wrap (axis, sections with their
        girth and middles, the inner end: the end nearer the rest of the object, or nearer inner_toward, a point in
        the object's frame) and the openings beside each run. Kept for the purposes and the hold search. hand_length_cm:
        the hand's length (Pose Lab's body_size), for the best handle size."""
        self._need()
        if hand_length_cm is None:
            try:
                hand_length_cm = self.body_size("r")["r"]["measures"]["hand_length_cm"]
            except Exception:
                hand_length_cm = 19.0
        self._pbvh = {}
        pts = self._part_points()
        allp = np.vstack([p for p, _n in pts.values()])
        out = {}
        for name, (P, N) in pts.items():
            if parts and name not in parts:
                continue
            ext = np.ptp(P, 0) * 100.0
            rec = {"points": len(P), "size_cm": [round(float(x), 1) for x in ext], "runs": []}
            if len(P) >= 60:
                runs = self._runs(P, hand_length_cm)
                rest = np.vstack([p for k, (p, _n) in pts.items() if k != name]) if len(pts) > 1 else P
                for r in runs:
                    d = np.array(r["axis"])
                    a = np.array(r["sections"][0]["centre"])
                    b = np.array(r["sections"][-1]["centre"])
                    tgt = np.array(inner_toward) if inner_toward is not None else rest.mean(0) if len(pts) > 1 else allp.mean(0)
                    # the end nearer the rest of the object (a grip's top meets the frame): with only one part, the
                    # end nearer the part's own bulk off the run
                    if inner_toward is None and len(pts) == 1:
                        off = P[np.abs((P - (a + b) / 2) @ d) > np.linalg.norm(b - a) / 2]
                        tgt = off.mean(0) if len(off) else (a + b) / 2
                    inner, outer = (a, b) if np.linalg.norm(a - tgt) < np.linalg.norm(b - tgt) else (b, a)
                    r["inner"], r["outer"] = [float(x) for x in inner], [float(x) for x in outer]
                    r["part"] = name
                    rec["runs"].append(r)
                # a part much smaller than this one does not close an opening (a trigger inside its guard); a
                # large one does (a slide over a frame's open top)
                small = tuple(k for k, (pk, _nk) in pts.items() if k != name and len(pk) < 0.1 * len(P))
                opens = self._part_openings(name, P, skip=small)
                for r in rec["runs"]:
                    r["openings"] = self._attach_openings(r, opens)
                self._part_P = getattr(self, "_part_P", {})
                self._part_P[name] = P
            out[name] = rec
        self._object = {"parts": out, "hand_length_cm": hand_length_cm}
        return self._object

    def object_summary(self, obj=None):
        """The reader's answer without its sample rings, in cm in the object's frame: for a person to look at."""
        obj = obj or getattr(self, "_object", None) or {"parts": {}}
        s = {}
        for name, rec in obj["parts"].items():
            s[name] = {"size_cm": rec["size_cm"], "runs": [{
                "length_cm": r["length_cm"], "diameter_cm": r["diameter_cm"],
                "axis": [round(x, 3) for x in r["axis"]],
                "inner_cm": [round(x * 100, 2) for x in r["inner"]], "outer_cm": [round(x * 100, 2) for x in r["outer"]],
                "girth_cm": [round(x["diameter_cm"] * math.pi, 1) for x in r["sections"][::4]],
                "openings": [{"centre_cm": [round(x * 100, 2) for x in o["centre"]], "area_cm2": o["area_cm2"],
                              "edge_cm": [[round(x * 100, 2) for x in e] for e in o["edge"]]} for o in r["openings"]]}
                for r in rec["runs"]]}
        return s

    def suggest_marks(self, moving=(), obj=None):
        """Marks suggested from the reader, for a person to confirm: the part with the best wrappable run a power
        wrap (its best run), a second run long enough for two hands a support, each part that moves (moving: part
        names, a rig's moving bones) no touch. Each with why. Nothing is final until confirmed."""
        obj = obj or getattr(self, "_object", None) or self.read_object()
        sug = []
        cands = sorted(((r["score"], name, i, r) for name, rec in obj["parts"].items() if name not in moving
                        for i, r in enumerate(rec["runs"])), key=lambda x: -x[0])
        if cands:
            _sc, name, i, r = cands[0]
            sug.append({"part": name, "role": "hold", "purpose": "power_wrap", "run": i, "web_end": "inner",
                        "thumb": "opposed", "index": "wrap", "confirmed": False,
                        "why": "the longest run a hand can wrap: %.1f cm long, %.1f cm across (best for this hand %.1f)"
                               % (r["length_cm"], r["diameter_cm"], HANDLE["best_share_of_hand_length"] * obj["hand_length_cm"])})
            if r["length_cm"] >= 15.0:
                sug.append({"part": name, "role": "hold", "purpose": "support", "run": i, "confirmed": False,
                            "why": "the run is %.1f cm long: room for a second hand" % r["length_cm"]})
            if r["openings"]:
                sug[0]["index"] = "extended"
                sug[0]["index_line"] = 0
                sug[0]["why"] += "; an opening of %.1f square cm beside its inner end (the index may lie along it)" % r["openings"][0]["area_cm2"]
        for m in moving:
            sug.append({"part": m, "role": "notouch", "confirmed": False, "why": "it moves: a hand on it is in its way"})
        return sug

    # --- the shoulder girdle -----------------------------------------------------------------------------------
    def _clav_rest(self, side):
        cn = "clavicle_" + side
        if cn not in self.arm.pose.bones:
            return None
        if not hasattr(self, "_clav0"):
            self._clav0 = {}
        if side not in self._clav0:
            self._clav0[side] = self.arm.pose.bones[cn].rotation_quaternion.copy()
        return cn

    def shoulder(self, side, protraction_deg=0.0, elevation_deg=0.0):
        """The shoulder girdle moved as a body moves it: the clavicle turned at its inner end, forward
        (protraction, below 0 back: retraction) about the body's up line and up (elevation, below 0 down) about its
        forward line, each kept inside its working range (CLAVICLE). The arm hangs from it: call before placing
        the hand. Returns the degrees used and whether a request was clipped."""
        cn = self._clav_rest(side)
        if cn is None:
            return {"protraction_deg": 0.0, "elevation_deg": 0.0, "clipped": False, "note": "the rig has no clavicle"}
        pr = max(-CLAVICLE["retraction_deg"], min(CLAVICLE["protraction_deg"], protraction_deg))
        el = max(-CLAVICLE["depression_deg"], min(CLAVICLE["elevation_deg"], elevation_deg))
        pb = self.arm.pose.bones[cn]
        pb.rotation_quaternion = self._clav0[side]
        bpy.context.view_layer.update()
        from lab import VIEW_FWD, VIEW_UP
        C = self._bw(cn).translation.copy()
        S = self._bw(self._n("upperarm", side)).translation.copy()
        out = (S - C)
        # forward: the turn about up that moves the shoulder's end toward VIEW_FWD; up: about forward toward VIEW_UP
        ax_up = VIEW_UP.normalized()
        sg_p = 1.0 if out.cross(VIEW_FWD).dot(ax_up) >= 0 else -1.0
        q1 = Quaternion(ax_up, math.radians(pr) * sg_p)
        ax_f = VIEW_FWD.normalized()
        o1 = q1 @ out
        sg_e = 1.0 if o1.cross(VIEW_UP).dot(ax_f) >= 0 else -1.0
        q2 = Quaternion(ax_f, math.radians(el) * sg_e)
        self._turn_about(cn, (q2 @ q1).axis if (q2 @ q1).angle > 1e-9 else Vector((0, 0, 1)), math.degrees((q2 @ q1).angle), C)
        bpy.context.view_layer.update()
        return {"protraction_deg": round(pr, 1), "elevation_deg": round(el, 1),
                "clipped": abs(pr - protraction_deg) > 1e-6 or abs(el - elevation_deg) > 1e-6}

    # --- the purposes, measured --------------------------------------------------------------------------------
    def _run_world(self, run):
        """A run's sections as world rings (24 points each) and middles, with its axis and inner end in the world,
        as its part is posed now."""
        F = self._obj_frame(run.get("part"))
        Fq = F.to_quaternion()
        d, u, v = Vector(run["axis"]), Vector(run["u"]), Vector(run["v"])
        o = Vector(run["origin"])
        secs = []
        for s_ in run["sections"]:
            ring = [F @ (o + d * s_["t"] + u * a + v * b) for a, b in s_["ring"]]
            secs.append((F @ Vector(s_["centre"]), ring))
        return {"axis": (Fq @ d).normalized(), "inner": F @ Vector(run["inner"]), "outer": F @ Vector(run["outer"]), "sections": secs}

    def _covered(self, pts, segs, band_cm=0.35):
        """Which points a capsule's skin lies within band_cm of (or inside)."""
        out = []
        for p in pts:
            ok = False
            for _n, a, b, r in segs:
                if self._seg_point_cm(a, b, r, p) <= band_cm:
                    ok = True
                    break
            out.append(ok)
        return out

    def _hand_skin(self, side):
        """The drawn skin of a hand (it and its digits, the glove as drawn), world points as posed now (n x 3)."""
        hand = self._n("hand", side)
        bones = self.arm.data.bones
        inside = {b.name for b in bones if b.name == hand or any(q.name == hand for q in b.parent_recursive)}
        pts = [p for p, b in self._skin_points() if b in inside]
        return np.array([[p.x, p.y, p.z] for p in pts]) if pts else np.zeros((0, 3))

    def _skin_covered(self, pts, skin, band_cm):
        """Which points have a skin vertex within band_cm of them."""
        if not len(pts) or not len(skin):
            return [False] * len(pts)
        Q = np.array([[p.x, p.y, p.z] for p in pts])
        out = []
        for q in Q:
            out.append(bool(np.min(np.sum((skin - q) ** 2, axis=1)) <= (band_cm / 100.0) ** 2))
        return out

    def _hand_span(self, side, RW):
        """The run's sections the hand spans: those between its index and little finger knuckles along the axis,
        a cm more each way."""
        ax = RW["axis"]
        a = self._bw(self._n("finger", side, "index", "01")).translation
        b = self._bw(self._n("finger", side, "pinky", "01")).translation
        ta, tb = sorted(((a - RW["inner"]).dot(ax), (b - RW["inner"]).dot(ax)))
        return [s_ for s_ in RW["sections"] if ta - 0.01 <= (s_[0] - RW["inner"]).dot(ax) <= tb + 0.01]

    def _thumb_dir(self, side):
        t2 = self._bw(self._n("finger", side, "thumb", "02")).translation
        return (self._finger_tip("thumb", side) - t2).normalized()

    def _index_dir(self, side):
        i1 = self._bw(self._n("finger", side, "index", "01")).translation
        return (self._finger_tip("index", side) - i1).normalized()

    def purpose_thumb_inline(self, side, mark, glove_cm=0.0, along=None):
        """along: the direction the thumb lies along (the index finger's, else this)."""
        P = PURPOSES["thumb_inline"]
        TL = self._thumb_limits()
        tdir = self._thumb_dir(side)
        ref = along if along is not None else self._index_dir(side)
        ang = math.degrees(tdir.angle(ref)) if tdir.length and ref.length else 180.0
        end = abs(self._thumb_report(side, TL)[0].get("thumb_03", [0, 0])[0])
        tree = self._tree()
        gap = self._digit_gap(side, "thumb", tree, glove_cm, False)
        lo, hi = P["gap_cm"]
        err = max(0.0, ang - P["angle_max_deg"]) / 10.0 + max(0.0, end - P["end_bend_max_deg"]) / 10.0 + max(0.0, gap - hi) + max(0.0, lo - gap)
        return {"purpose": "thumb_inline", "met": err <= 1e-6, "error": round(err, 3), "angle_deg": round(ang, 1),
                "angle_max_deg": P["angle_max_deg"], "end_bend_deg": round(end, 1), "end_bend_max_deg": P["end_bend_max_deg"],
                "gap_cm": round(gap, 2), "gap_band_cm": P["gap_cm"]}

    def purpose_index_extended(self, side, mark, glove_cm=0.0):
        P = PURPOSES["index_extended"]
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        op = run["openings"][mark.get("index_line", 0)]
        F = self._obj_frame(mark["part"])
        e0, e1 = F @ Vector(op["edge"][0]), F @ Vector(op["edge"][1])
        thru = (F.to_quaternion() @ Vector(op["through"])).normalized()
        ld = (e1 - e0)
        ld = (ld - thru * ld.dot(thru)).normalized()
        # up the grip: from the run's outer end to its inner end (the run may end below the opening, so the way
        # from the edge to the inner end is not always up)
        upw = (F @ Vector(run["inner"]) - F @ Vector(run["outer"])).normalized()
        upw = (upw - ld * upw.dot(ld) - thru * upw.dot(thru)).normalized()     # in the side view, square to the edge
        pts = [self._bw(self._n("finger", side, "index", j)).translation for j in ("02", "03")] + [self._finger_tip("index", side)]
        offs = [(p - e0).dot(upw) * 100.0 for p in pts]
        fdir = (pts[-1] - pts[0])
        fdir = (fdir - thru * fdir.dot(thru)).normalized()
        ang = math.degrees(min(fdir.angle(ld), fdir.angle(-ld))) if fdir.length else 90.0
        gap = self._digit_gap(side, "index", self._tree(), glove_cm, False)
        lo, hi = P["band_cm"]
        err = sum(max(0.0, o - hi) + max(0.0, lo - o) for o in offs) / len(offs) + max(0.0, ang - P["angle_max_deg"]) / 10.0 + max(0.0, gap - P["gap_max_cm"])
        return {"purpose": "index_extended", "met": err <= 1e-6, "error": round(err, 3), "offsets_cm": [round(o, 2) for o in offs],
                "band_cm": P["band_cm"], "angle_deg": round(ang, 1), "angle_max_deg": P["angle_max_deg"], "gap_cm": round(gap, 2)}

    def arm_excess(self, side):
        """How far past its limits the arm is, in degrees summed (0 inside every range): the wrist's bend, its
        side bend (radial, ulnar), the forearm's roll, an elbow locked straight. A seed 2 degrees over on the wrist
        is a start the search can mend; one 75 degrees over on the forearm is not."""
        Lm = self._limits()[0]
        rep_, bad = self._arm_report(side, Lm)
        if rep_.get("forearm_rotation_deg") is None and bad:
            return 200.0 + 10.0 * len(bad)            # an arm the report cannot read (out of reach): no start
        w = abs(rep_.get("wrist_bend_deg", 0.0))
        rad = rep_.get("wrist_radial_deg", 0.0)
        roll = rep_.get("forearm_rotation_deg", 0.0) or 0.0
        el = rep_.get("elbow_bend_deg", 90.0)
        ex = max(0.0, w - Lm["wrist_max_deg"]) + max(0.0, rad - Lm["wrist_radial_deg"]) + max(0.0, -rad - Lm["wrist_ulnar_deg"])
        ex += max(0.0, roll - Lm["supination_deg"]) + max(0.0, -roll - Lm["pronation_deg"])
        ex += max(0.0, Lm["elbow_bend_min_deg"] + 5.0 - el)
        return round(ex + 5.0 * len([b for b in bad if "shoulder" in b or "across" in b or "behind" in b]), 1)

    def purpose_comfort(self, side):
        P = PURPOSES["comfort"]
        rep_, bad = self._arm_report(side, self._limits()[0])
        w = abs(rep_.get("wrist_bend_deg", 0.0))
        roll = abs(rep_.get("forearm_rotation_deg", 0.0))
        el = rep_.get("elbow_bend_deg", 90.0)
        cl = getattr(self, "_clav_now", {}).get(side, (0.0, 0.0))
        share = max(abs(cl[0]) / (CLAVICLE["protraction_deg"] if cl[0] >= 0 else CLAVICLE["retraction_deg"]),
                    abs(cl[1]) / (CLAVICLE["elevation_deg"] if cl[1] >= 0 else CLAVICLE["depression_deg"]))
        lo, hi = P["elbow_deg"]
        err = (max(0.0, w - P["wrist_max_deg"]) / 5.0 + max(0.0, roll - P["roll_max_deg"]) / 10.0 + max(0.0, lo - el) / 10.0
               + max(0.0, el - hi) / 10.0 + max(0.0, share - P["clavicle_share_max"]) * 5.0 + 10.0 * len(bad))
        return {"purpose": "comfort", "met": err <= 1e-6, "error": round(err, 3), "wrist_deg": round(w, 1),
                "wrist_max_deg": P["wrist_max_deg"], "roll_deg": round(roll, 1), "elbow_deg": round(el, 1),
                "clavicle_deg": [round(x, 1) for x in cl], "arm_rules_broken": bad}

    def hold_purposes(self, side, marks, glove_cm=0.0, other=True):
        """Every purpose the marks give this hand, judged on the contact model (contactlab.contact_purposes): the
        drawn skin against the object's labelled surface and, with other, the other hand (a hand placed first is
        not charged for the other hand still hanging in its rest pose). One ruler for the search and the gate."""
        return self.contact_purposes(side, marks, glove_cm, other=other)

    # --- the hold search: the whole arm, the purposes as the score ------------------------------------------------
    def _hold_score(self, side, marks, glove_cm, other, soft, tree):
        """The search's score is the gate's ruler: each purpose's error (the contact pairs, the presses on the drawn
        skin, the comfort) 10, each body rule broken 200."""
        res = self.hold_purposes(side, marks, glove_cm, other=other)
        # the hand's job first: the grip itself (a power wrap, a support) counts three times its refinements (the
        # thumb's and the index's lines), so no search trades the palm on the grip for a finer line
        PRIMARY = {"power_wrap": 3.0, "support": 3.0}
        perr = sum(r["error"] * PRIMARY.get(r["purpose"], 1.0) for r in res)
        # the hard rules (no clipping either way, digits as a holding hand's) are never traded: each broken one costs
        # as much as a broken body rule
        hard = sum(1 for r in res if r.get("hard") and not r["met"])
        FL = self._limits()[1]
        TL = self._thumb_limits()
        fbad = len(self._finger_report(side, FL)[1]) + len(self._thumb_report(side, TL)[1])
        abad = len(self._arm_report(side, self._limits()[0])[1])
        sc = 10.0 * perr + 200.0 * (fbad + abad + hard)
        pr = next((r for r in res if r["purpose"] == "press"), {})
        return sc, {"purposes_error": round(perr, 3), "press": round(pr.get("object_cm2", 0.0) + pr.get("hand_cm2", 0.0), 3),
                    "presses": [], "digit_rules": fbad, "arm_rules": abad, "purposes": res}

    def web_point(self, mark, palm_dir, step_cm=0.25, most_cm=6.0, overhang_cm=0.8, crook_cm=0.5):
        """Where the web of the hand sits on a power wrap. The part's surface on the palm's side (palm_dir, object
        frame) is followed up from the run, past its inner end: how far it reaches out that way at each height.
        Where it reaches out overhang_cm or more past the run's end (a beavertail, a hammer's head, a lip) the web
        sits crook_cm into the crook under the farthest reach; with no overhang, crook_cm under the surface's end.
        Returns the point (object frame, m) and its height past the run's end (cm)."""
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        P = self._part_P[mark["part"]]
        d = np.array(run["axis"])
        inner, outer = np.array(run["inner"]), np.array(run["outer"])
        inward = d if float((inner - outer) @ d) >= 0 else -d
        pd = np.array(palm_dir, dtype=float)
        pd = pd - inward * float(pd @ inward)
        pd /= np.linalg.norm(pd)
        side_ = np.cross(inward, pd)
        h = (P - inner) @ inward
        prof = []                                      # (height, reach, the surface point)
        k = 0.0
        while k <= most_cm / 100.0 + 1e-9:
            m = (np.abs(h - k) <= step_cm / 200.0) & (np.abs((P - inner) @ side_) <= 0.012)
            if m.sum() < 3:
                break
            off = (P[m] - inner) @ pd
            e = float(np.max(off))
            if prof and e < prof[0][1] - 0.01:
                break                                  # the surface turns away: past the part's end on this side
            prof.append((k, e, P[m][int(np.argmax(off))]))
            k += step_cm / 100.0
        if not prof:
            return [float(x) for x in (inner + pd * 0.01)], 0.0
        # the reach smoothed over its neighbours (the surface is sampled, its farthest point jitters)
        es = [max(x[1] for x in prof[max(0, i - 1):i + 2]) for i in range(len(prof))]
        kmax = int(np.argmax(es))
        if es[kmax] - es[0] >= overhang_cm / 100.0:
            target = es[kmax] - crook_cm / 100.0
            i = kmax
            while i > 0 and es[i] > target:
                i -= 1
        else:
            i = len(prof) - 1
            while i > 0 and prof[i][0] > prof[-1][0] - crook_cm / 100.0:
                i -= 1
        q, kk = prof[i][2], prof[i][0]
        return [float(x) for x in q], round(kk * 100.0, 2)

    @deprecated()
    def open_dir(self, other, mark, glove_cm=0.0):
        """Which way the run is left open by the other hand (as it holds now): the direction, square to the run's
        axis and in the part's frame, from the run's middle line to the middle of the girth the other hand does not
        cover, over the sections it spans; and the open share. A support hand's palm goes there."""
        P = PURPOSES["support"]
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        g = glove_cm / 100.0
        theirs = [(n, a, b, r + g) for n, a, b, r in self._segments(other) if not n.startswith("forearm")]
        span = self._hand_span(other, RW) or RW["sections"]
        acc, n_open, n_all = Vector(), 0, 0
        skin = self._hand_skin(other)
        for cen, ring in span:
            cov = self._skin_covered(ring, skin, P["band_cm"])
            for q, c in zip(ring, cov):
                n_all += 1
                if not c:
                    v = q - cen
                    acc += (v - RW["axis"] * v.dot(RW["axis"])).normalized()
                    n_open += 1
        if acc.length < 1e-9:
            return None, 0.0
        Fq = self._obj_frame(mark["part"]).to_quaternion()
        return list(Fq.inverted() @ acc.normalized()), round(n_open / float(max(1, n_all)), 3)

    @deprecated()
    def seed_hold(self, side, mark, palm_dir=None, roll_deg=0.0, along_cm=0.0, flip=False):
        """A first place for a hand on its mark, from the run's geometry alone: the hand turned so its palm faces
        into the run from palm_dir (object frame; without one the run's own u), turned roll_deg about the run's
        axis, and its knuckles' line (little finger to index) runs along the axis toward the inner end; then
        moved so the palm's middle sits a palm's thickness off the surface, at the height where the web meets its
        point (a grip's web under its overhang) or, with no web point, at the run's middle; along_cm further from
        the inner end; flip: the knuckles' line the other way (the index toward the outer end). Returns where the
        hand went."""
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        F = self._obj_frame(mark["part"])
        ax = RW["axis"]
        if (RW["inner"] - RW["outer"]).dot(ax) < 0:
            ax = -ax
        if flip:
            ax = -ax                                  # the index toward the other end (a round handle has no top)
        pd = (F.to_quaternion() @ Vector(palm_dir)).normalized() if palm_dir is not None else (F.to_quaternion() @ Vector(run["u"])).normalized()
        pd = (pd - ax * pd.dot(ax)).normalized()
        pd = Quaternion(ax, math.radians(roll_deg)) @ pd
        hn = self._n("hand", side)
        # the turn: the hand's palm facing and knuckle line onto -pd and ax
        f0 = self._palm_facing(side)
        a0 = (self._bw(self._n("finger", side, "index", "01")).translation
              - self._bw(self._n("finger", side, "pinky", "01")).translation)      # the knuckles, little finger to index
        a0 = (a0 - f0 * a0.dot(f0)).normalized()
        cur = Matrix((f0, a0, f0.cross(a0))).transposed()
        f1, a1 = -pd, ax
        tgt = Matrix((f1, a1, f1.cross(a1))).transposed()
        Rm = (tgt @ cur.transposed()).to_4x4()
        H = self._bw(hn).copy()
        pc = H.translation.lerp(self._bw(self._n("finger", side, "middle", "01")).translation, 0.5)
        M = Matrix.Translation(pc) @ Rm @ Matrix.Translation(-pc) @ H
        self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
        bpy.context.view_layer.update()
        # the move: the palm's middle a palm's thickness off the surface on the palm's side, at its height
        palm_r = (self._fitted_radii() or {}).get("palm", 0.018)
        secs = RW["sections"]
        web_t = None
        if getattr(self, "_part_P", {}).get(mark["part"]) is not None and palm_dir is not None:
            wp, _past = self.web_point(mark, list(F.to_quaternion().inverted() @ pd))
            web_t = F @ Vector(wp)
        pc = self._bw(hn).translation.lerp(self._bw(self._n("finger", side, "middle", "01")).translation, 0.5)
        web = self._anchor(side, "web")
        hgt = lambda q: (q - RW["inner"]).dot(ax)         # noqa: E731  (height along the run, from its inner end)
        if web_t is not None:
            # the palm's middle as far below the web as it is now, the web just under its point
            h_pc = hgt(web_t) - 0.004 - (hgt(web) - hgt(pc))
        else:
            h_pc = hgt(secs[len(secs) // 2][0])
        h_pc -= along_cm / 100.0
        at = min(secs, key=lambda s_: abs(hgt(s_[0]) - h_pc))
        edge = max(at[1], key=lambda q: (q - at[0]).dot(pd))
        goal = at[0] + pd * ((edge - at[0]).dot(pd) + palm_r) + ax * (h_pc - hgt(at[0]))
        M = Matrix.Translation(goal - pc) @ self._bw(hn)
        short = self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
        bpy.context.view_layer.update()
        return {"short_cm": round(short, 2), "roll_deg": roll_deg}

    @deprecated()
    def hold_object(self, side, marks, glove_cm=0.0, other=True, soft=None, generations=None, pop=None, seed=None,
                    wrap=None, **_old):
        """A hand's hold on its marks: the target solver (target_hold), inside the walls. The old random search
        this name ran priced a broken body rule against contact and is gone; its search settings are ignored."""
        return self.target_hold(side, marks, glove_cm=glove_cm, other=other, wrap=wrap)

    # --- the walls: a real body's limits and physics. Never scored, never weighed, never traded ---------------
    WALL_TOL = {"deg": 0.5, "clip_cm2": 0.02, "clip_cm": 0.005}
    REPAIR_FRAME_S = 3.0      # a moving frame's repair stops after this long (the run has a time budget)

    def angle_faults(self, side):
        """The walls that need no contact read: the arm's rules, the fingers' and thumb's ranges, a holding digit's
        limits (v0.9.0). A step that breaks one of these is undone before the costly contact read."""
        return {k: v for k, v in self.wall_faults(side, angles_only=True).items()}

    def wall_faults(self, side, other=True, angles_only=False, within=None):
        """Every wall this hand breaks as posed now, each with how far past it is: the arm's rules, the fingers' and
        the thumb's ranges, a holding digit's limits (degrees past), and clipping (skin in the object past its give,
        the object in the flesh past it, a hand in the other: cm2 and cm). Empty when the hand is inside every wall."""
        import re
        out = {}

        def add(kind, msg):
            m = re.search(r"(-?\d+(?:\.\d+)?)\D*\(at most (-?\d+(?:\.\d+)?)\)", msg)
            if m:
                key, over = kind + ":" + msg[:m.start(1)].strip(), abs(float(m.group(1)) - float(m.group(2)))
            else:
                key, over = kind + ":" + re.sub(r"\(.*?\)", "", msg).strip(), 1.0
            out[key] = max(out.get(key, 0.0), over)
        FL = self._limits()[1]
        TL = self._thumb_limits()
        for msg in self._arm_report(side, self._limits()[0])[1]:
            add("arm", msg)
        for msg in self._finger_report(side, FL)[1]:
            add("finger", msg)
        for msg in self._thumb_report(side, TL)[1]:
            add("thumb", msg)
        for msg in self.hold_digit_rule(side)["faults"]:
            add("holding", msg)
        if angles_only:
            return out
        if getattr(self, "_object", None):
            # within: the walls this pose must stay inside (the last kept pose's); the pierced test is skipped when the
            # skin's side alone already goes past their clipping, since it can only add to it (v0.9.0)
            bound = None
            if within is not None:
                bound = (within.get("clip:area_cm2", CLIP_NOISE_CM2) + self.WALL_TOL["clip_cm2"],
                         within.get("clip:deepest_cm", 0.0) + self.WALL_TOL["clip_cm"])
            cp = self.contact_pairs(side, other=other, clip_bound=bound)
            area = cp["press_cm2"] + cp["pierced_cm2"] + cp["hand_press_cm2"]
            deep = max(cp["deepest_cm"], cp["pierced_deep_cm"], cp["hand_deepest_cm"])
            if area > CLIP_NOISE_CM2 or deep > 0.0:
                out["clip:area_cm2"], out["clip:deepest_cm"] = area, deep
        else:
            sp = self.skin_presses(side, other=other)
            if not sp["ok"]:
                s_ = sp.get(side, {})
                out["clip:deepest_cm"] = max(s_.get("gun_cm", 0.0), s_.get("hand_cm", 0.0), 0.01)
        return out

    def _walls_hold(self, now, before):
        """True when no wall is broken now that was not broken before, and none is broken further."""
        for k, v in now.items():
            if k not in before:
                return False
            tol = self.WALL_TOL["clip_cm2"] if k == "clip:area_cm2" else (self.WALL_TOL["clip_cm"] if k.startswith("clip") else self.WALL_TOL["deg"])
            if v > before[k] + tol:
                return False
        return True

    @staticmethod
    def _walls_rank(w):
        """Fewer walls broken first, then by less past them."""
        return (len(w), round(sum(w.values()), 3))

    def _walled(self, side, other, step, tag="_wall"):
        """step() kept only if it leaves the hand inside every wall it was inside, and no further past any it was
        already past; else undone. Returns (kept, the walls now)."""
        w0 = self.wall_faults(side, other)
        self.snapshot("save", tag)
        step()
        bpy.context.view_layer.update()
        w1 = self.wall_faults(side, other)
        if self._walls_hold(w1, w0):
            self._cnt("kept")
            return True, w1
        self.snapshot("load", tag)
        self._cnt("undone at a wall")
        return False, w0

    # --- the target solver: each purpose says where its hand part goes, the hand goes there --------------------
    def _region_points(self, names):
        """World points of the labelled samples in these regions (fnmatch patterns), as the object is posed."""
        import fnmatch
        SW, SL = self._object_now()[2:4]
        keep = [k for k, l_ in enumerate(SL) if any(fnmatch.fnmatch(l_, n_) for n_ in names)]
        return SW[keep] if keep else np.zeros((0, 3))

    def _skin_part(self, side, patterns):
        """World points of this hand's skin labelled as any of patterns (fnmatch), as posed."""
        import fnmatch
        P, N, _A, _b, _t = self._skin_arrays()
        idx, lab = self._hand_label_arrays(side, P, N)
        sel = [int(i) for i, l_ in zip(idx, lab) if any(fnmatch.fnmatch(l_, p_) for p_ in patterns)]
        return P[sel] if sel else np.zeros((0, 3))

    @deprecated()
    def _fit_hand(self, side, pairs, share=1.0):
        """The hand moved and turned as one body by a weighted least-squares fit (Kabsch) of its points onto their
        targets: pairs [(source world point, target world point, weight)]. The arm reaches it."""
        if len(pairs) < 3:
            return None
        Sx = np.array([list(a) for a, _b, _w in pairs])
        Dx = np.array([list(b) for _a, b, _w in pairs])
        w = np.array([w_ for _a, _b, w_ in pairs])
        w = w / w.sum()
        cs, cd = (Sx * w[:, None]).sum(0), (Dx * w[:, None]).sum(0)
        Hm = ((Sx - cs) * w[:, None]).T @ (Dx - cd)
        U, _S, Vt = np.linalg.svd(Hm)
        dd = np.sign(np.linalg.det(Vt.T @ U.T))
        Rm = Vt.T @ np.diag([1.0, 1.0, dd]) @ U.T
        R = Matrix([list(Rm[0]), list(Rm[1]), list(Rm[2])]).to_4x4()
        if share < 1.0:
            # a share of the move: the turn by slerp, the shift in proportion
            q = Matrix([list(Rm[0]), list(Rm[1]), list(Rm[2])]).to_quaternion()
            R = Quaternion().slerp(q, share).to_matrix().to_4x4()
            cd = cs + (cd - cs) * share
        T = Matrix.Translation(Vector(list(cd))) @ R @ Matrix.Translation(-Vector(list(cs)))
        hn = self._n("hand", side)
        M = T @ self._bw(hn)
        short = self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
        bpy.context.view_layer.update()
        return short

    @deprecated()
    def _place_targets(self, side, mark):
        """Where the hand goes for its purpose, as fit pairs: the palm patch's middle onto the nearest point of its
        regions and its facing into the surface there (weight 3, the grip itself), the web onto its point (2), the
        knuckles' line along the run toward its inner end (1)."""
        own, far = self._sides_named(side)
        named = bool(mark.get("palm_dir")) and own != "side_?"
        RUN = ["back", "front", "side_?", "left", "right", "tang", "end"]
        if mark["purpose"] == "support":
            regs = [own] if named else RUN
        else:
            regs = ["back", own, "tang"] if named else RUN
        palm = self._skin_part(side, ["palm"])
        R = self._region_points(regs)
        if not len(palm) or not len(R):
            return []
        if mark["purpose"] == "power_wrap":
            # a power wrap seats the handle across the distal palm, by the knuckle crease (the base segments then
            # wrap it; seated at the palm's middle only the fingertips reach): the palm's half nearest the knuckles
            kn = np.array(list(self._bw(self._n("finger", side, "middle", "01")).translation))
            dk = np.linalg.norm(palm - kn, axis=1)
            palm = palm[dk <= np.median(dk)]
        pc = palm.mean(0)
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        ax = np.array(list((RW["inner"] - RW["outer"]).normalized()))
        o_ = np.array(list(RW["outer"]))
        if mark["purpose"] == "power_wrap":
            # the palm wraps the run: its target on the run's wall (a surface along the run, not an end or a tang's
            # top) and its facing toward the run's axis
            _D0, _w0, NR = self.sdf_query(R)
            wall = np.abs(NR @ ax) < 0.5
            if wall.any():
                R = R[wall]
        k = int(np.argmin(np.linalg.norm(R - pc, axis=1)))
        tgt = R[k]
        if mark["purpose"] == "power_wrap":
            c_ = o_ + ax * float(np.dot(tgt - o_, ax))          # the axis point level with the target
            inw = c_ - tgt
            inw = inw / max(1e-9, np.linalg.norm(inw))
        else:
            inw = -self.sdf_query(tgt[None, :])[2][0]          # a support palm lies flat on its panel
        f0 = np.array(list(self._palm_facing(side)))
        pairs = [(Vector(pc), Vector(tgt), 3.0),
                 (Vector(pc + f0 * 0.03), Vector(tgt + inw * 0.03), 3.0)]
        if os.environ.get("POSELAB_DEBUG"):
            print("DBG targets", side, mark["purpose"], "palm->tgt cm", round(float(np.linalg.norm(tgt - pc)) * 100, 1),
                  "facing.inw", round(float(np.dot(f0, inw)), 2))
        a0 = np.array(list(self._bw(self._n("finger", side, "index", "01")).translation
                           - self._bw(self._n("finger", side, "pinky", "01")).translation))
        a0 /= max(1e-9, np.linalg.norm(a0))
        if os.environ.get("POSELAB_DEBUG"):
            print("DBG knuckles.ax", round(float(np.dot(a0, ax)), 2), "web mark", bool(mark.get("_web")))
        if np.dot(a0, ax) < 0 and not (mark.get("_web") and mark["purpose"] == "power_wrap"):
            ax = -ax        # the knuckles keep their start's way along the run (the body chose it); a web point pins it
        pairs += [(Vector(pc + a0 * 0.03), Vector(tgt + ax * 0.03), 1.0), (Vector(pc - a0 * 0.03), Vector(tgt - ax * 0.03), 1.0)]
        if mark.get("_web") and mark["purpose"] == "power_wrap":
            web = np.array(list(self._anchor(side, "web")))
            wt = np.array(list(self._obj_frame(mark["part"]) @ Vector(mark["_web"])))
            Dw, _w2, Nw = self.sdf_query(wt[None, :])
            pairs.append((Vector(web), Vector(wt + Nw[0] * 0.009), 2.0))   # the web's middle is a cm inside its skin
        elif mark["purpose"] == "power_wrap":
            # no web point read: the web onto the run's surface nearest it, just pressing (a power wrap seats it)
            web_s = self._skin_part(side, ["web"])
            if len(web_s):
                wc = web_s.mean(0)
                kw = int(np.argmin(np.linalg.norm(R - wc, axis=1)))
                _Dw, _w2, Nw = self.sdf_query(R[kw][None, :])
                pairs.append((Vector(wc), Vector(R[kw] - Nw[0] * 0.001), 2.0))
        return pairs

    @deprecated()
    def _body_eases(self, side, keep):
        """The body helps when the arm is past a limit, in a set order: the elbow swung (40 to -40 in 10 degree
        steps), then the shoulder girdle (forward and up, a small grid), each taken only if it leaves the arm less
        past its limits; the hand kept where it is (keep: its matrix)."""
        best = self.arm_excess(side)
        if best <= 0.0:
            return 0.0
        self.snapshot("save", "_ease")
        choice = None
        for sw in (-40.0, -30.0, -20.0, -10.0, 10.0, 20.0, 30.0, 40.0):
            self._cnt("elbow swings tried")
            self.snapshot("load", "_ease")
            self._swing_elbow(side, sw)
            bpy.context.view_layer.update()
            ex = self.arm_excess(side)
            if ex < best - 0.5:
                best, choice = ex, ("swing", sw)
        if best > 0.0:
            for pr in (-10.0, -5.0, 5.0, 10.0):
                for el in (-6.0, 0.0, 6.0):
                    self._cnt("girdle places tried")
                    self.snapshot("load", "_ease")
                    self.shoulder(side, pr, el)
                    self._put_hand(side, keep)
                    bpy.context.view_layer.update()
                    ex = self.arm_excess(side)
                    if ex < best - 0.5:
                        best, choice = ex, ("shoulder", pr, el)
        self.snapshot("load", "_ease")
        if choice and choice[0] == "swing":
            self._swing_elbow(side, choice[1])
        elif choice:
            self.shoulder(side, choice[1], choice[2])
            if not hasattr(self, "_clav_now"):
                self._clav_now = {}
            self._clav_now[side] = (choice[1], choice[2])
            self._put_hand(side, keep)
        bpy.context.view_layer.update()
        return best

    def _descend(self, err_fn, moves, side, other, steps=(8.0, 4.0, 2.0), angles_only=False):
        """A fixed pattern search (no chance in it): each move (a function of a step in degrees) tried both ways
        at each step size. A step that breaks a wall, or goes further past one, is undone whatever it does for the
        aim (the walls are not in the error); a step that leaves the walls less broken is kept first; else a step
        is kept when the error falls. Until no move helps at the finest step. Returns the error."""
        self.snapshot("save", "_ds")
        e0 = err_fn()
        w0 = self.angle_faults(side) if angles_only else self.wall_faults(side, other)
        import time as _t
        for st in steps:
            improved = True
            while improved:
                improved = False
                self._cnt("passes")
                for mv in moves:
                    for sg in (1.0, -1.0):
                        # the run's time budget (self._deadline): stop here, keeping the best pose so far
                        dl = getattr(self, "_deadline", None)
                        if dl is not None and _t.time() > dl:
                            self._cnt("stopped at the time budget")
                            return e0
                        self._cnt("steps tried")
                        self.snapshot("save", "_ds")
                        mv(sg * st)
                        bpy.context.view_layer.update()
                        a1 = self.angle_faults(side)
                        if not self._walls_hold(a1, w0):
                            self.snapshot("load", "_ds")
                            self._cnt("undone at an angle wall")
                            continue
                        w1 = a1 if angles_only else self.wall_faults(side, other, within=w0)
                        if not self._walls_hold(w1, w0):
                            self.snapshot("load", "_ds")
                            self._cnt("undone at a wall")
                            continue
                        e1 = err_fn()
                        if self._walls_rank(w1) < self._walls_rank(w0) or e1 < e0 - 1e-6:
                            e0, w0, improved = e1, w1, True
                            self._cnt("kept")
                        else:
                            self.snapshot("load", "_ds")
                            self._cnt("no gain")
        return e0

    @deprecated()
    def _fit_index(self, side, mark, glove_cm, other=True):
        """An extended index turned joint by joint onto its line (its error from the purpose), inside the walls."""
        FL = self._limits()[1]

        def err():
            return self.purpose_index_extended(side, mark, glove_cm)["error"]

        def turn(j):
            def mv(deg):
                cur = self._finger_report(side, FL)[0]["index_" + j][0]
                self._set_curl(side, "index", j, cur + deg, FL)
            return mv
        return self._descend(err, [turn("01"), turn("02"), turn("03")], side, other)

    @deprecated()
    def _fit_pads(self, side, fingers, other=True):
        """Each wrapping finger's joints turned by a fixed step pattern so more of its pads rest on the surface (within
        0.1 cm): a closing stops the joints behind a touching fingertip, and this lays the base and middle segments
        down too. Inside the walls (no pad past its give, no joint past a holding finger's limits)."""
        FL = self._limits()[1]
        P_, N_ = self._skin_arrays()[:2]
        ix, lb = self._hand_label_arrays(side, P_, N_)
        for fi in fingers:
            sel = np.array([i for i, l_ in zip(ix, lb) if l_.startswith(fi)])
            if not len(sel):
                continue
            is_pad = np.array([lb[k].endswith("_pad") for k in range(len(lb)) if lb[k].startswith(fi)])

            def err(sel=sel, is_pad=is_pad):
                P, _N, A = self._skin_arrays()[:3]
                D = self.sdf_query(P[sel], want_normal=False)[0]
                on = is_pad & (D <= 0.1) & (D >= -0.15)
                return -float(A[sel][on].sum())

            def turn(js, fi=fi):
                # js: (joint, share) turned together; a base closing while its end opens rolls a finger down onto
                # the surface without pushing the touching tip in
                def mv(deg):
                    fr = self._finger_report(side, FL)[0]
                    for j, k in js:
                        self._set_curl(side, fi, j, fr["%s_%s" % (fi, j)][0] + deg * k, FL)
                return mv
            self._descend(err, [turn([("01", 1.0)]), turn([("02", 1.0)]), turn([("03", 1.0)]),
                                turn([("01", 1.0), ("03", -1.0)]), turn([("01", 1.0), ("02", -1.0)]),
                                turn([("02", 1.0), ("03", -1.0)])], side, other, steps=(6.0, 3.0))

    def _fit_thumb(self, side, mark, glove_cm, along=None, inline=True, other=True):
        """A thumb placed: its base turned about three axes and its two joints bent, a fixed step pattern, its pad
        just on the object (a gap of -0.1 cm wanted) and an inline thumb on the index's line (or along). Inside the
        walls: a turn that takes the base past its spread or the joints past their ranges, or the thumb into the
        object either way, is undone."""
        TL = self._thumb_limits()
        n1 = self._n("finger", side, "thumb", "01")
        P_, N_ = self._skin_arrays()[:2]
        ix, lb = self._hand_label_arrays(side, P_, N_)
        pad = np.array([i for i, l_ in zip(ix, lb) if l_.startswith("thumb") and l_.endswith("_pad")])

        def err():
            P = self._skin_arrays()[0]
            gap = float(self.sdf_query(P[pad], want_normal=False)[0].min()) if len(pad) else 5.0
            e = abs(gap + 0.1) / 0.3
            if inline:
                e += self.purpose_thumb_inline(side, mark, glove_cm, along=along)["error"]
            return e

        def base(ax):
            def mv(deg):
                head = self._bw(n1).translation.copy()
                self._turn_about(n1, Vector(ax), deg, head)
            return mv

        def bend(j):
            def mv(deg):
                if self.R.get("thumb_hinge") == "ray":
                    self._thumb_flex(side, j, deg)
                else:
                    tr = self._thumb_report(side, TL)[0]
                    b2, b3 = tr.get("thumb_02", [0, 0])[0], tr.get("thumb_03", [0, 0])[0]
                    self._set_thumb_bend(side, b2 + (deg if j == "02" else 0.0), b3 + (deg if j == "03" else 0.0), TL)
            return mv
        return self._descend(err, [base((1, 0, 0)), base((0, 1, 0)), base((0, 0, 1)), bend("02"), bend("03")], side, other)

    @deprecated()
    def _polish(self, side, marks, glove_cm, other, fingers):
        """The whole hold tuned on the gate's own ruler (the hold score) by a fixed step pattern: the hand moved
        along and turned about three axes (a step of 4 is 0.2 cm or 2 degrees), then each finger's joints and the
        thumb's bends. Returns the score."""
        FL = self._limits()[1]
        TL = self._thumb_limits()
        hn = self._n("hand", side)

        def err():
            return sum(r["error"] for r in self.hold_purposes(side, marks, glove_cm, other=other) if not r.get("hard"))

        def move(ax):
            def mv(st):
                M = Matrix.Translation(Vector(ax) * (st * 0.0005)) @ self._bw(hn)
                self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
            return mv

        def turn(ax):
            def mv(st):
                M = self._bw(hn)
                pc = Vector(list(self._skin_part(side, ["palm"]).mean(0)))
                R = Matrix.Translation(pc) @ Matrix.Rotation(math.radians(st * 0.5), 4, Vector(ax)) @ Matrix.Translation(-pc)
                M = R @ M
                self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
            return mv

        def curl(fi, j):
            def mv(st):
                self._set_curl(side, fi, j, self._finger_report(side, FL)[0]["%s_%s" % (fi, j)][0] + st, FL)
            return mv

        def tbend(j):
            def mv(st):
                if self.R.get("thumb_hinge") == "ray":
                    self._thumb_flex(side, j, st)
                else:
                    tr = self._thumb_report(side, TL)[0]
                    b2, b3 = tr.get("thumb_02", [0, 0])[0], tr.get("thumb_03", [0, 0])[0]
                    self._set_thumb_bend(side, b2 + (st if j == "02" else 0.0), b3 + (st if j == "03" else 0.0), TL)
            return mv
        def swing(st):
            self._swing_elbow(side, st * 2.0)                 # the elbow about the shoulder-wrist line, the hand kept
            self._roll_forearm(side)

        def girdle(k):
            def mv(st):
                keep = self._bw(hn).copy()
                if not hasattr(self, "_clav_now"):
                    self._clav_now = {}
                pr, el = self._clav_now.get(side, (0.0, 0.0))
                pr, el = (pr + st, el) if k == 0 else (pr, el + st)
                got = self.shoulder(side, pr, el)
                self._clav_now[side] = (got["protraction_deg"], got["elevation_deg"])
                self._put_hand(side, keep)
            return mv
        m = next(x for x in marks if x.get("role") == "hold" and x.get("hand", side) == side)
        run = self._object["parts"][m["part"]]["runs"][m.get("run", 0)]

        def around(st):
            # the hand turned round the run's axis (st x 1.5 degrees): a hand finds its comfortable place on a handle
            RW = self._run_world(run)
            ax = (RW["inner"] - RW["outer"]).normalized()
            c = RW["outer"].lerp(RW["inner"], 0.5)
            R = Matrix.Translation(c) @ Matrix.Rotation(math.radians(st * 1.5), 4, ax) @ Matrix.Translation(-c)
            M = R @ self._bw(hn)
            self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))

        def along(st):
            RW = self._run_world(run)
            ax = (RW["inner"] - RW["outer"]).normalized()
            M = Matrix.Translation(ax * (st * 0.001)) @ self._bw(hn)
            self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
        AX = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        moves = [around, along] + [move(a) for a in AX] + [turn(a) for a in AX] + [swing, girdle(0), girdle(1)]
        moves += [curl(fi, j) for fi in fingers for j in ("01", "02", "03")] + [tbend("02"), tbend("03")]
        return self._descend(err, moves, side, other, steps=(4.0, 2.0, 1.0))

    # --- the direct hold: the hand placed from the marks, the body solved backward (docs/DIRECT_HOLD_PLAN.md) ---
    def reach_now(self, side):
        """The arm's lengths and the shoulder-to-wrist distance as posed now (m): (l1, l2, |W - S|)."""
        S = self._bw(self._n("upperarm", side)).translation
        E = self._bw(self._n("lowerarm", side)).translation
        W = self._bw(self._n("hand", side)).translation
        return (E - S).length, (W - E).length, (W - S).length

    BODY_LABELS = ("palm", "palm_side", "palm_back", "web", "forearm")

    def start_check(self, side, other=True):
        """The valid-start rule (docs/DIRECT_HOLD_PLAN.md), after Steps A to C: the arm inside its walls, the hand's
        body (palm, web, the forearm) not in the object or the other hand past its give, and the arm able to reach.
        The fingers are not judged here: they are open and Step D closes them; they are judged after it. Returns
        (valid, the walls broken with how far past, the reach margin in cm)."""
        w = {k: v for k, v in self.wall_faults(side, other, angles_only=True).items() if k.startswith("arm:")}
        if getattr(self, "_object", None):
            cp = self.contact_pairs(side, other=other)
            area, deep = 0.0, 0.0
            by = {}
            for (lab, reg), v in cp["pairs"].items():
                if lab in self.BODY_LABELS and v[1] > 0.0:
                    area += v[1]
                    deep = max(deep, v[2])
                    k_ = "%s in %s" % (lab, reg)
                    by[k_] = (round(by.get(k_, (0.0, 0.0))[0] + v[1], 2), round(max(by.get(k_, (0.0, 0.0))[1], v[2]), 2))
            for lab, (a_, d_) in cp.get("pierced_by", {}).items():
                if lab in self.BODY_LABELS:
                    area += a_
                    deep = max(deep, d_)
                    by["gun inside the " + lab] = (round(a_, 2), round(d_, 2))
            self._start_by = by
            if area > CLIP_NOISE_CM2 or deep > 0.0:
                w["clip:body area_cm2"], w["clip:body deepest_cm"] = round(area, 2), round(deep, 2)
        l1, l2, d = self.reach_now(side)
        margin = (l1 + l2 - d) * 100.0
        if margin < 1.0:
            w["reach:short of a 1 cm margin"] = round(1.0 - margin, 2)
        return (not w), {k: round(float(v), 2) for k, v in w.items()}, round(margin, 2)

    def _pairs_world(self, pairs):
        """Touch pairs from the gun's frame into the world as the gun sits now (each in the frame of its "part",
        the gun's body by default, so they ride the gun). Each pair has a kind (the owner's rule: a target is
        measured as what it is): "point" (a hand point on one gun point: "at", its surface normal "normal", the hand
        point's depth under its skin "off_cm"), "line" (a hand point anywhere along a gun line: "at" a point on it,
        "dir" its direction, "normal" the surface's), "facing" (the palm facing into a surface: "normal" only),
        "direction" (a hand line, two anchors "line", along "dir"). "center": the point the hand turns about;
        "check": reported only, not a target."""
        out = []
        for p in pairs:
            F = self._obj_frame(p.get("part", "Root"))
            q = dict(p)
            q.setdefault("kind", "direction" if "line" in p else ("facing" if p.get("facing_only") else "point"))
            if "at" in p:
                q["at_w"] = F @ Vector(p["at"])
            if "normal" in p:
                q["normal_w"] = (F.to_3x3() @ Vector(p["normal"])).normalized()
            if "dir" in p:
                q["dir_w"] = (F.to_3x3() @ Vector(p["dir"])).normalized()
            if "at" in p and "normal" in p:
                q["T_w"] = q["at_w"] + q["normal_w"] * (p.get("off_cm", 0.0) / 100.0)
            out.append(q)
        return out

    @staticmethod
    def _pair_reading(p, hand_pt, facing, line_ends=None):
        """One pair's reading for a hand: the distance (cm) for a point, the distance off the line (cm) for a line,
        the angle (degrees) for a facing or a direction."""
        if p["kind"] == "point":
            return round((hand_pt - p["T_w"]).length * 100.0, 2)
        if p["kind"] in ("line", "band"):
            d = hand_pt - p["T_w"]
            return round((d - p["dir_w"] * d.dot(p["dir_w"])).length * 100.0, 2)
        if p["kind"] == "surface":
            return None
        if p["kind"] == "facing":
            return round(math.degrees(facing.angle(-p["normal_w"])), 1)
        a, b = line_ends
        return round(math.degrees((b - a).angle(p["dir_w"])), 1)

    def touch_report(self, side, pairs):
        """Each pair's reading as the hand and the gun are posed now: a point's distance (cm), a hand point's
        distance off its line (cm), a facing's or a hand line's angle (degrees). Checks are marked."""
        rep_ = {}
        f0 = self._palm_facing(side)
        for p in self._pairs_world(pairs):
            if p["kind"] == "surface":
                continue                                    # the wrapping fingers: read by their closing
            ends = (self._anchor(side, p["line"][0]), self._anchor(side, p["line"][1])) if "line" in p else None
            key = ("%s to %s" % tuple(p["line"])) if "line" in p else ("palm" if p["kind"] == "facing" else p["hand"])
            key = "%s (%s%s)" % (key, p["kind"], ", check" if p.get("check") else "")
            rep_[key] = self._pair_reading(p, self._anchor(side, p["hand"]) if "hand" in p else None, f0, ends)
        return rep_

    # --- measured, not guessed (the owner's rule, 2026-10-21): every hand point a touch target uses is a point of the
    # real skin, found on the mesh and kept in a cache file next to the code (poselab_mcp/data/measured), in the
    # frame of the bone that carries it, with what it was measured from ---------------------------------------------
    MEASURE_VERSION = 1

    def _rig_key(self):
        """The arms' identity for the measurement cache: a hash of the arms, the idle pose and the bone edits files."""
        import hashlib
        h = hashlib.sha1(("measure %d" % self.MEASURE_VERSION).encode())
        R = self.R
        fold = R.get("folder", "")
        for k in ("arms", "idle", "bone_edits"):
            f = R.get(k)
            if not f:
                continue
            p = f if os.path.isabs(f) else os.path.join(fold, f)
            if os.path.exists(p):
                h.update(open(p, "rb").read())
        return h.hexdigest()[:16]

    def _measure_path(self):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "measured")
        os.makedirs(d, exist_ok=True)
        return os.path.join(d, "hands_%s.json" % self._rig_key())

    def _skin_exit(self, side, start, direction, reach=0.08):
        """Where a ray from start (inside the hand's flesh) along direction leaves this hand's skin: the last skin
        crossing within reach (world point), or None."""
        if side not in getattr(self, "_pierce_static", {}):
            self.contact_pairs(side, other=False)
        _th, vi_, polys_ = self._pierce_static[side]
        P = self._skin_arrays()[0]
        bvh = BVHTree.FromPolygons(P[vi_].tolist(), polys_)
        d = direction.normalized()
        o, last, n = start.copy(), None, 0
        while n < 16:
            hit = bvh.ray_cast(o, d, reach - (o - start).length)
            if hit[0] is None:
                break
            last = hit[0]
            o = hit[0] + d * 1e-5
            n += 1
        return Vector(last) if last is not None else None

    def measure_hand(self, side):
        """This hand's touch points measured on its skin, with the fingers open (call after natural_open), cached:
          web_fold: the bottom of the V between thumb and index: a ray from the thumb's base joint toward the middle
            of the gap between the thumb's knuckle and the index's knuckle, where it leaves the skin;
          index_pip_pad: the index's pad skin at its middle joint: a ray from the joint along the palm's facing;
          palm_skin: the palm's skin at its middle: a ray from the wrist-to-middle-knuckle midpoint along the
            palm's facing;
          tip_len: each digit's end segment, from its joint to its farthest tip skin along the segment's line.
        Each point is kept in the frame of the bone that carries it. Returns the side's entry."""
        import datetime
        import json
        path = self._measure_path()
        data = json.load(open(path, encoding="utf-8")) if os.path.exists(path) else {}
        if side in data.get("sides", {}):
            self._measured = data
            return data["sides"][side]
        b = lambda fi, j: self._bw(self._n("finger", side, fi, j)).translation    # noqa: E731
        hn = self._n("hand", side)
        f0 = self._palm_facing(side)
        pts = {}
        c = b("thumb", "01")
        pts["web_fold"] = (hn, self._skin_exit(side, c, b("thumb", "02").lerp(b("index", "01"), 0.5) - c))
        pts["index_pip_pad"] = (self._n("finger", side, "index", "02"), self._skin_exit(side, b("index", "02"), f0, 0.04))
        mid = self._bw(hn).translation.lerp(b("middle", "01"), 0.5)
        pts["palm_skin"] = (hn, self._skin_exit(side, mid, f0, 0.06))
        entry = {"points": {}, "tip_len_m": {}}
        for k, (bone, p) in pts.items():
            if p is None:
                raise ValueError("measure_hand: no skin found for %s on the %s hand" % (k, side))
            entry["points"][k] = {"bone": bone, "local": [round(x, 7) for x in (self._bw(bone).inverted() @ p)]}
        _bone, _T, sidev, kind, segi = self._skin_static()
        P = self._skin_arrays()[0]
        sc_ = 1 if side == "r" else 2
        for fi in DIGITS:
            n3, n2 = self._n("finger", side, fi, "03"), self._n("finger", side, fi, "02")
            b3, b2 = self.arm.data.bones[n3], self.arm.data.bones[n2]
            rest = b3.head_local - b2.head_local
            Pm = self.arm.pose.bones[n3].matrix @ b3.matrix_local.inverted()
            head = self.arm.matrix_world @ (Pm @ b3.head_local)
            dirv = ((self.arm.matrix_world @ (Pm @ (b3.head_local + rest.normalized()))) - head).normalized()
            vi = np.nonzero((sidev == sc_) & (kind == 3) & (segi == DIGITS.index(fi) * 3 + 2))[0]
            if len(vi):
                along = (P[vi] - np.array(list(head))) @ np.array(list(dirv))
                entry["tip_len_m"][fi] = round(float(along.max()), 6)
        entry["method"] = {"web_fold": "skin exit of the ray thumb base joint -> middle of thumb knuckle and index knuckle",
                           "index_pip_pad": "skin exit of the ray index middle joint -> palm facing",
                           "palm_skin": "skin exit of the ray wrist/middle-knuckle midpoint -> palm facing",
                           "tip_len_m": "farthest end-segment skin along the end segment's line, from its joint",
                           "pose": "fingers open (bind pose, uncurled flat)"}
        if "measured_from" not in data:
            import hashlib
            mf, fold = {}, self.R.get("folder", "")
            for k in ("arms", "idle", "bone_edits"):
                f = self.R.get(k)
                if f:
                    p = f if os.path.isabs(f) else os.path.join(fold, f)
                    mf[k] = {"file": os.path.basename(f),
                             "sha1": hashlib.sha1(open(p, "rb").read()).hexdigest()[:16] if os.path.exists(p) else None}
            data["measured_from"] = mf                      # file names and hashes only: no local paths published
        data.setdefault("sides", {})[side] = entry
        data["date"] = datetime.date.today().isoformat()
        data["rig_key"] = self._rig_key()
        json.dump(data, open(path, "w", encoding="utf-8"), indent=1)
        self._measured = data
        return entry

    def exact_signed(self, X):
        """The exact signed distance (cm, inside below 0) from points X to the object's mesh as posed: shell by shell
        (a game gun is many closed shells, some inside others), each shell's nearest face's distance, inside it by
        ray parity in six directions (the majority); the union's is the least (sdflab.shell_signed). No grid: for a
        gap that decides a wall, and for checking the grids."""
        from sdflab import shell_signed, shells
        key = self._gun_key()
        c = getattr(self, "_shell_bvhs", None)
        if c is None or c[0] != key:
            vs, ps = self._gun_verts()
            c = (key, [BVHTree.FromPolygons(vs, [ps[i] for i in sh]) for sh in shells(ps)])
            self._shell_bvhs = c
        return np.array([shell_signed(c[1], x) * 100.0 for x in X])

    def exact_gaps(self, X, D, limit, band=0.15):
        """The gaps D (cm, from the distance grids) made exact where they decide a wall: each point whose grid gap
        is within band of its limit is measured again on the object's mesh (exact_signed). The band is above the
        grids' largest measured error, 0.112 cm on the M1911 (examples/sdf_sign_check.py, grid v3, 6000 points), so
        a point outside the band is on the right side already. Returns the gaps."""
        D = np.array(D, dtype=float)
        lim = np.broadcast_to(np.asarray(limit, dtype=float), D.shape)
        near = np.nonzero(np.abs(D - lim) <= band)[0]
        if not len(near):
            return D
        D[near] = self.exact_signed(np.asarray(X)[near])
        return D

    def measured_point(self, side, name):
        """A measured hand point in the world as the hand is posed now (its bone's frame carried), or None."""
        m = getattr(self, "_measured", None)
        if m is None:
            import json
            path = self._measure_path()
            if not os.path.exists(path):
                return None
            m = self._measured = json.load(open(path, encoding="utf-8"))
        e = ((m.get("sides") or {}).get(side) or {}).get("points", {}).get(name)
        if e is None:
            return None
        return self._bw(e["bone"]) @ Vector(e["local"])

    def measured_tip_len(self, side, fi):
        m = getattr(self, "_measured", None)
        if m is None:
            return None
        return ((m.get("sides") or {}).get(side) or {}).get("tip_len_m", {}).get(fi)

    def press_flatten(self, side, name=None):
        """The hand's skin flattened where it presses into the object, as a real hand's soft tissue compresses toward
        its bones under the grip (the owner, 2026-10-21): every skin point of this hand inside the object by no more
        than its give (the sourced skin give) moves toward its own bone (the nearest point of its heaviest bone's
        line) by exactly the distance that brings it onto the object's surface:

            p' = p + u t,   u toward the bone,   d(p) + t (n . u) = 0  ->  t = -d / (n . u)

        (n the object's outward normal at p; where n . u < 0.3 the move is along n, t = -d). A point deeper than its
        give is not moved: it is a wall, refused elsewhere. The moves are made a shape key on the skin mesh (a morph
        target in a game engine), in the mesh's rest frame: s = (sum_i w_i B_i)^-1 M^-1 dp (the skinning's own
        linear blend, M the mesh's world turn). Returns how many points moved, by how much, and how many are past
        their give (left alone)."""
        name = name or "press_%s" % side
        ob = self.meshes[0]
        P, N = self._skin_arrays()[:2]
        idx, lab = self._hand_label_arrays(side, P, N)
        if not len(idx):
            return {"moved": 0}
        X = P[idx]
        Dg, _w, Nr = self.sdf_query(X)
        cand_ = np.nonzero(Dg < 0.5)[0]                    # every point the grid puts near or inside: measured exactly
        D = np.array(Dg, dtype=float)
        if len(cand_):
            D[cand_] = self.exact_signed(X[cand_])
        grid_check = {"points_checked": int(len(cand_)),
                      "sign_disagrees": int(np.sum(np.sign(Dg[cand_]) != np.sign(D[cand_]))) if len(cand_) else 0,
                      "max_abs_difference_cm": round(float(np.max(np.abs(Dg[cand_] - D[cand_]))), 3) if len(cand_) else 0.0}
        give = np.array([SKIN_GIVE.get(_lab_key(l_), SKIN_GIVE["default"]) for l_ in lab])
        inside = D < 0.0
        within = inside & (-D <= give)
        past = inside & ~within
        bone_names, _T, _sv, _k, _s = self._skin_static()
        names = {g.index: g.name for g in ob.vertex_groups}
        MW = np.array(ob.matrix_world)[:3, :3]
        MWi = np.linalg.inv(MW)
        AW = np.array(self.arm.matrix_world)
        AWi = np.linalg.inv(AW)
        MWf = np.array(ob.matrix_world)
        MWfi = np.linalg.inv(MWf)
        if ob.data.shape_keys is None:
            ob.shape_key_add(name="Basis", from_mix=False)
        kb = ob.data.shape_keys.key_blocks
        key = kb.get(name) or ob.shape_key_add(name=name, from_mix=False)
        basis = kb[0]
        co0 = np.empty(len(basis.data) * 3)
        basis.data.foreach_get("co", co0)
        co = co0.copy().reshape(-1, 3)
        moved, depth, moved_v, info = 0, [], [], []
        for k in np.nonzero(within)[0]:
            vi = int(idx[k])
            p = X[k]
            bn = bone_names[vi]
            if not bn or bn not in self.arm.pose.bones:
                continue
            pb = self.arm.pose.bones[bn]
            h = np.array(list(self.arm.matrix_world @ pb.head))
            tl = np.array(list(self.arm.matrix_world @ pb.tail))
            seg = tl - h
            s_ = float(np.clip(((p - h) @ seg) / max(1e-12, seg @ seg), 0.0, 1.0))
            u = (h + s_ * seg) - p
            u = u / max(1e-12, np.linalg.norm(u))
            n = Nr[k] / max(1e-12, np.linalg.norm(Nr[k]))
            nu = float(n @ u)
            d_m = float(D[k]) / 100.0
            dp = u * (-d_m / nu) if nu >= 0.3 else n * (-d_m)
            # the skinning's linear blend at this vertex, in the mesh's own frame
            B4 = np.zeros((4, 4))
            wsum = 0.0
            for g in ob.data.vertices[vi].groups:
                gn = names.get(g.group)
                if gn is None or gn not in self.arm.pose.bones or g.weight <= 0.0:
                    continue
                pbg = self.arm.pose.bones[gn]
                if not pbg.bone.use_deform:
                    continue                                # the armature modifier skips a bone that does not deform
                Bi = MWfi @ AW @ np.array(pbg.matrix) @ np.linalg.inv(np.array(pbg.bone.matrix_local)) @ AWi @ MWf
                B4 += g.weight * Bi
                wsum += g.weight
            if wsum <= 0.0:
                continue
            B4 /= wsum
            B = B4[:3, :3]
            # the skinning's own check: the rest point through this blend must land where the posed skin is
            pred = MWf @ (B4 @ np.r_[co0.reshape(-1, 3)[vi], 1.0])
            skin_err = float(np.linalg.norm(pred[:3] - p)) * 100.0
            s = np.linalg.solve(B, MWi @ dp)
            co[vi] = co[vi] + s
            moved += 1
            moved_v.append(vi)
            info.append({"part": lab[k], "gap_before_cm": round(float(D[k]), 3), "n_dot_u": round(nu, 3),
                         "move_cm": round(float(np.linalg.norm(dp)) * 100.0, 3), "bone": bn,
                         "weights": len(ob.data.vertices[vi].groups), "skinning_check_cm": round(skin_err, 4)})
            depth.append(-float(D[k]))
        key.data.foreach_set("co", co.ravel())
        key.value = 1.0
        ob.data.update()
        bpy.context.view_layer.update()
        self._skin_cache = None                              # the skin's shape changed; the pose did not
        # the check: each moved point's gap on the posed, pressed skin (0 = on the surface)
        after = None
        if moved_v:
            P2 = self._skin_arrays()[0]
            D2 = self.exact_signed(P2[moved_v])
            after = {"max_abs_gap_cm": round(float(np.max(np.abs(D2))), 4), "mean_gap_cm": round(float(np.mean(D2)), 4),
                     "worst": [dict(info[i], gap_after_cm=round(float(D2[i]), 3)) for i in np.argsort(-np.abs(D2))[:4]]}
        return {"shape_key": name, "moved": moved, "on_surface_after": after, "grid_vs_exact_before": grid_check, "depth_cm": {"max": round(max(depth), 3) if depth else 0.0,
                "mean": round(float(np.mean(depth)), 3) if depth else 0.0},
                "past_give_left": int(past.sum()), "by_part": {l_: int(sum(1 for k in np.nonzero(within)[0] if lab[k] == l_))
                                                              for l_ in sorted(set(lab[k] for k in np.nonzero(within)[0]))}}

    def _thumb_world(self, side, sample=20):
        """The thumb as digitsolve.thumb_solve reads it, in the world as posed now, and its limits (the AAOS ranges
        and a holding thumb's)."""
        P, N = self._skin_arrays()[:2]
        _bone, _T, sidev, kind, segi = self._skin_static()
        sc_ = 1 if side == "r" else 2
        TL = self._thumb_limits()
        tr, _bad, axes = self._thumb_report(side, TL)
        if "02" not in axes or "03" not in axes:
            return None, None
        b = lambda j: np.array(list(self._bw(self._n("finger", side, "thumb", j)).translation))   # noqa: E731
        idx, lab = self._hand_label_arrays(side, P, N)
        labs = dict(zip(idx.tolist(), lab))
        seg, give = {}, {}
        for jj, j in enumerate(("01", "02", "03")):
            vi = np.nonzero((sidev == sc_) & (kind == 3) & (segi == jj))[0]
            vi = vi[:: max(1, len(vi) // sample)]
            seg[j] = P[vi].copy()
            give[j] = np.array([SKIN_GIVE.get(_lab_key(labs.get(int(i), "")), SKIN_GIVE["default"]) for i in vi])
        padi = np.array([int(i) for i, l_ in zip(idx, lab) if l_ == "thumb3_pad"])
        padi = padi[:: max(1, len(padi) // 12)] if len(padi) else padi
        if not len(padi):
            return None, None
        pn_ = N[padi].mean(0)
        d = {"p01": b("01"), "p02": b("02"), "p03": b("03"), "pn": np.array(list(self._palm_facing(side))),
             "index_meta": np.array(list(self._bw(self._n("finger", side, "index", "01")).translation
                                         - self._bw(self._n("hand", side)).translation)),
             "h02": np.array(list(axes["02"])), "h03": np.array(list(axes["03"])),
             "bend02": float(tr["thumb_02"][0]), "bend03": float(tr["thumb_03"][0]), "seg": seg, "seg_give": give,
             "pad": P[padi].copy(), "pad_n": pn_ / max(1e-12, np.linalg.norm(pn_)), "give_pad": SKIN_GIVE["pad"]}
        lim = {"spread_max": TL["cmc_spread_max_deg"], "palmar_min": TL["cmc_palmar_min_deg"],
               "b02": (max(TL["02"][0], HOLD_DIGIT_LIMITS["thumb_02"][0]), TL["02"][1]),
               "b03": (max(TL["03"][0], HOLD_DIGIT_LIMITS["thumb_03"][0]), TL["03"][1])}
        return d, lim

    def _apply_digits(self, side, digits):
        """The fingers' and the thumb's angles from Step A posed on the rig: each finger joint by its closing angle
        (about its finger's plane, as the closing solver turned it); the thumb's base by alpha about the palm's
        normal, then beta about the line across its metacarpal and that normal, then its knuckle and end joint about
        their hinges (digitsolve's order)."""
        FL = self._limits()[1]
        for fi, closed in (digits.get("fingers") or {}).items():
            for j, dd in closed.items():
                if abs(dd) < 0.05:
                    continue
                cur = self._finger_report(side, FL)[0]["%s_%s" % (fi, j)][0]
                self._set_curl(side, fi, j, cur + dd, FL)
        th = digits.get("thumb")
        if th:
            b = lambda j: self._bw(self._n("finger", side, "thumb", j)).translation   # noqa: E731
            n01 = self._n("finger", side, "thumb", "01")
            pn = self._palm_facing(side)
            p01 = b("01").copy()
            a_lift = (b("02") - p01).cross(pn).normalized()
            self._turn_about(n01, pn, th["alpha_deg"], p01)
            bpy.context.view_layer.update()
            self._turn_about(n01, a_lift, th["beta_deg"], p01)
            bpy.context.view_layer.update()
            for j, key in (("02", "t2_deg"), ("03", "t3_deg")):
                axes = self._thumb_report(side, self._thumb_limits())[2]
                if j in axes:
                    self._turn_about(self._n("finger", side, "thumb", j), axes[j], th[key], b(j).copy())
                    bpy.context.view_layer.update()
        bpy.context.view_layer.update()

    # --- the hand (v0.13, the owner's way, 2026-10-21): a natural open hand placed on its touch points, then the grab
    # closed toward its targets on the posed rig. Sources in docs/DIRECT_HOLD_PLAN.md -----------------------------------
    PALM_CUP_DEG = {"ring": 7.65, "pinky": 14.67}       # Buffi, Crisco & Murray 2013, J Biomech 46(12):2104-2108:
                                                        # the 4th and 5th metacarpals' flexion ranges, CT in vivo
    DIP_PIP = 0.75                                      # Hahn et al. 1995 (in the PeerJ review of finger coupling):
                                                        # the end joint bends about 0.75 of the middle joint

    def natural_open(self, side, index_extended=True):
        """The natural open hand (the owner: "a natural hand with open gesture"): each finger at the rig's own relaxed
        pose (its bind pose, the artist's relaxed hand), the end joint set to DIP_PIP of the middle joint; the thumb at
        its relaxed bind pose; an index that lies along the frame straight (the trigger finger off the trigger). Returns
        the curls set."""
        FL = self._limits()[1]
        for fi in ("index", "middle", "ring", "pinky", "thumb"):
            for j in ("01", "02", "03"):
                pb = self.arm.pose.bones[self._n("finger", side, fi, j)]
                pb.rotation_mode = "QUATERNION"
                pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        bpy.context.view_layer.update()
        out = {}
        for fi in ("index", "middle", "ring", "pinky"):
            if fi == "index" and index_extended:
                for j in ("01", "02", "03"):
                    self._set_curl(side, fi, j, 0.0, FL)
            else:
                c2 = self._finger_report(side, FL)[0]["%s_02" % fi][0]
                self._set_curl(side, fi, "03", round(self.DIP_PIP * c2, 1), FL)
            fr = self._finger_report(side, FL)[0]
            out[fi] = [fr["%s_%s" % (fi, j)][0] for j in ("01", "02", "03")]
        bpy.context.view_layer.update()
        return out

    def place_on_touch(self, side, pairs):
        """Step A, the place (exact, one calculation): the hand as it is posed now (natural, open) moved so its web
        fold sits on its point (A), turned about A so the index's pad at its middle joint sits on its line (B): the
        point of the line at the hand's own distance r from A, |X - A| = r (contactsolve.point_on_line_at), then
        turned about the A-to-B line so the palm faces into its panel as squarely as that turn allows. Nothing is
        checked here: the walls are read once the grab is closed on the posed rig. Returns the hand bone's matrix
        (world, not posed) and the readings."""
        import contactsolve as CS
        hn = self._n("hand", side)
        H0 = np.array(self._bw(hn))
        Hi = np.linalg.inv(H0)
        tf = lambda M, x: M[:3, :3] @ x + M[:3, 3]           # noqa: E731
        pw = self._pairs_world(pairs)
        V = lambda a: np.array(list(a), dtype=float)        # noqa: E731

        def unit(v):
            return v / max(1e-12, np.linalg.norm(v))

        def rot(axis, ang):
            return np.array(Matrix.Rotation(ang, 3, Vector(list(unit(axis)))))

        def turn_onto(a, b):
            a, b = unit(a), unit(b)
            c = np.cross(a, b)
            s = np.linalg.norm(c)
            if s < 1e-12:
                return np.eye(3)
            return rot(c, math.atan2(s, float(a @ b)))

        def about(M, R, c):
            K = np.eye(4)
            K[:3, :3] = R
            K[:3, 3] = c - R @ c
            return K @ M
        center = next(p for p in pw if p.get("center"))
        line_p = next((p for p in pw if p["kind"] == "line"), None)
        dir_p = next((p for p in pw if p["kind"] == "direction" and not p.get("check")), None)
        face_p = next((p for p in pw if p.get("hand") in ("palm", "palm_skin") and "normal_w" in p and p["kind"] in ("facing", "point")), None)
        loc = {p["hand"]: Hi[:3, :3] @ V(self._anchor(side, p["hand"])) + Hi[:3, 3] for p in pw if "hand" in p and p["kind"] != "band"}
        for p in pw:
            for nme in p.get("line", []):
                loc[nme] = Hi[:3, :3] @ V(self._anchor(side, nme)) + Hi[:3, 3]
        f_loc = Hi[:3, :3] @ V(self._palm_facing(side))
        T0 = V(center["T_w"])
        M = H0.copy()
        M[:3, 3] += T0 - tf(M, loc[center["hand"]])
        note = {}
        axis_t = None
        if line_p is not None:
            q = tf(M, loc[line_p["hand"]])
            X, reached, short = CS.point_on_line_at(T0, float(np.linalg.norm(q - T0)), V(line_p["T_w"]), V(line_p["dir_w"]), q)
            note["line_reached"] = bool(reached)
            if not reached:
                note["line_short_cm"] = round(short * 100.0, 2)
            M = about(M, turn_onto(q - T0, X - T0), T0)
            axis_t = unit(X - T0)
        elif dir_p is not None:
            a_, b_ = tf(M, loc[dir_p["line"][0]]), tf(M, loc[dir_p["line"][1]])
            M = about(M, turn_onto(b_ - a_, V(dir_p["dir_w"])), T0)
            axis_t = unit(V(dir_p["dir_w"]))
        if face_p is not None and axis_t is not None:
            f = M[:3, :3] @ f_loc
            want = -V(face_p["normal_w"])
            fp, wp = f - axis_t * (f @ axis_t), want - axis_t * (want @ axis_t)
            if np.linalg.norm(fp) > 1e-9 and np.linalg.norm(wp) > 1e-9:
                ang = math.atan2(float(np.cross(unit(fp), unit(wp)) @ axis_t), float(unit(fp) @ unit(wp)))
                M = about(M, rot(axis_t, ang), T0)
        Hm = Matrix([list(r_) for r_ in M])
        f = M[:3, :3] @ f_loc
        read = {"web on A (cm)": round(float(np.linalg.norm(tf(M, loc[center["hand"]]) - T0)) * 100.0, 3)}
        if line_p is not None:
            q = tf(M, loc[line_p["hand"]])
            d_ = q - V(line_p["T_w"])
            dl = unit(V(line_p["dir_w"]))
            read["index off line B (cm)"] = round(float(np.linalg.norm(d_ - dl * (d_ @ dl))) * 100.0, 3)
        if face_p is not None:
            read["palm facing off the panel (deg)"] = round(math.degrees(math.acos(max(-1.0, min(1.0, float(unit(f) @ -V(face_p["normal_w"])))))), 1)
        return Hm, {"placed": read, "note": note}

    def _grip_wrap_radius(self, mark, at):
        """The grip's size where a finger wraps it: the run's section nearest the point at (along the run's axis),
        its outline's perimeter, and the radius of a circle of that perimeter (P / 2 pi): measured on the gun."""
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        ax = RW["axis"].normalized()
        sec = min(RW["sections"], key=lambda s_: abs((s_[0] - at).dot(ax)))
        c, ring = sec[0], sec[1]
        u = (ring[0] - c)
        u = (u - ax * u.dot(ax)).normalized()
        v = ax.cross(u)
        pts = sorted(ring, key=lambda q: math.atan2((q - c).dot(v), (q - c).dot(u)))
        per = sum((pts[i] - pts[i - 1]).length for i in range(len(pts)))
        return per / (2.0 * math.pi), per

    def close_grab(self, side, pairs, mark, other=False):
        """Step A, the grab, on the posed rig (the owner: "calculate the closing grab gesture to close in the gun,
        touch point must do as a target"):
          1. the palm cups: the ring and little metacarpals flexed toward the palm until their skin touches the gun,
             at most PALM_CUP_DEG (measured ranges), by the calculated closing (contactsolve.chain_close);
          2. the wrapping fingers (middle, ring, little) set from the grip's size at their height: a finger's bone line
             wraps a circle of radius R_b = R_grip + its pad's thickness (both measured), each segment of length l
             turns the line by phi = 2 atan(l / (2 R_b)), so the knuckle bends phi_1 / 2, the middle joint
             (phi_1 + phi_2) / 2, the end joint (phi_2 + phi_3) / 2; then closed onto the surface by the calculated
             closing, every segment touching or at its range;
          3. the thumb curled onto its target (the frame's other side, opposite the index) by its four angles over
             their full measured ranges (base spread and lift +-60, the AAOS and Gehrmann-sized range; digitsolve),
             the pose nearest the relaxed thumb kept.
        Returns each step's numbers."""
        import contactsolve as CS
        import digitsolve as DS
        FL = self._limits()[1]
        out = {}
        query = lambda X: (self.exact_gaps(X, self.sdf_query(X, want_normal=False)[0], np.zeros(len(X))),   # noqa: E731
                           self.sdf_query(X)[2])
        _bone, _T, sidev, kind, segi = self._skin_static()
        sc_ = 1 if side == "r" else 2
        P = self._skin_arrays()[0]
        names = {g.index: g.name for g in self.meshes[0].vertex_groups}
        f0 = np.array(list(self._palm_facing(side)))
        # 1. the palm's cup
        cup = {}
        for fi in ("ring", "pinky"):
            mc = self._n("finger", side, fi, "metacarpal")
            if mc not in self.arm.pose.bones:
                continue
            pbm = self.arm.pose.bones[mc]
            vm = [i for i in range(len(_bone)) if _bone[i] == mc]
            if not vm:
                continue
            head = np.array(list(self.arm.matrix_world @ pbm.head))
            tail = np.array(list(self.arm.matrix_world @ pbm.tail))
            hinge = np.cross(tail - head, f0)
            hinge = hinge / max(1e-12, np.linalg.norm(hinge))
            if float(np.dot(np.cross(hinge, tail - head), f0)) < 0.0:
                hinge = -hinge
            k0 = DIGITS.index(fi) * 3
            ride = np.nonzero((sidev == sc_) & (kind == 3) & (segi >= k0) & (segi < k0 + 3))[0]
            pts = {"00": P[vm[:: max(1, len(vm) // 30)]], "01": P[ride[:: max(1, len(ride) // 30)]]}
            closed, touching, _n, log = CS.chain_close(pts, {"00": head, "01": head}, {"00": hinge},
                                                       {"00": self.PALM_CUP_DEG[fi]}, {"00": 0.0}, {"00": 1.0}, query,
                                                       touch_cm=0.08, press_cm=0.12)
            deg = float(closed.get("00", 0.0))
            if abs(deg) > 0.05:
                self._turn_about(mc, Vector(list(hinge)), deg, Vector(list(head)))
                bpy.context.view_layer.update()
            cup[fi] = {"deg": round(deg, 2), "most_deg": self.PALM_CUP_DEG[fi], "touching": sorted(touching)}
        out["palm_cup"] = cup
        # 2. the fingers wrap the grip by its measured size, then touch
        wrap = {}
        for fi in ("middle", "ring", "pinky"):
            b = lambda j: self._bw(self._n("finger", side, fi, j)).translation    # noqa: E731
            Rg, per = self._grip_wrap_radius(mark, b("01"))
            pad = self._skin_exit(side, b("02"), self._palm_facing(side), 0.04)
            t_pad = (pad - b("02")).length if pad is not None else 0.0
            Rb = Rg + t_pad
            l1, l2 = (b("02") - b("01")).length, (b("03") - b("02")).length
            l3 = self.measured_tip_len(side, fi) or (self._finger_tip(fi, side) - b("03")).length
            phi = [2.0 * math.atan(l_ / (2.0 * Rb)) for l_ in (l1, l2, l3)]
            want = [math.degrees(phi[0] / 2.0), math.degrees((phi[0] + phi[1]) / 2.0), math.degrees((phi[1] + phi[2]) / 2.0)]
            for j, w in zip(("01", "02", "03"), want):
                self._set_curl(side, fi, j, w, FL)
            bpy.context.view_layer.update()
            wrap[fi] = {"grip_perimeter_cm": round(per * 100.0, 2), "grip_radius_cm": round(Rg * 100.0, 2),
                        "pad_thickness_cm": round(t_pad * 100.0, 2), "wrap_deg": [round(w, 1) for w in want],
                        "end_over_middle": round(want[2] / want[1], 2) if want[1] > 1e-6 else None}
        res_ = {}
        self.snapshot("save", "_grab_wrap")
        kept, _w = self._walled(side, other, lambda: res_.__setitem__("v", self.conform_digits(side, ("middle", "ring", "pinky"), other=other)))
        out["fingers"] = {"wrap_from_grip_size": wrap, "touch_closing": res_.get("v"), "closing_kept_inside_walls": kept}
        # 3. the thumb curls to its target on the other side
        thumb_p = next((p for p in self._pairs_world(pairs) if p["kind"] == "band"), None)
        line_p = next((p for p in self._pairs_world(pairs) if p["kind"] == "line"), None)
        td, tlim = self._thumb_world(side)
        if thumb_p is not None and td is not None:
            band = (np.array(list(thumb_p["T_w"])), np.array(list(thumb_p["dir_w"])), float(thumb_p.get("band_cm", 1.5)))
            n_ix = np.array(list(line_p["normal_w"])) if line_p is not None else -np.array(list(thumb_p["normal_w"]))
            qy = lambda X: (np.asarray(self.sdf_query(X, want_normal=False)[0], dtype=float), self.sdf_query(X)[2])   # noqa: E731
            best, counts = DS.thumb_solve(td, qy, tlim, band, n_ix, alphas=np.arange(-60.0, 60.1, 5.0),
                                          betas=np.arange(-60.0, 60.1, 5.0))
            out["thumb"] = {"pose": best, "poses_refused": counts}
            if best is not None:
                self._apply_digits(side, {"thumb": best})
        return out

    def _place_arm(self, side, H, elbow):
        """The arm posed onto the hand matrix H with its elbow at `elbow` (the two-bone solve with that pole), the
        hand turned to H, the forearm's roll spread."""
        hn = self._n("hand", side)
        self._two_bone(side, H.translation, pole_world=Vector(list(elbow)))
        self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, H.to_quaternion(), self._unit()))
        self._roll_forearm(side)
        bpy.context.view_layer.update()

    def body_solve(self, side, H, gun_shift=None, box_cm=(5.0, 3.0, 5.0), step_cm=1.0, f_pref=20.0, verify=12):
        """Steps B and C, solved by formula (bodysolve): the hand fixed on the gun at H (Step A); for every gun
        place in the box (right, forward, up from where it is, the ready place: only with gun_shift, the firing
        hand's solve), every shoulder girdle setting and every elbow swing, each arm wall is worked out with the
        walls' own formulas; the bodies inside every wall are ranked by comfort (no wall in it). The best is posed
        and read by the real walls; when the real walls find one the formulas cannot (a forearm link, the wrist's
        twist), the next is posed, at most `verify` of them. Returns what was chosen and why, or the nearest walls
        when no body keeps them all."""
        import bodysolve as BS
        from lab import VIEW_FWD, VIEW_RIGHT, VIEW_UP
        Lm = self._limits()[0]
        hn = self._n("hand", side)
        up_, lo_ = self._n("upperarm", side), self._n("lowerarm", side)
        l1 = (self._bw(lo_).translation - self._bw(up_).translation).length
        l2 = (self._bw(hn).translation - self._bw(lo_).translation).length
        # the hand's points in its own frame (the knuckles ride the hand bone)
        Hn = self._bw(hn)
        Hi = Hn.inverted()
        kn = {k: Hi @ self._anchor(side, k) for k in ("knuckle", "index_knuckle", "pinky_knuckle")}
        ax = [np.array(list(v)) for v in (VIEW_RIGHT, VIEW_FWD, VIEW_UP)]
        steps_ = [np.arange(-b, b + 1e-9, step_cm) for b in box_cm] if gun_shift is not None else [np.zeros(1)] * 3
        hand_at = []
        for a_ in steps_[0]:
            for b_ in steps_[1]:
                for c_ in steps_[2]:
                    v = (ax[0] * a_ + ax[1] * b_ + ax[2] * c_) / 100.0
                    Hw = Matrix.Translation(Vector(list(v))) @ H
                    hand_at.append((np.array([a_, b_, c_]), np.array(list(Hw.translation)),
                                    np.array(list(Hw @ kn["knuckle"])), np.array(list(Hw @ kn["index_knuckle"])),
                                    np.array(list(Hw @ kn["pinky_knuckle"]))))
        # the girdle's settings: where each puts the shoulder (read off the rig once each) and its share of its range
        self.snapshot("save", "_body0")
        girdles = []
        for pr in (-10.0, -5.0, 0.0, 5.0, 10.0, 15.0):
            for el in (-6.0, 0.0, 6.0, 12.0, 18.0):
                got = self.shoulder(side, pr, el)
                share = max(abs(got["protraction_deg"]) / (CLAVICLE["protraction_deg"] if pr >= 0 else CLAVICLE["retraction_deg"]),
                            abs(got["elevation_deg"]) / (CLAVICLE["elevation_deg"] if el >= 0 else CLAVICLE["depression_deg"]))
                girdles.append(((got["protraction_deg"], got["elevation_deg"]), np.array(list(self._bw(up_).translation)), share))
        self.snapshot("load", "_body0")
        cands, n_all = BS.solve_body(hand_at, girdles, l1, l2, side, Lm, ax[0], ax[1], f_pref, up=ax[2])
        out = {"bodies_worked_out": n_all, "inside_every_wall": len(cands), "gun_places": len(hand_at),
               "girdle_settings": len(girdles)}
        if not cands:
            out["result"] = "no body keeps every wall"
            out["nearest"] = BS.nearest_walls(hand_at, girdles, l1, l2, side, Lm, ax[0], ax[1], up=ax[2])
            return out
        tried = []
        for c in cands[:verify]:
            self.snapshot("load", "_body0")
            v = Vector(list((ax[0] * c["shift_cm"][0] + ax[1] * c["shift_cm"][1] + ax[2] * c["shift_cm"][2]) / 100.0))
            if gun_shift is not None:
                gun_shift(v)
            self.shoulder(side, *c["girdle"])
            if not hasattr(self, "_clav_now"):
                self._clav_now = {}
            self._clav_now[side] = c["girdle"]
            self._place_arm(side, Matrix.Translation(v) @ H, c["elbow"])
            rep_ = self._arm_report(side, Lm)[0]
            real = {k: v_ for k, v_ in self.wall_faults(side, angles_only=True).items() if k.startswith("arm:")}
            row = {"shift_cm": c["shift_cm"], "girdle": c["girdle"], "phi_deg": c["phi_deg"], "cost": round(c["cost"], 3),
                   "formula": c["values"], "posed": {k: rep_.get(k) for k in ("elbow_bend_deg", "wrist_bend_deg",
                                                                              "wrist_flexion_deg", "wrist_radial_deg",
                                                                              "forearm_rotation_deg", "elbow_under_shoulder_cm")},
                   "real_walls": real}
            tried.append(row)
            if not real:
                out.update({"result": "chosen", "chosen": row, "tried": len(tried)})
                return out
            if gun_shift is not None:
                gun_shift(-v)
        self.snapshot("load", "_body0")
        out.update({"result": "every body the formulas kept broke a wall the formulas cannot read", "tried_rows": tried})
        return out

    def start_clip_diagnosis(self, side, top=12):
        """Why the hand's body clips at the start when Step A said it would not. Step A moved the hand as one rigid
        body: each body skin point kept its place in the hand bone's frame (the rigid model). The posed skin is
        bent by every bone it is weighed to. For each body skin point past its give now: where the rigid model puts
        it and its distance to the gun there, where the posed skin really is and its distance, how far the two are
        apart, and the bones that move it (weights). Also whether the posed hand bone landed where Step A put it."""
        fr = getattr(self, "_fit_rigid", {}).get(side)
        if fr is None or len(fr) < 8:
            return {"note": "no Step A on this hand"}
        body_i, body_l, Pb_local, H_fit, G_fit, parts_fit, D_fit, W_fit = fr
        hn = self._n("hand", side)
        Hn = self._bw(hn)
        Hm = np.array(Hn)
        P = self._skin_arrays()[0]
        actual = P[body_i]
        rigid = Pb_local @ Hm[:3, :3].T + Hm[:3, 3]
        Da = self.sdf_query(actual, want_normal=False)[0]
        Dr, Wr, _n = self.sdf_query(rigid, want_normal=False)
        give = np.array([SKIN_GIVE.get(_lab_key(l_), SKIN_GIVE["default"]) for l_ in body_l])
        past = -Da - give
        ob = self.meshes[0]
        names = {g.index: g.name for g in ob.vertex_groups}
        rows = []
        for k in np.argsort(-past)[:top]:
            if past[k] <= 0.0:
                break
            v = ob.data.vertices[body_i[k]]
            ws = sorted(((names.get(g.group, "?"), round(g.weight, 2)) for g in v.groups if g.weight > 0.02), key=lambda x: -x[1])
            rows.append({"vertex": int(body_i[k]), "part": body_l[k], "past_give_cm": round(float(past[k]), 3),
                         "posed_gap_cm": round(float(Da[k]), 3), "rigid_gap_cm": round(float(Dr[k]), 3),
                         "gap_at_fit_cm": round(float(D_fit[k]), 3), "gun_part_now": str(Wr[k]), "gun_part_at_fit": str(W_fit[k]),
                         "posed_vs_rigid_cm": round(float(np.linalg.norm(actual[k] - rigid[k])) * 100.0, 3),
                         "bones": ws[:4]})
        moved = np.linalg.norm(actual - rigid, axis=1) * 100.0
        # the hand against the gun: where Step A put it in the gun's frame, and where it is in the gun's frame now
        G_now = self._obj_frame("Root")
        rel_fit, rel_now = G_fit.inverted() @ H_fit, G_now.inverted() @ Hn
        gb = self.names["gun"]
        chain, pb_ = [], self.arm.pose.bones[gb].parent
        while pb_ is not None:
            chain.append(pb_.name)
            pb_ = pb_.parent
        parts_moved = {}
        for p_, R0 in parts_fit.items():
            R1 = G_now.inverted() @ self._obj_frame(p_)
            dm = (R1.translation - R0.translation).length * 100.0
            if dm > 0.01:
                parts_moved[p_] = round(dm, 3)
        return {"gun_parts_moved_on_the_gun_cm": parts_moved,
                "hand_on_gun_moved_cm": round((rel_now.translation - rel_fit.translation).length * 100.0, 3),
                "hand_on_gun_turned_deg": round(math.degrees(rel_now.to_quaternion().rotation_difference(rel_fit.to_quaternion()).angle), 2),
                "gun_moved_since_fit_cm": round((G_now.translation - G_fit.translation).length * 100.0, 3),
                "gun_bone": gb, "gun_bone_parents": chain,
                "body_points": len(body_i), "past_give_now": int((past > 0.0).sum()),
                "past_give_in_the_rigid_model": int(((-Dr - give) > 0.0).sum()),
                "posed_vs_rigid_cm": {"mean": round(float(moved.mean()), 3), "max": round(float(moved.max()), 3)},
                "worst": rows}

    def hand_place(self, side, H, mark):
        """Where Step A put the hand, read on the gun (H: the hand bone's matrix from Step A, not posed): for the
        web, the palm's middle, the index's middle joint, the thumb's base, the knuckles and the wrist, in the rigid
        model at H: its height against the grip's top (along the grip's axis; below 0 under the top), its gap to the
        gun's surface (below 0 inside), the gun region nearest it, and the web point's own height for comparison."""
        fr = getattr(self, "_fit_rigid", {}).get(side)
        hn = self._n("hand", side)
        Hi = self._bw(hn).inverted()
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        top, ax = RW["inner"], (RW["inner"] - RW["outer"]).normalized()
        SW, SL = self._object_now()[2:4]
        kd = self._object_now()[4]
        out = {}
        names = (("web fold (skin)", "web_fold"), ("palm middle (skin)", "palm_skin"), ("index middle joint (pad skin)", "index_pip_pad"),
                 ("thumb base", "thumb_cmc"), ("middle knuckle", "knuckle"), ("wrist", "wrist"))
        for label, nm in names:
            p = H @ (Hi @ self._anchor(side, nm))
            D, _w, _n = self.sdf_query(np.array([list(p)]))
            loc = p - Vector(list(_n[0])) * (float(D[0]) / 100.0)
            _c, si, _d = kd.find(loc)
            out[label] = {"height_from_grip_top_cm": round((p - top).dot(ax) * 100.0, 2),
                          "gap_to_gun_cm": round(float(D[0]), 2),
                          "nearest_gun_region": SL[si] if si is not None else None}
        if mark.get("_web"):
            wt = self._obj_frame(mark["part"]) @ Vector(mark["_web"])
            out["the web point (gun)"] = {"height_from_grip_top_cm": round((wt - top).dot(ax) * 100.0, 2)}
        out["grip length (top to bottom) cm"] = round((RW["inner"] - RW["outer"]).length * 100.0, 2)
        if fr is not None:
            out["body skin points"] = len(fr[0])
        return out

    def direct_hold(self, side, marks, pairs, glove_cm=0.0, other=True, wrap=None, gun_shift=None, f_pref=20.0,
                    on_posed=None):
        """The hold, in the owner's order (docs/DIRECT_HOLD_PLAN.md, v0.13): the natural open hand (natural_open), its
        touch points measured on the skin (measure_hand), placed exactly (place_on_touch: web on A, index on line B, palm
        facing the panel); the body to it by formula (body_solve); then on the posed rig the grab closed toward its
        targets (close_grab: palm cup, fingers wrapping by the grip's size, the thumb curled to the other side); the
        skin within its give flattened (press_flatten); every wall read on the posed rig, the thumb and the fingers
        included. A broken wall stops the hold and is reported; nothing is moved to make it pass. on_posed(side):
        called on the posed hold (pictures)."""
        import time as _time
        t0 = _time.time()
        m = next(x for x in marks if x.get("role") == "hold" and x.get("hand", side) == side)
        out = {"method": "natural hand, placed, grab closed (v0.13)", "steps": {}, "trace": []}

        def stop(why, walls):
            out.update({"result": why, "walls": walls, "gate": False, "solve_s": round(_time.time() - t0, 1)})
            return out
        out["steps"]["A_natural_open"] = self.natural_open(side, index_extended=m.get("index") == "extended")
        out["steps"]["A_measured"] = self.measure_hand(side)
        H, place = self.place_on_touch(side, pairs)
        out["steps"]["A_place"] = place
        out["steps"]["A_place_on_gun"] = self.hand_place(side, H, m)
        body = self.body_solve(side, H, gun_shift=gun_shift, f_pref=f_pref)
        out["steps"]["BC_body"] = body
        if body.get("result") != "chosen":
            return stop("no body reaches the hand inside the walls", body.get("nearest") or body.get("tried_rows"))
        out["steps"]["A_grab"] = self.close_grab(side, pairs, m, other=other)
        out["steps"]["press_shape"] = self.press_flatten(side)
        out["trace"].append({"step": "hold posed", "touch": self.touch_report(side, pairs)})
        ok, swalls, margin = self.start_check(side, other)
        walls = dict(self.wall_faults(side, other))
        walls.update(swalls)
        out["start"] = {"valid": ok, "walls": swalls, "reach_margin_cm": margin,
                        "body_clip_by": dict(getattr(self, "_start_by", {}))}
        if on_posed is not None:
            out["pictures"] = on_posed(side)
        sc, why = self._hold_score(side, marks, glove_cm, other, None, self._tree())
        g = self.hold_gate(marks, sides=(side,), glove_cm=glove_cm, skin=False, other=other)
        out.update({"score": round(sc, 2), "gate": bool(g.get(side, {}).get("pass")) and not walls,
                    "walls": {k: round(float(v), 2) for k, v in walls.items()}, **why})
        if walls:
            out["result"] = "the posed hold breaks a wall: it stops here"
        out["unmet"] = [{"purpose": r["purpose"], "why": {k: v for k, v in r.items() if k not in ("purpose", "met", "error", "hard", "why")}}
                        for r in why["purposes"] if not r["met"]]
        out["solve_s"] = round(_time.time() - t0, 1)
        return out

    @deprecated()
    def target_hold(self, side, marks, glove_cm=0.0, other=True, rounds=4, wrap=None, polish=True):
        """A hand's hold solved toward its purposes' targets, with no chance in it: each round the hand is fitted
        onto its targets (palm patch and facing, web, knuckles' line), the palm settled onto the surface, the body
        eased when the arm is past a limit (elbow, then shoulder), the wrapping fingers closed by the skin, the
        extended index and the thumb turned onto their lines by a fixed step pattern; the best round is kept, the
        rounds stop when the hold passes its gate or stops getting better. Returns the score's parts, the rounds
        and what is still unmet, each with its reason."""
        import time as _time
        t0 = _time.time()
        m = next(x for x in marks if x.get("role") == "hold" and x.get("hand", side) == side)
        FL = self._limits()[1]
        wrap = wrap or {fi: "wrap" for fi in ("index", "middle", "ring", "pinky")}
        closing = tuple(fi for fi, how in wrap.items() if how == "wrap" and fi != "thumb")
        best, log = None, []
        self.snapshot("save", "_th_best")
        for rd in range(rounds):
            dl = getattr(self, "_deadline", None)
            if dl is not None and rd > 0 and _time.time() > dl:
                log.append({"round": rd, "stopped": "time budget"})
                self._cnt("rounds stopped at the time budget")
                break
            self._cnt("rounds")
            self.snapshot("save", "_th_round")
            w_round = self.wall_faults(side, other)
            pairs = self._place_targets(side, m)
            if pairs:
                # the hand goes onto its targets as far as the arm takes it: the whole move, else three quarters,
                # a half, a quarter (an arm no further past its limits than it was); else it stays
                # the fingers open first (a hand moving onto a hold comes open), inside the walls
                self._walled(side, other, lambda: [self._set_curl(side, fi, j, 0.0, FL) for fi in closing for j in ("01", "02", "03")])
                w_fit = self.wall_faults(side, other)
                self.snapshot("save", "_th_fit")
                took = 0.0
                for share in (1.0, 0.75, 0.5, 0.25):
                    self._cnt("fit shares tried")
                    self.snapshot("load", "_th_fit")
                    self._fit_hand(side, pairs, share)
                    self.settle_palm(side)
                    if not self._walls_hold(self.wall_faults(side, other), w_fit):
                        # the body helps first (the elbow, then the girdle), the hand kept on its targets
                        self._body_eases(side, self._bw(self._n("hand", side)).copy())
                    if self._walls_hold(self.wall_faults(side, other), w_fit):
                        took = share
                        break
                if os.environ.get("POSELAB_DEBUG"):
                    print("DBG fit", side, rd, "share", took)
                if took == 0.0:
                    self.snapshot("load", "_th_fit")
                    log.append({"round": rd, "fit": "refused: every share broke a wall"})
                elif took < 1.0:
                    log.append({"round": rd, "fit": "share %.2f, as far as the arm takes it" % took})
            W = lambda f: self._walled(side, other, f)           # noqa: E731  every step inside the walls
            W(lambda: self.settle_palm(side))
            keep = self._bw(self._n("hand", side)).copy()
            W(lambda: self._body_eases(side, keep))
            for fi, how in wrap.items():
                if how == "straight" and fi != "thumb" and rd == 0:
                    W(lambda fi=fi: [self._set_curl(side, fi, j, cc, FL) for j, cc in zip(("01", "02", "03"), (0.0, 0.0, 3.0))])
            W(lambda: self.conform_digits(side, closing, other=other))
            self._fit_pads(side, closing, other)
            if m.get("index") == "extended" and wrap.get("index") == "straight":
                self._fit_index(side, m, glove_cm, other)
            if m.get("thumb") == "inline":
                o_ = "r" if side == "l" else "l"
                self._fit_thumb(side, m, glove_cm, along=self._thumb_dir(o_) if m["purpose"] == "support" else None, other=other)
            elif m.get("thumb") in ("wrap", "opposed"):
                W(lambda: self._set_thumb_bend(side, 6.0, 4.0, self._thumb_limits()))
                W(lambda: self.conform_digits(side, ("thumb",), other=other))
                self._fit_thumb(side, m, glove_cm, inline=False, other=other)
            W(lambda: self.relax_fingers(side, tuple(fi for fi in ("index", "middle", "ring", "pinky", "thumb"))))
            w_end = self.wall_faults(side, other)
            if not self._walls_hold(w_end, w_round):
                # the round as a whole may not leave the hand past a wall it started inside (nor further past one)
                self.snapshot("load", "_th_round")
                log.append({"round": rd, "refused": "the round broke walls: %s" % sorted(set(w_end) - set(w_round))})
                self._cnt("rounds undone at a wall")
                w_end = w_round
            sc, why = self._hold_score(side, marks, glove_cm, other, None, self._tree())
            g = self.hold_gate(marks, sides=(side,), glove_cm=glove_cm, skin=False, other=other)
            ok = bool(g.get(side, {}).get("pass")) and not w_end
            prev = [x["score"] for x in log if "score" in x]
            log.append({"round": rd, "score": round(sc, 2), "gate": ok, "walls": len(w_end)})
            # holds compared in a fixed order: the walls first (fewer broken, less past), then the gate, then the score
            rank = (self._walls_rank(w_end), not ok, sc)
            if best is None or rank < best[3]:
                best = (sc, why, ok, rank, w_end)
                self.snapshot("save", "_th_best")
            if ok or (prev and sc >= prev[-1] - 0.5):
                break
        self.snapshot("load", "_th_best")
        dl = getattr(self, "_deadline", None)
        if polish and not best[2] and (dl is None or _time.time() < dl):
            self._polish(side, marks, glove_cm, other, tuple(fi for fi in ("index", "middle", "ring", "pinky")
                                                              if wrap.get(fi) == "wrap"))
            w_p = self.wall_faults(side, other)
            sc, why = self._hold_score(side, marks, glove_cm, other, None, self._tree())
            g = self.hold_gate(marks, sides=(side,), glove_cm=glove_cm, skin=False, other=other)
            ok = bool(g.get(side, {}).get("pass")) and not w_p
            log.append({"round": "polish", "score": round(sc, 2), "gate": ok, "walls": len(w_p)})
            rank = (self._walls_rank(w_p), not ok, sc)
            if rank < best[3]:
                best = (sc, why, ok, rank, w_p)
                self.snapshot("save", "_th_best")
            self.snapshot("load", "_th_best")
        sc, why, ok, _rank, walls = best
        unmet = [{"purpose": r["purpose"], "why": {k: v for k, v in r.items() if k not in ("purpose", "met", "error", "hard", "why")}}
                 for r in why["purposes"] if not r["met"]]
        out = {"score": round(sc, 2), "gate": ok, "rounds": log, "unmet": unmet, "solve_s": round(_time.time() - t0, 1),
               "walls": {k: round(v, 2) for k, v in walls.items()}, **why}
        if walls:
            # no hold inside the walls from this start: said plainly, never handed out as a hold
            out["result"] = "no hold inside the walls from this start"
            out["gate"] = False
        return out

    # --- one frame mended on the contact ruler ----------------------------------------------------------------
    def frame_fault(self, side, glove_cm=0.0, other=True):
        """What a moving frame's hand does wrong, on the drawn skin: its press into the object and into the other
        hand past the skin's give (cm2, deepest cm), and how far past its limits the arm is (degrees)."""
        cp = self.contact_pairs(side, other=other)
        return {"press_cm2": cp["press_cm2"], "deepest_cm": cp["deepest_cm"], "hand_press_cm2": cp["hand_press_cm2"],
                "hand_deepest_cm": cp["hand_deepest_cm"], "arm_excess_deg": self.arm_excess(side)}

    def repair_frame(self, side, glove_cm=0.0, other=True, keep_w=6.0, generations=None, pop=None, seed=None, **_old):
        """A moving frame's hand mended back inside the walls, with no chance in it: only when it breaks one (the
        arm's rules, the hand's ranges, clipping either way, a hand in the other); the hand moved (0.4 to 0.1 cm)
        and turned (2 to 0.5 degrees) about its palm and the elbow swung, by a fixed step pattern; a step that
        breaks a wall or goes further past one is undone, a step that leaves the walls less broken kept first, else
        the one nearest where the motion put the hand. Returns the faults before and after and the walls left, or
        None when the frame needed nothing."""
        w0 = self.wall_faults(side, other)
        if not w0:
            return None
        import time as _t
        old_dl = getattr(self, "_deadline", None)
        self._deadline = min(old_dl if old_dl is not None else float("inf"), _t.time() + self.REPAIR_FRAME_S)
        f0 = self.frame_fault(side, glove_cm, other)
        hn = self._n("hand", side)
        H0 = self._bw(hn).copy()
        P0 = H0.translation.lerp(self._bw(self._n("finger", side, "middle", "01")).translation, 0.5)

        def err():
            return (self._bw(hn).translation - H0.translation).length * 100.0

        def move(ax):
            def mv(st):
                M = Matrix.Translation(Vector(ax) * (st * 0.001)) @ self._bw(hn)
                self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
            return mv

        def turn(ax):
            def mv(st):
                R = Matrix.Translation(P0) @ Matrix.Rotation(math.radians(st * 0.5), 4, Vector(ax)) @ Matrix.Translation(-P0)
                M = R @ self._bw(hn)
                self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
            return mv

        def swing(st):
            self._swing_elbow(side, st * 2.0)
        AX = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        self._descend(err, [move(a) for a in AX] + [turn(a) for a in AX] + [swing], side, other, steps=(4.0, 2.0, 1.0))
        self._deadline = old_dl
        f1 = self.frame_fault(side, glove_cm, other)
        w1 = self.wall_faults(side, other)
        return {"before": f0, "after": f1, "moved_cm": round(err(), 2), "walls": {k: round(v, 2) for k, v in w1.items()}}

    # --- the gate --------------------------------------------------------------------------------------------
    def hold_gate(self, marks, sides=("r", "l"), glove_cm=0.0, skin=True, other=True):
        """Does each hand do its job: every purpose its marks give met, no no-touch contact, every joint in its
        comfort band, no body rule broken, and (skin) the drawn skin out of the object and the other hand. A hand
        the marks give a hold that is off its part fails. Returns per hand its results and pass, and pass."""
        out, ok = {}, True
        for side in sides:
            if not any(m.get("role") == "hold" and m.get("hand", side) == side for m in marks):
                continue
            res = self.hold_purposes(side, marks, glove_cm, other=other)
            FL = self._limits()[1]
            TL = self._thumb_limits()
            body = self._finger_report(side, FL)[1] + self._thumb_report(side, TL)[1]
            pr = next((r for r in res if r["purpose"] == "press"), {})
            rec = {"purposes": res, "body_rules": body, "press": round(pr.get("object_cm2", 0.0) + pr.get("hand_cm2", 0.0), 3)}
            rec["pass"] = all(r["met"] for r in res) and not body
            out[side] = rec
            ok = ok and rec["pass"]
        out["pass"] = ok                               # the presses are the drawn skin's already (the contact model)
        return out
