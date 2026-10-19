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

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

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
        FL = self._limits()[1]
        TL = self._thumb_limits()
        fbad = len(self._finger_report(side, FL)[1]) + len(self._thumb_report(side, TL)[1])
        abad = len(self._arm_report(side, self._limits()[0])[1])
        sc = 10.0 * perr + 200.0 * (fbad + abad)
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

    def hold_object(self, side, marks, glove_cm=0.0, other=True, soft=None, generations=40, pop=16, seed=5,
                    sigma_cm=1.0, sigma_deg=8.0, close=None, wrap=None, settle=True, conform=True):
        """A hand's hold searched over the whole arm, from where it is now: the hand's place (3) and turn (3), the
        elbow's swing (1), the shoulder girdle (2: forward, up), each finger's bend (4) and the thumb's base turn
        (3) and two bends (2), by an evolution strategy; each try scored by its purposes (hold_purposes), its
        presses into the object and the other hand, and the body's rules (each broken all but forbidden). close:
        {finger: [3 bends]} the fingers start from (else as they are). wrap: {finger: "wrap" | "straight"}: each
        try closes those fingers until their skin meets the object or the other hand (as real fingers close on a
        handle), the straight ones laid straight; their bends are then not searched. settle: each try first slides
        the hand until its palm's skin meets the object (palm first, then the fingers close), and a try whose palm
        cannot reach it is thrown out. Returns the score's parts and the time a try takes."""
        import time as _time
        import random as _random
        self._need()
        rnd = _random.Random(seed)
        FL = self._limits()[1]
        TL = self._thumb_limits()
        tree = self._tree()
        hn = self._n("hand", side)
        if close:
            for fi, c3 in close.items():
                for j, cc in zip(("01", "02", "03"), c3):
                    self._set_curl(side, fi, j, float(cc), FL)
            bpy.context.view_layer.update()
        H0 = self._bw(hn).copy()
        P0 = H0.translation.lerp(self._bw(self._n("finger", side, "middle", "01")).translation, 0.5)
        n1 = self._n("finger", side, "thumb", "01")
        fins = ("index", "middle", "ring", "pinky")
        fr_ = self._finger_report(side, FL)[0]
        base = {fi: [fr_["%s_%s" % (fi, j)][0] for j in ("01", "02", "03")] for fi in fins}
        tr_ = self._thumb_report(side, TL)[0]
        tb = [tr_.get("thumb_02", [0.0])[0], tr_.get("thumb_03", [0.0])[0]]
        clav0 = getattr(self, "_clav_now", {}).get(side, (0.0, 0.0))
        self.snapshot("save", "_ho")
        dim = 18
        scale = [sigma_cm] * 3 + [sigma_deg] * 3 + [10.0] + [4.0, 4.0] + [8.0] * 4 + [sigma_deg * 2] * 3 + [10.0, 10.0]
        plan = None
        if wrap:
            plan = {}
            for fi, how in wrap.items():
                if fi == "thumb":
                    plan[fi] = {"pose": "wrap", "max_curl": [0.0, 55.0, 80.0]}
                else:
                    plan[fi] = {"pose": "wrap", "max_curl": [90.0, 100.0, 80.0]} if how == "wrap" else {"pose": "straight", "curl": [0.0, 0.0, 3.0]}
            for k_, fi in enumerate(fins):
                if fi in wrap and wrap[fi] == "wrap":
                    scale[9 + k_] = 0.0          # closed by contact; a straight (extended) finger keeps its bend searched
            if "thumb" in wrap:
                scale[16:18] = [0.0, 0.0]          # the thumb's bends come from its closing; its base is searched

        prof = {}

        def lap(name, t0):
            prof[name] = prof.get(name, 0.0) + (_time.time() - t0)
            return _time.time()

        def apply(x):
            t_ = _time.time()
            self.snapshot("load", "_ho")
            pr, el = clav0[0] + x[7], clav0[1] + x[8]
            sh = self.shoulder(side, pr, el)
            if not hasattr(self, "_clav_now"):
                self._clav_now = {}
            self._clav_now[side] = (sh["protraction_deg"], sh["elevation_deg"])
            t = Vector(x[0:3]) / 100.0
            rv = Vector(x[3:6])
            q = Quaternion(rv.normalized(), math.radians(rv.length)) if rv.length > 1e-6 else Quaternion()
            R = Matrix.Translation(t) @ Matrix.Translation(P0) @ q.to_matrix().to_4x4() @ Matrix.Translation(-P0) @ H0
            short = self._put_hand(side, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))
            bpy.context.view_layer.update()
            if abs(x[6]) > 0.5:
                self._swing_elbow(side, max(-40.0, min(40.0, x[6])))
                bpy.context.view_layer.update()
            t_ = lap("place", t_)
            settled = (None, 0.0)
            if settle:
                settled = self.settle_palm(side)
            self._settled = settled
            t_ = lap("settle", t_)
            for k_, fi in enumerate(fins):
                if plan and fi in plan and plan[fi].get("pose") != "straight":
                    continue
                d = x[9 + k_]
                b3 = plan[fi]["curl"] if plan and fi in plan else base[fi]
                c3 = [b3[0] + d, b3[1] + d * 0.5, b3[2] + d * 0.3]
                for j, cc in zip(("01", "02", "03"), c3):
                    self._set_curl(side, fi, j, cc, FL)
            tv = Vector(x[13:16])
            if tv.length > 1e-6:
                head = self._bw(n1).translation.copy()
                self._turn_about(n1, tv.normalized(), tv.length, head)
            bpy.context.view_layer.update()
            if not (plan and "thumb" in plan):
                for j, b in (("02", tb[0] + x[16]), ("03", tb[1] + x[17])):
                    cur = self._thumb_report(side, TL)[0].get("thumb_" + j, [0, 0])[0]
                    if self.R.get("thumb_hinge") == "ray":
                        self._thumb_flex(side, j, b - cur)
                    else:
                        self._set_thumb_bend(side, tb[0] + x[16], tb[1] + x[17], TL)
                        break
            bpy.context.view_layer.update()
            if plan:
                # the fingers (and a wrapping thumb, its base turned) opened, then closed until they meet the
                # object or the other hand, then opened by the skin until they press nothing
                for fi in plan:
                    if fi == "thumb" or plan[fi].get("pose") == "straight":
                        continue
                    for j in ("01", "02", "03"):
                        self._set_curl(side, fi, j, 0.0, FL)
                bpy.context.view_layer.update()
                t_ = lap("thumb", t_)
                if conform:
                    # every joint closes, a touching segment stops the joints behind it (the skin, in numpy)
                    straight = [fi for fi, p_ in plan.items() if p_.get("pose") == "straight"]
                    if "thumb" in plan:
                        self._set_thumb_bend(side, 6.0, 4.0, TL)
                    bpy.context.view_layer.update()
                    self.conform_digits(side, tuple(fi for fi in plan if fi not in straight), other=other)
                    t_ = lap("close", t_)
                else:
                    self.close_hand(side, plan, tree, glove_cm, other, mend=False)
                    t_ = lap("close", t_)
                    self.relax_fingers(side, tuple(plan))
                    t_ = lap("relax", t_)
            return short

        def score(x):
            t0 = _time.time()
            short = apply(x)
            t1 = _time.time()
            sc, why = self._hold_score(side, marks, glove_cm, other, soft, tree)
            lap("score", t1)
            gap, _touch = getattr(self, "_settled", (None, 0.0))
            if settle and (gap is None or gap > 1.5):
                sc += 500.0 + 10.0 * (gap or 10.0)        # the palm could not reach the object: no hold
            why["palm_gap_cm"] = gap
            times.append(_time.time() - t0)
            return sc + 4.0 * short, why

        times = []

        mean = [0.0] * dim
        s0 = score(mean)
        best = (s0[0], list(mean), s0[1])
        mu = max(2, pop // 4)
        sig = 1.0
        for _g in range(generations):
            kids = []
            for _k in range(pop):
                x = [mean[i] + sig * scale[i] * rnd.gauss(0, 1) for i in range(dim)]
                sc, why = score(x)
                kids.append((sc, x, why))
            kids.sort(key=lambda k: k[0])
            if kids[0][0] < best[0]:
                best = kids[0]
                sig = min(2.0, sig * 1.15)
            else:
                sig = max(0.1, sig * 0.85)
            mean = [sum(k[1][i] for k in kids[:mu]) / mu for i in range(dim)]
        sc, why = score(best[1])
        return {"score": round(sc, 2), "start_score": round(s0[0], 2), "x": [round(v, 2) for v in best[1]],
                "shoulder_deg": list(getattr(self, "_clav_now", {}).get(side, (0.0, 0.0))),
                "evaluations": generations * pop + 2, "try_s": round(sum(times) / max(1, len(times)), 3),
                "try_parts_s": {k: round(v / max(1, len(times)), 3) for k, v in prof.items()}, **why}

    # --- one frame mended on the contact ruler ----------------------------------------------------------------
    def frame_fault(self, side, glove_cm=0.0, other=True):
        """What a moving frame's hand does wrong, on the drawn skin: its press into the object and into the other
        hand past the skin's give (cm2, deepest cm), and how far past its limits the arm is (degrees)."""
        cp = self.contact_pairs(side, other=other)
        return {"press_cm2": cp["press_cm2"], "deepest_cm": cp["deepest_cm"], "hand_press_cm2": cp["hand_press_cm2"],
                "hand_deepest_cm": cp["hand_deepest_cm"], "arm_excess_deg": self.arm_excess(side)}

    def repair_frame(self, side, glove_cm=0.0, other=True, keep_w=6.0, generations=6, pop=8, seed=3, sigma_cm=1.0,
                     sigma_deg=6.0, press_cm2=0.3):
        """A moving frame's hand mended by the contact ruler: only when its skin presses past the give (more than
        press_cm2) or its arm is past a limit; the hand moved and turned and the elbow swung (the digits as they
        are), each try scored by the skin's presses (25 a cm2, 100 a cm deep), the arm's excess (20 a degree) and
        keep_w a cm away from where the motion put the hand. Returns the fault before and after, or None when the
        frame needed nothing."""
        import random as _random
        f0 = self.frame_fault(side, glove_cm, other)
        if f0["press_cm2"] + f0["hand_press_cm2"] <= press_cm2 and f0["arm_excess_deg"] <= 0.0:
            return None
        rnd = _random.Random(seed)
        hn = self._n("hand", side)
        H0 = self._bw(hn).copy()
        P0 = H0.translation.lerp(self._bw(self._n("finger", side, "middle", "01")).translation, 0.5)
        self.snapshot("save", "_rf")

        def apply(x):
            self.snapshot("load", "_rf")
            rv = Vector(x[3:6])
            q = Quaternion(rv.normalized(), math.radians(rv.length)) if rv.length > 1e-6 else Quaternion()
            R = Matrix.Translation(Vector(x[0:3]) / 100.0) @ Matrix.Translation(P0) @ q.to_matrix().to_4x4() @ Matrix.Translation(-P0) @ H0
            self._put_hand(side, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))
            bpy.context.view_layer.update()
            if abs(x[6]) > 0.5:
                self._swing_elbow(side, max(-40.0, min(40.0, x[6])))
                bpy.context.view_layer.update()

        def score(x):
            apply(x)
            f = self.frame_fault(side, glove_cm, other)
            sc = (25.0 * (f["press_cm2"] + f["hand_press_cm2"]) + 100.0 * (f["deepest_cm"] + f["hand_deepest_cm"])
                  + 20.0 * f["arm_excess_deg"] + keep_w * Vector(x[0:3]).length)
            return sc, f

        scale = [sigma_cm] * 3 + [sigma_deg] * 3 + [10.0]
        mean = [0.0] * 7
        s0 = score(mean)
        best = (s0[0], list(mean), s0[1])
        sig = 1.0
        for _g in range(generations):
            kids = sorted((score(x) + (x,) for x in ([mean[i] + sig * scale[i] * rnd.gauss(0, 1) for i in range(7)] for _k in range(pop))),
                          key=lambda k: k[0])
            if kids[0][0] < best[0]:
                best = (kids[0][0], kids[0][2], kids[0][1])
                sig = min(2.0, sig * 1.15)
            else:
                sig = max(0.1, sig * 0.85)
            mu = max(2, pop // 4)
            mean = [sum(k[2][i] for k in kids[:mu]) / mu for i in range(7)]
        _sc, f1 = score(best[1])
        return {"before": f0, "after": f1, "moved_cm": round(Vector(best[1][0:3]).length, 2)}

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
