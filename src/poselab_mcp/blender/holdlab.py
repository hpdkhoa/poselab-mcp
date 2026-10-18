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

    def _part_openings(self, name, P, cell_cm=0.3):
        """Holes through the object beside a part (a trigger guard, a handle's eye), found once over the whole part:
        a grid of rays across its thinnest direction, against all the object's parts; a cell no ray hits but walled by hits on both sides in both grid directions
        is open; touching open cells are one opening. Returns each opening's cells (object frame, m), middle, area
        and its through direction."""
        # rays against the whole object (another part may close the hole: a slide over a frame's open top), in
        # this part's frame
        bvh = self._part_bvh_local(None, name)
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
            # the opening's boundary on the inner side, across its whole width: for each strip across, its
            # farthest cell inward; a straight line fitted through them (a guard's top, under the frame)
            k = pts @ across
            h = pts @ inward
            tops = []
            for kb in np.arange(k.min(), k.max() + 1e-9, 0.003):
                m = np.abs(k - kb) <= 0.0016
                if m.any():
                    tops.append(pts[m][int(np.argmax(h[m]))])
            tops = np.array(tops)
            keep = tops[int(len(tops) * 0.1):max(int(len(tops) * 0.9), int(len(tops) * 0.1) + 2)]
            c_ = keep.mean(0)
            _u, _s, vt = np.linalg.svd(keep - c_, full_matrices=False)
            ld = vt[0] - thin * float(vt[0] @ thin)
            ld /= np.linalg.norm(ld)
            t_ = (tops - c_) @ ld
            got.append({"centre": [float(x) for x in o["centre"]], "area_cm2": o["area_cm2"], "through": [float(x) for x in thin],
                        "edge": [[float(x) for x in (c_ + ld * t_.min())], [float(x) for x in (c_ + ld * t_.max())]]})
        got.sort(key=lambda g_: float(np.linalg.norm(np.array(g_["centre"]) - inner)))
        return got

    def _part_bvh_local(self, part, frame_part=None):
        """A part's polygons as a BVH in frame_part's frame (default its own); part None: the whole object's."""
        c = getattr(self, "_pbvh", {})
        if (part, frame_part) in c:
            return c[(part, frame_part)]
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
            if part is None or not any(vg) or max(set(gs), key=gs.count) == part:
                ps.append(tuple(p.vertices))
        ev.to_mesh_clear()
        bvh = BVHTree.FromPolygons(V, ps) if ps else None
        c[(part, frame_part)] = bvh
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
                opens = self._part_openings(name, P)
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

    def purpose_power_wrap(self, side, mark, glove_cm=0.0):
        P = PURPOSES["power_wrap"]
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        segs = [s_ for s_ in self._segments(side) if not s_[0].startswith("forearm")]
        g = glove_cm / 100.0
        segs = [(n, a, b, r + g) for n, a, b, r in segs]
        span = self._hand_span(side, RW) or RW["sections"]
        pts = [p for _c, ring in span for p in ring]
        # skin within band_cm of the surface counts: a finger's straight capsule stands off a curved grip between
        # its joints by about 0.3 cm, and skin gives where it presses
        cov = self._skin_covered(pts, self._hand_skin(side), P["band_cm"])
        share = sum(cov) / float(len(cov)) if cov else 0.0
        # the web (between the thumb's base and the index knuckle) against the run's inner end, on the palm's side
        web = self._anchor(side, "web")
        tgt = RW["inner"]
        if mark.get("palm_dir") is not None and getattr(self, "_part_P", {}).get(mark["part"]) is not None:
            if "_web" not in mark:
                mark["_web"] = self.web_point(mark, mark["palm_dir"])[0]
            tgt = self._obj_frame(mark["part"]) @ Vector(mark["_web"])
        web_gap = (web - tgt).length * 100.0
        palm = [s_ for s_ in segs if s_[0].startswith("palm")]
        pg = min((self._seg_point_cm(a, b, r, p) for _n, a, b, r in palm for p in pts), default=99.0)
        # the hand high on the part counts as much as its cover: 3 a cm the web is off its place
        err = max(0.0, P["coverage_min"] - share) * 10.0 + 3.0 * max(0.0, web_gap - P["web_gap_max_cm"]) + max(0.0, pg - P["palm_gap_max_cm"])
        return {"purpose": "power_wrap", "met": err <= 1e-6, "error": round(err, 3), "coverage": round(share, 3),
                "coverage_min": P["coverage_min"], "web_gap_cm": round(web_gap, 2), "web_gap_max_cm": P["web_gap_max_cm"],
                "palm_gap_cm": round(pg, 2), "sections": len(span), "covered_points": [p for p, c in zip(pts, cov) if c]}

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
        upw = (F.to_quaternion() @ Vector(run["axis"])).normalized()
        if (F @ Vector(run["inner"]) - e0).dot(upw) < 0:
            upw = -upw
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

    def purpose_support(self, side, mark, glove_cm=0.0):
        P = PURPOSES["support"]
        other = "r" if side == "l" else "l"
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        g = glove_cm / 100.0
        mine = [(n, a, b, r + g) for n, a, b, r in self._segments(side) if not n.startswith("forearm")]
        theirs = [(n, a, b, r + g) for n, a, b, r in self._segments(other) if not n.startswith("forearm")]
        span = self._hand_span(other, RW) or RW["sections"]
        pts = [p for _c, ring in span for p in ring]
        by_other = self._skin_covered(pts, self._hand_skin(other), P["band_cm"])
        open_pts = [p for p, c in zip(pts, by_other) if not c]
        mine_on_open = self._skin_covered(open_pts, self._hand_skin(side), P["band_cm"]) if open_pts else []
        open_cover = sum(mine_on_open) / float(len(mine_on_open)) if mine_on_open else 1.0
        # the support fingers over the other hand's fingers: each support finger's middle and end segment within
        # 0.5 cm of the other hand's middle, ring or little finger
        over = 0
        for fi in ("index", "middle", "ring", "pinky"):
            segs_f = [x for x in mine if x[0].startswith(fi + "2") or x[0].startswith(fi + "3")]
            near = False
            for _n, a, b, r in segs_f:
                for on, oa, ob, orr in theirs:
                    if on[:-2].rstrip("0123456789") in ("middle", "ring", "pinky") and \
                            (self._seg_seg(a, b, oa, ob) - r - orr) * 100.0 <= 0.5:
                        near = True
                        break
                if near:
                    break
            over += int(near)
        over_share = over / 4.0
        ang = math.degrees(self._thumb_dir(side).angle(self._thumb_dir(other)))
        err = (max(0.0, P["open_cover_min"] - open_cover) * 10.0 + max(0.0, P["over_share_min"] - over_share) * 6.0
               + max(0.0, ang - P["thumb_angle_max_deg"]) / 10.0)
        return {"purpose": "support", "met": err <= 1e-6, "error": round(err, 3), "open_cover": round(open_cover, 3),
                "open_cover_min": P["open_cover_min"], "open_points": len(open_pts), "over_share": over_share,
                "over_share_min": P["over_share_min"], "thumb_angle_deg": round(ang, 1), "thumb_angle_max_deg": P["thumb_angle_max_deg"]}

    def purpose_notouch(self, parts, sides=("l", "r"), glove_cm=0.0):
        """No capsule of the hands on a no-touch part (as posed now): the deepest press and where."""
        P = PURPOSES["notouch"]
        vs, ps = self._gun_verts()
        names = {g.index: g.name for g in self.rifle_mesh.vertex_groups}
        vg = []
        for v in self.rifle_mesh.data.vertices:
            gm = max(v.groups, key=lambda x: x.weight) if v.groups else None
            vg.append(names.get(gm.group) if gm else None)
        sel = [p for p in ps if max(set(vg[i] for i in p), key=[vg[i] for i in p].count) in parts]
        if not sel:
            return {"purpose": "notouch", "met": True, "error": 0.0, "worst_cm": None, "parts": list(parts)}
        bvh = BVHTree.FromPolygons(vs, sel)
        g = glove_cm / 100.0
        worst, where = -99.0, None
        for side in sides:
            for n, a, b, r in self._segments(side):
                k = max(2, int((b - a).length / 0.004))
                for i in range(k + 1):
                    q = a.lerp(b, i / k)
                    loc, _nr, _ix, dist = bvh.find_nearest(q)
                    if loc is None:
                        continue
                    into = (r + g - dist) * 100.0 + P["gap_min_cm"]
                    if into > worst:
                        worst, where = into, n
        err = 10.0 * max(0.0, worst)                 # a press into a part that must stay clear costs ten a cm
        return {"purpose": "notouch", "met": err <= 1e-6, "error": round(err, 3), "worst_cm": round(worst - P["gap_min_cm"], 2),
                "where": where, "parts": list(parts)}

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

    def hold_purposes(self, side, marks, glove_cm=0.0):
        """Every purpose the marks give this hand, measured: [result]. A hold mark for this hand (mark["hand"])
        gives its purpose and its thumb and index variants; every notouch mark is checked for both hands."""
        res = []
        for m in marks:
            if m.get("role") != "hold" or m.get("hand", side) != side:
                continue
            if m["purpose"] == "power_wrap":
                res.append(self.purpose_power_wrap(side, m, glove_cm))
                if m.get("index") == "extended" and self._object["parts"][m["part"]]["runs"][m.get("run", 0)]["openings"]:
                    res.append(self.purpose_index_extended(side, m, glove_cm))
                if m.get("thumb") == "inline":
                    res.append(self.purpose_thumb_inline(side, m, glove_cm))
            elif m["purpose"] == "support":
                res.append(self.purpose_support(side, m, glove_cm))
                if m.get("thumb") == "inline":
                    other = "r" if side == "l" else "l"
                    res.append(self.purpose_thumb_inline(side, m, glove_cm, along=self._thumb_dir(other)))
        nt = [m["part"] for m in marks if m.get("role") == "notouch"]
        if nt:
            res.append(self.purpose_notouch(nt, sides=(side,), glove_cm=glove_cm))
        res.append(self.purpose_comfort(side))
        return res

    # --- the hold search: the whole arm, the purposes as the score ------------------------------------------------
    def _hold_score(self, side, marks, glove_cm, other, soft, tree):
        res = self.hold_purposes(side, marks, glove_cm)
        perr = sum(r["error"] for r in res)
        pen, worst = self.hand_presses(side, tree, glove_cm, other=other, soft=soft)
        FL = self._limits()[1]
        TL = self._thumb_limits()
        fbad = len(self._finger_report(side, FL)[1]) + len(self._thumb_report(side, TL)[1])
        abad = len(self._arm_report(side, self._limits()[0])[1])
        sc = 10.0 * perr + 25.0 * pen + 200.0 * (fbad + abad)
        return sc, {"purposes_error": round(perr, 3), "press": round(pen, 3), "presses": worst[:3], "digit_rules": fbad,
                    "arm_rules": abad, "purposes": [{k: v for k, v in r.items() if k != "covered_points"} for r in res]}

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

    def seed_hold(self, side, mark, palm_dir=None, roll_deg=0.0, along_cm=0.0):
        """A first place for a hand on its mark, from the run's geometry alone: the web at the run's inner end on
        the palm's side, the palm facing into the run, the knuckles' line along its axis (toward the inner end),
        turned roll_deg about the axis from palm_dir (object frame; without one, roll_deg from the run's own u),
        along_cm further from the inner end (a support hand under the other).
        Returns place_hand's answer."""
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        F = self._obj_frame(mark["part"])
        ax = RW["axis"]
        if (RW["inner"] - RW["outer"]).dot(ax) < 0:
            ax = -ax
        pd = (F.to_quaternion() @ Vector(palm_dir)).normalized() if palm_dir is not None else (F.to_quaternion() @ Vector(run["u"])).normalized()
        pd = (pd - ax * pd.dot(ax)).normalized()
        pd = Quaternion(ax, math.radians(roll_deg)) @ pd
        # the section near the web and the one at mid hand, their surface on the palm's side
        secs = RW["sections"]
        top = min(secs, key=lambda s_: (s_[0] - RW["inner"]).length)
        mid = min(secs, key=lambda s_: abs((s_[0] - RW["inner"]).length - 0.045))
        edge = lambda s_: max(s_[1], key=lambda p: (p - s_[0]).dot(pd))         # noqa: E731
        palm_r = (self._fitted_radii() or {}).get("palm", 0.018)
        web_t = edge(top) + pd * 0.004 - ax * (0.005 + along_cm / 100.0)
        if getattr(self, "_part_P", {}).get(mark["part"]) is not None:
            # the web up under the part's step (the tang), as the reader finds it on the palm's side
            wp, _past = self.web_point(mark, list(F.to_quaternion().inverted() @ pd))
            web_t = F @ Vector(wp) + pd * 0.004 - ax * (0.003 + along_cm / 100.0)
        palm_t = edge(mid) + pd * palm_r - ax * (along_cm / 100.0)
        across = ax if side == "r" else ax           # the knuckles' line (little finger to index) runs toward the inner end
        return self.place_hand(side, {"web": list(web_t), "palm": list(palm_t)}, facing=list(-pd), frame="world", across=list(across))

    def hold_object(self, side, marks, glove_cm=0.0, other=True, soft=None, generations=40, pop=16, seed=5,
                    sigma_cm=1.0, sigma_deg=8.0, close=None, wrap=None):
        """A hand's hold searched over the whole arm, from where it is now: the hand's place (3) and turn (3), the
        elbow's swing (1), the shoulder girdle (2: forward, up), each finger's bend (4) and the thumb's base turn
        (3) and two bends (2), by an evolution strategy; each try scored by its purposes (hold_purposes), its
        presses into the object and the other hand, and the body's rules (each broken all but forbidden). close:
        {finger: [3 bends]} the fingers start from (else as they are). wrap: {finger: "wrap" | "straight"}: each
        try closes those fingers until their skin meets the object or the other hand (as real fingers close on a
        handle), the straight ones laid straight; their bends are then not searched. Returns the score's parts."""
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
            plan = {fi: ({"pose": "wrap", "max_curl": [90.0, 100.0, 80.0]} if how == "wrap" else {"pose": "straight", "curl": [0.0, 0.0, 3.0]})
                    for fi, how in wrap.items()}
            for k_, fi in enumerate(fins):
                if fi in wrap:
                    scale[9 + k_] = 0.0

        def apply(x):
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
            for k_, fi in enumerate(fins):
                if plan and fi in plan:
                    continue
                d = x[9 + k_]
                c3 = [base[fi][0] + d, base[fi][1] + d, base[fi][2] + d * 2.0 / 3.0]
                for j, cc in zip(("01", "02", "03"), c3):
                    self._set_curl(side, fi, j, cc, FL)
            if plan:
                # the fingers opened, then closed until they meet the object or the other hand
                for fi in plan:
                    for j in ("01", "02", "03"):
                        self._set_curl(side, fi, j, 0.0, FL)
                bpy.context.view_layer.update()
                self.close_hand(side, plan, tree, glove_cm, other)
            tv = Vector(x[13:16])
            if tv.length > 1e-6:
                head = self._bw(n1).translation.copy()
                self._turn_about(n1, tv.normalized(), tv.length, head)
            bpy.context.view_layer.update()
            for j, b in (("02", tb[0] + x[16]), ("03", tb[1] + x[17])):
                cur = self._thumb_report(side, TL)[0].get("thumb_" + j, [0, 0])[0]
                if self.R.get("thumb_hinge") == "ray":
                    self._thumb_flex(side, j, b - cur)
                else:
                    self._set_thumb_bend(side, tb[0] + x[16], tb[1] + x[17], TL)
                    break
            bpy.context.view_layer.update()
            return short

        def score(x):
            short = apply(x)
            sc, why = self._hold_score(side, marks, glove_cm, other, soft, tree)
            return sc + 4.0 * short, why

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
                "evaluations": generations * pop + 2, **why}

    # --- the gate --------------------------------------------------------------------------------------------
    def hold_gate(self, marks, sides=("r", "l"), glove_cm=0.0, skin=True):
        """Does each hand do its job: every purpose its marks give met, no no-touch contact, every joint in its
        comfort band, no body rule broken, and (skin) the drawn skin out of the object and the other hand. A hand
        the marks give a hold that is off its part fails. Returns per hand its results and pass, and pass."""
        out, ok = {}, True
        for side in sides:
            if not any(m.get("role") == "hold" and m.get("hand", side) == side for m in marks):
                continue
            res = [{k: v for k, v in r.items() if k != "covered_points"} for r in self.hold_purposes(side, marks, glove_cm)]
            FL = self._limits()[1]
            TL = self._thumb_limits()
            body = self._finger_report(side, FL)[1] + self._thumb_report(side, TL)[1]
            hp = self.hand_presses(side, self._tree(), glove_cm, other=True)[0]
            rec = {"purposes": res, "body_rules": body, "press": round(hp, 3)}
            rec["pass"] = all(r["met"] for r in res) and not body and hp <= 0.0
            out[side] = rec
            ok = ok and rec["pass"]
        if skin:
            sp = self.skin_presses("both", glove_cm=glove_cm, other=True)
            out["skin"] = sp
            ok = ok and sp["ok"]
        out["pass"] = ok
        return out
