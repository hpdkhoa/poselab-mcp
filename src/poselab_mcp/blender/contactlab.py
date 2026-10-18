"""The contact model (0.9.0): how a hand touches an object, measured on the drawn skin and the object's surface in 3D.

The object: its surface sampled every 2 mm (the object reader's samples), each sample labelled by its part and, on a
part a hand holds, by its region: where it lies round the run and which way its surface faces (the back, the front,
the left and right sides, the tang over the run, the opening's edge line). Labels come from the surface, not from
its polygons: one polygon of a low-poly model can span two regions. The hand: every skin vertex labelled by its hand
part (the palm, the web, each digit segment's pad, side and back), with its area and normal in the world.

A contact pair is a hand part on an object region: skin within band_cm of the surface (or inside it) and facing it,
summed by the skin's area. A press is skin inside the object (its signed distance from the exact mesh, ray parity
for inside) or inside the other hand's skin past what skin gives. A region's cover is the share of its surface the
skin touches. The same numbers colour the pictures."""
import math
import os

import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

DIGITS = ("thumb", "index", "middle", "ring", "pinky")
REGION_COLOURS = {"back": (0.85, 0.25, 0.2), "front": (0.2, 0.45, 0.9), "left": (0.2, 0.75, 0.3), "right": (0.95, 0.75, 0.15),
                  "tang": (0.75, 0.3, 0.85), "opening_edge": (0.1, 0.85, 0.85), "end": (0.6, 0.45, 0.3),
                  "other": (0.5, 0.5, 0.53), "notouch": (0.12, 0.12, 0.13)}
HAND_COLOURS = {"palm": (0.9, 0.3, 0.25), "palm_back": (0.5, 0.2, 0.2), "palm_side": (0.75, 0.45, 0.35),
                "web": (1.0, 0.6, 0.1), "forearm": (0.4, 0.4, 0.42),
                "pad": (0.2, 0.75, 0.35), "side": (0.3, 0.5, 0.9), "back": (0.55, 0.55, 0.6),
                "thumb_pad": (0.95, 0.85, 0.2), "thumb_side": (0.7, 0.6, 0.2), "thumb_back": (0.5, 0.45, 0.3)}
SAMPLE_CM = 0.2


def _lab_key(l_):
    """A hand label's colour key: a digit segment's pad, side or back (the thumb's apart), else the label."""
    head = l_.split("_")[0]
    if head[:-1] in DIGITS and head[-1:].isdigit():
        kind = l_.split("_")[-1]
        return ("thumb_" + kind) if head.startswith("thumb") else kind
    return l_


def _region_colour(r):
    if r.startswith("notouch"):
        return REGION_COLOURS["notouch"]
    if r.startswith("other"):
        return REGION_COLOURS["other"]
    return REGION_COLOURS.get(r, REGION_COLOURS["other"])


def _heat(d_cm, band_cm):
    """A colour for a signed gap: red deep inside, orange to yellow just inside, green touching, blue near, grey far."""
    if d_cm < 0.0:
        k = min(1.0, -d_cm / 0.6)
        return (1.0, 0.75 * (1.0 - k), 0.0)
    if d_cm <= band_cm:
        return (0.1, 0.85, 0.2)
    if d_cm <= 1.0:
        k = (d_cm - band_cm) / max(1e-6, 1.0 - band_cm)
        return (0.15, 0.4 + 0.3 * (1 - k), 0.95)
    return (0.6, 0.6, 0.62)


class ContactMixin:
    # --- the object's surface, labelled ------------------------------------------------------------------------
    def object_regions(self, marks=None, left_dir=None):
        """The object's surface sampled every 2 mm and each sample labelled: "notouch:<part>" on a part that must
        stay clear; on a held part, by where it lies round the run and which way it faces (back, front, left,
        right; "tang" above the run facing the palm's way; "end" facing along the run; "opening_edge" along the
        opening's edge line); else "other:<part>". left_dir: the object's left in the held part's frame (names the
        sides; else side_a and side_b). Keeps the samples in each part's frame. Returns {label: area cm2}."""
        self._need()
        obj = getattr(self, "_object", None) or self.read_object()
        pts = self._part_points(spacing_cm=SAMPLE_CM)
        notouch = {m["part"] for m in (marks or []) if m.get("role") == "notouch"}
        # the regions are named in the object's own frame, by the first hold on each part (a power wrap first): a
        # second hand's mark on the same part does not rename them
        holds, seen = [], set()
        for m in sorted((m for m in (marks or []) if m.get("role") == "hold"), key=lambda m: m.get("purpose") != "power_wrap"):
            if m["part"] not in seen:
                holds.append(m)
                seen.add(m["part"])
        samples = []                                   # (part, local point, local normal, label)
        for part, (P, N) in pts.items():
            labs = np.array(["notouch:" + part if part in notouch else "other:" + part] * len(P), dtype=object)
            for m in holds:
                if m["part"] != part or part not in obj["parts"] or not obj["parts"][part]["runs"]:
                    continue
                run = obj["parts"][part]["runs"][m.get("run", 0)]
                d = np.array(run["axis"])
                inner, outer = np.array(run["inner"]), np.array(run["outer"])
                inward = d if float((inner - outer) @ d) >= 0 else -d
                L_run = float(np.linalg.norm(inner - outer))
                pd = np.array(m.get("palm_dir") or run["u"], dtype=float)
                pd = pd - inward * float(pd @ inward)
                pd /= np.linalg.norm(pd)
                sd = np.cross(inward, pd)
                web_h = float((np.array(m["_web"]) - inner) @ inward) if m.get("_web") else 0.0
                h = (P - inner) @ inward
                radv = (P - inner) - np.outer(h, inward)
                rad = np.linalg.norm(radv, axis=1)
                zone = (h >= -L_run - 0.01) & (h <= web_h + 0.012) & (rad <= 0.05)
                nn = N - np.outer(N @ inward, inward)
                nl = np.linalg.norm(nn, axis=1)
                ang = np.degrees(np.arctan2(N @ sd, N @ pd))
                for i in np.nonzero(zone)[0]:
                    if nl[i] < 0.5:
                        labs[i] = "end"
                    elif h[i] > 0.003 and abs(ang[i]) <= 60.0:
                        labs[i] = "tang"
                    elif abs(ang[i]) <= 45.0:
                        labs[i] = "back"
                    elif abs(ang[i]) >= 135.0:
                        labs[i] = "front"
                    else:
                        nm = "side_a" if ang[i] > 0 else "side_b"
                        if left_dir is not None:
                            ld = np.array(left_dir, dtype=float)
                            nm = "left" if float((sd if ang[i] > 0 else -sd) @ ld) > 0 else "right"
                        labs[i] = nm
                for op in run.get("openings", [])[:1]:
                    e0, e1 = np.array(op["edge"][0]), np.array(op["edge"][1])
                    thru = np.array(op["through"])
                    ld_ = (e1 - e0) - thru * float((e1 - e0) @ thru)
                    ln = float(np.linalg.norm(ld_))
                    if ln < 1e-6:
                        continue
                    ld_ /= ln
                    q = (P - e0) - np.outer((P - e0) @ thru, thru)
                    t_ = q @ ld_
                    off = np.linalg.norm(q - np.outer(t_, ld_), axis=1)
                    sel = (t_ >= -0.005) & (t_ <= ln + 0.005) & (off <= 0.006)
                    labs[sel & ~np.char.startswith(labs.astype(str), "notouch")] = "opening_edge"
            for p_, n_, l_ in zip(P, N, labs):
                samples.append((part, p_, n_, str(l_)))
        self._samples = samples
        a = SAMPLE_CM * SAMPLE_CM
        out = {}
        for s_ in samples:
            out[s_[3]] = out.get(s_[3], 0.0) + a
        return {k: round(v, 2) for k, v in sorted(out.items())}

    def _samples_world(self):
        """The labelled samples in the world as the object is posed now: (points n x 3, labels)."""
        by_part = {}
        for i, (part, p_, _n, _l) in enumerate(self._samples):
            by_part.setdefault(part, []).append(i)
        W = np.zeros((len(self._samples), 3))
        for part, idx in by_part.items():
            F = self._obj_frame(part)
            R_ = np.array(F.to_3x3())
            T_ = np.array(F.translation)
            Pl = np.array([self._samples[i][1] for i in idx])
            W[idx] = Pl @ R_.T + T_
        return W, [s_[3] for s_ in self._samples]

    # --- the hand's labelled skin --------------------------------------------------------------------------------
    def _skin_now(self):
        """The skin as posed: world points, normals, each vertex's area in cm2 (its share of the world-size
        polygons round it), its heaviest bone, and the triangles."""
        ob = self.meshes[0]
        names = {g.index: g.name for g in ob.vertex_groups}
        bone = []
        for v0 in ob.data.vertices:
            gm = max(v0.groups, key=lambda x: x.weight) if v0.groups else None
            bone.append(names.get(gm.group, "") if gm else "")
        dg = bpy.context.evaluated_depsgraph_get()
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        M = ev.matrix_world
        R3 = M.to_3x3()
        P = np.array([list(M @ v.co) for v in me.vertices])
        N = np.array([list((R3 @ v.normal).normalized()) for v in me.vertices])
        A = np.zeros(len(P))
        tris = []
        for p in me.polygons:
            vs = list(p.vertices)
            for k in range(1, len(vs) - 1):
                tris.append((vs[0], vs[k], vs[k + 1]))
                a = 0.5 * float(np.linalg.norm(np.cross(P[vs[k]] - P[vs[0]], P[vs[k + 1]] - P[vs[0]]))) * 1e4
                for i in (vs[0], vs[k], vs[k + 1]):
                    A[i] += a / 3.0
        ev.to_mesh_clear()
        return P, N, A, bone, tris

    def hand_labels(self, side, P=None, N=None, bone=None):
        """Each skin vertex of a hand labelled by its hand part: palm (its palm side), palm_back, palm_side, web,
        forearm, and each digit segment's pad, side or back (index2_pad, thumb3_side ...); a finger segment's pad
        way follows its curl (a curled finger's pad faces back along the hand), the thumb's its own pad axis.
        Returns {vertex index: label}."""
        if P is None:
            P, N, _A, bone, _t = self._skin_now()
        hand = self._n("hand", side)
        f0 = self._palm_facing(side)
        w = self._bw(hand).translation
        d0 = (self._bw(self._n("finger", side, "middle", "01")).translation - w).normalized()
        d0 = (d0 - f0 * d0.dot(f0)).normalized()
        web = self._anchor(side, "web")
        TP = {}
        for j in ("01", "02", "03"):
            try:
                n = self._n("finger", side, "thumb", j)
                TP[j] = (self._bw(n).to_quaternion() @ self._thumb_pad_axis(side, j)).normalized()
            except Exception:
                TP[j] = f0
        seg = {}
        for fi in ("index", "middle", "ring", "pinky"):
            pts = [self._bw(self._n("finger", side, fi, j)).translation for j in ("01", "02", "03")] + [self._finger_tip(fi, side)]
            for k, j in enumerate(("01", "02", "03")):
                s = (pts[k + 1] - pts[k]).normalized()
                th = math.atan2(s.dot(f0), s.dot(d0))
                seg[(fi, j)] = (f0 * math.cos(th) - d0 * math.sin(th)).normalized()
        out = {}
        suf = "_" + side
        for i, b in enumerate(bone):
            if not b.endswith(suf):
                continue
            n = Vector(N[i])
            base = b[:-2]
            if base.startswith(("lowerarm", "forearm")) or "twist" in base:
                out[i] = "forearm"
                continue
            if base.startswith(("upperarm", "clavicle")):
                continue
            fi = next((x for x in DIGITS if base.startswith(x)), None)
            if fi and "metacarpal" not in base:
                j = base.split("_")[1] if "_" in base else "01"
                pad = TP.get(j, f0) if fi == "thumb" else seg.get((fi, j), f0)
                c = n.dot(pad)
                kind = "pad" if c > 0.5 else ("back" if c < -0.5 else "side")
                out[i] = "%s%s_%s" % (fi, j[-1], kind)
                continue
            if (Vector(P[i]) - web).length <= 0.012:
                out[i] = "web"
            else:
                c = n.dot(f0)
                out[i] = "palm" if c > 0.3 else ("palm_back" if c < -0.3 else "palm_side")
        return out

    def palm_check(self, side):
        """Is the palm's facing the side the fingers curl to: the facing against the middle finger's knuckle-to-tip
        line (above 0 when the finger curls toward the palm, as a hand closes), with that finger's curl."""
        f0 = self._palm_facing(side)
        k = self._bw(self._n("finger", side, "middle", "01")).translation
        tip = self._finger_tip("middle", side)
        FL = self._limits()[1]
        fr = self._finger_report(side, FL)[0]
        return {"facing_dot_curl": round(f0.dot((tip - k).normalized()), 3),
                "middle_curl_deg": [fr["middle_0%d" % j][0] for j in (1, 2, 3)]}

    # --- the contact map -------------------------------------------------------------------------------------------
    def contact_map(self, sides=("r", "l"), band_cm=0.2, near_cm=1.0, facing_max_deg=70.0, skin_give_cm=0.3,
                    marks=None, left_dir=None):
        """The hands' contact with the object and with each other, on the drawn skin, as posed now: every pair
        (hand part, object region or the other hand's part) with its contact area (cm2), its press area and
        deepest press, its mean facing; per hand the totals; per object region its area and the share the skin
        touches. Skin on skin presses only past skin_give_cm. Kept for the pictures."""
        self._need()
        if getattr(self, "_samples", None) is None:
            self.object_regions(marks, left_dir)
        P, N, A, bone, tris = self._skin_now()
        SW, SL = self._samples_world()
        skd = KDTree(len(SW))
        for i, p in enumerate(SW):
            skd.insert(Vector(p), i)
        skd.balance()
        dg = bpy.context.evaluated_depsgraph_get()
        ev = self.rifle_mesh.evaluated_get(dg)
        me = ev.to_mesh()
        Mw = self.rifle_mesh.matrix_world
        bvh = BVHTree.FromPolygons([Mw @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons])
        ev.to_mesh_clear()
        surf = self.Surface(bvh)
        Pv = [Vector(p) for p in P]
        hand_bvh = {}
        labs_all = {}
        for s in ("r", "l"):
            sel = {i for i, b in enumerate(bone) if b.endswith("_" + s)}
            ts = [t for t in tris if t[0] in sel and t[1] in sel and t[2] in sel]
            hand_bvh[s] = (BVHTree.FromPolygons(Pv, ts) if ts else None, ts)
            labs_all[s] = self.hand_labels(s, P, N, bone)
        vert_d, pairs, totals = {}, {}, {}
        for s in sides:
            other = "r" if s == "l" else "l"
            ob_bvh, ots = hand_bvh[other]
            tot = {"contact_cm2": 0.0, "press_cm2": 0.0, "deepest_cm": 0.0, "hand_press_cm2": 0.0, "hand_deepest_cm": 0.0,
                   "on_other_cm2": 0.0, "by_region": {}}
            for i, lab in labs_all[s].items():
                p, n, a = Pv[i], Vector(N[i]), A[i]
                loc, nrm, _pi, dist = bvh.find_nearest(p, near_cm / 100.0)
                if loc is not None:
                    d = -dist if dist < 0.02 and surf.inside(p) else dist
                    d_cm = d * 100.0
                    vert_d[i] = d_cm
                    _c, si, _dd = skd.find(loc)
                    reg = SL[si] if si is not None else "other"
                    face = math.degrees(n.angle(-nrm)) if n.length > 0 and nrm.length > 0 else 180.0
                    pr = pairs.setdefault((s, lab, reg), {"contact_cm2": 0.0, "press_cm2": 0.0, "deepest_cm": 0.0, "facing_sum": 0.0})
                    if d_cm < 0.0:
                        pr["press_cm2"] += a
                        pr["deepest_cm"] = max(pr["deepest_cm"], -d_cm)
                        tot["press_cm2"] += a
                        tot["deepest_cm"] = max(tot["deepest_cm"], -d_cm)
                    if d_cm <= band_cm and (d_cm < 0.0 or face <= facing_max_deg):
                        pr["contact_cm2"] += a
                        pr["facing_sum"] += face * a
                        tot["contact_cm2"] += a
                        tot["by_region"][reg] = tot["by_region"].get(reg, 0.0) + a
                if ob_bvh is not None:
                    l2, n2, ti, d2 = ob_bvh.find_nearest(p, near_cm / 100.0)
                    if l2 is not None:
                        inside = (p - l2).dot(n2) < 0.0 and d2 < 0.02
                        dd = (-d2 if inside else d2) * 100.0
                        okey = (s, lab, "hand_%s:%s" % (other, labs_all[other].get(ots[ti][0], "?")))
                        pr = pairs.setdefault(okey, {"contact_cm2": 0.0, "press_cm2": 0.0, "deepest_cm": 0.0, "facing_sum": 0.0})
                        if dd < -skin_give_cm:
                            pr["press_cm2"] += a
                            pr["deepest_cm"] = max(pr["deepest_cm"], -dd - skin_give_cm)
                            tot["hand_press_cm2"] += a
                            tot["hand_deepest_cm"] = max(tot["hand_deepest_cm"], -dd - skin_give_cm)
                        if dd <= band_cm:
                            pr["contact_cm2"] += a
                            tot["on_other_cm2"] += a
                        hd = dd + skin_give_cm if dd < 0.0 else dd
                        vert_d[i] = min(vert_d.get(i, 99.0), hd if hd < 0.0 else max(hd, 0.0))
            totals[s] = {k: (round(v, 2) if isinstance(v, float) else {kk: round(vv, 2) for kk, vv in sorted(v.items(), key=lambda x: -x[1])})
                         for k, v in tot.items()}
        # each region's cover: its samples with a skin vertex within band_cm (0.35 at least: the samples are 2 mm apart)
        kd = KDTree(len(Pv))
        for i, p in enumerate(Pv):
            kd.insert(p, i)
        kd.balance()
        owner = {}
        for s in ("r", "l"):
            owner.update({i: (s, l_) for i, l_ in labs_all[s].items()})
        a_s = SAMPLE_CM * SAMPLE_CM
        regions, touch = {}, {}
        for k, (p, reg) in enumerate(zip(SW, SL)):
            rc = regions.setdefault(reg, {"area_cm2": 0.0, "touched_cm2": 0.0, "by": {}})
            rc["area_cm2"] += a_s
            _c, vi, dist = kd.find(Vector(p))
            if vi is not None and dist * 100.0 <= max(band_cm, 0.35) and vi in owner:
                rc["touched_cm2"] += a_s
                who = "%s:%s" % (owner[vi][0], _lab_key(owner[vi][1]))
                rc["by"][who] = rc["by"].get(who, 0.0) + a_s
                touch[k] = owner[vi]
        for rc in regions.values():
            rc["share"] = round(rc["touched_cm2"] / rc["area_cm2"], 3) if rc["area_cm2"] else 0.0
            rc["area_cm2"] = round(rc["area_cm2"], 2)
            rc["touched_cm2"] = round(rc["touched_cm2"], 2)
            rc["by"] = {k: round(v, 2) for k, v in sorted(rc["by"].items(), key=lambda x: -x[1])}
        rows = []
        for (s, lab, reg), pr in pairs.items():
            if pr["contact_cm2"] <= 0.0 and pr["press_cm2"] <= 0.0:
                continue
            rows.append({"hand": s, "hand_part": lab, "touches": reg, "contact_cm2": round(pr["contact_cm2"], 2),
                         "press_cm2": round(pr["press_cm2"], 2), "deepest_cm": round(pr["deepest_cm"], 2),
                         "facing_deg": round(pr["facing_sum"] / pr["contact_cm2"], 1) if pr["contact_cm2"] > 0 and pr["facing_sum"] else None})
        rows.sort(key=lambda r: (-r["press_cm2"], -r["contact_cm2"]))
        self._cm_state = {"vert_d": vert_d, "labels": labs_all, "touch": touch, "band_cm": band_cm, "SW": SW, "SL": SL}
        return {"pairs": rows, "hands": totals, "regions": regions, "band_cm": band_cm}

    # --- the pictures -------------------------------------------------------------------------------------------
    def _paint(self, ob, name, colours):
        """Per-vertex colours on a mesh as a colour attribute, made the one drawn."""
        me = ob.data
        if name in me.color_attributes:
            me.color_attributes.remove(me.color_attributes[name])
        ca = me.color_attributes.new(name, "FLOAT_COLOR", "POINT")
        for i, c in enumerate(colours):
            ca.data[i].color = (c[0], c[1], c[2], 1.0)
        me.color_attributes.active_color = ca
        try:
            me.color_attributes.render_color_index = me.color_attributes.find(name)
        except Exception:
            pass

    def _fine_copy(self):
        """A drawn copy of the object as posed, its polygons cut small (about 2 mm), so colours follow the
        labelled surface, not the model's large polygons. Its vertices in the world."""
        dg = bpy.context.evaluated_depsgraph_get()
        ev = self.rifle_mesh.evaluated_get(dg)
        me = bpy.data.meshes.new_from_object(ev)
        me.transform(self.rifle_mesh.matrix_world)
        import bmesh
        bm = bmesh.new()
        bm.from_mesh(me)
        for _ in range(6):
            long_ = [e for e in bm.edges if e.calc_length() > 0.004]
            if not long_:
                break
            bmesh.ops.subdivide_edges(bm, edges=long_, cuts=1, use_grid_fill=True)
            bmesh.ops.triangulate(bm, faces=bm.faces[:])
        bm.to_mesh(me)
        bm.free()
        ob = bpy.data.objects.new("contact_fine", me)
        self.sc.collection.objects.link(ob)
        return ob

    def contact_pictures(self, out_dir, centre, frame_part=None, kinds=("object", "hand", "heat", "contact"),
                         views=("right", "left", "top", "front"), size=(900, 600), ortho_cm=26.0):
        """The model's vision drawn, straight-on views about centre (a world point) along the held part's axes:
        object (the surface by region), hand (the skin by hand part), heat (the skin by its signed gap to the
        object and the other hand), contact (the object's surface by the hand part touching it). Returns paths."""
        st = getattr(self, "_cm_state", None)
        if st is None:
            raise ValueError("run contact_map first")
        sc = self.sc
        Fq = self._obj_frame(frame_part).to_quaternion()
        skin = self.meshes[0]
        fine = self._fine_copy()
        FW = np.array([list(v.co) for v in fine.data.vertices])
        skd = KDTree(len(st["SW"]))
        for i, p in enumerate(st["SW"]):
            skd.insert(Vector(p), i)
        skd.balance()
        near = [skd.find(Vector(p))[1] for p in FW]
        nverts = len(skin.data.vertices)
        keep = (sc.render.engine, sc.render.resolution_x, sc.render.resolution_y, sc.display.shading.light,
                sc.display.shading.color_type, sc.camera, sc.display.shading.show_xray, sc.display.shading.xray_alpha)
        sc.render.engine = "BLENDER_WORKBENCH"
        sc.render.resolution_x, sc.render.resolution_y = size
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "VERTEX"
        hidden = [o for o in sc.objects if o.type == "MESH" and o not in (skin, fine) and not o.hide_render]
        for o in hidden:
            o.hide_render = True
        cam_d = bpy.data.cameras.get("contact_cam") or bpy.data.cameras.new("contact_cam")
        cam = bpy.data.objects.get("contact_cam") or bpy.data.objects.new("contact_cam", cam_d)
        if cam.name not in sc.collection.objects:
            sc.collection.objects.link(cam)
        cam_d.type = "ORTHO"
        cam_d.ortho_scale = ortho_cm / 100.0
        cam_d.clip_start = 0.001
        cam_d.clip_end = 5.0
        sc.camera = cam
        dirs = {"right": Vector((-1, 0, 0)), "left": Vector((1, 0, 0)), "top": Vector((0, 0, 1)), "front": Vector((0, -1, 0)),
                "back": Vector((0, 1, 0)), "under": Vector((0, 0, -1))}
        grey = (0.62, 0.62, 0.64)
        paths = []
        try:
            for kind in kinds:
                if kind == "object":
                    gcol = [_region_colour(st["SL"][k]) for k in near]
                    scol = [grey] * nverts
                    skin.hide_render = True
                elif kind == "hand":
                    gcol = [(0.25, 0.25, 0.27)] * len(near)
                    scol = [grey] * nverts
                    for s, labs in st["labels"].items():
                        for i, l_ in labs.items():
                            scol[i] = HAND_COLOURS.get(_lab_key(l_), grey)
                    skin.hide_render = False
                elif kind == "heat":
                    scol = [grey] * nverts
                    for i, d in st["vert_d"].items():
                        scol[i] = _heat(d, st["band_cm"])
                    gcol = [(0.1, 0.6, 0.2) if k in st["touch"] else (0.3, 0.3, 0.32) for k in near]
                    skin.hide_render = False
                elif kind == "contact":
                    gcol = []
                    for k in near:
                        if k in st["touch"]:
                            s, l_ = st["touch"][k]
                            c = HAND_COLOURS.get(_lab_key(l_), (0.9, 0.9, 0.9))
                            gcol.append(c if s == "r" else tuple(0.55 * x + 0.45 for x in c))
                        else:
                            gcol.append(_region_colour(st["SL"][k]) if False else (0.35, 0.35, 0.37))
                    scol = [grey] * nverts
                    skin.hide_render = True
                else:
                    raise ValueError("kinds: object, hand, heat, contact")
                # a press is skin inside something: the heat is drawn see-through so buried skin shows
                sc.display.shading.show_xray = kind == "heat"
                sc.display.shading.xray_alpha = 0.45
                self._paint(fine, "contact_view", gcol)
                self._paint(skin, "contact_view", scol)
                for v in views:
                    dv = (Fq @ dirs[v]).normalized()
                    upv = (Fq @ (Vector((0, 0, 1)) if v not in ("top", "under") else Vector((0, 1, 0)))).normalized()
                    cam.location = Vector(centre) + dv * 0.8
                    xc = upv.cross(dv).normalized()
                    cam.rotation_euler = Matrix((xc, dv.cross(xc), dv)).transposed().to_euler()
                    path = self._own(os.path.join(out_dir, "contact_%s_%s.png" % (kind, v)))
                    sc.render.filepath = path
                    bpy.ops.render.render(write_still=True)
                    paths.append(path)
        finally:
            skin.hide_render = False
            for o in hidden:
                o.hide_render = False
            bpy.data.objects.remove(fine)
            (sc.render.engine, sc.render.resolution_x, sc.render.resolution_y, sc.display.shading.light,
             sc.display.shading.color_type, sc.camera, sc.display.shading.show_xray, sc.display.shading.xray_alpha) = keep
        return paths
