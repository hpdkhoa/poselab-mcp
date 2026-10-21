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

import contactsolve as CS
from deprecation import deprecated
from sdflab import SDF_TOL_CM

DIGITS = ("thumb", "index", "middle", "ring", "pinky")
REGION_COLOURS = {"back": (0.85, 0.25, 0.2), "front": (0.2, 0.45, 0.9), "left": (0.2, 0.75, 0.3), "right": (0.95, 0.75, 0.15),
                  "tang": (0.75, 0.3, 0.85), "opening_edge": (0.1, 0.85, 0.85), "end": (0.6, 0.45, 0.3),
                  "other": (0.5, 0.5, 0.53), "notouch": (0.12, 0.12, 0.13)}
HAND_COLOURS = {"palm": (0.9, 0.3, 0.25), "palm_back": (0.5, 0.2, 0.2), "palm_side": (0.75, 0.45, 0.35),
                "web": (1.0, 0.6, 0.1), "forearm": (0.4, 0.4, 0.42),
                "pad": (0.2, 0.75, 0.35), "side": (0.3, 0.5, 0.9), "back": (0.55, 0.55, 0.6),
                "thumb_pad": (0.95, 0.85, 0.2), "thumb_side": (0.7, 0.6, 0.2), "thumb_back": (0.5, 0.45, 0.3)}
SAMPLE_CM = 0.2
# what each purpose asks, as contact on the drawn skin (cm2 of skin touching a region). The areas are first
# estimates for an adult hand on a pistol-sized grip, to be set by the hold benchmark's known holds
CONTACT_PURPOSES = {
    "power_wrap": {"palm_cm2": 6.0, "fingers_front_cm2": 4.0, "fingers_far_cm2": 1.5, "web_cm2": 0.3, "cover_min": 0.30,
                   "estimate": True,
                   "why": "the palm on the run's back and its own side, the finger pads round the front to the far side, the "
                          "web on the tang, a share of the grip's surface covered (the GRASP taxonomy's power wrap)"},
    "thumb_inline": {"pad_cm2": 0.8, "estimate": True,
                     "why": "the thumb's pad pressed a little on the far side, laid in line with the index (FM 3-23.35 2-1)"},
    "index_extended": {"why": "the index straight along the opening's edge, never on a no-touch part (the trigger)"},
    "thumb_wrap": {"pad_cm2": 0.8, "estimate": True,
                   "why": "the thumb closed round the handle opposite the fingers, its pad on it (a round handle's power wrap)"},
    "support": {"palm_cm2": 4.0, "over_cm2": 3.0, "cover_min": 0.20, "estimate": True,
                "why": "the support palm on the side the firing hand leaves open, its finger pads over the firing fingers' "
                       "backs, its own panel covered (FM 3-23.35 2-2)"},
    "press": {"object_cm2": 0.5, "object_deep_cm": 0.15, "hand_cm2": 1.0, "hand_deep_cm": 0.2,
              "why": "skin may give a little: no more than this area inside the object or the other hand, no deeper"},
}
# soft tissue gives under a grip: a press counts only past the tissue's give (cm). The palm's pad of fat and muscle
# flattens a few mm, a finger pad less, the bony back and sides little (rigid meshes do not squash: this is where a
# real grip's contact area comes from)
def _load_skin_give():
    """The skin's give per kind (cm) from poselab_mcp/data/sourced/skin_give.json, with its sources (the owner's
    rule: numbers from measurements or named sources). The palm (Spartacus et al. 2021) and the pads (Dzidek et al.
    2017) are sourced; the file marks the rest as guesses."""
    import json as _json
    p_ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "sourced", "skin_give.json")
    return dict(_json.load(open(p_, encoding="utf-8"))["values_cm"])


SKIN_GIVE = _load_skin_give()
# a holding hand's digits, as a real one holds (tighter than a free hand's ranges): no joint bent backward against the
# grip, no finger twisted out of its plane past what a grasp shows. (least curl, most twist out of plane) degrees
HOLD_DIGIT_LIMITS = {"01": (-5.0, 20.0), "02": (0.0, 8.0), "03": (-5.0, 10.0),
                     "thumb_02": (-5.0, 15.0), "thumb_03": (-15.0, 15.0)}
# clipping: no skin past its tissue's give inside the object, no object inside the flesh past it, no hand inside the
# other past the skin's give; a hair of sampling noise allowed (cm2), nothing deeper
CLIP_NOISE_CM2 = 0.05
PULL_W = 0.3                   # the pull's weight: a cm off its region costs 0.3 of a purpose's error
FINGERS3 = ["middle?_pad", "ring?_pad", "pinky?_pad", "middle?_side", "ring?_side", "pinky?_side"]


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
        idx, lab = self._hand_label_arrays(side, P, N)
        return dict(zip(idx.tolist(), lab))

    def _hand_label_arrays(self, side, P=None, N=None):
        """hand_labels as arrays: (vertex indices, labels), worked out with numpy (the search reads it each try)."""
        if P is None:
            P, N, _A, _b, _t = self._skin_arrays()
        _bone, _T, sidev, kind, segi = self._skin_static()
        sc_ = 1 if side == "r" else 2
        sel = (sidev == sc_) & (kind > 0)
        idx = np.nonzero(sel)[0]
        hand = self._n("hand", side)
        f0 = self._palm_facing(side)
        w = self._bw(hand).translation
        d0 = (self._bw(self._n("finger", side, "middle", "01")).translation - w).normalized()
        d0 = (d0 - f0 * d0.dot(f0)).normalized()
        web = np.array(self._anchor(side, "web"))
        pads = np.zeros((15, 3))
        for k, fi in enumerate(DIGITS):
            for jj, j in enumerate(("01", "02", "03")):
                if fi == "thumb":
                    try:
                        n_ = self._n("finger", side, "thumb", j)
                        pd_ = (self._bw(n_).to_quaternion() @ self._thumb_pad_axis(side, j)).normalized()
                    except Exception:
                        pd_ = f0
                else:
                    a_ = self._bw(self._n("finger", side, fi, j)).translation
                    b_ = self._bw(self._n("finger", side, fi, ("02", "03")[jj])).translation if jj < 2 else self._finger_tip(fi, side)
                    s_ = (b_ - a_).normalized()
                    th = math.atan2(s_.dot(f0), s_.dot(d0))
                    pd_ = (f0 * math.cos(th) - d0 * math.sin(th)).normalized()
                pads[k * 3 + jj] = list(pd_)
        lab = np.empty(len(idx), dtype=object)
        k_ = kind[idx]
        lab[k_ == 1] = "forearm"
        dig = k_ == 3
        if dig.any():
            si = segi[idx[dig]]
            c = np.einsum("ij,ij->i", N[idx[dig]], pads[si])
            kd_ = np.where(c > 0.5, "pad", np.where(c < -0.5, "back", "side"))
            names = np.array(["%s%d" % (DIGITS[q // 3], q % 3 + 1) for q in range(15)], dtype=object)
            lab[dig] = names[si] + "_" + kd_.astype(object)
        pal = k_ == 2
        if pal.any():
            pi_ = idx[pal]
            c = N[pi_] @ np.array(list(f0))
            nearw = np.linalg.norm(P[pi_] - web, axis=1) <= 0.012
            lab[pal] = np.where(nearw, "web", np.where(c > 0.3, "palm", np.where(c < -0.3, "palm_back", "palm_side")))
        return idx, lab.tolist()

    @deprecated()
    def settle_palm(self, side, rounds=4, gap_cm=-0.15, other=False, sample=3, turn=True, turn_len_m=0.15,
                    turn_most_deg=15.0):
        """The hand slid as one body until its palm's skin just meets the object (v0.10.0, calculated): each round
        every palm vertex's signed distance d to the object and the outward normal n there (the grids), and the
        shift x solved as the smallest one with
            n_i . x >= lo_i - d_i      for every palm point (no palm skin past its give: lo = -give + 0.05 cm)
            n_k . x <= gap_cm - d_k    for the nearest palm point k (it meets the surface, gap_cm in)
        (contactsolve.min_shift). With other, the rows also hold every skin point of this hand out of the other
        hand past the skin's give, and out of the object past its own give, so a support hand lies on the firing
        fingers. With turn, the move is a turn w about the palm's middle c as well as a shift t: each row's
        n . (t + w x (p - c)) = n . t + ((p - c) x n) . w, the turn weighed as a move of turn_len_m (15 cm: a turn
        of 0.1 rad costs what a 1.5 cm shift does, so the hand shifts first and turns only when it must, and keeps
        the turn Step A2 gave it), so a palm in the object from two sides (no shift alone gets it
        out) is turned out; at most turn_most_deg a round. The step is linear, so the next round reads the skin
        again. Returns the gap left (cm; below 0 still in the object), the palm's touching area (cm2) and the total
        move (cm; the turns in self._settle_log)."""
        hn = self._n("hand", side)
        left, touching, total = None, 0.0, 0.0
        o = "r" if side == "l" else "l"
        self._settle_log = []
        for _ in range(rounds):
            self._cnt("rounds")
            P, N, A, _b, _t = self._skin_arrays()
            idx, lab = self._hand_label_arrays(side, P, N)
            pv = np.array([int(i) for i, l_ in zip(idx, lab) if l_ in ("palm", "palm_side", "web")])
            if not len(pv):
                break
            give = np.array([SKIN_GIVE[l_] for l_ in lab if l_ in ("palm", "palm_side", "web")])
            D, _w, Nr = self.sdf_query(P[pv])            # the grids: signed distance (cm) and outward normal
            touching = float(A[pv][D <= 0.2].sum())
            left = float(D.min())
            near = D < 1.0
            rows = [Nr[near]]
            at = [P[pv][near]]
            need = [(-give[near] + 0.05 - D[near]) / 100.0]
            k = int(np.argmin(D))
            if np.linalg.norm(Nr[k]) > 1e-6:
                gap_k = max(gap_cm, -give[k] + 0.05 + 0.03)     # inside its own give's band (a palm side gives less)
                rows.append(-Nr[k][None, :])
                at.append(P[pv][k][None, :])
                need.append(np.array([(D[k] - gap_k) / 100.0]))
            if other:
                # every skin point of this hand (sampled): out of the object past its give, out of the other hand
                sel = np.array([int(i) for i in idx[::sample]])
                labs_ = [lab[j] for j in range(0, len(lab), sample)]
                Da, _w2, Na = self.sdf_query(P[sel])
                gv = np.array([SKIN_GIVE.get(_lab_key(l_), SKIN_GIVE["default"]) for l_ in labs_])
                m_ = Da < 1.0
                rows.append(Na[m_])
                at.append(P[sel][m_])
                need.append((-gv[m_] + 0.03 - Da[m_]) / 100.0)
                oh = self._other_hand_bvh(o, P, N)
                if oh is not None:
                    hb, _ts, _ol, olo, ohi = oh
                    for i in sel[np.all((P[sel] >= olo) & (P[sel] <= ohi), axis=1)]:
                        v = Vector(P[i])
                        l2, n2, _ti, d2 = hb.find_nearest(v, 0.03)      # a deep point is up to a finger's width in
                        if l2 is None:
                            continue
                        w_ = v - l2
                        inside = w_.dot(n2) < 0.0
                        dd = -d2 if inside else d2
                        nv = (-w_ if inside else w_).normalized() if w_.length > 1e-9 else n2
                        rows.append(np.array([list(nv)]))
                        at.append(P[i][None, :])
                        need.append(np.array([-0.003 + 0.0003 - dd]))
            # the other way round: the gun's samples inside the hand's flesh past its give (an edge between skin
            # vertices). Sample q at depth delta under the skin, whose outward normal there is n_h: the hand moved by
            # (t, w) takes its skin past q by -n_h . (t + w x (q - c)), so the row is -n_h with need delta - give
            if side not in getattr(self, "_pierce_static", {}):
                self.contact_pairs(side, other=False)       # builds this hand's triangles once
            hk_ = getattr(self, "_pierce_static", {}).get(side)
            if hk_ is not None:
                th_, vi_, polys_ = hk_
                SW_ = self._object_now()[2]
                Ph_ = P[vi_]
                labs_all = dict(zip(idx.tolist(), lab))
                for k_, (dq, lq, t3) in self._pierced_samples(side, Ph_, th_, polys_, SW_, labs_all).items():
                    gq = SKIN_GIVE.get(_lab_key(lq), SKIN_GIVE["default"])
                    if dq <= gq - 0.05:
                        continue
                    tri = Ph_[list(polys_[t3])]
                    nh = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                    if np.linalg.norm(nh) < 1e-12:
                        continue
                    nh /= np.linalg.norm(nh)
                    rows.append(-nh[None, :])
                    at.append(SW_[k_][None, :])
                    need.append(np.array([(dq - gq + 0.05) / 100.0]))
                    left = min(left, -(dq - gq))
            R3 = np.concatenate(rows)
            c = P[pv].mean(0)
            Rm = np.c_[R3, np.cross(np.concatenate(at) - c, R3) / turn_len_m] if turn else R3
            x, viol = CS.min_shift(Rm, np.concatenate(need))
            tr_ = x[:3]
            w = x[3:] / turn_len_m if turn else np.zeros(3)
            ang = float(np.linalg.norm(w))
            if ang > math.radians(turn_most_deg):
                w, tr_ = w * (math.radians(turn_most_deg) / ang), tr_ * (math.radians(turn_most_deg) / ang)
                ang = math.radians(turn_most_deg)
            step = float(np.linalg.norm(tr_))
            if step < 1e-5 and ang < 1e-4:
                break
            self._cnt("palm shifts")
            total += step * 100.0
            H = self._bw(hn).copy()
            Rt = Matrix.Rotation(ang, 4, Vector(list(w / ang))) if ang > 1e-9 else Matrix.Identity(4)
            cv = Vector(list(c))
            M = Matrix.Translation(Vector(list(tr_))) @ Matrix.Translation(cv) @ Rt @ Matrix.Translation(-cv) @ H
            short = self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
            bpy.context.view_layer.update()
            got = (self._bw(hn).translation - H.translation).length * 100.0
            self._settle_log.append({"deepest_cm": round(left, 2), "rows": int(sum(len(r_) for r_ in rows)),
                                     "shift_asked_cm": round(step * 100.0, 2), "turn_deg": round(math.degrees(ang), 2),
                                     "shift_got_cm": round(got, 2),
                                     "arm_short_cm": round(short, 2), "left_unmet_cm": round(viol * 100.0, 2)})
        return (round(left, 2) if left is not None else None), round(touching, 2), round(total, 2)

    def relax_fingers(self, side, fingers=("index", "middle", "ring", "pinky", "thumb"), step_deg=4.0, rounds=8,
                      allow_cm=0.05):
        """Every finger opened, by the drawn skin, until it no longer presses into the object: each round one read
        of the skin, and in each finger still pressing the joint whose segment presses deepest opens by the angle
        that brings its deepest point back to allow_cm (v0.10.0): s = (-allow - d) / (dd/ds) with
        dd/ds = n . (a x (p - c)) for that joint's hinge, cut to 0.5 to 20 degrees; step_deg is used only when the
        lever reads near 0. When closing moves that point out (the lever's sign) the joint closes instead. The drawn
        skin bends at the joints, so the angle is checked on the next round's read. Returns each finger's degrees
        opened (below 0: closed)."""
        bvh, surf = self._object_now()[:2]
        _bone, _T, sidev, kind, segi = self._skin_static()
        sc_ = 1 if side == "r" else 2
        FL = self._limits()[1]
        TL = self._thumb_limits()
        verts = {}
        for fi in fingers:
            k0 = DIGITS.index(fi) * 3
            vi = np.nonzero((sidev == sc_) & (kind == 3) & (segi >= k0) & (segi < k0 + 3))[0]
            if len(vi):
                verts[fi] = (k0, vi[:: max(1, len(vi) // 90)])
        opened = {fi: 0.0 for fi in verts}
        live = set(verts)
        for _ in range(rounds):
            self._cnt("rounds")
            if not live:
                break
            P, _N, _A, _b, _t = self._skin_arrays()
            moved = False
            for fi in sorted(live):
                k0, vi = verts[fi]
                deep = {}
                D, _w, Nn = self.sdf_query(P[vi])
                at = {}
                for i, dcm, nv in zip(vi, D, Nn):
                    if dcm < -allow_cm:
                        j = int(segi[i]) - k0
                        if -dcm / 100.0 > deep.get(j, 0.0):
                            deep[j] = -dcm / 100.0
                            at[j] = (P[i], dcm, nv)
                if not deep:
                    live.discard(fi)
                    continue
                jj = max(deep, key=deep.get)
                jn = ("01", "02", "03")[jj]
                if fi == "thumb":
                    jn = "02" if jn == "01" else jn
                # the opening angle: the joint's hinge (its finger's plane, + closes) and the deepest point's lever
                step_ = -step_deg                     # degrees of closing (below 0 opens)
                p_, dcm_, nv_ = at[jj]
                try:
                    if fi == "thumb":
                        a_ = np.array(list(self._thumb_report(side, TL)[2][jn]))
                    else:
                        a_ = np.array(list(self._finger_report(side, FL)[2][fi]))
                    c_ = np.array(list(self._bw(self._n("finger", side, fi, jn)).translation))
                    a_ = a_ / max(1e-12, np.linalg.norm(a_))
                    tip_ = np.array(list(self._finger_tip(fi, side)))
                    if float(np.dot(np.cross(a_, tip_ - c_), np.array(list(self._palm_facing(side))))) < 0.0 and fi != "thumb":
                        a_ = -a_
                    lever = float(np.dot(nv_, np.cross(a_, p_ - c_))) * 100.0     # cm per radian of closing
                    if abs(lever) > 1e-4:
                        s_ = math.degrees((-allow_cm - dcm_) / lever)            # below 0 opens
                        step_ = math.copysign(min(20.0, max(0.5, abs(s_))), s_)
                except (KeyError, IndexError):
                    pass
                if fi == "thumb":
                    if self.R.get("thumb_hinge") == "ray":
                        self._thumb_flex(side, jn, step_)
                    else:
                        tr = self._thumb_report(side, TL)[0]
                        b2, b3 = tr.get("thumb_02", [0, 0])[0], tr.get("thumb_03", [0, 0])[0]
                        self._set_thumb_bend(side, b2 + (step_ if jn == "02" else 0.0), b3 + (step_ if jn == "03" else 0.0), TL)
                else:
                    cur = self._finger_report(side, FL)[0]["%s_%s" % (fi, jn)][0]
                    self._set_curl(side, fi, jn, cur + step_, FL)
                opened[fi] -= step_
                moved = True
            if moved:
                bpy.context.view_layer.update()
        return opened

    def _rodrigues(self, X, h, a, ang):
        """Points X (n x 3) turned ang radians about the line through h along the unit axis a."""
        c, s_ = math.cos(ang), math.sin(ang)
        Y = X - h
        return h + Y * c + np.cross(a, Y) * s_ + np.outer(Y @ a, a) * (1.0 - c)

    def conform_digits(self, side, digits=("index", "middle", "ring", "pinky"), step_deg=3.0, touch_cm=0.08,
                       other=False, max_steps=60, sample=24, press_cm=0.12):
        """The digits closed onto the object the way a hand closes on a handle: every joint of a finger closes
        together (the end joints a little slower); when a segment's skin touches the object (or, with other, the
        other hand) the joints behind it stop and the ones past it keep closing, until each segment rests on the
        surface or its joint is at its limit; a segment pressed past press_cm is moved back out. Each step's angle is
        calculated (contactsolve.chain_close) from the object's signed distance and outward normal at the skin and
        each joint's lever, not a fixed step: the joints close until the first point would reach the surface, and a
        pressed segment opens by the angle that brings it back to half the press. The skin is read once and turned
        with the joints in numpy (a segment's skin rides its joint's hinge rigidly), so a closing costs no scene
        update per step; the bones are set once at the end. step_deg and max_steps are no longer used. Returns per
        digit the degrees each joint closed (below 0 opened), which segments touch, and the rounds taken."""
        bvh, surf = self._object_now()[:2]
        P, N, _A, _b, _t = self._skin_arrays()
        _bone, _T, sidev, kind, segi = self._skin_static()
        sc_ = 1 if side == "r" else 2
        FL = self._limits()[1]
        TL = self._thumb_limits()
        fr = self._finger_report(side, FL)[0]
        tr = self._thumb_report(side, TL)[0]
        planes = self._finger_report(side, FL)[2]
        f0 = self._palm_facing(side)
        oh = None
        if other:
            o = "r" if side == "l" else "l"
            oh = self._other_hand_bvh(o, P, N)
        # the object's samples inside each digit segment's flesh past (nearly) its give: an edge between skin
        # vertices that no skin point sees (the ring finger's back on the M1911, 0.86 cm in). Each becomes an inner
        # point of its segment for the solver, its distance 0 at the press wall's edge less press_cm, so the
        # solver's pressed rule is the wall's
        inner_by = {}
        if side not in getattr(self, "_pierce_static", {}):
            self.contact_pairs(side, other=False)
        hk_ = getattr(self, "_pierce_static", {}).get(side)
        if hk_ is not None:
            th_, vi_, polys_ = hk_
            SW_ = self._object_now()[2]
            Ph_ = P[vi_]
            ix_, lb_ = self._hand_label_arrays(side, P, N)
            for k_, (dq, lq, t3) in self._pierced_samples(side, Ph_, th_, polys_, SW_, dict(zip(ix_.tolist(), lb_))).items():
                head = lq.split("_")[0]
                if head[:-1] not in DIGITS or not head[-1:].isdigit():
                    continue
                gq = SKIN_GIVE.get(_lab_key(lq), SKIN_GIVE["default"])
                if dq <= gq - 0.05:
                    continue
                tri = Ph_[list(polys_[t3])]
                nh = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                if np.linalg.norm(nh) < 1e-12:
                    continue
                e_ = inner_by.setdefault((head[:-1], "0" + head[-1]), ([], [], []))
                e_[0].append(SW_[k_])
                e_[1].append(gq + 0.02 - dq - press_cm)
                e_[2].append(-nh / np.linalg.norm(nh))
        out = {}
        for fi in digits:
            k0 = DIGITS.index(fi) * 3
            joints = ("02", "03") if fi == "thumb" else ("01", "02", "03")
            heads = {}
            for j in ("01", "02", "03"):
                heads[j] = np.array(list(self._bw(self._n("finger", side, fi, j)).translation))
            tip = np.array(list(self._finger_tip(fi, side)))
            # the hinge of each joint and its closing sign: turning + moves the segment past it toward its pad
            hinge = {}
            for j in joints:
                if fi == "thumb":
                    try:
                        a = np.array(list(self._thumb_report(side, TL)[2][j]))
                    except Exception:
                        continue
                else:
                    a = np.array(list(planes[fi]))
                a = a / max(1e-9, np.linalg.norm(a))
                nxt = heads[("02", "03")[("01", "02").index(j)]] if j in ("01", "02") else tip
                seg = nxt - heads[j]
                pad_way = np.array(list(f0))
                if fi == "thumb":
                    pn = self._n("finger", side, "thumb", j)
                    pad_way = np.array(list((self._bw(pn).to_quaternion() @ self._thumb_pad_axis(side, j)).normalized()))
                if float(np.dot(np.cross(a, seg), pad_way)) < 0.0:
                    a = -a
                hinge[j] = a
            if not hinge:
                continue
            # each segment's skin (sampled), the points that ride each joint
            segv = {}
            for jj, j in enumerate(("01", "02", "03")):
                vi = np.nonzero((sidev == sc_) & (kind == 3) & (segi == k0 + jj))[0]
                segv[j] = vi[:: max(1, len(vi) // sample)]
            pts = {j: P[segv[j]].copy() for j in ("01", "02", "03")}
            hd = {j: heads[j].copy() for j in ("01", "02", "03")}
            room = {}
            for j in hinge:
                if fi == "thumb":
                    cur = tr.get("thumb_" + j, [0.0])[0]
                    room[j] = max(0.0, TL[j][1] - cur)
                else:
                    cur = fr["%s_%s" % (fi, j)][0]
                    room[j] = max(0.0, FL[j][1] - cur)
            # the knuckle leads, as a hand closes on a handle: the base segment comes down first, then the middle,
            # then the end (all three at once land the fingertip first and stop the finger short of a wrap)
            rate = {"01": 1.0, "02": 0.5, "03": 0.35} if fi != "thumb" else {"02": 1.0, "03": 0.6}
            # v0.10.0: each step's angle is calculated from the object's distance and normal at the skin (contactsolve),
            # not a fixed 3 degrees: s = (d - target) / (-dd/ds), dd/ds = n . (a x (p - c)). The joints may open
            # (a pressed segment) down to a holding finger's least bend
            room_open = {}
            for j in hinge:
                if fi == "thumb":
                    cur = tr.get("thumb_" + j, [0.0])[0]
                    room_open[j] = max(0.0, cur - HOLD_DIGIT_LIMITS["thumb_" + j][0])
                else:
                    cur = fr["%s_%s" % (fi, j)][0]
                    room_open[j] = max(0.0, cur - HOLD_DIGIT_LIMITS[j][0])

            def query(X):
                D, _w, Nr = self.sdf_query(X)
                D = np.array(D, dtype=float)
                Nr = np.array(Nr, dtype=float)
                if oh is not None:
                    for i_, q in enumerate(X):
                        v = Vector(q)
                        l2, n2, _t2, d2 = oh[0].find_nearest(v, 0.03)
                        if l2 is None:
                            continue
                        out_ = v - l2
                        sd2 = (-d2 if out_.dot(n2) < 0.0 else d2) * 100.0
                        if sd2 < D[i_]:
                            D[i_] = sd2
                            nv = out_.normalized() if out_.length > 1e-9 else n2
                            if sd2 < 0.0:
                                nv = -nv
                            Nr[i_] = list(nv)
                return D, Nr
            js_ = [j for j in ("01", "02", "03") if j in hinge]
            self._cnt("fingers solved")
            closed, touching, n_round, slog = CS.chain_close(
                {j: pts[j] for j in ("01", "02", "03")}, hd, hinge,
                {j: room[j] for j in js_}, room_open, {j: rate[j] for j in js_}, query,
                touch_cm=touch_cm, target_cm=0.0, press_cm=press_cm,
                inner={j: inner_by[(fi, j)] for j in ("01", "02", "03") if (fi, j) in inner_by})
            for _ in range(n_round):
                self._cnt("closing rounds")
            # the bones turned once, by the angles found
            for j, d_ang in closed.items():
                if abs(d_ang) < 0.05:
                    continue
                if fi == "thumb":
                    if self.R.get("thumb_hinge") == "ray":
                        self._thumb_flex(side, j, d_ang)
                    else:
                        t2 = self._thumb_report(side, TL)[0]
                        b2, b3 = t2.get("thumb_02", [0, 0])[0], t2.get("thumb_03", [0, 0])[0]
                        self._set_thumb_bend(side, b2 + (d_ang if j == "02" else 0.0), b3 + (d_ang if j == "03" else 0.0), TL)
                else:
                    cur = self._finger_report(side, FL)[0]["%s_%s" % (fi, j)][0]
                    self._set_curl(side, fi, j, cur + d_ang, FL)
            out[fi] = {"closed_deg": {j: round(v, 1) for j, v in closed.items()}, "touching": sorted(touching),
                       "rounds": n_round, "inner_points": sum(len(inner_by[(fi, j)][0]) for j in ("01", "02", "03") if (fi, j) in inner_by)}
        bpy.context.view_layer.update()
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

    # --- the fast contact read the purposes, the gate and the search use ---------------------------------------
    def _skin_static(self):
        """What does not change with the pose, once: each skin vertex's heaviest bone, the triangles (numpy), and
        per vertex its side (0 none, 1 r, 2 l), its kind (0 none, 1 forearm, 2 palm, 3 digit) and its digit
        segment (0..14: thumb, index, middle, ring, pinky, each 01, 02, 03)."""
        c = getattr(self, "_skin_st", None)
        if c is not None:
            return c
        ob = self.meshes[0]
        names = {g.index: g.name for g in ob.vertex_groups}
        bone = []
        for v0 in ob.data.vertices:
            gm = max(v0.groups, key=lambda x: x.weight) if v0.groups else None
            bone.append(names.get(gm.group, "") if gm else "")
        tris = []
        for p_ in ob.data.polygons:
            vs = list(p_.vertices)
            for k in range(1, len(vs) - 1):
                tris.append((vs[0], vs[k], vs[k + 1]))
        n = len(bone)
        side = np.zeros(n, dtype=np.int8)
        kind = np.zeros(n, dtype=np.int8)
        segi = np.full(n, -1, dtype=np.int16)
        for i, b in enumerate(bone):
            if b.endswith("_r"):
                side[i] = 1
            elif b.endswith("_l"):
                side[i] = 2
            else:
                continue
            base = b[:-2]
            if base.startswith(("lowerarm", "forearm")) or "twist" in base:
                kind[i] = 1
            elif base.startswith(("upperarm", "clavicle")):
                kind[i] = 0
            else:
                fi = next((x for x in DIGITS if base.startswith(x)), None)
                if fi and "metacarpal" not in base:
                    j = base.split("_")[1] if "_" in base else "01"
                    kind[i] = 3
                    segi[i] = DIGITS.index(fi) * 3 + max(0, min(2, int(j[-1]) - 1))
                else:
                    kind[i] = 2
        self._skin_st = (bone, np.array(tris, dtype=np.int64), side, kind, segi)
        return self._skin_st

    def _pose_key(self):
        """Every arm bone's location, turn and scale, and the arms' place: the skin's shape is a function of these."""
        k = [tuple(self.arm.matrix_world.translation)]
        for pb in self.arm.pose.bones:
            k.append((tuple(pb.location), tuple(pb.rotation_quaternion), tuple(pb.scale)))
        return hash(tuple(k))

    def _skin_arrays(self):
        """The skin as posed, read in bulk: world points, normals, each vertex's area (cm2). Kept for the pose it
        was read in (v0.9.0): a wall check and the aim's error read the same pose, and reading the deformed mesh is
        most of a skin read's cost. The arrays are read only, so no caller can change the kept copy."""
        key = self._pose_key()
        c = getattr(self, "_skin_cache", None)
        if c is not None and c[0] == key:
            return c[1]
        out = self._skin_arrays_read()
        for a_ in out[:3]:
            a_.flags.writeable = False
        self._skin_cache = (key, out)
        return out

    def _skin_arrays_read(self):
        ob = self.meshes[0]
        bone, T = self._skin_static()[:2]
        dg = bpy.context.evaluated_depsgraph_get()
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        n = len(me.vertices)
        co = np.empty(n * 3)
        me.vertices.foreach_get("co", co)
        nr = np.empty(n * 3)
        me.vertices.foreach_get("normal", nr)
        ev.to_mesh_clear()
        M = np.array(ev.matrix_world)
        P = co.reshape(n, 3) @ M[:3, :3].T + M[:3, 3]
        N = nr.reshape(n, 3) @ M[:3, :3].T
        N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
        a = 0.5 * np.linalg.norm(np.cross(P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]]), axis=1) * 1e4
        A = np.zeros(n)
        for k in range(3):
            np.add.at(A, T[:, k], a / 3.0)
        return P, N, A, bone, T

    def _object_now(self):
        """The object as posed: its BVH, its signed-distance surface and the labelled samples in the world (a KD
        tree), cached while the object does not move."""
        key = self._gun_key()
        c = getattr(self, "_obj_now", None)
        if c and c[0] == key:
            return c[1]
        vs, ps = self._gun_verts()
        bvh = BVHTree.FromPolygons(vs, ps)
        SW, SL = self._samples_world()
        kd = KDTree(len(SW))
        for i, p_ in enumerate(SW):
            kd.insert(Vector(p_), i)
        kd.balance()
        val = (bvh, self.Surface(bvh), SW, SL, kd)
        self._obj_now = (key, val)
        return val

    def _region_index(self):
        """Per object pose: each region's samples (indices), its sample count, and a KD tree per region."""
        bvh, surf, SW, SL, kd = self._object_now()
        key = self._gun_key()
        c = getattr(self, "_reg_idx", None)
        if c and c[0] == key:
            return c[1]
        by = {}
        for k, l_ in enumerate(SL):
            by.setdefault(l_, []).append(k)
        kds = {}
        for l_, ks in by.items():
            t_ = KDTree(len(ks))
            for k in ks:
                t_.insert(Vector(SW[k]), k)
            t_.balance()
            kds[l_] = t_
        val = {"by": by, "kd": kds}
        self._reg_idx = (key, val)
        return val

    def _other_hand_bvh(self, other, P, N):
        """The other hand's skin as a BVH, rebuilt only when it has moved (a search moves one hand at a time)."""
        _bone, T, sidev, kind, _segi = self._skin_static()
        sc_ = 1 if other == "r" else 2
        sel = (sidev == sc_) & (kind > 0)
        vi = np.nonzero(sel)[0]
        key = tuple(np.round(P[vi[:: max(1, len(vi) // 24)]].ravel() * 1e4).astype(np.int64).tolist())
        c = getattr(self, "_ohb", {}).get(other)
        if c and c[0] == key:
            return c[1]
        m = sel[T[:, 0]] & sel[T[:, 1]] & sel[T[:, 2]]
        ts = T[m]
        if not len(ts):
            return None
        bvh = BVHTree.FromPolygons([Vector(x) for x in P[vi]], [tuple(t) for t in np.searchsorted(vi, ts)])
        oi, ol = self._hand_label_arrays(other, P, N)
        olab = dict(zip(oi.tolist(), ol))
        val = (bvh, ts, olab, P[vi].min(0) - 0.01, P[vi].max(0) + 0.01)
        if not hasattr(self, "_ohb"):
            self._ohb = {}
        self._ohb[other] = (key, val)
        return val

    def _pierced_samples(self, side, Ph, th, polys, SW, labs, whole=False):
        """The object's samples inside this hand's flesh: {sample: (depth cm, the hand label there)}.

        Step E (v0.10.0): only the part that moved is tested again. The last answer is kept per hand with the skin
        it was read on; a sample can change only when skin near it moved, so the samples within a 2 cm grid cell
        (grown by one cell) of a moved vertex, at its old place or its new one, are tested again, and every other
        sample keeps its answer. A point in the flesh is within half the palm's thickness (1.5 cm) of a skin vertex,
        so the grown cells hold every sample whose answer can change. The object moving, or whole, tests all."""
        cell = 0.02
        key = lambda c: (c[:, 0] * 4096 + c[:, 1]) * 4096 + c[:, 2]   # noqa: E731  a cell as one integer
        offs = np.array([(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)])
        lo_h, hi_h = Ph.min(0) - 0.01, Ph.max(0) + 0.01
        cache = getattr(self, "_pierce_cache", {})
        pc = None if whole else cache.get(side)
        gk = (self._gun_key(), len(SW), round(float(SW.sum()), 6))      # a part that moves (a slide) moves its samples
        if pc is not None and (pc["gun"] != gk or len(pc["Ph"]) != len(Ph)):
            pc = None
        if pc is None:
            near_v = Ph
            keep = {}
        else:
            mv = np.any(Ph != pc["Ph"], axis=1)             # any move at all: a sample on the skin flips on a hair
            if not mv.any():
                self._cnt("pierce: nothing moved")
                return dict(pc["res"])
            near_v = np.concatenate([Ph[mv], pc["Ph"][mv]])
            keep = None
        base = np.floor(np.minimum(lo_h, near_v.min(0)) / cell).astype(np.int64) - 2     # cells compared within a call
        inbox = np.nonzero(np.all((SW >= np.minimum(lo_h, near_v.min(0) - 0.01)) & (SW <= np.maximum(hi_h, near_v.max(0) + 0.01)), axis=1))[0]
        if len(inbox):
            hc = np.floor(near_v / cell).astype(np.int64) - base
            cells = np.unique(key((hc[:, None, :] + offs[None, :, :]).reshape(-1, 3)))
            sc = np.floor(SW[inbox] / cell).astype(np.int64) - base
            test = inbox[np.isin(key(sc), cells)]
        else:
            test = inbox
        if keep is None:
            tset = set(test.tolist())
            keep = {k: v for k, v in pc["res"].items() if k not in tset}
            self._cnt("pierce: part tested")
        else:
            self._cnt("pierce: whole tested")
        res = dict(keep)
        if len(test):
            hb = BVHTree.FromPolygons(Ph.tolist(), polys)
            for k in test:
                hit = self.inside_flesh(Vector(SW[k]), hb)
                if hit is None:
                    continue
                d_cm, t3 = hit
                res[int(k)] = (d_cm, labs.get(int(th[t3][0]), "palm"), int(t3))
        if not whole:
            cache[side] = {"gun": gk, "Ph": Ph.copy(), "res": res}
            self._pierce_cache = cache
        return dict(res)

    def contact_pairs(self, side, other=True, band_cm=0.2, near_cm=1.0, facing_max_deg=70.0, skin_give_cm=0.3,
                      cover_regions=None, clip_bound=None):
        """One hand's contact as posed now, quickly: {(hand part, region): [contact cm2, press cm2, deepest cm]},
        the totals (contact, press into the object, press into the other hand past the skin's give), and the share
        of each of cover_regions this hand's skin touches. The same reading the contact map makes, for the
        purposes, the gate and the search. Only skin near the object is tested; inside is the nearest face's side,
        confirmed by ray parity before a press counts."""
        P, N, A, bone, T = self._skin_arrays()
        bvh, surf, SW, SL, kd = self._object_now()
        RI = self._region_index()
        idx, labl = self._hand_label_arrays(side, P, N)
        labs = dict(zip(idx.tolist(), labl))
        vs, _ps = self._gun_verts()
        lo = np.min(np.array([list(v) for v in vs[:: max(1, len(vs) // 400)]]), 0) - near_cm / 100.0 - 0.01
        hi = np.max(np.array([list(v) for v in vs[:: max(1, len(vs) // 400)]]), 0) + near_cm / 100.0 + 0.01
        Pi = P[idx]
        near = np.all((Pi >= lo) & (Pi <= hi), axis=1)
        pairs, press, deep = {}, 0.0, 0.0
        cos_f = math.cos(math.radians(facing_max_deg))
        touched = set()
        r_cov = max(band_cm, 0.35) / 100.0
        st_ = self.steps
        st_.begin("contact: object side")
        ks = np.nonzero(near)[0]
        if len(ks):
            vi_n = idx[ks]
            Dn, _wp, Nn = self.sdf_query(P[vi_n])           # the grids: distance (cm), outward normal, at once
            # a gap near its press limit is measured on the mesh itself (the grids can be up to 0.112 cm off)
            giv_ = np.array([SKIN_GIVE.get(_lab_key(labs[int(i)]), SKIN_GIVE["default"]) for i in vi_n])
            Dn = self.exact_gaps(P[vi_n], Dn, -giv_ - SDF_TOL_CM)
        for kk, k in enumerate(ks):
            i = int(idx[k])
            d_cm = float(Dn[kk])
            if d_cm > near_cm:
                continue
            nrm = Vector(Nn[kk])
            loc = Vector(P[i]) - nrm * (d_cm / 100.0)     # the nearest surface point
            _c, si, _dd = kd.find(loc)
            reg = SL[si] if si is not None else "other"
            pr = pairs.setdefault((labs[i], reg), [0.0, 0.0, 0.0])
            give = SKIN_GIVE.get(_lab_key(labs[i]), SKIN_GIVE["default"])
            if d_cm < -give - SDF_TOL_CM:                   # past the give by more than the grid can tell
                pr[1] += A[i]
                pr[2] = max(pr[2], -d_cm - give)
                press += A[i]
                deep = max(deep, -d_cm - give)
            if d_cm <= band_cm and (d_cm < 0.0 or -float(np.dot(N[i], nrm)) >= cos_f):
                pr[0] += A[i]
            if cover_regions and d_cm * 0.01 <= r_cov:
                for (_co, sk, _d) in kd.find_range(loc, r_cov):
                    touched.add(sk)
        st_.end()
        st_.begin("contact: other hand")
        hand_press, hand_deep = 0.0, 0.0
        o = "r" if side == "l" else "l"
        if other:
            oh = self._other_hand_bvh(o, P, N)
            if oh is not None:
                ob_bvh, ts, olab, olo, ohi = oh
                nearo = np.all((Pi >= olo) & (Pi <= ohi), axis=1)
                _b2, _T2, sidev, kind, _s2 = self._skin_static()
                vi_o = np.nonzero((sidev == (1 if o == "r" else 2)) & (kind > 0))[0]
                for k in np.nonzero(nearo)[0]:
                    i = int(idx[k])
                    p_ = Vector(P[i])
                    l2, n2, ti, d2 = ob_bvh.find_nearest(p_, near_cm / 100.0)
                    if l2 is None:
                        continue
                    dd = (-d2 if (p_ - l2).dot(n2) < 0.0 and d2 < 0.02 else d2) * 100.0
                    pr = pairs.setdefault((labs[i], "hand_%s:%s" % (o, olab.get(int(ts[ti][0]), "?"))), [0.0, 0.0, 0.0])
                    if dd < -skin_give_cm:
                        pr[1] += A[i]
                        pr[2] = max(pr[2], -dd - skin_give_cm)
                        hand_press += A[i]
                        hand_deep = max(hand_deep, -dd - skin_give_cm)
                    if dd <= band_cm:
                        pr[0] += A[i]
        # the other way round: the object's surface inside the hand's flesh past its give. A thin part (a trigger
        # guard's ring) can pass through a finger between its skin vertices; its own surface samples cannot
        st_.end()
        st_.begin("contact: pierced test")
        pierced, pierce_deep, pierce_at = 0.0, 0.0, None
        # v0.9.0: clip_bound (area cm2, depth cm) is a wall the caller already knows this pose must stay within. The
        # pierced test only adds to the clipping, so when the skin's side and the other hand already go past it the
        # answer is known: the test is skipped and the result says so (pierced_skipped).
        skip_ = clip_bound is not None and (press + hand_press > clip_bound[0] or max(deep, hand_deep) > clip_bound[1])
        _bone, _T, sidev, kind, segi = self._skin_static()
        sc_ = 1 if side == "r" else 2
        # v0.9.0: the hand's vertices and triangles are the same every call (only their places move), so they are
        # worked out once per hand; the tree takes plain lists, not one Vector per vertex; and only the object's
        # samples near the skin are tested: a point in the flesh is within half the palm's thickness (1.5 cm) of a
        # skin vertex, and a 2 cm grid of the skin grown by one cell keeps every sample within 2 cm, found in numpy
        # instead of testing the hand's whole box.
        hk = getattr(self, "_pierce_static", {})
        if side not in hk:
            selh = (sidev == sc_) & (kind > 0)
            th_ = T[selh[T[:, 0]] & selh[T[:, 1]] & selh[T[:, 2]]]
            vi_ = np.nonzero(selh)[0]
            hk[side] = (th_, vi_, [tuple(int(x) for x in t) for t in np.searchsorted(vi_, th_)])
            self._pierce_static = hk
        th, vi, polys = hk[side]
        pierced_by = {}
        if len(th) and not skip_:
            Ph = P[vi]
            res = self._pierced_samples(side, Ph, th, polys, SW, labs)
            if os.environ.get("POSELAB_PIERCE_CHECK"):
                # Step E's check: the moved-part answer against the whole test, every call (slow; for measuring)
                full = self._pierced_samples(side, Ph, th, polys, SW, labs, whole=True)
                self._cnt("pierce check: calls")
                if set(full) != set(res) or any(abs(full[k][0] - res[k][0]) > 1e-6 for k in full if k in res):
                    self._cnt("pierce check: different")
                    self._cnt("pierce check: samples different", len(set(full) ^ set(res)))
                    a_s_ = SAMPLE_CM * SAMPLE_CM
                    give_ = lambda l_: SKIN_GIVE.get(_lab_key(l_), SKIN_GIVE["default"])   # noqa: E731
                    af = sum(a_s_ for v in full.values() if v[0] > give_(v[1]) + 0.02)
                    ar = sum(a_s_ for v in res.values() if v[0] > give_(v[1]) + 0.02)
                    self._cnt("pierce check: clip area off (0.01 cm2)", int(round(abs(af - ar) * 100)))
            a_s = SAMPLE_CM * SAMPLE_CM
            for k, (d_cm, lab3, _t3) in res.items():
                give = SKIN_GIVE.get(_lab_key(lab3), SKIN_GIVE["default"])
                if d_cm > give + 0.02:
                    pierced += a_s
                    pb_ = pierced_by.setdefault(lab3, [0.0, 0.0])
                    pb_[0] += a_s
                    pb_[1] = max(pb_[1], d_cm - give)
                    if d_cm - give > pierce_deep:
                        pierce_deep, pierce_at = d_cm - give, lab3
        st_.end()
        st_.begin("contact: cover")
        cover = {}
        if cover_regions:
            hit_all, tot_all = 0, 0
            for l_ in cover_regions:
                ks = RI["by"].get(l_, [])
                if not ks:
                    continue
                h_ = sum(1 for k in ks if k in touched)
                cover[l_] = round(h_ / float(len(ks)), 3)
                hit_all += h_
                tot_all += len(ks)
            cover["_all"] = round(hit_all / float(max(1, tot_all)), 3)
        st_.end()
        return {"pairs": pairs, "press_cm2": round(press, 2), "deepest_cm": round(deep, 2),
                "hand_press_cm2": round(hand_press, 2), "hand_deepest_cm": round(hand_deep, 2), "cover": cover,
                "pierced_cm2": round(pierced, 2), "pierced_deep_cm": round(pierce_deep, 2), "pierced_at": pierce_at,
                "pierced_by": {k: (round(v[0], 2), round(v[1], 2)) for k, v in pierced_by.items()},
                "pierced_skipped": bool(skip_),
                "_P": P, "_idx": idx, "_lab": labl, "_touched": touched}

    def span_cover(self, cp, mark, regions):
        """The share of the run's surface the hand covers, over the stretch of the run the hand spans (along its
        axis, between the hand's palm and digit skin), not the whole run: a long handle is not held less for being
        long. regions: the run's region names. Returns the share."""
        run = self._object["parts"][mark["part"]]["runs"][mark.get("run", 0)]
        RW = self._run_world(run)
        ax = np.array(list(RW["axis"]))
        o = np.array(list(RW["inner"]))
        P, idx, lab = cp["_P"], cp["_idx"], cp["_lab"]
        hand = [int(i) for i, l_ in zip(idx, lab) if l_ != "forearm"]
        if not hand:
            return 0.0
        h = (P[hand] - o) @ ax
        # the hand's span along the run: where its skin lies within 3 cm of the run's line
        radv = (P[hand] - o) - np.outer(h, ax)
        near = np.linalg.norm(radv, axis=1) <= 0.03 + max(s_["diameter_cm"] for s_ in run["sections"]) / 200.0
        if not near.any():
            return 0.0
        t0, t1 = float(h[near].min()), float(h[near].max())
        SW, SL = self._object_now()[2:4]
        want = set(regions)
        hs = (SW - o) @ ax
        tot = hit = 0
        tch = cp.get("_touched", set())
        for k in np.nonzero((hs >= t0) & (hs <= t1))[0]:
            if SL[k] in want:
                tot += 1
                hit += 1 if int(k) in tch else 0
        return round(hit / float(max(1, tot)), 3)

    def pull_cm(self, cp, hand_parts, regions):
        """How far a hand part is from where it should touch: the least distance (cm) from up to 40 of its skin
        vertices to any sample of the regions. The search's pull toward an unmet contact."""
        import fnmatch
        RI = self._region_index()
        P, idx, lab = cp["_P"], cp["_idx"], cp["_lab"]
        want = sorted({l_ for l_ in set(lab) if any(fnmatch.fnmatch(l_, h) for h in hand_parts)})
        if not want:
            return 0.0
        ws = set(want)
        vi = [int(i) for i, l_ in zip(idx, lab) if l_ in ws]
        vi = vi[:: max(1, len(vi) // 40)]
        best = None
        for rg in RI["kd"]:
            if not any(fnmatch.fnmatch(rg, r) for r in regions):
                continue
            t_ = RI["kd"][rg]
            for i in vi:
                _c, _k, d = t_.find(Vector(P[i]))
                if d is not None and (best is None or d < best):
                    best = d
        return round((best or 0.0) * 100.0, 2)

    def _sides_named(self, side):
        """The region names of this hand's own side of the run and the far side (a right hand's own side is the
        object's right), when the regions have left and right; else either side."""
        names = {l_ for l_ in (getattr(self, "_samples", None) and [x[3] for x in self._samples] or [])}
        if "left" in names or "right" in names:
            return ("right", "left") if side == "r" else ("left", "right")
        return ("side_?", "side_?")

    @staticmethod
    def _short(got, need):
        return max(0.0, need - got) / need if need > 0 else 0.0

    def contact_purposes(self, side, marks, glove_cm=0.0, other=True):
        """Every purpose the marks give this hand, judged on its contact pairs (one reading of contact_pairs):
        [{purpose, met, error, the areas measured, the areas asked}], with the press check and the comfort."""
        C = CONTACT_PURPOSES
        own, far = self._sides_named(side)
        cover_r = ["back", "front", own, far, "tang"] if own != "side_?" else ["back", "front", "side_a", "side_b", "tang", "end"]
        cp = self.contact_pairs(side, other=other, cover_regions=cover_r)
        pa = self.pair_area
        res = []
        for m in marks:
            if m.get("role") != "hold" or m.get("hand", side) != side:
                continue
            if m["purpose"] == "power_wrap":
                q = C["power_wrap"]
                # a part with a palm side (a grip: its marks say where the palm goes, its sides are named) asks
                # for the palm on its back and own side, the fingers round the front to the far side; a part
                # with none (a round handle) asks for palm and fingers anywhere on the run
                named = bool(m.get("palm_dir")) and own != "side_?"
                RUN = ["back", "front", "side_?", "left", "right", "tang", "end"]
                pr_ = ["back", own, "tang"] if named else RUN
                fr_, ff_ = (["front"], [far]) if named else (RUN, RUN)
                got = {"palm_cm2": pa(cp, ["palm", "palm_side"], pr_),
                       "fingers_front_cm2": pa(cp, FINGERS3, fr_),
                       "fingers_far_cm2": pa(cp, FINGERS3, ff_) if named else 0.0,
                       "web_cm2": pa(cp, ["web"], ["tang", "back"] if named else RUN),
                       "cover": self.span_cover(cp, m, cover_r)}
                if not named:
                    got["fingers_far_cm2"] = q["fingers_far_cm2"]      # one finger area for a round handle
                err = (self._short(got["palm_cm2"], q["palm_cm2"]) + self._short(got["fingers_front_cm2"], q["fingers_front_cm2"])
                       + self._short(got["fingers_far_cm2"], q["fingers_far_cm2"]) + self._short(got["web_cm2"], q["web_cm2"])
                       + 2.0 * self._short(got["cover"], q["cover_min"]))
                met = err <= 1e-6
                # the pull: each contact still short, how far its hand part is from its region (cm), so a hand
                # off the object always costs more the farther it is
                pull = 0.0
                for key, hp, rg in (("palm_cm2", ["palm"], pr_), ("fingers_front_cm2", FINGERS3, fr_),
                                    ("fingers_far_cm2", FINGERS3, ff_), ("web_cm2", ["web"], ["tang", "back"] if named else RUN)):
                    if got[key] < q[key]:
                        pull += min(10.0, self.pull_cm(cp, hp, rg))
                err += PULL_W * pull
                got["pull_cm"] = round(pull, 2)
                res.append({"purpose": "power_wrap", "met": met, "error": round(err, 3), **got,
                            "asks": {k: v for k, v in q.items() if k not in ("why", "estimate")}})
                if m.get("index") == "extended" and self._object["parts"][m["part"]]["runs"][m.get("run", 0)]["openings"]:
                    g_ = self.purpose_index_extended(side, m, glove_cm)
                    on_nt = pa(cp, ["index*"], ["notouch:*"])
                    g_["notouch_cm2"] = on_nt
                    g_["error"] = round(g_["error"] + (1.0 if on_nt > 0 else 0.0), 3)
                    g_["met"] = g_["error"] <= 1e-6
                    res.append(g_)
                if m.get("thumb") == "inline":
                    res.append(self._thumb_contact(side, m, glove_cm, cp, [far, "tang", "back", "opening_edge"], None))
                elif m.get("thumb") in ("wrap", "opposed"):
                    # the thumb closed round the handle the other way from the fingers (the GRASP taxonomy's power
                    # wrap with the thumb opposed): its pad or side on the handle
                    q2 = CONTACT_PURPOSES["thumb_wrap"]
                    got2 = pa(cp, ["thumb?_pad", "thumb?_side"], pr_ if not named else RUN)
                    e2 = self._short(got2, q2["pad_cm2"])
                    met2 = e2 <= 1e-6
                    if not met2:
                        e2 += PULL_W * min(10.0, self.pull_cm(cp, ["thumb?_pad"], RUN))
                    res.append({"purpose": "thumb_wrap", "met": met2, "error": round(e2, 3), "pad_cm2": got2,
                                "asks": {"pad_cm2": q2["pad_cm2"]}})
            elif m["purpose"] == "support":
                q = C["support"]
                o = "r" if side == "l" else "l"
                got = {"palm_cm2": pa(cp, ["palm", "palm_side"], [own]),
                       "over_cm2": pa(cp, ["index?_pad", "middle?_pad", "ring?_pad", "pinky?_pad"],
                                      ["hand_%s:*_back" % o, "hand_%s:*_side" % o]),
                       "cover": cp["cover"].get(own, 0.0)}
                err = (self._short(got["palm_cm2"], q["palm_cm2"]) + self._short(got["over_cm2"], q["over_cm2"])
                       + 2.0 * self._short(got["cover"], q["cover_min"]))
                met = err <= 1e-6
                pull = min(10.0, self.pull_cm(cp, ["palm"], [own])) if got["palm_cm2"] < q["palm_cm2"] else 0.0
                err += PULL_W * pull
                got["pull_cm"] = round(pull, 2)
                res.append({"purpose": "support", "met": met, "error": round(err, 3), **got,
                            "asks": {k: v for k, v in q.items() if k not in ("why", "estimate")}})
                if m.get("thumb") == "inline":
                    res.append(self._thumb_contact(side, m, glove_cm, cp, [own, "hand_%s:thumb*" % o, "tang"], self._thumb_dir(o)))
        nt_c = pa(cp, ["*"], ["notouch:*"])
        nt_p = pa(cp, ["*"], ["notouch:*"], 1)
        res.append({"purpose": "notouch", "met": nt_c <= 0.0 and nt_p <= 0.0, "error": round(nt_c + 5.0 * nt_p, 3),
                    "contact_cm2": nt_c, "press_cm2": nt_p})
        # no clipping, ever: no skin inside the object past its give, no object inside the flesh past it, no hand
        # inside the other past the skin's give (a hair of sampling noise allowed, no depth)
        clip_a = cp["press_cm2"] + cp["pierced_cm2"] + cp["hand_press_cm2"]
        clip_d = max(cp["deepest_cm"], cp["pierced_deep_cm"], cp["hand_deepest_cm"])
        perr = max(0.0, clip_a - CLIP_NOISE_CM2) + 10.0 * clip_d
        res.append({"purpose": "press", "met": clip_a <= CLIP_NOISE_CM2 and clip_d <= 0.0, "error": round(perr, 3),
                    "hard": True, "object_cm2": cp["press_cm2"], "object_deepest_cm": cp["deepest_cm"],
                    "pierced_cm2": cp["pierced_cm2"], "pierced_deep_cm": cp["pierced_deep_cm"], "pierced_at": cp["pierced_at"],
                    "hand_cm2": cp["hand_press_cm2"], "hand_deepest_cm": cp["hand_deepest_cm"]})
        res.append(self.hold_digit_rule(side))
        res.append(self.purpose_comfort(side))
        return res

    @staticmethod
    def inside_flesh(q, skin_bvh, reach_m=0.03):
        """A point inside a closed skin (its BVH): (how deep, cm; the nearest triangle) or None when outside. Inside
        is the nearest face's back side (a skin's faces point out)."""
        loc, nrm, ti, d = skin_bvh.find_nearest(q, reach_m)
        if loc is None or (q - loc).dot(nrm) >= 0.0:
            return None
        return d * 100.0, ti

    def hold_digit_rule(self, side):
        """A holding hand's digits as a real one's: no joint bent backward against the grip, no finger twisted out of
        its plane past what a grasp shows (HOLD_DIGIT_LIMITS, tighter than a free hand's ranges). A hard rule."""
        FL = self._limits()[1]
        TL = self._thumb_limits()
        fr = self._finger_report(side, FL)[0]
        tr = self._thumb_report(side, TL)[0]
        bad = []
        for fi in ("index", "middle", "ring", "pinky"):
            for j in ("01", "02", "03"):
                curl, twist = fr["%s_%s" % (fi, j)]
                lo, tw = HOLD_DIGIT_LIMITS[j]
                if curl < lo:
                    bad.append("%s %s bent %d backward (a holding finger: at most %d)" % (fi, j, -curl, -lo))
                if abs(twist) > tw:
                    bad.append("%s %s twisted %d out of its plane (a holding finger: at most %d)" % (fi, j, abs(twist), tw))
        for j in ("02", "03"):
            if "thumb_" + j not in tr:
                continue
            bend, twist = tr["thumb_" + j]
            lo, tw = HOLD_DIGIT_LIMITS["thumb_" + j]
            if bend < lo:
                bad.append("thumb %s bent %d backward (at most %d)" % (j, -bend, -lo))
            if abs(twist) > tw:
                bad.append("thumb %s twisted %d out of its plane (at most %d)" % (j, abs(twist), tw))
        return {"purpose": "digits", "met": not bad, "error": float(len(bad)), "hard": True, "faults": bad}

    def _thumb_contact(self, side, mark, glove_cm, cp, regions, along):
        """The thumb in line (its direction and end joint, geometric) and its pad's contact on regions."""
        g_ = self.purpose_thumb_inline(side, mark, glove_cm, along=along)
        pad = self.pair_area(cp, ["thumb?_pad"], regions)
        q = CONTACT_PURPOSES["thumb_inline"]
        g_.pop("gap_cm", None)
        g_.pop("gap_band_cm", None)
        g_["pad_cm2"] = pad
        g_["pad_asks_cm2"] = q["pad_cm2"]
        TL = self._thumb_limits()
        tdir = self._thumb_dir(side)
        ref = along if along is not None else self._index_dir(side)
        ang = math.degrees(tdir.angle(ref)) if tdir.length and ref.length else 180.0
        end = abs(self._thumb_report(side, TL)[0].get("thumb_03", [0, 0])[0])
        from holdlab import PURPOSES as _GP
        G = _GP["thumb_inline"]
        err = max(0.0, ang - G["angle_max_deg"]) / 10.0 + max(0.0, end - G["end_bend_max_deg"]) / 10.0 + self._short(pad, q["pad_cm2"])
        g_["met"] = err <= 1e-6
        obj_regions = [r for r in regions if not r.startswith("hand_")]
        if pad < q["pad_cm2"] and obj_regions:
            err += PULL_W * min(10.0, self.pull_cm(cp, ["thumb?_pad"], obj_regions))
        g_["error"] = round(err, 3)
        return g_

    @staticmethod
    def pair_area(cp, hand_parts, regions, what=0):
        """The area (cm2) of a contact_pairs reading whose hand part matches any of hand_parts and whose region
        any of regions (fnmatch patterns); what 0 contact, 1 press."""
        import fnmatch
        tot = 0.0
        for (hp, rg), v in cp["pairs"].items():
            if any(fnmatch.fnmatch(hp, h) for h in hand_parts) and any(fnmatch.fnmatch(rg, r) for r in regions):
                tot += v[what]
        return round(tot, 2)

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
