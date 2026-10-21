"""The object as distance grids (0.9.0): each part's signed distance to its surface on a grid in the part's own frame,
built once and kept on disk, so "how far is this point from the object, inside or out, on which region, and which way
is out" is an array lookup for every skin vertex at once instead of one ray query each.

A part is rigid in its own frame (a gun's frame, its slide, its magazine), so its grid never changes; a part that
moves takes its grid with it. The object is the union of its parts: the nearest part's distance wins. Inside or out is
found by rays along every row of cells (a closed surface's crossings alternate), two ways, the face normal settling
ties; the distance is the nearest surface's. Cells near the surface also keep the region label of the nearest
labelled sample (contactlab's object regions)."""
import hashlib
import json
import math
import os

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

SDF_CELL_CM = 0.2
SDF_MARGIN_CM = 2.5
SDF_BAND_CM = 3.0          # distances past this are clamped (far is far)
SDF_TOL_CM = 0.03          # the grid's error near the surface (examples/sdf_tests.py measures it); the press rule
                           # does not count skin sunk less than this past its give
SDF_VERSION = 3            # 3: each closed shell on its own (the union's distance: the least over the shells), 2026-10-21


def shells(polys):
    """The mesh's separate closed shells (polygons joined through shared vertices): [list of polygon indices]. A game
    gun is many shells, some inside others (the magazine in the grip): counting ray crossings over all of them at
    once reads a point inside two shells as outside, so inside or out is read shell by shell."""
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for p in polys:
        r0 = find(p[0])
        for v in p[1:]:
            r1 = find(v)
            if r1 != r0:
                parent[r1] = r0
    groups = {}
    for i, p in enumerate(polys):
        groups.setdefault(find(p[0]), []).append(i)
    return list(groups.values())


RAYS6 = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0.577, 0.577, 0.577), (-0.577, 0.577, -0.577))


def shell_signed(bvhs, p):
    """The exact signed distance (m, inside below 0) from p to a union of closed shells (their BVHs): each shell's
    distance to its nearest face, inside it by ray parity in six directions (the majority); the union's is the
    least of them."""
    p = Vector(list(p))
    best = None
    for bvh in bvhs:
        _loc, _n, _i, dist = bvh.find_nearest(p)
        if dist is None:
            continue
        votes = 0
        for d in RAYS6:
            dv = Vector(d)
            o, hits = p.copy(), 0
            for _ in range(64):
                h = bvh.ray_cast(o, dv, 1.0)
                if h[0] is None:
                    break
                hits += 1
                o = h[0] + dv * 1e-6
            votes += hits % 2
        s = -dist if votes * 2 > len(RAYS6) else dist
        best = s if best is None else min(best, s)
    return best


def _trilinear(G, idx):
    """G (nx, ny, nz) at fractional cell coordinates idx (m x 3), clamped to the grid."""
    nx, ny, nz = G.shape
    f = np.clip(idx, 0.0, [nx - 1.001, ny - 1.001, nz - 1.001])
    i0 = np.floor(f).astype(np.int64)
    t = f - i0
    i1 = i0 + 1
    out = np.zeros(len(f))
    for dx in (0, 1):
        wx = t[:, 0] if dx else 1.0 - t[:, 0]
        ix = i1[:, 0] if dx else i0[:, 0]
        for dy in (0, 1):
            wy = t[:, 1] if dy else 1.0 - t[:, 1]
            iy = i1[:, 1] if dy else i0[:, 1]
            for dz in (0, 1):
                wz = t[:, 2] if dz else 1.0 - t[:, 2]
                iz = i1[:, 2] if dz else i0[:, 2]
                out += wx * wy * wz * G[ix, iy, iz]
    return out


class SdfMixin:
    def _sdf_dir(self):
        d = os.path.join(self.out_dir, "sdf")
        os.makedirs(d, exist_ok=True)
        return d

    def _part_mesh_local(self, part):
        """A part's polygons in its own frame (vertices, polygons), parts at rest."""
        dg = bpy.context.evaluated_depsgraph_get()
        me_ob = self.rifle_mesh
        ev = me_ob.evaluated_get(dg)
        me = ev.to_mesh()
        Fi = self._obj_frame(part).inverted() @ me_ob.matrix_world
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
            if not any(vg) or own == part:
                ps.append(tuple(p.vertices))
        ev.to_mesh_clear()
        return V, ps

    def build_sdf(self, part, cell_cm=SDF_CELL_CM, margin_cm=SDF_MARGIN_CM):
        """A part's signed distance grid in its own frame (m; inside below 0), and its region labels near the
        surface. Kept on disk under the part's mesh's hash; built once."""
        V, ps = self._part_mesh_local(part)
        if not ps:
            return None
        key = hashlib.sha1((part + json.dumps([[round(c, 5) for c in v] for v in V[:: max(1, len(V) // 500)]]) +
                            str(len(V)) + str(len(ps)) + str(cell_cm) + "v%d" % SDF_VERSION).encode()).hexdigest()[:16]
        path = os.path.join(self._sdf_dir(), "%s_%s.npz" % (part, key))
        if os.path.exists(path):
            z = np.load(path, allow_pickle=True)
            return {"origin": z["origin"], "cell": float(z["cell"]), "d": z["d"], "path": path}
        A = np.array([list(v) for v in V])
        cell = cell_cm / 100.0
        lo = A.min(0) - margin_cm / 100.0
        hi = A.max(0) + margin_cm / 100.0
        n = np.maximum(2, np.ceil((hi - lo) / cell).astype(int) + 1)
        nx, ny, nz = int(n[0]), int(n[1]), int(n[2])
        band = SDF_BAND_CM / 100.0
        # v3: each closed shell on its own. Inside a shell: a ray along every row of cells on each of the three axes,
        # each cell reading its parity both ways (crossings before it, after it): inside when at least 4 of the 6
        # votes say so. The shell's signed distance: its nearest face's distance, by that sign. The part's: the
        # least over its shells (a union of solids). (v1 counted crossings over all shells at once and read a point
        # inside two overlapping shells as outside: 15% of the points near the M1911's grip wrong, up to 2.26 cm)
        d = np.full((nx, ny, nz), band, dtype=np.float32)
        for sh in shells(ps):
            sps = [ps[i] for i in sh]
            bvh = BVHTree.FromPolygons(V, sps)
            Vs = A[sorted({v for p in sps for v in p})]
            slo = np.maximum(0, np.floor((Vs.min(0) - band - lo) / cell).astype(int))
            shi = np.minimum(n - 1, np.ceil((Vs.max(0) + band - lo) / cell).astype(int))

            def row_inside(axis, bvh=bvh, slo=slo, shi=shi):
                ins = np.zeros((nx, ny, nz), dtype=np.int8)
                a_, b_ = [k for k in range(3) if k != axis]
                dirv = Vector([1.0 if k == axis else 0.0 for k in range(3)])
                length = (n[axis] - 1) * cell
                for i in range(slo[a_], shi[a_] + 1):
                    for j in range(slo[b_], shi[b_] + 1):
                        o = [0.0, 0.0, 0.0]
                        o[a_] = lo[a_] + i * cell
                        o[b_] = lo[b_] + j * cell
                        o[axis] = lo[axis] - 0.001
                        o = Vector(o)
                        hits, t0 = [], 0.0
                        for _ in range(64):
                            h = bvh.ray_cast(o + dirv * t0, dirv, length + 0.002 - t0)
                            if h[0] is None:
                                break
                            t_hit = (h[0] - o).dot(dirv)
                            hits.append(t_hit)
                            t0 = t_hit + 1e-5
                        if not hits:
                            continue
                        ts = (np.arange(n[axis]) * cell + 0.001)
                        cnt = np.searchsorted(np.array(hits), ts)
                        line = (cnt % 2 == 1).astype(np.int8) + ((len(hits) - cnt) % 2 == 1).astype(np.int8)
                        sl = [slice(None)] * 3
                        sl[a_], sl[b_] = i, j
                        ins[tuple(sl)] = line
                return ins
            inside = (row_inside(0) + row_inside(1) + row_inside(2)) >= 4
            for i in range(slo[0], shi[0] + 1):
                for j in range(slo[1], shi[1] + 1):
                    for k in range(slo[2], shi[2] + 1):
                        q = Vector((lo[0] + i * cell, lo[1] + j * cell, lo[2] + k * cell))
                        loc, _nr, _ix, dist = bvh.find_nearest(q, band)
                        if loc is None:
                            continue
                        s_ = -dist if inside[i, j, k] else dist
                        if s_ < d[i, j, k]:
                            d[i, j, k] = s_
        np.savez_compressed(path, origin=lo, cell=cell, d=d)
        return {"origin": lo, "cell": cell, "d": d, "path": path}

    def object_sdf(self, parts=None):
        """Every part's grid, built once (or read from disk)."""
        c = getattr(self, "_sdfs", None)
        if c is not None:
            return c
        names = set()
        for g in self.rifle_mesh.vertex_groups:
            names.add(g.name)
        names = sorted(names) or ["body"]
        out = {}
        for p in (parts or names):
            g = self.build_sdf(p)
            if g is not None:
                out[p] = g
        self._sdfs = out
        return out

    def sdf_query(self, P, want_normal=True, parts=None):
        """Signed distance (cm, inside below 0) from world points P (n x 3) to the object as posed now (the nearest
        part wins), each point's nearest part, and with want_normal the outward normal (world, the grid's gradient)."""
        S = self.object_sdf()
        best = np.full(len(P), 99.0)
        which = np.full(len(P), "", dtype=object)
        nrm = np.zeros((len(P), 3)) if want_normal else None
        for part, g in S.items():
            if parts and part not in parts:
                continue
            F = self._obj_frame(part)
            Fi = np.array(F.inverted())
            R_, T_ = Fi[:3, :3], Fi[:3, 3]
            L_ = P @ R_.T + T_
            idx = (L_ - g["origin"]) / g["cell"]
            dd = _trilinear(g["d"], idx) * 100.0
            near = dd < best
            best[near] = dd[near]
            which[near] = part
            if want_normal and near.any():
                h = 0.5
                gx = (_trilinear(g["d"], idx[near] + [h, 0, 0]) - _trilinear(g["d"], idx[near] - [h, 0, 0]))
                gy = (_trilinear(g["d"], idx[near] + [0, h, 0]) - _trilinear(g["d"], idx[near] - [0, h, 0]))
                gz = (_trilinear(g["d"], idx[near] + [0, 0, h]) - _trilinear(g["d"], idx[near] - [0, 0, h]))
                gl = np.c_[gx, gy, gz]
                gl /= np.maximum(np.linalg.norm(gl, axis=1, keepdims=True), 1e-12)
                R3 = np.array(F.to_3x3())
                nrm[near] = gl @ R3.T
        return best, which, nrm
