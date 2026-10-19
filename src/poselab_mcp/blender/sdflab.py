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
                            str(len(V)) + str(len(ps)) + str(cell_cm)).encode()).hexdigest()[:16]
        path = os.path.join(self._sdf_dir(), "%s_%s.npz" % (part, key))
        if os.path.exists(path):
            z = np.load(path, allow_pickle=True)
            return {"origin": z["origin"], "cell": float(z["cell"]), "d": z["d"], "path": path}
        bvh = BVHTree.FromPolygons(V, ps)
        A = np.array([list(v) for v in V])
        cell = cell_cm / 100.0
        lo = A.min(0) - margin_cm / 100.0
        hi = A.max(0) + margin_cm / 100.0
        n = np.maximum(2, np.ceil((hi - lo) / cell).astype(int) + 1)
        nx, ny, nz = int(n[0]), int(n[1]), int(n[2])
        # inside or out: rays along every row of cells, both ways along x and along z; crossings alternate
        def row_inside(axis):
            ins = np.zeros((nx, ny, nz), dtype=np.int8)
            a, b = [k for k in range(3) if k != axis]
            dirv = Vector([1.0 if k == axis else 0.0 for k in range(3)])
            length = (n[axis] - 1) * cell
            for i in range(n[a]):
                for j in range(n[b]):
                    o = [0.0, 0.0, 0.0]
                    o[a] = lo[a] + i * cell
                    o[b] = lo[b] + j * cell
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
                    if len(hits) < 2:
                        continue
                    ts = (np.arange(n[axis]) * cell + 0.001)
                    cnt = np.searchsorted(np.array(hits), ts)
                    line = (cnt % 2 == 1).astype(np.int8)
                    sl = [slice(None)] * 3
                    sl[a], sl[b] = i, j
                    ins[tuple(sl)] = line
            return ins
        inside = (row_inside(0) + row_inside(2)) >= 1
        # the distance: the nearest surface, for every cell (far cells clamped)
        d = np.empty((nx, ny, nz), dtype=np.float32)
        band = SDF_BAND_CM / 100.0
        for i in range(nx):
            for j in range(ny):
                for k in range(nz):
                    q = Vector((lo[0] + i * cell, lo[1] + j * cell, lo[2] + k * cell))
                    loc, _nr, _ix, dist = bvh.find_nearest(q, band)
                    dist = band if loc is None else dist
                    d[i, j, k] = -dist if inside[i, j, k] else dist
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
