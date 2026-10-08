# Pose Lab, inside Blender: one first-person rig (arms on their idle pose, a rifle on the gun bone, clips) loaded once,
# and measured answers about it. Every position goes in and comes out in a named frame, so no caller guesses axes:
#   gun   the gun bone as it stands now, Unreal-style axes, cm: +X the gun's left, +Y along the barrel, +Z up
#   arms  the arms' space, Unreal-style axes, cm (the eye sits at the rig's "eye")
#   view  the player's view from the eye, cm: +X right, +Y forward, +Z up
# Turns use the player's words: roll + turns the gun's right side up, swing + the muzzle left, pitch + the muzzle up.
# worker.py runs it; poselab_mcp.server exposes it as MCP tools.
import fnmatch
import json
import math
import os
import random

import bpy
from mathutils import Matrix, Quaternion, Vector
from mathutils.kdtree import KDTree

FINGERS = ("index", "middle", "ring", "pinky", "thumb")
RADIUS = {"forearm": 0.030, "palm": 0.018, "finger": 0.0075}       # m: the capsules a body part is tested with
DEFAULT_POLES = {"l": [45.0, -20.0, 95.0], "r": [-45.0, -20.0, 95.0]}  # arms frame, cm: where each elbow points
# bone names: the Unreal mannequin's by default; a rig's "bones" entry renames any of them
DEFAULT_BONES = {"gun": "ik_hand_gun", "upperarm": "upperarm_{s}", "lowerarm": "lowerarm_{s}", "hand": "hand_{s}",
                 "finger": "{f}_{j}_{s}"}

# the view: the eye looks along +Y (Blender -Y), up is +Z, the player's right is -X
VIEW_RIGHT, VIEW_FWD, VIEW_UP = Vector((-1.0, 0.0, 0.0)), Vector((0.0, -1.0, 0.0)), Vector((0.0, 0.0, 1.0))
H_FOV, ASPECT = 90.0, 16.0 / 9.0
PUBLIC = ("load_rig", "describe", "list_rigs", "pose_idle", "pose_clip", "move_part", "move_gun", "reach", "snapshot",
          "record_clip", "load_clip", "scan_clip", "fix_clip", "save_clip",
          "where", "distance", "clearance", "faces_eye", "visible", "screen", "solve", "render")


def ue(v):
    """Unreal-style cm (x, y, z) to Blender metres."""
    return Vector((v[0], -v[1], v[2])) / 100.0


def ue_back(v):
    return Vector((v[0], -v[1], v[2])) * 100.0


def r2(v, n=2):
    return [round(c, n) for c in v]


def fit(src, dst):
    """The rotation, uniform scale and move that best carries the points src onto dst (Umeyama), as a matrix."""
    import numpy as np
    A, B = np.array([tuple(p) for p in src]), np.array([tuple(p) for p in dst])
    ca, cb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rm = Vt.T @ D @ U.T
    var = ((A - ca) ** 2).sum()
    k = (S * np.diag(D)).sum() / var if var > 0 else 1.0
    t = cb - k * Rm @ ca
    return Matrix([[Rm[i, 0] * k, Rm[i, 1] * k, Rm[i, 2] * k, t[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]])


class Lab:
    def __init__(self, rigs_file, out_dir):
        self.rigs_file = rigs_file
        self.out_dir = os.path.realpath(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.rig = None
        self.snapshots = {}

    # --- where it may write ----------------------------------------------------------------------------------
    def _own(self, path):
        """Pose Lab writes only inside its output folder: never the rigs, the models or anything else."""
        full = os.path.realpath(path)
        if os.path.commonpath([os.path.normcase(self.out_dir), os.path.normcase(full)]) != os.path.normcase(self.out_dir):
            raise PermissionError("Pose Lab writes only under %s, not %s" % (self.out_dir, path))
        return full

    # --- rigs -------------------------------------------------------------------------------------------------
    def _rigs(self):
        rigs = {"sample": {"title": "the built-in sample: simple arms and a rifle made in Blender (free to share)", "builtin": True}}
        if self.rigs_file and os.path.exists(self.rigs_file):
            for k, v in json.load(open(self.rigs_file)).items():
                if not k.startswith("_"):
                    rigs[k] = v
        return rigs

    def list_rigs(self):
        """The rigs: the built-in sample and those in the rigs file (POSELAB_RIGS)."""
        return {"rigs_file": self.rigs_file or None, "rigs": {k: v.get("title", "") for k, v in self._rigs().items()}}

    def load_rig(self, rig):
        """Loads a rig: the arms on their idle pose, the rifle on the gun bone, the rig's clips."""
        rigs = self._rigs()
        if rig not in rigs:
            raise ValueError("no rig %r; rigs: %s" % (rig, list(rigs)))
        R = dict(rigs[rig])
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.sc = bpy.context.scene
        self.names = dict(DEFAULT_BONES, **R.get("bones", {}))
        if R.get("builtin"):
            from sample import build
            made = build()
            self.arm, self.meshes, self.rifle_arm, self.rifle_mesh = made["arm"], made["meshes"], made["rifle_arm"], made["rifle_mesh"]
            R.update(made["config"])
        else:
            self._load_files(R)
        for pb in self.arm.pose.bones:
            pb.rotation_mode = 'QUATERNION'
        self.idle = {pb.name: (pb.location.copy(), pb.rotation_quaternion.copy(), pb.scale.copy()) for pb in self.arm.pose.bones}
        if self.arm.animation_data:
            self.arm.animation_data.action = None
        self.R, self.rig = R, rig
        self.G0 = (self.arm.matrix_world @ self.arm.pose.bones[self.names["gun"]].matrix).copy()
        self.rifle_rest = Matrix.Translation(self.G0.translation + ue(R.get("gun_offset", (0, 0, 0)))) @ self.rifle_arm.matrix_world
        lower = {b.name.lower(): b.name for b in self.rifle_arm.pose.bones}
        self.part_bones = {p: [lower[b.lower()] for b in bones if b.lower() in lower] for p, bones in R.get("parts", {}).items()}
        self.travel = {}
        self.poles = dict(DEFAULT_POLES, **R.get("poles", {}))
        self.eye = ue(R.get("eye", (0.0, 0.0, 160.0)))
        if not R.get("builtin"):
            self._load_clips(R)
        else:
            self.clips, self.clip_fit = {}, {}
        self.sc.render.fps, self.sc.render.fps_base = 30, 1.0
        self.snapshots = {}
        self.pose_idle()
        return self.describe()

    def _load_files(self, R):
        folder = os.path.join(os.path.dirname(os.path.abspath(self.rigs_file)), R["folder"])
        self.folder = folder
        bpy.ops.import_scene.fbx(filepath=os.path.join(folder, R["arms"]))
        old = [o for o in bpy.data.objects if o.type == 'ARMATURE'][0]
        mesh = [o for o in bpy.data.objects if o.type == 'MESH'][0]
        if R.get("idle"):
            # the idle clip's armature drives the mesh (a mesh file's own armature may have a bad rest)
            before = set(bpy.data.objects.keys())
            bpy.ops.import_scene.fbx(filepath=os.path.join(folder, R["idle"]))
            self.arm = [o for o in bpy.data.objects if o.type == 'ARMATURE' and o.name not in before][0]
            for m in mesh.modifiers:
                if m.type == 'ARMATURE':
                    m.object = self.arm
            mw = mesh.matrix_world.copy()
            mesh.parent = self.arm
            mesh.matrix_world = mw
            bpy.data.objects.remove(old)
            act = self.arm.animation_data.action
            self.sc.frame_set(int(act.frame_range[0]))
            bpy.context.view_layer.update()
        else:
            self.arm = old
        self.meshes = [mesh]
        before = set(bpy.data.objects.keys())
        bpy.ops.import_scene.fbx(filepath=os.path.join(folder, R["gun"]))
        new = [o for o in bpy.data.objects if o.name not in before]
        self.rifle_arm = [o for o in new if o.type == 'ARMATURE'][0]
        self.rifle_mesh = [o for o in new if o.type == 'MESH'][0]

    def _load_clips(self, R):
        # bone data (<clip>.pose.json, the same idle armature) is taken as it is; an FBX clip sits on its own hidden
        # armature and is carried over in world terms: ours = T @ clip @ C[bone], from the two rest poses
        self.clips, self.clip_fit = {}, {}
        rest = {b.name: self.arm.matrix_world @ b.matrix_local for b in self.arm.data.bones}
        for name, f in R.get("clips", {}).items():
            path = os.path.join(self.folder, f)
            if f.endswith(".pose.json"):
                data = json.load(open(path))
                self.clips[name] = ("json", data["frames"], float(data["fps"]), None, None)
                self.clip_fit[name] = {"source": "bone data"}
                continue
            before = set(bpy.data.objects.keys())
            bpy.ops.import_scene.fbx(filepath=path)
            fps = self.sc.render.fps / self.sc.render.fps_base
            for o in [o for o in bpy.data.objects if o.name not in before]:
                if o.type == 'ARMATURE' and o.animation_data and o.animation_data.action and name not in self.clips:
                    o.hide_render = True     # still evaluated (a viewport-hidden armature does not animate)
                    crest = {b.name: o.matrix_world @ b.matrix_local for b in o.data.bones}
                    shared = [b.name for b in self.arm.data.bones if b.name in crest]
                    T = fit([crest[b].translation for b in shared], [rest[b].translation for b in shared])
                    C = {b: crest[b].inverted() @ T.inverted() @ rest[b] for b in shared}
                    worst = max(((T @ crest[b]).translation - rest[b].translation).length for b in shared) * 100.0
                    self.clips[name] = (o, o.animation_data.action, fps, T, C)
                    # a large mismatch means the clip's skeleton is not this one (or Blender misread an FBX it wrote)
                    self.clip_fit[name] = {"source": "fbx", "rest_mismatch_cm": round(worst, 3)}
                else:
                    bpy.data.objects.remove(o)

    def describe(self):
        """What is loaded and the conventions every tool uses."""
        self._need()
        return {
            "rig": self.rig, "title": self.R.get("title"),
            "frames": {"gun": "the gun bone as it stands now, Unreal-style axes, cm: +X gun's left, +Y along the barrel, +Z up",
                       "arms": "the arms' space, Unreal-style axes, cm",
                       "view": "from the eye, cm: +X right, +Y forward, +Z up"},
            "turns": "roll + the gun's right side up; swing + the muzzle left; pitch + the muzzle up (degrees)",
            "points": self.R.get("points", {}), "normals": self.R.get("normals", {}),
            "clips": {k: {"seconds": round(self._clip_len(k), 3), **self.clip_fit.get(k, {})} for k in self.clips},
            "parts": self.part_bones, "eye_arms_cm": list(self.R.get("eye", (0.0, 0.0, 160.0))),
            "body_names": "bones by name, fingertips as <finger>_tip_<l|r>, rig points by name",
            "writes_only_to": self.out_dir,
        }

    def _need(self):
        if not self.rig:
            raise RuntimeError("no rig loaded: call load_rig first (list_rigs shows them)")

    # --- names and frames -------------------------------------------------------------------------------------
    def _n(self, part, side=None, f=None, j=None):
        return self.names[part].format(s=side, f=f, j=j)

    def _bw(self, n):
        return self.arm.matrix_world @ self.arm.pose.bones[n].matrix

    def _bone(self, b):
        """A bone by its rig name, or "gun" for the rig's gun bone; a clear error for any other name."""
        if b == "gun":
            return self.names["gun"]
        if b not in self.arm.pose.bones:
            raise ValueError("no bone %r; use \"gun\" or one of: %s" % (b, ", ".join(sorted(x.name for x in self.arm.pose.bones))))
        return b

    def _gun(self):
        return self._bw(self.names["gun"]) @ self.G0.inverted() @ Matrix.Translation(self.G0.translation)

    def _to(self, p, frame):
        if frame == "gun":
            return ue_back(self._gun().inverted() @ p)
        if frame == "arms":
            return ue_back(p)
        if frame == "view":
            d = (p - self.eye) * 100.0
            return Vector((d.dot(VIEW_RIGHT), d.dot(VIEW_FWD), d.dot(VIEW_UP)))
        raise ValueError("frame must be gun, arms or view")

    def _from(self, v, frame):
        v = Vector(v)
        if frame == "gun":
            return self._gun() @ ue(v)
        if frame == "arms":
            return ue(v)
        if frame == "view":
            return self.eye + (VIEW_RIGHT * v.x + VIEW_FWD * v.y + VIEW_UP * v.z) / 100.0
        raise ValueError("frame must be gun, arms or view")

    def _tip(self, f, side):
        return self.arm.matrix_world @ self.arm.pose.bones[self._n("finger", side, f, "03")].tail

    def _point(self, p, frame="gun"):
        if isinstance(p, str):
            pts = self.R.get("points", {})
            if p in pts:
                return self._gun() @ ue(pts[p])
            if "_tip_" in p:
                f, side = p.split("_tip_")
                return self._tip(f, side)
            if p in self.arm.pose.bones:
                return self._bw(p).translation
            raise ValueError("unknown point %r" % p)
        return self._from(p, frame)

    # --- posing -----------------------------------------------------------------------------------------------
    def _unit(self):
        """The scale every posed bone keeps: the armature's own. IK rounding must not build up into a stretch."""
        return self.arm.matrix_world.to_scale()

    def _set_world(self, n, M):
        self.arm.pose.bones[n].matrix = self.arm.matrix_world.inverted() @ M
        bpy.context.view_layer.update()

    def _turn(self, n, q):
        M = self._bw(n)
        R = Matrix.Translation(M.translation) @ q.to_matrix().to_4x4() @ Matrix.Translation(-M.translation) @ M
        self._set_world(n, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))

    def _two_bone(self, side, goal, pole=None, pole_world=None):
        upper, lower, hand = self._n("upperarm", side), self._n("lowerarm", side), self._n("hand", side)
        pole = pole_world if pole_world is not None else ue(pole or self.poles[side])
        S, E, W = self._bw(upper).translation, self._bw(lower).translation, self._bw(hand).translation
        l1, l2 = (E - S).length, (W - E).length
        d = min((goal - S).length, l1 + l2 - 1e-4)
        axis = (goal - S).normalized()
        sidev = (pole - S) - axis * (pole - S).dot(axis)
        sidev = sidev.normalized() if sidev.length > 1e-6 else Vector((0, 0, -1))
        x = (l1 * l1 - l2 * l2 + d * d) / (2 * d)
        h = max(l1 * l1 - x * x, 0.0) ** 0.5
        E2, W2 = S + axis * x + sidev * h, S + axis * d
        self._turn(upper, (E - S).rotation_difference(E2 - S))
        E, W = self._bw(lower).translation, self._bw(hand).translation
        self._turn(lower, (W - E).rotation_difference(W2 - E))
        return (self._bw(hand).translation - goal).length * 100.0

    def _place_rifle(self):
        self.rifle_arm.matrix_world = self._bw(self.names["gun"]) @ self.G0.inverted() @ self.rifle_rest
        for bones in self.part_bones.values():
            for b in bones:
                self.rifle_arm.pose.bones[b].matrix_basis = Matrix.Identity(4)
        bpy.context.view_layer.update()
        q = self._gun().to_quaternion()
        moved = {}
        for part, cm in self.travel.items():
            for b in self.part_bones.get(part, []):
                moved[b] = max(moved.get(b, 0.0), cm)
        for b, cm in moved.items():
            pb = self.rifle_arm.pose.bones[b]
            M = self.rifle_arm.matrix_world @ pb.matrix
            pb.matrix = self.rifle_arm.matrix_world.inverted() @ (Matrix.Translation(q @ ue((0.0, -cm, 0.0))) @ M)
        bpy.context.view_layer.update()

    def pose_idle(self):
        """The arms on the idle pose, the rifle on the gun bone, every part home."""
        self._need()
        for pb in self.arm.pose.bones:
            l, r, s = self.idle[pb.name]
            pb.location, pb.rotation_quaternion, pb.scale = l, r, s
        self.travel = {}
        bpy.context.view_layer.update()
        self._place_rifle()
        return {"pose": "idle"}

    def _clip_len(self, clip):
        o, act, fps, T, C = self.clips[clip]
        return (len(act) - 1) / fps if o == "json" else (act.frame_range[1] - act.frame_range[0]) / fps

    def pose_clip(self, clip, seconds):
        """The arms as a clip has them at a time; the rifle follows the gun bone, its parts stay where they are."""
        self._need()
        if clip not in self.clips:
            raise ValueError("no clip %r; clips: %s" % (clip, list(self.clips)))
        o, act, fps, T, C = self.clips[clip]
        if o == "json":
            x = min(max(seconds * fps, 0.0), len(act) - 1)
            i = int(math.floor(x))
            j, k = min(i + 1, len(act) - 1), x - i
            for pb in self.arm.pose.bones:
                if pb.name in act[i]:
                    (l0, q0), (l1, q1) = act[i][pb.name], act[j][pb.name]
                    pb.location = Vector(l0).lerp(Vector(l1), k)
                    pb.rotation_quaternion = Quaternion(q0).slerp(Quaternion(q1), k)
            p0, p1 = act[i].get("_parts", {}), act[j].get("_parts", {})
            if p0 or p1:
                self.travel = {n: p0.get(n, 0.0) + (p1.get(n, 0.0) - p0.get(n, 0.0)) * k for n in set(p0) | set(p1)}
            bpy.context.view_layer.update()
        else:
            f = act.frame_range[0] + seconds * fps
            self.sc.frame_set(int(math.floor(f)), subframe=f % 1.0)
            want = {b: T @ (o.matrix_world @ o.pose.bones[b].matrix) @ C[b] for b in C}
            inv = self.arm.matrix_world.inverted()
            for pb in self.arm.pose.bones:          # parents before children
                if pb.name in want:
                    pb.matrix = inv @ want[pb.name]
                    bpy.context.view_layer.update()
        self._place_rifle()
        return {"clip": clip, "seconds": seconds, "length": round(self._clip_len(clip), 3)}

    def move_part(self, part, cm):
        """A moving part of the rifle drawn back along the barrel (cm from home)."""
        self._need()
        if part not in self.part_bones:
            raise ValueError("no part %r; parts: %s" % (part, list(self.part_bones)))
        self.travel[part] = float(cm)
        self._place_rifle()
        return {"part": part, "back_cm": cm}

    def move_gun(self, roll=0.0, swing=0.0, pitch=0.0, right=0.0, forward=0.0, up=0.0, pivot="stock", keep_hands=("l", "r")):
        """Moves the rifle from where it stands: roll about the bore, swing and pitch about the pivot, then a move in
        the view (cm). The hands in keep_hands keep their hold (arm IK)."""
        self._need()
        G = self._gun()
        held = {s: G.inverted() @ self._bw(self._n("hand", s)) for s in keep_hands}
        barrel = (G.to_quaternion() @ Vector((0.0, -1.0, 0.0))).normalized()
        bore = self._point("bore") if "bore" in self.R.get("points", {}) else G.translation
        pv = self._point(pivot) if isinstance(pivot, str) else self._from(pivot, "gun")
        lateral = barrel.cross(VIEW_UP).normalized()

        def about(point, axis, deg):
            return Matrix.Translation(point) @ Quaternion(axis, math.radians(deg)).to_matrix().to_4x4() @ Matrix.Translation(-point)

        M = Matrix.Translation((VIEW_RIGHT * right + VIEW_FWD * forward + VIEW_UP * up) / 100.0) \
            @ about(pv, VIEW_UP, swing) @ about(pv, lateral, pitch) @ about(bore, -barrel, roll)
        self._set_world(self.names["gun"], M @ self._bw(self.names["gun"]))
        self._place_rifle()
        miss = {}
        G = self._gun()
        for s, H in held.items():
            W = G @ H
            miss[s] = round(self._two_bone(s, W.translation), 2)
            hn = self._n("hand", s)
            self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, W.to_quaternion(), self._unit()))
        return {"hands_off_their_hold_cm": miss, "barrel_off_deg": round(self._barrel_off(), 2)}

    def reach(self, side, target, frame="gun", pole=None):
        """A wrist to a point by arm IK; the hand keeps its turn. pole: where the elbow points (arms frame, cm)."""
        self._need()
        hn = self._n("hand", side)
        H = self._bw(hn)
        miss = self._two_bone(side, self._point(target, frame), pole)
        self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, H.to_quaternion(), self._unit()))
        return {"wrist_off_target_cm": round(miss, 2)}

    def snapshot(self, action, name):
        """save or load a named pose (the arms' bones and the rifle's parts)."""
        self._need()
        if action == "save":
            self.snapshots[name] = ({pb.name: (pb.location.copy(), pb.rotation_quaternion.copy()) for pb in self.arm.pose.bones}, dict(self.travel))
            return {"saved": name}
        if name not in self.snapshots:
            raise ValueError("no snapshot %r; saved: %s" % (name, list(self.snapshots)))
        bones, travel = self.snapshots[name]
        for pb in self.arm.pose.bones:
            pb.location, pb.rotation_quaternion = bones[pb.name]
            pb.scale = self.idle[pb.name][2]
        self.travel = dict(travel)
        bpy.context.view_layer.update()
        self._place_rifle()
        return {"loaded": name}

    # --- measuring --------------------------------------------------------------------------------------------
    def where(self, names, frame="gun"):
        """Where bones, fingertips and rig points are, in the frame."""
        self._need()
        return {n: r2(self._to(self._point(n), frame), 2) for n in names}

    def distance(self, a, b, frame="gun"):
        """The distance between two points, cm, and b minus a in the frame."""
        self._need()
        pa, pb = self._point(a, frame), self._point(b, frame)
        return {"cm": round((pb - pa).length * 100.0, 2), "b_minus_a": r2(self._to(pb, frame) - self._to(pa, frame), 2)}

    def _segments(self, parts):
        out = []
        for side in ("l", "r"):
            if parts not in ("both", side, {"l": "left", "r": "right"}[side]):
                continue
            b = lambda part: self._bw(self._n(part, side)).translation
            f = lambda fi, j: self._bw(self._n("finger", side, fi, j)).translation
            out.append(("forearm_" + side, b("lowerarm"), b("hand"), RADIUS["forearm"]))
            out.append(("palm_" + side, b("hand"), f("middle", "01"), RADIUS["palm"]))
            for fi in FINGERS:
                pts = [f(fi, "01"), f(fi, "02"), f(fi, "03"), self._tip(fi, side)]
                for i in range(3):
                    out.append(("%s%d_%s" % (fi, i + 1, side), pts[i], pts[i + 1], RADIUS["finger"]))
        return out

    def _tree(self):
        dg = bpy.context.evaluated_depsgraph_get()
        ev = self.rifle_mesh.evaluated_get(dg)
        me = ev.to_mesh()
        pts = [self.rifle_mesh.matrix_world @ v.co for v in me.vertices]
        ev.to_mesh_clear()
        tree = KDTree(len(pts))
        for i, p in enumerate(pts):
            tree.insert(p, i)
        tree.balance()
        return tree

    def clearance(self, parts="both", ignore=(), frame="gun", top=5):
        """How deep the rifle sits inside the arms (capsules: forearm 3 cm, palm 1.8 cm, each finger segment 0.75 cm).
        worst_cm 0: nothing touches. ignore: segment names, prefixes or wildcards (thumb, index3_r, *_l)."""
        self._need()
        tree = self._tree()
        hits = {}
        for name, a, b, r in self._segments(parts):
            if any(name.startswith(i) or fnmatch.fnmatch(name, i) for i in ignore):
                continue
            n = max(2, int((b - a).length / 0.004))
            for i in range(n + 1):
                p = a.lerp(b, i / n)
                co, idx, d = tree.find(p)
                if co is not None and d < r and (r - d) * 100.0 > hits.get(name, (0.0,))[0]:
                    hits[name] = ((r - d) * 100.0, co.copy())
        ranked = sorted(hits.items(), key=lambda kv: -kv[1][0])
        return {"worst_cm": round(ranked[0][1][0], 2) if ranked else 0.0,
                "contacts": [{"segment": k, "depth_cm": round(v[0], 2), "rifle_point": r2(self._to(v[1], frame), 1)} for k, v in ranked[:top]]}

    def faces_eye(self, point="port", normal=None):
        """How squarely a surface faces the eye: 1 square on, 0 edge on, below 0 turned away."""
        self._need()
        p = self._point(point)
        n = normal if normal is not None else self.R.get("normals", {}).get(point)
        if n is None:
            raise ValueError("no normal for %r: give one in the gun frame" % (point,))
        nw = (self._gun().to_quaternion() @ ue(n)).normalized()
        to_eye = self.eye - p
        dot = nw.dot(to_eye.normalized())
        return {"facing": round(dot, 3), "angle_from_square_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, dot)))), 1),
                "distance_cm": round(to_eye.length * 100.0, 1)}

    def visible(self, point="port", radius_cm=0.6, rings=2):
        """The share of a small disc round a point the eye sees with nothing (arms, rifle) in the way."""
        self._need()
        target = self._point(point)
        d = (target - self.eye).normalized()
        u = d.cross(VIEW_UP)
        u = u.normalized() if u.length > 1e-6 else Vector((1, 0, 0))
        v = d.cross(u).normalized()
        samples = [target]
        for ring in range(1, rings + 1):
            rr = radius_cm / 100.0 * ring / rings
            for k in range(8):
                a = 2 * math.pi * k / 8
                samples.append(target + (u * math.cos(a) + v * math.sin(a)) * rr)
        dg = bpy.context.evaluated_depsgraph_get()
        seen = 0
        for s in samples:
            ray = s - self.eye
            hit, loc, nrm, idx, obj, mat = self.sc.ray_cast(dg, self.eye, ray.normalized(), distance=ray.length + 0.01)
            if not hit or (loc - s).length < 0.006 or (loc - self.eye).length >= ray.length - 0.006:
                seen += 1
        return {"visible": round(seen / len(samples), 3), "samples": len(samples)}

    def screen(self, point):
        """Where a point falls on a 90 degree, 16:9 screen from the eye: x, y from -1 to 1; on_screen if inside."""
        self._need()
        p = self._to(self._point(point), "view")
        if p.y <= 0.0:
            return {"on_screen": False, "behind_eye": True}
        t = math.tan(math.radians(H_FOV / 2))
        x, y = p.x / p.y / t, p.z / p.y / (t / ASPECT)
        return {"on_screen": abs(x) <= 1.0 and abs(y) <= 1.0, "x": round(x, 3), "y": round(y, 3)}

    def _barrel_off(self):
        barrel = self._gun().to_quaternion() @ Vector((0.0, -1.0, 0.0))
        return math.degrees(barrel.angle(VIEW_FWD))

    # --- solving ----------------------------------------------------------------------------------------------
    def _goal(self, g):
        """(met, value, shortfall >= 0) for one goal."""
        t = g["type"]
        if t == "faces_eye":
            v = self.faces_eye(g.get("point", "port"), g.get("normal"))["facing"]
            return v >= g.get("min", 0.5), v, max(0.0, g.get("min", 0.5) - v)
        if t == "visible":
            v = self.visible(g.get("point", "port"))["visible"]
            return v >= g.get("min", 0.5), v, max(0.0, g.get("min", 0.5) - v)
        if t == "on_screen":
            v = 1.0 if self.screen(g["point"]).get("on_screen") else 0.0
            return v > 0.5, v, 1.0 - v
        if t == "clearance":
            v = self.clearance(g.get("parts", "both"), g.get("ignore", ()))["worst_cm"]
            return v <= g.get("max_cm", 0.0), v, max(0.0, v - g.get("max_cm", 0.0)) / 2.0
        if t == "barrel":
            v = self._barrel_off()
            return v <= g.get("max_deg", 2.0), round(v, 2), max(0.0, v - g.get("max_deg", 2.0)) / 30.0
        if t in ("distance", "contact"):
            v = self.distance(g["a"], g["b"])["cm"]
            return v <= g.get("max_cm", 0.5), v, max(0.0, v - g.get("max_cm", 0.5)) / 5.0
        raise ValueError("goal type %r: faces_eye, visible, on_screen, clearance, barrel, distance, contact" % t)

    def solve(self, dofs, goals, keep_hands=("l", "r"), pivot="stock", samples=120, maximize=None, seed=1):
        """Searches rifle moves (dofs: roll, swing, pitch, right, forward, up -> [min, max]) from the current pose for
        one that meets every goal; reports each goal at the best and how often any sample met it (never met: a fact
        of the geometry, not a tuning miss). Leaves the scene at the best."""
        self._need()
        names = [k for k in dofs if k in ("roll", "swing", "pitch", "right", "forward", "up")]
        if not names:
            raise ValueError("dofs: name -> [min, max] for roll, swing, pitch, right, forward, up")
        self.snapshot("save", "_solve_base")
        rng = random.Random(seed)
        met_count = [0] * len(goals)
        all_count = 0

        def evaluate(x):
            self.snapshot("load", "_solve_base")
            self.move_gun(pivot=pivot, keep_hands=tuple(keep_hands), **dict(zip(names, x)))
            res = [self._goal(g) for g in goals]
            short = sum(r[2] for r in res)
            bonus = res[maximize][1] if maximize is not None and short == 0 else 0.0
            return short * 10.0 - bonus, res

        lo = [float(dofs[n][0]) for n in names]
        hi = [float(dofs[n][1]) for n in names]
        tried, pool = 0, []
        for i in range(samples):
            x = [0.0 if (i == 0 and l <= 0.0 <= h) else rng.uniform(l, h) for l, h in zip(lo, hi)]
            score, res = evaluate(x)
            tried += 1
            for k, r in enumerate(res):
                met_count[k] += 1 if r[0] else 0
            all_count += 1 if all(r[0] for r in res) else 0
            pool.append((score, x, res))
        # a pattern search from each of the six best samples (goals met in different corners rarely meet in one)
        best = None
        for start in sorted(pool, key=lambda p: p[0])[:6]:
            cur = start
            step = [(h - l) / 8.0 for l, h in zip(lo, hi)]
            for _ in range(4):
                improved = True
                while improved:
                    improved = False
                    for k in range(len(names)):
                        for sgn in (1, -1):
                            x = list(cur[1])
                            x[k] = min(hi[k], max(lo[k], x[k] + sgn * step[k]))
                            score, res = evaluate(x)
                            tried += 1
                            if score < cur[0] - 1e-6:
                                cur, improved = (score, x, res), True
                step = [s_ / 2.0 for s_ in step]
            if best is None or cur[0] < best[0]:
                best = cur
        self.snapshot("load", "_solve_base")
        self.move_gun(pivot=pivot, keep_hands=tuple(keep_hands), **dict(zip(names, best[1])))
        return {"all_met": all(r[0] for r in best[2]), "best": {n: round(v, 2) for n, v in zip(names, best[1])},
                "goals": [{"goal": g, "met": bool(r[0]), "value": r[1], "met_in_samples": "%d of %d" % (m, samples)}
                          for g, r, m in zip(goals, best[2], met_count)],
                "evaluations": tried, "all_met_in_samples": "%d of %d" % (all_count, samples), "never_met": [g for g, m in zip(goals, met_count) if m == 0]}

    # --- motion: record, load, scan, fix, save --------------------------------------------------------------
    # A clip here is frames of bone data on this rig's armature (each bone's local location and rotation, and the
    # rifle's parts), at a frame rate. Scanning plays it frame by frame and runs checks; fixing changes only what a
    # failing check needs and stores a new clip. Files go only to the output folder.
    def _capture(self):
        fr = {pb.name: [list(pb.location), list(pb.rotation_quaternion)] for pb in self.arm.pose.bones}
        if self.travel:
            fr["_parts"] = dict(self.travel)
        return fr

    def _apply(self, fr):
        for pb in self.arm.pose.bones:
            if pb.name in fr:
                pb.location, pb.rotation_quaternion = Vector(fr[pb.name][0]), Quaternion(fr[pb.name][1])
            pb.scale = self.idle[pb.name][2]
        self.travel = dict(fr.get("_parts", {}))
        bpy.context.view_layer.update()
        self._place_rifle()

    def _frames(self, clip):
        """The clip as bone data at its own frame rate: (frames, fps)."""
        if clip not in self.clips:
            raise ValueError("no clip %r; clips: %s" % (clip, list(self.clips)))
        o, act, fps, T, C = self.clips[clip]
        if o == "json":
            return [dict(f) for f in act], fps
        n = int(round(self._clip_len(clip) * fps)) + 1
        out = []
        for i in range(n):
            self.pose_clip(clip, i / fps)
            out.append(self._capture())
        return out, fps

    def _store(self, clip, frames, fps, source):
        self.clips[clip] = ("json", frames, float(fps), None, None)
        self.clip_fit[clip] = {"source": source}

    @staticmethod
    def _blend(a, b, x):
        fr = {}
        for name, (l0, q0) in a.items():
            if name == "_parts":
                continue
            l1, q1 = b.get(name, (l0, q0))
            fr[name] = [list(Vector(l0).lerp(Vector(l1), x)), list(Quaternion(q0).slerp(Quaternion(q1), x))]
        p0, p1 = a.get("_parts", {}), b.get("_parts", {})
        if p0 or p1:
            fr["_parts"] = {k: p0.get(k, 0.0) + (p1.get(k, 0.0) - p0.get(k, 0.0)) * x for k in set(p0) | set(p1)}
        return fr

    def record_clip(self, action, clip=None, seconds=0.0, fps=30.0, ease=True):
        """Builds a clip from poses: start (a name), key (the current pose at a time, s), stop (bakes the frames:
        each bone eased from key to key). Between keys the bones blend by rotation, so hands may drift off the rifle:
        scan_clip's hold check finds that and fix_clip mends it."""
        self._need()
        if action == "start":
            if not clip:
                raise ValueError("start needs a clip name")
            self._rec = {"clip": clip, "fps": float(fps), "keys": []}
            return {"recording": clip, "fps": fps}
        rec = getattr(self, "_rec", None)
        if not rec:
            raise RuntimeError("no recording: record_clip start first")
        if action == "key":
            rec["keys"] = [k for k in rec["keys"] if abs(k[0] - seconds) > 1e-6] + [(float(seconds), self._capture())]
            rec["keys"].sort(key=lambda k: k[0])
            return {"keys": [round(k[0], 3) for k in rec["keys"]]}
        if action != "stop":
            raise ValueError("action: start, key or stop")
        keys = rec["keys"]
        if not keys:
            raise ValueError("no keys recorded")
        fps = rec["fps"]
        n = int(round(keys[-1][0] * fps)) + 1
        frames = []
        for i in range(n):
            t = i / fps
            a = max([k for k in keys if k[0] <= t + 1e-9], key=lambda k: k[0], default=keys[0])
            b = min([k for k in keys if k[0] >= t - 1e-9], key=lambda k: k[0], default=keys[-1])
            x = 0.0 if b[0] <= a[0] else (t - a[0]) / (b[0] - a[0])
            frames.append(self._blend(a[1], b[1], x * x * (3 - 2 * x) if ease else x))
        self._store(rec["clip"], frames, fps, "recorded")
        self._rec = None
        return {"clip": rec["clip"], "frames": n, "seconds": round((n - 1) / fps, 3)}

    def load_clip(self, clip, path, bone_map=None):
        """Adds a clip from a file: <clip>.pose.json bone data, an FBX animation, or a BVH. A relative path is taken
        from the rigs file's folder. bone_map renames the file's bones to this rig's ({file bone: rig bone})."""
        self._need()
        base = os.path.dirname(os.path.abspath(self.rigs_file)) if self.rigs_file else os.getcwd()
        full = path if os.path.isabs(path) else os.path.join(base, path)
        if not os.path.exists(full):
            raise ValueError("no file %s" % full)
        if full.endswith(".json"):
            data = json.load(open(full))
            self._store(clip, data["frames"], data["fps"], "bone data")
            return {"clip": clip, "seconds": round(self._clip_len(clip), 3), "source": "bone data"}
        before = set(bpy.data.objects.keys())
        if full.lower().endswith(".bvh"):
            bpy.ops.import_anim.bvh(filepath=full, update_scene_fps=False, update_scene_duration=False)
        elif full.lower().endswith(".fbx"):
            bpy.ops.import_scene.fbx(filepath=full)
        else:
            raise ValueError("clips: .pose.json, .fbx or .bvh")
        fps = self.sc.render.fps / self.sc.render.fps_base
        rest = {b.name: self.arm.matrix_world @ b.matrix_local for b in self.arm.data.bones}
        made = None
        for o in [o for o in bpy.data.objects if o.name not in before]:
            if o.type == 'ARMATURE' and o.animation_data and o.animation_data.action and made is None:
                if bone_map:
                    for b in o.data.bones:
                        if b.name in bone_map:
                            b.name = bone_map[b.name]
                o.hide_render = True
                crest = {b.name: o.matrix_world @ b.matrix_local for b in o.data.bones}
                shared = [b.name for b in self.arm.data.bones if b.name in crest]
                if len(shared) < 3:
                    raise ValueError("the file's bones do not match this rig's: give a bone_map")
                T = fit([crest[b].translation for b in shared], [rest[b].translation for b in shared])
                C = {b: crest[b].inverted() @ T.inverted() @ rest[b] for b in shared}
                worst = max(((T @ crest[b]).translation - rest[b].translation).length for b in shared) * 100.0
                self.clips[clip] = (o, o.animation_data.action, fps, T, C)
                self.clip_fit[clip] = {"source": os.path.splitext(full)[1][1:], "bones": len(shared), "rest_mismatch_cm": round(worst, 3)}
                made = o
            else:
                bpy.data.objects.remove(o)
        self.sc.render.fps, self.sc.render.fps_base = 30, 1.0
        if made is None:
            raise ValueError("no animation in %s" % full)
        return {"clip": clip, "seconds": round(self._clip_len(clip), 3), **self.clip_fit[clip]}

    def _hand_in_gun(self, side):
        return self._gun().inverted() @ self._bw(self._n("hand", side))

    def _check(self, c, prev, refs):
        """(met, value) for one check on the pose as it stands; prev: bone positions a frame before (pops)."""
        t = c["type"]
        if t == "pop":
            if prev is None:
                return True, 0.0
            v = max((self._bw(b).translation - prev[b]).length * 100.0 * c["_fps"] for b in c.get("bones", ["hand_l", "hand_r"]))
            return v <= c.get("max_cm_per_s", 300.0), round(v, 1)
        if t == "hold":
            v = (self._hand_in_gun(c["side"]).translation - refs[c["side"]].translation).length * 100.0
            return v <= c.get("max_cm", 0.5), round(v, 2)
        met, v, _ = self._goal(c)
        return met, v

    def scan_clip(self, clip, checks):
        """Plays a clip frame by frame and runs each check: clearance, faces_eye, visible, on_screen, barrel, contact
        (a, b, max_cm), hold (side, max_cm: the hand's drift on the rifle from its grip at ref_s), pop (bones, "gun" for the gun bone,
        max_cm_per_s). Any check takes during: [from_s, to_s]. Reports, for each check, the worst value and when, and
        the times it fails."""
        self._need()
        frames, fps = self._frames(clip)
        return self._scan(frames, fps, checks, clip)

    def _scan(self, frames, fps, checks, name):
        checks = [dict(c, _fps=fps) for c in checks]
        for c in checks:
            if c["type"] == "pop":
                c["bones"] = [self._bone(b) for b in c.get("bones", ["hand_l", "hand_r"])]
        refs = {}
        for c in checks:
            if c["type"] == "hold":
                self._apply(frames[min(len(frames) - 1, int(round(c.get("ref_s", 0.0) * fps)))])
                refs[c["side"]] = self._hand_in_gun(c["side"])
        pop_bones = {b for c in checks if c["type"] == "pop" for b in c.get("bones", ["hand_l", "hand_r"])}
        per = [[] for _ in checks]
        prev = None
        for i, fr in enumerate(frames):
            t = i / fps
            self._apply(fr)
            for k, c in enumerate(checks):
                d = c.get("during")
                if d and not (d[0] - 1e-9 <= t <= d[1] + 1e-9):
                    continue
                met, v = self._check(c, prev, refs)
                per[k].append((t, met, v))
            prev = {b: self._bw(b).translation.copy() for b in pop_bones}
        report = []
        for c, rows in zip(checks, per):
            bad = [r for r in rows if not r[1]]
            higher_worse = c["type"] in ("clearance", "barrel", "distance", "contact", "hold", "pop")
            worst = (max if higher_worse else min)(rows, key=lambda r: r[2]) if rows else None
            spans = []
            for t, met, v in rows:
                if met:
                    continue
                if spans and t - spans[-1][1] <= 1.5 / fps:
                    spans[-1][1] = t
                else:
                    spans.append([t, t])
            report.append({"check": {k: v for k, v in c.items() if k != "_fps"}, "passed": not bad,
                           "worst": worst[2] if worst else None, "worst_at_s": round(worst[0], 3) if worst else None,
                           "failing_frames": len(bad), "failing_s": [[round(a, 3), round(b, 3)] for a, b in spans]})
        return {"clip": name, "frames": len(frames), "fps": fps, "all_passed": all(r["passed"] for r in report), "checks": report}

    def _swing_elbow(self, side, deg):
        """The elbow turned about the shoulder-wrist line by deg; the wrist and the hand stay where they are."""
        up, lo, hn = self._n("upperarm", side), self._n("lowerarm", side), self._n("hand", side)
        S, E, W = self._bw(up).translation, self._bw(lo).translation, self._bw(hn).translation
        H = self._bw(hn).copy()
        E2 = S + Quaternion((W - S).normalized(), math.radians(deg)) @ (E - S)
        self._two_bone(side, W, pole_world=E2)
        self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, H.to_quaternion(), self._unit()))

    def _put_hand(self, side, M):
        """The hand to M by arm IK; returns how far the target lies beyond the arm's reach (cm, 0 if reachable)."""
        hn = self._n("hand", side)
        up, lo = self._n("upperarm", side), self._n("lowerarm", side)
        S, E, W = self._bw(up).translation, self._bw(lo).translation, self._bw(hn).translation
        short = max(0.0, (M.translation - S).length - (E - S).length - (W - E).length) * 100.0
        self._two_bone(side, M.translation)
        self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, M.to_quaternion(), self._unit()))
        return short

    @staticmethod
    def _in(c, t):
        d = c.get("during")
        return not d or d[0] - 1e-9 <= t <= d[1] + 1e-9

    def fix_clip(self, clip, checks, out=None, max_swing_deg=90.0, spread_frames=4):
        """Mends a clip against the checks and stores the result as a new clip (out, default <clip>_fixed):
        pop: frames that jump are blended again from the good frames round them;
        hold: the hand goes back onto its grip on the rifle (arm IK);
        contact: the wrist moves until a point of the hand (a) touches its mark (b);
        clearance: each elbow swings about its shoulder-wrist line, the wrist kept, by the least angle that clears,
        spread over the frames round it so nothing pops.
        Reports the scan before and after; save_clip writes the result."""
        self._need()
        out = out or clip + "_fixed"
        frames, fps = self._frames(clip)
        before = self._scan(frames, fps, checks, clip)
        changed = set()
        limits = {}   # check -> the frames whose target lies out of the arm's reach, and how far

        def out_of_reach(c, i, short):
            if short > 0.05:
                key = "%s %s" % (c["type"], c.get("side") or c.get("a"))
                n, worst, first, last = limits.get(key, (0, 0.0, i, i))
                limits[key] = (n + 1, max(worst, short), min(first, i), max(last, i))
        # pops: the frames that jump are blended again between the nearest good frames
        for c in [c for c in checks if c["type"] == "pop"]:
            rep = self._scan(frames, fps, [c], clip)["checks"][0]
            bad = set()
            for a, b in rep["failing_s"]:
                for i in range(int(round(a * fps)), int(round(b * fps)) + 1):
                    bad.update((i - 1, i))
            bad = sorted(i for i in bad if 0 <= i < len(frames))
            good = [i for i in range(len(frames)) if i not in bad]
            for i in bad:
                lo_ = max([g for g in good if g < i], default=None)
                hi_ = min([g for g in good if g > i], default=None)
                if lo_ is not None and hi_ is not None:
                    frames[i] = self._blend(frames[lo_], frames[hi_], (i - lo_) / (hi_ - lo_))
                    changed.add(i)
        # holds: each frame's hand back onto its grip as at ref_s
        for c in [c for c in checks if c["type"] == "hold"]:
            self._apply(frames[min(len(frames) - 1, int(round(c.get("ref_s", 0.0) * fps)))])
            ref = self._hand_in_gun(c["side"])
            for i, fr in enumerate(frames):
                if not self._in(c, i / fps):
                    continue
                self._apply(fr)
                if (self._hand_in_gun(c["side"]).translation - ref.translation).length * 100.0 > c.get("max_cm", 0.5):
                    out_of_reach(c, i, self._put_hand(c["side"], self._gun() @ ref))
                    frames[i] = self._capture()
                    changed.add(i)
        # contacts: the wrist moves until the point of the hand touches its mark
        for c in [c for c in checks if c["type"] == "contact"]:
            side = c["a"][-1] if c["a"][-2:] in ("_l", "_r") else c.get("side", "l")
            for i, fr in enumerate(frames):
                if not self._in(c, i / fps):
                    continue
                self._apply(fr)
                if self.distance(c["a"], c["b"])["cm"] <= c.get("max_cm", 0.5):
                    continue
                short = 0.0
                for _ in range(8):
                    err = self._point(c["b"]) - self._point(c["a"])
                    if err.length * 100.0 <= c.get("max_cm", 0.5) * 0.5:
                        break
                    short = self._put_hand(side, Matrix.Translation(err) @ self._bw(self._n("hand", side)))
                out_of_reach(c, i, short)
                frames[i] = self._capture()
                changed.add(i)
        # clearance: the least elbow swing that clears, per frame and side, spread over the frames round it
        steps = [0.0] + [sg * a for a in range(10, int(max_swing_deg) + 1, 10) for sg in (1, -1)]
        for c in [c for c in checks if c["type"] == "clearance"]:
            for side in [s_ for s_ in ("l", "r") if c.get("parts", "both") in ("both", s_, {"l": "left", "r": "right"}[s_])]:
                part = {"l": "left", "r": "right"}[side]
                angle = [0.0] * len(frames)
                for i, fr in enumerate(frames):
                    if not self._in(c, i / fps):
                        continue
                    self._apply(fr)
                    if self.clearance(part, c.get("ignore", ()))["worst_cm"] <= c.get("max_cm", 0.0):
                        continue
                    best = None
                    for a in steps:
                        self._apply(fr)
                        self._swing_elbow(side, a)
                        v = self.clearance(part, c.get("ignore", ()))["worst_cm"]
                        if best is None or v < best[0] - 1e-6:
                            best = (v, a)
                        if v <= c.get("max_cm", 0.0):
                            break
                    angle[i] = best[1]
                spread = list(angle)
                for i, a in enumerate(angle):
                    if not a:
                        continue
                    for k in range(-spread_frames, spread_frames + 1):
                        j = i + k
                        if 0 <= j < len(frames):
                            w = a * (1.0 - smooth(0.0, spread_frames + 1.0, abs(k)))
                            if abs(w) > abs(spread[j]):
                                spread[j] = w
                for i, a in enumerate(spread):
                    if a:
                        self._apply(frames[i])
                        self._swing_elbow(side, a)
                        frames[i] = self._capture()
                        changed.add(i)
        self._store(out, frames, fps, "fixed from %s" % clip)
        after = self._scan(frames, fps, checks, out)
        brief = lambda r: [{"check": x["check"]["type"], "passed": x["passed"], "worst": x["worst"], "failing_frames": x["failing_frames"]} for x in r["checks"]]
        # what no fix can mend: a hand asked to be where the arm cannot reach (the pose itself must change)
        reach = [{"check": k, "frames": n, "from_s": round(a / fps, 3), "to_s": round(b / fps, 3), "worst_short_cm": round(w, 2)}
                 for k, (n, w, a, b) in limits.items()]
        return {"clip": out, "changed_frames": len(changed), "before": brief(before), "after": brief(after),
                "all_passed_after": after["all_passed"], "out_of_reach": reach, "after_detail": after["checks"]}

    def save_clip(self, clip, formats=("pose.json",), root_name=None):
        """Writes a clip to the output folder's clips/: pose.json (bone data) and fbx (an armature animation for a
        game engine). root_name renames the armature object while it exports (Unreal reads the top node as the
        skeleton's root bone: give the root bone's name)."""
        self._need()
        frames, fps = self._frames(clip)
        folder = self._own(os.path.join(self.out_dir, "clips"))
        os.makedirs(folder, exist_ok=True)
        written = []
        if "pose.json" in formats:
            path = self._own(os.path.join(folder, clip + ".pose.json"))
            with open(path, "w") as fh:
                json.dump({"fps": fps, "frames": frames}, fh)
            written.append(path)
        if "fbx" in formats:
            path = self._own(os.path.join(folder, clip + ".fbx"))
            keep = self._capture()
            act = bpy.data.actions.new(clip)
            self.arm.animation_data_create()
            self.arm.animation_data.action = act
            for i, fr in enumerate(frames):
                for pb in self.arm.pose.bones:
                    if pb.name in fr:
                        pb.location, pb.rotation_quaternion = Vector(fr[pb.name][0]), Quaternion(fr[pb.name][1])
                        pb.keyframe_insert("location", frame=i)
                        pb.keyframe_insert("rotation_quaternion", frame=i)
            for o in bpy.context.view_layer.objects:
                o.select_set(False)
            self.arm.select_set(True)
            bpy.context.view_layer.objects.active = self.arm
            old_name = self.arm.name
            renamed = None
            if root_name:
                clash = bpy.data.objects.get(root_name)
                if clash and clash != self.arm:
                    renamed = clash
                    clash.name = clash.name + "_kept"
                self.arm.name = root_name
            self.sc.render.fps, self.sc.render.fps_base = int(round(fps)), 1.0
            self.sc.frame_start, self.sc.frame_end = 0, len(frames) - 1
            try:
                bpy.ops.export_scene.fbx(filepath=path, use_selection=True, object_types={'ARMATURE'}, add_leaf_bones=False,
                                         bake_anim=True, bake_anim_use_all_actions=False, bake_anim_use_nla_strips=False,
                                         bake_anim_force_startend_keying=True, bake_anim_simplify_factor=0.0)
            finally:
                self.arm.name = old_name
                if renamed:
                    renamed.name = root_name
                self.arm.animation_data.action = None
                bpy.data.actions.remove(act)
                self.sc.render.fps = 30
                self._apply(keep)
            written.append(path)
        return {"clip": clip, "frames": len(frames), "fps": fps, "written": written}

    # --- pictures ---------------------------------------------------------------------------------------------
    def render(self, views=("eye", "right", "left", "top"), size=(640, 360)):
        """Renders the pose: eye is the player's camera (90 degrees, 5 cm near plane); right, left, top and front
        look at the rifle from outside. Images go to the output folder."""
        self._need()
        out_dir = self._own(os.path.join(self.out_dir, "renders"))
        os.makedirs(out_dir, exist_ok=True)
        cam_data = bpy.data.cameras.get("poselab") or bpy.data.cameras.new("poselab")
        cam = bpy.data.objects.get("poselab") or bpy.data.objects.new("poselab", cam_data)
        if cam.name not in self.sc.collection.objects:
            self.sc.collection.objects.link(cam)
        cam_data.lens_unit = 'FOV'
        self.sc.camera = cam
        self.sc.render.engine = 'BLENDER_WORKBENCH'
        sh = self.sc.display.shading
        sh.light = 'STUDIO'
        sh.show_cavity, sh.cavity_type = True, 'BOTH'          # edges and creases read at a glance
        sh.show_shadows, sh.shadow_intensity = True, 0.35
        sh.show_specular_highlight = True
        self.sc.display.render_aa = '16'
        if self.R.get("styled"):
            sh.color_type = 'MATERIAL'                          # the sample's own colours
        else:
            sh.color_type = 'OBJECT'                            # a plain arms-and-rifle look for any rig
            for m in self.meshes:
                m.color = (0.55, 0.6, 0.7, 1)
            self.rifle_mesh.color = (0.12, 0.12, 0.13, 1)
        self.sc.render.resolution_x, self.sc.render.resolution_y = size
        G = self._gun()
        centre = self._point("bore") if "bore" in self.R.get("points", {}) else G.translation
        base = Vector(self.R.get("points", {}).get("bore", (0, 0, 0)))
        spots = {"right": (-70.0, 10.0, 25.0), "left": (70.0, 10.0, 25.0), "top": (-8.0, 10.0, 75.0), "front": (-25.0, 90.0, 20.0)}
        paths = []
        for v in views:
            if v == "eye":
                cam_data.angle = math.radians(H_FOV)
                cam_data.clip_start = 0.05
                cam.location = self.eye
                cam.rotation_euler = VIEW_FWD.to_track_quat('-Z', 'Y').to_euler()
            elif v in spots:
                cam_data.angle = math.radians(40.0)
                cam_data.clip_start = 0.01
                cam.location = G @ ue(base + Vector(spots[v]))
                zc = (cam.location - centre).normalized()
                upv = G.to_quaternion() @ Vector((0.0, 0.0, 1.0))
                if abs(upv.dot(zc)) > 0.95:
                    upv = G.to_quaternion() @ Vector((0.0, -1.0, 0.0))
                xc = upv.cross(zc).normalized()
                cam.rotation_euler = Matrix((xc, zc.cross(xc), zc)).transposed().to_euler()
            else:
                raise ValueError("views: eye, right, left, top, front")
            path = self._own(os.path.join(out_dir, "%s_%s.png" % (self.rig, v)))
            self.sc.render.filepath = path
            bpy.ops.render.render(write_still=True)
            paths.append({"view": v, "path": path})
        return {"images": paths}
