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
from mathutils.bvhtree import BVHTree

FINGERS = ("index", "middle", "ring", "pinky", "thumb")
RADIUS = {"forearm": 0.030, "palm": 0.018, "finger": 0.0075}       # m: the capsules a body part is tested with
DEFAULT_POLES = {"l": [45.0, -20.0, 95.0], "r": [-45.0, -20.0, 95.0]}  # arms frame, cm: where each elbow points
# bone names: the Unreal mannequin's by default; a rig's "bones" entry renames any of them
DEFAULT_BONES = {"gun": "ik_hand_gun", "upperarm": "upperarm_{s}", "lowerarm": "lowerarm_{s}", "hand": "hand_{s}",
                 "finger": "{f}_{j}_{s}"}

# the view: the eye looks along +Y (Blender -Y), up is +Z, the player's right is -X
def smooth(a, b, t):
    """0 before a, 1 after b, an S-curve between."""
    if b <= a:
        return 1.0 if t >= b else 0.0
    x = min(max((t - a) / (b - a), 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


VIEW_RIGHT, VIEW_FWD, VIEW_UP = Vector((-1.0, 0.0, 0.0)), Vector((0.0, -1.0, 0.0)), Vector((0.0, 0.0, 1.0))
H_FOV, ASPECT = 90.0, 16.0 / 9.0
PUBLIC = ("load_rig", "describe", "list_rigs", "pose_idle", "pose_clip", "move_part", "move_gun", "reach", "snapshot",
          "record_clip", "load_clip", "scan_clip", "fix_clip", "save_clip",
          "where", "distance", "clearance", "anatomy", "grip", "faces_eye", "visible", "screen", "solve", "render")
# How far a human arm bends, for a hand working a gun. The joint ranges are the AAOS normal values (elbow flexion
# 0-150, wrist flexion 80, extension 70, radial deviation 20, ulnar 30); the working rules are stricter: the wrist
# within 30 degrees of the forearm's line, the elbow (a hinge) never locked straight, and the elbow below the shoulder
# while the hand is on the gun. A rig's "limits" entry overrides any of these. The research behind each value and how
# it differs from the standard: poselab_mcp/ranges.py and docs/ANATOMY.md.
ARM_LIMITS = {"wrist_max_deg": 30.0, "wrist_flexion_deg": 80.0, "wrist_extension_deg": 70.0, "wrist_radial_deg": 20.0,
              "wrist_ulnar_deg": 30.0, "elbow_bend_max_deg": 150.0, "elbow_bend_min_deg": 5.0, "elbow_under_shoulder_cm": 2.0}
# Each finger joint, + curling toward the palm: the knuckle (MCP, "01"), the middle joint (PIP, "02"), the end joint
# (DIP, "03"): (least curl, most curl, most out of the finger's own plane), degrees. Below the least curl a joint is
# bent backward (hyperextended); out of its plane the finger is twisted. A rig's "finger_limits" entry overrides them.
FINGER_LIMITS = {"01": [-30.0, 100.0, 40.0], "02": [-5.0, 110.0, 15.0], "03": [-10.0, 90.0, 25.0]}
JOINT_NAMES = {"01": "knuckle", "02": "middle joint", "03": "end joint"}
# The thumb: its base joint (CMC, where the metacarpal "01" meets the wrist) moves on two axes; its knuckle (MCP, "02")
# and end joint (IP, "03") are hinges that bend it across the palm toward the little finger. cmc_spread_max_deg: the
# most angle between the thumb's metacarpal and the index finger's; cmc_palmar_min_deg: how far the metacarpal may sit
# behind the palm's plane (below 0); "02", "03": (least bend, most bend, most out of the hinge's plane), degrees. The
# hinge values are Eaton's hyperextension (MCP 10, IP 15) and the AAOS flexion (MCP 50, IP 80) plus the fingers' 10
# degree slack; the CMC values are working values (see ranges.py). A rig's "thumb_limits" entry overrides them.
THUMB_LIMITS = {"cmc_spread_max_deg": 80.0, "cmc_palmar_min_deg": -20.0, "02": [-10.0, 60.0, 30.0], "03": [-15.0, 90.0, 25.0]}
THUMB_JOINTS = {"02": "knuckle", "03": "end joint"}


def ue(v):
    """Unreal-style cm (x, y, z) to Blender metres."""
    return Vector((v[0], -v[1], v[2])) / 100.0


def ue_back(v):
    return Vector((v[0], -v[1], v[2])) * 100.0


def _xyz(v, what):
    """Three numbers, or a clear error that names the argument."""
    if not isinstance(v, (list, tuple)) or len(v) != 3 or not all(isinstance(c, (int, float)) for c in v):
        raise ValueError("%s must be [x, y, z] (three numbers), got %r" % (what, v))
    return v


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
            # utf-8-sig: a file saved with a byte order mark (Windows PowerShell writes one) reads the same
            with open(self.rigs_file, encoding="utf-8-sig") as fh:
                rigs_json = json.load(fh)
            for k, v in rigs_json.items():
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
                with open(path, encoding="utf-8-sig") as fh:
                    data = json.load(fh)
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
        v = Vector(_xyz(v, "a point"))
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
            raise ValueError("unknown point %r; use [x, y, z], a rig point (%s), a fingertip like index_tip_l, or a bone "
                             "name" % (p, ", ".join(sorted(pts))))
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
        asked, length = seconds, self._clip_len(clip)
        seconds = min(max(float(seconds), 0.0), length)   # a time outside the clip shows its first or last frame
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
        out = {"clip": clip, "seconds": round(seconds, 3), "length": round(length, 3)}
        if abs(asked - seconds) > 1e-9:
            out["note"] = "%s s is outside the clip; showing %s s" % (asked, round(seconds, 3))
        return out

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
        if pole is not None:
            _xyz(pole, "pole")
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

    class Surface:
        """The rifle's surface: the closest point on its faces and the signed distance to it, negative inside. The
        nearest corner point (a KD tree of vertices) read a hand 0.68 cm into a low-poly magazine that it was 1.68 cm
        into: a box's flat face has no vertex inside it. Inside or out is decided by ray parity (an odd count of
        crossings, the majority of three rays), not by the nearest face's normal, which reads wrong near an edge."""
        RAYS = (Vector((1.0, 0.13, 0.07)).normalized(), Vector((-0.11, 1.0, 0.05)).normalized(), Vector((0.09, -0.04, 1.0)).normalized())

        def __init__(self, bvh):
            self.bvh = bvh

        def _crossings(self, p, d):
            n, o = 0, p.copy()
            for _ in range(64):
                hit = self.bvh.ray_cast(o, d, 100.0)
                if hit[0] is None:
                    break
                n += 1
                o = hit[0] + d * 1e-5
            return n

        def inside(self, p):
            return sum(self._crossings(p, d) % 2 for d in self.RAYS) >= 2

        def nearest(self, p):
            loc, _nrm, _idx, d = self.bvh.find_nearest(p)
            if loc is None:
                return None, None
            # parity only where it matters: a point further out than any capsule's radius is outside either way
            return loc, (-d if d < 0.05 and self.inside(p) else d)

    def _tree(self):
        dg = bpy.context.evaluated_depsgraph_get()
        ev = self.rifle_mesh.evaluated_get(dg)
        me = ev.to_mesh()
        M = self.rifle_mesh.matrix_world
        bvh = BVHTree.FromPolygons([M @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons])
        ev.to_mesh_clear()
        return self.Surface(bvh)

    def clearance(self, parts="both", ignore=(), frame="gun", top=5):
        """How deep the rifle sits inside the arms (capsules: forearm 3 cm, palm 1.8 cm, each finger segment 0.75 cm),
        measured to the rifle's surface, not its corner points. worst_cm 0: nothing touches. ignore: segment names,
        prefixes or wildcards (thumb, index3_r, *_l)."""
        self._need()
        tree = self._tree()
        hits = {}
        for name, a, b, r in self._segments(parts):
            if any(name.startswith(i) or fnmatch.fnmatch(name, i) for i in ignore):
                continue
            n = max(2, int((b - a).length / 0.004))
            for i in range(n + 1):
                p = a.lerp(b, i / n)
                co, d = tree.nearest(p)
                if co is not None and d < r and (r - d) * 100.0 > hits.get(name, (0.0,))[0]:
                    hits[name] = ((r - d) * 100.0, co.copy())
        ranked = sorted(hits.items(), key=lambda kv: -kv[1][0])
        return {"worst_cm": round(ranked[0][1][0], 2) if ranked else 0.0,
                "contacts": [{"segment": k, "depth_cm": round(v[0], 2), "rifle_point": r2(self._to(v[1], frame), 1)} for k, v in ranked[:top]]}

    # --- the hand's contact with the rifle -------------------------------------------------------------------------
    def _hand_frame(self, s):
        """The hand's thumb-side line (across the knuckles) and the sign that turns across x a segment's direction
        into that segment's palm side (+: the side a finger curls toward)."""
        f = lambda fi: self._bw(self._n("finger", s, fi, "01")).translation
        return (f("index") - f("pinky")).normalized(), (-1.0 if s == "l" else 1.0)   # left palm down: thumb x fingers points up

    def _hand_contacts(self, s, tree):
        """The hand's contacts with the rifle: (closest gap cm, deepest palm-side cm and segment, deepest back-side cm,
        segment and rifle point). The thumb's pad faces sideways, not the way the fingers curl, so its contacts count
        only toward the palm side's depth."""
        across, k = self._hand_frame(s)
        palm_cm = back_cm = 0.0
        near, palm_seg, back_seg, back_pt = None, None, None, None
        for name, a, b, r in self._segments(s):
            if name.startswith("forearm"):
                continue
            d = b - a
            if d.length < 1e-9:
                continue
            dn = d.normalized()
            palmar = across.cross(dn) * k
            thumb = name.startswith("thumb")
            n = max(2, int(d.length / 0.003))
            for i in range(n + 1):
                p = a.lerp(b, i / n)
                q, dq = tree.nearest(p)
                if q is None:
                    continue
                gap = (dq - r) * 100.0
                near = gap if near is None else min(near, gap)
                if dq < r:
                    depth = (r - dq) * 100.0
                    # the side the rifle is on: toward its surface point from outside, away from it from inside
                    off = (q - p) if dq >= 0.0 else (p - q)
                    off = off - dn * off.dot(dn)
                    back = (not thumb and palmar.length > 1e-6 and off.length > 1e-9
                            and off.normalized().dot(palmar.normalized()) < -0.25)
                    if back and depth > back_cm:
                        back_cm, back_seg, back_pt = depth, name, q.copy()
                    elif not back and depth > palm_cm:      # the palm side, a finger's edge, or the thumb
                        palm_cm, palm_seg = depth, name
        return near, palm_cm, palm_seg, back_cm, back_seg, back_pt

    def grip(self, side="both", back_max_cm=0.1, palm_max_cm=1.0, hold_within_cm=1.0, palm_within_cm=1.5, hold=False,
             frame="gun"):
        """How each hand touches the rifle: the palm side or the back. A hand holds with its palm and the palm sides
        of its fingers; the rifle inside the back of the hand or the back of a finger is a physical error that
        clearance with palms and fingers ignored never shows. For each hand: palm_faces_rifle (the palm's direction
        dotted with the direction to the nearest rifle point, 1 square on, below 0 the back of the hand toward it),
        palm_contact_cm and back_contact_cm (the deepest rifle point inside the hand on each side), and bad: the rules
        broken. A hand within hold_within_cm of the rifle must face it with the palm (palm_faces_rifle above 0) and
        keep the back clear (back_contact_cm at most back_max_cm). The palm side may press into the rifle up to
        palm_max_cm: the capsules are rounder than a palm, so a firm grip reads a few millimetres deep. The thumb's
        contacts count toward the palm side only (its pad faces sideways). touching: some part of the hand within
        hold_within_cm. holding: touching, the palm facing the rifle and the palm's own surface within palm_within_cm of
        it (palm_gap_cm, below 0 pressed in); a fingertip on the rifle with the palm off it is touching, not holding.
        hold true: a hand that is not holding breaks a rule."""
        self._need()
        if side not in ("both", "l", "r"):
            raise ValueError("side must be both, l or r, got %r" % (side,))
        tree = self._tree()
        out = {"ok": True}
        for s_ in (("l", "r") if side == "both" else (side,)):
            near, palm_cm, palm_seg, back_cm, back_seg, back_pt = self._hand_contacts(s_, tree)
            across, k = self._hand_frame(s_)
            hn = self._bw(self._n("hand", s_)).translation
            mid = self._bw(self._n("finger", s_, "middle", "01")).translation
            centre = hn.lerp(mid, 0.5)
            palm_dir = across.cross((mid - hn).normalized()) * k
            co, _d = tree.nearest(centre)
            facing = palm_dir.normalized().dot((co - centre).normalized()) if co is not None and (co - centre).length > 1e-9 else 0.0
            # the palm's own surface: the capsule's skin on the palm side of its bone line
            _pc, palm_d = tree.nearest(centre + palm_dir.normalized() * RADIUS["palm"]) if palm_dir.length > 1e-9 else (None, None)
            palm_gap = palm_d * 100.0 if palm_d is not None else None
            touching = near is not None and near <= hold_within_cm
            holding = touching and facing > 0.0 and palm_gap is not None and palm_gap <= palm_within_cm
            bad = []
            if touching and facing <= 0.0:
                bad.append("holds the rifle with the back of the hand (palm faces %.2f away from it)" % -facing)
            if hold and not holding:
                why = ("nothing of the hand within %.1f cm of it" % hold_within_cm if not touching else
                       "the palm faces away from it" if facing <= 0.0 else
                       "only the fingers touch it: the palm is %.1f cm off (at most %.1f)" % (palm_gap, palm_within_cm))
                bad.append("does not hold the rifle: " + why)
            if palm_cm > palm_max_cm:
                bad.append("rifle %.2f cm inside the palm side of %s (at most %.2f)" % (palm_cm, palm_seg, palm_max_cm))
            if back_cm > back_max_cm:
                bad.append("rifle %.2f cm inside the back of %s (at most %.2f)" % (back_cm, back_seg, back_max_cm))
            rep_ = {"holding": holding, "touching": touching, "gap_cm": round(near, 2) if near is not None else None,
                    "palm_gap_cm": round(palm_gap, 2) if palm_gap is not None else None,
                    "palm_faces_rifle": round(facing, 2), "palm_contact_cm": round(palm_cm, 2),
                    "back_contact_cm": round(back_cm, 2), "bad": bad}
            if back_seg:
                rep_["back_contact"] = {"segment": back_seg, "rifle_point": r2(self._to(back_pt, frame), 1)}
            out[s_] = rep_
            out["ok"] = out["ok"] and not bad
        return out

    # --- the human arm's limits --------------------------------------------------------------------------------
    def _limits(self):
        fl = {k: list(v) for k, v in FINGER_LIMITS.items()}
        fl.update({k: list(v) for k, v in self.R.get("finger_limits", {}).items()})
        return dict(ARM_LIMITS, **self.R.get("limits", {})), fl

    @staticmethod
    def _deg(a, b):
        return math.degrees(a.angle(b)) if a.length > 1e-9 and b.length > 1e-9 else 0.0

    def _arm_report(self, s, L):
        up = Vector((0.0, 0.0, 1.0))
        b = lambda part: self._bw(self._n(part, s)).translation
        f = lambda fi: self._bw(self._n("finger", s, fi, "01")).translation
        shoulder, elbow, wrist, knuckle = b("upperarm"), b("lowerarm"), b("hand"), f("middle")
        upper, fore, hand = elbow - shoulder, wrist - elbow, knuckle - wrist
        bad = []
        under = (shoulder - elbow).dot(up) * 100.0
        # every rule reads the value as reported (the elbow's height to 0.1 cm, angles to the degree): a wrist reported
        # at its 20 degree limit passes, one reported at 21 fails
        if round(under, 1) < L["elbow_under_shoulder_cm"]:
            bad.append("elbow %.1f cm under the shoulder (at least %.1f)" % (under, L["elbow_under_shoulder_cm"]))
        bend = self._deg(upper, fore)
        if round(bend) > L["elbow_bend_max_deg"]:
            bad.append("elbow bent %.0f (at most %.0f)" % (bend, L["elbow_bend_max_deg"]))
        if round(bend) < L["elbow_bend_min_deg"]:
            bad.append("elbow locked straight (%.0f)" % bend)
        total = self._deg(fore, hand)
        thumb_side = f("index") - f("pinky")
        palm = thumb_side.cross(hand) * (-1.0 if s == "l" else 1.0)   # toward the palm (see _hand_frame)
        fn = fore.normalized()
        off = hand.normalized() - fn * hand.normalized().dot(fn)
        flat = lambda v: v - fn * v.dot(fn)
        asin = lambda x: math.degrees(math.asin(max(-1.0, min(1.0, x))))
        # the bend split on two perpendicular axes across the forearm: toward the palm, and toward the thumb
        c = hand.normalized().dot(fn)
        e1 = flat(palm).normalized() if flat(palm).length > 1e-9 else Vector((0.0, 0.0, 0.0))
        t = flat(thumb_side)
        e2 = t - e1 * t.dot(e1)
        e2 = e2.normalized() if e2.length > 1e-9 else Vector((0.0, 0.0, 0.0))
        flex = math.degrees(math.atan2(off.dot(e1), c))
        dev = math.degrees(math.atan2(off.dot(e2), c))
        if round(total) > L["wrist_max_deg"]:
            bad.append("wrist bent %.0f off the forearm (at most %.0f)" % (total, L["wrist_max_deg"]))
        rf, rd = round(flex), round(dev)
        if rf > L["wrist_flexion_deg"] or -rf > L["wrist_extension_deg"] or rd > L["wrist_radial_deg"] or -rd > L["wrist_ulnar_deg"]:
            bad.append("wrist past its joint range (%s %.0f, %s %.0f)" % ("flexion" if flex >= 0 else "extension", abs(flex),
                                                                          "radial" if dev >= 0 else "ulnar", abs(dev)))
        return {"elbow_under_shoulder_cm": round(under, 1), "upper_arm_raised_deg": round(self._deg(upper, -up)),
                "elbow_bend_deg": round(bend), "wrist_bend_deg": round(total), "wrist_flexion_deg": round(flex),
                "wrist_radial_deg": round(dev)}, bad

    def _finger_tip(self, fi, s):
        """The end segment's tip: the middle segment's rest line carried by the end bone's own turn from its rest
        (a rig loaded from FBX keeps no fingertip, so a last bone's tail can point anywhere)."""
        n3, n2 = self._n("finger", s, fi, "03"), self._n("finger", s, fi, "02")
        b3, b2 = self.arm.data.bones[n3], self.arm.data.bones[n2]
        rest = b3.head_local - b2.head_local
        if rest.length < 1e-9:
            return self._tip(fi, s)
        P = self.arm.pose.bones[n3].matrix @ b3.matrix_local.inverted()
        return self.arm.matrix_world @ (P @ (b3.head_local + rest.normalized() * rest.length * 0.8))

    def _finger_report(self, s, FL):
        """Each finger joint's curl (+ toward the palm) and its bend out of the finger's own plane; the joints out of
        range. A healthy finger is three hinges about parallel lines: it stays in one plane whatever its curl."""
        b = lambda fi, j: self._bw(self._n("finger", s, fi, j)).translation
        across = (b("index", "01") - b("pinky", "01")).normalized()
        sign = -1.0 if s == "l" else 1.0
        wrist_to_knuckle = b("middle", "01") - self._bw(self._n("hand", s)).translation
        out, bad, planes = {}, [], {}
        asin = lambda x: math.degrees(math.asin(max(-1.0, min(1.0, x))))
        for fi in ("index", "middle", "ring", "pinky"):
            pts = [b(fi, "01") - wrist_to_knuckle, b(fi, "01"), b(fi, "02"), b(fi, "03"), self._finger_tip(fi, s)]
            d1, d2 = pts[2] - pts[1], pts[4] - pts[1]
            n = d1.cross(d2)
            if n.length < 0.26 * d1.length * d2.length:   # under about 15 degrees of curl: no plane of its own
                n = across - d2.normalized() * across.dot(d2.normalized())
            n.normalize()
            planes[fi] = n
            for i, j in enumerate(("01", "02", "03")):
                a, c = (pts[i + 1] - pts[i]).normalized(), (pts[i + 2] - pts[i + 1]).normalized()
                curl = math.degrees(math.atan2(a.cross(c).dot(across) * sign, a.dot(c)))
                if j == "01":
                    side_axis = across - a * across.dot(a)
                    side_deg = asin(c.dot(side_axis.normalized())) if side_axis.length > 1e-6 else 0.0
                else:
                    side_deg = asin(c.dot(n)) - asin(a.dot(n))
                lo, hi, sd = FL[j]
                out["%s_%s" % (fi, j)] = [round(curl), round(side_deg)]
                if round(curl) < lo:
                    bad.append("%s %s bent %.0f backward (hyperextended; at most %.0f)" % (fi, JOINT_NAMES[j], -curl, -lo))
                elif round(curl) > hi:
                    bad.append("%s %s curled %.0f (at most %.0f)" % (fi, JOINT_NAMES[j], curl, hi))
                if abs(round(side_deg)) > sd:
                    bad.append("%s %s twisted %.0f out of the finger's plane (at most %.0f)" % (fi, JOINT_NAMES[j], abs(side_deg), sd))
        return out, bad, planes

    def _thumb_limits(self):
        tl = {k: (list(v) if isinstance(v, list) else v) for k, v in THUMB_LIMITS.items()}
        tl.update(self.R.get("thumb_limits", {}))
        return tl

    def _has_thumb(self, s):
        try:
            for j in ("01", "02", "03"):
                self.arm.pose.bones[self._n("finger", s, "thumb", j)]
            return True
        except KeyError:
            return False

    def _thumb_report(self, s, TL):
        """The thumb's base joint (spread from the index metacarpal, and how far in front of the palm's plane) and its
        knuckle and end joint (bend + across the palm toward the little finger, and the bend out of the hinge's
        plane). Returns the values, the rules broken, and each hinge's axis (for the mend)."""
        b = lambda j: self._bw(self._n("finger", s, "thumb", j)).translation
        across, k = self._hand_frame(s)
        wrist = self._bw(self._n("hand", s)).translation
        mid = self._bw(self._n("finger", s, "middle", "01")).translation
        index_meta = self._bw(self._n("finger", s, "index", "01")).translation - wrist
        palm = across.cross((mid - wrist).normalized()) * k
        palm = palm.normalized() if palm.length > 1e-9 else palm
        pts = [b("01"), b("02"), b("03"), self._finger_tip("thumb", s)]
        meta = pts[1] - pts[0]
        asin = lambda x: math.degrees(math.asin(max(-1.0, min(1.0, x))))
        spread = self._deg(meta, index_meta)
        palmar = asin(meta.normalized().dot(palm)) if meta.length > 1e-9 else 0.0
        out = {"cmc_spread_deg": round(spread), "cmc_palmar_deg": round(palmar)}
        bad, axes = [], {}
        if round(spread) > TL["cmc_spread_max_deg"]:
            bad.append("thumb base spread %.0f from the index (at most %.0f)" % (spread, TL["cmc_spread_max_deg"]))
        if round(palmar) < TL["cmc_palmar_min_deg"]:
            bad.append("thumb base %.0f behind the palm (at most %.0f)" % (-palmar, -TL["cmc_palmar_min_deg"]))
        flex = (palm - across).normalized()          # a thumb bends across the palm toward the little finger
        for i, j in ((0, "02"), (1, "03")):
            a, c = (pts[i + 1] - pts[i]), (pts[i + 2] - pts[i + 1])
            if a.length < 1e-9 or c.length < 1e-9:
                continue
            a, c = a.normalized(), c.normalized()
            f_perp = flex - a * flex.dot(a)
            if f_perp.length < 1e-6:
                continue
            f_perp.normalize()
            axis = a.cross(f_perp).normalized()      # the hinge: turning about it moves c toward flex
            in_plane = c - axis * c.dot(axis)
            bend = math.degrees(math.atan2(in_plane.dot(f_perp), in_plane.dot(a))) if in_plane.length > 1e-9 else 0.0
            side_deg = asin(c.dot(axis))
            axes[j] = axis
            lo, hi, sd = TL[j]
            out["thumb_" + j] = [round(bend), round(side_deg)]
            if round(bend) < lo:
                bad.append("thumb %s bent %.0f backward (hyperextended; at most %.0f)" % (THUMB_JOINTS[j], -bend, -lo))
            elif round(bend) > hi:
                bad.append("thumb %s bent %.0f (at most %.0f)" % (THUMB_JOINTS[j], bend, hi))
            if abs(round(side_deg)) > sd:
                bad.append("thumb %s bent %.0f out of its hinge's plane (at most %.0f)" % (THUMB_JOINTS[j], abs(side_deg), sd))
        return out, bad, axes

    def _has_fingers(self, s):
        try:
            for fi in ("index", "middle", "ring", "pinky"):
                for j in ("01", "02", "03"):
                    self.arm.pose.bones[self._n("finger", s, fi, j)]
            return True
        except KeyError:
            return False

    def anatomy(self, side="both", fingers=True):
        """Each arm against the human arm's limits (ARM_LIMITS, FINGER_LIMITS): the elbow under the shoulder, the elbow's
        bend, the wrist's bend split into flexion (+ toward the palm) and radial deviation (+ toward the thumb), and
        each finger joint's curl and twist, and the thumb (THUMB_LIMITS: its base joint's spread and place in front of
        the palm, its knuckle's and end joint's bend). ok false lists every rule the pose breaks. The hand's roll belongs to the
        forearm (radius over ulna), so a rolled hand turns the forearm, not the wrist joint."""
        self._need()
        if side not in ("both", "l", "r"):
            raise ValueError("side must be both, l or r, got %r" % (side,))
        L, FL = self._limits()
        TL = self._thumb_limits()
        out = {"ok": True, "limits": dict(L, fingers=FL, thumb=TL)}
        for s_ in (("l", "r") if side == "both" else (side,)):
            rep_, bad = self._arm_report(s_, L)
            if fingers and self._has_fingers(s_):
                fr, fbad, _ = self._finger_report(s_, FL)
                rep_["fingers"] = fr
                bad += fbad
            if fingers and self._has_thumb(s_):
                tr, tbad, _ = self._thumb_report(s_, TL)
                rep_["thumb"] = tr
                bad += tbad
            rep_["bad"] = bad
            out[s_] = rep_
            out["ok"] = out["ok"] and not bad
        return out

    def _turn_about(self, n, axis, deg, pivot):
        M = self._bw(n)
        q = Quaternion(axis, math.radians(deg))
        R = Matrix.Translation(pivot) @ q.to_matrix().to_4x4() @ Matrix.Translation(-pivot) @ M
        self._set_world(n, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))

    def _mend_hand(self, s, fingers=True):
        """The wrist back inside its limits (turned toward the forearm's line), then each finger joint back inside its
        curl range (turned about the finger's own hinge, the better way) and the knuckle and middle joint back into the
        finger's plane. Returns how many corrections it made."""
        L, FL = self._limits()
        made = 0
        hn = self._n("hand", s)
        tree = self._tree()
        back0 = self._hand_contacts(s, tree)[3]
        for _ in range(3):
            rep_, bad = self._arm_report(s, L)
            over = max(rep_["wrist_bend_deg"] - L["wrist_max_deg"], rep_["wrist_radial_deg"] - L["wrist_radial_deg"],
                       -rep_["wrist_radial_deg"] - L["wrist_ulnar_deg"], 0.0)
            if over <= 0.0 or rep_["wrist_bend_deg"] <= 0.0:
                break
            line = (self._bw(hn).translation - self._bw(self._n("lowerarm", s)).translation).normalized()
            hand = (self._bw(self._n("finger", s, "middle", "01")).translation - self._bw(hn).translation).normalized()
            want = min(1.0, (over + 1.0) / rep_["wrist_bend_deg"])
            H = self._bw(hn)
            turned = False
            for part in (1.0, 0.5, 0.25):          # the largest turn that keeps the back of the hand out of the rifle
                q = Quaternion().slerp(hand.rotation_difference(line), want * part)
                self._set_world(hn, Matrix.LocRotScale(H.translation, q @ H.to_quaternion(), self._unit()))
                if self._hand_contacts(s, tree)[3] <= max(back0, 0.1) + 1e-6:
                    turned = True
                    break
                self._set_world(hn, H)
            if not turned:
                break
            made += 1
        if not fingers or not self._has_fingers(s):
            return made
        for _ in range(4):
            changed = False
            for fi in ("index", "middle", "ring", "pinky"):
                for j in ("01", "02", "03"):
                    fr, _b, planes = self._finger_report(s, FL)
                    curl, side_deg = fr["%s_%s" % (fi, j)]
                    lo, hi, sd = FL[j]
                    n = self._n("finger", s, fi, j)
                    head = self._bw(n).translation
                    if curl <= lo + 0.5 or curl >= hi - 0.5:
                        fix = (lo + 3.0 - curl) if curl <= lo + 0.5 else (hi - 3.0 - curl)
                        axis = planes[fi]
                        over0 = (lo - curl) if curl < lo else (curl - hi)
                        M = self._bw(n)
                        for sg in (1.0, -1.0):
                            self._turn_about(n, axis, fix * sg, head)
                            c2 = self._finger_report(s, FL)[0]["%s_%s" % (fi, j)][0]
                            if ((lo - c2) if c2 < lo else (c2 - hi) if c2 > hi else -1.0) < over0:
                                break
                            self._set_world(n, M)
                        made += 1
                        changed = True
                    elif j != "03" and abs(side_deg) >= sd - 0.5:
                        child = self._bw(self._n("finger", s, fi, "%02d" % (int(j) + 1))).translation
                        axis = (child - head).cross(planes[fi])
                        if axis.length < 1e-9:
                            continue
                        fix = math.copysign(sd - 3.0, side_deg) - side_deg
                        M = self._bw(n)
                        for sg in (1.0, -1.0):
                            self._turn_about(n, axis.normalized(), fix * sg, head)
                            if abs(self._finger_report(s, FL)[0]["%s_%s" % (fi, j)][1]) < abs(side_deg):
                                break
                            self._set_world(n, M)
                        made += 1
                        changed = True
            if not changed:
                break
        if self._has_thumb(s):
            TL = self._thumb_limits()
            for _ in range(3):
                changed = False
                for j in ("02", "03"):
                    tr, _b, axes = self._thumb_report(s, TL)
                    if "thumb_" + j not in tr or j not in axes:
                        continue
                    bend = tr["thumb_" + j][0]
                    lo, hi, _sd = TL[j]
                    if lo + 0.5 < bend < hi - 0.5:
                        continue
                    fix = (lo + 3.0 - bend) if bend <= lo + 0.5 else (hi - 3.0 - bend)
                    over0 = (lo - bend) if bend < lo else (bend - hi)
                    n = self._n("finger", s, "thumb", j)
                    head, M = self._bw(n).translation.copy(), self._bw(n)
                    for sg in (1.0, -1.0):
                        self._turn_about(n, axes[j], fix * sg, head)
                        b2 = self._thumb_report(s, TL)[0]["thumb_" + j][0]
                        if ((lo - b2) if b2 < lo else (b2 - hi) if b2 > hi else -1.0) < over0:
                            break
                        self._set_world(n, M)
                    made += 1
                    changed = True
                if not changed:
                    break
        return made

    def faces_eye(self, point="port", normal=None):
        """How squarely a surface faces the eye: 1 square on, 0 edge on, below 0 turned away."""
        self._need()
        p = self._point(point)
        n = normal if normal is not None else (self.R.get("normals", {}).get(point) if isinstance(point, str) else None)
        if n is None:
            raise ValueError("no normal for %r: give one in the gun frame" % (point,))
        if ue(_xyz(n, "normal")).length < 1e-9:
            raise ValueError("normal must not be [0, 0, 0]")
        nw = (self._gun().to_quaternion() @ ue(n)).normalized()
        to_eye = self.eye - p
        dot = nw.dot(to_eye.normalized())
        return {"facing": round(dot, 3), "angle_from_square_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, dot)))), 1),
                "distance_cm": round(to_eye.length * 100.0, 1)}

    def visible(self, point="port", radius_cm=0.6, rings=2):
        """The share of a small disc round a point the eye sees with nothing (arms, rifle) in the way."""
        self._need()
        if radius_cm <= 0:
            raise ValueError("radius_cm must be above 0")
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
        if t == "anatomy":
            a = self.anatomy(g.get("side", "both"), g.get("fingers", True))
            n = sum(len(a[k]["bad"]) for k in ("l", "r") if k in a)
            return n == 0, n, float(n)
        if t == "grip":
            a = self.grip(g.get("side", "both"), g.get("back_max_cm", 0.1), g.get("palm_max_cm", 1.0), g.get("hold_within_cm", 1.0))
            n = sum(len(a[k]["bad"]) for k in ("l", "r") if k in a)
            return n == 0, n, float(n)
        if t in ("distance", "contact"):
            v = self.distance(g["a"], g["b"])["cm"]
            return v <= g.get("max_cm", 0.5), v, max(0.0, v - g.get("max_cm", 0.5)) / 5.0
        raise ValueError("goal type %r: faces_eye, visible, on_screen, clearance, anatomy, grip, barrel, distance, contact" % t)

    def solve(self, dofs, goals, keep_hands=("l", "r"), pivot="stock", samples=120, maximize=None, seed=1):
        """Searches rifle moves (dofs: roll, swing, pitch, right, forward, up -> [min, max]) from the current pose for
        one that meets every goal; reports each goal at the best and how often any sample met it (never met: a fact
        of the geometry, not a tuning miss). Leaves the scene at the best."""
        self._need()
        known = ("roll", "swing", "pitch", "right", "forward", "up")
        names = list(dofs)
        if not names or any(k not in known for k in names):
            raise ValueError("dofs: name -> [min, max] for roll, swing, pitch, right, forward, up; got %s" % ", ".join(map(str, names)))
        for k in names:
            r = dofs[k]
            if not isinstance(r, (list, tuple)) or len(r) != 2 or not all(isinstance(c, (int, float)) for c in r) or r[0] > r[1]:
                raise ValueError("dofs[%r] must be [min, max] with min <= max, got %r" % (k, r))
        if samples < 1:
            raise ValueError("samples must be 1 or more")
        if maximize is not None and not 0 <= maximize < len(goals):
            raise ValueError("maximize must be a goal's index, 0 to %d" % (len(goals) - 1))
        for g in goals:
            self._need_keys(g)
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
        each bone eased from key to key). Between keys the bones blend by rotation, so hands may drift off the rifle
        between keys."""
        self._need()
        if action == "start":
            if not clip:
                raise ValueError("start needs a clip name")
            if fps <= 0:
                raise ValueError("fps must be above 0")
            self._rec = {"clip": clip, "fps": float(fps), "keys": []}
            return {"recording": clip, "fps": fps}
        rec = getattr(self, "_rec", None)
        if not rec:
            raise RuntimeError("no recording: record_clip start first")
        if action == "key":
            if seconds < 0:
                raise ValueError("seconds must be 0 or more")
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
            with open(full, encoding="utf-8-sig") as fh:
                data = json.load(fh)
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
        """Plays a clip frame by frame and runs each check: clearance, anatomy (side, fingers: the rules a frame
        breaks, counted), faces_eye, visible, on_screen, barrel, contact
        (a, b, max_cm), hold (side, max_cm: the hand's drift on the rifle from its grip at ref_s), pop (bones, "gun" for the gun bone,
        max_cm_per_s). Any check takes during: [from_s, to_s]. Reports, for each check, the worst value and when, and
        the times it fails."""
        self._need()
        frames, fps = self._frames(clip)
        return self._scan(frames, fps, checks, clip)

    def _need_keys(self, c):
        """A clear error for a goal or check that lacks what its type needs."""
        if not isinstance(c, dict) or "type" not in c:
            raise ValueError("each goal or check is {type: ..., ...}, got %r" % (c,))
        t = c["type"]
        need = {"on_screen": ("point",), "distance": ("a", "b"), "contact": ("a", "b"), "hold": ("side",)}.get(t, ())
        missing = [k for k in need if k not in c]
        if missing:
            raise ValueError("%s needs %s" % (t, ", ".join(missing)))
        if t == "hold" and c["side"] not in ("l", "r"):
            raise ValueError("hold side must be l or r, got %r" % (c["side"],))
        if t == "anatomy" and c.get("side", "both") not in ("both", "l", "r"):
            raise ValueError("anatomy side must be both, l or r, got %r" % (c.get("side"),))
        if t == "pop" and "bones" in c and not c["bones"]:
            raise ValueError("pop bones must name at least one bone")
        d = c.get("during")
        if d is not None and (not isinstance(d, (list, tuple)) or len(d) != 2 or not all(isinstance(x, (int, float)) for x in d) or d[0] > d[1]):
            raise ValueError("during must be [from_s, to_s] with from_s <= to_s, got %r" % (d,))

    def _scan(self, frames, fps, checks, name):
        for c in checks:
            self._need_keys(c)
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
            higher_worse = c["type"] in ("clearance", "barrel", "distance", "contact", "hold", "pop", "anatomy")
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
        spread over the frames round it so nothing pops;
        anatomy: the elbow swings (as for clearance) by the least angle that brings it under the shoulder and inside
        its bend, then the wrist turns back inside its limits and each finger joint back inside its range.
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
        # anatomy: the least elbow swing that puts each elbow under its shoulder and inside its bend, spread over the
        # frames round it; then the wrist and the fingers inside their limits, frame by frame
        for c in [c for c in checks if c["type"] == "anatomy"]:
            L, _FL = self._limits()
            for side in [s_ for s_ in ("l", "r") if c.get("side", "both") in ("both", s_)]:
                elbow_bad = lambda: [b_ for b_ in self._arm_report(side, L)[1] if b_.startswith("elbow")]
                angle = [0.0] * len(frames)
                for i, fr in enumerate(frames):
                    if not self._in(c, i / fps):
                        continue
                    self._apply(fr)
                    if not elbow_bad():
                        continue
                    best = None
                    for a in steps:
                        self._apply(fr)
                        self._swing_elbow(side, a)
                        n_ = len(elbow_bad())
                        under = self._arm_report(side, L)[0]["elbow_under_shoulder_cm"]
                        if best is None or (n_, -under) < best[0]:
                            best = ((n_, -under), a)
                        if n_ == 0:
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
                for i, fr in enumerate(frames):
                    if not self._in(c, i / fps) and not spread[i]:
                        continue
                    self._apply(fr)
                    if spread[i]:
                        self._swing_elbow(side, spread[i])
                    made = self._mend_hand(side, c.get("fingers", True)) if self._in(c, i / fps) else 0
                    if spread[i] or made:
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
        if not views:
            raise ValueError("views: at least one of eye, right, left, top, front")
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
