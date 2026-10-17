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
PUBLIC = ("load_rig", "describe", "list_rigs", "pose_idle", "pose_clip", "move_part", "move_gun", "reach", "turn_bone", "snapshot",
          "record_clip", "load_clip", "scan_clip", "fix_clip", "save_clip",
          "where", "distance", "clearance", "anatomy", "grip", "purpose", "solve_hand", "hand_presses", "faces_eye", "visible", "screen", "solve", "render",
          "view_cover", "solve_grip", "retarget_clip", "check_set", "shift_hand", "body_size", "place_hand",
          "thumb_search", "contacts")
# How far a human arm bends, for a hand working a gun. The joint ranges are the AAOS normal values (elbow flexion
# 0-150, wrist flexion 80, extension 70, radial deviation 20, ulnar 30); the working rules are stricter: the wrist
# within 30 degrees of the forearm's line, the elbow (a hinge) never locked straight, and the elbow below the shoulder
# while the hand is on the gun. A rig's "limits" entry overrides any of these. The research behind each value and how
# it differs from the standard: poselab_mcp/ranges.py and docs/ANATOMY.md.
ARM_LIMITS = {"wrist_max_deg": 30.0, "wrist_flexion_deg": 80.0, "wrist_extension_deg": 70.0, "wrist_radial_deg": 20.0,
              "wrist_ulnar_deg": 30.0, "elbow_bend_max_deg": 150.0, "elbow_bend_min_deg": 5.0, "elbow_under_shoulder_cm": 2.0,
              "wrist_twist_max_deg": 30.0, "pronation_deg": 80.0, "supination_deg": 80.0, "forearm_link_max_deg": 50.0,
              "forearm_elbow_share": 0.1,
              # the shoulder (AAOS normal values): flexion 180, extension 60, horizontal adduction 135 from the side
              # (45 across the body past straight ahead), horizontal abduction 45 behind the side, internal rotation
              # 70, external 90; measured against the view (a first-person rig's camera rides the head)
              "shoulder_flexion_deg": 180.0, "shoulder_extension_deg": 60.0, "shoulder_cross_body_deg": 45.0,
              "shoulder_behind_deg": 45.0, "shoulder_internal_deg": 70.0, "shoulder_external_deg": 90.0}
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


def _adopt_orphans(mesh, full, arm):
    """A mesh's vertex groups whose bone the driving armature lacks (an idle clip's export may leave out a skeleton's
    corrective bones: Manny's upperarm_out_l, clavicle_scap_l, wrist_outer_l): Blender would leave those vertices at
    rest, a shoulder inside the eye. Each such group's weights go to its nearest ancestor the armature has (the mesh's
    own, full armature knows the chain), as the game moves an unkeyed bone: with its parent. The groups moved, by
    ancestor."""
    have = set(arm.data.bones.keys())
    moved = {}
    for vg in list(mesh.vertex_groups):
        if vg.name in have or vg.name not in full.data.bones:
            continue
        up = full.data.bones[vg.name].parent
        while up is not None and up.name not in have:
            up = up.parent
        if up is None:
            continue
        to = mesh.vertex_groups.get(up.name) or mesh.vertex_groups.new(name=up.name)
        for v in mesh.data.vertices:
            for g in v.groups:
                if g.group == vg.index and g.weight > 0.0:
                    to.add([v.index], g.weight, 'ADD')
        moved.setdefault(up.name, []).append(vg.name)
        mesh.vertex_groups.remove(vg)
    return moved


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
        self.orphans = {}
        if R.get("builtin"):
            from sample import build
            made = build()
            self.arm, self.meshes, self.rifle_arm, self.rifle_mesh = made["arm"], made["meshes"], made["rifle_arm"], made["rifle_mesh"]
            R.update(made["config"])
        else:
            self._load_files(R)
        # the rifle at its rest: a rifle's FBX may carry its own animation (a trigger, a magazine), which then followed
        # whatever scene frame a clip's sampling left; a finger on the trigger read 1.4 cm inside it (ToangTown's
        # AKS-74U, 2026-10-10). The rig's parts move only by move_part
        if self.rifle_arm.animation_data:
            self.rifle_arm.animation_data.action = None
        for pb in self.rifle_arm.pose.bones:     # the action's last pose stays on the bones without this
            pb.matrix_basis = Matrix.Identity(4)
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
        # the idle grip's forearm and hand: the twist between their bones there is the rig's own (a bind offset, not a
        # roll); anatomy measures the twist against it
        self.idle_world = {self._n(p, s): self._bw(self._n(p, s)).copy() for p in ("lowerarm", "hand") for s in ("l", "r")
                           if self._n(p, s) in self.arm.pose.bones}
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
            self.orphans = _adopt_orphans(mesh, old, self.arm)
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
            "orphan_groups_adopted": self.orphans,
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
                # a point on a moving part (the rig's "point_parts": a charging knob on the carrier) rides with it
                part = self.R.get("point_parts", {}).get(p)
                back = self.travel.get(part, 0.0) if part else 0.0
                return self._gun() @ ue((pts[p][0], pts[p][1] - back, pts[p][2]))
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
        the view (cm). The hands in keep_hands keep their hold (arm IK), each elbow pointing the way it pointed (the
        arm IK's default pole once bent a clip's right wrist past its limit in every frame: ToangTown's AK-47 run)."""
        self._need()
        G = self._gun()
        held = {s: G.inverted() @ self._bw(self._n("hand", s)) for s in keep_hands}
        bend = {}
        for s in keep_hands:
            S0 = self._bw(self._n("upperarm", s)).translation
            E0 = self._bw(self._n("lowerarm", s)).translation
            W0 = self._bw(self._n("hand", s)).translation
            line = W0 - S0
            foot = S0 + line * (max(0.0, min(1.0, (E0 - S0).dot(line) / max(line.length_squared, 1e-12))))
            bend[s] = (E0, E0 - foot, W0)
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
            E0, out, W0 = bend[s]
            pole = E0 + out * 2.0 + (W.translation - W0) * 0.5 if out.length > 1e-5 else None
            miss[s] = round(self._two_bone(s, W.translation, pole_world=pole), 2)
            hn = self._n("hand", s)
            self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, W.to_quaternion(), self._unit()))
            self._roll_forearm(s)
        return {"hands_off_their_hold_cm": miss, "barrel_off_deg": round(self._barrel_off(), 2)}

    def turn_bone(self, bone, axis, degrees, frame="gun"):
        """One bone turned by degrees about an axis through its head; the bones below it follow, nothing else moves.
        axis: a direction in the frame (gun, arms or view). The two signs turn opposite ways."""
        self._need()
        n = self._bone(bone)
        if n == self.names["gun"]:
            raise ValueError("turn the rifle with move_gun, not turn_bone")
        a = Vector(_xyz(axis, "axis"))
        if a.length < 1e-9:
            raise ValueError("the axis has no length")
        if frame == "gun":
            d = self._gun().to_quaternion() @ ue(a)
        elif frame == "arms":
            d = ue(a)
        elif frame == "view":
            d = VIEW_RIGHT * a.x + VIEW_FWD * a.y + VIEW_UP * a.z
        else:
            raise ValueError("frame must be gun, arms or view")
        head = self._bw(n).translation.copy()
        self._turn_about(n, d.normalized(), float(degrees), head)
        # no positions in the reply: a pose tool that answered where a joint sits would be a measuring tool
        return {"bone": n, "degrees": float(degrees), "axis": r2(Vector(a).normalized(), 3), "frame": frame}

    def reach(self, side, target, frame="gun", pole=None):
        """A wrist to a point by arm IK; the hand keeps its turn. pole: where the elbow points (arms frame, cm)."""
        self._need()
        if pole is not None:
            _xyz(pole, "pole")
        hn = self._n("hand", side)
        H = self._bw(hn)
        miss = self._two_bone(side, self._point(target, frame), pole)
        self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, H.to_quaternion(), self._unit()))
        self._roll_forearm(side)
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
            fit = self._fitted_radii()
            if fit:
                # capsules fitted to the rig's own skin (fit_capsules): the forearm in two halves (it tapers to the
                # wrist), the palm as three spokes from the wrist to the index, middle and little finger knuckles
                # (one line down its middle left its sides untested), each finger its own girth
                E, H = b("lowerarm"), b("hand")
                out.append(("forearm_" + side, E, E.lerp(H, 0.5), fit["forearm_elbow"]))
                out.append(("forearmw_" + side, E.lerp(H, 0.5), H, fit["forearm_wrist"]))
                out.append(("palm_" + side, H, f("middle", "01"), fit["palm"]))
                out.append(("palmi_" + side, H, f("index", "01"), fit.get("palm_index", fit["palm"])))
                out.append(("palmp_" + side, H, f("pinky", "01"), fit.get("palm_pinky", fit["palm"])))
            else:
                out.append(("forearm_" + side, b("lowerarm"), b("hand"), RADIUS["forearm"]))
                out.append(("palm_" + side, b("hand"), f("middle", "01"), RADIUS["palm"]))
            for fi in FINGERS:
                pts = [f(fi, "01"), f(fi, "02"), f(fi, "03"), self._tip(fi, side)]
                rf = fit.get("finger_" + fi, RADIUS["finger"]) if fit else RADIUS["finger"]
                for i in range(3):
                    out.append(("%s%d_%s" % (fi, i + 1, side), pts[i], pts[i + 1], rf))
        return out

    def _fitted_radii(self):
        """The capsules' radii fitted to this rig's skin when its rigs entry asks (fit_capsules), measured once on
        the rest pose: the forearm's mean radius at a quarter and three quarters of its length, the palm's half
        thickness at its middle, each finger's half thickness at its middle segment (m). None when not asked."""
        if not self.R.get("fit_capsules"):
            return None
        key = (self.rig, id(self.arm))
        if getattr(self, "_fit", None) and self._fit[0] == key:
            return self._fit[1]
        self._fit = (key, {})                       # guards the measure against calling itself
        self.snapshot("save", "_fit_caps")
        for pb in self.arm.pose.bones:
            pb.matrix_basis = Matrix.Identity(4)
        bpy.context.view_layer.update()
        try:
            fit = self._measure_radii()
        finally:
            self.snapshot("load", "_fit_caps")
            bpy.context.view_layer.update()
        self._fit = (key, fit)
        return fit

    def _measure_radii(self):
        pts = self._skin_points()
        got = {}
        for s_ in ("l", "r"):
            B = lambda n: self._bw(n).translation     # noqa: E731
            E, W = B(self._n("lowerarm", s_)), B(self._n("hand", s_))
            K = B(self._n("finger", s_, "middle", "01"))
            I, Pk = B(self._n("finger", s_, "index", "01")), B(self._n("finger", s_, "pinky", "01"))
            mine = [(p, g) for p, g in pts if g.endswith("_" + s_)]
            arm = [p for p, g in mine if g.startswith("lowerarm") or g.startswith("hand")]
            d = (W - E).normalized()
            u = d.orthogonal().normalized()
            v2 = d.cross(u)
            for name, t in (("forearm_elbow", 0.25), ("forearm_wrist", 0.75)):
                cc = E.lerp(W, t)
                sl = [p for p in arm if abs((p - cc).dot(d)) < 0.005 and ((p - cc) - d * (p - cc).dot(d)).length < 0.08]
                if len(sl) >= 8:
                    got.setdefault(name, []).append(self._hull_perimeter([((p - cc).dot(u), (p - cc).dot(v2)) for p in sl]) / (2 * math.pi))
            along = (K - W).normalized()
            across = (I - Pk) - along * (I - Pk).dot(along)
            across.normalize()
            nrm = across.cross(along).normalized()
            pm = W.lerp(K, 0.5)
            pt = [(p - pm).dot(nrm) for p, g in mine if (g.startswith("hand") or "metacarpal" in g)
                  and abs((p - pm).dot(along)) < 0.006 and abs((p - pm).dot(across)) < 0.015]
            if pt:
                got.setdefault("palm", []).append((max(pt) - min(pt)) / 2)
            # the palm's edges are thinner than its middle: each side spoke its own half thickness at its middle
            for name, Kx in (("palm_index", I), ("palm_pinky", Pk)):
                ax = (Kx - W).normalized()
                cm_ = W.lerp(Kx, 0.5)
                side_ = nrm.cross(ax).normalized()
                pe = [(p - cm_).dot(nrm) for p, g in mine if (g.startswith("hand") or "metacarpal" in g)
                      and abs((p - cm_).dot(ax)) < 0.006 and abs((p - cm_).dot(side_)) < 0.01]
                if pe:
                    got.setdefault(name, []).append((max(pe) - min(pe)) / 2)
            for fi in FINGERS:
                a2, a3 = B(self._n("finger", s_, fi, "02")), B(self._n("finger", s_, fi, "03"))
                fd = (a3 - a2).normalized()
                fc = a2.lerp(a3, 0.5)
                ring = [p - fc - fd * (p - fc).dot(fd) for p, g in mine if g.startswith(fi) and abs((p - fc).dot(fd)) < 0.003]
                if len(ring) >= 6:
                    uu = fd.orthogonal().normalized()
                    vv = fd.cross(uu)
                    got.setdefault("finger_" + fi, []).append(
                        self._hull_perimeter([(r.dot(uu), r.dot(vv)) for r in ring]) / (2 * math.pi))
        base = {"forearm_elbow": RADIUS["forearm"], "forearm_wrist": RADIUS["forearm"], "palm": RADIUS["palm"],
                "palm_index": RADIUS["palm"], "palm_pinky": RADIUS["palm"]}
        base.update({"finger_" + fi: RADIUS["finger"] for fi in FINGERS})
        return {k: (max(got[k]) if got.get(k) else v) for k, v in base.items()}

    class Surface:
        """The rifle's surface: the closest point on its faces and the signed distance to it, negative inside. The
        nearest corner point (a KD tree of vertices) read a hand 0.68 cm into a low-poly magazine that it was 1.68 cm
        into: a box's flat face has no vertex inside it. Inside or out is decided by ray parity (an odd count of
        crossings, the majority of three rays), not by the nearest face's normal, which reads wrong near an edge."""
        # seven rays in uneven directions, the majority deciding (0.4.0): with three, a ray grazing a thin open part (a
        # trigger, a guard) flipped the answer for a point moved 0.0001 cm, and a finger read 1.46 cm in or 0.04 out
        RAYS = tuple(Vector(v).normalized() for v in ((1.0, 0.13, 0.07), (-0.11, 1.0, 0.05), (0.09, -0.04, 1.0),
                                                       (-0.71, -0.29, 0.53), (0.37, -0.83, -0.31), (-0.23, 0.41, -0.88),
                                                       (0.61, 0.57, -0.47)))

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
            return sum(self._crossings(p, d) % 2 for d in self.RAYS) >= 4

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

    def _grasp(self, s, tree, contact_cm=0.3, friction=0.5, near=None, near_cm=None):
        """Whether the hand pinches the rifle between the thumb and a finger (an antipodal grasp): each digit's point
        nearest the rifle (its two outer segments), touching when its skin is within contact_cm of the surface, and
        the direction from the surface to it. Thumb and finger hold the part between them when those directions are
        at least 180 - 2 atan(friction) degrees apart: each pad pushes inside its friction cone (skin on metal or
        polymer, friction about 0.5: 127 degrees). Returns (grasping, the widest thumb to finger angle or None, the
        digits touching). near, near_cm: only the rifle's surface within near_cm of the point near counts (a hand that
        must pinch one part, not the rifle anywhere)."""
        need = 180.0 - 2.0 * math.degrees(math.atan(friction))
        segs = {x[0]: x for x in self._segments(s)}
        dirs = {}
        for fi in FINGERS:
            best = None
            for j in (2, 3):
                seg = segs.get("%s%d_%s" % (fi, j, s))
                if seg is None:
                    continue
                _n, a, b, r = seg
                n = max(2, int((b - a).length / 0.003))
                for i in range(n + 1):
                    p = a.lerp(b, i / n)
                    q, d = tree.nearest(p)
                    if q is None or (near is not None and (q - near).length * 100.0 > near_cm):
                        continue
                    gap = (d - r) * 100.0
                    if best is None or gap < best[0]:
                        u = (p - q) if d >= 0.0 else (q - p)
                        best = (gap, u.normalized() if u.length > 1e-9 else None)
            if best is not None and best[0] <= contact_cm and best[1] is not None:
                dirs[fi] = best[1]
        touching = sorted(dirs)
        if "thumb" not in dirs:
            return False, None, touching
        angles = [math.degrees(dirs["thumb"].angle(v)) for f, v in dirs.items() if f != "thumb"]
        if not angles:
            return False, None, touching
        top = max(angles)
        return top >= need, round(top), touching

    def grip(self, side="both", back_max_cm=0.1, palm_max_cm=1.0, hold_within_cm=1.0, palm_within_cm=1.5, hold=False,
             frame="gun", grasp=False, contact_cm=0.3, friction=0.5):
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
        hold true: a hand that is not holding breaks a rule. grasping: the thumb and a finger pinch the rifle from
        opposite sides (_grasp: each within contact_cm, their directions from the surface at least 180 - 2 atan(friction)
        degrees apart; grasp_opposition_deg the widest thumb to finger angle, digits_touching the digits on the rifle).
        grasp true: a hand that is not grasping breaks a rule (a hand meant to hold a part, not one resting on it)."""
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
            _pr = (self._fitted_radii() or {}).get("palm", RADIUS["palm"])
            _pc, palm_d = tree.nearest(centre + palm_dir.normalized() * _pr) if palm_dir.length > 1e-9 else (None, None)
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
            grasping, opp, digits = self._grasp(s_, tree, contact_cm, friction)
            if grasp and not grasping:
                need = 180.0 - 2.0 * math.degrees(math.atan(friction))
                why = ("the thumb is off it (more than %.1f cm)" % contact_cm if "thumb" not in digits else
                       "no finger on it with the thumb" if opp is None else
                       "the thumb and fingers do not oppose (%d degrees apart, at least %.0f)" % (opp, need))
                bad.append("does not grasp the rifle: " + why)
            if palm_cm > palm_max_cm:
                bad.append("rifle %.2f cm inside the palm side of %s (at most %.2f)" % (palm_cm, palm_seg, palm_max_cm))
            if back_cm > back_max_cm:
                bad.append("rifle %.2f cm inside the back of %s (at most %.2f)" % (back_cm, back_seg, back_max_cm))
            rep_ = {"holding": holding, "touching": touching, "gap_cm": round(near, 2) if near is not None else None,
                    "palm_gap_cm": round(palm_gap, 2) if palm_gap is not None else None,
                    "palm_faces_rifle": round(facing, 2), "palm_contact_cm": round(palm_cm, 2),
                    "back_contact_cm": round(back_cm, 2), "grasping": grasping, "grasp_opposition_deg": opp,
                    "digits_touching": digits, "bad": bad}
            if back_seg:
                rep_["back_contact"] = {"segment": back_seg, "rifle_point": r2(self._to(back_pt, frame), 1)}
            out[s_] = rep_
            out["ok"] = out["ok"] and not bad
        return out

    # --- what each hand is for (0.8.0) ---------------------------------------------------------------------------
    ACTS = ("hold", "pull", "press", "rest", "reach")

    def purpose(self, targets, target_cm=3.0, contact_cm=0.3, friction=0.5, curl_free_deg=120.0, free_gap_cm=2.0,
                hook_curl_deg=60.0, palm_within_cm=1.5, frame="gun"):
        """Every hand in an action works on a target: it holds, pulls, presses or rests on a part, or reaches toward
        one. targets: {"l": {"point": a rig point or [x, y, z], "act": hold|pull|press|rest|reach, "within_cm": the
        part's reach from the point (default target_cm)}, "r": {...}}; a hand with no target breaks a rule. The part is
        the rifle's surface within within_cm of the point; a rig point on a moving part rides with it. hold: the thumb
        and a finger pinch that part (grip's grasp rule, kept to the part), or the hand wraps it: the palm within
        palm_within_cm of it and two fingers hooked on it (a power grip on a pistol grip or a handguard). pull: a pinch, or a hook: a finger whose
        middle and end joints curl at least hook_curl_deg together with its pad on the part (a charging handle pulled
        with two fingers, no thumb). press: a digit touches it (within
        contact_cm). rest: some part of the hand touches it. reach: no contact yet. Every act: a finger (index to
        pinky) curled more than curl_free_deg in all (its three joints) must have its pad within free_gap_cm of the
        rifle; a curl around nothing breaks a rule (a hand reaching to grasp opens, it does not keep a fist). For each
        hand: target_gap_cm (the hand's skin to the point), digits_at_target, grasping, fingers (curl_total, pad_gap_cm),
        and bad."""
        self._need()
        if not isinstance(targets, dict):
            raise ValueError('targets: {"l": {"point": ..., "act": ...}, "r": {...}}')
        tree = self._tree()
        _L, FL = self._limits()
        need = 180.0 - 2.0 * math.degrees(math.atan(friction))
        out = {"ok": True}
        for s_ in ("l", "r"):
            t = targets.get(s_)
            bad, rep_ = [], {}
            segs = [x for x in self._segments(s_) if not x[0].startswith("forearm")]
            if t is not None:
                act = t.get("act")
                if act not in self.ACTS:
                    raise ValueError("act must be one of %s, got %r" % (", ".join(self.ACTS), act))
                name = t.get("point")
                within = float(t.get("within_cm", target_cm))
                p = self._point(name, frame)
                label = ("the " + name) if isinstance(name, str) else "the point %s" % (list(name),)
                gap = min(self._seg_point_cm(a, b, r, p) for _n, a, b, r in segs)
                grasping, opp, digits = self._grasp(s_, tree, contact_cm, friction, near=p, near_cm=within)
                rest = any(g is not None and g <= contact_cm for g in
                           (self._seg_gap(tree, a, b, r, p, within) for _n, a, b, r in segs))
                curls0, _b0, _p0 = self._finger_report(s_, FL)
                hooks = [fi for fi in ("index", "middle", "ring", "pinky") if fi in digits
                         and curls0["%s_02" % fi][0] + curls0["%s_03" % fi][0] >= hook_curl_deg]
                palm = [self._seg_gap(tree, a, b, r, p, within) for n, a, b, r in segs if n == "palm_" + s_]
                palm_gap = palm[0] if palm and palm[0] is not None else None
                wrapping = palm_gap is not None and palm_gap <= palm_within_cm and len(hooks) >= 2
                rep_.update({"target": name, "act": act, "target_gap_cm": round(gap, 2), "digits_at_target": digits,
                             "grasping": grasping, "grasp_opposition_deg": opp, "hooking": hooks, "wrapping": wrapping,
                             "palm_gap_cm": None if palm_gap is None else round(palm_gap, 2)})
                if act == "pull" and not grasping and not hooks:
                    bad.append("does not pull %s: no pinch and no finger hooked on it (digits on it: %s; the hand's skin "
                               "%.1f cm from it)" % (label, ", ".join(digits) or "none", gap))
                elif act == "hold" and not grasping and not wrapping:
                    pinch = ("the thumb is not on it" if "thumb" not in digits else
                             "no finger on it with the thumb" if opp is None else
                             "the thumb and fingers %d degrees apart (at least %.0f)" % (opp, need))
                    wrap = "the palm %s, %d fingers hooked on it (2 needed)" % (
                        "off it" if palm_gap is None else "%.1f cm from it (at most %.1f)" % (palm_gap, palm_within_cm), len(hooks))
                    bad.append("does not hold %s: no pinch (%s) and no wrap (%s)" % (label, pinch, wrap))
                elif act == "press" and not digits:
                    bad.append("presses nothing at %s (no digit within %.1f cm of it; the hand %.1f cm away)"
                               % (label, contact_cm, gap))
                elif act == "rest" and not rest:
                    bad.append("does not rest on %s (the hand %.1f cm from it)" % (label, gap))
            else:
                bad.append("has no target: every hand in an action holds, pulls, presses, rests on or reaches for a part")
            curls, _b, _pl = self._finger_report(s_, FL)
            fingers = {}
            for fi in ("index", "middle", "ring", "pinky"):
                total = sum(curls["%s_%s" % (fi, j)][0] for j in ("01", "02", "03"))
                pads = [self._seg_gap(tree, a, b, r) for n, a, b, r in segs if n in ("%s2_%s" % (fi, s_), "%s3_%s" % (fi, s_))]
                pad = min(g for g in pads if g is not None) if any(g is not None for g in pads) else None
                fingers[fi] = {"curl_total": total, "pad_gap_cm": None if pad is None else round(pad, 2)}
                if total > curl_free_deg and pad is not None and pad > free_gap_cm:
                    bad.append("%s curled %d around nothing (its pad %.1f cm from the rifle, at most %.1f)"
                               % (fi, total, pad, free_gap_cm))
            rep_.update({"fingers": fingers, "bad": bad})
            out[s_] = rep_
            out["ok"] = out["ok"] and not bad
        return out

    @staticmethod
    def _seg_point_cm(a, b, r, p):
        """The skin of a capsule (a to b, radius r) to the point p, cm."""
        ab = b - a
        t = 0.0 if ab.length_squared < 1e-12 else max(0.0, min(1.0, (p - a).dot(ab) / ab.length_squared))
        return ((a + ab * t) - p).length * 100.0 - r * 100.0

    @staticmethod
    def _seg_gap(tree, a, b, r, near=None, near_cm=None):
        """The capsule's skin to the rifle's surface, cm (below 0 inside), sampled every 3 mm; with near, only the
        surface within near_cm of that point counts. None when nothing counts."""
        n = max(2, int((b - a).length / 0.003))
        best = None
        for i in range(n + 1):
            q, d = tree.nearest(a.lerp(b, i / n))
            if q is None or (near is not None and (q - near).length * 100.0 > near_cm):
                continue
            g = (d - r) * 100.0
            best = g if best is None else min(best, g)
        return best

    # --- the grasp solver (0.8.0): the whole hand placed against the gun and the other hand -------------------------
    @staticmethod
    def _seg_seg(a1, b1, a2, b2):
        """The closest distance between two segments (m)."""
        d1, d2, r = b1 - a1, b2 - a2, a1 - a2
        a, e, f = d1.dot(d1), d2.dot(d2), d2.dot(r)
        if a <= 1e-12 and e <= 1e-12:
            return r.length
        if a <= 1e-12:
            s, t = 0.0, max(0.0, min(1.0, f / e))
        else:
            c = d1.dot(r)
            if e <= 1e-12:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            else:
                b = d1.dot(d2)
                den = a * e - b * b
                s = max(0.0, min(1.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
                t = (b * s + f) / e
                if t < 0.0:
                    t, s = 0.0, max(0.0, min(1.0, -c / a))
                elif t > 1.0:
                    t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
        return ((a1 + d1 * s) - (a2 + d2 * t)).length

    def hand_presses(self, side, tree=None, glove_cm=0.0, allow_cm=0.2, palm_allow_cm=0.4, other=True, soft=None):
        """How far the hand presses past what is allowed, into the gun and into the other hand, the glove's thickness
        counted on every capsule: (penalty, the worst presses [(segment, what, excess cm)]). The palm may press
        palm_allow_cm (it holds the gun); soft: {segment prefix: cm} a style allows. The forearm counts against the gun
        (a hand over the slide must not lay its forearm on the gun), not against the other hand."""
        tree = tree or self._tree()
        g = glove_cm / 100.0
        mine = list(self._segments(side))
        theirs = [x for x in self._segments({"l": "r", "r": "l"}[side]) if not x[0].startswith("forearm")] if other else []
        pen, worst = 0.0, []
        for name, a, b, r in mine:
            allow = palm_allow_cm if name.startswith("palm") else next((v for k, v in (soft or {}).items() if name.startswith(k)), allow_cm)
            n = max(2, int((b - a).length / 0.004))
            deep = 0.0
            for i in range(n + 1):
                q, dq = tree.nearest(a.lerp(b, i / n))
                if q is not None and dq < r + g:
                    deep = max(deep, (r + g - dq) * 100.0)
            if deep > allow:
                pen += 2.0 * (deep - allow) + (deep - allow) ** 2        # a small press counts too
                worst.append((name, "gun", round(deep - allow, 2)))
            if name.startswith("forearm"):
                continue
            # skin on skin gives: a hand may press the other 0.5 cm (the support palm on the firing fingers' tips, 0.8)
            skin = max(allow, 0.8 if name.startswith("palm") else 0.5)
            for oname, oa, ob, orr in theirs:
                d = self._seg_seg(a, b, oa, ob)
                into = (r + orr + 2 * g - d) * 100.0
                if into > skin:
                    pen += 2.0 * (into - skin) + (into - skin) ** 2
                    worst.append((name, oname, round(into - skin, 2)))
        worst.sort(key=lambda w: -w[2])
        return pen, worst[:4]

    def _digit_gap(self, side, fi, tree, glove_cm=0.0, other=True, skip_base=False):
        """A digit's skin (and glove) to the nearest of the gun and the other hand, cm (below 0 when inside).
        skip_base: the knuckle's segment left out (curling cannot move it off the gun; the hand's place must)."""
        g = glove_cm / 100.0
        theirs = [x for x in self._segments({"l": "r", "r": "l"}[side]) if not x[0].startswith("forearm")] if other else []
        best = 99.0
        for name, a, b, r in self._segments(side):
            if not name.startswith(fi):
                continue
            if skip_base and name[len(fi):len(fi) + 1] == "1":
                continue
            n = max(2, int((b - a).length / 0.004))
            for i in range(n + 1):
                q, dq = tree.nearest(a.lerp(b, i / n))
                if q is not None:
                    best = min(best, (dq - r - g) * 100.0)
            for _o, oa, ob, orr in theirs:
                best = min(best, (self._seg_seg(a, b, oa, ob) - r - orr - 2 * g) * 100.0)
        return best

    def _set_curl(self, side, fi, j, target, FL):
        rep_, _b, planes = self._finger_report(side, FL)
        cur = rep_["%s_%s" % (fi, j)][0]
        if abs(cur - target) < 0.5:
            return
        n = self._n("finger", side, fi, j)
        head, M = self._bw(n).translation.copy(), self._bw(n)
        for sg in (1.0, -1.0):
            self._turn_about(n, planes[fi], (target - cur) * sg, head)
            if abs(self._finger_report(side, FL)[0]["%s_%s" % (fi, j)][0] - target) < abs(cur - target):
                return
            self._set_world(n, M)

    def _set_thumb_bend(self, side, b2, b3, TL):
        for j, target in (("02", b2), ("03", b3)):
            tr, _b, axes = self._thumb_report(side, TL)
            if "thumb_" + j not in tr or j not in axes:
                continue
            cur = tr["thumb_" + j][0]
            n = self._n("finger", side, "thumb", j)
            head, M = self._bw(n).translation.copy(), self._bw(n)
            for sg in (1.0, -1.0):
                self._turn_about(n, axes[j], (target - cur) * sg, head)
                if abs(self._thumb_report(side, TL)[0]["thumb_" + j][0] - target) < abs(cur - target):
                    break
                self._set_world(n, M)
        for _ in range(2):                              # back into the hinges' plane
            for j in ("02", "03"):
                tr, _b, axes = self._thumb_report(side, TL)
                if "thumb_" + j not in tr or j not in axes or abs(tr["thumb_" + j][1]) < 3.0:
                    continue
                n = self._n("finger", side, "thumb", j)
                head = self._bw(n).translation.copy()
                tip = self._bw(self._n("finger", side, "thumb", "03")).translation if j == "02" else self._finger_tip("thumb", side)
                w = (tip - head).cross(axes[j])
                if w.length < 1e-9:
                    continue
                w.normalize()
                M, sd = self._bw(n), tr["thumb_" + j][1]
                for sg in (1.0, -1.0):
                    self._turn_about(n, w, -sd * sg, head)
                    if abs(self._thumb_report(side, TL)[0]["thumb_" + j][1]) < abs(sd):
                        break
                    self._set_world(n, M)

    def close_hand(self, side, plan, tree=None, glove_cm=0.0, other=True, steps=12, from_pose=False, rules=None):
        """The digits posed by a plan {"index": {"pose": "straight", "curl": [a, b, c]} | {"pose": "wrap",
        "max_curl": [a, b, c]} | {"pose": "rest"}, ..., "thumb": ...}: straight ones set, wrapped ones closed a step
        at a time until their skin (and glove) meets the gun or the other hand, a step into either taken back; then
        the joints mended back in range and into their fingers' planes. from_pose: each wrapped finger starts from
        its curl now (a hold already placed): opened a step at a time while it is in the gun or the other hand, else
        closed until it meets either. rules: contact rules; a wrapped finger with a rule of its own closes to the
        step that best meets that rule (its pad laid on its region), pressing no deeper than allowed, instead of
        stopping at its first touch."""
        tree = tree or self._tree()
        _L, FL = self._limits()
        TL = self._thumb_limits()
        self._wraps = {}           # each wrapped digit: how far it closed (0 to 1 of its max) before it met something
        for fi in ("index", "middle", "ring", "pinky"):
            f = (plan or {}).get(fi)
            if not f or f.get("pose") == "rest":
                continue
            if f["pose"] == "straight":
                for j, c in zip(("01", "02", "03"), f["curl"]):
                    self._set_curl(side, fi, j, c, FL)
                continue
            if from_pose:
                rep0 = self._finger_report(side, FL)[0]
                mx = f["max_curl"]
                fr = [(rep0["%s_%s" % (fi, j)][0] - 4.0) / (m - 4.0) if abs(m - 4.0) > 1e-6 else 0.0
                      for j, m in zip(("01", "02", "03"), mx)]
                t = min(1.0, max(0.0, sum(fr) / 3.0))
                at_t = lambda t_: [4.0 + (m - 4.0) * t_ for m in mx]      # noqa: E731
                for j, c in zip(("01", "02", "03"), at_t(t)):
                    self._set_curl(side, fi, j, c, FL)
                gap = self._digit_gap(side, fi, tree, glove_cm, other, skip_base=True)
                if gap < -0.1:
                    while t > 0.0 and gap < -0.1:
                        t = max(0.0, t - 1.0 / steps)
                        for j, c in zip(("01", "02", "03"), at_t(t)):
                            self._set_curl(side, fi, j, c, FL)
                        gap = self._digit_gap(side, fi, tree, glove_cm, other, skip_base=True)
                else:
                    while t < 1.0 and gap > 0.05:
                        t2 = min(1.0, t + 1.0 / steps)
                        for j, c in zip(("01", "02", "03"), at_t(t2)):
                            self._set_curl(side, fi, j, c, FL)
                        g2 = self._digit_gap(side, fi, tree, glove_cm, other, skip_base=True)
                        if g2 < -0.1:
                            for j, c in zip(("01", "02", "03"), at_t(t)):
                                self._set_curl(side, fi, j, c, FL)
                            break
                        t, gap = t2, g2
                self._wraps[fi] = t
                continue
            mine = [r_ for r_ in (rules or []) if r_["patch"].split(":")[0] == fi]
            if mine:
                vv = self._gun_verts()
                best_ = None
                for k in range(0, steps + 1):
                    c3 = [4.0 + (m - 4.0) * k / steps for m in f["max_curl"]]
                    for j, c in zip(("01", "02", "03"), c3):
                        self._set_curl(side, fi, j, c, FL)
                    gap = self._digit_gap(side, fi, tree, glove_cm, other)
                    err = sum(self._rule_eval(side, r_, glove_cm, vv)["error"] for r_ in mine)
                    sc_ = err + 4.0 * max(0.0, -gap - 0.15)
                    if best_ is None or sc_ < best_[0] - 1e-6:
                        best_ = (sc_, k, c3)
                for j, c in zip(("01", "02", "03"), best_[2]):
                    self._set_curl(side, fi, j, c, FL)
                self._wraps[fi] = best_[1] / steps
                continue
            prev = [4.0, 4.0, 4.0]
            for j, c in zip(("01", "02", "03"), prev):
                self._set_curl(side, fi, j, c, FL)
            done = 1.0
            for k in range(1, steps + 1):
                c3 = [4.0 + (m - 4.0) * k / steps for m in f["max_curl"]]
                for j, c in zip(("01", "02", "03"), c3):
                    self._set_curl(side, fi, j, c, FL)
                gap = self._digit_gap(side, fi, tree, glove_cm, other)
                if gap <= 0.05:
                    done = (k - (1 if gap < -0.1 else 0)) / steps
                    if gap < -0.1:
                        for j, c in zip(("01", "02", "03"), prev):
                            self._set_curl(side, fi, j, c, FL)
                    break
                prev = c3
            self._wraps[fi] = done
        t = (plan or {}).get("thumb")
        if t and t.get("pose") == "to" and t.get("to_world") is not None and t.get("via_world") is not None:
            self._thumb_via(side, Vector(t["via_world"]), Vector(t["to_world"]), TL)
        elif t and t.get("pose") == "to" and t.get("to_world") is not None:
            # the base turned toward the point (it moves freely within its rules); the knuckle and end joint bent
            # only in their hinges (a grid of bends, the one that brings the tip nearest); the base once more
            tgt = Vector(t["to_world"])
            self._set_thumb_bend(side, 10.0, 6.0, TL)
            self._thumb_to(side, tgt, tree, glove_cm, other, joints=("01",), guard=False)
            for _round in range(2):
                best = None
                for b2 in (0.0, 15.0, 30.0, 45.0):
                    for b3 in (0.0, 20.0, 40.0, 60.0):
                        self._set_thumb_bend(side, b2, b3, TL)
                        dd = (self._finger_tip("thumb", side) - tgt).length
                        if best is None or dd < best[0]:
                            best = (dd, b2, b3)
                self._set_thumb_bend(side, best[1], best[2], TL)
                self._thumb_to(side, tgt, tree, glove_cm, other, joints=("01",), guard=False)
        elif t and t.get("pose") == "straight":
            self._set_thumb_bend(side, t["curl"][1], t["curl"][2], TL)
        elif t and t.get("pose") == "wrap":
            prev = (6.0, 4.0)
            self._set_thumb_bend(side, *prev, TL)
            for k in range(1, steps + 1):
                b = (6.0 + (t["max_curl"][1] - 6.0) * k / steps, 4.0 + (t["max_curl"][2] - 4.0) * k / steps)
                self._set_thumb_bend(side, *b, TL)
                gap = self._digit_gap(side, "thumb", tree, glove_cm, other)
                if gap <= 0.05:
                    if gap < -0.1:
                        self._set_thumb_bend(side, *prev, TL)
                    break
                prev = b
        self._mend_hand(side)
        self._end_joints_flat(side, FL)
        self._thumb_base_clear(side, tree, glove_cm, other)
        bpy.context.view_layer.update()

    def _end_joints_flat(self, side, FL):
        """Each finger's end joint turned back into its finger's plane (a wrap can twist it; the mend straightens the
        knuckle and the middle joint only)."""
        for _ in range(4):
            rep_, _b, planes = self._finger_report(side, FL)
            for fi in ("index", "middle", "ring", "pinky"):
                sd = rep_["%s_03" % fi][1]
                if abs(sd) < FL["03"][2] - 3.0:
                    continue
                n = self._n("finger", side, fi, "03")
                head = self._bw(n).translation.copy()
                axis = (self._finger_tip(fi, side) - head).cross(planes[fi])
                if axis.length < 1e-9:
                    continue
                axis.normalize()
                M = self._bw(n)
                kept = False
                for f in (1.0, 0.6):
                    for sg in (1.0, -1.0):
                        self._turn_about(n, axis, -sd * f * sg, head)
                        if abs(self._finger_report(side, FL)[0]["%s_03" % fi][1]) < abs(sd) - 1.0:
                            kept = True
                            break
                        self._set_world(n, M)
                    if kept:
                        break
        # an end joint still twisted is straightened: a straight end joint has no plane to leave
        rep_ = self._finger_report(side, FL)[0]
        for fi in ("index", "middle", "ring", "pinky"):
            if abs(rep_["%s_03" % fi][1]) > FL["03"][2]:
                self._set_curl(side, fi, "03", 2.0, FL)

    def _thumb_via(self, side, via, target, TL, rounds=24, step_deg=6.0):
        """The thumb laid along a path, as it wraps a pistol's backstrap: its base joint turned (a few degrees at a
        time) until its knuckle (the thumb's MCP) sits on the waypoint via; then its knuckle and end joint bent in
        their hinges (a grid) and the base nudged so the tip comes nearest the target. Returns the distances left."""
        n1 = self._n("finger", side, "thumb", "01")
        n2 = self._n("finger", side, "thumb", "02")
        for _ in range(rounds):
            head = self._bw(n1).translation.copy()
            a, b = self._bw(n2).translation - head, via - head
            if a.length < 1e-6 or b.length < 1e-6:
                break
            ang = math.degrees(a.angle(b))
            if ang < 0.5:
                break
            self._turn_about(n1, a.cross(b).normalized(), min(step_deg, ang), head)
            bpy.context.view_layer.update()
        best = None
        for b2 in (-5.0, 5.0, 15.0, 25.0, 35.0, 45.0, 55.0):
            for b3 in (-10.0, 0.0, 15.0, 30.0, 45.0, 60.0):
                self._set_thumb_bend(side, b2, b3, TL)
                dd = (self._finger_tip("thumb", side) - target).length
                if best is None or dd < best[0]:
                    best = (dd, b2, b3)
        self._set_thumb_bend(side, best[1], best[2], TL)
        bpy.context.view_layer.update()
        return {"via_left_cm": round((self._bw(n2).translation - via).length * 100, 2), "tip_left_cm": round(best[0] * 100, 2)}

    def _thumb_to(self, side, target, tree, glove_cm=0.0, other=True, rounds=14, step_deg=8.0, joints=("03", "02", "01"),
                  guard=True):
        """The thumb's tip brought to a rest point (the firing thumb on the safety): its joints, tip to base, each
        turned about its head toward the point by at most step_deg a round (cyclic coordinate descent); a turn that
        breaks a thumb rule, or puts the thumb deeper into the gun or the other hand, is undone. Returns the tip's
        distance left, cm."""
        TL = self._thumb_limits()
        rules0 = len(self._thumb_report(side, TL)[1])
        for _ in range(rounds):
            moved = False
            for j in joints:
                n = self._n("finger", side, "thumb", j)
                head = self._bw(n).translation.copy()
                a, b = self._finger_tip("thumb", side) - head, target - head
                if a.length < 1e-6 or b.length < 1e-6:
                    continue
                axis = a.cross(b)
                ang = min(step_deg, math.degrees(a.angle(b)))
                if axis.length < 1e-9 or ang < 0.5:
                    continue
                M = self._bw(n)
                g0 = self._digit_gap(side, "thumb", tree, glove_cm, other)
                self._turn_about(n, axis.normalized(), ang, head)
                if (len(self._thumb_report(side, TL)[1]) > rules0
                        or guard and self._digit_gap(side, "thumb", tree, glove_cm, other) < min(g0, 0.0) - 0.05):
                    self._set_world(n, M)
                    continue
                moved = True
            if not moved:
                break
        return (self._finger_tip("thumb", side) - target).length * 100.0

    # --- contact rules: which skin touches which surface, measured ----------------------------------------------
    PATCH_SEGS = {"pad": ("2", "3"), "tip": ("3",), "base": ("1",), "whole": ("1", "2", "3")}

    def _patch_points(self, side, patch, glove_cm=0.0, n=5):
        """A named patch of the hand's skin as surface points with their outward normals (world): a digit's pad
        (the palm side of its middle and end segments), tip (the end segment's pad), base, back, whole; the palm
        (its palm side), heel (the palm side near the wrist), web (the thumb's first segment). Each point lies on
        the capsule's skin, the glove on it."""
        digit, _, kind = patch.partition(":")
        kind = kind or "pad"
        segs = {x[0]: x for x in self._segments(side)}
        g = glove_cm / 100.0
        across, k = self._hand_frame(side)
        facing = self._palm_facing(side)
        out = []

        def add(a, b, r, nrm_fn, t0=0.0, t1=1.0):
            for i in range(n):
                t = t0 + (t1 - t0) * (i + 0.5) / n
                c = a.lerp(b, t)
                nv = nrm_fn(b - a)
                if nv.length > 1e-9:
                    out.append((c + nv.normalized() * (r + g), nv.normalized()))

        if digit in ("palm", "heel"):
            for nm in ("palm_", "palmi_", "palmp_"):
                x = segs.get(nm + side)
                if x:
                    add(x[1], x[2], x[3], lambda d: facing, 0.0, 0.5 if digit == "heel" else 1.0)
            return out
        if digit == "web":
            x = segs.get("thumb1_" + side)
            if x:
                add(x[1], x[2], x[3], lambda d: (facing + across * (-k)).normalized())
            return out
        if digit == "thumb":
            for j in self.PATCH_SEGS.get(kind, ("2", "3")):
                x = segs.get("thumb%s_%s" % (j, side))
                if not x:
                    continue
                d = (x[2] - x[1]).normalized()
                bn = self._n("finger", side, "thumb", "0" + j if len(j) == 1 else j)
                pad = self._bw(bn).to_quaternion() @ self._thumb_pad_axis(side)
                pad = (pad - d * pad.dot(d))
                pad = pad.normalized() if pad.length > 1e-9 else facing
                add(x[1], x[2], x[3], (lambda d_, p_=pad: -p_) if kind == "back" else (lambda d_, p_=pad: p_))
            return out
        for j in self.PATCH_SEGS.get(kind if kind != "back" else "whole", ("2", "3")):
            x = segs.get("%s%s_%s" % (digit, j, side))
            if not x:
                continue
            pad_fn = (lambda d: (across.cross(d.normalized()) * k)) if kind != "back" else (lambda d: -(across.cross(d.normalized()) * k))
            add(x[1], x[2], x[3], pad_fn)
        return out

    def _gun_bvh(self, polys=None):
        """The gun's evaluated surface as a BVH, all of it or only the given polygons, with their normals."""
        dg = bpy.context.evaluated_depsgraph_get()
        ev = self.rifle_mesh.evaluated_get(dg)
        me = ev.to_mesh()
        M = self.rifle_mesh.matrix_world
        vs = [M @ v.co for v in me.vertices]
        ps = [tuple(p.vertices) for p in me.polygons] if polys is None else [tuple(me.polygons[i].vertices) for i in polys if i < len(me.polygons)]
        ev.to_mesh_clear()
        return BVHTree.FromPolygons(vs, ps) if ps else None, vs

    def _thumb_pad_axis(self, side):
        """The thumb pad's direction in the thumb's end bone's own frame, found once on the idle pose: of the bone's
        six local axes, the one that points most toward the opposing fingers (the middle finger's middle joint), the
        pad of an opposable thumb facing them. It then turns with the bone, its roll included."""
        cache = getattr(self, "_tpa", None) or {}
        if side in cache:
            return cache[side]
        self.snapshot("save", "_tpa")
        self.pose_idle()
        n3 = self._n("finger", side, "thumb", "03")
        M = self._bw(n3)
        c = M.translation.lerp(self._finger_tip("thumb", side), 0.5)
        d = (self._finger_tip("thumb", side) - M.translation).normalized()
        want = self._bw(self._n("finger", side, "middle", "02")).translation - c
        want = (want - d * want.dot(d)).normalized()
        q = M.to_quaternion()
        best = None
        for ax in (Vector((1, 0, 0)), Vector((-1, 0, 0)), Vector((0, 1, 0)), Vector((0, -1, 0)), Vector((0, 0, 1)), Vector((0, 0, -1))):
            w = q @ ax
            sc = w.dot(want)
            if best is None or sc > best[0]:
                best = (sc, ax)
        self.snapshot("load", "_tpa")
        bpy.context.view_layer.update()
        cache[side] = best[1]
        self._tpa = cache
        return best[1]

    def _gun_verts(self):
        dg = bpy.context.evaluated_depsgraph_get()
        ev = self.rifle_mesh.evaluated_get(dg)
        me = ev.to_mesh()
        M = self.rifle_mesh.matrix_world
        vs = [M @ v.co for v in me.vertices]
        ps = [tuple(p.vertices) for p in me.polygons]
        ev.to_mesh_clear()
        return vs, ps

    def _rule_eval(self, side, rule, glove_cm=0.0, verts=None):
        """One contact rule measured: the patch's surface point that best meets the region (gap and facing), its
        gap (cm, below 0 inside), the facing angle (the patch's normal against the surface's, degrees), the touch
        point and the surface normal (world), whether the rule is met, and an error (0 when met)."""
        pts = self._patch_points(side, rule["patch"], glove_cm)
        vs, ps = verts or self._gun_verts()
        sub = [ps[i] for i in rule["polys"] if i < len(ps)] if rule.get("polys") is not None else ps
        bvh = BVHTree.FromPolygons(vs, sub) if sub else None
        if not pts or bvh is None:
            return {"id": rule.get("id"), "met": False, "error": 10.0, "why": "no patch or no region"}
        lo, hi = rule.get("gap_cm", [-0.05, 0.2])
        fmax = rule.get("facing_max_deg", 45.0)
        best = None
        good = 0
        for p, nv in pts:
            q, nn, _i, _d = bvh.find_nearest(p)
            if q is None:
                continue
            gap = (p - q).dot(nn) * 100.0
            dist = (p - q).length * 100.0
            gap = gap if abs(gap) > 1e-6 else dist
            if gap > 0:
                gap = dist                          # off the surface: the straight distance to it
            ang = math.degrees(nv.angle(-nn)) if nv.length > 1e-9 else 180.0
            err = max(0.0, gap - hi) + max(0.0, lo - gap) + max(0.0, ang - fmax) / 30.0
            good += 1 if err <= 1e-6 else 0
            if best is None or err < best[0]:
                best = (err, gap, ang, q, nn)
        if best is None:
            return {"id": rule.get("id"), "met": False, "error": 10.0, "why": "region out of reach"}
        err, gap, ang, q, nn = best
        # how much of the patch lies on the region: a pad laid along a surface, not one point of it touching
        share = good / len(pts)
        need_share = rule.get("min_share", 0.0)
        err += 2.0 * max(0.0, need_share - share)
        return {"id": rule.get("id"), "patch": rule["patch"], "met": err <= 1e-6, "error": round(err, 3),
                "gap_cm": round(gap, 2), "facing_deg": round(ang), "share": round(share, 2), "min_share": need_share,
                "touch": q, "normal": nn}

    def _touches(self, sides, glove_cm=0.0, within_cm=0.3):
        """Where the hands touch the gun: each segment's nearest surface point within within_cm of its skin, with
        the gun's outward normal there (world)."""
        bvh, _vs = self._gun_bvh()
        g = glove_cm / 100.0
        out = []
        for s_ in sides:
            for name, a, b, r in self._segments(s_):
                if name.startswith("forearm"):
                    continue
                best = None
                for i in range(5):
                    c = a.lerp(b, (i + 0.5) / 5)
                    q, nn, _i, d = bvh.find_nearest(c)
                    if q is None:
                        continue
                    gap = (d - r - g) * 100.0
                    if best is None or gap < best[0]:
                        best = (gap, q, nn)
                if best and best[0] <= within_cm:
                    out.append((name, best[1], best[2]))
        return out

    def _closure(self, touches, friction=0.5, length_cm=5.0, dirs=150, seed=11):
        """Force closure from touches (point, outward normal): each contact pushes into the gun inside its friction
        cone (6 edges); the wrenches (force, torque over length_cm) must surround the origin: for every direction
        some wrench points along it. Sampled over dirs directions, so a near miss reads as a small margin. Returns
        (closed, margin: the least best support over the directions, 0 when open)."""
        import random as _random
        if len(touches) < 2:
            return False, 0.0
        c0 = sum((t[1] for t in touches), Vector()) / len(touches)
        L_ = length_cm / 100.0
        W = []
        for _n, q, nn in touches:
            t1 = nn.orthogonal().normalized()
            t2 = nn.cross(t1).normalized()
            for k_ in range(6):
                a_ = 2 * math.pi * k_ / 6
                f = (-nn + (t1 * math.cos(a_) + t2 * math.sin(a_)) * friction).normalized()
                tq = (q - c0).cross(f) / L_
                W.append((f.x, f.y, f.z, tq.x, tq.y, tq.z))
        rnd = _random.Random(seed)
        worst = None
        axes = [tuple((1.0 if i == j else 0.0) * sg for i in range(6)) for j in range(6) for sg in (1.0, -1.0)]
        for i in range(dirs + len(axes)):
            if i < len(axes):
                d = axes[i]
            else:
                v = [rnd.gauss(0, 1) for _ in range(6)]
                ln = math.sqrt(sum(x * x for x in v)) or 1.0
                d = tuple(x / ln for x in v)
            sup = max(sum(w[j] * d[j] for j in range(6)) for w in W)
            worst = sup if worst is None or sup < worst else worst
        return worst > 0.0, round(max(0.0, worst), 3)

    def _wrap(self, side_set, polys, glove_cm=0.0, within_cm=0.3):
        """The share of a band of the gun's surface (polygons) the hands cover: each polygon's centre within
        within_cm of a hand's skin."""
        dg = bpy.context.evaluated_depsgraph_get()
        ev = self.rifle_mesh.evaluated_get(dg)
        me = ev.to_mesh()
        M = self.rifle_mesh.matrix_world
        cs = [M @ me.polygons[i].center for i in polys if i < len(me.polygons)]
        ev.to_mesh_clear()
        if not cs:
            return 0.0
        segs = [x for s_ in side_set for x in self._segments(s_) if not x[0].startswith("forearm")]
        g = glove_cm / 100.0
        hit = 0
        for c in cs:
            for _n, a, b, r in segs:
                ab = b - a
                t = 0.0 if ab.length_squared < 1e-12 else max(0.0, min(1.0, (c - a).dot(ab) / ab.length_squared))
                if ((a + ab * t) - c).length - r - g <= within_cm / 100.0:
                    hit += 1
                    break
        return hit / len(cs)

    def contacts(self, side, rules, glove_cm=0.0, friction=0.5, wrap=None, closure_sides=None, frame="gun"):
        """Contact rules on a hold, measured, not looked at. rules: each {id, patch (thumb:pad, middle:tip, palm,
        heel, web, index:back ...), polys (the region: the gun mesh's polygons), gap_cm [low, high] (skin to surface,
        glove counted), facing_max_deg (the patch's normal against the surface's), need (required|optional)}. For
        each: met, the gap, the facing, the touch point and surface normal (in frame). wrap: {polys, min} the share
        of a band of the gun the hand covers. Force closure: whether the touches (friction friction) can hold the gun
        against any slip or twist, with a margin. ok: every required rule met, wrap at least its min, closure."""
        self._need()
        out = {"rules": [], "ok": True}
        verts = self._gun_verts()
        for rule in rules:
            r_ = self._rule_eval(side, rule, glove_cm, verts)
            if "touch" in r_:
                r_["touch"] = r2(self._to(r_["touch"], frame), 2) if frame != "world" else [round(x, 4) for x in r_["touch"]]
                G = self._gun().to_quaternion()
                r_["normal"] = r2(G.inverted() @ r_["normal"], 2) if frame == "gun" else [round(x, 3) for x in r_["normal"]]
            r_["need"] = rule.get("need", "required")
            out["rules"].append(r_)
            if r_["need"] == "required" and not r_["met"]:
                out["ok"] = False
        if wrap:
            w = self._wrap((side,), wrap["polys"], glove_cm)
            out["wrap"] = {"share": round(w, 2), "min": wrap.get("min", 0.5), "met": w >= wrap.get("min", 0.5)}
            out["ok"] = out["ok"] and out["wrap"]["met"]
        tc = self._touches(tuple(closure_sides or (side,)), glove_cm)
        closed, margin = self._closure(tc, friction)
        out["closure"] = {"closed": closed, "margin": margin, "touches": len(tc), "friction": friction}
        out["ok"] = out["ok"] and closed
        return out

    def hold_chain(self, side, rules, plan=None, glove_cm=0.0, other=True, soft=None, thumb_point=None, rounds=8,
                   step_cm=0.5, step_deg=6.0, clavicle_max_deg=16.0):
        """A hold's contact rules met down the arm, as a body does: the digits close toward their rules within their
        joint ranges; where one still falls short (at its limit), the hand moves and turns a step (the wrist inside
        its ranges, the forearm's roll, the elbow and the shoulder following by the arm's reach, the clavicle
        lending up to clavicle_max_deg when the arm is at full stretch); a step is kept only when the rules' error,
        the presses and the arm's and digits' rules broken all fall together. Steps halve when none helps.
        Returns the error before and after, the steps taken, and which limits bind at the end."""
        self._need()
        tree = self._tree()
        Lm, FL = self._limits()
        hn = self._n("hand", side)

        def settle():
            self.close_hand(side, plan, tree, glove_cm, other, rules=rules)
            if thumb_point is not None:
                self._set_thumb_bend(side, 10.0, 6.0, self._thumb_limits())
                self._thumb_to(side, Vector(thumb_point), tree, glove_cm, other, joints=("01",), guard=False)
            vv = self._gun_verts()
            err = sum(self._rule_eval(side, r_, glove_cm, vv)["error"] * (1.0 if r_.get("need", "required") == "required" else 0.3)
                      for r_ in rules)
            pen = self.hand_presses(side, tree, glove_cm, other=other, soft=soft)[0]
            arm, abad = self._arm_report(side, Lm)
            fbad = len(self._finger_report(side, FL)[1]) + len(self._thumb_report(side, self._thumb_limits())[1])
            return 8.0 * err + 25.0 * pen + 3.0 * (len(abad) + fbad), err, abad

        def put(M):
            self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
            cn = "clavicle_" + side
            if cn in self.arm.pose.bones and self._arm_report(side, Lm)[0].get("elbow_bend_deg", 90.0) < Lm["elbow_bend_min_deg"] + 5.0:
                # at full stretch the shoulder blade reaches too, a little at a time
                C = self._bw(cn).translation
                S = self._bw(self._n("upperarm", side)).translation
                ax = (S - C).cross(M.translation - C)
                if ax.length > 1e-9:
                    for deg in (4.0, 8.0, 12.0, clavicle_max_deg):
                        self._turn_about(cn, ax.normalized(), 4.0, C)
                        bpy.context.view_layer.update()
                        self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
                        if self._arm_report(side, Lm)[0].get("elbow_bend_deg", 90.0) >= Lm["elbow_bend_min_deg"] + 5.0:
                            break
            bpy.context.view_layer.update()

        H = self._bw(hn).copy()
        put(H)
        sc0, e0, _b0 = settle()
        best = (sc0, H)
        st_cm, st_deg = step_cm, step_deg
        taken = 0
        axes = (Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1)))
        for _r in range(rounds * 4):
            improved = False
            P = best[1].translation.lerp(self._bw(self._n("finger", side, "middle", "01")).translation, 0.5)
            moves = [Matrix.Translation(a * (sg * st_cm / 100.0)) @ best[1] for a in axes for sg in (1.0, -1.0)]
            moves += [Matrix.Translation(P) @ Quaternion(a, math.radians(sg * st_deg)).to_matrix().to_4x4() @ Matrix.Translation(-P) @ best[1]
                      for a in axes for sg in (1.0, -1.0)]
            for M in moves:
                self.snapshot("save", "_chain")
                put(M)
                sc, e, _b = settle()
                if sc < best[0] - 1e-4:
                    best = (sc, M)
                    improved = True
                    taken += 1
                    self.snapshot("save", "_chain_best")
                self.snapshot("load", "_chain")
            if not improved:
                st_cm, st_deg = st_cm / 2.0, st_deg / 2.0
                if st_cm < 0.06:
                    break
        put(best[1])
        sc1, e1, abad = settle()
        binds = list(abad) + self._finger_report(side, FL)[1] + self._thumb_report(side, self._thumb_limits())[1]
        return {"error_before": round(e0, 3), "error_after": round(e1, 3), "score_before": round(sc0, 2),
                "score_after": round(sc1, 2), "steps": taken, "limits_at_end": binds[:6]}

    def thumb_search(self, side, point, glove_cm=0.0, other=True, frame="world", samples=220, refine=80, seed=3,
                     spread_deg=70.0, rule=None):
        """The thumb laid on a rest point (the firing thumb on the safety) by search: turns of its base joint within
        spread_deg of where it is, each with a few bends of its knuckle and end joint in their hinges; the try whose
        tip lies nearest the point wins, while the thumb presses into the gun or the other hand no deeper than the
        allowance (0.2 cm, glove counted) and breaks no thumb rule. Then refined round itself. Returns the tip's
        distance left (cm), its gap to the gun (cm) and the rules broken."""
        import random as _random
        self._need()
        rnd = _random.Random(seed)
        tgt = Vector(point) if frame == "world" else self._point(point, frame)
        tree = self._tree()
        TL = self._thumb_limits()
        n1 = self._n("finger", side, "thumb", "01")
        head = self._bw(n1).translation.copy()
        M0 = self._bw(n1)
        bends = ((10.0, 6.0), (20.0, 15.0), (30.0, 30.0), (5.0, 40.0))

        def score():
            dd = (self._finger_tip("thumb", side) - tgt).length * 100.0
            if rule is not None:
                dd = 0.1 * dd + 10.0 * self._rule_eval(side, rule, glove_cm)["error"]   # the rule first, the point a guide
            gap = self._digit_gap(side, "thumb", tree, glove_cm, other)
            rules = len(self._thumb_report(side, TL)[1])
            return dd + 6.0 * max(0.0, -gap - 0.05) + 10.0 * rules, dd, gap, rules

        def put(q, b):
            self._set_world(n1, M0)
            R = Matrix.Translation(head) @ q.to_matrix().to_4x4() @ Matrix.Translation(-head) @ M0
            self._set_world(n1, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))
            self._set_thumb_bend(side, b[0], b[1], TL)

        def rand_q(deg):
            ax = Vector((rnd.gauss(0, 1), rnd.gauss(0, 1), rnd.gauss(0, 1))).normalized()
            return Quaternion(ax, math.radians(rnd.uniform(0.0, deg)))

        best = None
        for i in range(samples):
            q = Quaternion() if i == 0 else rand_q(spread_deg)
            for b in bends:
                put(q, b)
                sc = score()
                if best is None or sc[0] < best[0][0]:
                    best = (sc, q, b)
        for _ in range(refine):
            q = rand_q(12.0) @ best[1]
            b = (best[2][0] + rnd.uniform(-8, 8), best[2][1] + rnd.uniform(-10, 10))
            put(q, b)
            sc = score()
            if sc[0] < best[0][0]:
                best = (sc, q, b)
        put(best[1], best[2])
        bpy.context.view_layer.update()
        return {"left_cm": round(best[0][1], 2), "gap_cm": round(best[0][2], 2), "rules": best[0][3],
                "bend": [round(x, 1) for x in best[2]]}

    def _thumb_base_clear(self, side, tree, glove_cm=0.0, other=True, steps=20):
        """The thumb's base joint turned, 3 degrees at a time about three axes, the way that opens the whole thumb's
        gap to the gun and the other hand most, until it is clear; a turn that breaks a thumb rule is not taken (the
        base bone of the thumb sits where its base joint puts it, which no other step turns)."""
        TL = self._thumb_limits()
        n = self._n("finger", side, "thumb", "01")
        for _ in range(steps):
            gap = self._digit_gap(side, "thumb", tree, glove_cm, other)
            if gap >= 0.0:
                return gap
            head = self._bw(n).translation.copy()
            across, _k = self._hand_frame(side)
            d = self._finger_tip("thumb", side) - head
            if d.length < 1e-9:
                return gap
            d.normalize()
            best = None
            M = self._bw(n)
            for axis in (across, d.cross(across).normalized(), d):
                for sg in (1.0, -1.0):
                    self._turn_about(n, axis, 3.0 * sg, head)
                    g2 = self._digit_gap(side, "thumb", tree, glove_cm, other)
                    if not self._thumb_report(side, TL)[1] and (best is None or g2 > best[0]):
                        best = (g2, axis, sg)
                    self._set_world(n, M)
            if best is None or best[0] <= gap + 1e-4:
                return gap
            self._turn_about(n, best[1], 3.0 * best[2], head)
        return self._digit_gap(side, "thumb", tree, glove_cm, other)

    def _lay(self, side, at, normal, toward, thickness_cm):
        """The palm turned to face into the surface (the least turn), turned about its normal so the fingers point
        toward (when given), its middle set thickness_cm off the surface; returns the reach left over (cm)."""
        hn = self._n("hand", side)
        H = self._bw(hn).copy()
        w = H.translation
        mid = self._bw(self._n("finger", side, "middle", "01")).translation
        across, k = self._hand_frame(side)
        P = w.lerp(mid, 0.5)
        facing = (across.cross((mid - w).normalized()) * k).normalized()
        q = facing.rotation_difference(-normal)
        R = Matrix.Translation(P) @ q.to_matrix().to_4x4() @ Matrix.Translation(-P) @ H
        if toward is not None:
            fw = (q @ (mid - w)).normalized()
            want = toward - normal * toward.dot(normal)
            have = fw - normal * fw.dot(normal)
            if want.length > 1e-6 and have.length > 1e-6:
                q2 = have.normalized().rotation_difference(want.normalized())
                R = Matrix.Translation(P) @ q2.to_matrix().to_4x4() @ Matrix.Translation(-P) @ R
        R = Matrix.Translation(at + normal * (thickness_cm / 100.0) - P) @ R
        return self._put_hand(side, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))

    def _hand_score(self, side, tree, glove_cm, other, soft, short, margin_deg=0.0):
        Lm, FL = self._limits()
        pen, worst = self.hand_presses(side, tree, glove_cm, other=other, soft=soft)
        segs = {x[0]: x for x in self._segments(side)}
        palm = segs.get("palm_" + side)
        pg = None
        if palm is not None:
            _n, a, b, r = palm
            pg = self._seg_gap(tree, a, b, r)
            pg = None if pg is None else pg - glove_cm
            if other:
                # a support palm rests on the firing hand's fingers wrapped round the grip: they are its surface too
                for _on, oa, ob, orr in self._segments({"l": "r", "r": "l"}[side]):
                    if _on.startswith("forearm"):
                        continue
                    d = (self._seg_seg(a, b, oa, ob) - r - orr) * 100.0 - 2.0 * glove_cm
                    pg = d if pg is None else min(pg, d)
        arm, arm_bad = self._arm_report(side, Lm)
        # the wrist kept margin_deg inside its limits: a hold at the very limit breaks it as soon as the gun moves
        m = margin_deg
        over = max(0.0, arm.get("wrist_bend_deg", 0.0) - (Lm["wrist_max_deg"] - m))
        over += max(0.0, arm.get("wrist_radial_deg", 0.0) - (Lm["wrist_radial_deg"] - m / 2)) + max(0.0, -arm.get("wrist_radial_deg", 0.0) - (Lm["wrist_ulnar_deg"] - m / 2))
        over += 0.5 * max(0.0, 12.0 - arm.get("elbow_bend_deg", 90.0))      # an elbow kept bent: locked straight breaks a rule
        fx = arm.get("wrist_flexion_deg", 0.0)
        over += max(0.0, fx - (Lm["wrist_flexion_deg"] - m)) + max(0.0, -fx - (Lm["wrist_extension_deg"] - m))
        fbad = len(self._finger_report(side, FL)[1]) + len(self._thumb_report(side, self._thumb_limits())[1])
        # the grip's own rules (the back of the hand toward the gun, a back or palm too deep): what a gate checks
        _g = self.grip(side, hold=False).get(side, {})
        fbad += len(_g.get("bad", []))
        # and the palm's centre on the gun as the grip check measures it (holding needs it within 1.5 cm)
        _pc = _g.get("palm_gap_cm")
        centre_off = 3.0 if _pc is None else max(0.0, _pc - 1.2)
        palm_off = 3.0 if pg is None else max(0.0, pg - 0.2)
        # a hold holds: each finger the plan wraps closes at least half way round the gun and ends on it (a hand
        # standing beside the gun with its fingers straight touches nothing and presses nothing, yet holds nothing)
        loose = 0.0
        for fi, done in (getattr(self, "_wraps", None) or {}).items():
            loose += max(0.0, 0.5 - done) * 2.0
            loose += min(2.0, max(0.0, self._digit_gap(side, fi, tree, glove_cm, other) - 0.3))
        score = (25.0 * pen + 10.0 * palm_off + 0.15 * over + 2.0 * (fbad + len(arm_bad)) + 4.0 * short
                 + 6.0 * loose + 10.0 * centre_off)
        return score, {"press": round(pen, 3), "worst": worst, "palm_gap_cm": None if pg is None else round(pg, 2),
                       "wrist_over_deg": round(over, 1), "rules": fbad + len(arm_bad), "short_cm": round(short, 2),
                       "loose": round(loose, 2), "wraps": {k: round(v, 2) for k, v in (getattr(self, "_wraps", None) or {}).items()}}

    def _anchor(self, side, name):
        """A named point of the hand, in the world: web (between the thumb's base and the index knuckle), knuckle
        (the middle finger's), index_knuckle, pinky_knuckle, wrist, thumb_tip, middle_tip, palm (its middle)."""
        B = lambda n: self._bw(n).translation      # noqa: E731
        f = lambda fi, j: B(self._n("finger", side, fi, j))    # noqa: E731
        if name == "web":
            return f("thumb", "01").lerp(f("index", "01"), 0.5)
        if name == "knuckle":
            return f("middle", "01")
        if name in ("index_knuckle", "pinky_knuckle"):
            return f(name.split("_")[0], "01")
        if name == "wrist":
            return B(self._n("hand", side))
        if name == "palm":
            return B(self._n("hand", side)).lerp(f("middle", "01"), 0.5)
        if name in ("thumb_tip", "middle_tip", "index_tip"):
            return self._finger_tip(name.split("_")[0], side)
        raise ValueError("no hand point %r" % name)

    def _palm_facing(self, side):
        hn = self._n("hand", side)
        w = self._bw(hn).translation
        mid = self._bw(self._n("finger", side, "middle", "01")).translation
        across, k = self._hand_frame(side)
        return (across.cross((mid - w).normalized()) * k).normalized()

    def place_hand(self, side, anchors, facing=None, frame="world", facing_cm=3.0, across=None):
        """The hand moved and turned as one rigid body so its named points (anchors: {web|knuckle|...: point}) land
        as close as they can on their points, and its palm faces facing (a direction), by a least-squares fit
        (Kabsch). across: the way the line across the knuckles (little finger to index) should run, for a grip
        the grip's own axis upward, so the fingers curl round it. The arm reaches it. Returns each anchor's
        distance left (cm) and the reach short (cm)."""
        import numpy as np
        self._need()
        to_w = (lambda p: Vector(p)) if frame == "world" else (lambda p: self._point(p, frame))
        tg = {k: to_w(v) for k, v in anchors.items()}
        src, dst = [], []
        for k, t in tg.items():
            src.append(self._anchor(side, k))
            dst.append(t)
        if facing is not None:
            fw = Vector(facing).normalized()
            if frame == "gun":
                fw = (self._gun().to_quaternion() @ ue(facing)).normalized()
            c0 = sum(src, Vector()) / len(src)
            c1 = sum(dst, Vector()) / len(dst)
            src.append(c0 + self._palm_facing(side) * (facing_cm / 100.0))
            dst.append(c1 + fw * (facing_cm / 100.0))
        if across is not None:
            aw = Vector(across).normalized()
            if frame == "gun":
                aw = (self._gun().to_quaternion() @ ue(across)).normalized()
            ah = self._hand_frame(side)[0]
            c0 = sum(src, Vector()) / len(src)
            c1 = sum(dst, Vector()) / len(dst)
            for sg in (1.0, -1.0):
                src.append(c0 + ah * (sg * facing_cm / 100.0))
                dst.append(c1 + aw * (sg * facing_cm / 100.0))
        A = np.array([list(p) for p in src])
        Bm = np.array([list(p) for p in dst])
        ca, cb = A.mean(0), Bm.mean(0)
        Hm = (A - ca).T @ (Bm - cb)
        U, _S, Vt = np.linalg.svd(Hm)
        dd = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.diag([1.0, 1.0, dd])
        Rm = Vt.T @ D @ U.T
        R = Matrix([list(Rm[0]), list(Rm[1]), list(Rm[2])]).to_4x4()
        T = Matrix.Translation(Vector(list(cb))) @ R @ Matrix.Translation(-Vector(list(ca)))
        hn = self._n("hand", side)
        M = T @ self._bw(hn)
        short = self._put_hand(side, Matrix.LocRotScale(M.translation, M.to_quaternion(), self._unit()))
        bpy.context.view_layer.update()
        left = {k: round((self._anchor(side, k) - t).length * 100.0, 2) for k, t in tg.items()}
        return {"left_cm": left, "short_cm": round(short, 2)}

    def solve_hand(self, side, point, normal, plan=None, toward=None, glove_cm=0.0, frame="gun", span_cm=1.5,
                   turn_deg=20.0, samples=40, refine=16, seed=1, other=True, soft=None, from_current=False, tilt_deg=None,
                   anchors=None, anchor_w=3.0, contact_rules=None, contact_w=8.0):
        """The grasp solver: the hand placed on a hold as a whole. Around the palm laid on the point (facing into
        the surface's normal, its fingers toward the given way), places up to span_cm off along the surface and its
        normal and turns up to turn_deg about the normal (half that about the other two axes) are tried, the digits
        closed by the plan for each; the place that presses least past what is allowed (into the gun and into the
        other hand, the glove counted), keeps the palm on the surface, and leaves the arm and the fingers within their
        limits wins, then is refined round itself. Returns its score's parts; the hand is left there.
        from_current: the tries move and turn the hand from where it is now (about its palm's middle), keeping what
        a placement before it set (the thumb's base across the gun), instead of laying the palm afresh."""
        import random as _random
        self._need()
        rnd = _random.Random(seed)
        at = Vector(point) if frame == "world" else self._point(point, frame)
        G = self._gun().to_quaternion()
        nrm = (G @ ue(normal)).normalized() if frame == "gun" else Vector(normal).normalized()
        tw = None if toward is None else ((G @ ue(toward)).normalized() if frame == "gun" else Vector(toward).normalized())
        t1 = nrm.orthogonal().normalized()
        t2 = nrm.cross(t1).normalized()
        tree = self._tree()
        thick = (self._fitted_radii() or {}).get("palm", RADIUS["palm"]) * 100.0 + glove_cm - 0.15
        self.snapshot("save", "_solve_hand")
        hn0 = self._n("hand", side)
        H0 = self._bw(hn0).copy()
        P0 = H0.translation.lerp(self._bw(self._n("finger", side, "middle", "01")).translation, 0.5)

        def place(p):
            self.snapshot("load", "_solve_hand")
            dx, dy, dz, yaw, pa, pb = p
            if from_current:
                q = Quaternion(nrm, math.radians(yaw)) @ Quaternion(t1, math.radians(pa)) @ Quaternion(t2, math.radians(pb))
                R = (Matrix.Translation((t1 * dx + t2 * dy + nrm * dz) / 100.0) @ Matrix.Translation(P0)
                     @ q.to_matrix().to_4x4() @ Matrix.Translation(-P0) @ H0)
                short = self._put_hand(side, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))
                self.close_hand(side, plan, tree, glove_cm, other, from_pose=not anchors and not contact_rules,
                                rules=contact_rules)
                sc, why = self._hand_score(side, tree, glove_cm, other, soft, short)
                if contact_rules:
                    vv_ = self._gun_verts()
                    ev_ = [self._rule_eval(side, r_, glove_cm, vv_) for r_ in contact_rules]
                    cerr = sum(e["error"] * (1.0 if r_.get("need", "required") == "required" else 0.3)
                               for e, r_ in zip(ev_, contact_rules))
                    sc += contact_w * cerr
                    why = dict(why, contacts={e["id"]: e["error"] for e in ev_})
                if anchors:
                    off = {k: (self._anchor(side, k) - Vector(v)).length * 100.0 for k, v in anchors.items()}
                    sc += anchor_w * sum(max(0.0, d - 0.5) for d in off.values())
                    why = dict(why, anchors_cm={k: round(d, 2) for k, d in off.items()})
                return sc, why
            n2 = (Quaternion(t1, math.radians(pa)) @ Quaternion(t2, math.radians(pb)) @ nrm).normalized()
            way = tw if tw is not None else None
            if way is not None:
                way = Quaternion(nrm, math.radians(yaw)) @ way
            short = self._lay(side, at + (t1 * dx + t2 * dy + nrm * dz) / 100.0, n2, way, thick)
            if way is None and abs(yaw) > 1e-6:
                hn = self._n("hand", side)
                H = self._bw(hn)
                P = H.translation
                R = Matrix.Translation(P) @ Quaternion(nrm, math.radians(yaw)).to_matrix().to_4x4() @ Matrix.Translation(-P) @ H
                short = self._put_hand(side, Matrix.LocRotScale(R.translation, R.to_quaternion(), self._unit()))
            self.close_hand(side, plan, tree, glove_cm, other)
            return self._hand_score(side, tree, glove_cm, other, soft, short)

        best = (None, None, None)
        tried = 0
        tl = turn_deg / 2 if tilt_deg is None else tilt_deg
        # a wide turn (no finger direction to start from) is swept first: every 30 degrees about the face's normal
        sweep = [(0.0, 0.0, 0.0, float(y), 0.0, 0.0) for y in range(-180, 180, 30)] if turn_deg >= 90.0 else []
        cands = [(0.0,) * 6] + sweep + [(rnd.uniform(-span_cm, span_cm), rnd.uniform(-span_cm, span_cm), rnd.uniform(-0.3, 0.6 * span_cm),
                                         rnd.uniform(-turn_deg, turn_deg), rnd.uniform(-tl, tl), rnd.uniform(-tl, tl))
                                        for _ in range(max(0, samples - 1))]
        for p in cands:
            sc, why = place(p)
            tried += 1
            if best[0] is None or sc < best[0]:
                best = (sc, p, why)
        for _ in range(refine):
            p0 = best[1]
            tr = min(turn_deg, 30.0)
            p = (p0[0] + rnd.uniform(-0.3, 0.3) * span_cm, p0[1] + rnd.uniform(-0.3, 0.3) * span_cm, p0[2] + rnd.uniform(-0.15, 0.15) * span_cm,
                 p0[3] + rnd.uniform(-0.3, 0.3) * tr, p0[4] + rnd.uniform(-0.3, 0.3) * min(tl, 15.0), p0[5] + rnd.uniform(-0.3, 0.3) * min(tl, 15.0))
            sc, why = place(p)
            tried += 1
            if sc < best[0]:
                best = (sc, p, why)
        sc, why = place(best[1])
        return {"score": round(sc, 3), "move_cm": [round(v, 2) for v in best[1][:3]], "turn_deg": [round(v, 1) for v in best[1][3:]],
                "tried": tried, **why}

    # --- the body's sizes ---------------------------------------------------------------------------------------
    @staticmethod
    def _hull_perimeter(pts):
        """The convex girth of 2D points (a tape round them), and the hull."""
        p = sorted(set((round(x, 5), round(y, 5)) for x, y in pts))
        if len(p) < 3:
            return 0.0
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
        return sum(math.dist(h[i], h[(i + 1) % len(h)]) for i in range(len(h)))

    def _skin_points(self):
        """Every skin vertex in the world as posed now, with the bone that weighs it most."""
        dg = bpy.context.evaluated_depsgraph_get()
        out = []
        for ob in self.meshes:
            names = {g.index: g.name for g in ob.vertex_groups}
            ev = ob.evaluated_get(dg)
            me = ev.to_mesh()
            M = ev.matrix_world
            for v, v0 in zip(me.vertices, ob.data.vertices):
                if not v0.groups:
                    continue
                gmax = max(v0.groups, key=lambda x: x.weight)
                out.append((M @ v.co, names.get(gmax.group, "")))
            ev.to_mesh_clear()
        return out

    def body_size(self, side="both", ranges=None):
        """The arm's and hand's sizes as built, in cm: the bones joint to joint (upper arm, forearm), the hand's length
        to the farthest skin of its middle finger, the skin's breadth and girth at the knuckles (thumb left out), the
        wrist's least girth and the forearm's most. ranges ({measure: {low, high, ...}}, from Pose Lab's human_sizes):
        each measure outside its range is listed in bad. skin_vs_capsules: how far the drawn skin lies outside the
        capsules clearance tests the body with (palm, forearm, fingers); past 0.3 cm the clearance checks miss clips
        the picture shows, and that is listed in bad too. Measured on the rig's rest pose (fingers straight, as the
        survey measures a hand); the pose is put back after."""
        self._need()
        self.snapshot("save", "_body_size")
        for pb in self.arm.pose.bones:
            pb.matrix_basis = Matrix.Identity(4)
        bpy.context.view_layer.update()
        try:
            return self._body_size(side, ranges)
        finally:
            self.snapshot("load", "_body_size")
            bpy.context.view_layer.update()

    def _body_size(self, side, ranges):
        pts = self._skin_points()
        sides = ("l", "r") if side == "both" else (side,)
        out = {"ok": True}
        for s_ in sides:
            B = lambda n: self._bw(n).translation     # noqa: E731
            S, E, W = B(self._n("upperarm", s_)), B(self._n("lowerarm", s_)), B(self._n("hand", s_))
            K = B(self._n("finger", s_, "middle", "01"))
            I, Pk = B(self._n("finger", s_, "index", "01")), B(self._n("finger", s_, "pinky", "01"))
            along = (K - W).normalized()
            across = (I - Pk) - along * (I - Pk).dot(along)
            across.normalize()
            nrm = across.cross(along).normalized()
            suf = "_" + s_
            mine = [(p, g) for p, g in pts if g.endswith(suf)]
            arm = [p for p, g in mine if g.startswith("lowerarm") or g.startswith("hand")]
            handv = [(p, g) for p, g in mine if g.startswith("hand") or "metacarpal" in g or
                     any(g.startswith(f) for f in ("index", "middle", "ring", "pinky"))]
            m = {"upper_arm_cm": round((E - S).length * 100, 2), "forearm_cm": round((W - E).length * 100, 2)}
            mid = [p for p, g in mine if g.startswith("middle")]
            m["hand_length_cm"] = round(max(((p - W).dot(along) for p in mid), default=0.0) * 100, 2)
            # at the knuckles: the hand's own skin within 0.6 cm of the knuckle line, thumb left out
            kn = [p for p, g in handv if abs((p - K).dot(along)) < 0.006 and (p - K).length < 0.08]
            if kn:
                xs = [(p - K).dot(across) for p in kn]
                m["hand_breadth_cm"] = round((max(xs) - min(xs)) * 100, 2)
                m["hand_circumference_cm"] = round(self._hull_perimeter([((p - K).dot(across), (p - K).dot(nrm)) for p in kn]) * 100, 2)
            # the forearm in slices from the elbow to the wrist
            d = (W - E).normalized()
            u = d.orthogonal().normalized()
            v2 = d.cross(u)
            girth = []
            for k in range(1, 20):
                t = k / 20.0
                c = E.lerp(W, t)
                sl = [p for p in arm if abs((p - c).dot(d)) < 0.005 and ((p - c) - d * (p - c).dot(d)).length < 0.08]
                if len(sl) >= 8:
                    girth.append((t, self._hull_perimeter([((p - c).dot(u), (p - c).dot(v2)) for p in sl]) * 100))
            if girth:
                m["forearm_circumference_cm"] = round(max((g for t, g in girth if t <= 0.5), default=0.0), 2)
                m["wrist_circumference_cm"] = round(min((g for t, g in girth if t >= 0.8), default=0.0), 2)
            # the drawn skin against the capsules clearance tests: the palm's thickness, the forearm's middle, a finger
            cap = {}
            pm = W.lerp(K, 0.5)
            pt = [(p - pm).dot(nrm) for p, g in handv if abs((p - pm).dot(along)) < 0.006 and abs((p - pm).dot(across)) < 0.015]
            fit = self._fitted_radii() or {}
            if pt:
                cap["palm"] = round(((max(pt) - min(pt)) / 2 - fit.get("palm", RADIUS["palm"])) * 100, 2)
            c = E.lerp(W, 0.5)
            fr = [((p - c) - d * (p - c).dot(d)).length for p in arm if abs((p - c).dot(d)) < 0.005]
            if fr:
                # the mean radius at the middle (the capsule halves meet there)
                rf_ = (fit.get("forearm_elbow", RADIUS["forearm"]) + fit.get("forearm_wrist", RADIUS["forearm"])) / 2
                cap["forearm"] = round((self._hull_perimeter([((p - c).dot(u), (p - c).dot(v2)) for p in arm
                                                               if abs((p - c).dot(d)) < 0.005]) / (2 * math.pi) - rf_) * 100, 2)
            a2, a3 = B(self._n("finger", s_, "index", "02")), B(self._n("finger", s_, "index", "03"))
            fd = (a3 - a2).normalized()
            fc = a2.lerp(a3, 0.5)
            fv = [p - fc - fd * (p - fc).dot(fd) for p, g in mine if g.startswith("index") and abs((p - fc).dot(fd)) < 0.003]
            if len(fv) >= 6:
                uu = fd.orthogonal().normalized()
                vv = fd.cross(uu)
                cap["finger"] = round((self._hull_perimeter([(r.dot(uu), r.dot(vv)) for r in fv]) / (2 * math.pi)
                                       - fit.get("finger_index", RADIUS["finger"])) * 100, 2)
            bad = []
            for k, val in m.items():
                rg = (ranges or {}).get(k)
                if rg and not (rg["low"] <= val <= rg["high"]):
                    bad.append("%s %.1f cm outside the human range %.1f to %.1f (1st to 99th percentile%s)"
                               % (k, val, rg["low"], rg["high"], ", landmark allowance counted" if rg["low"] != rg.get("p1") or rg["high"] != rg.get("p99") else ""))
            for k, val in cap.items():
                if val > 0.3:
                    bad.append("the %s's skin lies %.1f cm outside the capsule clearance tests it with: the checks miss clips the picture shows" % (k, val))
            out[s_] = {"measures": m, "skin_vs_capsules_cm": cap, "bad": bad}
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
        rep_ = {"elbow_under_shoulder_cm": round(under, 1), "upper_arm_raised_deg": round(self._deg(upper, -up)),
                "elbow_bend_deg": round(bend), "wrist_bend_deg": round(total), "wrist_flexion_deg": round(flex),
                "wrist_radial_deg": round(dev)}
        # the forearm's rotation: pronation and supination, measured from thumb up against the elbow's hinge plane (the
        # bones' bind does not enter it). The radius turns over the ulna, so a hand's roll is the forearm's
        rot = self._forearm_rotation(s)
        if rot is not None:
            rep_["forearm_rotation_deg"] = round(rot)
            if round(rot) > L["supination_deg"]:
                bad.append("forearm supinated %.0f (at most %.0f)" % (rot, L["supination_deg"]))
            if -round(rot) > L["pronation_deg"]:
                bad.append("forearm pronated %.0f (at most %.0f)" % (-rot, L["pronation_deg"]))
        if self._twist_bones(s):
            # a rig with twist bones carries the roll along the forearm's skin: each link (the elbow's end, each twist
            # bone, the wrist) turns only part of it. The whole roll in one link wrings the skin there
            links, total = self._forearm_links(s)
            rep_["forearm_roll_deg"] = round(total)
            rep_["forearm_links_deg"] = {n: round(v) for n, v, _share in links}
            for n, v, share in links:
                if round(abs(v)) > L["forearm_link_max_deg"]:
                    bad.append("forearm's %s link rolled %.0f (at most %.0f: the skin wrings there; its even share is %.0f)"
                               % (n, abs(v), L["forearm_link_max_deg"], abs(share)))
        else:
            # without twist bones: the hand turned about the forearm's line against the forearm's bone, beyond the idle
            # grip's (a forearm turned 154 degrees under a still hand passed before 0.3.1)
            twist = self._wrist_twist(s)
            if twist is not None:
                rep_["wrist_twist_deg"] = round(twist)
                if round(twist) > L["wrist_twist_max_deg"]:
                    bad.append("wrist twisted %.0f against the idle grip (at most %.0f): the hand's roll belongs to the forearm"
                               % (twist, L["wrist_twist_max_deg"]))
        # the shoulder in its range, against the view's axes (right, forward, up); lat: the arm's own side
        lat = VIEW_RIGHT if s == "r" else -VIEW_RIGHT
        u = upper.normalized()
        sf = math.degrees(math.atan2(u.dot(VIEW_FWD), -u.dot(VIEW_UP)))     # 0 hanging, 90 straight ahead, 180 up
        rep_["shoulder_flexion_deg"] = round(sf)
        if round(sf) > L["shoulder_flexion_deg"]:
            bad.append("shoulder raised %.0f forward (at most %.0f)" % (sf, L["shoulder_flexion_deg"]))
        if -round(sf) > L["shoulder_extension_deg"]:
            bad.append("shoulder swung %.0f back (at most %.0f)" % (-sf, L["shoulder_extension_deg"]))
        hz = Vector((u.dot(lat), u.dot(VIEW_FWD)))
        if hz.length > 0.5:                       # the arm well off hanging: its swing in the horizontal plane means something
            cross = math.degrees(math.atan2(-hz.x, hz.y))       # + across the body past straight ahead
            rep_["shoulder_across_deg"] = round(cross)
            if round(cross) > L["shoulder_cross_body_deg"]:
                bad.append("upper arm %.0f across the body past straight ahead (at most %.0f)" % (cross, L["shoulder_cross_body_deg"]))
            behind = math.degrees(math.atan2(-hz.y, hz.x))       # + behind the side
            if round(behind) > L["shoulder_behind_deg"]:
                bad.append("upper arm %.0f behind the side (at most %.0f)" % (behind, L["shoulder_behind_deg"]))
        if bend >= 25.0:                          # the upper arm's own turn reads only with the elbow bent
            ref = VIEW_FWD - u * VIEW_FWD.dot(u) if abs(u.dot(VIEW_FWD)) < 0.9 else VIEW_UP - u * VIEW_UP.dot(u)
            fp = fore - u * fore.dot(u)
            if ref.length > 1e-6 and fp.length > 1e-6:
                ref, fp = ref.normalized(), fp.normalized()
                rr = math.degrees(math.atan2(ref.cross(fp).dot(u), ref.dot(fp)))
                internal = -rr if s == "r" else rr
                rep_["shoulder_internal_rot_deg"] = round(internal)
                if round(internal) > L["shoulder_internal_deg"]:
                    bad.append("upper arm turned %.0f inward (at most %.0f)" % (internal, L["shoulder_internal_deg"]))
                if -round(internal) > L["shoulder_external_deg"]:
                    bad.append("upper arm turned %.0f outward (at most %.0f)" % (-internal, L["shoulder_external_deg"]))
        return rep_, bad

    # --- the forearm's roll, physically (0.4.0) ------------------------------------------------------------------
    def _rest_turn(self, n):
        """The bone's turn from its bind (rest) pose, in world terms: what the skin bound to it sees."""
        pb = self.arm.pose.bones[n]
        return (self.arm.matrix_world @ pb.matrix).to_quaternion() @ (self.arm.matrix_world @ pb.bone.matrix_local).to_quaternion().inverted()

    @staticmethod
    def _twist_about(q, axis):
        """The signed turn of q about axis (degrees), q split as a swing after this twist."""
        a = 2.0 * math.degrees(math.atan2(Vector((q.x, q.y, q.z)).dot(axis), q.w))
        return (a + 180.0) % 360.0 - 180.0

    def _twist_bones(self, s):
        """The forearm's twist bones (its children other than the hand whose names hold "twist"), elbow to wrist, each
        with its place along the forearm (0 the elbow, 1 the wrist)."""
        lo, hn = self._n("lowerarm", s), self._n("hand", s)
        if lo not in self.arm.pose.bones or hn not in self.arm.pose.bones:
            return []
        E, W = self._bw(lo).translation, self._bw(hn).translation
        a = W - E
        if a.length_squared < 1e-12:
            return []
        out = [(min(max((self.arm.matrix_world @ pb.head - E).dot(a) / a.length_squared, 0.0), 1.0), pb.name)
               for pb in self.arm.pose.bones[lo].children if pb.name != hn and "twist" in pb.name.lower()]
        return sorted(out)

    def _forearm_rotation(self, s):
        """+ supination, - pronation, degrees from thumb up against the elbow's hinge plane; None with the elbow near
        straight (no hinge plane)."""
        S = self._bw(self._n("upperarm", s)).translation
        E = self._bw(self._n("lowerarm", s)).translation
        W = self._bw(self._n("hand", s)).translation
        fore = (W - E).normalized()
        up = S - E
        if fore.length < 1e-9 or up.length < 1e-9 or math.degrees(fore.angle(-up.normalized())) < 20.0:
            return None
        r = up - fore * up.dot(fore)
        t = self._bw(self._n("finger", s, "index", "01")).translation - self._bw(self._n("finger", s, "pinky", "01")).translation
        t = t - fore * t.dot(fore)
        if r.length < 1e-9 or t.length < 1e-9:
            return None
        r, t = r.normalized(), t.normalized()
        ang = math.degrees(math.atan2(r.cross(t).dot(fore), r.dot(t)))
        return ang if s == "r" else -ang

    def _forearm_links(self, s):
        """([(link, roll, share)], total): each link's roll about the forearm against the skin's bind, and the roll it
        carries when the roll is spread in proportion (the elbow's end forearm_elbow_share of it)."""
        L, _fl = self._limits()
        share0 = L["forearm_elbow_share"]
        up, lo, hn = self._n("upperarm", s), self._n("lowerarm", s), self._n("hand", s)
        a = (self._bw(hn).translation - self._bw(lo).translation).normalized()
        rest_fore = (self.arm.matrix_world @ self.arm.pose.bones[hn].bone.head_local) - (self.arm.matrix_world @ self.arm.pose.bones[lo].bone.head_local)
        v0 = (self._rest_turn(up) @ rest_fore).normalized()   # the forearm's line as the upper arm carries it, unbent
        out = [("elbow", self._twist_about(self._rest_turn(lo) @ self._rest_turn(up).inverted(), v0))]
        prev = lo
        tb = self._twist_bones(s)
        for _p, n in tb:
            out.append((n, self._twist_about(self._rest_turn(n) @ self._rest_turn(prev).inverted(), a)))
            prev = n
        out.append(("wrist", self._twist_about(self._rest_turn(hn) @ self._rest_turn(prev).inverted(), a)))
        total = sum(v for _n, v in out)
        places = [0.0] + [p for p, _n in tb] + [1.0]
        shares = [share0] + [(1.0 - share0) * (places[i + 1] - places[i]) for i in range(len(places) - 1)]
        return [(n, v, total * sh) for (n, v), sh in zip(out, shares)], total

    def _spread_roll(self, s):
        """The forearm's roll spread along it: the elbow's end its share, each twist bone the share up to its place, the
        hand left where it is. Nothing the eye sees of the hand or the elbow moves; the skin between turns evenly."""
        L, _fl = self._limits()
        share0 = L["forearm_elbow_share"]
        lo, hn = self._n("lowerarm", s), self._n("hand", s)
        H = self._bw(hn).copy()
        for _ in range(3):        # the links are measured about lines that move a little as they turn
            links, total = self._forearm_links(s)
            E, W = self._bw(lo).translation, self._bw(hn).translation
            a = (W - E).normalized()
            self._turn_about(lo, a, links[0][2] - links[0][1], E)
            self._set_world(hn, H)
            for p, n in self._twist_bones(s):
                now = self._twist_about(self._rest_turn(n) @ self._rest_turn(lo).inverted(), a)
                self._turn_about(n, a, (1.0 - share0) * p * total - now, self._bw(n).translation)
            self._set_world(hn, H)

    def _twist_change(self, s):
        """The hand's twist about the forearm's line against the forearm's bone, beyond the idle grip's, as a turn in
        the forearm's own frame (None without an idle grip)."""
        lo, hn = self._n("lowerarm", s), self._n("hand", s)
        idle = getattr(self, "idle_world", {})
        if lo not in idle or hn not in idle:
            return None

        def twist(Lm, Hm):
            lq = Lm.to_quaternion()
            axis = lq.inverted() @ (Hm.translation - Lm.translation)
            if axis.length < 1e-9:
                return None
            rel = lq.inverted() @ Hm.to_quaternion()
            p = Vector((rel.x, rel.y, rel.z)).project(axis.normalized())
            return Quaternion((rel.w, p.x, p.y, p.z)).normalized()
        now, base = twist(self._bw(lo), self._bw(hn)), twist(idle[lo], idle[hn])
        if now is None or base is None:
            return None
        return now @ base.inverted()

    def _wrist_twist(self, s):
        """Degrees the hand is twisted about the forearm's line against the forearm's bone, beyond the idle grip's."""
        q = self._twist_change(s)
        if q is None:
            return None
        a = math.degrees(q.angle)
        return 360.0 - a if a > 180.0 else a

    def _roll_forearm(self, s):
        """The hand's twist moved into the forearm (the radius turns over the ulna), the hand left where it is: after a
        hand is placed with a turn, the wrist joint only bends. With twist bones the roll is spread along them (0.4.0):
        all of it in the forearm bone wrung the skin at the elbow instead of the wrist."""
        if self._twist_bones(s):
            self._spread_roll(s)
            return
        q = self._twist_change(s)
        if q is None:
            return
        lo, hn = self._n("lowerarm", s), self._n("hand", s)
        H = self._bw(hn).copy()
        lq = self._bw(lo).to_quaternion()
        self._turn(lo, lq @ q @ lq.inverted())
        self._set_world(hn, Matrix.LocRotScale(H.translation, H.to_quaternion(), self._unit()))

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
                elif j == "02":
                    # the middle joint against its own hinge: square to the segment before it and to the palm's
                    # normal (across the knuckles). The plane through the tip follows a kink here and hid it
                    h = across - a * across.dot(a)
                    side_deg = asin(c.dot(h.normalized())) if h.length > 1e-6 else 0.0
                else:
                    # the end joint against the middle joint: one hinge line, so their bending planes are one plane. A
                    # plane fitted through the tip follows a kink at the middle joint and hid a 35 degree one (ToangTown's
                    # M24 benchmark); the two joints' own planes do not. A joint bent under about 10 degrees has no plane:
                    # then the end segment against the middle joint's plane
                    p_mid = (pts[2] - pts[1]).cross(pts[3] - pts[2])
                    p_end = (pts[3] - pts[2]).cross(pts[4] - pts[3])
                    s_mid = 0.17 * (pts[2] - pts[1]).length * (pts[3] - pts[2]).length
                    s_end = 0.17 * (pts[3] - pts[2]).length * (pts[4] - pts[3]).length
                    if p_mid.length >= s_mid and p_end.length >= s_end:
                        ang = math.degrees(p_mid.angle(p_end))
                        side_deg = math.copysign(min(ang, 180.0 - ang), p_mid.cross(p_end).dot(pts[3] - pts[2]))
                    elif p_mid.length >= s_mid:
                        side_deg = asin(c.dot(p_mid.normalized()))
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

    def _clear_fingers(self, s, back_max_cm=0.1):
        """A finger whose back is inside the rifle (a hand moved onto a new hold keeps its old curl) turns, a few
        degrees at a time from the tip inward, about its own hinge, until its back is clear: open first (curled past
        the surface), else curled in (a trigger finger lying across another gun's guard closes onto the trigger),
        each joint within its range. Returns how many turns it made."""
        if not self._has_fingers(s):
            return 0
        import re as _re
        _L, FL = self._limits()
        tree = self._tree()
        made = 0
        for _ in range(16):
            back_cm, seg = self._hand_contacts(s, tree)[3:5]
            if back_cm <= back_max_cm or not seg:
                break
            m = _re.match(r"(index|middle|ring|pinky)(\d)", seg)
            if not m:
                break
            fi, k = m.group(1), int(m.group(2))
            moved = False
            for j in ["%02d" % x for x in range(min(k, 3), 0, -1)]:
                fr, _b, planes = self._finger_report(s, FL)
                curl = fr["%s_%s" % (fi, j)][0]
                lo, hi = FL[j][0], FL[j][1]
                n = self._n("finger", s, fi, j)
                M = self._bw(n)
                head = M.translation.copy()
                for want_open in (True, False):
                    room = (curl - lo - 1.0) if want_open else (hi - curl - 1.0)
                    if room <= 0.0:
                        continue
                    step = min(6.0, room)
                    for sg in (1.0, -1.0):
                        self._turn_about(n, planes[fi], step * sg, head)
                        c2 = self._finger_report(s, FL)[0]["%s_%s" % (fi, j)][0]
                        if ((c2 < curl - 0.5) if want_open else (c2 > curl + 0.5)) and lo <= c2 <= hi and \
                                self._hand_contacts(s, tree)[3] < back_cm - 1e-4:
                            moved = True
                            break
                        self._set_world(n, M)
                    if moved:
                        break
                if moved:
                    break
            if not moved:
                break
            made += 1
        return made

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
        self._roll_forearm(side)
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
                    # a finger whose back is inside the rifle opens off it (a hand moved onto another gun's grip)
                    if self._in(c, i / fps) and c.get("fingers", True):
                        made += self._clear_fingers(side)
                    # the forearm's roll spread along its twist bones (0.4.0's rule), where a link rolls past its limit
                    if self._in(c, i / fps) and self._twist_bones(side):
                        rep_, bad_ = self._arm_report(side, self._limits()[0])
                        if any("link rolled" in b for b in bad_):
                            self._spread_roll(side)
                            made += 1
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
    # --- the view the arms leave (0.5.0) ------------------------------------------------------------------------
    def view_cover(self, size=(192, 108)):
        """The share of the player's view the arms cover: all of it, its lower left third and its bottom middle third.
        The arms render alone from the eye (90 degrees, 16:9) as a mask, the rifle hidden, their back faces culled as a
        game culls them (a shoulder the near plane cuts open shows nothing inside). A forearm or sleeve that fills the
        lower left hides the gun's left side and the world; ToangTown's Game Probe measures the same in the game (within
        about 0.06 of it on the AKS-74U arms, 2026-10-13)."""
        self._need()
        hidden = []
        for o in self.sc.objects:
            if o.type == 'MESH' and o not in self.meshes and not o.hide_render:
                o.hide_render = True
                hidden.append(o)
        cam_data = bpy.data.cameras.get("poselab") or bpy.data.cameras.new("poselab")
        cam = bpy.data.objects.get("poselab") or bpy.data.objects.new("poselab", cam_data)
        if cam.name not in self.sc.collection.objects:
            self.sc.collection.objects.link(cam)
        cam_data.lens_unit = 'FOV'
        cam_data.angle = math.radians(H_FOV)
        cam_data.clip_start = 0.05
        cam.location = self.eye
        cam.rotation_euler = VIEW_FWD.to_track_quat('-Z', 'Y').to_euler()
        self.sc.camera = cam
        r = self.sc.render
        keep = (r.engine, r.resolution_x, r.resolution_y, r.film_transparent, r.filepath, self.sc.display.render_aa)
        shading = self.sc.display.shading
        keep_cull = shading.show_backface_culling
        shading.show_backface_culling = True
        r.engine = 'BLENDER_WORKBENCH'
        r.resolution_x, r.resolution_y = size
        r.film_transparent = True
        self.sc.display.render_aa = 'OFF'
        path = self._own(os.path.join(self.out_dir, "renders", "_view_cover.png"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        r.filepath = path
        try:
            bpy.ops.render.render(write_still=True)
        finally:
            for o in hidden:
                o.hide_render = False
            r.engine, r.resolution_x, r.resolution_y, r.film_transparent, r.filepath, self.sc.display.render_aa = keep
            shading.show_backface_culling = keep_cull
        img = bpy.data.images.load(path, check_existing=False)
        W, H = img.size
        px = list(img.pixels)
        bpy.data.images.remove(img)
        total = ll = bm = 0
        third = (W // 3) * (H // 3)
        for y in range(H):                     # Blender's rows start at the bottom
            for x in range(W):
                if px[(y * W + x) * 4 + 3] <= 0.5:
                    continue
                total += 1
                if y < H // 3:
                    if x < W // 3:
                        ll += 1
                    elif x < 2 * W // 3:
                        bm += 1
        return {"total": round(total / float(W * H), 3), "lower_left": round(ll / float(third), 3),
                "bottom_middle": round(bm / float(third), 3), "hand_l_shown": self.hand_shown("l")}

    def hand_shown(self, side="l"):
        """How much of a hand the player sees: the share of the hand's own vertices (it and its fingers) that are inside
        the view (90 degrees, 16:9) and nearest the eye along their line of sight, the arms and the rifle in the way.
        A hand on top of a handguard shows about half of itself, one under it or below the view's edge little or none.
        ToangTown's Game Probe measures the same (view.hand_l_shown)."""
        self._need()
        hand = self._n("hand", side)
        bones = self.arm.data.bones
        if hand not in bones:
            return None
        inside = {b.name for b in bones if b.name == hand or any(p.name == hand for p in b.parent_recursive)}
        dg = bpy.context.evaluated_depsgraph_get()
        verts, polys, mine = [], [], []
        for ob in self.meshes:
            ev = ob.evaluated_get(dg)
            me = ev.to_mesh()
            base = len(verts)
            M = ob.matrix_world
            names = {g.index: g.name for g in ob.vertex_groups}
            for v, v0 in zip(me.vertices, ob.data.vertices):
                verts.append(M @ v.co)
                top = max(v0.groups, key=lambda g: g.weight, default=None)
                if top is not None and top.weight > 0.0 and names.get(top.group) in inside:
                    mine.append(base + v.index)
            polys += [tuple(base + i for i in p.vertices) for p in me.polygons]
            ev.to_mesh_clear()
        if not mine:
            return None
        ev = self.rifle_mesh.evaluated_get(dg)
        me = ev.to_mesh()
        base = len(verts)
        M = self.rifle_mesh.matrix_world
        verts += [M @ v.co for v in me.vertices]
        polys += [tuple(base + i for i in p.vertices) for p in me.polygons]
        ev.to_mesh_clear()
        bvh = BVHTree.FromPolygons(verts, polys)
        tan_x = math.tan(math.radians(H_FOV) / 2.0)
        tan_y = tan_x / ASPECT
        seen = 0
        for i in mine:
            p = verts[i]
            d = p - self.eye
            ahead, right, up = d.dot(VIEW_FWD), d.dot(VIEW_RIGHT), d.dot(VIEW_UP)
            if ahead < 0.01 or abs(right) >= ahead * tan_x or abs(up) >= ahead * tan_y:
                continue
            hit = bvh.ray_cast(self.eye, d.normalized(), d.length + 0.05)
            if hit[0] is not None and abs((hit[0] - self.eye).length - d.length) <= 0.02:      # the first surface is its own (2 cm)
                seen += 1
        return round(seen / float(len(mine)), 3)

    # --- a hold searched for, with the view as a goal (0.5.0) ---------------------------------------------------
    def solve_grip(self, side, mark, along=(-8.0, 8.0), steps=9, poles=None, view_budget=None, clearance_max_cm=0.2,
                   ignore=(), frame="gun", rolls=(0.0, -30.0, 30.0, -60.0, 60.0)):
        """Searches a hand's hold: its palm on a mark (gun frame, cm) moved along the barrel by along (cm, steps), each
        with the elbow pointed toward each pole (arms frame, cm; the idle elbow and four around it if left out) and the
        hand rolled about the barrel by each of rolls (degrees: the forearm pronates or supinates with it). A hold
        counts when the hand holds the rifle with its palm (grip, hold: the palm's press within grip's own limit), nothing
        else clips past clearance_max_cm (declared contacts in ignore), every arm and finger rule holds (anatomy), and the
        view's lower left and bottom middle stay
        within view_budget {lower_left, bottom_middle}. Of the holds that count, the one nearest the mark wins, then the
        one that covers least of the view. The rig is left in it (snapshot 'solve_grip_start' holds the pose before)."""
        self._need()
        if side not in ("l", "r"):
            raise ValueError("side must be l or r")
        self.snapshot("save", "solve_grip_start")
        hn = self._n("hand", side)
        G = self._gun()
        H0 = self._bw(hn).copy()
        rel = G.inverted() @ H0                                   # the hand's turn against the gun, kept
        mid = self._bw(self._n("finger", side, "middle", "01")).translation
        # the palm's middle from the wrist, in the hand's turn (world units: an FBX armature's scale would otherwise
        # stretch it a hundredfold)
        palm_off = H0.to_quaternion().inverted() @ (H0.translation.lerp(mid, 0.5) - H0.translation)
        target = self._point(mark, frame)
        axis = (G.to_quaternion() @ Vector((0.0, -1.0, 0.0))).normalized()   # along the barrel (ue +Y is Blender -Y)
        if poles is None:
            e = self._to(self._bw(self._n("lowerarm", side)).translation, "arms")
            poles = [None] + [[e.x + dx, e.y + dy, e.z + dz] for dx, dy, dz in ((0, 0, -15), (15, 0, -10), (-15, 0, -10), (0, 15, -15))]
        rows, best = [], None
        n = max(1, int(steps))
        combos = [(i, pole, roll) for i in range(n) for pole in poles for roll in rolls]
        for i, pole, roll in combos:
            d = along[0] + (along[1] - along[0]) * (i / (n - 1) if n > 1 else 0.5)
            if True:
                self.snapshot("load", "solve_grip_start")
                Gn = self._gun()
                palm_goal = target + axis * d / 100.0
                Hw = Gn @ rel
                if roll:
                    R = Quaternion(axis, math.radians(roll)).to_matrix().to_4x4()
                    Hw = Matrix.Translation(palm_goal) @ R @ Matrix.Translation(-palm_goal) @ Matrix.LocRotScale(palm_goal - (Hw.to_quaternion() @ palm_off), Hw.to_quaternion(), Hw.to_scale())
                wrist_goal = palm_goal - (Hw.to_quaternion() @ (palm_off * 1.0))
                self._two_bone(side, wrist_goal, pole)
                self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, Hw.to_quaternion(), self._unit()))
                self._roll_forearm(side)
                self._mend_hand(side)
                # settle the palm on the rifle's surface: pressed in, the hand backs off along the palm's own direction
                # (from inside, the grip reads a negative palm gap); a few rounds, the hand's turn kept
                for _ in range(3):
                    g = self.grip(side, hold=False)[side]
                    gap = g.get("palm_gap_cm")
                    if gap is None or -0.1 <= gap <= 0.4:
                        break
                    across, k = self._hand_frame(side)
                    hw = self._bw(hn)
                    mid_now = self._bw(self._n("finger", side, "middle", "01")).translation
                    palm_dir = (across.cross((mid_now - hw.translation).normalized()) * k).normalized()
                    move = palm_dir * ((gap - 0.15) / 100.0)          # toward the rifle if off it, away if pressed in
                    self._two_bone(side, hw.translation + move, pole)
                    self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, hw.to_quaternion(), self._unit()))
                    self._roll_forearm(side)
                self._clear_fingers(side)
                g = self.grip(side, hold=True)[side]
                # the palm's own press is grip's to judge (palm_max_cm, its capsule is rounder than a palm): clearance
                # covers the forearm, the back of the hand and whatever the caller did not declare a contact
                cr = self.clearance(parts="left" if side == "l" else "right", ignore=list(ignore) + ["palm*"], top=2)
                c = cr["worst_cm"]
                a = self.anatomy(side)[side]
                v = self.view_cover()
                over = None
                if view_budget:
                    over = max(v["lower_left"] - view_budget.get("lower_left", 1.0), v["bottom_middle"] - view_budget.get("bottom_middle", 1.0),
                               view_budget.get("hand_l_shown_min", 0.0) - (v.get("hand_l_shown") or 0.0), 0.0)
                ok = g["holding"] and not g["bad"] and c <= clearance_max_cm and not a["bad"] and (over is None or over <= 0.0)
                row = {"along_cm": round(d, 1), "pole": pole, "roll_deg": roll, "holding": g["holding"], "palm_gap_cm": g["palm_gap_cm"],
                       "clearance_cm": c, "clips": [x["segment"] for x in cr["contacts"]], "rules_broken": a["bad"][:3], "grip_bad": g["bad"][:3], "view": v, "ok": ok}
                rows.append(row)
                if ok and (best is None or (abs(d), abs(roll), v["lower_left"] + v["bottom_middle"]) < (abs(best["along_cm"]), abs(best["roll_deg"]), best["view"]["lower_left"] + best["view"]["bottom_middle"])):
                    best = row
                    self.snapshot("save", "solve_grip_best")
        if best:
            self.snapshot("load", "solve_grip_best")
        else:
            self.snapshot("load", "solve_grip_start")
        return {"found": best is not None, "best": best, "tried": len(rows), "passed": sum(r["ok"] for r in rows),
                "rows": sorted(rows, key=lambda r: (not r["ok"], abs(r["along_cm"]), abs(r["roll_deg"])))[:12]}

    # --- shared motion, the gun's own grip (0.5.0) --------------------------------------------------------------
    def retarget_clip(self, clip, side, mark=None, out=None, frame="gun", keep_from_s=None, keep_to_s=None, hold="idle",
                      gun_move=None, pole=None):
        """A donor rig's clip played on this rig with a hand held on this gun's own hold: each frame keeps the clip's
        body, then the hand is put back where it holds the gun at rest (or on mark, gun frame) by arm IK, its turn
        against the gun kept, its fingers mended. keep_from_s..keep_to_s: the span in which the hand lets go (a reload),
        left as the clip has it. hold: 'idle' (the rig's idle hold) or 'current' (the hold the rig stands in now, as
        solve_grip leaves it). gun_move: move_gun's arguments (roll, swing, pitch, right, forward, up, pivot), applied to
        each frame before the hand is placed, the other hand keeping its hold: a rifle carried lower or closer than the
        clip has it. pole: where the elbow points (arms frame, cm), as solve_grip's best gives it; the arm IK's own
        choice if left out. Stores the result as out (default clip + '_retarget')."""
        self._need()
        frames, fps = self._frames(clip)
        hn = self._n("hand", side)
        if hold not in ("idle", "current"):
            raise ValueError("hold must be idle or current")
        if hold == "idle":
            self.pose_idle()
        other = "r" if side == "l" else "l"
        move = dict(gun_move or {})
        move.pop("keep_hands", None)
        G = self._gun()
        rel = G.inverted() @ self._bw(hn)
        fingers = {pb.name: (pb.location.copy(), pb.rotation_quaternion.copy()) for pb in self.arm.pose.bones
                   if pb.name.endswith("_" + side) and any(f in pb.name for f in ("thumb", "index", "middle", "ring", "pinky"))}
        if mark is not None:
            p = self._point(mark, frame)
            rel = Matrix.LocRotScale(G.inverted() @ p, rel.to_quaternion(), rel.to_scale())
        made = []
        for i, fr in enumerate(frames):
            t = i / fps
            self._apply(fr)
            if keep_from_s is not None and keep_to_s is not None and keep_from_s <= t <= keep_to_s:
                made.append(self._capture())
                continue
            if any(v for k, v in move.items() if k != "pivot"):
                self.move_gun(keep_hands=(other,), **move)
            Hw = self._gun() @ rel
            self._two_bone(side, Hw.translation, pole)
            self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, Hw.to_quaternion(), self._unit()))
            for name, (l, q) in fingers.items():           # the hold's own fingers
                pb = self.arm.pose.bones[name]
                pb.location, pb.rotation_quaternion = l, q
            bpy.context.view_layer.update()
            self._roll_forearm(side)
            self._mend_hand(side)
            self._clear_fingers(side)
            made.append(self._capture())
        name = out or clip + "_retarget"
        self._store(name, made, fps, "retarget_clip(%s, %s)" % (clip, side))
        return {"clip": name, "frames": len(made), "fps": fps}

    def shift_hand(self, clip, side, deltas, out=None):
        """A clip with one hand moved, frame by frame, by deltas: [[seconds, [x, y, z]], ...] in the gun frame (cm),
        straight lines between them (a hand that holds a part whose track changed follows the part's new path). The
        hand keeps its turn against the gun, its elbow its direction, its fingers their curl; the wrist and fingers
        are mended. Stores the result as out (default clip + '_shifted')."""
        self._need()
        frames, fps = self._frames(clip)
        pts = sorted((float(t), Vector(_xyz(d, "delta"))) for t, d in deltas)
        if not pts:
            raise ValueError("deltas: at least one [seconds, [x, y, z]]")

        def at(t):
            if t <= pts[0][0]:
                return pts[0][1]
            for (t0, a), (t1, b) in zip(pts, pts[1:]):
                if t <= t1:
                    return a.lerp(b, (t - t0) / max(t1 - t0, 1e-9))
            return pts[-1][1]
        hn = self._n("hand", side)
        made = []
        for i, fr in enumerate(frames):
            self._apply(fr)
            d = at(i / fps)
            if d.length > 1e-4:
                G = self._gun()
                H = self._bw(hn)
                rel = G.inverted() @ H
                move = G.to_quaternion() @ ue(d)          # gun frame cm to world
                S0 = self._bw(self._n("upperarm", side)).translation
                E0 = self._bw(self._n("lowerarm", side)).translation
                line = H.translation - S0
                foot = S0 + line * max(0.0, min(1.0, (E0 - S0).dot(line) / max(line.length_squared, 1e-12)))
                out_dir = E0 - foot
                pole = E0 + out_dir * 2.0 + move * 0.5 if out_dir.length > 1e-5 else None
                fingers = {pb.name: (pb.location.copy(), pb.rotation_quaternion.copy()) for pb in self.arm.pose.bones
                           if pb.name.endswith("_" + side) and any(f in pb.name for f in ("thumb", "index", "middle", "ring", "pinky"))}
                self._two_bone(side, H.translation + move, pole_world=pole)
                W = G @ rel
                self._set_world(hn, Matrix.LocRotScale(self._bw(hn).translation, W.to_quaternion(), self._unit()))
                for name, (l, q) in fingers.items():
                    pb = self.arm.pose.bones[name]
                    pb.location, pb.rotation_quaternion = l, q
                bpy.context.view_layer.update()
                self._roll_forearm(side)
                self._mend_hand(side)
                self._clear_fingers(side)
            made.append(self._capture())
        name = out or clip + "_shifted"
        self._store(name, made, fps, "shift_hand(%s, %s)" % (clip, side))
        return {"clip": name, "frames": len(made), "fps": fps}

    # --- every check at once, for a gate (0.5.0) ----------------------------------------------------------------
    def check_set(self, clip, ignore=(), clearance_max_cm=0.2, hold_sides=(), sides="both"):
        """Every check on a clip, as one summary a gate can store: the worst clearance (declared contacts in ignore) and
        when, the frames that break an arm or finger rule, the frames that break a grip rule (the back of a hand on the
        rifle, a palm pressed too deep), and for each side in hold_sides the frames its hand is not holding the rifle
        (its palm on it, and its thumb opposing its fingers or two fingers wrapped onto it).
        sides: both, l or r (the side the clip is about: a magazine hold is the left hand's). rules: every distinct rule
        broken, with how many frames break it. Also each frame's numbers, for a table."""
        if sides not in ("both", "l", "r"):
            raise ValueError("sides must be both, l or r")
        part = {"both": "both", "l": "left", "r": "right"}[sides]
        which = ("l", "r") if sides == "both" else (sides,)
        import re as _re
        seen = {}
        self._need()
        frames, fps = self._frames(clip)
        worst, worst_t, breaks, not_holding, grip_bad = 0.0, 0.0, 0, {s_: 0 for s_ in hold_sides}, 0
        per = []
        for i, fr in enumerate(frames):
            self._apply(fr)
            c = self.clearance(parts=part, ignore=ignore)["worst_cm"]
            a = self.anatomy(side=sides)
            b = sum(len(a.get(s_, {}).get("bad", [])) for s_ in which)
            for s_ in which:
                for txt in a.get(s_, {}).get("bad", []):
                    k = "%s: %s" % (s_, _re.sub(r"[-0-9.]+", "#", txt))
                    seen[k] = seen.get(k, 0) + 1
            if c > worst:
                worst, worst_t = c, i / fps
            breaks += 1 if b else 0
            row = {"t": round(i / fps, 3), "clearance_cm": c, "rules_broken": b}
            g = self.grip("both", hold=False)
            gb = sum(len(g[s_]["bad"]) for s_ in which)
            for s_ in which:
                for txt in g[s_]["bad"]:
                    k = "%s grip: %s" % (s_, _re.sub(r"[-0-9.]+", "#", txt))
                    seen[k] = seen.get(k, 0) + 1
            grip_bad += 1 if gb else 0
            row["grip_rules_broken"] = gb
            for s_ in hold_sides:
                # holding: the palm on the gun facing it, and either the thumb opposing the fingers on it (a pinch)
                # or two or more fingers wrapped onto it against the palm (a power grip, as a pistol is held); a
                # palm laid flat on the gun with the fingers straight touches it but holds nothing
                wrapped = [d for d in g[s_].get("digits_touching", []) if d != "thumb"]
                held = g[s_]["holding"] and (g[s_]["grasping"] or len(wrapped) >= 2)
                if not held:
                    not_holding[s_] += 1
                row["holding_" + s_] = held
            per.append(row)
        return {"clip": clip, "frames": len(frames), "clearance_worst_cm": round(worst, 2), "clearance_worst_s": round(worst_t, 3),
                "clearance_over_frames": sum(1 for r in per if r["clearance_cm"] > clearance_max_cm),
                "anatomy_break_frames": breaks, "grip_break_frames": grip_bad, "not_holding_frames": not_holding,
                "rules": dict(sorted(seen.items(), key=lambda kv: -kv[1])), "per_frame": per}

    def render(self, views=("eye", "right", "left", "top"), size=(640, 360), ortho_cm=None):
        """Renders the pose: eye is the player's camera (90 degrees, 5 cm near plane); right, left, top and front
        look at the rifle from outside. ortho_cm: those outside views straight on (orthographic), ortho_cm across,
        so sizes compare true (a perspective view draws the hands, nearer the lens, bigger than the gun). Images go
        to the output folder."""
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
                cam_data.type = 'PERSP'
                cam_data.angle = math.radians(H_FOV)
                cam_data.clip_start = 0.05
                cam.location = self.eye
                cam.rotation_euler = VIEW_FWD.to_track_quat('-Z', 'Y').to_euler()
            elif v in spots:
                cam_data.type = 'ORTHO' if ortho_cm else 'PERSP'
                if ortho_cm:
                    cam_data.ortho_scale = ortho_cm / 100.0
                cam_data.angle = math.radians(40.0)
                cam_data.clip_start = 0.01
                cam.location = G @ ue(base + Vector(spots[v]))
                if ortho_cm:
                    # straight on, the frame centred on the hold (the hands are below the bore)
                    drop = G.to_quaternion() @ Vector((0.0, 0.0, -0.05))
                    centre_v = centre + drop
                    cam.location = cam.location + drop
                else:
                    centre_v = centre
                zc = (cam.location - centre_v).normalized()
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
