# The built-in sample rig, made in Blender from code so it is free to share: two arms (Unreal mannequin bone names)
# holding a simple rifle (a receiver, a barrel, a handguard, a stock, a grip, a magazine and a charging handle on its
# right side that slides back). Coordinates are Unreal-style cm: arms space for the body, gun space for the rifle
# (+X the gun's left, +Y along the barrel, +Z up), the gun bone at its grip.
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

EYE = (0.0, 0.0, 160.0)
GUN = (-6.0, 22.0, 146.0)                        # the gun bone (the grip), arms space
SHOULDER = {"r": (-17.0, -6.0, 145.0), "l": (17.0, -6.0, 145.0)}
POLE = {"r": (-45.0, -20.0, 95.0), "l": (45.0, -20.0, 95.0)}
UPPER, LOWER = 28.0, 26.0
# each wrist and the way its hand points, gun space: the right on the grip, the left under the handguard
WRIST = {"r": ((-2.5, -9.0, -8.0), (0.1, 1.0, 0.35)), "l": ((4.5, 21.0, -3.5), (-0.35, 1.0, 0.15))}
FINGER = {"index": (0.9, 3.2, 2.2, 1.8), "middle": (0.3, 3.5, 2.4, 1.9), "ring": (-0.3, 3.2, 2.2, 1.8),
          "pinky": (-0.9, 2.6, 1.8, 1.5), "thumb": (2.0, 3.0, 2.4, 2.0)}   # across the knuckles, then three lengths
RIFLE_BOXES = {  # gun space, cm: (min, max)
    "receiver": ((-1.5, -6.0, 0.0), (1.5, 16.0, 6.5)),
    "handguard": ((-2.4, 16.0, 0.5), (2.4, 38.0, 6.0)),
    "stock": ((-1.8, -32.0, -3.0), (1.8, -6.0, 5.0)),
    "grip": ((-1.4, -5.0, -11.0), (1.4, -1.0, 0.0)),
    "magazine": ((-1.2, 6.0, -16.0), (1.2, 12.0, 0.0)),
    "sight": ((-0.8, -2.0, 6.5), (0.8, 2.0, 9.0)),
}
CARRIER = ((-2.6, 2.0, 3.5), (-1.5, 6.0, 5.5))   # the handle on the right side: it slides back with the carrier
CONFIG = {
    "eye": list(EYE), "gun_offset": [0.0, 0.0, 0.0], "parts": {"carrier": ["carrier"]},
    "points": {"port": [-1.5, 9.0, 4.0], "bore": [0.0, 0.0, 3.5], "stock": [0.0, -25.0, 0.0],
               "muzzle": [0.0, 62.0, 3.5], "handle": [-2.6, 4.0, 4.5]},
    "normals": {"port": [-1.0, 0.0, 0.0]},
    "poles": {k: list(v) for k, v in POLE.items()},
}


def ue(v):
    return Vector((v[0], -v[1], v[2])) / 100.0


def _elbow(S, W, pole):
    d = min((W - S).length, (UPPER + LOWER) / 100.0 - 1e-4)
    axis = (W - S).normalized()
    side = (pole - S) - axis * (pole - S).dot(axis)
    side = side.normalized()
    l1, l2 = UPPER / 100.0, LOWER / 100.0
    x = (l1 * l1 - l2 * l2 + d * d) / (2 * d)
    return S + axis * x + side * max(l1 * l1 - x * x, 0.0) ** 0.5


def _arms():
    data = bpy.data.armatures.new("sample_arms")
    arm = bpy.data.objects.new("sample_arms", data)
    bpy.context.scene.collection.objects.link(arm)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode='EDIT')
    eb = data.edit_bones
    root = eb.new("root")
    root.head, root.tail = Vector((0, 0, 0)), Vector((0, 0, 0.1))
    G = ue(GUN)
    gun = eb.new("ik_hand_gun")
    gun.head, gun.tail, gun.parent = G, G + ue((0.0, 6.0, 0.0)), root
    for s in ("l", "r"):
        S = ue(SHOULDER[s])
        w, d = WRIST[s]
        W = G + ue(w)
        E = _elbow(S, W, ue(POLE[s]))
        up = eb.new("upperarm_" + s)
        up.head, up.tail, up.parent = S, E, root
        lo = eb.new("lowerarm_" + s)
        lo.head, lo.tail, lo.parent, lo.use_connect = E, W, up, True
        fwd = (ue(d) - ue((0, 0, 0))).normalized()
        hand = eb.new("hand_" + s)
        hand.head, hand.tail, hand.parent, hand.use_connect = W, W + fwd * 0.08, lo, True
        across = fwd.cross(Vector((0, 0, 1))).normalized() * (1 if s == "l" else -1)
        for f, (off, *lengths) in FINGER.items():
            start = W + fwd * (0.03 if f == "thumb" else 0.085) + across * off / 100.0
            direction = (fwd + across * 0.6).normalized() if f == "thumb" else fwd
            parent = hand
            for j, L in zip(("01", "02", "03"), lengths):
                b = eb.new("%s_%s_%s" % (f, j, s))
                b.head, b.tail, b.parent = start, start + direction * L / 100.0, parent
                b.use_connect = parent is not hand
                start, parent = b.tail.copy(), b
    bpy.ops.object.mode_set(mode='OBJECT')
    return arm


def _limb_meshes(arm):
    """A tube on each bone, parented to it (rendering and the eye's ray tests see the arms)."""
    meshes = []
    size = {"upperarm": 0.05, "lowerarm": 0.04, "hand": 0.03}
    for b in arm.data.bones:
        kind = b.name.rsplit("_", 1)[0]
        r = size.get(kind, 0.0075 if b.name[:-2].split("_")[0] in FINGER else 0.0)
        if r <= 0.0:
            continue
        head, tail = arm.matrix_world @ b.head_local, arm.matrix_world @ b.tail_local
        bm = bmesh.new()
        bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=r, radius2=r * 0.85, depth=(tail - head).length)
        me = bpy.data.meshes.new("limb_" + b.name)
        bm.to_mesh(me)
        bm.free()
        for poly in me.polygons:
            poly.use_smooth = True
        ob = bpy.data.objects.new("limb_" + b.name, me)
        bpy.context.scene.collection.objects.link(ob)
        q = Vector((0, 0, 1)).rotation_difference((tail - head).normalized())
        ob.matrix_world = Matrix.Translation((head + tail) / 2) @ q.to_matrix().to_4x4()
        mw = ob.matrix_world.copy()
        ob.parent, ob.parent_type, ob.parent_bone = arm, 'BONE', b.name
        bpy.context.view_layer.update()
        ob.matrix_world = mw
        meshes.append(ob)
    return meshes


def _rifle():
    data = bpy.data.armatures.new("sample_rifle")
    rig = bpy.data.objects.new("sample_rifle", data)
    bpy.context.scene.collection.objects.link(rig)
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='EDIT')
    body = data.edit_bones.new("body")
    body.head, body.tail = Vector((0, 0, 0)), ue((0.0, 5.0, 0.0))
    car = data.edit_bones.new("carrier")
    car.head, car.tail, car.parent = ue((-2.0, 4.0, 4.5)), ue((-2.0, 8.0, 4.5)), body
    bpy.ops.object.mode_set(mode='OBJECT')
    bm = bmesh.new()
    groups = []

    def box(lo, hi, group):
        lo, hi = ue(lo), ue(hi)
        centre = (lo + hi) / 2
        scale = Vector((abs(hi.x - lo.x), abs(hi.y - lo.y), abs(hi.z - lo.z)))
        made = bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(centre) @ Matrix.Diagonal(scale.to_4d()))
        groups.extend((v, group) for v in made["verts"])

    for lo, hi in RIFLE_BOXES.values():
        box(lo, hi, "body")
    box(*CARRIER, "carrier")
    made = bmesh.ops.create_cone(bm, cap_ends=True, segments=16, radius1=0.011, radius2=0.011, depth=0.46,
                                 matrix=Matrix.Translation(ue((0.0, 39.0, 3.5))) @ Matrix.Rotation(math.radians(90), 4, 'X'))
    groups.extend((v, "body") for v in made["verts"])
    bm.verts.index_update()
    index = {v: v.index for v, g in groups}
    me = bpy.data.meshes.new("sample_rifle_mesh")
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new("sample_rifle_mesh", me)
    bpy.context.scene.collection.objects.link(ob)
    vg = {g: ob.vertex_groups.new(name=g) for g in ("body", "carrier")}
    for v, g in groups:
        vg[g].add([index[v]], 1.0, 'REPLACE')
    ob.parent = rig
    mod = ob.modifiers.new("rig", 'ARMATURE')
    mod.object = rig
    return rig, ob


def build():
    """The sample rig in the open scene: {arm, meshes, rifle_arm, rifle_mesh, config}."""
    arm = _arms()
    meshes = _limb_meshes(arm)
    rifle_arm, rifle_mesh = _rifle()
    bpy.context.view_layer.update()
    return {"arm": arm, "meshes": meshes, "rifle_arm": rifle_arm, "rifle_mesh": rifle_mesh, "config": dict(CONFIG)}
