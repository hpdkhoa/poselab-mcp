# The built-in sample rig, made in Blender from code so it is free to share: two arms (Unreal mannequin bone names) in
# sleeves holding an AR-style rifle with a charging handle on its right side that slides back. Coordinates are
# Unreal-style cm: arms space for the body, gun space for the rifle (+X the gun's left, +Y along the barrel, +Z up),
# the gun bone at the grip.
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

EYE = (0.0, 0.0, 160.0)
GUN = (-6.0, 22.0, 146.0)                        # the gun bone (the grip), arms space
SHOULDER = {"r": (-17.0, -6.0, 145.0), "l": (17.0, -6.0, 145.0)}
POLE = {"r": (-45.0, -20.0, 95.0), "l": (45.0, -20.0, 95.0)}
UPPER, LOWER = 28.0, 26.0
# each hand, gun space: the wrist, the way the knuckles point, the way the palm faces, the index finger's side.
# The right hand holds the pistol grip as a fist round it: the grip runs up through the fist (index on top, pinky
# below), the palm on its right side, the fingers round its front. The left hand is under the handguard, palm up,
# the fingers across it and curling up round its far side.
HANDS = {"r": ((-3.6, -9.5, -5.0), (0.25, 1.0, 0.1), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
         "l": ((5.8, 24.0, -1.8), (-1.0, 0.3, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0))}
# across the knuckles (cm), then the three segment lengths (cm)
FINGER = {"index": (2.4, 4.0, 2.4, 2.0), "middle": (0.8, 4.4, 2.7, 2.1), "ring": (-0.8, 4.1, 2.5, 2.0),
          "pinky": (-2.4, 3.2, 2.0, 1.8), "thumb": (3.0, 3.4, 2.8, 2.3)}
CURL = 0.95                                     # how much each finger joint bends toward the palm (about 45 degrees)
FINGER_R = {"index": 0.85, "middle": 0.9, "ring": 0.85, "pinky": 0.75, "thumb": 1.0}   # cm
COLOURS = {"receiver": (0.045, 0.047, 0.05), "furniture": (0.07, 0.07, 0.075), "rail": (0.11, 0.11, 0.12),
           "steel": (0.2, 0.21, 0.22), "magazine": (0.28, 0.24, 0.17), "handle": (0.5, 0.52, 0.55),
           "sleeve": (0.2, 0.23, 0.17), "cuff": (0.12, 0.13, 0.1), "skin": (0.72, 0.53, 0.42)}
CONFIG = {
    "eye": list(EYE), "gun_offset": [0.0, 0.0, 0.0], "parts": {"carrier": ["carrier"]},
    "points": {"port": [-1.5, 9.0, 4.0], "bore": [0.0, 0.0, 3.5], "stock": [0.0, -25.0, 0.0],
               "muzzle": [0.0, 64.0, 3.5], "handle": [-2.6, 4.0, 4.5]},
    "normals": {"port": [-1.0, 0.0, 0.0]},
    "poles": {k: list(v) for k, v in POLE.items()},
}


def ue(v):
    return Vector((v[0], -v[1], v[2])) / 100.0


def material(name):
    m = bpy.data.materials.get("poselab_" + name)
    if not m:
        m = bpy.data.materials.new("poselab_" + name)
        m.diffuse_color = COLOURS[name] + (1.0,)
        m.roughness = 0.35 if name in ("steel", "handle") else 0.6
        m.metallic = 0.6 if name in ("steel", "handle") else 0.0
    return m


def new_object(name, bm, mat, smooth=True):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(material(mat))
    for p in me.polygons:
        p.use_smooth = smooth
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


# --- the rifle ----------------------------------------------------------------------------------------------------
def rifle_box(lo, hi, bevel=0.0025, turn_deg=0.0):
    """A bevelled box from lo to hi (gun space, cm), turned about the gun's left-right axis through its middle."""
    lo, hi = ue(lo), ue(hi)
    size = Vector((abs(hi.x - lo.x), abs(hi.y - lo.y), abs(hi.z - lo.z)))
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Diagonal(size.to_4d()))
    b = min(bevel, min(size) * 0.3)
    if b > 0:
        bmesh.ops.bevel(bm, geom=list(bm.edges), offset=b, segments=2, profile=0.5, affect='EDGES', clamp_overlap=True)
    bmesh.ops.transform(bm, matrix=Matrix.Translation((lo + hi) / 2) @ Matrix.Rotation(math.radians(turn_deg), 4, 'X'), verts=bm.verts)
    return bm


def rifle_tube(y0, y1, r, z=3.5, segments=24):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segments, radius1=r / 100.0, radius2=r / 100.0, depth=(y1 - y0) / 100.0)
    bmesh.ops.transform(bm, matrix=Matrix.Translation(ue((0.0, (y0 + y1) / 2, z))) @ Matrix.Rotation(math.radians(90), 4, 'X'), verts=bm.verts)
    return bm


def rifle_parts():
    """(name, bmesh, material, vertex group, smooth) for every part of the rifle."""
    P = []
    add = lambda name, bm, mat, group="body", smooth=False: P.append((name, bm, mat, group, smooth))
    add("upper", rifle_box((-1.5, -6.0, 2.0), (1.5, 16.0, 6.5)), "receiver")
    add("lower", rifle_box((-1.4, -4.0, -1.5), (1.4, 12.0, 2.2)), "receiver")
    add("handguard", rifle_box((-2.3, 16.0, 0.6), (2.3, 38.0, 6.2), bevel=0.006), "furniture")
    for i in range(6):   # vents along the handguard's sides
        y = 19.0 + i * 3.2
        add("vent_l%d" % i, rifle_box((2.2, y, 2.4), (2.45, y + 1.8, 4.4), bevel=0.001), "rail")
        add("vent_r%d" % i, rifle_box((-2.45, y, 2.4), (-2.2, y + 1.8, 4.4), bevel=0.001), "rail")
    for i in range(40):  # the top rail's teeth
        y = -5.5 + i * 1.08
        add("rail%d" % i, rifle_box((-1.05, y, 6.2), (1.05, y + 0.55, 6.9), bevel=0.0008), "rail")
    add("rail_base", rifle_box((-1.0, -6.0, 6.0), (1.0, 38.0, 6.4), bevel=0.0008), "rail")
    add("rear_sight", rifle_box((-0.9, -4.5, 6.9), (0.9, -1.5, 9.2), bevel=0.003), "furniture")
    add("front_sight", rifle_box((-0.9, 33.5, 6.9), (0.9, 36.0, 8.3), bevel=0.002), "furniture")
    add("front_post", rifle_box((-0.2, 34.4, 8.3), (0.2, 35.0, 10.2), bevel=0.0005), "steel")
    add("barrel", rifle_tube(38.0, 60.0, 0.95), "steel", smooth=True)
    add("muzzle", rifle_tube(58.0, 64.0, 1.35, segments=12), "steel")
    add("buffer_tube", rifle_tube(-20.0, -6.0, 1.5), "receiver", smooth=True)
    add("stock", rifle_box((-1.9, -32.0, -3.5), (1.9, -18.0, 5.2), bevel=0.008), "furniture")
    add("butt_pad", rifle_box((-2.0, -33.0, -4.0), (2.0, -31.5, 5.6), bevel=0.006), "receiver")
    add("grip", rifle_box((-1.4, -6.5, -11.0), (1.4, -2.0, -0.5), bevel=0.006, turn_deg=-18.0), "furniture")
    add("trigger_guard", rifle_box((-0.6, -2.0, -4.2), (0.6, 5.0, -3.6), bevel=0.001), "receiver")
    add("trigger", rifle_box((-0.3, 0.4, -3.6), (0.3, 1.0, -1.4), bevel=0.001, turn_deg=12.0), "steel")
    add("mag_well", rifle_box((-1.6, 5.0, -4.0), (1.6, 12.5, -1.0), bevel=0.003), "receiver")
    add("magazine", rifle_box((-1.2, 5.6, -12.0), (1.2, 11.6, -3.5), bevel=0.004), "magazine")
    add("magazine_low", rifle_box((-1.2, 6.8, -17.5), (1.2, 12.8, -11.5), bevel=0.004, turn_deg=10.0), "magazine")
    add("dust_cover", rifle_box((-1.62, 6.0, 2.6), (-1.5, 12.5, 5.6), bevel=0.0005), "rail")
    add("carrier", rifle_box((-2.8, 2.0, 3.5), (-1.5, 6.0, 5.5), bevel=0.002), "handle", group="carrier")
    return P


def build_rifle():
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
    obs = []
    for name, bm, mat, group, smooth in rifle_parts():
        ob = new_object("rifle_" + name, bm, mat, smooth)
        g = ob.vertex_groups.new(name=group)
        g.add(list(range(len(ob.data.vertices))), 1.0, 'REPLACE')
        obs.append(ob)
    # one mesh: the clearance tests and the eye's rays see the rifle as one object
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    for o in obs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = obs[0]
    bpy.ops.object.join()
    ob = obs[0]
    ob.name = ob.data.name = "sample_rifle_mesh"
    ob.parent = rig
    mod = ob.modifiers.new("rig", 'ARMATURE')
    mod.object = rig
    return rig, ob


# --- the arms -----------------------------------------------------------------------------------------------------
def _elbow(S, W, pole):
    d = min((W - S).length, (UPPER + LOWER) / 100.0 - 1e-4)
    axis = (W - S).normalized()
    side = ((pole - S) - axis * (pole - S).dot(axis)).normalized()
    l1, l2 = UPPER / 100.0, LOWER / 100.0
    x = (l1 * l1 - l2 * l2 + d * d) / (2 * d)
    return S + axis * x + side * max(l1 * l1 - x * x, 0.0) ** 0.5


def build_arms():
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
        w, d, palm, index_side = HANDS[s]
        W = G + ue(w)
        E = _elbow(S, W, ue(POLE[s]))
        up = eb.new("upperarm_" + s)
        up.head, up.tail, up.parent = S, E, root
        lo = eb.new("lowerarm_" + s)
        lo.head, lo.tail, lo.parent, lo.use_connect = E, W, up, True
        fwd = ue(d).normalized()
        palm_n = ue(palm)
        palm_n = (palm_n - fwd * palm_n.dot(fwd)).normalized()          # the way the palm faces
        palm_up = -palm_n                                               # the back of the hand
        across = palm_n.cross(fwd).normalized()                         # across the knuckles, to the index side
        if across.dot(ue(index_side)) < 0:
            across = -across
        hand = eb.new("hand_" + s)
        hand.head, hand.tail, hand.parent, hand.use_connect = W, W + fwd * 0.085, lo, True
        hand.align_roll(palm_up)       # the hand bone's Z: out of the back of the hand, its X: across the knuckles
        for f, (off, *lengths) in FINGER.items():
            start = W + fwd * (0.035 if f == "thumb" else 0.085) + across * off / 100.0 + palm_n * (0.012 if f == "thumb" else 0.0)
            direction = (fwd * 0.7 + across * 0.5 + palm_n * 0.5).normalized() if f == "thumb" else (fwd + palm_n * 0.15).normalized()
            parent = hand
            for j, L in zip(("01", "02", "03"), lengths):
                b = eb.new("%s_%s_%s" % (f, j, s))
                b.head, b.tail, b.parent = start, start + direction * L / 100.0, parent
                b.use_connect = parent is not hand
                b.align_roll(palm_up)
                start, parent = b.tail.copy(), b
                if f != "thumb":
                    direction = (direction - palm_up * CURL).normalized()   # each joint curls toward the palm
    bpy.ops.object.mode_set(mode='OBJECT')
    return arm


def _tube(r0, r1, length, segments=24):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segments, radius1=r0, radius2=r1, depth=length)
    bmesh.ops.transform(bm, matrix=Matrix.Translation((0, 0, length / 2)), verts=bm.verts)
    return bm


def _ball(r):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=20, v_segments=12, radius=r)
    return bm


def _attach(arm, bone, ob, local):
    """ob placed by `local` in the bone's frame (its Z along the bone from the head), parented to the bone."""
    b = arm.data.bones[bone]
    head = arm.matrix_world @ b.head_local
    frame = (arm.matrix_world @ b.matrix_local).to_3x3()
    # the bone frame has Y along the bone; the shapes are made along Z: turn Z onto Y
    M = Matrix.Translation(head) @ frame.to_4x4() @ Matrix.Rotation(math.radians(-90), 4, 'X') @ local
    ob.matrix_world = M
    mw = ob.matrix_world.copy()
    ob.parent, ob.parent_type, ob.parent_bone = arm, 'BONE', bone
    bpy.context.view_layer.update()
    ob.matrix_world = mw
    return ob


def build_body(arm):
    """Sleeves, cuffs, skin-toned hands with palms, rounded joints and tapered fingers, each on its bone."""
    obs = []
    for s in ("l", "r"):
        L = lambda n: arm.data.bones[n].length
        obs.append(_attach(arm, "upperarm_" + s, new_object("sleeve_up_" + s, _tube(0.052, 0.046, L("upperarm_" + s)), "sleeve"), Matrix()))
        obs.append(_attach(arm, "lowerarm_" + s, new_object("sleeve_fore_" + s, _tube(0.046, 0.036, L("lowerarm_" + s) - 0.03), "sleeve"), Matrix()))
        obs.append(_attach(arm, "lowerarm_" + s, new_object("elbow_" + s, _ball(0.048), "sleeve"), Matrix()))
        obs.append(_attach(arm, "lowerarm_" + s, new_object("cuff_" + s, _tube(0.038, 0.037, 0.022), "cuff"),
                            Matrix.Translation((0, 0, L("lowerarm_" + s) - 0.034))))
        obs.append(_attach(arm, "hand_" + s, new_object("wrist_" + s, _ball(0.026), "skin"), Matrix()))
        # the palm: a rounded block across the knuckles, thinner than it is wide
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Diagonal((0.085, 0.03, 0.09, 1.0)))
        bmesh.ops.bevel(bm, geom=list(bm.edges), offset=0.011, segments=3, profile=0.5, affect='EDGES', clamp_overlap=True)
        obs.append(_attach(arm, "hand_" + s, new_object("palm_" + s, bm, "skin"), Matrix.Translation((0, 0, 0.045))))
        for f in FINGER:
            r = FINGER_R[f] / 100.0
            for j, taper in (("01", 1.0), ("02", 0.9), ("03", 0.8)):
                n = "%s_%s_%s" % (f, j, s)
                obs.append(_attach(arm, n, new_object("finger_" + n, _tube(r * taper, r * taper * 0.92, L(n)), "skin"), Matrix()))
                obs.append(_attach(arm, n, new_object("knuckle_" + n, _ball(r * taper * 1.04), "skin"), Matrix()))
            n = "%s_03_%s" % (f, s)
            obs.append(_attach(arm, n, new_object("tip_" + n, _ball(r * 0.74), "skin"), Matrix.Translation((0, 0, L(n)))))
    return obs


def build():
    """The sample rig in the open scene: {arm, meshes, rifle_arm, rifle_mesh, config}."""
    arm = build_arms()
    meshes = build_body(arm)
    rifle_arm, rifle_mesh = build_rifle()
    world = bpy.data.worlds.get("poselab") or bpy.data.worlds.new("poselab")
    world.color = (0.035, 0.04, 0.05)
    bpy.context.scene.world = world
    bpy.context.view_layer.update()
    return {"arm": arm, "meshes": meshes, "rifle_arm": rifle_arm, "rifle_mesh": rifle_mesh, "config": dict(CONFIG, styled=True)}
