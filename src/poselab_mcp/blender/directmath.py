"""The direct hold's equations, in plain Python so they are tested without Blender (docs/DIRECT_HOLD_PLAN.md)."""
import math


def reach_for_elbow(l1, l2, flexion_deg):
    """The shoulder-to-wrist distance for an elbow flexed flexion_deg (0 = straight): the law of cosines with the
    elbow's inner angle 180 - flexion, D^2 = l1^2 + l2^2 + 2 l1 l2 cos(flexion)."""
    f = math.radians(flexion_deg)
    return math.sqrt(max(0.0, l1 * l1 + l2 * l2 + 2.0 * l1 * l2 * math.cos(f)))


def elbow_for_reach(l1, l2, reach):
    """The elbow flexion (degrees) that gives this shoulder-to-wrist distance; None when it cannot be reached."""
    c = (reach * reach - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if c > 1.0 + 1e-9 or c < -1.0 - 1e-9:
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def bisect(fn, target, lo, hi, tol=1e-4, most=60):
    """The x in [lo, hi] with fn(x) = target, for fn growing with x. Returns (x, fn(x), steps); when the target is
    outside [fn(lo), fn(hi)] the nearer end is returned (the caller reports the miss)."""
    f_lo, f_hi = fn(lo), fn(hi)
    if target <= f_lo:
        return lo, f_lo, 0
    if target >= f_hi:
        return hi, f_hi, 0
    n = 0
    while hi - lo > tol and n < most:
        mid = 0.5 * (lo + hi)
        f_mid = fn(mid)
        if f_mid < target:
            lo = mid
        else:
            hi = mid
        n += 1
    x = 0.5 * (lo + hi)
    return x, fn(x), n
