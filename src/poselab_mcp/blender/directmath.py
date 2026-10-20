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


def preferred_lop(S, pocket, bore, up, grip_drop, l1, l2, f_pref_deg, lop_range=(0.30, 0.40)):
    """The preferred length of pull for a long gun (the owner's idea, 2026-10-20: "calculate a preferred length before
    match ... better for all the guns"). The butt rests in the shoulder pocket; the firing hand's grip point sits
    LOP along the bore from it and grip_drop under the bore line:

        G(LOP) = pocket + bore * LOP - up * grip_drop,     |G(LOP) - S| = D*,  D*^2 = l1^2 + l2^2 + 2 l1 l2 cos f_pref

    |G - S| grows with LOP while the bore leads away from the shoulder, so bisection finds it; it is then held to
    the stock's range (lop_range, metres). Points and directions: 3 numbers each (metres, unit). Returns (LOP the
    arm prefers, LOP used, whether the stock's range cut it)."""
    def G(lop):
        return [pocket[i] + bore[i] * lop - up[i] * grip_drop for i in range(3)]

    def dist(lop):
        g_ = G(lop)
        return math.sqrt(sum((g_[i] - S[i]) ** 2 for i in range(3)))
    target = reach_for_elbow(l1, l2, f_pref_deg)
    lop, _got, _n = bisect(dist, target, 0.0, 1.0, tol=1e-5)
    used = min(max(lop, lop_range[0]), lop_range[1])
    return lop, used, abs(used - lop) > 1e-9
