"""The old hand code, marked deprecated by the owner (2026-10-20). The direct hold (holdlab.direct_hold: the hand at its
touch points on the object, then the wrist, arm, elbow and shoulder solved to it) replaces it. It is kept only while
something still needs it: Pose Lab's hold tool for an object with no marks and the cylinder benchmark. Each use warns
once and is listed in self._deprecated_used, so a run's report can say it ran old code."""
import functools
import warnings

WHY = "the old hold code; use direct_hold (the hand at its touch points, then the body solved to it)"


def deprecated(why=WHY):
    """Marks a Lab method deprecated: its docstring starts with DEPRECATED, and its first call warns and is noted."""
    def deco(f):
        @functools.wraps(f)
        def w(self, *a, **k):
            seen = self.__dict__.setdefault("_deprecated_used", set())
            if f.__name__ not in seen:
                seen.add(f.__name__)
                warnings.warn("%s is deprecated: %s" % (f.__name__, why), DeprecationWarning, stacklevel=2)
            return f(self, *a, **k)
        w.__doc__ = "DEPRECATED (2026-10-20): %s.\n\n%s" % (why, f.__doc__ or "")
        w.__deprecated__ = why
        return w
    return deco
