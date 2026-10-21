"""Every working step of a run timed and counted (0.9.0): which step called which, how many times each ran, the
time it took in all and by itself (its own time, its children's taken out), and the counts of its iterations (a
pattern search's steps tried, kept, undone at a wall; a solve's rounds; a fit's shares). Written as steps.json; the
viewer (examples/steps_view.py) draws one run or several side by side.

    S = Steps()
    with S.span("stance search"):
        ...
    S.count("starts", 13)
    S.wrap(lab, ["target_hold", "wall_faults", ...])     # methods timed as steps wherever they are called
    S.write(path, meta)"""
import functools
import json
import time
from contextlib import contextmanager


class Steps:
    def __init__(self):
        self.stack = []                    # the open steps' paths
        self.nodes = {}                    # path -> {"calls", "total_s", "child_s", "max_s", "counts"}
        self.t0 = time.perf_counter()

    def _node(self, path):
        n = self.nodes.get(path)
        if n is None:
            n = self.nodes[path] = {"calls": 0, "total_s": 0.0, "child_s": 0.0, "max_s": 0.0, "counts": {}}
        return n

    @contextmanager
    def span(self, name):
        path = (self.stack[-1] if self.stack else ()) + (name,)
        self.stack.append(path)
        t = time.perf_counter()
        try:
            yield
        finally:
            dt = time.perf_counter() - t
            self.stack.pop()
            n = self._node(path)
            n["calls"] += 1
            n["total_s"] += dt
            n["max_s"] = max(n["max_s"], dt)
            if self.stack:
                self._node(self.stack[-1])["child_s"] += dt

    def begin(self, name):
        """A step opened by hand (a script's section); end() closes it."""
        path = (self.stack[-1] if self.stack else ()) + (name,)
        self.stack.append(path)
        self._open = getattr(self, "_open", [])
        self._open.append((path, time.perf_counter()))

    def end(self):
        path, t = self._open.pop()
        while self.stack and self.stack[-1] != path:
            self.stack.pop()                       # a span left open inside (an exception): closed with it
        dt = time.perf_counter() - t
        self.stack.pop()
        n = self._node(path)
        n["calls"] += 1
        n["total_s"] += dt
        n["max_s"] = max(n["max_s"], dt)
        if self.stack:
            self._node(self.stack[-1])["child_s"] += dt

    def phase(self, name, level=0):
        """A script's sections in order: the open sections at this level and deeper closed, then this one opened.
        With out_path set, the file is written at every top-level phase, so a run that is stopped still shows where
        its time went."""
        while len(getattr(self, "_open", [])) > level:
            self.end()
        if level == 0 and getattr(self, "out_path", None):
            self.write(self.out_path, dict(getattr(self, "meta", {}), partial=True, at_phase=name))
        self.begin(name)

    def end_all(self):
        while getattr(self, "_open", []):
            self.end()

    def wrap_dict(self, d, names):
        """A script's own functions (its globals) timed as steps, like wrap."""
        for nm in names:
            f = d.get(nm)
            if f is None or getattr(f, "_steps_wrapped", False):
                continue

            def make(f, nm):
                @functools.wraps(f)
                def w(*a, **k):
                    with self.span(nm):
                        return f(*a, **k)
                w._steps_wrapped = True
                return w
            d[nm] = make(f, nm)

    def count(self, name, k=1):
        """An iteration count on the step open now (or on the run)."""
        path = self.stack[-1] if self.stack else ()
        c = self._node(path)["counts"]
        c[name] = c.get(name, 0) + k

    def wrap(self, obj, names):
        """Each named method of obj timed as a step wherever it is called (nested under its caller's step)."""
        for nm in names:
            f = getattr(obj, nm, None)
            if f is None or getattr(f, "_steps_wrapped", False):
                continue

            def make(f, nm):
                @functools.wraps(f)
                def w(*a, **k):
                    with self.span(nm):
                        return f(*a, **k)
                w._steps_wrapped = True
                return w
            setattr(obj, nm, make(f, nm))

    def tree(self):
        rows = []
        for path, n in self.nodes.items():
            if not path:
                continue
            rows.append({"path": list(path), "name": path[-1], "depth": len(path) - 1, "calls": n["calls"],
                         "total_s": round(n["total_s"], 4), "self_s": round(n["total_s"] - n["child_s"], 4),
                         "mean_ms": round(1000.0 * n["total_s"] / max(1, n["calls"]), 3), "max_ms": round(1000.0 * n["max_s"], 3),
                         "counts": n["counts"]})
        rows.sort(key=lambda r: r["path"])
        return rows

    def write(self, path, meta=None):
        out = {"meta": dict(meta or {}), "wall_s": round(time.perf_counter() - self.t0, 2),
               "run_counts": self._node(())["counts"], "steps": self.tree()}
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=1)
        return out


# the Lab's working steps and the primitives they pay for, timed wherever they run
LAB_STEPS = [
    # the object and its read
    "read_object", "object_regions", "object_sdf", "build_sdf",
    # starts
    "seed_hold", "arm_excess", "shoulder",
    # the target solver and its parts
    "direct_hold", "natural_open", "place_on_touch", "close_grab", "_grip_wrap_radius", "body_solve", "_place_arm",
    "_thumb_world", "_apply_digits", "hand_place", "start_check", "reach_now", "measure_hand", "press_flatten",
    "exact_signed", "_pierced_samples",
    "target_hold", "_place_targets", "_fit_hand", "_body_eases", "settle_palm", "conform_digits", "_fit_pads",
    "_fit_index", "_fit_thumb", "relax_fingers", "_polish", "_descend", "_walled", "repair_frame",
    # judging
    "hold_gate", "hold_purposes", "_hold_score", "wall_faults", "contact_pairs", "contact_map", "contact_pictures",
    "hold_digit_rule", "skin_presses", "frame_fault",
    # the primitives under all of it
    "_skin_arrays", "sdf_query", "_put_hand", "_set_curl", "_swing_elbow", "snapshot", "_object_now",
    # clips
    "record_clip", "save_clip", "check_set", "pose_clip", "render", "view_report",
]
