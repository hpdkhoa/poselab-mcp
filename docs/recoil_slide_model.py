"""Recoil time model with the slide (owner 2026-10-22: "yes add the slide and show numbers").

Bodies along the bore (x > 0 = toward the shooter):
  slide (+ bushing, plug, sights): m_s; barrel: m_b, locked to the slide until it has gone back d_b, then it stops on
  the frame (a dead hit: its momentum goes into the frame and it rides the frame from then on);
  frame (everything else, magazine loaded): m_f, held by the two-arm chain (fitted to Zhang 2021 Fig. 8, straight arm).
Forces:
  the shot: half-sine, 3.762 N*s over 1.045 ms, on slide + barrel (the frame gets none of it directly);
  recoil spring 5013200 between slide and frame: 8.00 lb at 3.72 in (battery), 2.88 lb/in;
  hammer (mainspring 5013208: 22.0 lb at 1.312 in, 29.5 lb at 1.062 in): its cocking work (0.727 J) spread as an even
  push over the slide's whole travel back (the hammer stroke itself is not measured);
  steel stops: at full travel (4.755 cm, our model; the drawing's 3.72 - 1.81 = 1.91 in) and at battery (0): a stiff
  spring K_c with damping ratio zeta (no source: run at several values).
Out: the force each hand puts on the frame, over time; the slide's travel; the cycle time."""
import json
import math
import sys

import numpy as np

sys.path.insert(0, ".")
import recoil_time_model as R  # noqa: E402

R.LO = np.log([0.005, 0.01, 0.05, 10, 10, 10, 1, 1, 1])
LBF, IN = 4.4482216, 0.0254
P_ARM = np.load("recoil_out/p180_wide.npy")


def run(m_s=0.380, m_b=0.098, gun=1.2528, d_b=0.25 * IN, travel=0.04755, K_c=1e8, zeta=0.1, arms=2,
        I=3.762, t_b=1.045e-3, t_end=0.08, dt=2e-7, keep_every=50, feed=True, lock=True,
        m_round=331 * 0.06479891e-3, rim_at=0.0402, lips_end=0.01234, F_strip=2 * 0.35 * 11.0 * 4.4482216, F_ret=0.0):
    """feed: on the way forward the slide meets the top round's rim at rim_at (behind battery), takes it along (a dead
    hit: the round's mass joins the slide) and strips it under the lips with friction F_strip until its rim passes
    lips_end; lock: at d_b the slide picks up the stopped barrel (a dead hit) and carries it into battery."""
    m_f = gun - m_s - m_b
    m1, m2, m3, k1, k2, k3, c1, c2, c3 = R.unpack(P_ARM)
    m1, m2, m3, k1, k2, k3, c1, c2, c3 = (arms * v for v in (m1, m2, m3, k1, k2, k3, c1, c2, c3))
    F0 = 8.00 * LBF                        # spring at battery
    kr = 2.88 * LBF / IN                   # spring rate N/m
    W_h = 0.5 * (22.0 + 29.5) * LBF * (1.312 - 1.062) * IN
    F_h = W_h / travel                     # hammer: its cocking work spread over the travel
    A = math.pi * I / (2 * t_b)
    # state: slide x_s, v_s; barrel x_b, v_b (x_b == x_s while locked); frame+contact flesh x_f, v_f; arm x2, v2, x3, v3
    xs = vs = xb = vb = xf = vf = x2 = v2 = x3 = v3 = 0.0
    locked, barrel_stopped = True, False
    M_front = m_f + m1
    m_s0 = m_s
    rec = []
    returning, fed, stripping, relocked = False, False, False, False
    hits = []
    n = int(t_end / dt)
    prev_contact = None
    for i in range(n + 1):
        t = i * dt
        shot = A * math.sin(math.pi * t / t_b) if t <= t_b else 0.0
        rel = xs - xf                                  # slide back from battery, relative to the frame
        f_int = F0 + kr * max(rel, 0.0)                # spring pushes slide forward, frame back
        f_int += F_h if (vs - vf) > 0 and rel < travel else 0.0
        vr = vs - vf
        c_c_back = 2 * zeta * math.sqrt(K_c * m_s)
        stop = 0.0
        if rel > travel:
            stop = K_c * (rel - travel) + c_c_back * vr
            stop = max(stop, 0.0)
        elif rel < 0:
            stop = K_c * rel + c_c_back * vr               # negative: pushes slide back, frame forward
            stop = min(stop, 0.0)
        contact = "back" if stop > 0 else ("front" if stop < 0 else None)
        if contact and contact != prev_contact:
            hits.append([round(t * 1e3, 3), contact])
        prev_contact = contact
        f_strip = F_strip if (stripping and (vs - vf) < 0) else 0.0
        f_strip += F_ret if (returning and (vs - vf) < 0 and rel > 0) else 0.0   # fitted return drag (rails, hammer, extractor, ramp)
        f_on_slide = -f_int - stop + f_strip            # spring and stop push the slide forward; stripping holds it back
        f_on_frame = +f_int + stop - f_strip            # the magazine (in the frame) is dragged forward
        # arm chain forces on the front mass (frame + contact flesh)
        fa1 = -k1 * (xf - x2) - c1 * (vf - v2)
        fa2 = k1 * (xf - x2) + c1 * (vf - v2) - k2 * (x2 - x3) - c2 * (v2 - v3)
        fa3 = k2 * (x2 - x3) + c2 * (v2 - v3) - k3 * x3 - c3 * v3
        if locked:
            a_s = (shot + f_on_slide) / (m_s + m_b)
            a_b = a_s
        else:
            a_s = f_on_slide / m_s
            a_b = 0.0
        mf_now = M_front + (m_b if (barrel_stopped and not relocked) else 0.0)
        a_f = (f_on_frame + fa1) / mf_now
        hand_on_frame = fa1                             # what the hands push on the frame (negative = forward push)
        if i % keep_every == 0:
            rec.append((t, xs, xf, -hand_on_frame / arms, rel, vs))
        vs += a_s * dt; xs += vs * dt
        if locked:
            vb, xb = vs, xs
        vf += a_f * dt; xf += vf * dt
        v2 += fa2 / m2 * dt; x2 += v2 * dt
        v3 += fa3 / m3 * dt; x3 += v3 * dt
        rel_now = xs - xf
        if not returning and not locked and (vs - vf) < 0 and rel_now > 0.5 * travel:
            returning = True
        if feed and returning and not fed and rel_now <= rim_at:
            vs = (m_s * vs + m_round * vf) / (m_s + m_round)      # the round (at rest in the magazine, in the frame)
            m_s += m_round
            fed, stripping = True, True
            hits.append([round(t * 1e3, 3), "slide takes the round"])
        if stripping and rel_now <= lips_end:
            stripping = False
            hits.append([round(t * 1e3, 3), "round leaves the lips"])
        if lock and returning and barrel_stopped and not relocked and rel_now <= d_b:
            mf_old = mf_now
            vs = (m_s * vs + m_b * vf) / (m_s + m_b)                # the barrel, at rest on the frame, picked up
            m_s += m_b
            relocked = True
            hits.append([round(t * 1e3, 3), "slide picks up the barrel"])
        if locked and (xs - xf) >= d_b:
            # the barrel stops on the frame: a dead hit, momentum shared with the frame
            locked, barrel_stopped = False, True
            vs_keep = vs
            vf = (mf_now * vf + m_b * vs_keep) / (mf_now + m_b)
            hits.append([round(t * 1e3, 3), "barrel stops on frame"])
    return np.array(rec), hits


def summarize(rec, hits):
    t, xs, xf, fh, rel, vs = rec.T
    out = {"hits_ms": hits}
    out["slide_full_travel_cm"] = round(float(rel.max()) * 100, 3)
    back = [h[0] for h in hits if h[1] == "back"]
    front = [h[0] for h in hits if h[1] == "front" and back and h[0] > back[0]]
    out["slide_hits_back_at_ms"] = back[0] if back else None
    out["slide_closes_at_ms"] = front[0] if front else None
    def peak(lo, hi):
        m = (t * 1e3 >= lo) & (t * 1e3 < hi)
        if not m.any():
            return None
        j = int(np.argmax(np.abs(fh[m])))
        return [round(float(t[m][j]) * 1e3, 2), round(float(fh[m][j]))]
    b = back[0] if back else 3.0
    c = front[0] if front else 60.0
    out["each_hand_peak_while_slide_runs_back_ms_N"] = peak(0, b - 0.05)
    out["closing_hits_ms"] = front[:4]
    out["each_hand_peak_slide_hits_back_ms_N"] = peak(b - 0.05, b + 6)
    out["each_hand_peak_slide_closes_ms_N"] = peak(c - 0.05, c + 6)
    out["frame_back_max_cm"] = round(float(xf.max()) * 100, 3)
    out["frame_forward_max_cm"] = round(float(-xf.min()) * 100, 3)
    out["force_each_hand_at_ms"] = {str(ms): round(float(fh[np.argmin(np.abs(t * 1e3 - ms))])) for ms in (0.5, 1, 2, 4, 6, 8, 10, 15, 20, 25, 30, 40, 50, 60)}
    return out


if __name__ == "__main__":
    res = {}
    rec, hits = run()
    np.save("recoil_out/slide_feed_base.npy", rec)
    res["base_feed_lock"] = summarize(rec, hits)
    r0, h0 = run(feed=False, lock=False)
    np.save("recoil_out/slide_nofeed.npy", r0)
    res["no_feed_no_lock (last model)"] = summarize(r0, h0)
    for name, kw in (("strip_half_17N", dict(F_strip=17.1)), ("strip_x1.5_51N", dict(F_strip=51.4)), ("zeta_0.02", dict(zeta=0.02)),
                     ("zeta_0.5", dict(zeta=0.5)), ("lock_only", dict(feed=False)), ("feed_only", dict(lock=False)), ("one_hand", dict(arms=1))):
        r, h = run(**kw)
        res[name] = summarize(r, h)
    json.dump(res, open("recoil_out/slide_feed_model.json", "w"), indent=1)
    for k, v in res.items():
        print(k, {kk: vv for kk, vv in v.items() if kk not in ("force_each_hand_at_ms",)})
    print("base force/hand", res["base_feed_lock"]["force_each_hand_at_ms"])
