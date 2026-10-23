"""Recoil time model (owner's request 2026-10-22: "yes build the time model and show numbers").

1. Fit a 3-mass chain (hand m1, forearm m2, upper arm + body m3; each joined by a spring k and damper c, the last
   to the ground) to Zhang et al. 2021 Figure 8 (arm push-back |Z| against shake speed, along the forearm).
2. Put the gun's mass on the hand (one arm, or two arms side by side = every arm value doubled).
3. Fire: the shot's push I (3.76 N*s, SAAMI equation, FM 23-35 and TM 43-0001-27 numbers) as a half-sine over the
   bullet's time in the barrel, then follow the gun, the hand and the force between them in time.
numpy only (no scipy here): Nelder-Mead in log space, many starts; time steps of 1 microsecond."""
import json
import math
import sys

import numpy as np

F = np.array([10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500], float)
Z180 = np.array([300, 370, 450, 525, 510, 480, 410, 310, 250, 200, 155, 150, 115, 95, 115, 130, 88, 140], float)
Z90 = np.array([105, 140, 178, 165, 165, 250, 400, 420, 320, 220, 160, 120, 100, 87, 105, 125, 108, 160], float)

LO = np.log([0.05, 0.05, 0.05, 10, 10, 10, 1, 1, 1])        # m1 m2 m3 k1 k2 k3 c1 c2 c3
HI = np.log([10, 10, 30, 1e6, 1e6, 1e6, 5000, 5000, 5000])


def unpack(p):
    m1, m2, m3, k1, k2, k3, c1, c2, c3 = np.exp(np.clip(p, LO, HI))
    return m1, m2, m3, k1, k2, k3, c1, c2, c3


def mats(p, scale=1.0, gun=0.0):
    m1, m2, m3, k1, k2, k3, c1, c2, c3 = unpack(p)
    M = np.diag([m1 * scale + gun, m2 * scale, m3 * scale])
    K = scale * np.array([[k1, -k1, 0], [-k1, k1 + k2, -k2], [0, -k2, k2 + k3]])
    C = scale * np.array([[c1, -c1, 0], [-c1, c1 + c2, -c2], [0, -c2, c2 + c3]])
    return M, K, C


def zmag(p, f):
    M, K, C = mats(p)
    out = []
    for fr in f:
        w = 2 * math.pi * fr
        D = -w * w * M + 1j * w * C + K
        h = np.linalg.solve(D, np.array([1, 0, 0], complex))[0]
        out.append(abs(1.0 / (1j * w * h)))
    return np.array(out)


def err(p, target):
    return float(np.mean((np.log(zmag(p, F)) - np.log(target)) ** 2))


def nelder(fun, x0, steps=4000, scale=0.6):
    n = len(x0)
    pts = [x0] + [x0 + np.eye(n)[i] * scale for i in range(n)]
    vals = [fun(x) for x in pts]
    for _ in range(steps):
        o = np.argsort(vals)
        pts = [pts[i] for i in o]
        vals = [vals[i] for i in o]
        c = np.mean(pts[:-1], axis=0)
        r = c + (c - pts[-1])
        fr = fun(r)
        if fr < vals[0]:
            e = c + 2 * (c - pts[-1])
            fe = fun(e)
            pts[-1], vals[-1] = (e, fe) if fe < fr else (r, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = r, fr
        else:
            k = c + 0.5 * (pts[-1] - c)
            fk = fun(k)
            if fk < vals[-1]:
                pts[-1], vals[-1] = k, fk
            else:
                pts = [pts[0] + 0.5 * (x - pts[0]) for x in pts]
                vals = [fun(x) for x in pts]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def fit(target, seed=1, starts=12):
    rng = np.random.default_rng(seed)
    best = None
    for s in range(starts):
        x0 = LO + (HI - LO) * rng.random(9)
        x, v = nelder(lambda p: err(p, target), x0, steps=1500)
        x, v = nelder(lambda p: err(p, target), x, steps=1500, scale=0.2)
        if best is None or v < best[1]:
            best = (x, v)
    return best


def fire(p, arms, gun_kg, impulse, t_barrel, t_end=0.06, dt=1e-6):
    M, K, C = mats(p, scale=arms, gun=gun_kg)
    Mi = np.linalg.inv(M)
    x = np.zeros(3)
    v = np.zeros(3)
    A = math.pi * impulse / (2 * t_barrel)
    rec = []
    n = int(t_end / dt)
    for i in range(n + 1):
        t = i * dt
        shot = A * math.sin(math.pi * t / t_barrel) if t <= t_barrel else 0.0
        f = np.array([-shot, 0.0, 0.0])          # the shot pushes the gun back (negative = toward the shooter)
        a = Mi @ (f - C @ v - K @ x)
        # the hand's push on the gun = gun mass * gun accel - shot force (the part of the hand chain's force)
        hand_on_gun = gun_kg * a[0] - (-shot)
        rec.append((t, x[0], v[0], hand_on_gun))
        v = v + a * dt                             # semi-implicit Euler
        x = x + v * dt
    return np.array(rec)


def summary(rec, impulse, arms):
    t, x, v, fh = rec.T
    ipk = int(np.argmax(fh))
    stop = next((t[i] for i in range(1, len(v)) if v[i - 1] < 0 <= v[i]), None)
    cum = np.cumsum(fh) * (t[1] - t[0])
    t95 = next((t[i] for i in range(len(cum)) if cum[i] >= 0.95 * impulse), None)
    return {"peak_total_N": round(float(fh[ipk])), "peak_each_hand_N": round(float(fh[ipk]) / arms),
            "peak_at_ms": round(float(t[ipk]) * 1e3, 2), "gun_back_max_cm": round(float(-x.min()) * 100, 2),
            "gun_stops_at_ms": round(stop * 1e3, 2) if stop else None, "push_95pct_taken_ms": round(t95 * 1e3, 2) if t95 else None,
            "force_at_ms": {str(ms): round(float(fh[int(ms / 1e3 / (t[1] - t[0]))])) for ms in (0.5, 1, 2, 3, 5, 8, 12, 20, 30)}}


if __name__ == "__main__":
    out = {}
    GR, FT, LB = 0.06479891e-3, 0.3048, 0.45359237
    I = 230 * GR * 802 * FT + 5 * GR * 1.5 * 802 * FT
    gun = (2.437 + 0.481 - 0.156) * LB
    t_barrel = 2 * (5.03 * 0.0254) / (802 * FT)       # barrel 5 1/32 in (C3838), bullet speeding up evenly
    for name, target in (("180", Z180), ("90", Z90)):
        p, e = fit(target)
        m1, m2, m3, k1, k2, k3, c1, c2, c3 = unpack(p)
        zf = zmag(p, F)
        out[name] = {"fit_rms_log_error": round(math.sqrt(e), 3),
                     "params": {"m1_hand_kg": round(m1, 3), "m2_kg": round(m2, 3), "m3_kg": round(m3, 3),
                                "k1_N_per_m": round(k1), "k2_N_per_m": round(k2), "k3_N_per_m": round(k3),
                                "c1_Ns_per_m": round(c1, 1), "c2_Ns_per_m": round(c2, 1), "c3_Ns_per_m": round(c3, 1)},
                     "fit_vs_paper": [[float(a), float(b), round(float(c))] for a, b, c in zip(F, target, zf)]}
        for arms in (1, 2):
            for tb in (t_barrel, 0.5e-3, 2e-3):
                rec = fire(p, arms, gun, I, tb)
                out[name]["%d_arm_tbarrel_%.2fms" % (arms, tb * 1e3)] = summary(rec, I, arms)
            if arms == 2:
                rec = fire(p, arms, gun, I, t_barrel)
                np.save(sys.argv[1] + "/rec_%s.npy" % name, rec)
    out["inputs"] = {"impulse_Ns": round(I, 3), "gun_kg": round(gun, 3), "t_barrel_ms": round(t_barrel * 1e3, 3)}
    json.dump(out, open(sys.argv[1] + "/recoil_time_model.json", "w"), indent=1)
    print(json.dumps(out, indent=1))
