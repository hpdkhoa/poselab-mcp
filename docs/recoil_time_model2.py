"""Same model, the three masses fixed to the body's own segment masses (Plagenhoef, Evans and Abdelnour 1983, male:
hand 0.65%, forearm 1.87%, upper arm 3.25% of body mass) at Zhang's subjects' average 59 kg; only springs and dampers
are fitted. The upper arm's far end is held by the body through k3, c3."""
import json, math, sys
import numpy as np
import recoil_time_model as R

BODY = 59.0
MASS = np.log([0.0065 * BODY, 0.0187 * BODY, 0.0325 * BODY])
LO6, HI6 = R.LO[3:], R.HI[3:]

def full(q):
    return np.concatenate([MASS, np.clip(q, LO6, HI6)])

def fit6(target, seed=2, starts=12):
    rng = np.random.default_rng(seed)
    best = None
    for s in range(starts):
        x0 = LO6 + (HI6 - LO6) * rng.random(6)
        f = lambda q: R.err(full(q), target)
        x, v = R.nelder(f, x0, steps=1500)
        x, v = R.nelder(f, x, steps=1500, scale=0.2)
        if best is None or v < best[1]:
            best = (x, v)
    return full(best[0]), best[1]

if __name__ == "__main__":
    GR, FT, LB = 0.06479891e-3, 0.3048, 0.45359237
    I = 230 * GR * 802 * FT + 5 * GR * 1.5 * 802 * FT
    gun = (2.437 + 0.481 - 0.156) * LB
    tb = 2 * (5.03 * 0.0254) / (802 * FT)
    out = {}
    for name, target in (("180", R.Z180), ("90", R.Z90)):
        p, e = fit6(target)
        m1, m2, m3, k1, k2, k3, c1, c2, c3 = R.unpack(p)
        out[name] = {"fit_rms_log_error": round(math.sqrt(e), 3),
                     "params": {"m1_hand_kg": round(m1, 3), "m2_forearm_kg": round(m2, 3), "m3_upperarm_kg": round(m3, 3),
                                "k1_N_per_m": round(k1), "k2_N_per_m": round(k2), "k3_N_per_m": round(k3),
                                "c1_Ns_per_m": round(c1, 1), "c2_Ns_per_m": round(c2, 1), "c3_Ns_per_m": round(c3, 1)},
                     "fit_vs_paper": [[float(a), float(b), round(float(c))] for a, b, c in zip(R.F, target, R.zmag(p, R.F))]}
        for arms in (1, 2):
            for t_b in (tb, 0.5e-3, 2e-3):
                rec = R.fire(p, arms, gun, I, t_b, t_end=0.1)
                out[name]["%d_arm_tbarrel_%.2fms" % (arms, t_b * 1e3)] = R.summary(rec, I, arms)
                if abs(t_b - tb) < 1e-9:
                    np.save(sys.argv[1] + "/rec2_%s_%d.npy" % (name, arms), rec)
    json.dump(out, open(sys.argv[1] + "/recoil_time_model2.json", "w"), indent=1)
