"""Is each surface sensor's response linear in how much of it sees black?

Paul's reflectance exercise 05, on runs we already have (the robot spinning
over the course tape), no new test.

Each tape crossing gives a reading profile. Its centre is the midpoint of the
two 50 % crossings, which a reading lag can't move (both crossings shift the
same way). Around that centre the tape covers a share c of the sensor's
Gaussian spot. Three models of reading against c are fitted on equal terms:

    linear     raw = W + S c
    power law  raw = W + S c^gamma
    quadratic  raw = W + S1 c + S2 c^2

Each also fits the spot size (rho = sigma / tape width) and the tape's true
length along the track (kappa times the distance between the 50 % crossings).
A symmetric spot with a linear response puts the 50 % crossings on the true
edges (kappa = 1); a curved response moves them, and skews each edge.

Held back: fit on every other crossing, test on the rest, then swap.

    python3 bench/3.2_sensor_levels/nonlinearity.py logs/spread_profile_test2.csv logs/spread_cmd70_01.csv
"""
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.special import erf
from scipy.stats import norm

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import surface_map as sm                       # noqa: E402
from sensor_levels import label_readings, SURFACE_MAX, MARGIN_MM   # noqa: E402

SEPARATION = 83.2      # what levels.csv used
TAPE_MM = 18.5         # ruler
REACH = 2.0            # keep readings within this many half-lengths of a crossing's centre
BINS = np.linspace(0, 1, 11)


class Quiet:
    def print(self, *a, **k):
        pass


def crossings(path):
    """Per sensor: offset from the crossing's centre / half-length, half-length (mm), raw, crossing id."""
    data = sm.load(path, None, None)
    levels = sm.darkness(data, Quiet())
    x, y, theta = sm.odometry(data["enc_left"], data["enc_right"], sm.WHEEL_RADIUS_MM,
                              SEPARATION, sm.COUNTS_PER_REV)
    out = {}
    for name, angle, level in zip(sm.SENSORS, sm.SENSOR_ANGLES_DEG, levels):
        lx = sm.SENSOR_RADIUS_MM * math.cos(math.radians(angle))
        ly = sm.SENSOR_RADIUS_MM * math.sin(math.radians(angle))
        sx = x + lx * np.cos(theta) - ly * np.sin(theta)
        sy = y + lx * np.sin(theta) + ly * np.cos(theta)
        raw = data[name]
        _, passes, track, unpaired = label_readings(raw, level, sx, sy, data["t_ms"], MARGIN_MM)
        u, h, r, k = [], [], [], []
        for i, (a, b) in enumerate(passes):
            c, half = (a + b) / 2, (b - a) / 2
            near = np.abs(track - c) < REACH * half + 6
            for p in unpaired:
                near &= np.abs(track - p) > 15
            near &= raw < SURFACE_MAX - 2
            u.append((track[near] - c) / half)
            h.append(np.full(near.sum(), half))
            r.append(raw[near])
            k.append(np.full(near.sum(), i))
        out[name] = tuple(map(np.concatenate, (u, h, r, k)))
    return out


def coverage(u, h, kappa, rho):
    """Share of the spot over the tape. u, h: offset/half-length, half-length (mm)."""
    half = kappa                          # true half-length in units of the 50 % half-length
    sigma = rho * 2 * kappa               # sigma in the same units (rho = sigma / width)
    s = sigma * math.sqrt(2)
    return np.clip(0.5 * (erf((u + half) / s) - erf((u - half) / s)), 0, 1)


def design(c, model, gamma=1.0):
    one = np.ones_like(c)
    if model == "linear":
        return np.column_stack([one, c])
    if model == "power":
        return np.column_stack([one, c ** gamma])
    return np.column_stack([one, c, c * c])


def sse(params, model, u, h, r):
    kappa, rho = params[0], params[1]
    gamma = params[2] if model == "power" else 1.0
    if not (0.6 < kappa < 1.6 and 0.02 < rho < 0.6 and 0.2 < gamma < 5):
        return 1e18, None
    A = design(coverage(u, h, kappa, rho), model, gamma)
    coef, *_ = np.linalg.lstsq(A, r, rcond=None)
    return float(np.sum((A @ coef - r) ** 2)), coef


def fit(model, u, h, r):
    starts = [[1.0, 0.18], [1.1, 0.15], [0.95, 0.22]]
    if model == "power":
        starts = [s + [g] for s in starts for g in (0.8, 1.0, 1.4)]
    best = None
    for x0 in starts:
        res = minimize(lambda p: sse(p, model, u, h, r)[0], x0, method="Nelder-Mead",
                       options={"xatol": 1e-4, "fatol": 1e-3, "maxiter": 4000})
        if best is None or res.fun < best.fun:
            best = res
    return best.x, sse(best.x, model, u, h, r)[1]


def predict(params, coef, model, u, h):
    gamma = params[2] if model == "power" else 1.0
    return design(coverage(u, h, params[0], params[1]), model, gamma) @ coef


def analyse(name, u, h, r, k):
    print(f"\n{name}: {len(r)} readings over {len(np.unique(k))} crossings")
    print(f"  {'model':10s} {'kappa':>6s} {'sigma mm':>8s} {'gamma':>6s} {'fit rms':>8s} "
          f"{'held-back rms':>13s} {'worst bin':>9s}  c at 50% / 30%  edge shift mm (50% / 30%)")
    res = {}
    for model in ("linear", "power", "quadratic"):
        params, coef = fit(model, u, h, r)
        fit_rms = math.sqrt(np.mean((predict(params, coef, model, u, h) - r) ** 2))
        test_err, binned = [], []
        for hold in (0, 1):
            tr, te = (k % 2) != hold, (k % 2) == hold
            p, c = fit(model, u[tr], h[tr], r[tr])
            e = predict(p, c, model, u[te], h[te]) - r[te]
            test_err.append(e)
            cov = coverage(u[te], h[te], p[0], p[1])
            binned.append((cov, e))
        e = np.concatenate(test_err)
        cov = np.concatenate([b[0] for b in binned])
        idx = np.digitize(cov, BINS[1:-1])
        means = [np.mean(e[idx == i]) for i in range(len(BINS) - 1) if np.sum(idx == i) > 20]
        worst = max(abs(m) for m in means)
        # coverage at which the reading is 50 % / 30 % of the way from white to black
        cs = np.linspace(0, 1, 20001)
        yv = design(cs, model, params[2] if model == "power" else 1.0) @ coef
        n = (yv - yv[0]) / (yv[-1] - yv[0])
        c50, c30 = (float(np.interp(q, n, cs)) for q in (0.5, 0.3))
        sigma_mm = params[1] * TAPE_MM
        # how far inside the true edge the sensor reads 50 % / 30 % (positive: inside the tape)
        shift50, shift30 = (sigma_mm * norm.ppf(c) for c in (c50, c30))
        g = f"{params[2]:.2f}" if model == "power" else "-"
        print(f"  {model:10s} {params[0]:6.3f} {sigma_mm:8.2f} {g:>6s} {fit_rms:8.1f} "
              f"{math.sqrt(np.mean(e ** 2)):13.1f} {worst:9.1f}  {c50:.2f} / {c30:.2f}"
              f"        {shift50:+.2f} / {shift30:+.2f}")
        res[model] = dict(kappa=params[0], sigma=sigma_mm, gamma=params[2] if model == "power" else 1,
                          test=math.sqrt(np.mean(e ** 2)), worst=worst, c50=c50, c30=c30,
                          shift50=shift50, shift30=shift30, means=means, span=coef[1:].sum())
    return res


OUT = Path(__file__).resolve().parent / "nonlinearity"


def main():
    import csv
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    OUT.mkdir(exist_ok=True)
    runs = sys.argv[1:]
    fig, axes = plt.subplots(len(runs), 5, figsize=(16, 3.2 * len(runs)), sharey=True, squeeze=False)
    rows = []
    for row, path in enumerate(runs):
        print(f"===== {Path(path).name}  (tape {TAPE_MM} mm by ruler)")
        for col, (name, (u, h, r, k)) in enumerate(crossings(path).items()):
            ax = axes[row][col]
            if len(np.unique(k)) < 6:
                print(f"\n{name}: too few crossings")
                continue
            res = analyse(name, u, h, r, k)
            mids = (BINS[:-1] + BINS[1:]) / 2
            for model, colour in (("linear", "#222222"), ("power", "#d0622b"), ("quadratic", "#2b6cd0")):
                m = res[model]
                print(f"  held-back mean residual per coverage bin, {model:9s}:",
                      " ".join(f"{v:+.0f}" for v in m["means"]))
                ax.plot(mids[:len(m["means"])], m["means"], "o-", ms=3, color=colour, label=model)
                rows.append({"run": Path(path).stem, "sensor": name, "model": model,
                             **{key: round(float(v), 4) for key, v in m.items() if key != "means"}})
            ax.axhline(0, color="#999999", lw=0.8)
            ax.set_title(f"{Path(path).stem}: {name}", fontsize=9)
            ax.set_xlabel("share of the spot over the tape")
    axes[0][0].set_ylabel("held-back residual (raw counts)")
    axes[0][0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "residuals.png", dpi=120)
    with open(OUT / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"\nsaved {OUT / 'residuals.png'} and {OUT / 'summary.csv'}")


if __name__ == "__main__":
    main()
