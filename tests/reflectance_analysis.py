#!/usr/bin/env python3
"""Analyse reflectance-sensor experiment CSVs written by experiments.py.

Subcommands (course exercises):
    gain           Ex 4: baseline-subtracted linear fit, held-back prediction
    nonlinearity   Ex 5: linear / quadratic / power-law models, held-back MAE
    noise          Ex 6: per-trial statistics, Gaussian vs uniform models
    footprint      Ex 7: normalised strip response z(position), peak and FWHM

Each writes PNG plots and summary.md into --out (default reports/<subcommand>).
Requires numpy and matplotlib (pip install -r requirements.txt).

Example:
    python tests\\reflectance_analysis.py gain data\\experiments\\sweep_*.csv
"""

import argparse
import csv
import glob
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

SENSORS = [1, 2, 3, 4, 5]
RAW_UNIT = "raw RC discharge time (µs)"


# ------------------------------------------------------------------ loading ---

def load_rows(paths):
    """Read one or more experiment CSVs into a list of dicts (numbers parsed)."""
    rows = []
    for pattern in paths:
        matches = sorted(glob.glob(str(pattern))) or [str(pattern)]
        for path in matches:
            with open(path, newline="", encoding="utf-8") as handle:
                for row in csv.DictReader(handle):
                    rows.append({
                        "test_name": row["test_name"],
                        "trial_num": int(row["trial_num"]),
                        "sample_num": int(row["sample_num"]),
                        "sample_time": int(row["sample_time"]),
                        "sensor_num": int(row["sensor_num"]),
                        "reading": float(row["sensor_reading"]),
                        "pct": float(row["blackness_pct"]),
                        "role": row["role"],
                        "position_mm": float(row["position_mm"]) if row["position_mm"] else None,
                        "width_mm": (float(row["rectangle_width_mm"])
                                     if row["rectangle_width_mm"] else None),
                        "direction": row["direction"],
                        "source": Path(path).name,
                    })
    if not rows:
        raise SystemExit("no rows found in {}".format(", ".join(map(str, paths))))
    return rows


def placement_medians(rows):
    """{(sensor, pct, role): [median per placement]} for surface-sweep data."""
    batches = defaultdict(list)
    for row in rows:
        key = (row["source"], row["sensor_num"], row["pct"], row["role"], row["trial_num"])
        batches[key].append(row["reading"])
    result = defaultdict(list)
    for (_, sensor, pct, role, _), values in sorted(batches.items()):
        result[(sensor, pct, role)].append(float(np.median(values)))
    return result


# ------------------------------------------------------------- pure maths ---

def linear_fit(x, y):
    """Least-squares y = m x + c. Returns (m, c, r_squared)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    m, c = np.polyfit(x, y, 1)
    predicted = m * x + c
    ss_res = float(np.sum((y - predicted) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return float(m), float(c), r2


def mae(actual, predicted):
    actual = np.asarray(actual, dtype=float)
    predicted = np.asarray(predicted, dtype=float)
    if actual.size == 0:
        return float("nan")
    return float(np.mean(np.abs(actual - predicted)))


def golden_section_min(func, lo, hi, tol=1e-6, max_iter=200):
    """Minimise a unimodal 1-D function on [lo, hi]."""
    ratio = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    c = b - ratio * (b - a)
    d = a + ratio * (b - a)
    fc, fd = func(c), func(d)
    for _ in range(max_iter):
        if abs(b - a) < tol:
            break
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - ratio * (b - a)
            fc = func(c)
        else:
            a, c, fc = c, d, fd
            d = a + ratio * (b - a)
            fd = func(d)
    return (a + b) / 2


def fit_power_gamma(u, v, lo=0.05, hi=10.0):
    """Least-squares gamma for v = u ** gamma (u, v in 0..1)."""
    u = np.clip(np.asarray(u, dtype=float), 0.0, None)
    v = np.asarray(v, dtype=float)
    return golden_section_min(lambda g: float(np.sum((u ** g - v) ** 2)), lo, hi)


def lag1_autocorrelation(values):
    x = np.asarray(values, dtype=float)
    if x.size < 3:
        return float("nan")
    x = x - x.mean()
    denom = float(np.sum(x * x))
    if denom == 0:
        return float("nan")
    return float(np.sum(x[:-1] * x[1:]) / denom)


def noise_stats(values):
    """Per-trial descriptive statistics, in sample order."""
    x = np.asarray(values, dtype=float)
    median = float(np.median(x))
    return {
        "count": int(x.size),
        "mean": float(x.mean()),
        "median": median,
        "sd": float(x.std(ddof=1)) if x.size > 1 else 0.0,
        "mad": float(np.median(np.abs(x - median))),
        "min": float(x.min()),
        "max": float(x.max()),
        "levels": int(np.unique(x).size),
        "lag1": lag1_autocorrelation(x),
    }


def normalise_z(reading, white, black):
    span = black - white
    if span == 0:
        return float("nan")
    return (reading - white) / span


def peak_and_fwhm(positions, z, level=0.5):
    """Return (peak_z, peak_position, fwhm) of a single-peaked profile.

    FWHM is the distance between the linearly interpolated crossings of
    `level` on either side of the peak, or None if either side never drops
    below the level (or the peak never reaches it).
    """
    order = np.argsort(positions)
    x = np.asarray(positions, dtype=float)[order]
    y = np.asarray(z, dtype=float)[order]
    peak_index = int(np.nanargmax(y))
    peak_z = float(y[peak_index])
    peak_position = float(x[peak_index])
    if peak_z < level:
        return peak_z, peak_position, None

    left = None
    for i in range(peak_index, 0, -1):
        if y[i - 1] < level <= y[i]:
            left = x[i - 1] + (level - y[i - 1]) * (x[i] - x[i - 1]) / (y[i] - y[i - 1])
            break
    right = None
    for i in range(peak_index, len(y) - 1):
        if y[i + 1] < level <= y[i]:
            right = x[i] + (y[i] - level) * (x[i + 1] - x[i]) / (y[i] - y[i + 1])
            break
    if left is None or right is None:
        return peak_z, peak_position, None
    return peak_z, peak_position, float(right - left)


# ------------------------------------------------------------- reporting ---

def _table(headers, rows):
    lines = ["| " + " | ".join(headers) + " |",
             "|" + "|".join("---" for _ in headers) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def _f(value, digits=2):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "–"
    return "{:.{}f}".format(value, digits)


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _write(out, name, lines):
    out.mkdir(parents=True, exist_ok=True)
    path = out / name
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _condition_means(medians, sensor, role):
    """{pct: (mean, sd, n)} over placements for one sensor and role."""
    result = {}
    for (s, pct, r), values in medians.items():
        if s == sensor and r == role:
            arr = np.asarray(values)
            result[pct] = (float(arr.mean()), float(arr.std(ddof=1)) if arr.size > 1 else 0.0,
                           int(arr.size))
    return dict(sorted(result.items()))


# ------------------------------------------------------------------- gain ---

def analyse_gain(rows, out, fit_range=None):
    medians = placement_medians(rows)
    plt = _plt()
    lines = ["# Gain (Exercise 4)", "",
             "Condition value = median of each placement batch; points are the mean over "
             "placements (± SD). Readings are baseline-subtracted per sensor "
             "(white baseline = mean placement median at 0 % blackness); raw values are kept "
             "in the CSV.", ""]
    if fit_range is None:
        lines.append("Fit range: all *fit* conditions (no range selected).")
    else:
        lines.append("Fit range: {:g}–{:g} % blackness, **chosen by the operator**, not by "
                     "maximising R².".format(*fit_range))
    lines.append("")

    results = {}
    shared_x, shared_y = [], []
    for sensor in SENSORS:
        fit = _condition_means(medians, sensor, "fit")
        hold = _condition_means(medians, sensor, "holdout")
        if 0.0 not in fit:
            raise SystemExit("gain needs a 0 % (white) fit condition for the baseline")
        baseline = fit[0.0][0]
        xs, ys = [], []
        for (s, pct, role), values in medians.items():
            if s != sensor or role != "fit":
                continue
            if fit_range and not fit_range[0] <= pct <= fit_range[1]:
                continue
            xs += [pct] * len(values)
            ys += [v - baseline for v in values]
        m, c, r2 = linear_fit(xs, ys)
        shared_x += xs
        shared_y += ys
        results[sensor] = dict(baseline=baseline, m=m, c=c, r2=r2, fit=fit, hold=hold)

    sm, sc, sr2 = linear_fit(shared_x, shared_y)

    table = []
    for sensor, res in results.items():
        hold = res["hold"]
        hx = list(hold)
        hy = [hold[p][0] - res["baseline"] for p in hx]
        res["hold_mae"] = mae(hy, [res["m"] * x + res["c"] for x in hx])
        res["hold_mae_shared"] = mae(hy, [sm * x + sc for x in hx])
        table.append(["DN{}".format(sensor), _f(res["baseline"], 1), _f(res["m"], 3),
                      _f(-res["m"] * 100 / 255, 3), _f(res["c"], 1), _f(res["r2"], 4),
                      _f(res["hold_mae"], 1), _f(res["hold_mae_shared"], 1)])
    lines += ["## Fitted gain per sensor", "",
              _table(["Sensor", "Baseline", "Gain m (/% black)", "Gain (/fill unit)",
                      "Intercept c", "R² (fit pts)", "Held-back MAE", "Held-back MAE (shared)"],
                     table), "",
              "Shared gain across all sensors: m = {:.3f} per % blackness, c = {:.1f}.".format(
                  sm, sc), "",
              "Gain units: {} per % blackness. Per fill unit = −m × 100/255 "
              "(fill 255 = white, 0 = black).".format(RAW_UNIT),
              "A slope against printed blackness is not sensitivity per unit of measured "
              "infrared reflectance.", ""]

    lines += ["## Held-back predictions", ""]
    hold_rows = []
    for sensor, res in results.items():
        for pct, (mean, sd, n) in res["hold"].items():
            measured = mean - res["baseline"]
            predicted = res["m"] * pct + res["c"]
            hold_rows.append(["DN{}".format(sensor), _f(pct, 0), _f(measured, 1),
                              _f(predicted, 1), _f(measured - predicted, 1), _f(sd, 1), str(n)])
    lines.append(_table(["Sensor", "% black", "Measured", "Predicted", "Residual",
                         "SD over placements", "Placements"], hold_rows) if hold_rows
                 else "_No held-back conditions in the data._")
    lines += ["", "Residuals combine model error, placement variation and noise; they are not "
              "automatically noise.", "",
              "Decide whether per-sensor gains predict held-back conditions enough better than "
              "the shared gain to justify five separate calibrations.", ""]

    fig, axes = plt.subplots(2, 5, figsize=(20, 7), sharex=True)
    for col, (sensor, res) in enumerate(results.items()):
        ax, rax = axes[0][col], axes[1][col]
        for data, marker, label in ((res["fit"], "o", "fit"), (res["hold"], "s", "held back")):
            if not data:
                continue
            x = list(data)
            y = [data[p][0] - res["baseline"] for p in x]
            e = [data[p][1] for p in x]
            ax.errorbar(x, y, yerr=e, fmt=marker, capsize=3, label=label)
            rax.plot(x, [yy - (res["m"] * xx + res["c"]) for xx, yy in zip(x, y)], marker,
                     label=label)
        line_x = np.linspace(0, 100, 50)
        ax.plot(line_x, res["m"] * line_x + res["c"], "-", alpha=0.6,
                label="fit m={:.2f}".format(res["m"]))
        if fit_range:
            ax.axvspan(*fit_range, alpha=0.08)
        ax.set_title("DN{}".format(sensor))
        rax.axhline(0, color="grey", lw=0.8)
        rax.set_xlabel("blackness (%)")
        if col == 0:
            ax.set_ylabel("reading − white baseline\n" + RAW_UNIT)
            rax.set_ylabel("residual")
        ax.legend(fontsize=8)
    fig.tight_layout()
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "gain_fit_and_residuals.png", dpi=120)
    plt.close(fig)
    lines += ["![fit and residuals](gain_fit_and_residuals.png)"]
    return _write(out, "summary.md", lines)


# ---------------------------------------------------------- non-linearity ---

def analyse_nonlinearity(rows, out):
    medians = placement_medians(rows)
    plt = _plt()
    models = ["linear v̂=u", "quadratic A+Bu+Cu²", "power u^γ (per sensor)",
              "power u^γ (shared γ)"]
    data = {}
    shared_u, shared_v = [], []
    for sensor in SENSORS:
        fit = _condition_means(medians, sensor, "fit")
        hold = _condition_means(medians, sensor, "holdout")
        if 0.0 not in fit or 100.0 not in fit:
            raise SystemExit("nonlinearity needs 0 % and 100 % fit conditions")
        b, s = fit[0.0][0], fit[100.0][0]
        def to_v(table):
            return ([p / 100 for p in table], [(table[p][0] - b) / (s - b) for p in table])
        fu, fv = to_v(fit)
        hu, hv = to_v(hold)
        shared_u += fu
        shared_v += fv
        data[sensor] = dict(b=b, s=s, fu=fu, fv=fv, hu=hu, hv=hv)
    gamma_shared = fit_power_gamma(shared_u, shared_v)

    rows_out = []
    for sensor, d in data.items():
        coeffs = np.polyfit(d["fu"], d["fv"], 2)
        gamma = fit_power_gamma(d["fu"], d["fv"])
        d["predict"] = {
            models[0]: lambda u: np.asarray(u, dtype=float),
            models[1]: lambda u, c=coeffs: np.polyval(c, u),
            models[2]: lambda u, g=gamma: np.asarray(u, dtype=float) ** g,
            models[3]: lambda u, g=gamma_shared: np.asarray(u, dtype=float) ** g,
        }
        d["gamma"], d["coeffs"] = gamma, coeffs
        for model in models:
            f = d["predict"][model]
            rows_out.append(["DN{}".format(sensor), model,
                             _f(mae(d["fv"], f(d["fu"])), 4),
                             _f(mae(d["hv"], f(d["hu"])), 4)])

    lines = ["# Non-linearity (Exercise 5)", "",
             "u = blackness / 100 (0 = white, 1 = black). For sensor i, "
             "v = (r − b_i) / (s_i − b_i) with b_i, s_i the mean placement medians at 0 % and "
             "100 %. Models are fitted on *fit* conditions only, frozen, then scored on "
             "*held-back* conditions. MAE is dimensionless.", "",
             _table(["Sensor", "b_i", "s_i", "γ (per sensor)", "Quadratic A, B, C"],
                    [["DN{}".format(s), _f(d["b"], 1), _f(d["s"], 1), _f(d["gamma"], 3),
                      ", ".join(_f(c, 3) for c in d["coeffs"][::-1])]
                     for s, d in data.items()]), "",
             "Shared γ across sensors: {:.3f}".format(gamma_shared), "",
             "## Model comparison (MAE)", "",
             _table(["Sensor", "Model", "Fit MAE", "Held-back MAE"], rows_out), "",
             "Prefer the simplest adequate model; keep a non-linear model only if its "
             "held-back improvement is repeatable. A low MAE uses condition means and does "
             "not show that individual readings are repeatable. Normalisation does not make "
             "these calibrated physical measurements.", ""]

    fig, axes = plt.subplots(2, 5, figsize=(20, 7), sharex=True)
    u_line = np.linspace(0, 1, 101)
    for col, (sensor, d) in enumerate(data.items()):
        ax, rax = axes[0][col], axes[1][col]
        ax.plot(d["fu"], d["fv"], "o", label="fit")
        if d["hu"]:
            ax.plot(d["hu"], d["hv"], "s", label="held back")
        for colour, model in zip(("C2", "C3", "C4", "C5"), models):
            ax.plot(u_line, d["predict"][model](u_line), lw=1, color=colour, label=model)
            res_fit = np.asarray(d["fv"]) - d["predict"][model](d["fu"])
            rax.plot(d["fu"], res_fit, "o-", lw=0.8, ms=3, color=colour)
            if d["hu"]:
                res_hold = np.asarray(d["hv"]) - d["predict"][model](d["hu"])
                rax.plot(d["hu"], res_hold, "s", ms=5, mfc="none", color=colour)
        ax.set_title("DN{}  γ={:.2f}".format(sensor, d["gamma"]))
        rax.axhline(0, color="grey", lw=0.8)
        rax.set_xlabel("u (blackness fraction)")
        if col == 0:
            ax.set_ylabel("v (normalised reading)")
            rax.set_ylabel("residual (o fit, □ held back)")
            ax.legend(fontsize=7)
    fig.tight_layout()
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "nonlinearity_models.png", dpi=120)
    plt.close(fig)
    lines.append("![models and residuals](nonlinearity_models.png)")
    return _write(out, "summary.md", lines)


# ------------------------------------------------------------------ noise ---

def analyse_noise(rows, out):
    plt = _plt()
    trials = defaultdict(list)
    for row in sorted(rows, key=lambda r: (r["source"], r["trial_num"], r["sample_num"])):
        key = (row["pct"], row["sensor_num"], row["source"], row["trial_num"], row["role"])
        trials[key].append(row["reading"])
    points = sorted({k[0] for k in trials})

    lines = ["# Measurement noise (Exercise 6)", "",
             "Each trial is one placement (robot lifted and replaced). Residuals are taken "
             "about each trial's own median, so placement-to-placement shifts are kept out of "
             "the noise model and reported separately. σ (Gaussian) and a (uniform half-width "
             "= max |residual|) are estimated from *fit* placements, then applied unchanged to "
             "the *held-back* placement. ±1σ ≈ 68 % and ±2σ ≈ 95 % are Gaussian reference "
             "values, not pass criteria.", ""]

    stat_rows, model_rows, spread_rows = [], [], []
    for pct in points:
        fig_ts, ax_ts = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
        fig_h, ax_h = plt.subplots(1, 5, figsize=(20, 4))
        for col, sensor in enumerate(SENSORS):
            keys = [k for k in trials if k[0] == pct and k[1] == sensor]
            fit_res, hold_res, medians, sds = [], [], [], []
            for key in keys:
                values = trials[key]
                st = noise_stats(values)
                medians.append(st["median"])
                sds.append(st["sd"])
                stat_rows.append(["{:g}".format(pct), "DN{}".format(sensor),
                                  "{}#{}".format(key[2], key[3]), key[4], str(st["count"]),
                                  _f(st["mean"], 1), _f(st["median"], 1), _f(st["sd"], 2),
                                  _f(st["mad"], 1), _f(st["min"], 0), _f(st["max"], 0),
                                  str(st["levels"]), _f(st["lag1"], 2)])
                residuals = np.asarray(values) - st["median"]
                (hold_res if key[4] == "holdout" else fit_res).append(residuals)
                ax_ts[col].plot(values, lw=0.6, label="trial {} ({})".format(key[3], key[4]))
            ax_ts[col].set_ylabel("DN{}\n{}".format(sensor, "raw (µs)"))
            ax_ts[col].legend(fontsize=7, loc="upper right")

            within = float(np.mean(sds)) if sds else float("nan")
            between = float(np.std(medians, ddof=1)) if len(medians) > 1 else float("nan")
            spread_rows.append(["{:g}".format(pct), "DN{}".format(sensor), _f(within, 2),
                                _f(between, 2), str(len(medians))])

            if fit_res:
                pooled = np.concatenate(fit_res)
                sigma = float(pooled.std(ddof=1))
                a = float(np.max(np.abs(pooled)))
                ax_h[col].hist(pooled, bins=40, density=True, alpha=0.5, label="fit residuals")
                grid = np.linspace(pooled.min() - sigma, pooled.max() + sigma, 200)
                if sigma > 0:
                    ax_h[col].plot(grid, np.exp(-grid ** 2 / (2 * sigma ** 2))
                                   / (sigma * math.sqrt(2 * math.pi)), label="Gaussian σ")
                if a > 0:
                    ax_h[col].plot([-a, -a, a, a], [0, 1 / (2 * a), 1 / (2 * a), 0],
                                   label="uniform ±a")
                if hold_res:
                    held = np.concatenate(hold_res)
                    ax_h[col].hist(held, bins=40, density=True, histtype="step",
                                   label="held-back")
                    within1 = float(np.mean(np.abs(held) <= sigma)) if sigma > 0 else float("nan")
                    within2 = (float(np.mean(np.abs(held) <= 2 * sigma)) if sigma > 0
                               else float("nan"))
                    ll_g = (float(np.mean(-0.5 * (held / sigma) ** 2
                                          - math.log(sigma * math.sqrt(2 * math.pi))))
                            if sigma > 0 else float("nan"))
                    outside = int(np.sum(np.abs(held) > a))
                    ll_u = -math.log(2 * a) if a > 0 and outside == 0 else float("-inf")
                    model_rows.append(["{:g}".format(pct), "DN{}".format(sensor),
                                       _f(sigma, 2), _f(a, 1), _f(100 * within1, 1),
                                       _f(100 * within2, 1), _f(ll_g, 3),
                                       _f(ll_u, 3) if math.isfinite(ll_u) else "−∞",
                                       "{}/{}".format(outside, held.size)])
                ax_h[col].set_title("DN{} @ {:g}%".format(sensor, pct))
                ax_h[col].set_xlabel("residual about trial median (µs)")
                ax_h[col].legend(fontsize=7)
        ax_ts[-1].set_xlabel("sample number (~10 ms apart)")
        fig_ts.suptitle("Time series at {:g}% blackness (inspect before histograms)".format(pct))
        fig_ts.tight_layout()
        fig_h.tight_layout()
        out.mkdir(parents=True, exist_ok=True)
        fig_ts.savefig(out / "noise_timeseries_{:g}pct.png".format(pct), dpi=110)
        fig_h.savefig(out / "noise_histograms_{:g}pct.png".format(pct), dpi=110)
        plt.close(fig_ts)
        plt.close(fig_h)

    lines += ["## Per-trial statistics", "",
              _table(["% black", "Sensor", "Trial", "Role", "n", "Mean", "Median", "SD", "MAD",
                      "Min", "Max", "Levels", "Lag-1 r"], stat_rows), "",
              "Lag-1 r well above 0 indicates serially correlated samples (drift or slow "
              "variation), which makes the effective number of independent samples smaller "
              "than n.", "",
              "## Within- vs between-placement variation", "",
              _table(["% black", "Sensor", "Mean within-trial SD", "SD of trial medians",
                      "Trials"], spread_rows), "",
              "## Noise models applied to the held-back placement", "",
              (_table(["% black", "Sensor", "σ (fit)", "a (fit)", "% within ±1σ",
                       "% within ±2σ", "Mean log-lik Gaussian", "Mean log-lik uniform",
                       "Outside ±a"], model_rows) if model_rows
               else "_No held-back placement in the data._"), "",
              "Readings are integers. When σ spans only a few counts (see the Levels column), "
              "coverage fractions are distorted by quantisation: with σ = 4.85, ±1σ admits only "
              "|e| ≤ 4. Judge the models with that in mind.", "",
              "Use a noise model in later work only over the operating conditions where it "
              "remained adequate here.", ""]
    for pct in points:
        lines += ["![time series {0:g}%](noise_timeseries_{0:g}pct.png)".format(pct),
                  "![histograms {0:g}%](noise_histograms_{0:g}pct.png)".format(pct)]
    return _write(out, "summary.md", lines)


# -------------------------------------------------------------- footprint ---

def analyse_footprint(rows, out, sensor):
    plt = _plt()
    sel = [r for r in rows if r["sensor_num"] == sensor]
    white = [r["reading"] for r in sel if r["test_name"] == "footprintWhiteReference"]
    black = [r["reading"] for r in sel if r["test_name"] == "footprintBlackReference"]
    if not white or not black:
        raise SystemExit("footprint data needs white and black reference batches")
    W, B = float(np.median(white)), float(np.median(black))

    batches = defaultdict(list)
    for r in sel:
        if r["test_name"] == "footprintTest":
            key = (r["width_mm"], r["source"], r["trial_num"], r["direction"], r["position_mm"])
            batches[key].append(r["reading"])
    traverses = defaultdict(list)
    for (width, source, trial, direction, position), values in batches.items():
        median = float(np.median(values))
        sd = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
        traverses[(width, source, trial, direction)].append(
            (position, normalise_z(median, W, B), sd / (B - W)))

    widths = sorted({k[0] for k in traverses}, reverse=True)
    lines = ["# Spatial footprint (Exercise 7)", "",
             "Sensor DN{}. z = (R − W) / (B − W), R = batch median at a position, "
             "W = {:.1f} and B = {:.1f} the white/black reference medians held fixed for "
             "every strip. Error bars are sample SD / (B − W), **not** uncertainty of the "
             "median.".format(sensor, W, B), ""]
    table, per_width = [], defaultdict(list)
    for key in sorted(traverses, key=lambda k: (-k[0], k[1], k[2])):
        width, source, trial, direction = key
        points = sorted(traverses[key])
        positions = [p[0] for p in points]
        z = [p[1] for p in points]
        peak, peak_pos, fwhm = peak_and_fwhm(positions, z)
        per_width[width].append((peak, peak_pos, fwhm))
        table.append([_f(width, 0), "{}#{}".format(source, trial), direction or "–",
                      _f(peak, 3), _f(peak_pos, 1), _f(fwhm, 2),
                      _f(fwhm - width, 2) if fwhm is not None else "–"])
    lines += [_table(["Strip (mm)", "Traverse", "Direction", "Peak z", "Peak at (mm)",
                      "FWHM (mm)", "FWHM − width"], table), ""]

    comparison = []
    for width in widths:
        results = per_width[width]
        if len(results) > 1:
            peaks = [r[0] for r in results]
            positions = [r[1] for r in results]
            fwhms = [r[2] for r in results if r[2] is not None]
            comparison.append([_f(width, 0), str(len(results)),
                               _f(max(peaks) - min(peaks), 3),
                               _f(max(positions) - min(positions), 1),
                               _f(max(fwhms) - min(fwhms), 2) if len(fwhms) > 1 else "–"])
    if comparison:
        lines += ["## Repeatability (repeats / directions)", "",
                  _table(["Strip (mm)", "Traverses", "Peak-z range", "Peak-position range (mm)",
                          "FWHM range (mm)"], comparison), ""]

    resolved = [w for w in widths if all(r[0] >= 0.5 for r in per_width[w])]
    lines += ["## Interpretation", ""]
    if resolved:
        lines.append("Narrowest strip whose peak reached z ≥ 0.5 on every traverse: "
                     "**{:g} mm**. Narrower strips are only partly seen, i.e. they look like "
                     "a broader grey target (perceptual aliasing).".format(min(resolved)))
    wide = [r[2] - w for w in widths for r in per_width[w] if r[2] is not None and r[0] >= 0.95]
    if wide:
        lines.append("For strips that saturate (peak ≥ 0.95), FWHM exceeds the strip width by "
                     "{:.2f} mm on average — an estimate of the sensor's effective spatial "
                     "spread.".format(float(np.mean(wide))))
    lines += ["", "Limitation: positions are set by hand against a ruler, so position error "
              "(and any tilt of the robot relative to the strip) adds directly to the "
              "measured profile width.", ""]

    cols = min(3, len(widths))
    rows_n = math.ceil(len(widths) / cols)
    fig, axes = plt.subplots(rows_n, cols, figsize=(6 * cols, 4 * rows_n), squeeze=False)
    for index, width in enumerate(widths):
        ax = axes[index // cols][index % cols]
        for key in sorted(k for k in traverses if k[0] == width):
            points = sorted(traverses[key])
            ax.errorbar([p[0] for p in points], [p[1] for p in points],
                        yerr=[p[2] for p in points], fmt="o-", ms=3, capsize=2,
                        label="trial {} {}".format(key[2], key[3]))
        ax.axhline(0.5, color="grey", ls="--", lw=0.8)
        ax.axvspan(-width / 2, width / 2, color="k", alpha=0.08)
        ax.set_title("{:g} mm strip".format(width))
        ax.set_xlabel("position from strip centre (mm)")
        ax.set_ylabel("z (0 white, 1 black)")
        ax.legend(fontsize=7)
    for index in range(len(widths), rows_n * cols):
        axes[index // cols][index % cols].axis("off")
    fig.tight_layout()
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "footprint_profiles.png", dpi=120)
    plt.close(fig)
    lines.append("![footprint profiles](footprint_profiles.png)")
    return _write(out, "summary.md", lines)


# ------------------------------------------------------------------- main ---

def main():
    parser = argparse.ArgumentParser(description="Analyse reflectance experiment CSVs.")
    parser.add_argument("analysis", choices=["gain", "nonlinearity", "noise", "footprint"])
    parser.add_argument("csv", nargs="+", help="experiment CSV file(s) or glob patterns")
    parser.add_argument("--out", type=Path, help="output directory (default reports/<analysis>)")
    parser.add_argument("--fit-range", nargs=2, type=float, metavar=("LO", "HI"),
                        help="gain: blackness %% range treated as linear")
    parser.add_argument("--sensor", type=int, default=3, choices=SENSORS,
                        help="footprint: sensor to analyse")
    args = parser.parse_args()

    rows = load_rows(args.csv)
    out = args.out or Path("reports") / args.analysis
    if args.analysis == "gain":
        path = analyse_gain(rows, out, tuple(args.fit_range) if args.fit_range else None)
    elif args.analysis == "nonlinearity":
        path = analyse_nonlinearity(rows, out)
    elif args.analysis == "noise":
        path = analyse_noise(rows, out)
    else:
        path = analyse_footprint(rows, out, args.sensor)
    print("Wrote {}".format(path))


if __name__ == "__main__":
    main()
