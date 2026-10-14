"""Speed vs PWM (gain test) analysis.

usage: python3 bench/1.5_gain/analyse.py [--fit-max PWM] logs/bench_gain_<time>.csv [...]

Writes holds.csv, levels.csv, summary.txt (and plot.png) to
bench/1.5_gain/<csv name>/.

Speed at each level is the counts over the hold phase divided by its time
(enc_us). Holds with no counts are stalls and are left out of the fits.

Fits, on the speed of every hold:
  linear     speed = G * pwm + c, which gives the firmware feedforward
             pwm = offset + speed / slope with slope = G, offset = -c / G
  quadratic  for comparison
Every 4th level (starting with the 3rd) is held back from the fit used for
the held-back error, so the two fits are compared on levels they didn't see.
The suggested config uses the linear fit on all levels up to --fit-max
(default: all), so a curve near the top doesn't bend the line.
"""
import csv
import json
import statistics
import sys
from pathlib import Path

import numpy as np

COUNTS_PER_REV = 358.3
DEFAULT_RADIUS_MM = 16.0
SLICE_MS = 100
OUT_DIR = Path(__file__).resolve().parent


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    meta_path = Path(path).with_suffix(".json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    return rows, meta


def mm_per_count(meta, wheel):
    side = "left" if wheel == "L" else "right"
    radius = meta.get("config", {}).get(f"{side}_wheel_radius_mm", DEFAULT_RADIUS_MM)
    return 2 * np.pi * radius / COUNTS_PER_REV


def unwrap_us(values):
    # micros() wraps every ~71 min
    t = np.array(values, dtype=np.int64)
    steps = np.diff(t) % (1 << 32)
    return np.concatenate([[0], np.cumsum(steps)]) + t[0]


def holds(rows, meta):
    groups = {}
    order = []
    for r in rows:
        if r["phase"] != "hold" or r["enc_ok"] != "1":
            continue
        key = (int(r["trial"]), r["leg"], int(r["pwm"]))
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(r)

    out = []
    for trial, leg, pwm in order:
        group = groups[(trial, leg, pwm)]
        if len(group) < 3:
            continue
        wheel = group[0]["wheel"]
        mm = mm_per_count(meta, wheel)
        sign = 1 if pwm >= 0 else -1
        t = (unwrap_us([r["enc_us"] for r in group]) - int(group[0]["enc_us"])) / 1000
        c = sign * np.array([int(r["enc_l" if wheel == "L" else "enc_r"]) for r in group])
        speed = (c[-1] - c[0]) / (t[-1] / 1000) * mm if t[-1] > 0 else 0.0

        slices = []
        for lo in np.arange(0, t[-1] - SLICE_MS + 1e-9, SLICE_MS):
            inside = (t >= lo) & (t <= lo + SLICE_MS)
            if inside.sum() >= 2:
                ts, cs = t[inside], c[inside]
                slices.append((cs[-1] - cs[0]) / ((ts[-1] - ts[0]) / 1000) * mm)
        out.append({
            "trial": trial, "leg": leg, "wheel": wheel, "pwm": pwm,
            "speed_mm_s": round(float(speed), 2),
            "slice_sd_mm_s": round(float(np.std(slices)), 2) if len(slices) > 1 else None,
            "stalled": int(c[-1] == c[0]),
            "hold_ms": round(float(t[-1]), 1),
        })
    return out


def levels(hold_rows):
    out = []
    for pwm in sorted({h["pwm"] for h in hold_rows}, key=abs):
        group = [h for h in hold_rows if h["pwm"] == pwm and not h["stalled"]]
        down = [h["speed_mm_s"] for h in group if h["leg"] == "down"]
        up = [h["speed_mm_s"] for h in group if h["leg"] == "up"]
        speeds = down + up
        ripple = [h["slice_sd_mm_s"] for h in group if h["slice_sd_mm_s"] is not None]
        out.append({
            "pwm": pwm, "holds": len(speeds),
            "stalls": sum(1 for h in hold_rows if h["pwm"] == pwm and h["stalled"]),
            "speed_mm_s": round(statistics.mean(speeds), 2) if speeds else None,
            "sd_mm_s": round(statistics.stdev(speeds), 2) if len(speeds) > 1 else None,
            "down_mm_s": round(statistics.mean(down), 2) if down else None,
            "up_mm_s": round(statistics.mean(up), 2) if up else None,
            "ripple_sd_mm_s": round(statistics.median(ripple), 2) if ripple else None,
        })
    return out


def fits(hold_rows, fit_max=None):
    good = [h for h in hold_rows if not h["stalled"] and (fit_max is None or abs(h["pwm"]) <= fit_max)]
    pwm = np.array([abs(h["pwm"]) for h in good], dtype=float)
    speed = np.array([h["speed_mm_s"] for h in good])
    mags = sorted(set(pwm))
    held_back = set(mags[2::4])
    train = np.array([p not in held_back for p in pwm])

    result = {"held_back_pwm": sorted(int(p) for p in held_back)}
    for name, degree in (("linear", 1), ("quadratic", 2)):
        if train.sum() > degree and (~train).any():
            coef = np.polyfit(pwm[train], speed[train], degree)
            result[f"{name}_held_back_mae"] = float(np.mean(np.abs(np.polyval(coef, pwm[~train]) - speed[~train])))
    g, c = np.polyfit(pwm, speed, 1)
    residual = speed - (g * pwm + c)
    result.update(gain=float(g), intercept=float(c), offset_pwm=float(-c / g),
                  rms_residual=float(np.sqrt(np.mean(residual ** 2))))
    return result


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def plot(path, hold_rows, fit, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        return f"plot skipped ({e.__class__.__name__}: {e})"
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for leg, colour in (("down", "tab:blue"), ("up", "tab:orange")):
        pts = [h for h in hold_rows if h["leg"] == leg and not h["stalled"]]
        ax.plot([abs(h["pwm"]) for h in pts], [h["speed_mm_s"] for h in pts], "o", color=colour,
                alpha=0.6, ms=4, label=leg)
    x = np.linspace(min(abs(h["pwm"]) for h in hold_rows), max(abs(h["pwm"]) for h in hold_rows), 50)
    ax.plot(x, fit["gain"] * x + fit["intercept"], "k-", lw=1, label="linear fit")
    ax.set_xlabel("PWM magnitude")
    ax.set_ylabel("speed (mm/s)")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return f"plot: {path}"


def fmt(x, spec=".1f"):
    return "-" if x is None else format(x, spec)


def runs(paths):
    """(path, suffix, rows, meta) per wheel. An S (spin) run drives both wheels,
    the right one opposite, so it's split into _L and _R with the right wheel's
    PWM sign flipped, and each half is analysed like a single-wheel run."""
    for path in paths:
        rows, meta = load(path)
        if not rows or rows[0]["wheel"] != "S":
            yield path, "", rows, meta
            continue
        for side, sign in (("L", 1), ("R", -1)):
            yield path, f"_{side}", [dict(r, wheel=side, pwm=str(sign * int(r["pwm"]))) for r in rows], meta


def wheel_label(rows, params):
    wheel = rows[0]["wheel"] if rows else params.get("wheel", "?")
    return f"{wheel} (spin)" if params.get("wheel") == "S" else wheel


def main():
    args = sys.argv[1:]
    fit_max = None
    if "--fit-max" in args:
        i = args.index("--fit-max")
        fit_max = int(args[i + 1])
        del args[i:i + 2]
    for path, suffix, rows, meta in runs(args):
        hold_rows = holds(rows, meta)
        level_rows = levels(hold_rows)
        out = OUT_DIR / (Path(path).stem + suffix)
        out.mkdir(parents=True, exist_ok=True)
        write_csv(out / "holds.csv", hold_rows)
        write_csv(out / "levels.csv", level_rows)

        params = meta.get("params", {})
        wheel = rows[0]["wheel"] if rows else "?"
        title = f"gain {wheel_label(rows, params)} dir {params.get('dir', '?')} ({Path(path).name})"
        lines = [title, f"end: {meta.get('end', 'unknown')}",
                 f"settle {params.get('settle_ms', '?')} ms, hold {params.get('hold_ms', '?')} ms; "
                 f"holds: {len(hold_rows)}, stalled: {sum(h['stalled'] for h in hold_rows)}",
                 "",
                 " PWM  holds  speed mm/s   sd   down     up   ripple sd"]
        for l in level_rows:
            lines.append(f"{l['pwm']:+4d}  {l['holds']:5d}  {fmt(l['speed_mm_s']):>10}  {fmt(l['sd_mm_s']):>4}"
                         f"  {fmt(l['down_mm_s']):>5}  {fmt(l['up_mm_s']):>5}  {fmt(l['ripple_sd_mm_s']):>9}"
                         + (f"   {l['stalls']} stalled" if l["stalls"] else ""))

        good = [l for l in level_rows if l["speed_mm_s"] is not None]
        if len(good) >= 3:
            fit = fits(hold_rows, fit_max)
            lines += ["", f"linear fit{f' up to {fit_max} PWM' if fit_max else ''}: speed = {fit['gain']:.3f} * pwm {fit['intercept']:+.1f} mm/s, "
                          f"rms residual {fit['rms_residual']:.1f} mm/s"]
            if "linear_held_back_mae" in fit:
                lines.append(f"held back PWM {fit['held_back_pwm']}: mean error linear "
                             f"{fit['linear_held_back_mae']:.1f} mm/s, quadratic "
                             f"{fit.get('quadratic_held_back_mae', float('nan')):.1f} mm/s")

            mags = [abs(l["pwm"]) for l in good]
            speeds = [l["speed_mm_s"] for l in good]
            slopes = np.diff(speeds) / np.diff(mags)
            if len(slopes) >= 3:
                lines.append(f"slope between levels: median {np.median(slopes):.2f}, top step "
                             f"{slopes[-1]:.2f} mm/s per PWM ({slopes[-1] / np.median(slopes):.0%} of median)")
            both = [l for l in good if l["down_mm_s"] is not None and l["up_mm_s"] is not None]
            if both:
                worst = max(both, key=lambda l: abs(l["down_mm_s"] - l["up_mm_s"]))
                lines.append(f"down vs up: largest difference {worst['down_mm_s'] - worst['up_mm_s']:+.1f} mm/s "
                             f"at {worst['pwm']:+d}")
            top = max(abs(h["pwm"]) for h in hold_rows)
            per_trial = [statistics.mean(h["speed_mm_s"] for h in hold_rows if abs(h["pwm"]) == top
                                         and h["trial"] == t and not h["stalled"])
                         for t in sorted({h["trial"] for h in hold_rows})
                         if any(abs(h["pwm"]) == top and h["trial"] == t and not h["stalled"] for h in hold_rows)]
            lines.append(f"speed at {top} PWM by trial: " + ", ".join(f"{s:.1f}" for s in per_trial))

            side = "left" if wheel == "L" else "right"
            lines += ["", "suggested config:",
                      f'  "{side}_ff_offset_pwm": {fit["offset_pwm"]:.1f},',
                      f'  "{side}_ff_mm_s_per_pwm": {fit["gain"]:.2f}']
            lines.append(plot(out / "plot.png", hold_rows, fit, title))
        text = "\n".join(lines)
        (out / "summary.txt").write_text(text + "\n")
        print(text)
        print(f"-> {out}\n")


if __name__ == "__main__":
    main()
