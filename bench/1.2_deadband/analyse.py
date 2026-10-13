"""Deadband test analysis.

usage: python3 bench/1.2_deadband/analyse.py logs/bench_deadband_<time>.csv [...]

Writes trials.csv, levels.csv, summary.txt (and plot.png) to
bench/1.2_deadband/<csv name>/.

Each run is classified using the tested wheel's counts:
  sustained - at least one count in every 100 ms slice of the last 500 ms
  twitch    - moved >= 3 counts but not sustained
  none      - otherwise
"""
import csv
import json
import statistics
import sys
from pathlib import Path

import numpy as np

SUSTAIN_MS = 500
SLICE_MS = 100
TWITCH_COUNTS = 3
MM_PER_COUNT = 2 * np.pi * 16.0 / 358.3
OUT_DIR = Path(__file__).resolve().parent


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    meta_path = Path(path).with_suffix(".json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    return rows, meta


def unwrap_us(values):
    # micros() wraps every ~71 min
    t = np.array(values, dtype=np.int64)
    steps = np.diff(t) % (1 << 32)
    return np.concatenate([[0], np.cumsum(steps)]) + t[0]


def trials(rows):
    by_trial = {}
    for r in rows:
        by_trial.setdefault(int(r["trial"]), []).append(r)
    out = []
    for trial, group in sorted(by_trial.items()):
        run = [r for r in group if r["phase"] == "run" and r["enc_ok"] == "1"]
        if len(run) < 2:
            continue
        wheel = run[0]["wheel"]
        pwm = int(run[0]["pwm"])
        sign = 1 if pwm >= 0 else -1
        t_ms = (unwrap_us([r["enc_us"] for r in run]) - int(run[0]["enc_us"])) / 1000
        counts = sign * (np.array([int(r["enc_l" if wheel == "L" else "enc_r"]) for r in run]) -
                         int(run[0]["enc_l" if wheel == "L" else "enc_r"]))
        moved = counts[-1]
        first = np.flatnonzero(counts > 0)
        delay = t_ms[first[0]] if len(first) else None

        end = t_ms[-1]
        window = t_ms >= end - SUSTAIN_MS
        slices = []
        for k in range(SUSTAIN_MS // SLICE_MS):
            lo, hi = end - SUSTAIN_MS + k * SLICE_MS, end - SUSTAIN_MS + (k + 1) * SLICE_MS
            inside = (t_ms >= lo) & (t_ms <= hi)
            slices.append(counts[inside].max() - counts[inside].min() if inside.any() else 0)
        sustained = all(s >= 1 for s in slices) and moved >= TWITCH_COUNTS
        w_counts = counts[window]
        w_t = t_ms[window]
        speed = ((w_counts[-1] - w_counts[0]) / ((w_t[-1] - w_t[0]) / 1000) * MM_PER_COUNT
                 if len(w_t) > 1 and w_t[-1] > w_t[0] else 0.0)
        out.append({
            "trial": trial, "repeat": int(run[0]["repeat"]), "wheel": wheel, "pwm": pwm,
            "rest_ok": int(run[-1]["rest_ok"]), "outcome": "sustained" if sustained else
            "twitch" if moved >= TWITCH_COUNTS else "none",
            "moved_counts": int(moved), "first_count_ms": None if delay is None else round(float(delay), 1),
            "final_speed_mm_s": round(float(speed), 2), "run_ms": round(float(end), 1),
        })
    return out


def levels(trial_rows):
    out = []
    for pwm in sorted({t["pwm"] for t in trial_rows}, key=abs):
        group = [t for t in trial_rows if t["pwm"] == pwm and t["rest_ok"]]
        n = len(group)
        sus = [t for t in group if t["outcome"] == "sustained"]
        delays = [t["first_count_ms"] for t in sus if t["first_count_ms"] is not None]
        out.append({
            "pwm": pwm, "trials": n,
            "sustained": len(sus), "twitch": sum(t["outcome"] == "twitch" for t in group),
            "none": sum(t["outcome"] == "none" for t in group),
            "sustained_rate": round(len(sus) / n, 3) if n else None,
            "speed_median_mm_s": round(statistics.median(t["final_speed_mm_s"] for t in sus), 1) if sus else None,
            "first_count_median_ms": round(statistics.median(delays), 1) if delays else None,
            "first_count_max_ms": round(max(delays), 1) if delays else None,
            "excluded_not_still": sum(1 for t in trial_rows if t["pwm"] == pwm and not t["rest_ok"]),
        })
    return out


def threshold(level_rows):
    rated = [l for l in level_rows if l["trials"]]
    mags = [abs(l["pwm"]) for l in rated]
    rates = [l["sustained_rate"] for l in rated]
    never = None
    for m, r in zip(mags, rates):
        if r == 0:
            never = m
        else:
            break
    always = None
    for m, r in zip(reversed(mags), reversed(rates)):
        if r == 1:
            always = m
        else:
            break
    p50 = None
    for (m0, r0), (m1, r1) in zip(zip(mags, rates), zip(mags[1:], rates[1:])):
        if r0 < 0.5 <= r1:
            p50 = m0 + (0.5 - r0) / (r1 - r0) * (m1 - m0)
            break
    return never, always, p50


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def plot(path, level_rows, trial_rows, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        return f"plot skipped ({e.__class__.__name__}: {e})"
    fig, (a, b) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    mags = [abs(l["pwm"]) for l in level_rows if l["trials"]]
    a.plot(mags, [l["sustained_rate"] for l in level_rows if l["trials"]], "o-")
    a.set_ylabel("sustained starts (share)")
    a.set_title(title)
    a.grid(alpha=0.3)
    for t in trial_rows:
        colour = {"sustained": "tab:green", "twitch": "tab:orange", "none": "tab:grey"}[t["outcome"]]
        b.plot(abs(t["pwm"]), t["final_speed_mm_s"], "o", color=colour, alpha=0.6, ms=4)
    b.set_xlabel("PWM magnitude")
    b.set_ylabel("speed, last 500 ms (mm/s)")
    b.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return f"plot: {path}"


def main():
    for path in sys.argv[1:]:
        rows, meta = load(path)
        trial_rows = trials(rows)
        level_rows = levels(trial_rows)
        never, always, p50 = threshold(level_rows)
        out = OUT_DIR / Path(path).stem
        out.mkdir(parents=True, exist_ok=True)
        write_csv(out / "trials.csv", trial_rows)
        write_csv(out / "levels.csv", level_rows)

        params = meta.get("params", {})
        title = (f"deadband {params.get('wheel', '?')} dir {params.get('dir', '?')} "
                 f"({Path(path).name})")
        lines = [title, f"end: {meta.get('end', 'unknown')}",
                 f"rule: sustained = >= 1 count in every {SLICE_MS} ms of the last {SUSTAIN_MS} ms; "
                 f"twitch = moved >= {TWITCH_COUNTS} counts otherwise",
                 f"trials with a run: {len(trial_rows)}; not started from rest (excluded): "
                 f"{sum(1 for t in trial_rows if not t['rest_ok'])}",
                 "",
                 " PWM  trials  sustained  twitch  none   rate   speed mm/s   first count ms (median/max)"]
        for l in level_rows:
            rate = "-" if l["sustained_rate"] is None else f"{l['sustained_rate']:.0%}"
            lines.append(f"{l['pwm']:+4d}  {l['trials']:6d}  {l['sustained']:9d}  {l['twitch']:6d}  {l['none']:4d}  "
                         f"{rate:>5}   {l['speed_median_mm_s'] if l['speed_median_mm_s'] is not None else '-':>10}   "
                         f"{l['first_count_median_ms'] if l['first_count_median_ms'] is not None else '-'} / "
                         f"{l['first_count_max_ms'] if l['first_count_max_ms'] is not None else '-'}")
        lines += ["",
                  f"never started at or below: {never if never is not None else '- (moved at the lowest level)'}",
                  f"always started at or above: {always if always is not None else '- (not reached)'}",
                  f"50 % starting point (interpolated): {p50:.1f}" if p50 is not None else "50 % point: -",
                  plot(out / "plot.png", level_rows, trial_rows, title)]
        (out / "summary.txt").write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        print(f"-> {out}\n")


if __name__ == "__main__":
    main()
