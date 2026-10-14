"""Keep-turning test analysis.

usage: python3 bench/1.3_keepturn/analyse.py logs/bench_keepturn_<time>.csv [...]

Writes trials.csv and summary.txt (and plot.png) to bench/1.3_keepturn/<csv name>/.

For each trial:
  spin ok   - the wheel was moving at the end of the spin (a count in each
              of the last five 100 ms slices); trials that weren't are skipped
  stop pwm  - the PWM applied when the last count of the ramp came in, i.e.
              the lowest PWM the wheel was still turning at
"""
import csv
import json
import statistics
import sys
from pathlib import Path

import numpy as np

SLICE_MS = 100
SPIN_CHECK_MS = 500
COUNTS_PER_REV = 358.3
DEFAULT_RADIUS_MM = 16.0
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


def mm_per_count(meta, wheel):
    # radius from the robot's config saved with the test, if the logger recorded it
    side = "left" if wheel == "L" else "right"
    radius = meta.get("config", {}).get(f"{side}_wheel_radius_mm", DEFAULT_RADIUS_MM)
    return 2 * np.pi * radius / COUNTS_PER_REV


def trials(rows, meta):
    by_trial = {}
    for r in rows:
        if r["enc_ok"] == "1":
            by_trial.setdefault(int(r["trial"]), []).append(r)
    out = []
    for trial, group in sorted(by_trial.items()):
        spin = [r for r in group if r["phase"] == "spin"]
        ramp = [r for r in group if r["phase"] == "ramp"]
        if len(spin) < 2 or len(ramp) < 2:
            continue
        wheel = group[0]["wheel"]
        mm = mm_per_count(meta, wheel)
        key = "enc_l" if wheel == "L" else "enc_r"
        sign = 1 if int(spin[0]["pwm"]) >= 0 else -1

        t = (unwrap_us([r["enc_us"] for r in spin]) - int(spin[0]["enc_us"])) / 1000
        c = sign * np.array([int(r[key]) for r in spin])
        end = t[-1]
        spin_ok = True
        for k in range(SPIN_CHECK_MS // SLICE_MS):
            inside = (t >= end - SPIN_CHECK_MS + k * SLICE_MS) & (t <= end - SPIN_CHECK_MS + (k + 1) * SLICE_MS)
            if not inside.any() or c[inside].max() == c[inside].min():
                spin_ok = False
        last = t >= end - SPIN_CHECK_MS
        spin_speed = (c[last][-1] - c[last][0]) / ((t[last][-1] - t[last][0]) / 1000) * mm

        counts = sign * np.array([int(r[key]) for r in ramp])
        pwms = np.array([abs(int(r["pwm"])) for r in ramp])
        changed = np.flatnonzero(np.diff(counts) != 0) + 1
        stop_pwm = int(pwms[changed[-1]]) if len(changed) else None

        out.append({
            "trial": trial, "wheel": wheel, "dir": sign,
            "spin_pwm": abs(int(spin[0]["pwm"])), "spin_ok": int(spin_ok),
            "spin_speed_mm_s": round(float(spin_speed), 1),
            "stop_pwm": stop_pwm, "ramp_counts": int(counts[-1] - counts[0]),
        })
    return out


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def plot(path, rows, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        return f"plot skipped ({e.__class__.__name__}: {e})"
    good = [r for r in rows if r["spin_ok"] and r["stop_pwm"] is not None]
    fig, ax = plt.subplots(figsize=(6, 3.5))
    ax.plot([r["trial"] for r in good], [r["stop_pwm"] for r in good], "o")
    ax.set_xlabel("trial")
    ax.set_ylabel("stop PWM")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return f"plot: {path}"


def main():
    for path in sys.argv[1:]:
        rows, meta = load(path)
        trial_rows = trials(rows, meta)
        out = OUT_DIR / Path(path).stem
        out.mkdir(parents=True, exist_ok=True)
        write_csv(out / "trials.csv", trial_rows)

        params = meta.get("params", {})
        title = f"keepturn {params.get('wheel', '?')} dir {params.get('dir', '?')} ({Path(path).name})"
        good = [r for r in trial_rows if r["spin_ok"] and r["stop_pwm"] is not None]
        lines = [title, f"end: {meta.get('end', 'unknown')}",
                 f"ramp: 1 PWM every {params.get('step_ms', '?')} ms from {params.get('start', '?')}",
                 f"trials: {len(trial_rows)}, used: {len(good)} (skipped if not moving at the end of the spin)",
                 "",
                 "trial  spin ok  spin speed mm/s  stop pwm"]
        for r in trial_rows:
            lines.append(f"{r['trial']:5d}  {'yes' if r['spin_ok'] else 'NO':>7}  {r['spin_speed_mm_s']:15.1f}  "
                         f"{r['stop_pwm'] if r['stop_pwm'] is not None else '-':>8}")
        if good:
            stops = [r["stop_pwm"] for r in good]
            lines += ["", f"stop pwm: median {statistics.median(stops)}, min {min(stops)}, max {max(stops)}"]
        lines.append(plot(out / "plot.png", trial_rows, title))
        text = "\n".join(lines)
        (out / "summary.txt").write_text(text + "\n")
        print(text)
        print(f"-> {out}\n")


if __name__ == "__main__":
    main()
