"""Line follower runs on the square course, 2026-10-29: old vs offset follower.

    python3 bench/line_position/follower_20261029/analyse.py

Reads raw/runs.csv (logger records of each run, labelled in the run column,
with 0.5 s either side) and the "#L" replies in raw/line_replies_*.txt.
Writes summary.csv and paths.png next to this script and prints both tables.

Per run, over the records where a motor is driven: duration, turns (odometry
heading change / 360, + anticlockwise), distance and mean speed (encoders),
and the line sensor re-run on the logged readings (tools/line_position.py,
config/lokum.json): how often and how long the line was lost, and |offset|.
"""
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "tools"))
import line_position as lp    # noqa: E402

COUNTS_PER_REV = 358.3
RADIUS_MM = 16.26
RUNS = [   # label, follower, settings, how it was started
    ("new_100_3s", "offset", "speed 100, kp 1.5, kd 0", "start, 3 s"),
    ("new_100", "offset", "speed 100, kp 1.5, kd 0", "start, 20 s"),
    ("old", "old (raw PWM)", "base_pwm 45, gain 0.01", "start, 20 s"),
    ("new_150", "offset", "speed 150, kp 1.5, kd 0.5", "start, 20 s"),
    ("new_150_button", "offset", "speed 150, kp 1.5, kd 0.5", "button, stopped by hand"),
]


def load_runs():
    with open(HERE / "raw" / "runs.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    runs = {}
    for label, *_ in RUNS:
        rs = [r for r in rows if r["run"] == label]
        runs[label] = {k: np.array([float(r[k]) for r in rs]) for k in rs[0] if k != "run"}
    return runs


def run_stats(d):
    on = np.flatnonzero((d["pwm_left"] != 0) | (d["pwm_right"] != 0))
    a, b = on[0], on[-1] + 1
    sensor = lp.LineSensor(lp.settings(lp.CONFIG))
    raw = np.column_stack([d[s] for s in lp.SENSORS])
    for r in raw[:a]:
        sensor.update(r)
    res = [sensor.update(r) for r in raw[a:b]]
    seen = np.array([r[3] for r in res])
    offset = np.array([r[1] for r in res])[seen]
    t = d["t_ms"][a:b] / 1000
    dt = float(np.median(np.diff(t)))
    lost = []
    i = 0
    while i < len(seen):
        if not seen[i]:
            j = i
            while j < len(seen) and not seen[j]:
                j += 1
            lost.append((j - i) * dt)
            i = j
        else:
            i += 1
    mm = 2 * math.pi * RADIUS_MM / COUNTS_PER_REV
    dist = ((d["enc_left"][b - 1] - d["enc_left"][a]) + (d["enc_right"][b - 1] - d["enc_right"][a])) / 2 * mm
    duration = t[-1] - t[0]
    turns = (d["theta_rad"][b - 1] - d["theta_rad"][a]) / (2 * math.pi)
    x, y = d["x_mm"][a:b], d["y_mm"][a:b]
    area = 0.5 * float(np.sum(x[:-1] * y[1:] - x[1:] * y[:-1]))
    return {"duration_s": duration, "turns": turns, "distance_mm": dist, "speed_mm_s": dist / duration,
            "direction": "anticlockwise" if area > 0 else "clockwise",
            "lost_n": len(lost), "lost_s": sum(lost), "lost_max_s": max(lost) if lost else 0.0,
            "offset_median_mm": float(np.median(np.abs(offset))), "offset_p95_mm": float(np.percentile(np.abs(offset), 95)),
            "x": x, "y": y, "t": t - t[0]}


def timing(name):
    us = [int(m.group(1)) for line in open(HERE / "raw" / name) if (m := re.search(r" us=(\d+)", line))]
    longest = max(int(m.group(1)) for line in open(HERE / "raw" / name) if (m := re.search(r"max_us=(\d+)", line)))
    return len(us), float(np.median(us)), longest


def main():
    runs = load_runs()
    results = []
    print(f"{'run':16} {'follower':14} {'s':>6} {'turns':>6} {'mm/s':>5} {'s/lap':>6} {'lost':>5} {'lost s':>7} "
          f"{'worst':>6} {'|off| med':>9} {'p95':>5}")
    for label, follower, settings, how in RUNS:
        r = run_stats(runs[label])
        # The old follower's turns include its search spins, so no lap time for it.
        lap = r["duration_s"] / abs(r["turns"]) if follower == "offset" and abs(r["turns"]) > 0.5 else math.nan
        r.update({"run": label, "follower": follower, "settings": settings, "started": how, "s_per_lap": lap})
        results.append(r)
        print(f"{label:16} {follower:14} {r['duration_s']:6.1f} {r['turns']:+6.2f} {r['speed_mm_s']:5.0f} {lap:6.1f} "
              f"{r['lost_n']:5d} {r['lost_s']:7.1f} {r['lost_max_s']:6.2f} {r['offset_median_mm']:9.1f} {r['offset_p95_mm']:5.1f}")
    print("turns: odometry heading change / 360, + anticlockwise (the old run's includes its search spins). "
          "s/lap: duration / |turns|. lost: times the line sensor saw no line; worst: longest spell.")

    for name, what in (("line_replies_float.txt", "float search"), ("line_replies_int.txt", "integer search")):
        n, median, longest = timing(name)
        print(f"line estimate, {what}: median {median:.0f} us over {n} replies, longest {longest} us")

    keys = ["run", "follower", "settings", "started", "direction", "duration_s", "turns", "distance_mm", "speed_mm_s",
            "s_per_lap", "lost_n", "lost_s", "lost_max_s", "offset_median_mm", "offset_p95_mm"]
    with open(HERE / "summary.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        for r in results:
            writer.writerow([f"{r[k]:.2f}" if isinstance(r[k], float) else r[k] for k in keys])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    shown = [("new_100", "offset follower, speed 100, 20 s"), ("old", "old follower, 20 s"),
             ("new_150", "offset follower, speed 150, 20 s"), ("new_150_button", "offset follower, speed 150, 123 s")]
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.6))
    by_run = {r["run"]: r for r in results}
    for ax, (label, title) in zip(axes, shown):
        r = by_run[label]
        sc = ax.scatter(r["x"], r["y"], c=r["t"], s=1, cmap="viridis")
        ax.plot(r["x"][0], r["y"][0], "r^", ms=8)
        ax.set_title(title, fontsize=10)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("x mm (odometry)")
        fig.colorbar(sc, ax=ax, label="s", shrink=0.8)
    axes[0].set_ylabel("y mm (odometry)")
    fig.tight_layout()
    fig.savefig(HERE / "paths.png", dpi=100)
    print(f"saved {HERE / 'summary.csv'} and {HERE / 'paths.png'}")


if __name__ == "__main__":
    main()
