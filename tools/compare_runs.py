"""Compare two line-following runs (robot log or twin log) on the same metrics.

    python3 tools/compare_runs.py logs/run_x.csv logs/twin_y.csv

Only rows with signal == 1 are used. Line detection and offsets come from
running tools/line_position.py on the logged sensor readings. Twin logs also
have the true pose, so for those we get the odometry heading error too.

Assumes lokum (wheel radius, run-min floors) and an anticlockwise course, so
the left wheel is the inner one.
"""
import csv
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

WHEEL_RADIUS_MM = 16.26
COUNTS_PER_REV = 358.3
RUN_MIN_PWM = (26, 19)  # left, right
SPEED_STEP = 5          # rows between encoder samples for speed (~50 ms)


def line_estimates(rows):
    """Run line_position.py over the rows and return its output keyed by t_ms."""
    fields = list(rows[0].keys())[:14]  # drop the twin's true pose columns
    with tempfile.TemporaryDirectory() as out:
        tmp = Path(out) / "in.csv"
        with open(tmp, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in rows:
                w.writerow({k: r[k] for k in fields})
        script = Path(__file__).resolve().parent / "line_position.py"
        subprocess.run([sys.executable, str(script), str(tmp), "--out", out],
                       check=True, capture_output=True)
        with open(Path(out) / "line.csv") as f:
            return {r["t_ms"]: r for r in csv.DictReader(f)}


def column(rows, name):
    return np.array([float(r[name]) for r in rows])


def pwm_flips(pwm):
    return int(np.sum(np.sign(pwm[1:]) * np.sign(pwm[:-1]) < 0))


def metrics(path):
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r["signal"] == "1"]
    line = line_estimates(rows)

    t = column(rows, "t_ms")
    theta = column(rows, "theta_rad")
    enc_l = column(rows, "enc_left")
    enc_r = column(rows, "enc_right")
    pwm_l = column(rows, "pwm_left")
    pwm_r = column(rows, "pwm_right")
    seen = np.array([line[r["t_ms"]]["detected"] == "1" for r in rows])
    offset = np.array([float(line[r["t_ms"]]["offset_mm"] or "nan") for r in rows])

    duration = (t[-1] - t[0]) / 1000
    laps = (theta[-1] - theta[0]) / (2 * math.pi)

    mm_per_count = 2 * math.pi * WHEEL_RADIUS_MM / COUNTS_PER_REV
    k = SPEED_STEP
    dt = (t[k:] - t[:-k]) / 1000
    v_l = (enc_l[k:] - enc_l[:-k]) * mm_per_count / dt
    v_r = (enc_r[k:] - enc_r[:-k]) * mm_per_count / dt
    v = (v_l + v_r) / 2

    losses = np.sum(np.diff(seen.astype(int)) == -1)

    m = {
        "duration_s": duration,
        "laps": laps,
        "s_per_lap": duration / abs(laps) if laps else float("nan"),
        "speed_p5": np.percentile(v, 5),
        "speed_med": np.median(v),
        "speed_p95": np.percentile(v, 95),
        "line_lost_per_lap": losses / abs(laps),
        "lost_time_pct": 100 * np.mean(~seen),
        "abs_offset_med": np.nanmedian(abs(offset)),
        "abs_offset_p95": np.nanpercentile(abs(offset), 95),
        "inner_pwm_flips_s": pwm_flips(pwm_l) / duration,
        "outer_pwm_flips_s": pwm_flips(pwm_r) / duration,
        "inner_at_floor_pct": 100 * np.mean(np.abs(np.round(pwm_l)) == RUN_MIN_PWM[0]),
        "outer_speed_max": v_r.max(),
        "inner_speed_min": v_l.min(),
    }

    if "true_theta_rad" in rows[0]:
        true_theta = column(rows, "true_theta_rad")
        err = (theta[-1] - theta[0]) - (true_theta[-1] - true_theta[0])
        m["odo_heading_err_deg_per_lap"] = math.degrees(err) / abs(laps)

    return m


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    a = metrics(sys.argv[1])
    b = metrics(sys.argv[2])
    print(f"{'metric':28s} {'first':>10s} {'second':>10s}")
    for k in list(a) + [k for k in b if k not in a]:
        print(f"{k:28s} {a.get(k, float('nan')):10.2f} {b.get(k, float('nan')):10.2f}")


if __name__ == "__main__":
    main()
