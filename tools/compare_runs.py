"""Compare template-follower runs on the robot with runs in the twin.

    python3 tools/compare_runs.py --robot logs/run_a.csv logs/run_b.csv --twin logs/twin_*_seed*.csv
    python3 tools/compare_runs.py --start cw --robot ... --twin logs/twin_*_cw_seed*.csv --plot out.png

All runs in one call should start the same way round: from the twin's home
pose (left line, halfway down) facing down the page is anticlockwise (acw, the
default), facing up is clockwise (cw). Laps are counted by how far the
robot's position goes round the course's middle, which is COURSE_CENTRE_MM to
the robot's left (acw) or right (cw) at the start. Spinning on the spot
doesn't count; it shows up in spin_turns instead.

Each run is cut to the first --seconds after the start (signal 1), so long
and short runs are compared on the same stretch. Every metric is worked out
per run, then shown as the median and range over each side's runs.

The follower's state is read back from the logged PWM: (recovery_left_pwm,
recovery_right_pwm) or the same swapped (a right spin) is searching, (0, 0)
is stopped, anything else is following. Settings and wheel size come from config/lokum.json. Line offsets
come from tools/line_position.py on the logged sensor readings. Twin logs also
have the true pose, so for those we get the odometry heading error too.
"""
import argparse
import csv
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

CONFIG = Path(__file__).resolve().parent.parent / "config" / "lokum.json"
COUNTS_PER_REV = 358.3
SPEED_STEP_MS = 50     # encoder window for speed
STEER_SWING = 3         # PWM
COURSE_CENTRE_MM = 140  # sideways from the home pose to the middle of the course


def parse_args():
    p = argparse.ArgumentParser(description="Robot vs twin, template line follower.")
    p.add_argument("--robot", nargs="+", default=[], help="robot logger CSVs")
    p.add_argument("--twin", nargs="+", default=[], help="twin CSVs")
    p.add_argument("--start", choices=["acw", "cw"], default="acw", help="way round the runs start (default acw)")
    p.add_argument("--seconds", type=float, default=60, help="compare this long from the start (default 60)")
    p.add_argument("--config", type=Path, default=CONFIG)
    p.add_argument("--plot", type=Path, help="save paths and search timelines to this image")
    return p.parse_args()


def load(path, seconds):
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r["signal"] == "1"]
    if not rows:
        sys.exit(f"{path}: no rows with signal 1")
    t0 = float(rows[0]["t_ms"])
    return [r for r in rows if float(r["t_ms"]) - t0 <= seconds * 1000]


def col(rows, name):
    return np.array([float(r[name]) for r in rows])


def line_offsets(rows, config):
    """line_position.py over the rows: offset (mm, nan when not seen) per row."""
    fields = list(rows[0].keys())[:14]   # drop the twin's true pose columns
    with tempfile.TemporaryDirectory() as out:
        tmp = Path(out) / "in.csv"
        with open(tmp, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in rows:
                w.writerow({k: r[k] for k in fields})
        script = Path(__file__).resolve().parent / "line_position.py"
        subprocess.run([sys.executable, str(script), str(tmp), "--out", out, "--config", str(config)],
                       check=True, capture_output=True)
        with open(Path(out) / "line.csv") as f:
            line = {r["t_ms"]: r for r in csv.DictReader(f)}
    return np.array([float(line[r["t_ms"]]["offset_mm"] or "nan") for r in rows])


def runs_of(mask):
    """(start, end) index pairs of each run of True, end exclusive."""
    d = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def metrics(path, cfg, args):
    rows = load(path, args.seconds)
    t = (col(rows, "t_ms") - float(rows[0]["t_ms"])) / 1000
    theta = col(rows, "theta_rad")
    pwm_l, pwm_r = col(rows, "pwm_left"), col(rows, "pwm_right")
    enc_l, enc_r = col(rows, "enc_left"), col(rows, "enc_right")

    rl, rr = cfg["recovery_left_pwm"], cfg["recovery_right_pwm"]
    searching = ((pwm_l == rl) & (pwm_r == rr)) | ((pwm_l == rr) & (pwm_r == rl))
    stopped = (pwm_l == 0) & (pwm_r == 0)
    following = ~searching & ~stopped

    duration = t[-1] - t[0]
    # laps round the course's middle, + the way the run started
    x, y = path_from_start(rows)
    side = 1 if args.start == "acw" else -1
    around = np.unwrap(np.arctan2(y - side * COURSE_CENTRE_MM, x))
    laps = side * (around[-1] - around[0]) / (2 * math.pi)
    turns = side * (theta[-1] - theta[0]) / (2 * math.pi)
    searches = runs_of(searching)
    search_ms = [1000 * (t[min(e, len(t) - 1)] - t[s]) for s, e in searches]

    # Going the wrong way: share of the distance driven while the position
    # moves backwards round the course.
    step = np.diff(around) * side
    wrong_way_pct = 100 * np.sum(-step[step < 0]) / max(np.sum(abs(step)), 1e-9)

    # wheel speeds over about 50 ms
    k = max(1, int(round(SPEED_STEP_MS / 1000 / np.median(np.diff(t)))))
    mm_l = 2 * math.pi * cfg["left_wheel_radius_mm"] / COUNTS_PER_REV
    mm_r = 2 * math.pi * cfg["right_wheel_radius_mm"] / COUNTS_PER_REV
    dt = t[k:] - t[:-k]
    v = ((enc_l[k:] - enc_l[:-k]) * mm_l + (enc_r[k:] - enc_r[:-k]) * mm_r) / 2 / dt
    v_follow = v[following[k:] & following[:-k]]

    # steering while following: half the PWM difference, and how often it
    # swings from one side to the other (past +-STEER_SWING, so noise round 0 doesn't count)
    steer = (pwm_r - pwm_l)[following] / 2
    sides = np.sign(steer[abs(steer) >= STEER_SWING])
    flips = np.sum(sides[1:] != sides[:-1])

    offset = line_offsets(rows, args.config)[following]

    m = {
        "duration_s": duration,
        "laps": laps,
        "s_per_lap": duration / abs(laps) if abs(laps) > 0.05 else math.nan,
        "searches_per_lap": len(searches) / abs(laps) if abs(laps) > 0.05 else math.nan,
        "search_ms_med": np.median(search_ms) if search_ms else 0.0,
        "search_ms_max": max(search_ms) if search_ms else 0.0,
        "search_time_pct": 100 * searching.mean(),
        "first_search_s": t[searches[0][0]] if searches else math.nan,
        "wrong_way_pct": wrong_way_pct,
        "spin_turns": turns - laps,
        "stopped_at_s": t[np.argmax(stopped)] if stopped.any() else math.nan,
        "speed_follow_med": np.median(v_follow) if len(v_follow) else math.nan,
        "speed_follow_p95": np.percentile(v_follow, 95) if len(v_follow) else math.nan,
        "steer_pwm_rms": math.sqrt(np.mean(steer ** 2)) if len(steer) else math.nan,
        "steer_swings_per_s": flips / max(following.mean() * duration, 1e-9),
        "abs_offset_med": np.nanmedian(abs(offset)),
        "abs_offset_p95": np.nanpercentile(abs(offset), 95),
    }
    if "true_theta_rad" in rows[0]:
        true_theta = col(rows, "true_theta_rad")
        err = (theta[-1] - theta[0]) - (true_theta[-1] - true_theta[0])
        m["odo_heading_err_deg_per_turn"] = math.degrees(err) / abs(turns) if abs(turns) > 0.05 else math.nan
    return m, rows, t, searching, stopped


def summary(values):
    v = np.array([x for x in values if not math.isnan(x)])
    if len(v) == 0:
        return "-"
    if len(v) == 1:
        return f"{v[0]:.2f}"
    return f"{np.median(v):.2f} [{v.min():.2f}, {v.max():.2f}]"


def path_from_start(rows):
    """Odometry path turned so each run starts at (0, 0) heading +x."""
    x, y, th = col(rows, "x_mm"), col(rows, "y_mm"), col(rows, "theta_rad")
    c, s = math.cos(-th[0]), math.sin(-th[0])
    dx, dy = x - x[0], y - y[0]
    return c * dx - s * dy, s * dx + c * dy


def plot(results, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colours = {"robot": "#d0622b", "twin": "#2b6cd0"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5), gridspec_kw={"width_ratios": [1, 1.4]})
    row = 0
    for side, runs in results.items():
        for name, (m, rows, t, searching, stopped) in runs:
            x, y = path_from_start(rows)
            ax1.plot(x, y, color=colours[side], lw=0.8, alpha=0.6)
            for s, e in runs_of(searching):
                ax2.plot([t[s], t[min(e, len(t) - 1)]], [row, row], color=colours[side], lw=5,
                         solid_capstyle="butt")
            if stopped.any():
                ax2.plot(t[np.argmax(stopped)], row, "kx")
            ax2.text(t[-1] + 0.5, row, Path(name).stem, va="center", fontsize=7)
            row += 1
    for side, c in colours.items():
        ax1.plot([], [], color=c, label=side)
    ax1.set_aspect("equal")
    ax1.set_xlabel("x from start (mm)")
    ax1.set_ylabel("y from start (mm)")
    ax1.set_title("Odometry paths, each from its own start")
    ax1.legend()
    ax2.set_xlabel("time from start (s)")
    ax2.set_yticks([])
    ax2.set_title("Searching for the line (bars); x = stopped")
    fig.tight_layout()
    fig.savefig(out, dpi=130)


def main():
    args = parse_args()
    cfg = json.loads(args.config.read_text())
    cfg = {"recovery_left_pwm": -35, "recovery_right_pwm": 35,
           "left_wheel_radius_mm": 16.0, "right_wheel_radius_mm": 16.0, **cfg}

    results = {side: [(f, metrics(f, cfg, args)) for f in getattr(args, side)]
               for side in ("robot", "twin") if getattr(args, side)}
    if not results:
        sys.exit("give --robot and/or --twin runs")

    names = []
    for runs in results.values():
        for _, r in runs:
            names += [k for k in r[0] if k not in names]
    print(f"first {args.seconds:g} s of each run, starting {args.start}; median [min, max] over "
          + " and ".join(f"{len(runs)} {side}" for side, runs in results.items()) + " runs")
    print(f"{'metric':28s}" + "".join(f" {side:>26s}" for side in results))
    for k in names:
        cells = [summary([r[0].get(k, math.nan) for _, r in runs]) for runs in results.values()]
        print(f"{k:28s}" + "".join(f" {c:>26s}" for c in cells))
    if args.plot:
        plot(results, args.plot)
        print(f"plot: {args.plot}")


if __name__ == "__main__":
    main()
