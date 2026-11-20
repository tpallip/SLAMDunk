"""How far the robot was off the line: recorded run vs pure pursuit replays.

The line sensors are the ground truth. For every telemetry record the line's
sideways offset at the sensors (mm, + = line to the robot's left) is worked
out the way tools/line_position.py does, from all five readings. The
recorded run is the follower steering on the line; a replay is the robot
driving the recorded odometry with the line follower off, so its offset is
how far the reproduction drifted from the real path.

    python tools/path_error.py logs/run_20261119_231317.csv logs/path_reproduction/replay_01.csv
    python tools/path_error.py logs/<record>.csv logs/path_reproduction/replay_*.csv --plot

The offset is measured at the sensors, about 40 mm in front of the axle, so
a heading error shows up in it too. Distance along the path is the
odometry's distance from the start of each run. Results go to
bench/path_reproduction/<record name>/.
"""
import argparse
import csv
import math
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from line_position import CONFIG, SENSORS, LineSensor, settings

OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "path_reproduction"
BIN_MM = 50


def parse_args():
    p = argparse.ArgumentParser(description="Line offset of a recorded run and its replays.")
    p.add_argument("record", type=Path, help="logger CSV of the recorded run")
    p.add_argument("replays", type=Path, nargs="+", help="replay CSVs from trajectory_controller.py")
    p.add_argument("--run", type=int, default=1, help="which signal=1 run in the record (default 1)")
    p.add_argument("--config", type=Path, default=CONFIG)
    p.add_argument("--out", type=Path)
    p.add_argument("--plot", action="store_true")
    return p.parse_args()


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) if r[k] != "" else math.nan for r in rows]) for k in rows[0]}


def section(data, flag, run=1):
    """Row range of the run-th stretch where flag is 1."""
    edges = np.flatnonzero(np.diff(np.r_[0, (data[flag] == 1).astype(int), 0]))
    runs = list(zip(edges[::2], edges[1::2]))
    if not 1 <= run <= len(runs):
        raise SystemExit(f"no run {run} with {flag} = 1")
    return runs[run - 1]


def offsets(data, a, b, cfg):
    sensor = LineSensor(cfg)
    raw = np.column_stack([data[s] for s in SENSORS])[a:b]
    out = np.full(b - a, np.nan)
    for i, r in enumerate(raw):
        _, off, _, seen = sensor.update(r)
        if seen:
            out[i] = off
    along = np.r_[0, np.cumsum(np.hypot(np.diff(data["x_mm"][a:b]), np.diff(data["y_mm"][a:b])))]
    return along, out


def stats(off):
    seen = ~np.isnan(off)
    o = off[seen]
    return {"seen": 100 * seen.mean(), "mean": o.mean(), "rms": math.sqrt((o ** 2).mean()),
            "p95": np.percentile(np.abs(o), 95), "max": np.abs(o).max()}


def main():
    args = parse_args()
    console = Console(record=True)
    cfg = settings(args.config)

    runs = []
    rec = load(args.record)
    a, b = section(rec, "signal", args.run)
    runs.append(("record (follower)", *offsets(rec, a, b, cfg), None))
    for path in args.replays:
        d = load(path)
        a, b = section(d, "driving")
        along, off = offsets(d, a, b, cfg)
        runs.append((path.stem, along, off, d["off_mm"][a:b]))

    table = Table(title=f"Line offset at the sensors, mm ({args.record.name})")
    for column in ["run", "length mm", "line seen %", "mean", "rms", "95 % |off|", "max |off|", "odometry off path rms"]:
        table.add_column(column, justify="right")
    for name, along, off, track in runs:
        s = stats(off)
        odo = "" if track is None else f"{math.sqrt(np.nanmean(track ** 2)):.1f}"
        table.add_row(name, f"{along[-1]:.0f}", f"{s['seen']:.0f}", f"{s['mean']:+.1f}", f"{s['rms']:.1f}",
                      f"{s['p95']:.1f}", f"{s['max']:.1f}", odo)
    console.print(table)
    console.print("+ = line to the robot's left. 'odometry off path' is how well pure pursuit kept the "
                  "odometry on the recorded path; the line offset is how far that put the robot off the real line.")

    # offset every BIN_MM along the path, to see where the replay drifts
    edges = np.arange(0, max(r[1][-1] for r in runs) + BIN_MM, BIN_MM)
    table = Table(title=f"Mean line offset per {BIN_MM} mm along the path")
    table.add_column("from mm", justify="right")
    for name, *_ in runs:
        table.add_column(name, justify="right")
    for lo in edges[:-1]:
        cells = []
        for _, along, off, _ in runs:
            m = (along >= lo) & (along < lo + BIN_MM) & ~np.isnan(off)
            cells.append(f"{off[m].mean():+.1f}" if m.any() else "")
        table.add_row(f"{lo:.0f}", *cells)
    console.print(table)

    out = args.out or OUT_DIR / args.record.stem
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "offsets.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run", "along_mm", "line_offset_mm"])
        for name, along, off, _ in runs:
            for s_, o in zip(along, off):
                w.writerow([name, f"{s_:.1f}", "" if math.isnan(o) else f"{o:.2f}"])
    (out / "summary.txt").write_text(console.export_text())
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 4))
        for name, along, off, _ in runs:
            ax.plot(along, off, lw=0.9, label=name)
        ax.axhline(0, color="0.5", lw=0.5)
        ax.set_xlabel("distance along the path (mm)")
        ax.set_ylabel("line offset at the sensors (mm, + left)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out / "offsets.png", dpi=110)
    console.print(f"saved {out}/")


if __name__ == "__main__":
    main()
