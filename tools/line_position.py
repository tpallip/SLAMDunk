"""Line offset from logged surface readings, the same way the robot works it out.

Mirror of StampC3_Template/LineSensor.cpp, for checking it on logged runs
without reflashing: each sensor normalised to 0 on white and 1 on black with
its own levels, then the line's sideways offset (mm, + to the robot's left)
as the offset whose predicted readings match best. For a line of width w
seen through each sensor's Gaussian footprint sigma, a sensor y mm to the
left of the robot's middle reads

    n(e) = Phi((y - e + w/2) / sigma) - Phi((y - e - w/2) / sigma)

with the sensors on a 45 mm circle at 45, 14, 0, -14, -45 deg (DN1 left to
DN5 right). The search runs every 2 mm over +-50 mm, then every 0.5 mm within
2 mm of the best, with a parabola through the best point; a weak pull towards the previous offset picks the side when
only DN1 or DN5 sees the line. The line counts as seen when any sensor reads
at least line_detect_level.

Levels, footprints, line width and detect level come from the robot's config
file (dn<k>_white, dn<k>_black, dn<k>_sigma_mm, line_width_mm,
line_detect_level), the same keys the logger sends to the robot.

    python tools/line_position.py logs/run_x.csv
    python tools/line_position.py logs/run_x.csv --config config/lokum.json --plot
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from rich.console import Console

SENSORS = ["dn1", "dn2", "dn3", "dn4", "dn5"]
ANGLES_DEG = [45, 14, 0, -14, -45]     # DN1 (left) to DN5 (right); 0 is straight ahead
RADIUS_MM = 45.0
MAX_OFFSET_MM = 50.0
STEP_MM = 0.5
COARSE = 4                              # coarse search every 4 steps (2 mm)
CONTINUITY = 1e-5                       # per mm^2 from the previous offset, squared-reading units
CONFIG = Path(__file__).resolve().parent.parent / "config" / "lokum.json"
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "line_position"


def parse_args():
    parser = argparse.ArgumentParser(description="Line offset from logged surface readings (LineSensor_c mirror).")
    parser.add_argument("file", help="logger CSV")
    parser.add_argument("--config", type=Path, default=CONFIG, help=f"robot config (default: {CONFIG.name})")
    parser.add_argument("--out", type=Path, help=f"folder for the results (default: {OUT_DIR}/<file name>)")
    parser.add_argument("--plot", action="store_true", help="also save a plot (needs matplotlib)")
    return parser.parse_args()


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}


def settings(path):
    c = json.loads(path.read_text())
    get = lambda key: float(c[key])
    return {"white": np.array([get(f"{s}_white") for s in SENSORS]),
            "black": np.array([get(f"{s}_black") for s in SENSORS]),
            "sigma": np.array([get(f"{s}_sigma_mm") for s in SENSORS]),
            "width": get("line_width_mm"), "detect": get("line_detect_level")}


def normal_cdf(z):
    return 0.5 * (1.0 + np.vectorize(math.erf)(z / math.sqrt(2.0)))


class LineSensor:
    def __init__(self, cfg):
        self.cfg = cfg
        self.y = RADIUS_MM * np.sin(np.radians(ANGLES_DEG))
        self.offsets = np.arange(-MAX_OFFSET_MM, MAX_OFFSET_MM + STEP_MM / 2, STEP_MM)
        # predicted[i, j]: sensor i's reading with the line's middle at offsets[j]
        d = self.y[:, None] - self.offsets[None, :]
        w, sigma = cfg["width"], cfg["sigma"][:, None]
        self.predicted = normal_cdf((d + w / 2) / sigma) - normal_cdf((d - w / 2) / sigma)
        self.offset = 0.0

    def normalise(self, raw):
        span = self.cfg["black"] - self.cfg["white"]
        return np.clip((raw - self.cfg["white"]) / span, 0.0, 1.0)

    def update(self, raw):
        """(normalised readings, offset mm, residual, detected) for one record."""
        n = self.normalise(raw)
        if n.max() < self.cfg["detect"]:
            return n, self.offset, math.nan, False
        fit = ((n[:, None] - self.predicted) ** 2).sum(axis=0)
        cost = fit + CONTINUITY * (self.offsets - self.offset) ** 2
        coarse = np.arange(0, len(cost), COARSE)
        best = int(coarse[np.argmin(cost[coarse])])
        lo, hi = max(best - COARSE, 0), min(best + COARSE, len(cost) - 1)
        best = lo + int(np.argmin(cost[lo:hi + 1]))
        shift = 0.0
        if lo < best < hi:
            a, b, c = cost[best - 1:best + 2]
            if a - 2 * b + c > 1e-9:
                shift = 0.5 * (a - c) / (a - 2 * b + c)
        self.offset = self.offsets[best] + shift * STEP_MM
        return n, self.offset, math.sqrt(max(fit[best], 0.0) / len(n)), True


def main():
    args = parse_args()
    console = Console()
    data = load(args.file)
    sensor = LineSensor(settings(args.config))
    raw = np.column_stack([data[s] for s in SENSORS])
    rows = []
    for t, r in zip(data["t_ms"], raw):
        n, offset, residual, seen = sensor.update(r)
        rows.append([t, *n, offset if seen else math.nan, residual, int(seen)])
    rows = np.array(rows)
    seen = rows[:, -1] == 1
    console.print(f"{len(rows)} records, line seen in {seen.sum()} ({100 * seen.mean():.0f} %)")
    if seen.any():
        off, res = rows[seen, 6], rows[seen, 7]
        console.print(f"offset {off.min():+.1f} to {off.max():+.1f} mm, median {np.median(off):+.1f}; "
                      f"residual median {np.median(res):.3f}, p95 {np.percentile(res, 95):.3f}")

    out = args.out or OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "line.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ms", *[f"n_{s}" for s in SENSORS], "offset_mm", "residual", "detected"])
        for row in rows:
            writer.writerow([f"{row[0]:.0f}", *[f"{x:.3f}" for x in row[1:6]],
                             "" if math.isnan(row[6]) else f"{row[6]:.2f}",
                             "" if math.isnan(row[7]) else f"{row[7]:.3f}", f"{row[8]:.0f}"])
    console.print(f"saved {out / 'line.csv'}")
    if args.plot:
        plot(rows, out / "line.png", console)


def plot(rows, path, console):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        console.print("matplotlib not installed: no plot")
        return
    t = (rows[:, 0] - rows[0, 0]) / 1000
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for i, s in enumerate(SENSORS):
        top.plot(t, rows[:, 1 + i], lw=0.8, label=s)
    top.set_ylabel("normalised reading")
    top.legend(ncol=5, fontsize=8)
    bottom.plot(t, rows[:, 6], ".", ms=2)
    bottom.set_ylabel("line offset (mm, + left)")
    bottom.set_xlabel("time (s)")
    for ax in (top, bottom):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    console.print(f"saved {path}")


if __name__ == "__main__":
    main()
