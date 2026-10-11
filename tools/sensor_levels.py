"""Black and white reading levels of each surface sensor, from a run over tape.

Readings are labelled by where the sensor was relative to the tape: black when
at least --margin mm inside a tape pass (from both edges), white when at least
--margin mm from any edge. Readings near an edge, where the sensor's footprint
covers both, are left out. Edges are each sensor's own 50 % crossings along its
track, positioned from encoder odometry as in tools/surface_map.py.

For each sensor it reports the level, noise and range on white and on black,
the separation between them (also in noise units), the threshold that splits
them best, and how LINE_THRESHOLD (Controller_c) does. TODO items 3.1-3.3.

It also saves raw_vs_greyscale.png: each reading against the 8-bit floor
greyscale under the sensor (0 black .. 255 white, as image brightness and as
DigitalTwin's SurfaceSensorModel), every reading near a tape pass included,
and readings_greyscale.csv with those values. A sensor sees an area, not a
point, so near a tape edge it reads in between: the greyscale it sees is the
footprint-weighted average of the floor, 255 * (1 - share of the footprint
over the tape), with the footprint a Gaussian spot whose width (sigma) is
fitted per sensor. The plot shows how linear the response is across greys.

    python tools/sensor_levels.py logs/run_x.csv
    python tools/sensor_levels.py logs/run_x.csv --separation 83.2 --margin 4
"""
import argparse
import csv
import math
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

import surface_map as sm
from sensor_spread import erf

LINE_THRESHOLD = 1400     # Controller_c::LINE_THRESHOLD
GREY_WHITE = 255          # 8-bit greyscale of white floor; black tape is 0
SURFACE_MAX = 2942        # highest reading the sensors report (saturated)
MARGIN_MM = 5.0
MAX_PASS_MM = 40.0        # longer "tape passes" are not crossings (sensor running along the tape)
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "3.2_sensor_levels"


def parse_args():
    parser = argparse.ArgumentParser(description="Per-sensor black/white levels from a run over tape.")
    parser.add_argument("file", help="logger CSV")
    parser.add_argument("--separation", type=float,
                        help="wheel separation in mm (default: fitted as in surface_map.py --fit-separation)")
    parser.add_argument("--margin", type=float, default=MARGIN_MM,
                        help=f"mm from a tape edge before a reading counts (default: {MARGIN_MM})")
    parser.add_argument("--radius", type=float, nargs="+", default=[sm.SENSOR_RADIUS_MM], metavar="MM")
    parser.add_argument("--angles", type=float, nargs=5, default=sm.SENSOR_ANGLES_DEG, metavar="DEG")
    parser.add_argument("--start", type=float, help="ignore records before this t_ms")
    parser.add_argument("--end", type=float, help="ignore records after this t_ms")
    parser.add_argument("--out", type=Path, help=f"folder for the results (default: {OUT_DIR}/<file name>)")
    args = parser.parse_args()
    # surface_map's helpers read these from args.
    args.wheel_radius, args.counts_per_rev = sm.WHEEL_RADIUS_MM, sm.COUNTS_PER_REV
    return args


def label_readings(raw, level, sx, sy, t_ms, margin):
    """+1 black, -1 white, 0 unused, for one sensor's readings along its track.

    Also returns the tape passes as (rising, falling) positions along the track,
    the track itself, and the edges that belong to no pass (or whose position
    is unknown), near which a reading's greyscale can't be worked out.
    """
    dt = np.diff(t_ms)
    track = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(sx), np.diff(sy)))])   # mm along the track
    dark = level >= 0.5
    edges = []      # (position along track, rising to black?)
    for i in np.flatnonzero(np.diff(dark.astype(int))):
        if dt[i] > 3 * np.median(dt):
            edges.append(None)      # crossing inside a telemetry gap: position unknown
            continue
        f = (0.5 - level[i]) / (level[i + 1] - level[i])
        edges.append((track[i] + f * (track[i + 1] - track[i]), bool(dark[i + 1])))

    label = np.zeros(len(raw), int)
    known = [e for e in edges if e is not None]
    if not known:
        return label, [], track, []
    positions = np.array([p for p, _ in known])
    # Distance to the nearest edge decides white; unknown edges make nearby readings unusable.
    nearest = np.min(np.abs(track[:, None] - positions[None, :]), axis=1)
    label[(~dark) & (nearest >= margin)] = -1

    passes, paired = [], set()
    for k, (a, b) in enumerate(zip(edges, edges[1:])):
        if a and b and a[1] and not b[1] and b[0] - a[0] <= MAX_PASS_MM:
            inside = (track >= a[0] + margin) & (track <= b[0] - margin)
            label[inside & dark] = 1
            passes.append((a[0], b[0]))
            paired |= {k, k + 1}
    # A gap edge leaves a reading's side unknown: drop readings next to gaps.
    gaps = [track[i] for i in np.flatnonzero(dt > 3 * np.median(dt))]
    for position in gaps:
        label[np.abs(track - position) < margin] = 0
    unpaired = [e[0] for k, e in enumerate(edges) if e is not None and k not in paired] + gaps
    return label, passes, track, unpaired


def greyscale(track, passes, sigma):
    """Share of a Gaussian footprint (sigma mm) over the tape at each track position."""
    g = np.zeros(len(track))
    scale = sigma * math.sqrt(2.0)
    for a, b in passes:
        g += 0.5 * (erf((track - a) / scale) - erf((track - b) / scale))
    return np.clip(g, 0, 1)


def to_grey(tape_share):
    """8-bit greyscale (0 black, 255 white) from the share of the footprint over black tape."""
    return GREY_WHITE * (1.0 - tape_share)


def grey_at(raw_value, row):
    """Greyscale at which the fitted response reaches raw_value."""
    return float(np.clip((row["raw_at_grey0"] - raw_value) / row["raw_per_grey"], 0, GREY_WHITE)) \
        if row["raw_per_grey"] else float("nan")


def fit_response(raw, track, passes, unpaired, reach=15.0):
    """Footprint sigma and linear response raw = white + span * greyscale.

    Uses readings within reach mm of a tape pass and away from unpaired edges;
    readings at the SURFACE_MAX ceiling are left out of the fit (clipped).
    Returns (sigma, white, span, mask of the readings used).
    """
    near = np.zeros(len(track), bool)
    for a, b in passes:
        near |= (track > a - reach) & (track < b + reach)
    for position in unpaired:
        near &= np.abs(track - position) > reach
    fit = near & (raw < SURFACE_MAX - 2)
    best = None
    for sigma in np.arange(0.3, 8.0, 0.05):
        g = greyscale(track[fit], passes, sigma)
        design = np.column_stack([np.ones(fit.sum()), g])
        (white, span), *_ = np.linalg.lstsq(design, raw[fit], rcond=None)
        sse = float(np.sum((design @ [white, span] - raw[fit]) ** 2))
        if best is None or sse < best[0]:
            best = (sse, sigma, white, span)
    _, sigma, white, span = best
    return float(sigma), float(white), float(span), near


def best_threshold(white, black):
    """Middle of the gap between the classes, or, if they overlap, the threshold
    with the fewest misclassified readings (each class weighted equally)."""
    low, high = np.percentile(white, 99.9), np.percentile(black, 0.1)
    if high > low:
        return float((low + high) / 2), 0.0
    candidates = np.unique(np.concatenate([white, black]))
    errors = [np.mean(white >= c) + np.mean(black < c) for c in candidates]
    return float(candidates[int(np.argmin(errors))]), float(min(errors)) / 2


def main():
    args = parse_args()
    console = Console()
    data = sm.load(args.file, args.start, args.end)
    radii = args.radius * 5 if len(args.radius) == 1 else args.radius
    levels = sm.darkness(data, console)
    separation = args.separation or sm.fit_separation(data, levels, args, radii, args.angles)
    x, y, theta = sm.odometry(data["enc_left"], data["enc_right"], args.wheel_radius,
                              separation, args.counts_per_rev)
    console.print(f"wheel separation {separation:.1f} mm; margin {args.margin} mm from tape edges")

    table = Table(title=f"Surface sensor levels: {Path(args.file).name}")
    for column in ["sensor", "passes", "white n", "white mean", "white sd", "white p99.9",
                   "black n", "black mean", "black sd", "black p0.1", "separation", "sep / sd",
                   "gap", "best threshold", f"errors at {LINE_THRESHOLD}"]:
        table.add_column(column, justify="right")
    summary, curves = [], {}
    for name, radius, angle, level in zip(sm.SENSORS, radii, args.angles, levels):
        lx, ly = radius * math.cos(math.radians(angle)), radius * math.sin(math.radians(angle))
        sx = x + lx * np.cos(theta) - ly * np.sin(theta)
        sy = y + lx * np.sin(theta) + ly * np.cos(theta)
        raw = data[name]
        label, pass_list, track, unpaired = label_readings(raw, level, sx, sy, data["t_ms"], args.margin)
        passes = len(pass_list)
        white, black = raw[label == -1], raw[label == 1]
        if len(white) < 10 or len(black) < 10:
            table.add_row(name, str(passes), str(len(white)), *[""] * 3, str(len(black)), *[""] * 8)
            console.print(f"[yellow]{name}: too few labelled readings[/]")
            continue
        sep = black.mean() - white.mean()
        pooled = math.sqrt((white.var() + black.var()) / 2)
        gap = np.percentile(black, 0.1) - np.percentile(white, 99.9)
        threshold, error = best_threshold(white, black)
        at_line = (np.mean(white >= LINE_THRESHOLD) + np.mean(black < LINE_THRESHOLD)) / 2
        saturated = np.mean(black >= SURFACE_MAX)
        row = {"sensor": name, "passes": passes, "white_n": len(white), "white_mean": white.mean(),
               "white_sd": white.std(), "white_p0.1": np.percentile(white, 0.1),
               "white_p99.9": np.percentile(white, 99.9), "black_n": len(black),
               "black_mean": black.mean(), "black_sd": black.std(), "black_p0.1": np.percentile(black, 0.1),
               "black_p99.9": np.percentile(black, 99.9), "separation": sep, "sep_over_sd": sep / pooled,
               "gap": gap, "best_threshold": threshold, "best_threshold_error": error,
               "midpoint": (white.mean() + black.mean()) / 2, "line_threshold_error": at_line,
               "black_saturated": saturated}
        if passes:
            sigma, fit_white, fit_span, near = fit_response(raw, track, pass_list, unpaired)
            # Fitted line in 8-bit greyscale: raw = raw_at_grey0 - raw_per_grey * grey.
            row.update({"footprint_sigma_mm": sigma, "raw_at_grey0": fit_white + fit_span,
                        "raw_at_grey255": fit_white, "raw_per_grey": fit_span / GREY_WHITE})
            row["best_threshold_grey"] = grey_at(row["best_threshold"], row)
            row["line_threshold_grey"] = grey_at(LINE_THRESHOLD, row)
            curves[name] = (to_grey(greyscale(track[near], pass_list, sigma)), raw[near],
                            data["t_ms"][near], row)
        summary.append(row)
        table.add_row(name, str(passes), str(len(white)), f"{white.mean():.0f}", f"{white.std():.1f}",
                      f"{row['white_p99.9']:.0f}", str(len(black)), f"{black.mean():.0f}", f"{black.std():.1f}",
                      f"{row['black_p0.1']:.0f}", f"{sep:.0f}", f"{sep / pooled:.0f}",
                      f"[{'green' if gap > 0 else 'red'}]{gap:.0f}[/]", f"{threshold:.0f}",
                      f"{at_line * 100:.2f} %")
        if saturated > 0.01:
            console.print(f"[yellow]{name}: {saturated * 100:.0f} % of black readings at the {SURFACE_MAX} "
                          f"maximum, so its black mean and sd are clipped[/]")
    console.print(table)
    console.print("sep / sd: separation in units of the pooled noise. gap: lowest black (0.1 %) minus "
                  "highest white (99.9 %); positive means a threshold in between never errs. best threshold: "
                  "middle of the gap (or fewest misclassified readings if none). errors: share of readings on the wrong side "
                  f"of {LINE_THRESHOLD}.")

    out = args.out or OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "levels.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary[0]) + ["separation_mm", "margin_mm"])
        writer.writeheader()
        for row in summary:
            writer.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in row.items()}
                            | {"separation_mm": f"{separation:.1f}", "margin_mm": args.margin})
    console.print(f"saved {out / 'levels.csv'}")
    with open(out / "readings_greyscale.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sensor", "t_ms", "grey_0_255", "raw"])
        for name, (grey, raw, t_ms, _) in curves.items():
            for g, r, t in zip(grey, raw, t_ms):
                writer.writerow([name, f"{t:.0f}", f"{g:.1f}", f"{r:.0f}"])
    console.print(f"saved {out / 'readings_greyscale.csv'}")
    for name, (*_, row) in curves.items():
        console.print(f"{name}: raw = {row['raw_at_grey0']:.0f} - {row['raw_per_grey']:.2f} x grey  "
                      f"(black {row['raw_at_grey0']:.0f}, white {row['raw_at_grey255']:.0f}); "
                      f"{LINE_THRESHOLD} at grey {row['line_threshold_grey']:.0f}, "
                      f"suggested {row['best_threshold']:.0f} at grey {row['best_threshold_grey']:.0f}")
    plot_response(curves, out / "raw_vs_greyscale.png", Path(args.file).name, console)


def plot_response(curves, path, title, console):
    """One panel per sensor: raw reading against floor greyscale under the sensor."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        console.print("[yellow]plots need matplotlib: pip install matplotlib[/]")
        return
    if not curves:
        return
    fig, axes = plt.subplots(1, len(curves), figsize=(3.3 * len(curves), 4.0), sharey=True, squeeze=False)
    grey_line = np.linspace(0, GREY_WHITE, 100)
    for ax, (name, (grey, raw, _, row)) in zip(axes[0], curves.items()):
        ax.plot(grey, raw, ".", ms=2, alpha=0.25, color="#4a6fa5", rasterized=True, label="readings")
        fitted = np.minimum(row["raw_at_grey0"] - row["raw_per_grey"] * grey_line, SURFACE_MAX)
        ax.plot(grey_line, fitted, "-", color="#222222", lw=1.3, label="linear fit")
        ax.axhline(LINE_THRESHOLD, color="#c0392b", ls="--", lw=1, label=f"LINE_THRESHOLD {LINE_THRESHOLD}")
        ax.axhline(row["best_threshold"], color="#27ae60", ls=":", lw=1.5,
                   label="suggested threshold")
        ax.axhline(SURFACE_MAX, color="#999999", ls="-", lw=0.6)
        ax.axvline(row["best_threshold_grey"], color="#27ae60", ls=":", lw=1)
        ax.set_title(f"{name}: raw = {row['raw_at_grey0']:.0f} - {row['raw_per_grey']:.2f} x grey\n"
                     f"threshold {row['best_threshold']:.0f} at grey {row['best_threshold_grey']:.0f}, "
                     f"sigma {row['footprint_sigma_mm']:.1f} mm", fontsize=9)
        ax.set_xlim(-5, GREY_WHITE + 5)
        ax.set_xticks([0, 64, 128, 192, 255])
        ax.grid(alpha=0.3)
        # Greyscale strip along the bottom, so the axis reads as the shade itself.
        strip = ax.inset_axes([0, -0.035, 1, 0.03], transform=ax.transAxes)
        strip.imshow(np.linspace(0, 1, 256)[None, :], cmap="gray", aspect="auto",
                     extent=[0, GREY_WHITE, 0, 1], vmin=0, vmax=1)
        strip.set_xlim(ax.get_xlim())
        strip.axis("off")
    axes[0][0].set_ylabel("raw reading")
    axes[0][0].legend(loc="upper left", fontsize=7, markerscale=4)
    fig.supxlabel("floor greyscale under the sensor's footprint (0 black, 255 white)", fontsize=10)
    fig.suptitle(f"Surface sensor response: {title}", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    console.print(f"saved {path}")


if __name__ == "__main__":
    main()
