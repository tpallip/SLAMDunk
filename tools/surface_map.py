"""Reconstructs the floor pattern from a logged run of the surface sensors.

Each record's encoder counts give the robot pose; each sensor's mounting
position then places its reading on the floor. Readings are averaged into a
grid of square cells and saved as an image. The pose is recomputed from the
encoder columns rather than taken from x/y/theta, so the wheel geometry can
be calibrated here (--separation, --wheel-radius) without reflashing. Any
motion works: the robot doesn't need to turn about its centre.

    python tools/surface_map.py logs/run_x.csv
    python tools/surface_map.py logs/run_x.csv --cell 0.5 --separation 92.3 --plot

If the separation is unknown, --fit-separation picks the value at which
readings of the same spot from different passes agree best (a wrong
separation twists later passes against earlier ones). It needs passes that
overlap. The heading scale tools/sensor_spread.py prints also converts to a
separation: --separation (90 / scale).

The five sensors sit on one 45 mm circle at different angles, so on a spin
they sweep the same ring and their samples interleave, which sharpens edges
beyond one sensor's sample spacing, but only if --angles matches the robot:
each degree of error shifts that sensor's samples about 0.8 mm along the
ring. Readings are interpolated along each sensor's track (--no-interp to
turn off), and --sharp adds map_sharp.png with each cell set black or white
at the 50 % level.

A spin on the spot images only that thin ring around the wheel midpoint. To image an area, move between spins or
drive in lanes. --fill spreads each reading over nearby cells to close the
gaps between tracks, at the cost of sharpness.
"""
import argparse
import csv
import math
import struct
import zlib
from pathlib import Path

import numpy as np
from rich.console import Console

SENSORS = ["dn1", "dn2", "dn3", "dn4", "dn5"]
SENSOR_RADIUS_MM = 45.0                     # DigitalTwin/RobotSimulation.pde
SENSOR_ANGLES_DEG = [45, 14, 0, -14, -45]   # dn1 (left) to dn5 (right); 0 is straight ahead
WHEEL_RADIUS_MM = 16.0                      # OdometryModel_c defaults
SEPARATION_MM = 90.0
COUNTS_PER_REV = 358.3
CELL_MM = 1.0
MIN_CONTRAST = 800        # black minus white needed to set a sensor's own black level
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "surface_map"


def parse_args():
    parser = argparse.ArgumentParser(description="Rebuild the floor pattern from logged surface readings.")
    parser.add_argument("file", help="logger CSV")
    parser.add_argument("--cell", type=float, default=CELL_MM, help=f"grid cell size in mm (default: {CELL_MM})")
    parser.add_argument("--separation", type=float, default=SEPARATION_MM,
                        help=f"wheel separation in mm (default: {SEPARATION_MM})")
    parser.add_argument("--wheel-radius", type=float, default=WHEEL_RADIUS_MM,
                        help=f"wheel radius in mm (default: {WHEEL_RADIUS_MM})")
    parser.add_argument("--counts-per-rev", type=float, default=COUNTS_PER_REV,
                        help=f"encoder counts per wheel turn (default: {COUNTS_PER_REV})")
    parser.add_argument("--radius", type=float, nargs="+", default=[SENSOR_RADIUS_MM], metavar="MM",
                        help=f"sensor distance from the wheel midpoint, one or five values (default: {SENSOR_RADIUS_MM})")
    parser.add_argument("--angles", type=float, nargs=5, default=SENSOR_ANGLES_DEG, metavar="DEG",
                        help=f"sensor angles dn1..dn5 in degrees (default: {SENSOR_ANGLES_DEG})")
    parser.add_argument("--fit-separation", action="store_true",
                        help="choose the wheel separation that makes overlapping passes agree best")
    parser.add_argument("--no-interp", dest="interp", action="store_false",
                        help="don't interpolate readings along each sensor's track")
    parser.add_argument("--sharp", action="store_true",
                        help="also save map_sharp.png, black or white at the 50 %% level")
    parser.add_argument("--fill", type=float, default=0.0, metavar="MM",
                        help="spread each reading over cells within this distance (default: 0, off)")
    parser.add_argument("--start", type=float, help="ignore records before this t_ms")
    parser.add_argument("--end", type=float, help="ignore records after this t_ms")
    parser.add_argument("--out", type=Path, help=f"folder for the results (default: {OUT_DIR}/<file name>)")
    parser.add_argument("--plot", action="store_true", help="also save a plot with mm axes (needs matplotlib)")
    args = parser.parse_args()
    if len(args.radius) not in (1, 5):
        parser.error("--radius takes 1 or 5 values")
    return args


def load(path, start, end):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    data = {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}
    keep = np.ones(len(rows), bool)
    if start is not None:
        keep &= data["t_ms"] >= start
    if end is not None:
        keep &= data["t_ms"] <= end
    return {key: values[keep] for key, values in data.items()}


def odometry(left_counts, right_counts, wheel_radius, separation, counts_per_rev):
    """Pose at every record from encoder counts, starting at (0, 0, 0).

    Midpoint integration: each step's distance is applied along the heading
    halfway through the step, which follows arcs better than Euler.
    """
    mm_per_count = 2 * math.pi * wheel_radius / counts_per_rev
    dl = np.diff(left_counts, prepend=left_counts[0]) * mm_per_count
    dr = np.diff(right_counts, prepend=right_counts[0]) * mm_per_count
    dtheta = (dr - dl) / separation
    theta = np.cumsum(dtheta)
    mid = theta - dtheta / 2
    distance = (dl + dr) / 2
    x = np.cumsum(distance * np.cos(mid))
    y = np.cumsum(distance * np.sin(mid))
    return x, y, theta


def darkness(data, console):
    """Each sensor's readings scaled to 0 (white) .. 1 (black).

    Sensors differ in their white and black levels, so each uses its own. A
    sensor that never saw black borrows the typical span of the others.
    """
    lows, spans = [], []
    for name in SENSORS:
        low, high = np.percentile(data[name], [2, 98])
        lows.append(low)
        spans.append(high - low)
    good = [s for s in spans if s >= MIN_CONTRAST]
    fallback = float(np.median(good)) if good else max(spans)
    levels = []
    for name, low, span in zip(SENSORS, lows, spans):
        if span < MIN_CONTRAST:
            console.print(f"[yellow]{name} saw little contrast ({span:.0f}); using a span of {fallback:.0f}[/]")
            span = fallback
        levels.append(np.clip((data[name] - low) / span, 0, 1))
    return levels


def floor_points(data, levels, separation, args, radii, angles, step_mm=None):
    """Floor position and darkness of every reading: robot pose plus sensor mounting offset.

    With step_mm, pose and readings are interpolated linearly between records so
    each sensor moves at most about step_mm between points.
    """
    x, y, theta = odometry(data["enc_left"], data["enc_right"], args.wheel_radius,
                           separation, args.counts_per_rev)
    if step_mm:
        # Don't bridge telemetry gaps (dropped WiFi records): the reading in the
        # missing stretch is unknown, so leave it empty rather than draw a ramp.
        dt = np.diff(data["t_ms"])
        gap = dt > 3 * np.median(dt)
        # Sensor movement between records: turning plus driving.
        move = np.abs(np.diff(theta)) * max(radii) + np.hypot(np.diff(x), np.diff(y))
        typical = np.percentile(move[~gap], 99) if (~gap).any() else move.max()
        factor = int(min(50, max(1, math.ceil(typical / step_mm))))
        where = np.linspace(0, len(x) - 1, (len(x) - 1) * factor + 1)
        inside_gap = gap[np.minimum(where.astype(int), len(gap) - 1)] & (where % 1 != 0)
        where = where[~inside_gap]
        index = np.arange(len(x))
        x, y, theta = (np.interp(where, index, v) for v in (x, y, theta))
        levels = [np.interp(where, index, level) for level in levels]
    px, py, pv = [], [], []
    for radius, angle, level in zip(radii, angles, levels):
        lx, ly = radius * math.cos(math.radians(angle)), radius * math.sin(math.radians(angle))
        px.append(x + lx * np.cos(theta) - ly * np.sin(theta))
        py.append(y + lx * np.sin(theta) + ly * np.cos(theta))
        pv.append(level)
    return (x, y, theta), np.concatenate(px), np.concatenate(py), np.concatenate(pv)


def disagreement(px, py, pv, cell=2.0):
    """Pooled variance of readings that land in the same cell: low when passes line up."""
    ix = ((px - px.min()) / cell).astype(np.int64)
    iy = ((py - py.min()) / cell).astype(np.int64)
    key = iy * (ix.max() + 1) + ix
    order = np.argsort(key, kind="stable")
    _, start, count = np.unique(key[order], return_index=True, return_counts=True)
    v = pv[order]
    sums = np.add.reduceat(v, start)
    squares = np.add.reduceat(v * v, start)
    shared = count > 1
    if not shared.any():
        return float("inf")
    return float((squares - sums ** 2 / count)[shared].sum() / (count[shared] - 1).sum())


def fit_separation(data, levels, args, radii, angles):
    """Wheel separation (mm) that minimises disagreement, coarse 1 mm then fine 0.1 mm."""
    def score(sep):
        _, px, py, pv = floor_points(data, levels, sep, args, radii, angles)
        return disagreement(px, py, pv)
    coarse = min(np.arange(70.0, 120.0, 1.0), key=score)
    return float(min(np.arange(coarse - 1, coarse + 1.01, 0.1), key=score))


def write_png(path, image):
    """Greyscale PNG from a 2D array of 0..255 (no extra libraries needed)."""
    height, width = image.shape
    raw = b"".join(b"\0" + row.tobytes() for row in image.astype(np.uint8))
    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload
                + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def main():
    args = parse_args()
    console = Console()
    data = load(args.file, args.start, args.end)
    radii = args.radius * 5 if len(args.radius) == 1 else args.radius

    levels = darkness(data, console)
    separation, angles = args.separation, list(args.angles)
    if args.fit_separation:
        separation = fit_separation(data, levels, args, radii, angles)
    if args.fit_separation:
        console.print(f"fitted wheel separation: {separation:.1f} mm "
                      f"(odometry heading scale {SEPARATION_MM / separation:.4f} vs the {SEPARATION_MM:g} mm default)")
    _, px, py, pv = floor_points(data, levels, separation, args, radii, angles)
    console.print(f"pass disagreement: {disagreement(px, py, pv):.4f} (lower = passes line up better)")
    x, y, theta = odometry(data["enc_left"], data["enc_right"], args.wheel_radius,
                           separation, args.counts_per_rev)
    if args.interp:
        _, px, py, pv = floor_points(data, levels, separation, args, radii, angles,
                                     step_mm=args.cell / 2)

    # Average the readings in each cell; cells nobody passed over stay empty.
    # With --fill, each reading also counts for cells within that distance,
    # weighted down with distance.
    reach = int(math.ceil(args.fill / args.cell))
    x0, y0 = px.min() - reach * args.cell, py.min() - reach * args.cell
    cols = int((px.max() - x0) / args.cell) + reach + 1
    rows = int((py.max() - y0) / args.cell) + reach + 1
    ix = ((px - x0) / args.cell).astype(int)
    iy = ((py - y0) / args.cell).astype(int)
    total = np.zeros((rows, cols))
    count = np.zeros((rows, cols))
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            dist = math.hypot(dx, dy) * args.cell
            if dist > args.fill and (dx or dy):
                continue
            weight = 1.0 if args.fill == 0 else 1.0 - dist / (args.fill + args.cell)
            np.add.at(total, (iy + dy, ix + dx), weight * pv)
            np.add.at(count, (iy + dy, ix + dx), weight)
    with np.errstate(invalid="ignore"):
        grid = total / count

    turned = (theta.max() - theta.min()) / (2 * math.pi)
    covered = np.count_nonzero(count) * args.cell ** 2
    console.print(f"{len(theta)} records; robot moved {math.hypot(x[-1], y[-1]):.0f} mm start to end, "
                  f"heading range {turned:.2f} turns; map {cols * args.cell:.0f} x {rows * args.cell:.0f} mm, "
                  f"{covered / 100:.0f} cm2 covered")

    out = args.out or OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)
    # Image rows run downwards, so flip to keep +y up. White floor = white pixel;
    # empty cells are mid grey.
    image = np.where(np.isnan(grid), 128, 255 * (1 - np.nan_to_num(grid)))[::-1]
    write_png(out / "map.png", image)
    np.savez_compressed(out / "map.npz", darkness=grid, count=count, x0_mm=x0, y0_mm=y0,
                        cell_mm=args.cell, separation_mm=separation, angles_deg=angles)
    console.print(f"saved {out / 'map.png'} ({cols} x {rows} px, {args.cell} mm per px) and map.npz")
    if args.sharp:
        sharp = np.where(np.isnan(grid), 128, np.where(np.nan_to_num(grid) >= 0.5, 0, 255))[::-1]
        write_png(out / "map_sharp.png", sharp)
        console.print(f"saved {out / 'map_sharp.png'}")

    if args.plot:
        plot(grid, x0, y0, args.cell, x, y, out / "map_plot.png", console)


def plot(grid, x0, y0, cell, x, y, path, console):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        console.print("[yellow]--plot needs matplotlib: pip install matplotlib[/]")
        return
    fig, ax = plt.subplots(figsize=(7, 7))
    extent = [x0, x0 + grid.shape[1] * cell, y0, y0 + grid.shape[0] * cell]
    cmap = plt.get_cmap("gray_r").copy()
    cmap.set_bad("#c8d4e0")
    ax.imshow(np.ma.masked_invalid(grid), origin="lower", extent=extent, cmap=cmap, vmin=0, vmax=1)
    ax.plot(x, y, color="C3", lw=0.8, label="wheel midpoint")
    ax.plot(x[0], y[0], "o", color="C3", ms=4)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm, start heading)")
    ax.set_ylabel("y (mm)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    console.print(f"saved {path}")


if __name__ == "__main__":
    main()
