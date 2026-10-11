"""Measures the spatial spread (footprint) of the surface sensors DN1-DN5.

Experiment (TODO 3.4): put the robot's spin centre (the midpoint between the
wheels) over a long, straight black/white edge, then spin on the spot for
several turns with the logger recording, e.g. teleop "manual", "vel -25 25".
Each sensor sweeps a circle of radius RADIUS_MM and crosses the edge twice per
turn, so around each crossing its distance from the edge is known from the
odometry heading:

    d = e + r * sin(phi)        (d > 0 on black)

where phi is the sensor's angle around the circle and e is how far the spin
centre sits on the black side of the edge. The readings against d form the
sensor's edge response; fitting a blurred edge (Gaussian footprint, sigma)
gives the spread. The run calibrates itself: crossings of one kind repeat every
true 360 degrees, which corrects the odometry heading scale, and the share of
each turn spent on black gives e. Only the sensor radius must be supplied.

Each crossing is aligned on its own 50 % point, then refined against the pooled
fit, so a constant reading lag or a misplaced spin centre doesn't widen the
result. Usage:

    python tools/sensor_spread.py logs/run_x.csv
    python tools/sensor_spread.py logs/run_x.csv --radius 44 45 45 45 44 --plot
"""
import argparse
import csv
import math
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

SENSORS = ["dn1", "dn2", "dn3", "dn4", "dn5"]
RADIUS_MM = 45.0          # sensor distance from the spin centre (DigitalTwin/RobotSimulation.pde)
WINDOW_MM = 15.0          # distance either side of each crossing used in the fit
LOW_HIGH = (0.3, 0.7)     # hysteresis on the normalised reading for detecting crossings
MIN_CONTRAST = 800        # black minus white, in raw units, needed to analyse a sensor (white ~500, black ~2900)
MAX_FIT_RMS = 0.15        # normalised fit residual above which the result isn't trusted
HEADING_SCALE_RANGE = (0.8, 1.25)   # outside this the "turns" weren't real crossings
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "3.4_spatial_response"


def parse_args():
    parser = argparse.ArgumentParser(description="Surface-sensor footprint from a spin over a straight edge.")
    parser.add_argument("file", help="logger CSV recorded while spinning over the edge")
    parser.add_argument("--radius", type=float, nargs="+", default=[RADIUS_MM], metavar="MM",
                        help=f"sensor distance from the spin centre in mm, one value or one per sensor (default: {RADIUS_MM})")
    parser.add_argument("--window", type=float, default=WINDOW_MM,
                        help=f"mm either side of a crossing to use (default: {WINDOW_MM})")
    parser.add_argument("--start", type=float, help="ignore records before this t_ms")
    parser.add_argument("--end", type=float, help="ignore records after this t_ms")
    parser.add_argument("--out", type=Path, help=f"folder for the results (default: {OUT_DIR}/<file name>)")
    parser.add_argument("--plot", action="store_true", help="also save plots (needs matplotlib)")
    args = parser.parse_args()
    if len(args.radius) not in (1, len(SENSORS)):
        parser.error(f"--radius takes 1 or {len(SENSORS)} values")
    return args


def load(path, start, end):
    """Columns of the logger CSV as float arrays."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    data = {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}
    keep = np.ones(len(rows), bool)
    if start is not None:
        keep &= data["t_ms"] >= start
    if end is not None:
        keep &= data["t_ms"] <= end
    return {key: values[keep] for key, values in data.items()}


def spin_runs(data, min_records=20):
    """Index ranges where the wheels turn in opposite directions (spinning on the spot)."""
    spinning = data["pwm_left"] * data["pwm_right"] < 0
    runs, start = [], None
    for i, s in enumerate(np.append(spinning, False)):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if i - start >= min_records:
                runs.append((start, i))
            start = None
    return runs


def erf(x):
    """Error function (Abramowitz and Stegun 7.1.26, error below 1.5e-7)."""
    sign = np.sign(x)
    x = np.abs(x)
    t = 1.0 / (1.0 + 0.3275911 * x)
    poly = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))))
    return sign * (1.0 - poly * np.exp(-x * x))


def edge_model(d, centre, sigma):
    """A straight edge seen through a Gaussian footprint: 0 on white, 1 on black."""
    return 0.5 * (1.0 + erf((d - centre) / (sigma * math.sqrt(2.0))))


def fit_edge(d, n):
    """Fits n = low + (high - low) * edge_model(d, centre, sigma) by grid search.

    For each (centre, sigma) on the grid, low and high follow from linear least
    squares, so only two parameters are searched. Two passes, coarse then fine.
    """
    def search(centres, sigmas):
        best = None
        for sigma in sigmas:
            for centre in centres:
                m = edge_model(d, centre, sigma)
                design = np.column_stack([1.0 - m, m])
                (low, high), *_ = np.linalg.lstsq(design, n, rcond=None)
                sse = float(np.sum((design @ [low, high] - n) ** 2))
                if best is None or sse < best[0]:
                    best = (sse, centre, sigma, low, high)
        return best

    _, c, s, *_ = search(np.linspace(-5, 5, 41), np.geomspace(0.2, 20, 40))
    sse, c, s, low, high = search(np.linspace(c - 0.3, c + 0.3, 31),
                                  np.linspace(max(0.05, s * 0.85), s * 1.15, 31))
    return {"centre": c, "sigma": s, "low": low, "high": high, "rms": math.sqrt(sse / len(d))}


def find_crossings(theta, n):
    """50 % crossings of the normalised reading, with hysteresis against noise.

    Returns (index after the crossing, interpolated theta, +1 rising to black
    or -1 falling to white).
    """
    low, high = LOW_HIGH
    crossings, state = [], None
    for i, v in enumerate(n):
        new = 1 if v > high else -1 if v < low else state
        if state is not None and new != state:
            # Walk back to where the reading passed 0.5 and interpolate there.
            j = i
            while j > 0 and (n[j - 1] - 0.5) * new > 0:
                j -= 1
            j = max(j, 1)
            a, b = n[j - 1], n[j]
            frac = 0.5 if a == b else (0.5 - a) / (b - a)
            crossings.append((j, theta[j - 1] + frac * (theta[j] - theta[j - 1]), new))
        state = new
    return crossings


def heading_scale(all_crossings):
    """Ratio of true rotation to odometry rotation, from same-kind crossings 360 degrees apart."""
    steps = []
    for crossings in all_crossings:
        for kind in (1, -1):
            thetas = [t for _, t, k in crossings if k == kind]
            steps += [abs(b - a) for a, b in zip(thetas, thetas[1:])]
    if not steps:
        return 1.0, 0
    return 2 * math.pi / float(np.median(steps)), len(steps)


def black_fraction(crossings, scale):
    """Share of a turn the sensor spends on black, from rising-then-falling pairs."""
    arcs = [abs(t2 - t1) * scale for (_, t1, k1), (_, t2, k2) in zip(crossings, crossings[1:])
            if k1 == 1 and k2 == -1]
    if not arcs:
        return None
    return float(np.median(arcs)) / (2 * math.pi)


def crossing_profiles(theta, n, raw, crossings, scale, offset_ratio, radius, window, direction):
    """Distance from the edge (mm) for samples around each crossing."""
    profiles = []
    s = max(-0.99, min(0.99, offset_ratio))
    for idx, (i, theta50, kind) in enumerate(crossings):
        # Don't reach past the neighbouring crossings.
        lo = (crossings[idx - 1][0] + i) // 2 if idx > 0 else 0
        hi = (i + crossings[idx + 1][0]) // 2 if idx + 1 < len(crossings) else len(n)
        # Angle around the sensor circle at the crossing: d = 0 where sin(phi) = -e/r.
        phi50 = -math.asin(s) if kind == 1 else math.pi + math.asin(s)
        delta = (theta[lo:hi] - theta50) * scale * direction
        d = radius * (np.sin(phi50 + delta) - math.sin(phi50))
        near = np.abs(d) <= window
        if near.sum() >= 3:
            profiles.append({"id": idx, "kind": kind, "d": d[near], "n": n[lo:hi][near],
                             "raw": raw[lo:hi][near]})
    return profiles


def refine_alignment(profiles, fit, rounds=3):
    """Shifts each crossing to best match the pooled fit, then refits."""
    shifts = np.linspace(-3, 3, 121)
    for _ in range(rounds):
        for p in profiles:
            errors = [np.sum((fit["low"] + (fit["high"] - fit["low"])
                              * edge_model(p["d"] + s, fit["centre"], fit["sigma"]) - p["n"]) ** 2)
                      for s in shifts]
            p["d"] = p["d"] + shifts[int(np.argmin(errors))]
        d = np.concatenate([p["d"] for p in profiles])
        n = np.concatenate([p["n"] for p in profiles])
        fit = fit_edge(d, n)
    return fit


def empirical_rise(d, n, fit, bin_mm=0.5):
    """10-90 % rise distance from the binned median profile, without assuming a shape."""
    span = fit["high"] - fit["low"]
    level = (n - fit["low"]) / span
    edges = np.arange(d.min(), d.max() + bin_mm, bin_mm)
    centres, medians = [], []
    for a, b in zip(edges, edges[1:]):
        inside = (d >= a) & (d < b)
        if inside.sum() >= 3:
            centres.append((a + b) / 2)
            medians.append(float(np.median(level[inside])))
    centres, medians = np.array(centres), np.maximum.accumulate(np.array(medians))
    if len(centres) < 4 or medians[0] > 0.1 or medians[-1] < 0.9:
        return None
    return float(np.interp(0.9, medians, centres) - np.interp(0.1, medians, centres))


def analyse_sensor(data, runs, name, radius):
    """Crossings and pooled edge fit for one sensor, over all spin runs."""
    raw_all = data[name]
    spin_idx = np.concatenate([np.arange(a, b) for a, b in runs])
    white, black = np.percentile(raw_all[spin_idx], [5, 95])
    result = {"sensor": name, "white": white, "black": black, "radius": radius}
    if black - white < MIN_CONTRAST:
        result["error"] = f"contrast {black - white:.0f} too low: did it cross the edge?"
        return result, []

    per_run = []
    for a, b in runs:
        theta, raw = data["theta_rad"][a:b], raw_all[a:b]
        n = (raw - white) / (black - white)
        direction = 1.0 if theta[-1] >= theta[0] else -1.0
        per_run.append((theta, n, raw, find_crossings(theta, n), direction))
    return result, per_run


def main():
    args = parse_args()
    console = Console()
    data = load(args.file, args.start, args.end)
    runs = spin_runs(data)
    if not runs:
        console.print("No spinning found (left and right PWM of opposite sign). Check --start/--end.")
        return
    radii = args.radius * len(SENSORS) if len(args.radius) == 1 else args.radius
    turns = sum(abs(data["theta_rad"][b - 1] - data["theta_rad"][a]) for a, b in runs) / (2 * math.pi)
    console.print(f"{len(runs)} spin run(s), {sum(b - a for a, b in runs)} records, "
                  f"about {turns:.1f} turns by odometry")

    sensors = [analyse_sensor(data, runs, name, r) for name, r in zip(SENSORS, radii)]

    # One heading scale for the whole run, from every sensor's crossings.
    # Sensors without enough contrast have no per-run data, so they don't count.
    scale, n_steps = heading_scale([c for _, per_run in sensors for *_, c, _ in per_run])
    if n_steps and not HEADING_SCALE_RANGE[0] <= scale <= HEADING_SCALE_RANGE[1]:
        console.print(f"[yellow]heading scale {scale:.3f} is implausible (noisy crossings?); using 1.0[/]")
        scale = 1.0
    elif n_steps:
        console.print(f"odometry heading scale: {scale:.4f} (true / odometry, from {n_steps} full turns)")
    else:
        console.print("[yellow]fewer than one full turn per sensor: heading scale not calibrated (1.0)[/]")

    results, all_profiles = [], {}
    for (result, per_run), radius in zip(sensors, radii):
        results.append(result)
        if "error" in result:
            continue
        fractions = [f for *_, c, _ in per_run if (f := black_fraction(c, scale)) is not None]
        offset_ratio = math.sin(math.pi * (float(np.median(fractions)) - 0.5)) if fractions else 0.0
        result["offset_mm"] = offset_ratio * radius

        profiles = []
        for theta, n, raw, crossings, direction in per_run:
            profiles += crossing_profiles(theta, n, raw, crossings, scale, offset_ratio,
                                          radius, args.window, direction)
        if len(profiles) < 2:
            result["error"] = f"only {len(profiles)} crossing(s) found"
            continue
        d = np.concatenate([p["d"] for p in profiles])
        n = np.concatenate([p["n"] for p in profiles])
        fit = refine_alignment(profiles, fit_edge(d, n))
        d = np.concatenate([p["d"] for p in profiles])
        n = np.concatenate([p["n"] for p in profiles])

        if fit["rms"] > MAX_FIT_RMS:
            result["error"] = f"readings don't look like an edge (fit rms {fit['rms']:.2f})"
            continue
        spacing = float(np.median(np.concatenate([np.abs(np.diff(p["d"])) for p in profiles])))
        result.update(fit)
        result.update({"crossings": len(profiles), "points": len(d), "spacing": spacing,
                       "rise_10_90": empirical_rise(d, n, fit)})
        all_profiles[result["sensor"]] = profiles

    table = Table(title=f"Surface sensor spread (mm): {Path(args.file).name}")
    for column in ["sensor", "white", "black", "cross", "step", "sigma", "FWHM",
                   "10-90 fit", "10-90 data", "rms", "offset"]:
        table.add_column(column, justify="right")
    for r in results:
        if "error" in r:
            table.add_row(r["sensor"], f"{r['white']:.0f}", f"{r['black']:.0f}", "[red]-[/]",
                          *[""] * 7)
            continue
        rise = "-" if r["rise_10_90"] is None else f"{r['rise_10_90']:.2f}"
        table.add_row(r["sensor"], f"{r['white']:.0f}", f"{r['black']:.0f}", str(r["crossings"]),
                      f"{r['spacing']:.2f}", f"{r['sigma']:.2f}", f"{2.3548 * r['sigma']:.2f}",
                      f"{2.5631 * r['sigma']:.2f}", rise, f"{r['rms']:.3f}", f"{r['offset_mm']:+.1f}")
    console.print(table)
    console.print("cross: edge crossings used. step: distance between samples. sigma: Gaussian footprint "
                  "fitted to the edge response. FWHM: footprint width at half height. 10-90: distance over "
                  "which the reading goes from 10 % to 90 % of white-to-black, from the fit and from the "
                  "binned data. rms: fit residual (normalised). offset: spin centre's distance onto black.")
    for r in results:
        if "error" in r:
            console.print(f"[red]{r['sensor']}: {r['error']}[/]")
        if "spacing" in r and r["spacing"] > r["sigma"]:
            console.print(f"[yellow]{r['sensor']}: samples are {r['spacing']:.1f} mm apart, coarse for a "
                          f"{r['sigma']:.1f} mm spread. Spin slower or lower TELEMETRY_INTERVAL_MS.[/]")

    out = args.out or OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "summary.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sensor", "radius_mm", "white", "black", "crossings", "points", "sample_step_mm",
                         "sigma_mm", "fwhm_mm", "rise_10_90_fit_mm", "rise_10_90_data_mm", "fit_rms",
                         "centre_offset_mm", "heading_scale", "error"])
        for r in results:
            ok = "error" not in r
            writer.writerow([r["sensor"], r["radius"], f"{r['white']:.0f}", f"{r['black']:.0f}",
                             r.get("crossings", ""), r.get("points", ""),
                             f"{r['spacing']:.3f}" if ok else "", f"{r['sigma']:.3f}" if ok else "",
                             f"{2.3548 * r['sigma']:.3f}" if ok else "", f"{2.5631 * r['sigma']:.3f}" if ok else "",
                             "" if not ok or r["rise_10_90"] is None else f"{r['rise_10_90']:.3f}",
                             f"{r['rms']:.4f}" if ok else "", f"{r['offset_mm']:.2f}" if ok else "",
                             f"{scale:.4f}", r.get("error", "")])
    with open(out / "profiles.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sensor", "crossing", "kind", "d_mm", "normalised", "raw"])
        for name, profiles in all_profiles.items():
            for p in profiles:
                kind = "to_black" if p["kind"] == 1 else "to_white"
                for d, n, raw in zip(p["d"], p["n"], p["raw"]):
                    writer.writerow([name, p["id"], kind, f"{d:.3f}", f"{n:.4f}", f"{raw:.0f}"])
    console.print(f"saved {out / 'summary.csv'} and {out / 'profiles.csv'}")

    if args.plot:
        plot(results, all_profiles, out / "spread.png", console)


def plot(results, all_profiles, path, console):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        console.print("[yellow]--plot needs matplotlib: pip install matplotlib[/]")
        return
    fitted = [r for r in results if "sigma" in r]
    if not fitted:
        return
    fig, axes = plt.subplots(1, len(fitted), figsize=(3.2 * len(fitted), 3.4), sharey=True, squeeze=False)
    for ax, r in zip(axes[0], fitted):
        for p in all_profiles[r["sensor"]]:
            ax.plot(p["d"], p["n"], ".", ms=2, alpha=0.4, color="C0" if p["kind"] == 1 else "C1")
        d = np.linspace(-15, 15, 300)
        ax.plot(d, r["low"] + (r["high"] - r["low"]) * edge_model(d, r["centre"], r["sigma"]), "k-", lw=1.2)
        ax.set_title(f"{r['sensor']}: FWHM {2.3548 * r['sigma']:.1f} mm")
        ax.set_xlabel("distance onto black (mm)")
        ax.grid(alpha=0.3)
    axes[0][0].set_ylabel("normalised reading (0 white, 1 black)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    console.print(f"saved {path}")


if __name__ == "__main__":
    main()
