"""Twist command grid: hold "cmd <v> <omega>" over a grid, report achieved vs commanded.

Plan Phase 4 (MOTOR_PROFILING.local.md): achieved (v, omega) against the
command over the working range, checked against the Decision 7 thresholds,
and the motion noise model for the state estimator.

Run (logger recording, teleop on, robot on the floor):

    python tools/twist_test.py run --set straight --repeats 5   # needs about 1.1 m of floor
    python tools/twist_test.py run --set spin --repeats 5
    python tools/twist_test.py run --set arc --repeats 5        # wanders up to about 0.5 m
    python tools/twist_test.py run --points "100 0" "0 90" --repeats 3

or, on a 20 x 20 cm patch (the robot's centre stays inside it):

    python tools/twist_test.py run --set straight20 --repeats 5
    python tools/twist_test.py run --set spin20 --repeats 5
    python tools/twist_test.py run --set arc20 --repeats 5      # stops once to have the robot turned round

then, on each CSV the logger saved (same --set/--points and --repeats):

    python tools/twist_test.py report logs/<run>.csv --set spin --repeats 5
    python tools/twist_test.py model bench/twist_test/<run1> bench/twist_test/<run2> ...

"run" holds each point for --hold seconds ("cmd" resent every 0.2 s), then
sends "cmd 0 0", which slows down to a stop at accel_mm_s2 with the heading
loop on, waits --gap seconds and sends "stop". Points are run in order, then
the whole list again for each repeat, so a slow drift (battery) spreads over
all of them. Straight points come in +v / -v pairs and arcs in +omega / -omega
pairs so the robot doesn't wander far.

The 20 x 20 cm sets ("run" prints where to put the robot):
- straight20: start mark 15 mm in from the middle of one edge, facing
  across. Each hold runs about 165 mm (hold cut to 165/|v| s) and its
  pair runs back. Up to 100 mm/s: faster has no steady part in 165 mm.
- spin20: the spin set, robot in the middle.
- arc20: arcs of radius 69-82 mm, so a whole circle stays inside.
  Start mark 15 mm in from the middle of one edge, facing along it with
  the patch on the right: the right-hand arcs, each (v, -w) then
  (-v, +w), which drives back along the same circle. Then the run stops
  for the robot to be turned round on the mark (patch on the left) for
  the left-hand arcs. Repeats of one side run together.
For these sets the steady part starts once the ramp is done
(fastest wheel / 300 mm/s^2 + 0.35 s) instead of at --settle.

"report" finds each hold in the CSV (rows where either wheel has PWM) and
gives, over the steady part (from --settle s after the start to --hold s,
before the slow-down):
mean v and omega from the encoders, and their sd over 100 ms and 500 ms
windows. Per point it checks Decision 7: the mean over the repeats within
the larger of 2 % and 2 mm/s (2 deg/s) of the command, and every repeat
within the larger of 5 % and 4. It also lists each hold's end pose from
the robot's odometry, coast included, in the hold's starting frame, to
compare with the grid; with --truth (CSV: n, dist_mm, turn_deg, either
may be blank) it checks those against odometry (mean within 2 %, each
within 3 %). Results go to bench/twist_test/<csv name>/.

"model" fits the noise model on the holds of one or more report folders,
leaving out the held-back points (marked "held" in the sets), then predicts
the held-back points: sigma_v^2 = a1 v^2 + a2 w^2 and
sigma_w^2 = a3 v^2 + a4 w^2 (Thrun's velocity motion model; v in mm/s,
w in rad/s), for 100 ms and 500 ms windows.
"""
import argparse
import csv
import json
import math
import socket
import time
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

HOST, PORT = "127.0.0.1", 9100     # logger's teleop port (tools/teleop.py)
HOLD_S = 5.0
GAP_S = 1.5
RESEND_S = 0.2
SETTLE_S = 1.2                      # accel 300 mm/s^2 reaches 220 mm/s in 0.73 s, plus the wheel lag
END_MARGIN_S = 0.1                  # steady part ends this long before --hold, ahead of the slow-down
WINDOWS_S = (0.1, 0.5)
SEPARATION_MM = 84.43
RADIUS_MM = 16.26
COUNTS_PER_REV = 358.3
MIN_WHEEL_MM_S, MAX_WHEEL_MM_S = 50.0, 250.0   # Decision 7 working range per wheel
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "twist_test"

# (v mm/s, omega deg/s, held back from the noise model fit)
SETS = {
    "straight": [(v, 0, False) for s in (50, 60, 100, 150, 200) for v in (s, -s)] + [(80, 0, True), (-80, 0, True)],
    "spin": [(0, w, False) for s in (70, 90, 180, 300) for w in (s, -s)] + [(0, 120, True), (0, -120, True)],
    "arc": [(v, w, False) for v, s in ((100, 60), (150, 60), (150, 120), (200, 60), (50, 200)) for w in (s, -s)]
           + [(120, 45, True), (120, -45, True)],
}
# 20 x 20 cm patch. PAUSE splits a set into sides run one after the other.
PAUSE = None
STRAIGHT_RUN_MM = 165               # 200 less 15 mm start inset and 20 mm margin
ACCEL_MM_S2 = 300                   # accel_mm_s2 in lokum.json
LAG_S = 0.35                        # wheel speed reaching the ramp's end
SMALL_ARCS = [(110, 77, False), (130, 92, False), (150, 105, False), (150, 125, False), (120, 90, True)]
SETS["straight20"] = [(v, 0, False) for s in (50, 60, 100) for v in (s, -s)] + [(80, 0, True), (-80, 0, True)]
SETS["spin20"] = SETS["spin"]
SETS["arc20"] = ([p for v, w, held in SMALL_ARCS for p in ((v, -w, held), (-v, w, held))] + [PAUSE]
                 + [p for v, w, held in SMALL_ARCS for p in ((v, w, held), (-v, -w, held))])
SETUP = {
    "straight20": "Start mark 15 mm in from the middle of one edge, robot facing across the patch.",
    "spin20": "Robot in the middle of the patch.",
    "arc20": "Start mark 15 mm in from the middle of one edge, robot facing along the edge, patch on its right.",
}


def parse_args():
    parser = argparse.ArgumentParser(description="Twist command (v, omega) grid test.")
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("run", "report"):
        p = sub.add_parser(name)
        group = p.add_mutually_exclusive_group(required=True)
        group.add_argument("--set", choices=SETS, help="a predefined grid")
        group.add_argument("--points", nargs="+", help='"v omega" pairs, e.g. "100 0" "0 90" (none held back)')
        p.add_argument("--repeats", type=int, default=1)
        p.add_argument("--hold", type=float, default=HOLD_S,
                       help=f"seconds per point (default: {HOLD_S:g}; straight20 cuts it to fit)")
        p.add_argument("--separation", type=float, default=SEPARATION_MM,
                       help=f"wheel_separation_mm the robot used (default: {SEPARATION_MM:g})")
    run = sub.choices["run"]
    run.add_argument("--gap", type=float, default=GAP_S, help=f"stopped seconds between points (default: {GAP_S:g})")
    run.add_argument("--port", type=int, default=PORT, help=f"logger's teleop port (default: {PORT})")
    report = sub.choices["report"]
    report.add_argument("file", help="logger CSV recorded during the run")
    report.add_argument("--settle", type=float,
                        help=f"seconds skipped at each hold's start (default: {SETTLE_S:g}; "
                             "on the 20 x 20 cm sets, the ramp plus the wheels' lag)")
    report.add_argument("--radius", type=float, nargs=2, default=[RADIUS_MM, RADIUS_MM], metavar=("LEFT", "RIGHT"),
                        help=f"wheel radii the robot used, mm (default: {RADIUS_MM:g} {RADIUS_MM:g})")
    report.add_argument("--truth", type=Path, help="grid readings: CSV with n, dist_mm, turn_deg")
    report.add_argument("--out", type=Path, help=f"folder for the results (default: {OUT_DIR}/<file name>)")
    model = sub.add_parser("model")
    model.add_argument("folders", nargs="+", type=Path, help="report folders (with holds.csv)")
    model.add_argument("--out", type=Path, default=OUT_DIR / "noise_model.json")
    return parser.parse_args()


def points_of(args):
    """[(v, omega, held)] for one pass of the grid."""
    if args.set:
        return [p for p in SETS[args.set] if p is not PAUSE]
    out = []
    for p in args.points:
        v, w = (float(x) for x in p.split())
        out.append((v, w, False))
    return out


def schedule(args):
    """Every hold in run order as (v, omega, held), with PAUSE where the robot is turned round."""
    sides, side = [], []
    for p in (SETS[args.set] if args.set else points_of(args)):
        if p is PAUSE:
            sides.append(side)
            side = []
        else:
            side.append(p)
    sides.append(side)
    out = []
    for side in sides:
        out += [PAUSE] if out else []
        out += [p for _ in range(args.repeats) for p in side]
    return out


def small(args):
    return bool(args.set) and args.set.endswith("20")


def hold_of(args, v, w):
    """Seconds this point is held: straight20 cuts it so the run fits the patch."""
    if args.set == "straight20" and v and not w:
        return min(args.hold, STRAIGHT_RUN_MM / abs(v))
    return args.hold


def settle_of(args, v, w, separation):
    """Seconds skipped at the hold's start: the ramp plus the wheels' lag on the small sets."""
    if args.settle is not None or not small(args):
        return SETTLE_S if args.settle is None else args.settle
    return max(abs(s) for s in wheel_targets(v, w, separation)) / ACCEL_MM_S2 + LAG_S


def wheel_targets(v, w, separation):
    half = math.radians(w) * separation / 2
    return v - half, v + half


def in_range(v, w, separation):
    """Whether each wheel target is 0 or within the Decision 7 working range."""
    return all(abs(s) < 1e-9 or MIN_WHEEL_MM_S <= abs(s) <= MAX_WHEEL_MM_S
               for s in wheel_targets(v, w, separation))


# ---------------------------------------------------------------- run

def run(args, console):
    points = points_of(args)
    outside = [f"{v:g} {w:g}" for v, w, _ in points if not in_range(v, w, args.separation)]
    if outside:
        console.print(f"[yellow]outside the working range (a wheel target in (0, 50) or over 250 mm/s): "
                      f"{', '.join(outside)}[/]")
    try:
        sock = socket.create_connection((HOST, args.port), timeout=3)
    except OSError as e:
        console.print(f"can't reach the logger on {HOST}:{args.port} ({e}). Is logger.py running with teleop?")
        return
    greeting = sock.recv(256).decode("utf-8", errors="replace").strip()
    if not greeting.startswith("connected"):
        console.print(greeting or "logger closed the connection")
        return
    console.print(greeting)

    def send(command):
        sock.sendall(command.encode() + b"\n")

    order = schedule(args)
    holds = [p for p in order if p is not PAUSE]
    seconds = sum(hold_of(args, v, w) + args.gap for v, w, _ in holds)
    if args.set in SETUP:
        console.print(f"[bold]{SETUP[args.set]}[/]")
    console.print(f"{len(holds)} holds, about {seconds / 60:.1f} min. Ctrl+C stops the robot.")
    try:
        send("manual")
        time.sleep(args.gap)
        i = 0
        for p in order:
            if p is PAUSE:
                send("stop")
                input("Turn the robot round on the start mark (facing the other way along the edge, "
                      "patch on its left), then press Enter: ")
                send("manual")
                time.sleep(args.gap)
                continue
            v, w, _ = p
            i += 1
            command = f"cmd {v:g} {w:g}"
            console.print(f"  [{i}/{len(holds)}] {command}")
            end = time.monotonic() + hold_of(args, v, w)
            while time.monotonic() < end:
                send(command)
                time.sleep(min(RESEND_S, max(0.0, end - time.monotonic())))
            send("cmd 0 0")
            time.sleep(args.gap)
            send("stop")
    except KeyboardInterrupt:
        console.print("interrupted")
    finally:
        try:
            send("stop")
        except OSError:
            pass
        sock.close()
    grid = f"--set {args.set}" if args.set else "--points " + " ".join(f'"{p}"' for p in args.points)
    console.print(f"done. Stop the logger, then: python tools/twist_test.py report logs/<run>.csv {grid} "
                  f"--repeats {args.repeats}" + ("" if args.hold == HOLD_S else f" --hold {args.hold:g}"))


# ---------------------------------------------------------------- report

def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}


def holds_in(data, join_s=0.3, min_s=1.0):
    """Index ranges where either wheel is driven, joined across short dropouts."""
    t = data["t_ms"] / 1000
    on = (data["pwm_left"] != 0) | (data["pwm_right"] != 0)
    runs = []
    edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
    for a, b in zip(edges[::2], edges[1::2]):
        if runs and t[a] - t[runs[-1][1] - 1] < join_s:
            runs[-1][1] = b
        else:
            runs.append([a, b])
    return [(a, b) for a, b in runs if t[b - 1] - t[a] >= min_s]


def windowed_rates(t, left_mm, right_mm, window, separation):
    """v (mm/s) and omega (deg/s) over non-overlapping windows, using record times."""
    edges = np.searchsorted(t, np.arange(t[0], t[-1] + 1e-9, window))
    edges = np.unique(np.r_[edges, len(t) - 1])
    vs, ws = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        dt = t[b] - t[a]
        if dt < 0.8 * window:
            continue
        dl, dr = left_mm[b] - left_mm[a], right_mm[b] - right_mm[a]
        vs.append((dl + dr) / 2 / dt)
        ws.append(math.degrees((dr - dl) / separation) / dt)
    return np.array(vs), np.array(ws)


def limit(command, pct, floor):
    return max(pct / 100 * abs(command), floor)


def report(args, console):
    data = load(args.file)
    t = data["t_ms"] / 1000
    mm = [2 * math.pi * r / COUNTS_PER_REV for r in args.radius]
    left_mm, right_mm = data["enc_left"] * mm[0], data["enc_right"] * mm[1]
    points = points_of(args)
    wanted = [p for p in schedule(args) if p is not PAUSE]
    holds = holds_in(data)
    if len(holds) < len(wanted):
        console.print(f"[red]found {len(holds)} holds, expected {len(wanted)}; check --set/--points and --repeats[/]")
        return
    if len(holds) > len(wanted):
        console.print(f"[yellow]found {len(holds)} holds; using the last {len(wanted)}[/]")
        holds = holds[-len(wanted):]

    results = []
    for n, ((a, b), (v_cmd, w_cmd, held)) in enumerate(zip(holds, wanted)):
        since = t[a:b] - t[a]
        settle = settle_of(args, v_cmd, w_cmd, args.separation)
        steady = np.flatnonzero((since >= settle) & (since <= hold_of(args, v_cmd, w_cmd) - END_MARGIN_S))
        if len(steady) < 2:
            console.print(f"[red]hold {n} has no steady part between the settle time and the hold[/]")
            return
        s, e = a + steady[0], a + steady[-1]
        dt = t[e] - t[s]
        dl, dr = left_mm[e] - left_mm[s], right_mm[e] - right_mm[s]
        r = {"n": n, "v_cmd": v_cmd, "w_cmd": w_cmd, "held": held,
             "in_range": in_range(v_cmd, w_cmd, args.separation),
             "v": (dl + dr) / 2 / dt, "w": math.degrees((dr - dl) / args.separation) / dt, "steady_s": dt}
        for window in WINDOWS_S:
            vs, ws = windowed_rates(t[s:e + 1], left_mm[s:e + 1], right_mm[s:e + 1], window, args.separation)
            tag = f"{window * 1000:.0f}"
            # One window has no spread to measure (short straight20 holds at 500 ms).
            r[f"sd_v_{tag}"], r[f"sd_w_{tag}"] = (float(vs.std()), float(ws.std())) if len(vs) > 1 else (math.nan,) * 2
        # End pose from the robot's odometry, coast included (to the next hold's start),
        # in the frame the hold started in.
        stop = holds[n + 1][0] - 1 if n + 1 < len(holds) else len(t) - 1
        x0, y0, th0 = data["x_mm"][a - 1], data["y_mm"][a - 1], data["theta_rad"][a - 1]
        dx, dy = data["x_mm"][stop] - x0, data["y_mm"][stop] - y0
        r["end_fwd_mm"] = dx * math.cos(th0) + dy * math.sin(th0)
        r["end_left_mm"] = -dx * math.sin(th0) + dy * math.cos(th0)
        r["dist_mm"] = ((left_mm[stop] - left_mm[a - 1]) + (right_mm[stop] - right_mm[a - 1])) / 2
        r["turn_deg"] = math.degrees(((right_mm[stop] - right_mm[a - 1]) - (left_mm[stop] - left_mm[a - 1]))
                                     / args.separation)
        results.append(r)

    holds_table = Table(title=f"Holds: {Path(args.file).name}")
    for column in ["#", "cmd v", "cmd w", "v mm/s", "w deg/s", "sd v 100/500", "sd w 100/500",
                   "end fwd / left mm", "dist mm", "turned deg"]:
        holds_table.add_column(column, justify="right")
    for r in results:
        holds_table.add_row(str(r["n"]), f"{r['v_cmd']:g}", f"{r['w_cmd']:g}", f"{r['v']:+.1f}", f"{r['w']:+.1f}",
                            f"{r['sd_v_100']:.1f} / {r['sd_v_500']:.1f}", f"{r['sd_w_100']:.1f} / {r['sd_w_500']:.1f}",
                            f"{r['end_fwd_mm']:+.0f} / {r['end_left_mm']:+.0f}", f"{r['dist_mm']:+.1f}",
                            f"{r['turn_deg']:+.1f}")
    console.print(holds_table)
    span = ("from the end of the ramp" if args.settle is None and small(args)
            else f"from {SETTLE_S if args.settle is None else args.settle:g} s")
    console.print(f"v, w: mean over the steady part ({span} to {END_MARGIN_S:g} s before the hold ends), encoders. "
                  "sd: over 100 / 500 ms windows. End pose, distance and turn: whole hold, coast included.")

    summary = Table(title="Per point vs Decision 7 (bias: mean of repeats; each: worst repeat)")
    for column in ["cmd v", "cmd w", "n", "v bias", "v worst", "w bias", "w worst", "spread v / w", "pass"]:
        summary.add_column(column, justify="right")
    point_rows = []
    for v_cmd, w_cmd, held in points:
        rs = [r for r in results if r["v_cmd"] == v_cmd and r["w_cmd"] == w_cmd]
        ev = np.array([r["v"] - v_cmd for r in rs])
        ew = np.array([r["w"] - w_cmd for r in rs])
        worst_v, worst_w = ev[np.argmax(abs(ev))], ew[np.argmax(abs(ew))]
        ok = (abs(ev.mean()) <= limit(v_cmd, 2, 2) and abs(worst_v) <= limit(v_cmd, 5, 4)
              and abs(ew.mean()) <= limit(w_cmd, 2, 2) and abs(worst_w) <= limit(w_cmd, 5, 4))
        verdict = ("pass" if ok else "FAIL") if in_range(v_cmd, w_cmd, args.separation) else "out of range"
        point_rows.append({"v_cmd": v_cmd, "w_cmd": w_cmd, "held": held, "n": len(rs),
                           "v_bias": float(ev.mean()), "v_worst": float(worst_v), "v_spread": float(ev.std()),
                           "w_bias": float(ew.mean()), "w_worst": float(worst_w), "w_spread": float(ew.std()),
                           "verdict": verdict})
        colour = {"pass": "green", "FAIL": "red"}.get(verdict, "yellow")
        summary.add_row(f"{v_cmd:g}", f"{w_cmd:g}" + (" (held)" if held else ""), str(len(rs)),
                        f"{ev.mean():+.2f}", f"{worst_v:+.2f}", f"{ew.mean():+.2f}", f"{worst_w:+.2f}",
                        f"{ev.std():.2f} / {ew.std():.2f}", f"[{colour}]{verdict}[/]")
    console.print(summary)
    console.print("Limits: bias within max(2 %, 2) and each repeat within max(5 %, 4), mm/s and deg/s. "
                  "spread: sd of the repeat means.")

    if args.truth:
        check_truth(args.truth, results, console)

    out = args.out or OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)
    for name, rows in (("holds.csv", results), ("points.csv", point_rows)):
        with open(out / name, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            for row in rows:
                writer.writerow({k: (f"{v:.3f}" if isinstance(v, float) else v) for k, v in row.items()})
    (out / "summary.txt").write_text(console.export_text())
    console.print(f"saved {out}/")


def check_truth(path, results, console):
    """Grid readings against odometry: mean within 2 %, each within 3 % (Decision 7)."""
    with open(path) as f:
        truth = list(csv.DictReader(f))
    table = Table(title="Grid vs odometry, whole hold")
    for column in ["#", "cmd", "quantity", "grid", "odometry", "error %"]:
        table.add_column(column, justify="right")
    errors = {"dist_mm": [], "turn_deg": []}
    for row in truth:
        r = results[int(row["n"])]
        for key in errors:
            if not row.get(key, "").strip():
                continue
            grid, odo = float(row[key]), r[key]
            err = 100 * (odo / grid - 1)
            errors[key].append(err)
            table.add_row(row["n"], f"{r['v_cmd']:g} {r['w_cmd']:g}", key, f"{grid:+.1f}", f"{odo:+.1f}",
                          f"{err:+.2f}")
    console.print(table)
    for key, errs in errors.items():
        if errs:
            errs = np.array(errs)
            ok = abs(errs.mean()) <= 2 and abs(errs).max() <= 3
            console.print(f"{key}: mean {errs.mean():+.2f} %, worst {errs[np.argmax(abs(errs))]:+.2f} % "
                          f"(n {len(errs)}): {'[green]pass' if ok else '[red]FAIL'}[/]")


# ---------------------------------------------------------------- model

def fit_variance(sd, v, w):
    """Non-negative a, b with sd^2 ~ a v^2 + b w^2 (w in rad/s); drops a term that fits negative."""
    y = sd ** 2
    X = np.column_stack([v ** 2, w ** 2])
    coef = np.linalg.lstsq(X, y, rcond=None)[0]
    if coef.min() < 0:
        keep = int(np.argmax(coef))
        coef = np.zeros(2)
        col = X[:, keep]
        coef[keep] = float(col @ y / (col @ col))
    return coef


def model(args, console):
    rows = []
    for folder in args.folders:
        with open(folder / "holds.csv") as f:
            rows += [r for r in csv.DictReader(f) if r["in_range"] == "True"]
    col = lambda key, rs: np.array([float(r[key]) for r in rs])
    fit = [r for r in rows if r["held"] == "False"]
    held = [r for r in rows if r["held"] == "True"]
    result = {"sources": [str(f) for f in args.folders], "n_fit": len(fit), "n_held": len(held),
              "units": "sigma_v mm/s, sigma_w rad/s; v mm/s, w rad/s", "windows": {}}
    table = Table(title=f"Noise model: {len(fit)} holds fitted, {len(held)} held back")
    for column in ["window", "a1 (v on v)", "a2 (w on v)", "a3 (v on w)", "a4 (w on w)", "held-back rms err v / w"]:
        table.add_column(column, justify="right")
    for window in WINDOWS_S:
        tag = f"{window * 1000:.0f}"
        fit_w = [r for r in fit if r[f"sd_v_{tag}"] != "nan"]
        held_w = [r for r in held if r[f"sd_v_{tag}"] != "nan"]
        v, w = col("v_cmd", fit_w), np.radians(col("w_cmd", fit_w))
        a12 = fit_variance(col(f"sd_v_{tag}", fit_w), v, w)
        a34 = fit_variance(np.radians(col(f"sd_w_{tag}", fit_w)), v, w)
        entry = {"a1": a12[0], "a2": a12[1], "a3": a34[0], "a4": a34[1]}
        err = "-"
        if held_w:
            hv, hw = col("v_cmd", held_w), np.radians(col("w_cmd", held_w))
            pv = np.sqrt(a12[0] * hv ** 2 + a12[1] * hw ** 2)
            pw = np.degrees(np.sqrt(a34[0] * hv ** 2 + a34[1] * hw ** 2))
            ev = pv - col(f"sd_v_{tag}", held_w)
            ew = pw - col(f"sd_w_{tag}", held_w)
            entry["held_rms_err_v_mm_s"] = float(np.sqrt(np.mean(ev ** 2)))
            entry["held_rms_err_w_deg_s"] = float(np.sqrt(np.mean(ew ** 2)))
            err = f"{entry['held_rms_err_v_mm_s']:.2f} mm/s / {entry['held_rms_err_w_deg_s']:.2f} deg/s"
        result["windows"][f"{tag}ms"] = {k: float(x) for k, x in entry.items()}
        table.add_row(f"{tag} ms", *(f"{entry[k]:.3g}" for k in ("a1", "a2", "a3", "a4")), err)
    console.print(table)
    console.print("sigma_v^2 = a1 v^2 + a2 w^2, sigma_w^2 = a3 v^2 + a4 w^2 (v mm/s, w rad/s, sigma_w rad/s).")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    console.print(f"saved {args.out}")


def main():
    args = parse_args()
    console = Console(record=True)
    {"run": run, "report": report, "model": model}[args.action](args, console)


if __name__ == "__main__":
    main()
