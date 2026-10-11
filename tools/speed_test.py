"""Wheel speed control test: drive vel steps, then report how well each was held.

Run (logger running with teleop on, robot on a stand or with room to drive):

    python tools/speed_test.py run                     # 20, 30, 50, 80 mm/s, 4 s each
    python tools/speed_test.py run --spin              # spin on the spot instead
    python tools/speed_test.py run --speeds 15 20 25 30

Each step sends "vel v v" (resent every 0.2 s, like the teleop), then a full
stop for --gap seconds, so steps show up in the CSV as runs of non-zero PWM.
Then, on the CSV the logger saved:

    python tools/speed_test.py report logs/<run>.csv   # same --speeds/--spin as the run

The report matches the last N runs of non-zero PWM to the N speeds, in order,
and per step and wheel gives: mean speed and its error, wobble (sd of speed
over 100 ms windows, after --settle seconds), the 5th-95th percentile range,
time stuck, time to reach 90 % of target, start overshoot, and PWM mean and
sd. Results and a plot of every step go to bench/2.6_speed_control/<run>/.
"""
import argparse
import csv
import math
import socket
import time
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

HOST, PORT = "127.0.0.1", 9100     # logger's teleop port (tools/teleop.py)
SPEEDS = [20, 30, 50, 80]
STEP_S = 4.0
GAP_S = 1.5
RESEND_S = 0.2
SETTLE_S = 0.8                      # skipped at the start of each step for the steady-state numbers
WINDOW_S = 0.1                      # speed is measured over this window
STUCK_MM_S = 3.0                    # slower than this over 50 ms counts as stuck
MM_PER_COUNT = 2 * math.pi * 16.0 / 358.3
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "2.6_speed_control"


def parse_args():
    parser = argparse.ArgumentParser(description="Wheel speed control test.")
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("run", "report"):
        p = sub.add_parser(name)
        p.add_argument("--speeds", type=float, nargs="+", default=SPEEDS, help=f"mm/s (default: {SPEEDS})")
        p.add_argument("--spin", action="store_true", help="spin on the spot (vel -v v) instead of driving straight")
    run = sub.choices["run"]
    run.add_argument("--step", type=float, default=STEP_S, help=f"seconds per step (default: {STEP_S})")
    run.add_argument("--gap", type=float, default=GAP_S, help=f"stopped seconds between steps (default: {GAP_S})")
    run.add_argument("--port", type=int, default=PORT, help=f"logger's teleop port (default: {PORT})")
    report = sub.choices["report"]
    report.add_argument("file", help="logger CSV recorded during the run")
    report.add_argument("--settle", type=float, default=SETTLE_S,
                        help=f"seconds skipped at each step's start for steady-state numbers (default: {SETTLE_S})")
    report.add_argument("--out", type=Path, help=f"folder for the results (default: {OUT_DIR}/<file name>)")
    return parser.parse_args()


# ---------------------------------------------------------------- run

def run(args, console):
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

    total = len(args.speeds) * (args.step + args.gap)
    console.print(f"{len(args.speeds)} steps, about {total:.0f} s. Ctrl+C stops the robot.")
    try:
        send("manual")
        time.sleep(args.gap)
        for v in args.speeds:
            command = f"vel {-v:g} {v:g}" if args.spin else f"vel {v:g} {v:g}"
            console.print(f"  {command} for {args.step:g} s")
            end = time.monotonic() + args.step
            while time.monotonic() < end:
                send(command)
                time.sleep(min(RESEND_S, max(0.0, end - time.monotonic())))
            send("vel 0 0")
            time.sleep(args.gap)
    except KeyboardInterrupt:
        console.print("interrupted")
    finally:
        try:
            send("stop")
        except OSError:
            pass
        sock.close()
    console.print("done. Stop the logger, then: python tools/speed_test.py report logs/<run>.csv"
                  + (" --spin" if args.spin else "")
                  + ("" if args.speeds == SPEEDS else " --speeds " + " ".join(f"{v:g}" for v in args.speeds)))


# ---------------------------------------------------------------- report

def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}


def steps_in(data, min_s=0.5, join_s=0.3):
    """Index ranges where both wheels are driven, joined across short dropouts."""
    t = data["t_ms"] / 1000
    on = (data["pwm_left"] != 0) & (data["pwm_right"] != 0)
    runs, start = [], None
    for i, value in enumerate(np.append(on, False)):
        if value and start is None:
            start = i
        elif not value and start is not None:
            if runs and t[start] - t[runs[-1][1] - 1] < join_s:
                runs[-1] = (runs[-1][0], i)
            else:
                runs.append((start, i))
            start = None
    return [(a, b) for a, b in runs if t[b - 1] - t[a] >= min_s]


def windowed_speed(t, counts, window):
    """mm/s over the preceding window at each record, using record times (gaps are fine)."""
    j = np.searchsorted(t, t - window)
    span = t - t[j]
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(span > 0, (counts - counts[j]) * MM_PER_COUNT / span, np.nan)


def wheel_step(t, counts, pwm, target, settle):
    """Numbers for one wheel over one step; counts and pwm already signed so forward is positive."""
    v = windowed_speed(t, counts, WINDOW_S)
    v_short = windowed_speed(t, counts, 0.05)
    since = t - t[0]
    steady = (since >= settle) & ~np.isnan(v)
    vs = v[steady]
    reached = np.flatnonzero(v >= 0.9 * target)
    early = (since <= 1.0) & ~np.isnan(v)
    return {
        "mean": float(vs.mean()), "error_pct": 100 * float(vs.mean() / target - 1),
        "sd": float(vs.std()), "sd_pct": 100 * float(vs.std() / target),
        "p5": float(np.percentile(vs, 5)), "p95": float(np.percentile(vs, 95)),
        "stuck_pct": 100 * float(np.mean(v_short[steady & ~np.isnan(v_short)] < STUCK_MM_S)),
        "rise_ms": float(since[reached[0]] * 1000) if len(reached) else float("nan"),
        "overshoot_pct": 100 * float(np.nanmax(v[early]) / target - 1) if early.any() else float("nan"),
        "pwm_mean": float(pwm[steady].mean()), "pwm_sd": float(pwm[steady].std()),
        "trace": (since, v),
    }


def report(args, console):
    data = load(args.file)
    t = data["t_ms"] / 1000
    steps = steps_in(data)
    if len(steps) < len(args.speeds):
        console.print(f"[red]found {len(steps)} driven stretches but {len(args.speeds)} speeds; "
                      "check --speeds, or that the run was recorded[/]")
        return
    if len(steps) > len(args.speeds):
        console.print(f"[yellow]found {len(steps)} driven stretches; using the last {len(args.speeds)}[/]")
        steps = steps[-len(args.speeds):]

    table = Table(title=f"Wheel speed control: {Path(args.file).name}" + (" (spin)" if args.spin else ""))
    for column in ["target", "wheel", "mean", "error", "wobble sd", "sd % of target", "p5-p95",
                   "stuck", "90 % at", "overshoot", "pwm mean", "pwm sd"]:
        table.add_column(column, justify="right")
    results = []
    for (a, b), target in zip(steps, args.speeds):
        for wheel in ("left", "right"):
            sign = -1.0 if (args.spin and wheel == "left") else 1.0
            r = wheel_step(t[a:b], sign * data[f"enc_{wheel}"][a:b], sign * data[f"pwm_{wheel}"][a:b],
                           target, args.settle)
            r.update({"target": target, "wheel": wheel, "duration_s": float(t[b - 1] - t[a])})
            results.append(r)
            table.add_row(f"{target:g}", wheel, f"{r['mean']:.1f}", f"{r['error_pct']:+.1f} %",
                          f"{r['sd']:.1f}", f"{r['sd_pct']:.1f} %", f"{r['p5']:.0f}-{r['p95']:.0f}",
                          f"{r['stuck_pct']:.1f} %", f"{r['rise_ms']:.0f} ms", f"{r['overshoot_pct']:+.0f} %",
                          f"{r['pwm_mean']:.1f}", f"{r['pwm_sd']:.1f}")
    console.print(table)
    console.print(f"Speeds in mm/s over {WINDOW_S * 1000:.0f} ms windows. Steady numbers skip the first "
                  f"{args.settle:g} s. stuck: share of 50 ms windows under {STUCK_MM_S:g} mm/s. overshoot: "
                  "highest speed in the first second over target.")

    out = args.out or OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)
    keys = ["target", "wheel", "duration_s", "mean", "error_pct", "sd", "sd_pct", "p5", "p95",
            "stuck_pct", "rise_ms", "overshoot_pct", "pwm_mean", "pwm_sd"]
    with open(out / "summary.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(keys + ["spin"])
        for r in results:
            writer.writerow([f"{r[k]:.3f}" if isinstance(r[k], float) else r[k] for k in keys] + [args.spin])
    console.print(f"saved {out / 'summary.csv'}")
    plot(results, args.speeds, out / "speed_steps.png", Path(args.file).name, console)


def plot(results, speeds, path, title, console):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        console.print("[yellow]plots need matplotlib: pip install matplotlib[/]")
        return
    fig, axes = plt.subplots(1, len(speeds), figsize=(3.4 * len(speeds), 3.6), squeeze=False)
    colours = {"left": "#4a6fa5", "right": "#c0392b"}
    for ax, target in zip(axes[0], speeds):
        for r in results:
            if r["target"] != target:
                continue
            since, v = r["trace"]
            ax.plot(since, v, lw=1, color=colours[r["wheel"]],
                    label=f"{r['wheel']}: sd {r['sd']:.1f} ({r['sd_pct']:.0f} %)")
        ax.axhline(target, color="#222222", ls="--", lw=1)
        ax.set_title(f"vel {target:g} mm/s", fontsize=10)
        ax.set_xlabel("time in step (s)")
        ax.set_ylim(0, max(target * 2, 10))
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="lower right")
    axes[0][0].set_ylabel(f"wheel speed, mm/s ({WINDOW_S * 1000:.0f} ms window)")
    fig.suptitle(f"Wheel speed steps: {title}", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    console.print(f"saved {path}")


def main():
    args = parse_args()
    console = Console()
    if args.action == "run":
        run(args, console)
    else:
        report(args, console)


if __name__ == "__main__":
    main()
