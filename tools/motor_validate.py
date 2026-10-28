"""Validate the open-loop motor model on a command sequence it has never seen
(plan 1.11, Paul's Exercise 8).

The model is frozen in MODEL below, every number taken from the floor tests
(spin on the spot): steady speed per PWM from the gain test (1.5), start and
keep-turning thresholds from the deadband (1.2) and keep-turning (1.3) tests,
dead time and tau from the step test (1.7). Nothing here is fitted to the
validation run.

    python tools/motor_validate.py predict      # writes the prediction first
    python tools/logger.py --name motors/validation/<run name>
    python tools/motor_validate.py run          # drives SEQUENCE, spin on the spot
    python tools/motor_validate.py check logs/motors/validation/<run>.csv

"predict" saves the model and the prediction for the planned timing to
bench/1.7_step_response/validation/ before anything runs. "check" predicts
again on the PWM timing the robot actually logged (the teleop's timing
wanders by a few ms), with the same frozen model, and compares: speed error
per level once settled, error over the transients, and whether each wheel
stalled or started when the model said it would.

Per wheel, at each PWM change the model picks the new target speed and a
(dead time, tau) pair by the kind of change:
- PWM 0: target 0, "coast" timing.
- moving the same way, PWM below the keep-turning PWM: target 0 (stalls),
  "down" timing.
- moving the same way: target from the gain line, "up" or "down" timing.
- moving the other way (reversal): target from the gain line, "down" timing.
- at rest, PWM at or above the 50 % start point: target from the gain line,
  "rest" timing; below it the wheel stays still.
After the dead time the speed moves to the target as a first-order lag.
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

HOST, PORT = "127.0.0.1", 9100
RESEND_S = 0.2
MM_PER_COUNT = 2 * math.pi * 16.0 / 358.3
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "1.7_step_response" / "validation"
WINDOW_S = 0.05                     # measured speed window, centred
SETTLED_S = 0.5                     # steady error: the last this-long of each level
MOVING_MM_S = 5.0                   # slower than this counts as stopped

# Left wheel PWM (the right wheel gets the opposite sign: spin on the spot) and seconds.
# Levels 45, 75, 35, 21, 55, 60, 90, 40, 65 were not fitted in any test;
# at 21 the model says the left wheel stalls and the right keeps turning.
SEQUENCE = [(0, 1.0), (45, 2.0), (75, 1.5), (35, 2.0), (21, 2.0), (55, 1.5),
            (-60, 2.0), (-90, 1.5), (-40, 2.0), (0, 2.0), (65, 1.5), (0, 2.0)]

# Frozen 2026-10-23. "fwd"/"rev" is the wheel's own direction.
MODEL = {
    "source": {
        "speed": "bench/1.5_gain floor spin runs 20261014_202947 (dir 1) and _203235 (dir -1): speed = a * pwm + b",
        "start": "bench/1.2_deadband floor spin runs 20261014_202249 / _202500: 50 % start point",
        "keep": "bench/1.3_keepturn floor spin runs 20261014_202738 / _202838: median stop PWM",
        "timing": "bench/1.7_step_response/step_floor: medians of dead time and tau (s)",
    },
    "left": {
        "fwd": {"a": 3.419, "b": -62.0, "start": 32.5, "keep": 25},
        "rev": {"a": 3.762, "b": -75.2, "start": 32.5, "keep": 23},
        "timing": {"rest": [0.098, 0.074], "up": [0.059, 0.067], "down": [0.040, 0.110], "coast": [0.068, 0.044]},
    },
    "right": {
        "fwd": {"a": 3.852, "b": -47.2, "start": 28.7, "keep": 18},
        "rev": {"a": 3.811, "b": -45.1, "start": 32.5, "keep": 18},
        "timing": {"rest": [0.098, 0.054], "up": [0.056, 0.056], "down": [0.050, 0.050], "coast": [0.062, 0.042]},
    },
}


def parse_args():
    parser = argparse.ArgumentParser(description="Validate the motor model on an unseen sequence.")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("predict")
    run = sub.add_parser("run")
    run.add_argument("--port", type=int, default=PORT)
    check = sub.add_parser("check")
    check.add_argument("file")
    return parser.parse_args()


def steady_speed(params, pwm):
    """Signed steady speed for a signed PWM on one wheel."""
    p = params["fwd" if pwm > 0 else "rev"]
    return math.copysign(max(0.0, p["a"] * abs(pwm) + p["b"]), pwm)


def simulate(t, pwm, params):
    """Predicted signed speed (mm/s) at each time t (s) for one wheel's signed PWM column."""
    v = np.zeros(len(t))
    speed, target, tau = 0.0, 0.0, 0.05
    latest = 0.0                        # newest target, even while its dead time runs
    pending = []                        # (time it takes effect, target, tau)
    timing = params["timing"]
    for k in range(len(t)):
        if k and pwm[k] != pwm[k - 1]:
            u = pwm[k]
            moving = abs(speed) > MOVING_MM_S or abs(latest) > MOVING_MM_S
            direction = math.copysign(1.0, speed if abs(speed) > MOVING_MM_S else latest)
            if u == 0:
                new, kind = 0.0, "coast"
            elif moving and math.copysign(1.0, u) == direction:
                p = params["fwd" if u > 0 else "rev"]
                if abs(u) < p["keep"]:
                    new, kind = 0.0, "down"
                else:
                    new = steady_speed(params, u)
                    kind = "up" if abs(new) > abs(latest) else "down"
            elif moving:
                new, kind = steady_speed(params, u), "down"
            else:
                p = params["fwd" if u > 0 else "rev"]
                new, kind = (steady_speed(params, u), "rest") if abs(u) >= p["start"] else (0.0, "rest")
            dead, tau_k = timing[kind]
            pending.append((t[k] + dead, new, tau_k))
            latest = new
        while pending and t[k] >= pending[0][0]:
            _, target, tau = pending.pop(0)
        if k:
            dt = t[k] - t[k - 1]
            speed += (target - speed) * (1.0 - math.exp(-dt / tau))
        v[k] = speed
    return v


def nominal_timeline():
    t, left = [], []
    now = 0.0
    for u, seconds in SEQUENCE:
        for _ in range(int(round(seconds / 0.01))):
            t.append(now)
            left.append(u)
            now += 0.01
    return np.array(t), np.array(left, dtype=float)


def predict(console):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    t, left = nominal_timeline()
    right = -left
    vl = simulate(t, left, MODEL["left"])
    vr = simulate(t, right, MODEL["right"])
    (OUT_DIR / "model.json").write_text(json.dumps({"frozen": time.strftime("%Y-%m-%d %H:%M:%S"),
                                                    "sequence": SEQUENCE, "model": MODEL}, indent=2))
    with open(OUT_DIR / "prediction_nominal.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "pwm_left", "pwm_right", "v_left_mm_s", "v_right_mm_s"])
        for row in zip(t, left, right, vl, vr):
            w.writerow([f"{row[0]:.2f}", f"{row[1]:g}", f"{row[2]:g}", f"{row[3]:.1f}", f"{row[4]:.1f}"])
    table = Table(title="Prediction, end of each level (mm/s, signed)")
    for column in ["#", "pwm L / R", "seconds", "left", "right"]:
        table.add_column(column, justify="right")
    end = 0.0
    for i, (u, seconds) in enumerate(SEQUENCE):
        end += seconds
        k = min(len(t) - 1, int(round(end / 0.01)) - 1)
        table.add_row(str(i), f"{u:g} / {-u:g}", f"{seconds:g}", f"{vl[k]:.1f}", f"{vr[k]:.1f}")
    console.print(table)
    console.print(f"saved {OUT_DIR / 'model.json'} and {OUT_DIR / 'prediction_nominal.csv'}")


def run(args, console):
    if not (OUT_DIR / "model.json").exists():
        console.print("run 'predict' first, so the prediction is on disk before the robot moves")
        return
    try:
        sock = socket.create_connection((HOST, args.port), timeout=3)
    except OSError as e:
        console.print(f"can't reach the logger on {HOST}:{args.port} ({e}). Is logger.py running with teleop?")
        return
    greeting = sock.recv(256).decode("utf-8", errors="replace").strip()
    if not greeting.startswith("connected"):
        console.print(greeting or "logger closed the connection")
        return

    def send(command):
        sock.sendall(command.encode() + b"\n")

    console.print(f"{greeting}. Spin on the spot, about {sum(s for _, s in SEQUENCE):.0f} s. Ctrl+C stops the robot.")
    try:
        send("manual")
        for u, seconds in SEQUENCE:
            command = f"pwm {u:g} {-u:g}"
            console.print(f"  {command} for {seconds:g} s")
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                send(command)
                time.sleep(min(RESEND_S, max(0.0, end - time.monotonic())))
    except KeyboardInterrupt:
        console.print("interrupted")
    finally:
        try:
            send("stop")
        except OSError:
            pass
        sock.close()
    console.print("done. Stop the logger, then: python tools/motor_validate.py check <csv>")


def centred_speed(t, x, window):
    lo = np.searchsorted(t, t - window / 2)
    hi = np.clip(np.searchsorted(t, t + window / 2, side="right") - 1, 0, len(t) - 1)
    span = t[hi] - t[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(span > 0, (x[hi] - x[lo]) / span, np.nan)


def check(args, console):
    frozen = json.loads((OUT_DIR / "model.json").read_text())
    model = frozen["model"]
    with open(args.file) as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t = (col("t_ms") - float(rows[0]["t_ms"])) / 1000
    out = OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)

    # the sequence: from the first non-zero PWM back 1 s, to 2 s after the last change
    pl = col("pwm_left")
    changes = np.flatnonzero(np.diff(pl) != 0) + 1
    start = np.searchsorted(t, t[changes[0]] - 1.0)
    stop = np.searchsorted(t, t[changes[-1]] + 2.0)
    sl = slice(start, stop)
    t = t[sl] - t[start]

    results = {}
    for wheel in ("left", "right"):
        pwm = col(f"pwm_{wheel}")[sl]
        x = col(f"enc_{wheel}")[sl] * MM_PER_COUNT
        v_meas = centred_speed(t, x, WINDOW_S)
        v_pred = simulate(t, pwm, model[wheel])
        results[wheel] = (pwm, v_meas, v_pred)

    table = Table(title=f"Validation: {Path(args.file).name} (signed mm/s, last {SETTLED_S:g} s of each level)")
    for column in ["#", "pwm L / R", "left meas", "left pred", "left err", "right meas", "right pred", "right err", "moving as predicted"]:
        table.add_column(column, justify="right")
    pl = results["left"][0]
    bounds = [0] + list(np.flatnonzero(np.diff(pl) != 0) + 1) + [len(t)]
    steady_err = {"left": [], "right": []}
    level_rows = []
    agree_all = True
    for i, (a, b) in enumerate(zip(bounds, bounds[1:])):
        settled = (t >= t[b - 1] - SETTLED_S) & (np.arange(len(t)) >= a) & (np.arange(len(t)) < b)
        cells, row = [], {"level": i, "pwm_left": pl[a], "pwm_right": results["right"][0][a]}
        agree = []
        for wheel in ("left", "right"):
            _, vm, vp = results[wheel]
            m, p = float(np.nanmean(vm[settled])), float(np.mean(vp[settled]))
            cells += [f"{m:.1f}", f"{p:.1f}", f"{m - p:+.1f}"]
            row.update({f"{wheel}_meas": m, f"{wheel}_pred": p})
            if abs(p) > MOVING_MM_S:
                steady_err[wheel].append(m - p)
            agree.append((abs(m) > MOVING_MM_S) == (abs(p) > MOVING_MM_S))
        agree_all &= all(agree)
        level_rows.append(row)
        table.add_row(str(i), f"{pl[a]:g} / {results['right'][0][a]:g}", *cells,
                      "yes" if all(agree) else "[red]no[/]")
    console.print(table)

    for wheel in ("left", "right"):
        _, vm, vp = results[wheel]
        ok = np.isfinite(vm)
        e = np.array(steady_err[wheel])
        moving = ok & (np.abs(vp) > MOVING_MM_S)
        rel = np.abs(vm[moving] - vp[moving]) / np.maximum(np.abs(vp[moving]), 1.0)
        console.print(f"{wheel}: settled error mean {e.mean():+.1f}, rms {np.sqrt((e ** 2).mean()):.1f} mm/s "
                      f"over {len(e)} moving levels; whole trace rms {np.sqrt(np.mean((vm[ok] - vp[ok]) ** 2)):.1f} mm/s "
                      f"({WINDOW_S * 1000:.0f} ms window); within 10 % of prediction {100 * np.mean(rel < 0.10):.0f} % of moving time")
    console.print("stall/start decisions: " + ("all as predicted" if agree_all else "[red]some not as predicted[/]"))

    with open(out / "levels.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(level_rows[0]))
        w.writeheader()
        for r in level_rows:
            w.writerow({k: (f"{v:.2f}" if isinstance(v, float) else v) for k, v in r.items()})
    plot(t, results, out / "plot.png", Path(args.file).name, console)
    (out / "summary.txt").write_text(console.export_text())
    console.print(f"saved {out}/")


def plot(t, results, path, title, console):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        console.print("[yellow]plots need matplotlib[/]")
        return
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, gridspec_kw={"height_ratios": [3, 3, 1]})
    colours = {"left": "#4a6fa5", "right": "#c0392b"}
    for ax, wheel in zip(axes, ("left", "right")):
        pwm, vm, vp = results[wheel]
        ax.plot(t, vm, color=colours[wheel], lw=0.8, label=f"measured ({WINDOW_S * 1000:.0f} ms window)")
        ax.plot(t, vp, color="#222222", lw=1.2, label="predicted (frozen model)")
        ax.set_ylabel(f"{wheel} speed (mm/s)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
    axes[2].step(t, results["left"][0], where="post", color=colours["left"], label="left")
    axes[2].step(t, results["right"][0], where="post", color=colours["right"], label="right")
    axes[2].set_ylabel("PWM")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(alpha=0.3)
    axes[2].legend(fontsize=8)
    fig.suptitle(f"Motor model validation: {title}", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    console.print(f"saved {path}")


def main():
    args = parse_args()
    console = Console(record=True)
    if args.action == "predict":
        predict(console)
    elif args.action == "run":
        run(args, console)
    else:
        check(args, console)


if __name__ == "__main__":
    main()
