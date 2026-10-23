"""Motor step response and coast-down test (plan 1.7 and 1.8, Paul's Exercise 7).

Open loop: sends raw "pwm" levels, so the speed controller, feedforward and
config play no part. Run with the logger recording and teleop on:

    python tools/logger.py --name motors/step-response/<run name>
    python tools/step_test.py run                 # floor, spin on the spot, dir 1
    python tools/step_test.py run --dir -1        # spin the other way
    python tools/step_test.py run --stand         # wheels clear, both forward

Each repeat drives three blocks, holding every level for --hold seconds
(resent every 0.2 s, like the teleop):

    steps     0 -> 50 -> 80 -> 50 -> 30 -> 80 -> 30 -> 0
    coast     0 -> 60 -> 0,  0 -> 80 -> 0,  0 -> 100 -> 0   (--rest s after each stop)
    held back 0 -> 40 -> 70 -> 40 -> 0

The held-back block uses levels no fitted step uses, and is only predicted.

Then, on the CSV the logger saved (several files are pooled):

    python tools/step_test.py report logs/motors/step-response/<run>.csv [...]

Every PWM change in the CSV is a step. For each step and wheel the report
fits, on the encoder counts (not a differentiated speed), a dead time L and
a first-order lag tau:

    v(t) = v0                                        t < L
    v(t) = v1 + (v0 - v1) exp(-(t - L) / tau)        t >= L

v0 and v1 are the measured steady speeds before and after. Steps are
grouped as from rest (v0 = 0), up, down and coast (to PWM 0). From the
coasts it also gives the distance and time to stop, and the stop lead
(coast distance / speed) for move_stop_lead_s. The held-back steps are
predicted from each wheel's median L and tau and compared with what they
did. Results go to bench/1.7_step_response/<first file name>/.
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
HOLD_S = 2.5
REST_S = 2.5
REPEATS = 3
RESEND_S = 0.2
STEP_LEVELS = [50, 80, 50, 30, 80, 30, 0]
COAST_LEVELS = [60, 80, 100]
HELD_BACK_LEVELS = [40, 70, 40, 0]
HELD_BACK = {40, 70}                # a step to or from these levels is only predicted, never fitted
MM_PER_COUNT = 2 * math.pi * 16.0 / 358.3
PRE_S = 0.6                         # steady speed before a step: fit over this long before it
POST_FIT_S = 1.2                    # model fitted over this long after a step
STEADY_S = 0.8                      # steady speed after a step: the last this-long of the level
QUIET_S = 0.3                       # a wheel has stopped after this long without a count
WINDOW_S = 0.03                     # speed window for plots and the held-back check
DEADS = np.arange(0.0, 0.2001, 0.002)
TAUS = np.arange(0.01, 0.5001, 0.002)
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "1.7_step_response"


def parse_args():
    parser = argparse.ArgumentParser(description="Motor step response and coast-down test.")
    sub = parser.add_subparsers(dest="action", required=True)
    run = sub.add_parser("run")
    run.add_argument("--dir", type=int, choices=(1, -1), default=1,
                     help="1: left wheel forward (right back when spinning); -1 the other way")
    run.add_argument("--stand", action="store_true", help="both wheels the same way, for the stand")
    run.add_argument("--repeats", type=int, default=REPEATS, help=f"default: {REPEATS}")
    run.add_argument("--hold", type=float, default=HOLD_S, help=f"seconds per level (default: {HOLD_S})")
    run.add_argument("--rest", type=float, default=REST_S, help=f"seconds stopped after a stop (default: {REST_S})")
    run.add_argument("--port", type=int, default=PORT, help=f"logger's teleop port (default: {PORT})")
    report = sub.add_parser("report")
    report.add_argument("files", nargs="+", help="logger CSVs recorded during runs")
    report.add_argument("--out", type=Path, help=f"folder for the results (default: {OUT_DIR}/<first file name>)")
    return parser.parse_args()


# ---------------------------------------------------------------- run

def sequence(args):
    """(pwm level, seconds) in order, for the left wheel's direction; 0 levels hold for --rest."""
    block = [(u, args.rest if u == 0 else args.hold) for u in STEP_LEVELS]
    for u in COAST_LEVELS:
        block += [(u, args.hold), (0, args.rest)]
    block += [(u, args.rest if u == 0 else args.hold) for u in HELD_BACK_LEVELS]
    return [(0, args.rest)] + block * args.repeats


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

    steps = sequence(args)
    right_sign = 1 if args.stand else -1
    total = sum(s for _, s in steps)
    console.print(f"{'stand' if args.stand else 'spin'}, dir {args.dir}: {len(steps)} levels, "
                  f"about {total:.0f} s. Ctrl+C stops the robot.")
    try:
        send("manual")
        for i, (u, seconds) in enumerate(steps):
            left = args.dir * u
            command = f"pwm {left:g} {right_sign * left:g}"
            console.print(f"  [{i + 1}/{len(steps)}] {command} for {seconds:g} s")
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
    console.print("done. Stop the logger, then: python tools/step_test.py report <csv>")


# ---------------------------------------------------------------- report

def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}


def slope(t, x):
    """Least-squares slope of x over t, or nan with too few points."""
    if len(t) < 5:
        return float("nan")
    return float(np.polyfit(t, x, 1)[0])


def model_position(t, v0, v1, dead, tau):
    """Position (without its offset) of the first-order-plus-dead-time model; t from the step."""
    s = np.clip(t - dead, 0.0, None)
    return v0 * t + (v1 - v0) * (s - tau * (1.0 - np.exp(-s / tau)))


def fit_step(t, x, v0, v1):
    """Grid search for dead time and tau on position; returns dead, tau, rms residual (mm)."""
    s = np.clip(t[None, None, :] - DEADS[:, None, None], 0.0, None)
    tau = TAUS[None, :, None]
    shape = s - tau * (1.0 - np.exp(-s / tau))
    pred = v0 * t[None, None, :] + (v1 - v0) * shape
    resid = x[None, None, :] - pred
    resid -= resid.mean(axis=2, keepdims=True)     # free offset
    sse = (resid ** 2).sum(axis=2)
    i, j = np.unravel_index(np.argmin(sse), sse.shape)
    return float(DEADS[i]), float(TAUS[j]), float(np.sqrt(sse[i, j] / len(t)))


def windowed_speed(t, x, window):
    j = np.searchsorted(t, t - window)
    span = t - t[j]
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(span > 0, (x - x[j]) / span, np.nan)


def find_steps(data, source):
    """One dict per wheel per PWM change, with the signed position trace around it."""
    t = data["t_ms"] / 1000
    steps = []
    for wheel in ("left", "right"):
        pwm = data[f"pwm_{wheel}"]
        pos = data[f"enc_{wheel}"] * MM_PER_COUNT
        changes = np.flatnonzero(np.diff(pwm) != 0) + 1
        bounds = list(changes) + [len(t)]
        for k, i in enumerate(changes):
            u0, u1 = pwm[i - 1], pwm[i]
            sign = math.copysign(1.0, u1 if u1 != 0 else u0)
            prev = changes[k - 1] if k else 0
            nxt = bounds[k + 1]
            hold_s = t[nxt - 1] - t[i]
            pre_s = t[i - 1] - t[prev]
            tr = t - t[i]
            x = sign * pos
            before = (tr >= -min(PRE_S, pre_s)) & (tr < 0) & (np.arange(len(t)) >= prev)
            v0 = 0.0 if u0 == 0 else slope(tr[before], x[before])
            if u1 == 0:
                v1 = 0.0
            else:
                after = (tr >= hold_s - STEADY_S) & (np.arange(len(t)) < nxt) & (tr >= 0)
                v1 = slope(tr[after], x[after]) if hold_s >= STEADY_S + 0.5 else float("nan")
            kind = ("coast" if u1 == 0 else "rest" if u0 == 0 else "up" if abs(u1) > abs(u0) else "down")
            steps.append({
                "source": source, "wheel": wheel, "i": int(i), "t_s": float(t[i] - t[0]),
                "u0": float(abs(u0)), "u1": float(abs(u1)), "kind": kind,
                "held_back": bool({abs(u0), abs(u1)} & HELD_BACK),
                "v0": v0, "v1": v1, "hold_s": float(hold_s),
                "_t": t, "_x": x, "_tr": tr, "_prev": int(prev), "_next": int(nxt),
            })
    return steps


def analyse_step(st):
    tr, x = st["_tr"], st["_x"]
    idx = np.arange(len(tr))
    end_s = min(POST_FIT_S, st["hold_s"])
    window = (tr >= -0.1) & (tr <= end_s) & (idx >= st["_prev"]) & (idx < st["_next"])
    st.update(dead_ms=float("nan"), tau_ms=float("nan"), rms_mm=float("nan"),
              coast_mm=float("nan"), stop_ms=float("nan"), lead_s=float("nan"))
    if not (np.isfinite(st["v0"]) and np.isfinite(st["v1"])) or window.sum() < 20:
        return
    if abs(st["v1"] - st["v0"]) < 10:
        return
    dead, tau, rms = fit_step(tr[window], x[window], st["v0"], st["v1"])
    st.update(dead_ms=dead * 1000, tau_ms=tau * 1000, rms_mm=rms)
    if st["kind"] == "coast":
        # last count before QUIET_S without one, within the stopped stretch
        seg = (idx >= st["i"] - 1) & (idx < st["_next"])
        ts, xs = tr[seg], x[seg]
        moved = np.flatnonzero(np.diff(xs) != 0)
        last = 0
        for m in moved:
            if ts[m + 1] - ts[last] > QUIET_S and last > 0:
                break
            last = m + 1
        st.update(coast_mm=float(xs[last] - xs[0]), stop_ms=float(ts[last] * 1000),
                  lead_s=float((xs[last] - xs[0]) / st["v0"]) if st["v0"] > 0 else float("nan"))


def stats(values):
    v = np.array([x for x in values if np.isfinite(x)])
    if not len(v):
        return "-"
    return f"{np.median(v):.0f} [{v.min():.0f}-{v.max():.0f}]"


def report(args, console):
    steps = []
    for f in args.files:
        steps += find_steps(load(f), Path(f).name)
    for st in steps:
        analyse_step(st)
    fitted = [s for s in steps if np.isfinite(s["tau_ms"])]
    console.print(f"{len(steps)} wheel steps in {len(args.files)} file(s), {len(fitted)} fitted")

    # per wheel and kind, held back excluded
    table = Table(title="Dead time and tau per wheel (median \\[min-max], ms), held-back steps excluded")
    for column in ["wheel", "kind", "n", "dead time", "tau", "63 % at (dead + tau)", "fit rms mm"]:
        table.add_column(column, justify="right")
    model = {}
    for wheel in ("left", "right"):
        for kind in ("rest", "up", "down", "coast"):
            group = [s for s in fitted if s["wheel"] == wheel and s["kind"] == kind and not s["held_back"]]
            if not group:
                continue
            table.add_row(wheel, kind, str(len(group)), stats(s["dead_ms"] for s in group),
                          stats(s["tau_ms"] for s in group), stats(s["dead_ms"] + s["tau_ms"] for s in group),
                          f"{np.median([s['rms_mm'] for s in group]):.2f}")
        moving = [s for s in fitted if s["wheel"] == wheel and s["kind"] in ("up", "down") and not s["held_back"]]
        if moving:
            model[wheel] = (float(np.median([s["dead_ms"] for s in moving])) / 1000,
                            float(np.median([s["tau_ms"] for s in moving])) / 1000)
    console.print(table)
    for wheel, (dead, tau) in model.items():
        console.print(f"model {wheel}: dead time {dead * 1000:.0f} ms, tau {tau * 1000:.0f} ms "
                      "(median over up and down steps while moving)")

    # coast
    coasts = sorted((s for s in fitted if s["kind"] == "coast" and np.isfinite(s["coast_mm"])),
                    key=lambda s: (s["wheel"], s["v0"]))
    table = Table(title="Coast to a stop after PWM 0")
    for column in ["wheel", "from PWM", "speed mm/s", "coast mm", "stop ms", "lead s (coast / speed)", "tau ms"]:
        table.add_column(column, justify="right")
    for s in coasts:
        table.add_row(s["wheel"], f"{s['u0']:.0f}", f"{s['v0']:.1f}", f"{s['coast_mm']:.1f}",
                      f"{s['stop_ms']:.0f}", f"{s['lead_s']:.3f}", f"{s['tau_ms']:.0f}")
    console.print(table)
    coast_fit = {}
    for wheel in ("left", "right"):
        cs = [s for s in coasts if s["wheel"] == wheel]
        if len(cs) >= 3:
            v = np.array([s["v0"] for s in cs])
            d = np.array([s["coast_mm"] for s in cs])
            a, b = np.linalg.lstsq(np.c_[v, v ** 2], d, rcond=None)[0]
            coast_fit[wheel] = (a, b)
            leads = ", ".join(f"{speed} mm/s: {a + b * speed:.3f} s" for speed in (40, 60, 100, 200))
            console.print(f"coast {wheel}: distance = {a:.4f} v + {b:.6f} v^2 mm; stop lead {leads}")

    # held-back prediction
    held = [s for s in steps if s["held_back"] and s["kind"] != "coast" and s["wheel"] in model
            and np.isfinite(s["v0"]) and np.isfinite(s["v1"])]
    if held:
        table = Table(title="Held-back steps: predicted from the median model vs measured")
        for column in ["wheel", "step", "n", "speed rms error mm/s", "as % of step", "fitted tau ms"]:
            table.add_column(column, justify="right")
        groups = {}
        for s in held:
            dead, tau = model[s["wheel"]]
            tr, x = s["_tr"], s["_x"]
            idx = np.arange(len(tr))
            w = (tr >= -0.1) & (tr <= min(POST_FIT_S, s["hold_s"])) & (idx >= s["_prev"]) & (idx < s["_next"])
            pred = model_position(tr[w], s["v0"], s["v1"], dead, tau)
            pred += (x[w] - pred).mean()
            v_meas = windowed_speed(tr[w], x[w], WINDOW_S)
            v_pred = windowed_speed(tr[w], pred, WINDOW_S)
            ok = np.isfinite(v_meas) & np.isfinite(v_pred) & (tr[w] >= WINDOW_S - 0.1)
            err = float(np.sqrt(np.mean((v_meas[ok] - v_pred[ok]) ** 2)))
            s["pred_err"] = err
            s["_pred"] = (tr[w], v_pred)
            groups.setdefault((s["wheel"], f"{s['u0']:.0f}->{s['u1']:.0f}"), []).append(s)
        for (wheel, name), group in sorted(groups.items()):
            errs = [g["pred_err"] for g in group]
            pct = [100 * g["pred_err"] / abs(g["v1"] - g["v0"]) for g in group]
            table.add_row(wheel, name, str(len(group)), f"{np.median(errs):.1f}", f"{np.median(pct):.0f} %",
                          stats(g["tau_ms"] for g in group))
        console.print(table)
        console.print(f"Speeds over {WINDOW_S * 1000:.0f} ms windows, so part of the error is count noise "
                      "(about 9 mm/s at 1 count per 30 ms).")

    out = args.out or OUT_DIR / Path(args.files[0]).stem
    out.mkdir(parents=True, exist_ok=True)
    keys = ["source", "wheel", "t_s", "u0", "u1", "kind", "held_back", "v0", "v1", "hold_s",
            "dead_ms", "tau_ms", "rms_mm", "coast_mm", "stop_ms", "lead_s"]
    with open(out / "steps.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        for s in steps:
            writer.writerow([f"{s[k]:.4f}" if isinstance(s[k], float) else s[k] for k in keys])
    console.print(f"saved {out / 'steps.csv'}")
    plot(fitted, model, coast_fit, held, out / "plot.png", ", ".join(Path(f).name for f in args.files), console)
    (out / "summary.txt").write_text(console.export_text())


def plot(fitted, model, coast_fit, held, path, title, console):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        console.print("[yellow]plots need matplotlib: pip install matplotlib[/]")
        return
    colours = {"left": "#4a6fa5", "right": "#c0392b"}
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    t_model = np.linspace(-0.1, 0.8, 400)
    for col, kinds in enumerate((("up", "down"), ("rest",), ("coast",))):
        for row, wheel in enumerate(("left", "right")):
            ax = axes[row][col]
            for s in fitted:
                if s["wheel"] != wheel or s["kind"] not in kinds or s["held_back"]:
                    continue
                tr, x = s["_tr"], s["_x"]
                idx = np.arange(len(tr))
                w = (tr >= -0.15) & (tr <= 0.8) & (idx >= s["_prev"]) & (idx < s["_next"])
                v = windowed_speed(tr[w], x[w], WINDOW_S)
                ax.plot(tr[w] - WINDOW_S / 2, (v - s["v0"]) / (s["v1"] - s["v0"]),
                        color=colours[wheel], lw=0.5, alpha=0.35)
            if wheel in model:
                dead, tau = model[wheel]
                step = np.where(t_model < dead, 0.0, 1.0 - np.exp(-np.clip(t_model - dead, 0, None) / tau))
                ax.plot(t_model, step, color="#222222", lw=1.5,
                        label=f"model: dead {dead * 1000:.0f} ms, tau {tau * 1000:.0f} ms")
                ax.legend(fontsize=8, loc="lower right")
            ax.set_ylim(-0.3, 1.4)
            ax.axhline(1, color="#888888", lw=0.5)
            ax.axvline(0, color="#888888", lw=0.5)
            ax.grid(alpha=0.3)
            ax.set_title(f"{wheel}: {' and '.join(kinds)}", fontsize=10)
            ax.set_xlabel("time from PWM change (s)")
        axes[0][0].set_ylabel("fraction of the speed change")
        axes[1][0].set_ylabel("fraction of the speed change")
    fig.suptitle(f"Step responses ({WINDOW_S * 1000:.0f} ms speed window): {title}", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    console.print(f"saved {path}")

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    ax = axes[0]
    for wheel in ("left", "right"):
        cs = [s for s in fitted if s["kind"] == "coast" and s["wheel"] == wheel and np.isfinite(s["coast_mm"])]
        ax.plot([s["v0"] for s in cs], [s["coast_mm"] for s in cs], "o", color=colours[wheel], label=wheel, ms=4)
        if wheel in coast_fit:
            a, b = coast_fit[wheel]
            v = np.linspace(0, 360, 100)
            ax.plot(v, a * v + b * v ** 2, color=colours[wheel], lw=1)
    ax.set_xlabel("speed before PWM 0 (mm/s)")
    ax.set_ylabel("coast distance (mm)")
    ax.set_title("Coast-down", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax = axes[1]
    for s in held:
        tr, x = s["_tr"], s["_x"]
        idx = np.arange(len(tr))
        w = (tr >= -0.1) & (tr <= min(POST_FIT_S, s["hold_s"])) & (idx >= s["_prev"]) & (idx < s["_next"])
        ax.plot(tr[w], windowed_speed(tr[w], x[w], WINDOW_S), color=colours[s["wheel"]], lw=0.5, alpha=0.4)
        tp, vp = s["_pred"]
        ax.plot(tp, vp, color="#222222", lw=0.8, alpha=0.6)
    ax.set_xlabel("time from PWM change (s)")
    ax.set_ylabel("speed (mm/s)")
    ax.set_title("Held-back steps: measured (colour) vs predicted (black)", fontsize=10)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path2 = path.with_name("coast_and_held_back.png")
    fig.savefig(path2, dpi=130)
    plt.close(fig)
    console.print(f"saved {path2}")


def main():
    args = parse_args()
    console = Console(record=True)
    if args.action == "run":
        run(args, console)
    else:
        report(args, console)


if __name__ == "__main__":
    main()
