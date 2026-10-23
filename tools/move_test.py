"""Run "move" / "turn" commands and measure where each wheel ends up.

Run (logger recording, teleop on):

    python tools/move_test.py run --cmds "turn 90 40" "turn -90 40" --repeats 3
    python tools/move_test.py report logs/<run>.csv --cmds "turn 90 40" "turn -90 40" --repeats 3

"run" sends each command, waits for the robot's "#I move/turn done" (or
timed out) reply, then waits --gap seconds so the wheels have coasted to a
stop before the next one. The commands are repeated --repeats times, in
order.

"report" finds each motion in the CSV (rows where either wheel has PWM),
takes each wheel's travel from the count before the motion to the count
just before the next one, so the coast is included, and compares it with
the wheel's target: the distance for move, the angle times half of
--separation for turn (the same arithmetic as the firmware). It gives the
end error per wheel, and from the two wheels the heading change, both from
the encoders. Results go to bench/move_test/<csv name>/.
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

HOST, PORT = "127.0.0.1", 9100
GAP_S = 1.5
SEPARATION_MM = 84.0
RADIUS_MM = 16.0
COUNTS_PER_REV = 358.3
OUT_DIR = Path(__file__).resolve().parent.parent / "bench" / "move_test"


def parse_args():
    parser = argparse.ArgumentParser(description="move/turn end error test.")
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("run", "report"):
        p = sub.add_parser(name)
        p.add_argument("--cmds", nargs="+", required=True, help='e.g. "turn 90 40" "move 300 60"')
        p.add_argument("--repeats", type=int, default=1)
    run = sub.choices["run"]
    run.add_argument("--gap", type=float, default=GAP_S, help=f"seconds after each reply (default: {GAP_S})")
    run.add_argument("--port", type=int, default=PORT)
    report = sub.choices["report"]
    report.add_argument("file")
    report.add_argument("--separation", type=float, default=SEPARATION_MM,
                        help=f"wheel_separation_mm the robot used (default: {SEPARATION_MM:g})")
    report.add_argument("--radius", type=float, nargs=2, default=[RADIUS_MM, RADIUS_MM], metavar=("LEFT", "RIGHT"),
                        help=f"wheel radii the robot used, mm (default: {RADIUS_MM:g} {RADIUS_MM:g})")
    report.add_argument("--out", type=Path)
    return parser.parse_args()


def wait_for_done(sock, buffer, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        while b"\n" in buffer:
            line, _, rest = buffer.partition(b"\n")
            buffer[:] = rest
            text = line.decode("utf-8", errors="replace").strip()
            if text.startswith(("#I move ", "#I turn ", "#E")):
                return text
        sock.settimeout(max(0.05, end - time.monotonic()))
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            return "logger closed the connection"
        buffer.extend(chunk)
    return None


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
    commands = args.cmds * args.repeats
    buffer = bytearray()
    try:
        sock.sendall(b"manual\n")
        time.sleep(args.gap)
        for i, command in enumerate(commands):
            sock.sendall(command.encode() + b"\n")
            result = wait_for_done(sock, buffer, 30)
            console.print(f"  [{i + 1}/{len(commands)}] {command}: {result or 'no reply: stopping'}")
            if result is None or not result.startswith(("#I move", "#I turn")):
                break
            time.sleep(args.gap)
    except KeyboardInterrupt:
        console.print("interrupted")
    finally:
        try:
            sock.sendall(b"stop\n")
        except OSError:
            pass
        sock.close()


def targets(command, separation):
    """(left mm, right mm) the firmware aims each wheel at for a move/turn command."""
    parts = command.split()
    value = float(parts[1])
    if parts[0] == "move":
        return value, value
    arc = math.radians(value) * separation / 2
    return -arc, arc


def report(args, console):
    with open(args.file) as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t = col("t_ms") / 1000
    pl, pr = col("pwm_left"), col("pwm_right")
    el, er = col("enc_left"), col("enc_right")
    mm = [2 * math.pi * r / COUNTS_PER_REV for r in args.radius]
    on = (pl != 0) | (pr != 0)
    # stretches with either wheel driven; a wheel can drop to PWM 0 for a moment
    # inside a motion, so stretches closer than 0.3 s are one motion
    motions = []
    edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
    for s_, e_ in zip(edges[::2], edges[1::2]):
        if motions and t[s_] - t[motions[-1][1] - 1] < 0.3:
            motions[-1][1] = e_
        else:
            motions.append([s_, e_])
    commands = args.cmds * args.repeats
    if len(motions) < len(commands):
        console.print(f"[red]found {len(motions)} motions, expected {len(commands)}[/]")
        return
    if len(motions) > len(commands):
        console.print(f"[yellow]found {len(motions)} motions; using the last {len(commands)}[/]")
        motions = motions[-len(commands):]

    table = Table(title=f"End error per wheel, coast included: {Path(args.file).name}")
    for column in ["#", "command", "left target", "left end err", "right target", "right end err", "heading deg (enc)", "coast L / R mm", "time s"]:
        table.add_column(column, justify="right")
    results = []
    for i, ((a, b), command) in enumerate(zip(motions, commands)):
        nxt = motions[i + 1][0] if i + 1 < len(motions) else len(t)
        end = nxt - 1
        tl, tr = targets(command, args.separation)
        dl = (el[end] - el[a - 1]) * mm[0]
        dr = (er[end] - er[a - 1]) * mm[1]
        cl = (el[end] - el[b - 1]) * mm[0]
        cr = (er[end] - er[b - 1]) * mm[1]
        heading = math.degrees((dr - dl) / args.separation)
        r = {"n": i, "command": command, "left_target": tl, "left_err": dl - tl, "right_target": tr,
             "right_err": dr - tr, "heading_deg": heading, "coast_left": abs(cl), "coast_right": abs(cr),
             "time_s": float(t[b - 1] - t[a])}
        results.append(r)
        table.add_row(str(i), command, f"{tl:+.1f}", f"{r['left_err'] * np.sign(tl):+.2f}", f"{tr:+.1f}",
                      f"{r['right_err'] * np.sign(tr):+.2f}", f"{heading:+.2f}",
                      f"{r['coast_left']:.1f} / {r['coast_right']:.1f}", f"{r['time_s']:.2f}")
    console.print(table)
    console.print("End error is along each wheel's direction of travel: + overshot, - stopped short.")

    summary = Table(title="Per command (mean ± sd over repeats)")
    for column in ["command", "n", "left end err mm", "right end err mm", "heading deg (enc)"]:
        summary.add_column(column, justify="right")
    for command in args.cmds:
        rs = [r for r in results if r["command"] == command]
        lerr = np.array([r["left_err"] * np.sign(r["left_target"]) for r in rs])
        rerr = np.array([r["right_err"] * np.sign(r["right_target"]) for r in rs])
        hd = np.array([r["heading_deg"] for r in rs])
        summary.add_row(command, str(len(rs)), f"{lerr.mean():+.2f} ± {lerr.std():.2f}",
                        f"{rerr.mean():+.2f} ± {rerr.std():.2f}", f"{hd.mean():+.2f} ± {hd.std():.2f}")
    console.print(summary)

    out = args.out or OUT_DIR / Path(args.file).stem
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "motions.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0]))
        w.writeheader()
        for r in results:
            w.writerow({k: (f"{v:.3f}" if isinstance(v, float) else v) for k, v in r.items()})
    (out / "summary.txt").write_text(console.export_text())
    console.print(f"saved {out}/")


def main():
    args = parse_args()
    console = Console(record=True)
    if args.action == "run":
        run(args, console)
    else:
        report(args, console)


if __name__ == "__main__":
    main()
