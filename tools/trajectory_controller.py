"""Replay a recorded line-follower run with pure pursuit (path reproduction).

The path is the odometry from a logger CSV, the rows where signal is 1:
from leaving the start pad to the end pad. Put the robot back on the start
pad the same way and run this with logger.py going (teleop and hub on). The
path is placed at the robot's pose when the replay starts, so placement
errors carry through the whole run.

    python tools/trajectory_controller.py logs/run_20261119_231317.csv
    python tools/trajectory_controller.py logs/<run>.csv --speed 120 --lookahead 40

Each telemetry record (10 ms) the robot's odometry pose is compared with the
path: the closest point is tracked forwards only, the lookahead point is the
first one at least --lookahead mm away, and the robot is sent
"cmd <v> <omega>" with omega = v * 2 y / d^2 (y the lookahead point's
sideways offset, d its distance). It speeds up from rest on the same
S-curve as the follower (line_start_ramp_ms in the config). At the end of
the path it sends "stop",
which cuts the PWM like the follower does on the end pad.

The line follower is off (manual mode). The line sensors are still in every
telemetry record, and tools/path_error.py works out from them how far the
robot was off the line. Saves every record with the controller's values to
logs/path_reproduction/replay_NN.csv.
"""
import argparse
import csv
import json
import math
import socket
import threading
import time
from pathlib import Path

import numpy as np

HOST = "127.0.0.1"
TELEOP_PORT, HUB_PORT = 9100, 9200
ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "config" / "lokum.json"
LOG_DIR = ROOT / "logs" / "path_reproduction"
TELEMETRY = ["t_ms", "x_mm", "y_mm", "theta_rad", "pwm_left", "pwm_right", "enc_left", "enc_right",
             "dn1", "dn2", "dn3", "dn4", "dn5", "signal"]
SPEED = 120          # the follower's line_speed_mm_s when the path was recorded
LOOKAHEAD = 50       # mm; longer cuts the 90 mm curves (simulated: 70 gives 4.3 mm rms, 50 gives 2.9)
STEP = 2.0           # path resampled every STEP mm
SEARCH = 100         # closest point searched this many points ahead (200 mm)
MAX_OFF = 50         # stop if the odometry is this far from the path, mm


def parse_args():
    p = argparse.ArgumentParser(description="Pure pursuit replay of a recorded run.")
    p.add_argument("record", type=Path, help="logger CSV of the recorded run")
    p.add_argument("--run", type=int, default=1, help="which signal=1 run in the CSV (default 1)")
    p.add_argument("--speed", type=float, default=SPEED, help=f"mm/s (default {SPEED})")
    p.add_argument("--lookahead", type=float, default=LOOKAHEAD, help=f"mm (default {LOOKAHEAD})")
    p.add_argument("--out", type=Path, help="CSV to save (default logs/path_reproduction/replay_NN.csv)")
    return p.parse_args()


def load_path(path, run):
    """Recorded run's odometry as (x, y) in the frame of its first pose, every STEP mm."""
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f)]
    sig = np.array([float(r["signal"]) for r in rows])
    edges = np.flatnonzero(np.diff(np.r_[0, (sig == 1).astype(int), 0]))
    runs = list(zip(edges[::2], edges[1::2]))
    if not 1 <= run <= len(runs):
        raise SystemExit(f"{path.name} has {len(runs)} runs with signal 1")
    a, b = runs[run - 1]
    x = np.array([float(r["x_mm"]) for r in rows[a:b]])
    y = np.array([float(r["y_mm"]) for r in rows[a:b]])
    th0 = float(rows[a]["theta_rad"])
    c, s = math.cos(th0), math.sin(th0)
    dx, dy = x - x[0], y - y[0]
    local = np.column_stack([c * dx + s * dy, -s * dx + c * dy])
    along = np.r_[0, np.cumsum(np.hypot(*np.diff(local, axis=0).T))]
    keep = np.r_[True, np.diff(along) > 0]   # the robot stands still for a moment at the start
    along, local = along[keep], local[keep]
    at = np.arange(0, along[-1], STEP)
    return np.column_stack([np.interp(at, along, local[:, 0]), np.interp(at, along, local[:, 1])])


def place(path, pose):
    """Path from the start frame into the odometry frame, starting at pose."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([x + c * path[:, 0] - s * path[:, 1], y + s * path[:, 0] + c * path[:, 1]])


class PurePursuit:
    def __init__(self, path, lookahead, separation, min_wheel):
        self.path = path
        self.lookahead = lookahead
        self.separation = separation
        self.min_wheel = min_wheel
        self.i = 0

    def step(self, x, y, th, v):
        """(omega rad/s, values to log), or None once the end is passed."""
        p = self.path
        window = p[self.i:self.i + SEARCH]
        self.i += int(np.argmin(np.hypot(window[:, 0] - x, window[:, 1] - y)))
        # sideways distance to the path, + = robot left of it
        j = min(self.i + 1, len(p) - 1)
        tx, ty = p[j] - p[j - 1]
        norm = math.hypot(tx, ty)
        off = ((x - p[j - 1][0]) * -ty + (y - p[j - 1][1]) * tx) / norm
        # past the end of the path: along the last segment beyond the last point
        ex, ey = p[-1] - p[-2]
        if self.i >= len(p) - 2 and (x - p[-1][0]) * ex + (y - p[-1][1]) * ey >= 0:
            return None
        d = np.hypot(p[self.i:, 0] - x, p[self.i:, 1] - y)
        ahead = np.flatnonzero(d >= self.lookahead)
        k = self.i + (ahead[0] if len(ahead) else len(d) - 1)
        dx, dy = p[k] - (x, y)
        c, s = math.cos(th), math.sin(th)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        omega = v * 2 * ly / max(lx * lx + ly * ly, 1e-6)
        # as the follower: the inner wheel never below line_min_mm_s
        max_omega = max(v - self.min_wheel, 0) / (self.separation / 2)
        omega = max(-max_omega, min(max_omega, omega))
        return omega, {"path_i": self.i, "s_mm": self.i * STEP, "off_mm": off, "look_i": k, "omega_deg_s": math.degrees(omega)}


class Telemetry:
    """Latest telemetry record from the logger's hub, with a wait for the next one."""

    def __init__(self, port):
        self.sock = socket.create_connection((HOST, port), timeout=3)
        self.row = None
        self.count = 0
        self.lock = threading.Condition()
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        buffer = b""
        self.sock.settimeout(None)
        while True:
            try:
                chunk = self.sock.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                if line.startswith(b"#"):
                    text = line.decode(errors="replace").strip()
                    if text.startswith("#E"):
                        print(f"robot: {text}")
                    continue
                fields = line.decode(errors="replace").split(",")
                if len(fields) != len(TELEMETRY):
                    continue
                try:
                    values = [float(v) for v in fields]
                except ValueError:
                    continue
                with self.lock:
                    self.row = dict(zip(TELEMETRY, values))
                    self.count += 1
                    self.lock.notify_all()

    def next(self, timeout=0.5):
        with self.lock:
            seen = self.count
            self.lock.wait_for(lambda: self.count != seen, timeout)
            return self.row if self.count != seen else None


def next_file():
    n = 1
    while (LOG_DIR / f"replay_{n:02d}.csv").exists():
        n += 1
    return LOG_DIR / f"replay_{n:02d}.csv"


def main():
    args = parse_args()
    cfg = json.loads(CONFIG.read_text())
    path = load_path(args.record, args.run)
    print(f"{args.record.name} run {args.run}: path {len(path) * STEP:.0f} mm")

    try:
        sock = socket.create_connection((HOST, TELEOP_PORT), timeout=3)
        telemetry = Telemetry(HUB_PORT)
    except OSError as e:
        print(f"can't reach the logger ({e}). Is logger.py running with teleop and hub?")
        return
    greeting = sock.recv(256).decode("utf-8", errors="replace").strip()
    if not greeting.startswith("connected"):
        print(greeting or "logger closed the connection")
        return
    send = lambda command: sock.sendall(command.encode() + b"\n")

    out = args.out or next_file()
    out.parent.mkdir(parents=True, exist_ok=True)
    extra = ["driving", "path_i", "s_mm", "off_mm", "look_i", "v_cmd", "omega_deg_s"]
    f = open(out, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(TELEMETRY + extra)
    blank = {k: "" for k in extra}

    def log(row, values):
        writer.writerow([f"{row[k]:g}" for k in TELEMETRY] + [values.get(k, "") for k in extra])

    try:
        send("manual")
        # the follower's wheel targets have no ramp (its start S-curve is in
        # the speed it asks for), so the replay's don't either
        send("set accel_mm_s2 0")
        time.sleep(0.5)
        row = telemetry.next(2.0)
        if row is None:
            print("no telemetry on the hub")
            return
        start = (row["x_mm"], row["y_mm"], row["theta_rad"])
        pp = PurePursuit(place(path, start), args.lookahead,
                         cfg["wheel_separation_mm"], cfg.get("line_min_mm_s", 40))
        # S-curve start from rest, the same as the follower's (line_start_ramp_ms)
        ramp_ms = cfg.get("line_start_ramp_ms", 0)
        t_start = row["t_ms"]
        print(f"start pose ({start[0]:.1f}, {start[1]:.1f}) mm, {math.degrees(start[2]):.1f} deg; driving")
        while True:
            row = telemetry.next()
            if row is None:
                print("telemetry stopped")
                break
            v = args.speed
            if ramp_ms > 0 and row["t_ms"] - t_start < ramp_ms:
                k = (row["t_ms"] - t_start) / ramp_ms
                v *= k * k * (3 - 2 * k)
            result = pp.step(row["x_mm"], row["y_mm"], row["theta_rad"], v)
            if result is None:
                send("stop")
                log(row, {**blank, "driving": 0})
                print("end of the path: stopped")
                break
            omega, values = result
            if abs(values["off_mm"]) > MAX_OFF:
                send("stop")
                print(f"{values['off_mm']:.0f} mm off the path at {values['s_mm']:.0f} mm: stopped")
                break
            send(f"cmd {v:.1f} {math.degrees(omega):.2f}")
            log(row, {**values, "driving": 1, "v_cmd": f"{v:.1f}",
                      "off_mm": f"{values['off_mm']:.2f}", "omega_deg_s": f"{values['omega_deg_s']:.2f}"})
        # keep logging while it coasts to a stop
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            row = telemetry.next()
            if row is not None:
                log(row, {**blank, "driving": 0})
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        try:
            send("stop")
            send(f"set accel_mm_s2 {cfg.get('accel_mm_s2', 300):g}")
        except OSError:
            pass
        sock.close()
        f.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
