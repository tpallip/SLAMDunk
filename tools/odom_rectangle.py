"""Drive the line course's rectangle and record the odometry (TODO 3.9).

UMBmark-style: straight sides with on-the-spot 90 deg turns, repeated laps,
clockwise or anticlockwise. The rectangle is the centreline of the black
line in DigitalTwin/surface.png (1 px = 1 mm): 301.3 x 212.0 mm. The line
sensors aren't used, so you can watch the robot drift off the line.

Start at the course start, the same as the twin and compare_runs.py: wheel
axle centre on the left line's centreline, halfway along it (where the
dashed line crosses it), sheet landscape with the dashed circle on the left.

    acw: facing down the page (turns left)
    cw:  facing up the page (turns right)

Each lap ends back at the start, facing the way it started.

Run with logger.py going (teleop and hub on), one lap per run, 5 runs each
way (Paul's odometry exercise 4), putting the robot back on the start mark
each time:

    python tools/odom_rectangle.py --dir acw
    python tools/odom_rectangle.py --dir cw

Saves t_ms, x_mm, y_mm, theta_rad from the robot's telemetry to
logs/odom_rectangle/<dir>_01.csv, <dir>_02.csv, ... (next free number).
"""
import argparse
import csv
import socket
import threading
import time
from pathlib import Path

from move_test import wait_for_done

HOST = "127.0.0.1"
TELEOP_PORT, HUB_PORT = 9100, 9200
LONG_MM, SHORT_MM = 301.3, 212.0   # line centreline, measured from surface.png
SPEED, TURN_SPEED = 60, 40
GAP_S = 1.0
LOG_DIR = Path(__file__).resolve().parent.parent / "logs" / "odom_rectangle"


def parse_args():
    parser = argparse.ArgumentParser(description="Drive the course rectangle and record odometry.")
    parser.add_argument("--dir", choices=["acw", "cw"], default="acw")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--speed", type=float, default=SPEED, help=f"move speed mm/s (default {SPEED})")
    parser.add_argument("--turn-speed", type=float, default=TURN_SPEED, help=f"wheel speed in turns, mm/s (default {TURN_SPEED})")
    parser.add_argument("--gap", type=float, default=GAP_S, help=f"pause after each move/turn, s (default {GAP_S})")
    parser.add_argument("--out", type=Path, help="CSV to save (default: logs/odom_rectangle/<dir>_<next number>.csv)")
    return parser.parse_args()


def commands(direction, laps, speed, turn_speed):
    # from the middle of the left line: half of it, then long, short, long,
    # and the left line again (half on the last lap, so it ends at the start)
    turn = f"turn {90 if direction == 'acw' else -90} {turn_speed:g}"
    move = lambda mm: f"move {mm:.1f} {speed:g}"
    out = [move(SHORT_MM / 2)]
    for lap in range(laps):
        last = lap == laps - 1
        out += [turn, move(LONG_MM), turn, move(SHORT_MM), turn, move(LONG_MM), turn,
                move(SHORT_MM / 2 if last else SHORT_MM)]
    return out


def next_file(direction):
    n = 1
    while (LOG_DIR / f"{direction}_{n:02d}.csv").exists():
        n += 1
    return LOG_DIR / f"{direction}_{n:02d}.csv"


def record(sock, path, stop):
    """Writes every telemetry record from the hub until stop is set."""
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ms", "x_mm", "y_mm", "theta_rad"])
        buffer = b""
        sock.settimeout(0.2)
        while not stop.is_set():
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                if line.startswith(b"#"):
                    continue
                fields = line.decode(errors="replace").split(",")
                if len(fields) == 14:   # telemetry record, see logger.COLUMNS
                    writer.writerow(fields[:4])


def main():
    args = parse_args()
    out = args.out or next_file(args.dir)
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        sock = socket.create_connection((HOST, TELEOP_PORT), timeout=3)
        hub = socket.create_connection((HOST, HUB_PORT), timeout=3)
    except OSError as e:
        print(f"can't reach the logger ({e}). Is logger.py running with teleop and hub?")
        return
    greeting = sock.recv(256).decode("utf-8", errors="replace").strip()
    if not greeting.startswith("connected"):
        print(greeting or "logger closed the connection")
        return
    print(greeting)

    stop = threading.Event()
    recorder = threading.Thread(target=record, args=(hub, out, stop))
    recorder.start()
    buffer = bytearray()
    cmds = commands(args.dir, args.laps, args.speed, args.turn_speed)
    try:
        sock.sendall(b"manual\n")
        time.sleep(args.gap)
        for i, command in enumerate(cmds):
            sock.sendall(command.encode() + b"\n")
            # generous deadline: the side at 40 mm/s, twice over, plus 10 s
            result = wait_for_done(sock, buffer, 2 * LONG_MM / 40 + 10)
            print(f"[{i + 1}/{len(cmds)}] {command}: {result or 'no reply: stopping'}")
            if result is None or not result.startswith(("#I move", "#I turn")):
                break
            time.sleep(args.gap)
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        try:
            sock.sendall(b"stop\n")
        except OSError:
            pass
        sock.close()
        time.sleep(0.5)   # catch the last records after the stop
        stop.set()
        recorder.join()
        hub.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
