"""Stand-in for the robot, for testing logger.py and teleop.py without hardware.

Replays a saved CSV as telemetry every TELEMETRY_INTERVAL_MS, on a loop, and
prints every command it receives. Commands change the replayed records the way
the firmware would, so the logger shows them: "pwm" sets the pwm columns in
manual mode, "start" sets signal. Run the logger with --host 127.0.0.1 --port 9000.
"""
import argparse
import select
import socket
import time

TELEMETRY_INTERVAL_MS = 50   # Controller_c::TELEMETRY_INTERVAL_MS
MANUAL_MAX_PWM = 100.0       # Controller_c::MANUAL_MAX_PWM
PWM_LEFT, PWM_RIGHT, SIGNAL = 4, 5, 13   # column indexes in a telemetry record


def parse_args():
    parser = argparse.ArgumentParser(description="Pretend to be the robot on a local port.")
    parser.add_argument("file", help="CSV to replay, e.g. logs/all_white.csv")
    parser.add_argument("--port", type=int, default=9000, help="port to listen on (default: 9000)")
    return parser.parse_args()


def load_records(path):
    """Telemetry records from a saved CSV, skipping the header and any bad lines."""
    records = []
    with open(path) as f:
        for line in f:
            fields = line.strip().split(",")
            try:
                [float(x) for x in fields]
            except ValueError:
                continue
            records.append(fields)
    return records


class FakeController:
    """The parts of Controller_c's command handling that show up in telemetry."""

    def __init__(self):
        self.manual = False
        self.signal = 0
        self.pwm = (0.0, 0.0)

    def handle(self, command):
        parts = command.split()
        if command == "manual":
            self.manual, self.signal, self.pwm = True, 0, (0.0, 0.0)
        elif command == "auto":
            self.manual, self.signal, self.pwm = False, 0, (0.0, 0.0)
        elif command == "start":
            if not self.manual:
                self.signal = 1
        elif command == "stop":
            if not self.manual:
                self.signal = 0
            self.pwm = (0.0, 0.0)
        elif len(parts) == 3 and parts[0] == "pwm":
            try:
                left, right = float(parts[1]), float(parts[2])
            except ValueError:
                return "unknown command"
            if not self.manual:
                return "ignored: not in manual mode"
            clamp = lambda v: max(-MANUAL_MAX_PWM, min(MANUAL_MAX_PWM, v))
            self.pwm = (clamp(left), clamp(right))
        else:
            return "unknown command"
        return f"mode={'manual' if self.manual else 'auto'} signal={self.signal} pwm={self.pwm[0]:g},{self.pwm[1]:g}"

    def apply(self, fields):
        """A replayed record with this controller's pwm and signal written in."""
        fields = list(fields)
        if self.manual:
            fields[PWM_LEFT], fields[PWM_RIGHT] = f"{self.pwm[0]:g}", f"{self.pwm[1]:g}"
        fields[SIGNAL] = str(self.signal)
        return fields


def serve(conn, records):
    """Streams records to one client and handles its commands until it leaves."""
    controller = FakeController()
    start = time.monotonic()
    buffer = b""
    i = 0
    while True:
        fields = controller.apply(records[i % len(records)])
        fields[0] = str(int((time.monotonic() - start) * 1000))   # own clock, so a replay loop doesn't jump back
        try:
            conn.sendall((",".join(fields) + "\n").encode())
        except OSError:
            return
        i += 1

        readable, _, _ = select.select([conn], [], [], TELEMETRY_INTERVAL_MS / 1000)
        if readable:
            data = conn.recv(4096)
            if not data:
                return
            buffer += data
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                command = line.decode("utf-8", errors="replace").strip()
                if command:
                    print(f"robot got {command!r:<16} -> {controller.handle(command)}")


def main():
    args = parse_args()
    records = load_records(args.file)
    if not records:
        print(f"no telemetry records in {args.file}")
        return

    server = socket.create_server(("127.0.0.1", args.port))
    print(f"fake robot on 127.0.0.1:{args.port}, replaying {len(records)} records from {args.file}")
    try:
        while True:
            print("waiting for the logger...")
            conn, addr = server.accept()
            print(f"logger connected from {addr[0]}:{addr[1]}")
            with conn:
                serve(conn, records)
            print("logger disconnected")
    except KeyboardInterrupt:
        print()
    finally:
        server.close()


if __name__ == "__main__":
    main()
