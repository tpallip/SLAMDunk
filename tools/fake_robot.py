"""Stand-in for the robot, for testing logger.py and teleop.py without hardware.

Replays a saved CSV as telemetry every TELEMETRY_INTERVAL_MS, on a loop, and
prints each command it receives (repeats only once). Commands change the replayed records the way
the firmware would, so the logger shows them: "pwm" sets the pwm columns in
manual mode ("vel" sets them to the firmware's feedforward PWM; there is no
simulated speed loop), "start" sets signal, and manual mode stops after
MANUAL_TIMEOUT_MS without a "pwm" or "vel" command. Run the logger with --host 127.0.0.1 --port 9000.
"""
import argparse
import select
import socket
import time

TELEMETRY_INTERVAL_MS = 50   # Controller_c::TELEMETRY_INTERVAL_MS
MANUAL_MAX_PWM = 100.0       # Controller_c::MANUAL_MAX_PWM
MANUAL_TIMEOUT_MS = 500      # Controller_c::MANUAL_TIMEOUT_MS
MANUAL_MAX_SPEED_MM_S = 250.0   # Controller_c::MANUAL_MAX_SPEED_MM_S
# Controller_c::{LEFT,RIGHT}_FF_OFFSET_PWM and _FF_MM_S_PER_PWM
FEEDFORWARD = ((17.0, 3.5), (12.0, 3.75))
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


def feedforward_pwm(speed_mm_s, offset_pwm, mm_s_per_pwm):
    """The firmware's feedforward PWM for a wheel speed, whole like the robot reports."""
    if speed_mm_s == 0:
        return 0.0
    pwm = offset_pwm + abs(speed_mm_s) / mm_s_per_pwm
    return float(int(pwm if speed_mm_s > 0 else -pwm))


class FakeController:
    """The parts of Controller_c's command handling that show up in telemetry."""

    def __init__(self):
        self.manual = False
        self.signal = 0
        self.pwm = (0.0, 0.0)
        self.pwm_time = 0.0   # time.monotonic() of the last "pwm" or "vel" command

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
        elif len(parts) == 2 and parts[0] == "move":
            if not self.manual:
                return "ignored: not in manual mode"
            self.pwm = (0.0, 0.0)
            return f"move {parts[1]} mm accepted (not simulated: the fake robot doesn't drive)"
        elif len(parts) == 3 and parts[0] in ("pwm", "vel"):
            try:
                left, right = float(parts[1]), float(parts[2])
            except ValueError:
                return "unknown command"
            if not self.manual:
                return "ignored: not in manual mode"
            clamp = lambda v, limit: max(-limit, min(limit, v))
            if parts[0] == "vel":
                self.pwm = tuple(
                    feedforward_pwm(clamp(v, MANUAL_MAX_SPEED_MM_S), *ff)
                    for v, ff in zip((left, right), FEEDFORWARD))
            else:
                self.pwm = (clamp(left, MANUAL_MAX_PWM), clamp(right, MANUAL_MAX_PWM))
            self.pwm_time = time.monotonic()
        else:
            return "unknown command"
        return f"mode={'manual' if self.manual else 'auto'} signal={self.signal} pwm={self.pwm[0]:g},{self.pwm[1]:g}"

    def check_timeout(self):
        """Stops the motors like the firmware does; True if it just did."""
        timed_out = time.monotonic() - self.pwm_time > MANUAL_TIMEOUT_MS / 1000
        if self.manual and self.pwm != (0.0, 0.0) and timed_out:
            self.pwm = (0.0, 0.0)
            return True
        return False

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
    last_command = None
    i = 0
    while True:
        if controller.check_timeout():
            print(f"no pwm for {MANUAL_TIMEOUT_MS} ms -> motors stopped")
            last_command = None
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
                    result = controller.handle(command)
                    # The teleop resends pwm commands; print only changes.
                    if command != last_command:
                        print(f"robot got {command!r:<16} -> {result}")
                    last_command = command


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
