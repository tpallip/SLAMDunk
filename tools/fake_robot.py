"""Stand-in for the robot, for testing logger.py and teleop.py without hardware.

Replays a saved CSV as telemetry every TELEMETRY_INTERVAL_MS, on a loop, and
prints each command it receives (repeats only once). Commands change the replayed records the way
the firmware would, so the logger shows them: "pwm" sets the pwm columns in
manual mode ("vel" sets them to the firmware's feedforward PWM; there is no
simulated speed loop), "start" sets signal, and manual mode stops after
MANUAL_TIMEOUT_MS without a "pwm" or "vel" command. "ping <id>" is answered
with "#P <id> <t_ms>" and unknown commands with "#E ...", like the firmware.
"test deadband ...", "test keepturn ..." and "test gain ..." run fake versions
of the on-robot tests (FakeDeadband, FakeKeepTurn, FakeGain). "id", "set", "get", "save" and
"config reset" work like the firmware, with the keys, defaults and ranges read
from StampC3_Template/RobotConfig.*.
Run the logger with --host 127.0.0.1 --port 9000.
"""
import argparse
import random
from pathlib import Path
import re
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


def turn(count, wheel, counts):
    """Adds counts to the tested wheel; S (spin) turns the right wheel the other way."""
    if wheel == "S":
        count["L"] += counts
        count["R"] -= counts
    else:
        count[wheel] += counts


class FakeDeadband:
    """Same phases as DeadbandTest_c with a made up wheel, one step per 10 ms."""
    STILL_MS, START_PWM, COAST_MS = 300, {"L": 29, "R": 24, "S": 32}, 150

    def __init__(self, command):
        args = command.split()[2:]
        defaults = ["L", "1", "0", "60", "5", "3", "1500", "1500"]
        args = args + defaults[len(args):]
        self.wheel = args[0].upper()
        self.name = "deadband"
        self.dir, self.lo, self.hi, self.pwm_step, self.repeats, self.rest_ms, self.run_ms = (int(a) for a in args[1:8])
        self.trial, self.repeat, self.level = 0, 1, self.lo
        self.count = {"L": 0, "R": 0}
        self.moving, self.coast_until, self.last_move, self.t = False, 0, 0, 0
        self.rows = 0
        self.trials = self.repeats * ((self.hi - self.lo) // self.pwm_step + 1)
        self.begin_rest()

    def begin_rest(self):
        self.trial += 1
        self.phase, self.phase_start, self.rest_ok = "rest", self.t, 0

    def header(self, t_ms):
        self.t = self.phase_start = self.last_move = t_ms
        return [f"#BH deadband wheel={self.wheel} dir={self.dir} min={self.lo} max={self.hi} step={self.pwm_step} "
                f"repeats={self.repeats} rest_ms={self.rest_ms} run_ms={self.run_ms} still_ms={self.STILL_MS} "
                f"trials={self.trials}",
                "#BC trial,repeat,phase,wheel,pwm,t_ms,enc_us,enc_ok,enc_l,enc_r,rest_ok"]

    def step(self):
        self.t += 10
        if self.phase == "rest":
            resting = self.t - self.phase_start
            still = self.t - self.last_move >= self.STILL_MS
            if resting >= self.rest_ms and (still or resting >= self.rest_ms + 3000):
                self.phase, self.phase_start, self.rest_ok = "run", self.t, int(still)
                threshold = self.START_PWM[self.wheel]
                self.moving = random.random() < min(1, max(0, (self.level - threshold + 3) / 6))
        elif self.t - self.phase_start >= self.run_ms:
            self.level += self.pwm_step
            if self.level > self.hi:
                self.level, self.repeat = self.lo, self.repeat + 1
                if self.repeat > self.repeats:
                    return f"#BE deadband done rows={self.rows} trials={self.trial}"
            self.coast_until = self.t + self.COAST_MS
            self.begin_rest()
        pwm = self.dir * self.level if self.phase == "run" else 0
        delta = 0.0
        if self.phase == "run" and self.moving:
            delta = (self.level - 12) * 0.13
        elif self.t < self.coast_until:
            delta = 1.0
        if delta:
            moved = int(delta + random.random())
            if moved:
                turn(self.count, self.wheel, self.dir * moved)
                self.last_move = self.t
        self.rows += 1
        return (f"#B,{self.trial},{self.repeat},{self.phase},{self.wheel},{pwm},{self.t},{self.t * 1000 + 3000},1,"
                f"{self.count['L']},{self.count['R']},{self.rest_ok if self.phase == 'run' else 0}")


class FakeKeepTurn:
    """Same phases as KeepTurnTest_c with a made up wheel, one step per 10 ms."""
    STILL_MS, STOP_PWM = 300, {"L": 15, "R": 11, "S": 18}

    def __init__(self, command):
        args = command.split()[2:]
        defaults = ["L", "1", "50", "200", "5", "1000", "1500"]
        args = args + defaults[len(args):]
        self.wheel = args[0].upper()
        self.dir, self.start_pwm, self.step_ms, self.trials, self.spin_ms, self.rest_ms = (int(a) for a in args[1:7])
        self.name = "keepturn"
        self.trial, self.t, self.rows, self.last_move = 0, 0, 0, 0
        self.count = {"L": 0, "R": 0}
        self.begin_rest()

    def begin_rest(self):
        self.trial += 1
        self.phase, self.phase_start, self.level = "rest", self.t, self.start_pwm
        self.stop_at = self.STOP_PWM[self.wheel] + random.choice([-1, 0, 0, 1])

    def header(self, t_ms):
        self.t = self.phase_start = self.last_move = t_ms
        return [f"#BH keepturn wheel={self.wheel} dir={self.dir} start={self.start_pwm} step_ms={self.step_ms} "
                f"repeats={self.trials} spin_ms={self.spin_ms} rest_ms={self.rest_ms} still_ms={self.STILL_MS} "
                f"trials={self.trials}",
                "#BC trial,phase,wheel,pwm,t_ms,enc_us,enc_ok,enc_l,enc_r"]

    def step(self):
        self.t += 10
        still = self.t - self.last_move >= self.STILL_MS
        if self.phase == "rest":
            if self.t - self.phase_start >= self.rest_ms and still:
                self.phase, self.phase_start = "spin", self.t
        elif self.phase == "spin":
            if self.t - self.phase_start >= self.spin_ms:
                self.phase, self.phase_start, self.last_step = "ramp", self.t, self.t
        elif still or self.level <= 0:
            if self.trial >= self.trials:
                return f"#BE keepturn done rows={self.rows} trials={self.trial}"
            self.begin_rest()
        elif self.t - self.last_step >= self.step_ms:
            self.level -= 1
            self.last_step += self.step_ms
        pwm = self.dir * self.level if self.phase in ("spin", "ramp") else 0
        if pwm and self.level >= self.stop_at:
            if int((self.level - 12) * 0.13 + random.random()):
                turn(self.count, self.wheel, self.dir)
                self.last_move = self.t
        self.rows += 1
        return (f"#B,{self.trial},{self.phase},{self.wheel},{pwm},{self.t},{self.t * 1000 + 3000},1,"
                f"{self.count['L']},{self.count['R']}")


class FakeGain:
    """Same phases as GainTest_c with a made up wheel, one step per 10 ms.

    Counts per 10 ms are (pwm - 12) * 0.13, falling off a little above 60 PWM.
    """
    STILL_MS = 300

    def __init__(self, command):
        args = command.split()[2:]
        defaults = ["L", "1", "20", "100", "5", "3", "500", "1000", "1500"]
        args = args + defaults[len(args):]
        self.wheel = args[0].upper()
        (self.dir, self.lo, self.hi, self.pwm_step, self.trials,
         self.settle_ms, self.hold_ms, self.rest_ms) = (int(a) for a in args[1:9])
        self.name = "gain"
        self.top = (self.hi - self.lo) // self.pwm_step
        self.trial, self.t, self.rows, self.last_move, self.carry = 0, 0, 0, 0, 0.0
        self.count = {"L": 0, "R": 0}
        self.begin_rest()

    def begin_rest(self):
        self.trial += 1
        self.phase, self.phase_start, self.up, self.index = "rest", self.t, False, self.top

    def header(self, t_ms):
        self.t = self.phase_start = self.last_move = t_ms
        return [f"#BH gain wheel={self.wheel} dir={self.dir} min={self.lo} max={self.hi} step={self.pwm_step} "
                f"repeats={self.trials} settle_ms={self.settle_ms} hold_ms={self.hold_ms} rest_ms={self.rest_ms} "
                f"still_ms={self.STILL_MS} trials={self.trials}",
                "#BC trial,leg,phase,wheel,pwm,t_ms,enc_us,enc_ok,enc_l,enc_r"]

    def step(self):
        self.t += 10
        elapsed = self.t - self.phase_start
        if self.phase == "rest":
            if elapsed >= self.rest_ms and self.t - self.last_move >= self.STILL_MS:
                self.phase, self.phase_start = "settle", self.t
        elif self.phase == "settle":
            if elapsed >= self.settle_ms:
                self.phase, self.phase_start = "hold", self.t
        elif elapsed >= self.hold_ms:
            if not self.up and self.index > 0:
                self.index -= 1
            else:
                self.up = True
                if self.index < self.top:
                    self.index += 1
                elif self.trial >= self.trials:
                    return f"#BE gain done rows={self.rows} trials={self.trial}"
                else:
                    self.begin_rest()
            if self.phase != "rest":
                self.phase, self.phase_start = "settle", self.t
        level = self.lo + self.index * self.pwm_step
        pwm = self.dir * level if self.phase != "rest" else 0
        if pwm and level >= 15:
            rate = (level - 12) * 0.13 * (1 - 0.004 * max(0, level - 60))
            self.carry += rate * random.uniform(0.95, 1.05)
            moved, self.carry = int(self.carry), self.carry - int(self.carry)
            if moved:
                turn(self.count, self.wheel, self.dir * moved)
                self.last_move = self.t
        self.rows += 1
        return (f"#B,{self.trial},{'up' if self.up else 'down'},{self.phase},{self.wheel},{pwm},{self.t},"
                f"{self.t * 1000 + 3000},1,{self.count['L']},{self.count['R']}")


class FakeConfig:
    ROBOT_ID = "SLAMDunk-FAKE"

    def __init__(self):
        firmware = Path(__file__).resolve().parent.parent / "StampC3_Template"
        header = (firmware / "RobotConfig.h").read_text()
        self.defaults = {k: float(v) for k, v in re.findall(r"float (\w+) = (-?[\d.]+)f?;", header)}
        source = (firmware / "RobotConfig.cpp").read_text()
        num = lambda x: self.defaults[x.split(".")[1]] if x.startswith("defaults.") else float(x.rstrip("f"))
        self.ranges = {k: (num(lo), num(hi)) for k, lo, hi in
                       re.findall(r'add\("(\w+)", &v\.\w+, ([^,]+), ([^)]+)\)', source)}
        self.values = dict(self.defaults)
        self.saved = dict(self.defaults)

    def command(self, command):
        """Reply lines for a config command, or None if it isn't one."""
        parts = command.split()
        if command == "id":
            return [f"#ID {self.ROBOT_ID}"]
        if len(parts) == 2 and parts[0] == "name":
            return [f"#I name saved: {parts[1]} (reboot to use it)"]
        if command == "get":
            return [f"#C {k} {self.values[k]:g}" for k in self.ranges]
        if len(parts) == 2 and parts[0] == "get":
            if parts[1] not in self.ranges:
                return [f"#E get: unknown key {parts[1]}"]
            return [f"#C {parts[1]} {self.values[parts[1]]:g}"]
        if len(parts) == 3 and parts[0] == "set":
            key = parts[1]
            if key not in self.ranges:
                return [f"#E set: unknown key {key}"]
            lo, hi = self.ranges[key]
            try:
                value = float(parts[2])
            except ValueError:
                return ["#E usage: set <key> <value> | get [key] | save | config reset | id"]
            if not lo <= value <= hi:
                return [f"#E set: {key} must be {lo:g} to {hi:g}"]
            self.values[key] = value
            return [f"#C {key} {value:g}"]
        if command == "save":
            changed = self.values != self.saved
            self.saved = dict(self.values)
            return [f"#I config {'saved' if changed else 'unchanged, nothing written'}"]
        if command == "config reset":
            self.values = dict(self.defaults)
            return ["#I config reset to defaults (send save to keep it)"]
        return None


def serve(conn, records):
    """Streams records to one client and handles its commands until it leaves."""
    controller = FakeController()
    start = time.monotonic()
    buffer = b""
    last_command = None
    test = None
    config = FakeConfig()
    i = 0
    while True:
        if test is not None:
            lines = []
            while test is not None and test.t + 10 <= int((time.monotonic() - start) * 1000):
                line = test.step()
                lines.append(line)
                if line.startswith("#BE"):
                    test = None
            try:
                conn.sendall("".join(l + "\n" for l in lines).encode())
            except OSError:
                return
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
            try:
                data = conn.recv(4096)
            except OSError:
                return
            if not data:
                return
            buffer += data
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                command = line.decode("utf-8", errors="replace").strip()
                parts = command.split()
                replies = config.command(command)
                if replies is not None:
                    conn.sendall("".join(r + "\n" for r in replies).encode())
                    continue
                fakes = {"deadband": FakeDeadband, "keepturn": FakeKeepTurn, "gain": FakeGain}
                if len(parts) >= 2 and parts[0] == "test" and parts[1] in fakes:
                    test = fakes[parts[1]](command)
                    t_ms = int((time.monotonic() - start) * 1000)
                    conn.sendall(("\n".join(test.header(t_ms)) + "\n").encode())
                    print(f"robot got {command!r} -> fake {parts[1]} test, {test.trials} trials")
                    continue
                if command == "stop" and test is not None:
                    conn.sendall(f"#BE {test.name} stop rows=0 trials=0\n".encode())
                    test = None
                if len(parts) == 2 and parts[0] == "ping" and parts[1].isdigit():
                    t_ms = int((time.monotonic() - start) * 1000)
                    try:
                        conn.sendall(f"#P {parts[1]} {t_ms}\n".encode())
                    except OSError:
                        return
                    continue
                if command:
                    result = controller.handle(command)
                    if result == "unknown command":
                        try:
                            conn.sendall(f"#E unknown command: {command}\n".encode())
                        except OSError:
                            return
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
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # like setNoDelay on the robot
            with conn:
                serve(conn, records)
            print("logger disconnected")
    except KeyboardInterrupt:
        print()
    finally:
        server.close()


if __name__ == "__main__":
    main()
