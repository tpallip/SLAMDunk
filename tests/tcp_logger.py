#!/usr/bin/env python3
"""Interactive TCP logger and controller for the StampC3 robot.

Collects sensor trials, records full-state logs, loads runtime parameters from
JSON, drives the robot, and shows a live in-place telemetry table.
"""

import argparse
import collections
import csv
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


TRIAL_COLUMNS = [
    "timestamp",
    "trial_no",
    "percentage",
    "reading0",
    "reading1",
    "reading2",
    "reading3",
    "reading4",
]
TRIAL_SAMPLE_COUNT = 500
DATA_COLUMNS = [
    "timestamp_ms",
    "x_mm",
    "y_mm",
    "theta_rad",
    "left_pwm",
    "right_pwm",
    "left_encoder",
    "right_encoder",
    "reading0",
    "reading1",
    "reading2",
    "reading3",
    "reading4",
    "signal",
]
LOG_COLUMNS = [
    "timestamp_ms",
    "read_ok_mask",
    "reading0",
    "reading1",
    "reading2",
    "reading3",
    "reading4",
    "left_encoder",
    "right_encoder",
    "left_pwm",
    "right_pwm",
    "odom_x_mm",
    "odom_y_mm",
    "odom_theta_rad",
    "mw_x",
    "mw_y",
    "mw_theta",
    "mode",
    "signal",
    "motion_state",
    "cfg_version",
    "dn1_norm",
    "dn2_norm",
    "dn3_norm",
    "dn4_norm",
    "dn5_norm",
]

# Must match the descriptor table in StampC3_Template/RobotParams.cpp.
KNOWN_PARAMS = [
    "update_interval_ms",
    "telemetry_interval_ms",
    "log_interval_ms",
    "trial_sample_count",
    "recovery_timeout_ms",
    "line_threshold",
    "base_pwm",
    "line_follow_gain",
    "recovery_left_pwm",
    "recovery_right_pwm",
    "motor_pwm_limit",
    "left_wheel_radius_mm",
    "right_wheel_radius_mm",
    "wheel_separation_mm",
    "encoder_counts_per_rev",
    "white_dn1",
    "white_dn2",
    "white_dn3",
    "white_dn4",
    "white_dn5",
    "black_dn1",
    "black_dn2",
    "black_dn3",
    "black_dn4",
    "black_dn5",
]
WHITE_KEYS = ["white_dn{}".format(i) for i in range(1, 6)]
BLACK_KEYS = ["black_dn{}".format(i) for i in range(1, 6)]

DEFAULT_MOTOR_LIMIT = 100
DEFAULT_LINE_THRESHOLD = 0.5
MAX_MOVE_MM = 2000.0
MAX_ROTATE_DEG = 360.0
REPLY_TIMEOUT_S = 1.0

WATCH_LABELS = [
    ("timestamp", "ms"),
    ("x", "mm"),
    ("y", "mm"),
    ("theta", "rad"),
    ("pwm L", ""),
    ("pwm R", ""),
    ("enc L", "counts"),
    ("enc R", "counts"),
    ("DN1", ""),
    ("DN2", ""),
    ("DN3", ""),
    ("DN4", ""),
    ("DN5", ""),
    ("signal", ""),
]
WATCH_BAR_WIDTH = 30

HELP_TEXT = """Robot commands:
  start telemetry | stop telemetry   periodic DATA stream (~50 ms)
  status                             one DATA record now
  trial <trial_no>, <percentage>     collect a sensor trial into the trial CSV
  motor <left> <right>               set motor PWM (within motor_pwm_limit)
  stop                               stop both motors
  move <mm>                          straight-line helper motion
  rotate <deg>                       in-place rotation helper (+ = left)
  selftest                           2 s motor/encoder/sensor self-test (robot spins!)
  odom-reset [x y theta]             reset local odometry pose (mm, mm, rad)
Parameters:
  params                             list all parameters on the robot
  get <name> | set <name> <value>    read or change one parameter (RAM)
  load <file.json>                   send every parameter in a JSON file
  save-config <file.json>            write the robot's current parameters to JSON
  save | defaults                    store to flash / restore compiled defaults
Logging and display:
  log start [file.csv] | log stop    full-state LOG stream to a CSV file
  watch                              live in-place table (n: raw/normalised, Enter or Ctrl+C: exit)
  quit"""


# ---------------------------------------------------------------- parsing ---

def parse_response(line):
    """Return (record_type, values) for a data-bearing line, else None."""
    fields = next(csv.reader([line]))
    if not fields:
        return None

    expected = {
        "TRIAL_SAMPLE": ("trial_sample", TRIAL_COLUMNS),
        "DATA": ("telemetry", DATA_COLUMNS),
        "LOG": ("log", LOG_COLUMNS),
    }
    if fields[0] in expected:
        record_type, columns = expected[fields[0]]
        values = fields[1:]
        if len(values) != len(columns):
            raise ValueError("invalid {} field count".format(fields[0]))
        return record_type, values

    if fields[0] == "PARAM":
        if len(fields) != 3:
            raise ValueError("invalid PARAM field count")
        return "param", [fields[1], fields[2]]

    return None


def json_to_set_commands(config):
    """Convert a flat {name: number} mapping to (name, SET command) pairs.

    Keys beginning with "_" are treated as comments and skipped.
    """
    if not isinstance(config, dict):
        raise ValueError("config must be a JSON object of name: number pairs")

    unknown = [k for k in config if not k.startswith("_") and k not in KNOWN_PARAMS]
    if unknown:
        raise ValueError("unknown parameter(s): {}".format(", ".join(sorted(unknown))))

    commands = []
    for name, value in config.items():
        if name.startswith("_"):
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("{} must be a number, got {!r}".format(name, value))
        if not math.isfinite(value):
            raise ValueError("{} must be finite".format(name))
        commands.append((name, "SET,{},{}\n".format(name, _format_number(value))))
    return commands


def params_from_lines(lines):
    """Build an ordered {name: number} dict from PARAM,<name>,<value> lines."""
    params = {}
    for line in lines:
        record = parse_response(line)
        if record is None or record[0] != "param":
            continue
        name, text = record[1]
        params[name] = _parse_number(text)
    return params


def _format_number(value):
    if isinstance(value, int):
        return str(value)
    return repr(float(value))


def _parse_number(text):
    try:
        return int(text)
    except ValueError:
        return float(text)


def build_command(text, motor_limit=DEFAULT_MOTOR_LIMIT):
    """Translate one typed command into a protocol line (or None)."""
    parts = text.strip().split()
    if not parts:
        return None

    lower_parts = [part.lower() for part in parts]
    command = lower_parts[0]

    if command == "trial":
        payload = text.strip()[len(parts[0]):].strip()
        if "," not in payload:
            raise ValueError("use trial <trial_no>, <percentage>")
        trial_text, percentage_text = payload.split(",", 1)
        trial_number = int(trial_text.strip())
        percentage = float(percentage_text.strip())
        if trial_number < 0:
            raise ValueError("trial number must be non-negative")
        if not math.isfinite(percentage) or not 0 <= percentage <= 100:
            raise ValueError("percentage must be between 0 and 100")
        return "TRIAL,{},{}\n".format(trial_number, percentage)

    if command == "motor" and len(parts) == 3:
        left_pwm = int(parts[1])
        right_pwm = int(parts[2])
        if not (-motor_limit <= left_pwm <= motor_limit
                and -motor_limit <= right_pwm <= motor_limit):
            raise ValueError(
                "motor PWM values must be between -{0} and {0}".format(motor_limit)
            )
        return "MOTOR,{},{}\n".format(left_pwm, right_pwm)

    if lower_parts == ["start", "telemetry"]:
        return "START TELEMETRY\n"
    if lower_parts == ["stop", "telemetry"]:
        return "STOP TELEMETRY\n"
    if len(parts) >= 2 and lower_parts[:2] == ["log", "start"] and len(parts) <= 3:
        return "LOG START\n"
    if lower_parts == ["log", "stop"]:
        return "LOG STOP\n"

    if command in ("status", "stop", "params", "save", "defaults", "selftest") \
            and len(parts) == 1:
        return "{}\n".format(command.upper())

    if command == "get" and len(parts) == 2:
        return "GET,{}\n".format(parts[1])

    if command == "set" and len(parts) == 3:
        value = float(parts[2])
        if not math.isfinite(value):
            raise ValueError("value must be finite")
        return "SET,{},{}\n".format(parts[1], parts[2])

    if command == "move" and len(parts) == 2:
        distance = float(parts[1])
        if not math.isfinite(distance) or abs(distance) > MAX_MOVE_MM:
            raise ValueError("move distance must be within +/-{:g} mm".format(MAX_MOVE_MM))
        return "MOVE,{}\n".format(distance)

    if command == "rotate" and len(parts) == 2:
        degrees = float(parts[1])
        if not math.isfinite(degrees) or abs(degrees) > MAX_ROTATE_DEG:
            raise ValueError("rotation must be within +/-{:g} deg".format(MAX_ROTATE_DEG))
        return "ROTATE,{:.6f}\n".format(math.radians(degrees))

    if command == "odom-reset" and len(parts) in (1, 4):
        if len(parts) == 1:
            return "ODOM_RESET\n"
        x, y, theta = (float(p) for p in parts[1:])
        if not all(math.isfinite(v) for v in (x, y, theta)):
            raise ValueError("pose values must be finite")
        return "ODOM_RESET,{},{},{}\n".format(x, y, theta)

    if command in ("help", "?") and len(parts) == 1:
        print(HELP_TEXT)
        return None

    if command in ("quit", "exit") and len(parts) == 1:
        return "QUIT\n"

    raise ValueError("unknown command or incorrect arguments; type help")


# ------------------------------------------------------- sensor calibration ---

def normalise_readings(raw_values, white, black):
    """Map raw DN readings to 0 (white) .. 1 (black), clamped.

    Mirrors Controller_c::normaliseSurface: a sensor whose black limit is not
    above its white limit reads 0.
    """
    result = []
    for raw, lo, hi in zip(raw_values, white, black):
        span = hi - lo
        if span <= 0:
            result.append(0.0)
            continue
        result.append(min(1.0, max(0.0, (float(raw) - lo) / span)))
    return result


def calibration_from_params(params):
    """Return (white, black) lists from a params dict, or None if incomplete."""
    if not all(key in params for key in WHITE_KEYS + BLACK_KEYS):
        return None
    return ([float(params[k]) for k in WHITE_KEYS],
            [float(params[k]) for k in BLACK_KEYS])


# ---------------------------------------------------------- live display ---

def _bar(fraction, threshold=None):
    """Text bar for a 0..1 fraction with an optional '|' threshold marker."""
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(fraction * WATCH_BAR_WIDTH))
    cells = ["#"] * filled + ["."] * (WATCH_BAR_WIDTH - filled)
    if threshold is not None:
        marker = int(round(max(0.0, min(1.0, threshold)) * (WATCH_BAR_WIDTH - 1)))
        cells[marker] = "|"
    return "".join(cells)


def format_watch_table(values, rate_hz, age_ms, line_threshold=DEFAULT_LINE_THRESHOLD,
                       status="", calibration=None, show_norm=False):
    """Render the 14 DATA values as a fixed-height text table.

    values is the list of 14 strings from a DATA record, or None before the
    first record. calibration is (white, black) per-sensor limits or None.
    With show_norm, DN rows show calibrated 0..1 values instead of raw counts.
    The output always has the same number of lines so it can be redrawn in place.
    """
    width = 64
    mode = "NORMALISED 0-1" if show_norm else "RAW"
    lines = [
        " Live telemetry [{}]   n: raw/normalised   Enter/Ctrl+C: exit".format(mode),
        " " + "-" * width,
    ]

    norm = None
    if values is not None and calibration is not None:
        try:
            norm = normalise_readings([float(v) for v in values[8:13]], *calibration)
        except ValueError:
            norm = None

    for index, (label, unit) in enumerate(WATCH_LABELS):
        raw = values[index] if values is not None else None
        extra = ""
        if raw is None:
            shown = "-"
        else:
            shown = raw
            try:
                number = float(raw)
            except ValueError:
                number = None
            if number is not None:
                if label == "theta":
                    extra = "({:8.2f} deg)".format(math.degrees(number))
                elif label.startswith("DN"):
                    sensor = index - 8
                    if norm is None:
                        # No calibration: plain bar over a nominal raw range.
                        extra = _bar(number / 3000.0)
                        if show_norm:
                            extra += " (no calibration)"
                    else:
                        if show_norm:
                            shown = "{:.3f}".format(norm[sensor])
                        extra = _bar(norm[sensor], line_threshold)
                        if norm[sensor] >= line_threshold:
                            extra += " LINE"
        lines.append(" {:<10}{:>12} {:<7}{}".format(label, shown, unit, extra).rstrip())

    lines.append(" " + "-" * width)
    age_text = "-" if age_ms is None else "{:.0f} ms".format(age_ms)
    footer = " rate {:5.1f} Hz | last sample {} ago".format(rate_hz, age_text)
    if status:
        footer += " | " + status
    lines.append(footer)
    return "\n".join(lines)


def _enable_ansi():
    if os.name == "nt":
        # Empty system() call switches the Windows console into VT mode.
        os.system("")


class _KeyWatcher:
    """Non-blocking check for Enter without leaving a stray stdin reader."""

    def __enter__(self):
        if os.name != "nt" and sys.stdin.isatty():
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if os.name != "nt" and sys.stdin.isatty():
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        return False

    def pressed(self):
        """Return the next pending key as a character, or None."""
        if os.name == "nt":
            import msvcrt
            if msvcrt.kbhit():
                return msvcrt.getwch()
            return None
        import select
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if ready:
            return sys.stdin.read(1)
        return None


# ---------------------------------------------------------------- session ---

class RobotSession:
    """Owns the socket, receiver thread, and every output file."""

    def __init__(self, sock, telemetry_csv, log_dir):
        self.sock = sock
        self.log_dir = log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.output_lock = threading.Lock()
        self.telemetry_csv_path = telemetry_csv
        self.trial_file = telemetry_csv.open("a", newline="", encoding="utf-8")
        self.trial_writer = csv.writer(self.trial_file, lineterminator="\n")
        self.transcript_path = log_dir / "session_{}.txt".format(stamp)
        self.transcript = self.transcript_path.open("a", encoding="utf-8", buffering=1)

        self.log_file = None
        self.log_writer = None
        self.log_path = None
        self.log_rows = 0

        self.stopped = threading.Event()
        self.stop_acknowledged = threading.Event()
        self.replies = queue.Queue()
        self.collecting = False
        self.quiet = False

        self.telemetry_on = False
        self.latest_data = None
        self.latest_data_time = None
        self.data_times = collections.deque(maxlen=100)
        self.params = {}

        self.trial_rows_received = 0
        self.trial_expected = TRIAL_SAMPLE_COUNT

        # While batch_rows is a list, TRIAL samples go there instead of the
        # trial CSV (used by collect_batch / experiments.py).
        self.batch_rows = None
        self.batch_error = None
        self.batch_done = threading.Event()
        self.batch_counter = 0

        self.receiver = threading.Thread(target=self._receive_loop, daemon=True)

    # -- lifecycle --
    def start(self):
        self.receiver.start()

    def close(self):
        self.stop_acknowledged.clear()
        try:
            self.send("STOP\n")
            self.stop_acknowledged.wait(timeout=0.5)
        except OSError:
            pass
        self.stopped.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        self.receiver.join(timeout=1)
        self.close_log_file()
        self.trial_file.close()
        self.transcript.close()

    # -- I/O --
    def _record(self, direction, line):
        with self.output_lock:
            self.transcript.write("{}\t{}\t{}\n".format(
                datetime.now().isoformat(timespec="milliseconds"), direction, line))

    def send(self, command):
        self._record(">", command.rstrip("\n"))
        self.sock.sendall(command.encode("ascii"))

    def echo(self, text):
        if not self.quiet:
            print(text)

    @property
    def motor_limit(self):
        return int(self.params.get("motor_pwm_limit", DEFAULT_MOTOR_LIMIT))

    # -- log CSV --
    def open_log_file(self, path=None):
        self.close_log_file()
        if path is None:
            path = self.log_dir / "log_{}.csv".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not path.exists() or path.stat().st_size == 0
        with self.output_lock:
            self.log_file = path.open("a", newline="", encoding="utf-8")
            self.log_writer = csv.writer(self.log_file, lineterminator="\n")
            if new_file:
                self.log_writer.writerow(LOG_COLUMNS)
            self.log_path = path
            self.log_rows = 0
        return path

    def close_log_file(self):
        with self.output_lock:
            if self.log_file is not None:
                self.log_file.close()
            self.log_file = None
            self.log_writer = None

    # -- receive --
    def _receive_loop(self):
        buffer = bytearray()
        while not self.stopped.is_set():
            try:
                data = self.sock.recv(4096)
            except OSError as error:
                if not self.stopped.is_set():
                    print("TCP receive error: {}".format(error), file=sys.stderr)
                break
            if not data:
                if not self.stopped.is_set():
                    print("Robot closed the TCP connection.", file=sys.stderr)
                break

            buffer.extend(data)
            if len(buffer) > 4096 and b"\n" not in buffer:
                buffer.clear()
                print("Discarded an oversized response line.", file=sys.stderr)
                continue

            while b"\n" in buffer:
                raw_line, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                line = raw_line.rstrip(b"\r").decode("utf-8", errors="replace")
                if line:
                    self._handle_line(line)

    def _handle_line(self, line):
        self._record("<", line)
        try:
            record = parse_response(line)
        except (csv.Error, ValueError) as error:
            print("Invalid robot response {!r}: {}".format(line, error), file=sys.stderr)
            return

        if line == "ACK,STOP":
            self.stop_acknowledged.set()
        if line == "ACK,TELEMETRY,STARTED":
            self.telemetry_on = True
        elif line == "ACK,TELEMETRY,STOPPED":
            self.telemetry_on = False

        batch = self.batch_rows
        if batch is not None:
            if line.startswith("TRIAL_STARTED,"):
                return
            if line.startswith("TRIAL_DONE,"):
                self.batch_done.set()
                return
            if line.startswith("ERROR,"):
                self.batch_error = line
                self.batch_done.set()
                return
            if record is not None and record[0] == "trial_sample":
                values = record[1]
                batch.append((int(values[0]), [int(v) for v in values[3:8]]))
                return

        if line.startswith("TRIAL_STARTED,"):
            fields = line.split(",")
            self.trial_rows_received = 0
            if len(fields) >= 4 and fields[3].isdigit():
                self.trial_expected = int(fields[3])
            with self.output_lock:
                self.trial_writer.writerow(TRIAL_COLUMNS)
                self.trial_file.flush()
            self.echo("< {}".format(line))
            return

        if record is not None:
            record_type, values = record
            if record_type == "trial_sample":
                self._handle_trial_sample(values)
                return
            if record_type == "telemetry":
                now = time.perf_counter()
                self.latest_data = values
                self.latest_data_time = now
                self.data_times.append(now)
                self.echo("< {}".format(line))
                return
            if record_type == "log":
                with self.output_lock:
                    if self.log_writer is not None:
                        self.log_writer.writerow(values)
                        self.log_rows += 1
                return
            if record_type == "param":
                self.params[values[0]] = _parse_number(values[1])

        if line.startswith("ACK,SET,"):
            fields = line.split(",")
            if len(fields) == 4:
                self.params[fields[2]] = _parse_number(fields[3])

        if self.collecting:
            self.replies.put(line)
            return
        self.echo("< {}".format(line))

    def _handle_trial_sample(self, values):
        self.trial_rows_received += 1
        with self.output_lock:
            self.trial_writer.writerow(values)
            self.trial_file.flush()
        n = self.trial_rows_received
        if n == 1 or n % 50 == 0 or n == self.trial_expected:
            self.echo("Trial samples received: {}/{}".format(n, self.trial_expected))

    # -- request/reply helpers --
    def _drain_replies(self):
        while True:
            try:
                self.replies.get_nowait()
            except queue.Empty:
                return

    def _wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                line = self.replies.get(timeout=remaining)
            except queue.Empty:
                return None
            if predicate(line):
                return line

    def _send_set(self, name, command):
        """Send one SET (collecting must be on); return the reply or None."""
        self.send(command)
        return self._wait_for(
            lambda line: line.startswith("ACK,SET,{},".format(name))
            or (line.startswith("ERROR,") and line.endswith("," + name)),
            REPLY_TIMEOUT_S,
        )

    def set_param(self, name, value):
        """Set one parameter and wait for the robot to acknowledge it."""
        self._drain_replies()
        self.collecting = True
        try:
            reply = self._send_set(name, "SET,{},{}\n".format(name, _format_number(value)))
        finally:
            self.collecting = False
        if reply is None or not reply.startswith("ACK,"):
            raise RuntimeError("could not set {}: {}".format(name, reply or "no reply"))

    def collect_batch(self, n, label_pct=0.0):
        """Collect n fresh stationary surface readings via a TRIAL.

        Returns [(timestamp_ms, [r0..r4]), ...]. The firmware stops the motors
        before sampling. Samples bypass the trial CSV.
        """
        if self.params.get("trial_sample_count") != n:
            self.set_param("trial_sample_count", n)
        self.batch_counter += 1
        self.batch_error = None
        self.batch_done.clear()
        self.batch_rows = []
        try:
            self.send("TRIAL,{},{}\n".format(self.batch_counter, float(label_pct)))
            timeout = n * 0.03 + 2.0
            if not self.batch_done.wait(timeout):
                raise TimeoutError("batch did not finish within {:.1f} s".format(timeout))
            rows = self.batch_rows
        finally:
            self.batch_rows = None
        if self.batch_error is not None:
            raise RuntimeError("robot reported {}".format(self.batch_error))
        if len(rows) != n:
            raise RuntimeError("expected {} samples, received {}".format(n, len(rows)))
        return rows

    def load_config(self, path):
        with Path(path).open(encoding="utf-8") as handle:
            config = json.load(handle)
        commands = json_to_set_commands(config)
        ok, failed = 0, []
        self._drain_replies()
        self.collecting = True
        try:
            for name, command in commands:
                reply = self._send_set(name, command)
                if reply is not None and reply.startswith("ACK,"):
                    ok += 1
                else:
                    failed.append("{} ({})".format(name, reply or "no reply"))
        finally:
            self.collecting = False
        print("Loaded {}/{} parameters from {}".format(ok, len(commands), path))
        for item in failed:
            print("  failed: {}".format(item), file=sys.stderr)

    def fetch_params(self):
        """Request the full PARAMS listing and return it as a dict."""
        self._drain_replies()
        self.collecting = True
        lines = []
        try:
            self.send("PARAMS\n")
            while True:
                line = self._wait_for(
                    lambda l: l.startswith("PARAM,") or l.startswith("PARAMS_END,"),
                    REPLY_TIMEOUT_S * 2,
                )
                if line is None:
                    raise TimeoutError("robot did not finish the PARAMS listing")
                if line.startswith("PARAMS_END,"):
                    break
                lines.append(line)
        finally:
            self.collecting = False
        return params_from_lines(lines)

    def save_config(self, path):
        params = self.fetch_params()
        with Path(path).open("w", encoding="utf-8") as handle:
            json.dump(params, handle, indent=2)
            handle.write("\n")
        print("Wrote {} parameters to {}".format(len(params), path))

    def watch(self):
        # Use the robot's own calibration and threshold for the normalised view.
        try:
            self.fetch_params()
        except (TimeoutError, OSError) as error:
            print("Could not read parameters ({}); normalised view unavailable.".format(error),
                  file=sys.stderr)
        calibration = calibration_from_params(self.params)
        threshold = float(self.params.get("line_threshold", DEFAULT_LINE_THRESHOLD))
        show_norm = False

        started_here = False
        if not self.telemetry_on:
            self.send("START TELEMETRY\n")
            started_here = True
        self.quiet = True
        _enable_ansi()
        sys.stdout.write("\x1b[?25l\x1b[2J")
        try:
            with _KeyWatcher() as keys:
                while not self.stopped.is_set():
                    key = keys.pressed()
                    if key in ("\r", "\n", "\x03"):
                        break
                    if key in ("n", "N"):
                        show_norm = not show_norm
                    now = time.perf_counter()
                    recent = [t for t in self.data_times if now - t <= 2.0]
                    rate = 0.0
                    if len(recent) >= 2 and recent[-1] > recent[0]:
                        rate = (len(recent) - 1) / (recent[-1] - recent[0])
                    age = None
                    if self.latest_data_time is not None:
                        age = (now - self.latest_data_time) * 1000.0
                    status = ""
                    if self.log_writer is not None:
                        status = "logging {} rows -> {}".format(self.log_rows, self.log_path.name)
                    table = format_watch_table(
                        self.latest_data, rate, age, threshold, status,
                        calibration, show_norm)
                    sys.stdout.write("\x1b[H" + table.replace("\n", "\x1b[K\n") + "\x1b[K\x1b[J")
                    sys.stdout.flush()
                    time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            sys.stdout.write("\x1b[?25h\n")
            sys.stdout.flush()
            self.quiet = False
            if started_here:
                try:
                    self.send("STOP TELEMETRY\n")
                except OSError:
                    pass


# ------------------------------------------------------------------- main ---

def connect_to_robot(host, port):
    sock = socket.create_connection((host, port), timeout=10)
    sock.settimeout(None)
    return sock


def handle_local_command(session, text):
    """Run commands the logger handles itself. Returns True if handled."""
    parts = text.strip().split()
    if not parts:
        return False
    command = parts[0].lower()

    if command == "load" and len(parts) == 2:
        session.load_config(parts[1])
        return True
    if command == "save-config" and len(parts) == 2:
        session.save_config(parts[1])
        return True
    if command == "watch" and len(parts) == 1:
        session.watch()
        return True
    if [p.lower() for p in parts[:2]] == ["log", "start"] and len(parts) <= 3:
        path = session.open_log_file(parts[2] if len(parts) == 3 else None)
        session.send("LOG START\n")
        print("Logging to {}".format(path))
        return True
    if [p.lower() for p in parts] == ["log", "stop"]:
        session.send("LOG STOP\n")
        rows = session.log_rows
        path = session.log_path
        # Give in-flight LOG lines a moment to land before closing the file.
        time.sleep(0.2)
        session.close_log_file()
        if path is not None:
            print("Stopped logging: {} rows in {}".format(rows, path))
        return True
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Collect trials and logs, tune parameters, and drive the robot over TCP."
    )
    parser.add_argument("--host", default="192.168.4.1", help="robot access-point IP")
    parser.add_argument("--port", type=int, default=80, help="robot TCP port")
    parser.add_argument("--telemetry-csv", type=Path, default=Path("telemetry.csv"),
                        help="trial CSV (appended)")
    parser.add_argument("--log-dir", type=Path, default=Path("logs"),
                        help="directory for session transcripts and LOG CSVs")
    parser.add_argument("--config", type=Path, help="parameter JSON to load on connect")
    parser.add_argument("--watch", action="store_true", help="open the live table on connect")
    args = parser.parse_args()

    try:
        sock = connect_to_robot(args.host, args.port)
    except OSError as error:
        parser.error("could not connect to {}:{}: {}".format(args.host, args.port, error))

    session = RobotSession(sock, args.telemetry_csv, args.log_dir)
    session.start()
    print("Connected to {}:{}".format(args.host, args.port))
    print("Trial samples append to: {}".format(args.telemetry_csv))
    print("Session transcript: {}".format(session.transcript_path))
    print("Type help for commands.")

    try:
        if args.config is not None:
            session.load_config(args.config)
        if args.watch:
            session.watch()

        while True:
            try:
                text = input("> ")
            except EOFError:
                break
            try:
                if handle_local_command(session, text):
                    continue
                command = build_command(text, session.motor_limit)
            except (ValueError, OverflowError, OSError, TimeoutError,
                    json.JSONDecodeError) as error:
                print("Command failed: {}".format(error), file=sys.stderr)
                continue
            if command is None:
                continue
            if command == "QUIT\n":
                break
            try:
                session.send(command)
            except OSError as error:
                print("TCP send error: {}".format(error), file=sys.stderr)
                break
    except KeyboardInterrupt:
        print()
    finally:
        session.close()


if __name__ == "__main__":
    main()
