"""Motor TUI: drive each wheel by typed command, watch the encoders, time the link.

Three ways to reach the robot:

    python tools/motor_tui.py                        # through logger.py (start it first)
    python tools/motor_tui.py --direct               # straight to the robot over WiFi
                                                     # (logger not running: one client only)
    python tools/motor_tui.py --serial /dev/ttyACM0  # over the USB cable

Type commands in the box at the bottom ("help" lists them). Each wheel's
panel shows the PWM asked for, the PWM the robot reports, the encoder count
and the wheel speed over the last SPEED_WINDOW_MS of robot time. Every reply
line from the robot ("#..."), and anything else it prints, goes to the log.

"ping 200" measures the round trip: 200 "ping <id>" commands, each answered
by the robot with "#P <id> <t_ms>" in its next 10 ms controller cycle. Through
the logger the answer also carries the logger's own measurement of the robot
leg, so the round trip splits into the logger's share and the robot's (WiFi +
controller cycle). Results are summarised in the log and saved to
logs/latency_<link>_<time>.csv. A slow ping every LIVE_PING_S also keeps the
Link panel's numbers current.
"""
import argparse
import csv
import queue
import socket
import statistics
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from rich.markup import escape
from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Input, RichLog, Static

LOGGER_HOST, LOGGER_PORT = "127.0.0.1", 9100   # logger's teleop port (logger.py TELEOP_*)
ROBOT_HOST, ROBOT_PORT = "192.168.4.1", 80
SERIAL_BAUD = 115200                           # Serial.begin() in StampC3_Template.ino
CONNECT_TIMEOUT_S = 3
MAX_COMMAND_LEN = 63         # sizeof(Controller_c::cmd_buf) - 1
MANUAL_MAX_PWM = 100         # Controller_c::MANUAL_MAX_PWM; the robot clamps to this
# The robot stops after MANUAL_TIMEOUT_MS (500 ms) without a "pwm", so a moving
# one is resent this often.
RESEND_S = 0.2
SPEED_WINDOW_MS = 100        # wheel speed over this much robot time
STALE_RECORDS = 50           # records checked for "count didn't change while driven"
MM_PER_COUNT = 2 * 3.141592653589793 * 16.0 / 358.3   # OdometryModel_c defaults
LIVE_PING_S = 0.5
LIVE_PINGS_KEPT = 20
PING_COUNT, PING_INTERVAL_MS = 200, 20
PING_LOST_S = 2.0            # an answer later than this counts as lost
LOG_DIR = Path(__file__).resolve().parent.parent / "logs"

# Field order sent by Controller_c::publishTelemetry (as in logger.py).
COLUMNS = [
    "t_ms", "x_mm", "y_mm", "theta_rad",
    "pwm_left", "pwm_right", "enc_left", "enc_right",
    "dn1", "dn2", "dn3", "dn4", "dn5", "signal",
]

HELP = """\
[b]Motors[/b] (switches the robot to manual mode first; resent every 0.2 s while moving)
  l <pwm>          left wheel, e.g. l 40, l -25; the right wheel keeps its value
  r <pwm>          right wheel
  b <pwm>          both wheels the same
  spin <pwm>       left +pwm, right -pwm (turn on the spot)
  pwm <l> <r>      both wheels at once
  stop  (or s, 0)  both to 0 and send "stop"
[b]Measuring[/b]
  zero             restart the "since zero" encoder counts
  ping [n] [ms]    round-trip test: n pings, one every ms (default 200, 20)
  test deadband <L|R> [dir min max step repeats rest_ms run_ms]
                   on-robot deadband test (defaults 1 0 60 5 3 1500 1500);
                   the logger saves its rows to logs/bench_deadband_<time>.csv;
                   stop, the StampC3 button or quit ends it early
  test keepturn <L|R> [dir start step_ms repeats spin_ms rest_ms]
                   keep-turning test (defaults 1 50 200 5 1000 1500): spin,
                   then drop the PWM by 1 every step_ms until the wheel stops
[b]Settings[/b] (see config/ and the README)
  id               the robot's name, which picks config/<name>.json
  get [key]        show one setting, or all of them
  set <key> <val>  change a setting until reboot
  save             keep the current settings over a reboot
  config reset     back to the firmware defaults (then save to keep)
  name <new>       rename the robot (WiFi and config file) from the next boot
[b]Other[/b]
  help, clear, quit (or q, Ctrl+C: stops the motors first)
  anything else is sent to the robot as typed, e.g. manual, auto, start,
  vel 50 50, move 10. Max PWM on the robot: ±100."""


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drive each motor by typed command and watch the encoders.")
    link = parser.add_mutually_exclusive_group()
    link.add_argument("--direct", action="store_true",
                      help=f"connect to the robot itself ({ROBOT_HOST}:{ROBOT_PORT}), not the logger")
    link.add_argument("--serial", metavar="DEVICE",
                      help="use the USB cable, e.g. /dev/ttyACM0 (Windows: COM3)")
    parser.add_argument("--host", help=f"address to connect to (default: {LOGGER_HOST}, "
                                       f"or {ROBOT_HOST} with --direct)")
    parser.add_argument("--port", type=int, help=f"port (default: {LOGGER_PORT}, "
                                                 f"or {ROBOT_PORT} with --direct)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help=f"serial baud rate (default: {SERIAL_BAUD})")
    return parser.parse_args()


class TcpLink:
    """A line-based TCP connection, to the logger's teleop port or the robot."""

    def __init__(self, host, port, via_logger):
        self.name = f"{'logger' if via_logger else 'direct'} {host}:{port}"
        self.kind = "logger" if via_logger else "direct"
        self.sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT_S)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.leftover = b""
        if via_logger:
            # The logger answers first with "connected ..." or "busy ...".
            greeting = self.read_greeting()
            if not greeting.startswith(b"connected"):
                self.sock.close()
                raise ConnectionError(greeting.decode(errors="replace") or "logger closed the connection")
            self.send("#telemetry on")
        self.sock.settimeout(None)

    def read_greeting(self):
        data = b""
        while b"\n" not in data:
            chunk = self.sock.recv(256)
            if not chunk:
                break
            data += chunk
        greeting, _, self.leftover = data.partition(b"\n")
        return greeting.strip()

    def send(self, line):
        self.sock.sendall(line.encode() + b"\n")

    def read_lines(self, on_line):
        """Calls on_line(line, perf_counter time) for each line until the link closes."""
        buffer = self.leftover
        while True:
            try:
                data = self.sock.recv(4096)
            except OSError:
                return
            received_at = time.perf_counter()
            if not data:
                return
            buffer += data
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                on_line(line.strip(), received_at)

    def close(self):
        self.sock.close()


class SerialLink:
    """The robot's USB serial port: telemetry and replies out, commands in."""

    def __init__(self, device, baud):
        import serial   # pyserial; only needed for --serial
        self.name = f"serial {device} @ {baud}"
        self.kind = "serial"
        self.port = serial.Serial()
        self.port.port, self.port.baudrate, self.port.timeout = device, baud, 0.1
        # Keep DTR/RTS low on open, or the USB-serial chip may reset the StampC3.
        self.port.dtr = self.port.rts = False
        self.port.open()
        self.closed = False

    def send(self, line):
        self.port.write(line.encode() + b"\n")
        self.port.flush()

    def read_lines(self, on_line):
        buffer = b""
        while not self.closed:
            try:
                data = self.port.read(self.port.in_waiting or 1)
            except Exception:   # unplugged, or closed by close()
                return
            if not data:
                continue
            received_at = time.perf_counter()
            buffer += data
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                on_line(line.strip(), received_at)

    def close(self):
        self.closed = True
        self.port.close()


def percentile(values, p):
    """p-th percentile (0-100) of a non-empty list, nearest rank."""
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, round(p / 100 * len(ordered)) - 1))]


class Wheel:
    """What the TUI knows about one wheel."""

    def __init__(self, name):
        self.name = name
        self.requested = 0         # PWM asked for by this TUI
        self.reported = None       # PWM in the latest telemetry record
        self.count = None          # latest encoder count
        self.zero = None           # count at the last "zero"
        self.speed = None          # counts/s over SPEED_WINDOW_MS
        self.unchanged = deque(maxlen=STALE_RECORDS)   # per driven record: count didn't move

    def panel(self):
        def num(v, fmt):
            return "-" if v is None else format(v, fmt)
        since = None if self.count is None or self.zero is None else self.count - self.zero
        mm_s = None if self.speed is None else self.speed * MM_PER_COUNT
        stale = (f"{100 * sum(self.unchanged) / len(self.unchanged):.0f} % of last "
                 f"{len(self.unchanged)}" if self.unchanged else "-")
        return (
            f"[b]{self.name} wheel[/b]\n"
            f"requested PWM  {self.requested:+d}\n"
            f"robot PWM      {num(self.reported, '+.0f')}\n"
            f"encoder        {num(self.count, 'd')}\n"
            f"since zero     {num(since, '+d')}\n"
            f"speed          {num(self.speed, '+.0f')} counts/s\n"
            f"               {num(mm_s, '+.1f')} mm/s\n"
            f"no count change while driven\n"
            f"               {stale}"
        )


class PingRun:
    """One "ping N" test: sends N pings at a fixed interval, then summarises."""

    def __init__(self, count, interval_ms):
        self.count, self.interval_ms = count, interval_ms
        self.sent = 0
        self.rows = []             # dicts saved to the CSV
        self.finished_sending_at = None


class MotorTui(App):
    CSS = """
    #panels { height: 11; }
    .panel { width: 1fr; border: round $accent; padding: 0 1; }
    #link { width: 1.4fr; }
    RichLog { border: round $secondary; }
    Input { dock: bottom; }
    """
    BINDINGS = [("ctrl+c", "quit_safely", "Quit")]

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.left, self.right = Wheel("Left"), Wheel("Right")
        self.history = deque()           # (t_ms, enc_left, enc_right) for the speed window
        self.arrivals = deque()          # host times of records, last second
        self.gaps = deque()              # (host time, robot t_ms gap), last 5 s
        self.last_t_ms = None
        self.records = self.bad_lines = 0
        self.manual_sent = False
        self.next_ping_id = 1
        self.pending = {}                # ping id -> (perf_counter sent, PingRun or None)
        self.live = deque(maxlen=LIVE_PINGS_KEPT)   # (rtt_ms, logger_ms or None)
        self.ping_run = None
        self.lost_live = 0
        self.closed = False
        self.test = None
        # Lines from the reader thread, with their arrival time. The reader never
        # waits for the UI, so a busy screen can't delay (and inflate) a ping's
        # arrival time.
        self.incoming = queue.SimpleQueue()

    def compose(self) -> ComposeResult:
        with Horizontal(id="panels"):
            yield Static(id="left", classes="panel")
            yield Static(id="right", classes="panel")
            yield Static(id="link", classes="panel")
        yield RichLog(markup=True, wrap=True)
        yield Input(placeholder="command (help for the list)")

    def on_mount(self):
        self.title = "SLAMDunk motor TUI"
        self.say(f"connected: {self.link.name}. Type [b]help[/b] for commands.")
        threading.Thread(target=self.reader, daemon=True).start()
        self.set_interval(0.02, self.drain)
        self.set_interval(0.1, self.refresh_panels)
        self.set_interval(RESEND_S, self.resend)
        self.set_interval(LIVE_PING_S, self.live_ping)
        self.set_interval(0.05, self.tick_ping_run)
        self.query_one(Input).focus()

    # ---- incoming -------------------------------------------------------

    def reader(self):
        """Reader thread: queues every line with its arrival time until the link closes."""
        self.link.read_lines(lambda line, received_at: self.incoming.put((line, received_at)))
        self.incoming.put((None, None))

    def drain(self):
        """Handles the lines the reader has queued since the last call."""
        while True:
            try:
                line, received_at = self.incoming.get_nowait()
            except queue.Empty:
                return
            if line is None:
                if not self.closed:
                    self.say("[red]link closed[/red]: restart the TUI to reconnect")
                continue
            self.on_line(line, received_at)

    def on_line(self, line, received_at):
        if not line:
            return
        text = line.decode("utf-8", errors="replace")
        if text.startswith("#P"):
            self.on_pong(text, received_at)
        elif text.startswith("#B,"):
            self.on_test_row(text)
        elif text.startswith("#BC"):
            if self.test is not None:
                self.test["columns"] = text[4:].split(",")
        elif text.startswith("#"):
            if text.startswith("#BH"):
                self.test = {"name": text.split()[1] if len(text.split()) > 1 else "test",
                             "trials": next((kv.split("=")[1] for kv in text.split() if kv.startswith("trials=")), "?"),
                             "rows": 0, "row": None, "columns": []}
            elif text.startswith("#BE"):
                self.test = None
            self.say(f"[cyan]robot[/cyan] {escape(text)}")
        elif not self.on_record(text):
            self.bad_lines += 1
            self.say(f"[dim]robot[/dim] {escape(text)}")

    def on_record(self, text):
        fields = text.split(",")
        if len(fields) != len(COLUMNS):
            return False
        try:
            values = dict(zip(COLUMNS, (float(f) for f in fields)))
        except ValueError:
            return False

        now = time.monotonic()
        self.records += 1
        self.arrivals.append(now)
        while self.arrivals and now - self.arrivals[0] > 1.0:
            self.arrivals.popleft()

        t_ms = values["t_ms"]
        if self.last_t_ms is not None and t_ms < self.last_t_ms:
            self.history.clear()   # robot reset
            self.say("[yellow]robot clock went backwards: robot reset?[/yellow]")
        elif self.last_t_ms is not None:
            self.gaps.append((now, t_ms - self.last_t_ms))
        while self.gaps and now - self.gaps[0][0] > 5.0:
            self.gaps.popleft()
        self.last_t_ms = t_ms

        counts = (int(values["enc_left"]), int(values["enc_right"]))
        self.history.append((t_ms, *counts))
        while self.history and t_ms - self.history[0][0] > SPEED_WINDOW_MS:
            self.history.popleft()
        oldest = self.history[0]
        span_s = (t_ms - oldest[0]) / 1000

        for i, (wheel, pwm) in enumerate(((self.left, values["pwm_left"]),
                                          (self.right, values["pwm_right"]))):
            previous = wheel.count
            wheel.count, wheel.reported = counts[i], pwm
            if wheel.zero is None:
                wheel.zero = wheel.count
            wheel.speed = (wheel.count - oldest[1 + i]) / span_s if span_s > 0 else None
            if pwm != 0 and previous is not None:
                wheel.unchanged.append(wheel.count == previous)
        return True

    def on_test_row(self, text):
        if self.test is None:
            return
        self.test["rows"] += 1
        self.test["row"] = text[3:].split(",")

    def on_pong(self, text, received_at):
        parts = text.split()
        try:
            ping_id, robot_ms = int(parts[1]), int(parts[2])
            logger_us = int(parts[3]) if len(parts) > 3 else None
        except (IndexError, ValueError):
            self.say(f"[red]bad ping answer[/red] {escape(text)}")
            return
        entry = self.pending.pop(ping_id, None)
        if entry is None:
            return   # answered after it was counted lost
        sent_at, run = entry
        rtt_ms = (received_at - sent_at) * 1000
        logger_ms = None if logger_us is None else logger_us / 1000
        self.live.append((rtt_ms, logger_ms))
        if run is not None:
            run.rows.append({"id": ping_id, "sent_s": f"{sent_at:.6f}", "rtt_ms": f"{rtt_ms:.3f}",
                             "robot_leg_ms": "" if logger_ms is None else f"{logger_ms:.3f}",
                             "robot_t_ms": robot_ms, "lost": 0,
                             "pwm_left": self.left.requested, "pwm_right": self.right.requested})

    # ---- outgoing -------------------------------------------------------

    def send(self, line):
        try:
            self.link.send(line)
        except OSError as e:
            self.say(f"[red]send failed: {e}[/red]")
            return False
        return True

    def send_pwm(self):
        if not self.manual_sent:
            self.send("manual")
            self.manual_sent = True
        self.send(f"pwm {self.left.requested} {self.right.requested}")

    def resend(self):
        if self.left.requested or self.right.requested:
            self.send_pwm()

    def send_ping(self, run=None):
        ping_id = self.next_ping_id
        self.next_ping_id += 1
        self.pending[ping_id] = (time.perf_counter(), run)
        self.send(f"ping {ping_id}")

    def live_ping(self):
        now = time.perf_counter()
        for ping_id, (sent_at, run) in list(self.pending.items()):
            if run is None and now - sent_at > PING_LOST_S:
                del self.pending[ping_id]
                self.lost_live += 1
        if self.ping_run is None:   # keep the test's timing clean
            self.send_ping()

    # ---- ping test ------------------------------------------------------

    def start_ping_run(self, count, interval_ms):
        if self.ping_run is not None:
            self.say("a ping test is already running")
            return
        self.ping_run = PingRun(count, interval_ms)
        self.ping_timer = self.set_interval(interval_ms / 1000, self.ping_step)
        self.say(f"ping test: {count} pings every {interval_ms} ms over {self.link.name}")

    def ping_step(self):
        run = self.ping_run
        if run is None or run.sent >= run.count:
            return
        self.send_ping(run)
        run.sent += 1
        if run.sent >= run.count:
            self.ping_timer.stop()
            run.finished_sending_at = time.perf_counter()

    def tick_ping_run(self):
        run = self.ping_run
        if run is None or run.finished_sending_at is None:
            return
        waiting = [i for i, (_, r) in self.pending.items() if r is run]
        if waiting and time.perf_counter() - run.finished_sending_at < PING_LOST_S:
            return
        for ping_id in waiting:
            sent_at, _ = self.pending.pop(ping_id)
            run.rows.append({"id": ping_id, "sent_s": f"{sent_at:.6f}", "rtt_ms": "",
                             "robot_leg_ms": "", "robot_t_ms": "", "lost": 1,
                             "pwm_left": self.left.requested, "pwm_right": self.right.requested})
        self.ping_run = None
        self.report_ping_run(run)

    def report_ping_run(self, run):
        rows = sorted(run.rows, key=lambda r: r["id"])
        rtts = [float(r["rtt_ms"]) for r in rows if not r["lost"]]
        legs = [float(r["robot_leg_ms"]) for r in rows if r["robot_leg_ms"] != ""]
        lost = sum(r["lost"] for r in rows)

        def summary(name, values):
            if not values:
                return f"  {name:<12} no data"
            return (f"  {name:<12} median {statistics.median(values):6.2f}  "
                    f"p95 {percentile(values, 95):6.2f}  p99 {percentile(values, 99):6.2f}  "
                    f"max {max(values):6.2f}  min {min(values):6.2f} ms")

        lines = [f"[b]ping test done[/b]: {len(rtts)}/{len(rows)} answered, {lost} lost, "
                 f"{self.link.name}, telemetry {len(self.arrivals)} rec/s",
                 summary("round trip", rtts)]
        if legs:
            # Per ping: the logger's share is the round trip minus its robot leg.
            shares = [float(r["rtt_ms"]) - float(r["robot_leg_ms"])
                      for r in rows if r["robot_leg_ms"] != "" and not r["lost"]]
            lines += [summary("robot leg", legs), summary("logger share", shares)]

        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / f"latency_{self.link.kind}_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["id"])
            writer.writeheader()
            writer.writerows(rows)
        lines.append(f"  saved to {path.relative_to(LOG_DIR.parent)}")
        self.say("\n".join(lines))

    # ---- commands -------------------------------------------------------

    def on_input_submitted(self, event: Input.Submitted):
        command = event.value.strip()
        event.input.value = ""
        if command:
            self.say(f"[b]>[/b] {escape(command)}")
            self.handle(command)

    def handle(self, command):
        parts = command.split()
        word, args = parts[0].lower(), parts[1:]

        def pwm_arg(i):
            value = int(round(float(args[i])))
            if abs(value) > MANUAL_MAX_PWM:
                self.say(f"[yellow]{value} is above ±{MANUAL_MAX_PWM}; the robot will clamp it[/yellow]")
            return value

        try:
            if word in ("help", "?"):
                self.say(HELP)
            elif word in ("quit", "q", "exit"):
                self.action_quit_safely()
            elif word == "clear":
                self.query_one(RichLog).clear()
            elif word in ("stop", "s", "0"):
                self.left.requested = self.right.requested = 0
                self.send("stop")
            elif word == "l" and len(args) == 1:
                self.left.requested = pwm_arg(0)
                self.send_pwm()
            elif word == "r" and len(args) == 1:
                self.right.requested = pwm_arg(0)
                self.send_pwm()
            elif word == "b" and len(args) == 1:
                self.left.requested = self.right.requested = pwm_arg(0)
                self.send_pwm()
            elif word == "spin" and len(args) == 1:
                self.left.requested = pwm_arg(0)
                self.right.requested = -self.left.requested
                self.send_pwm()
            elif word == "pwm" and len(args) == 2:
                self.left.requested, self.right.requested = pwm_arg(0), pwm_arg(1)
                self.send_pwm()
            elif word == "zero":
                self.left.zero, self.right.zero = self.left.count, self.right.count
            elif word == "ping":
                count = int(args[0]) if args else PING_COUNT
                interval_ms = int(args[1]) if len(args) > 1 else PING_INTERVAL_MS
                if count < 1 or interval_ms < 1:
                    raise ValueError
                self.start_ping_run(count, interval_ms)
            else:
                # Robot commands: manual, auto, start, vel, move, ...
                if len(command) > MAX_COMMAND_LEN:
                    self.say(f"too long ({len(command)} chars); the robot keeps {MAX_COMMAND_LEN}")
                    return
                if word in ("vel", "move", "manual", "test"):
                    self.manual_sent = True
                    self.left.requested = self.right.requested = 0   # the robot drives now
                elif word == "auto":
                    self.manual_sent = False
                    self.left.requested = self.right.requested = 0
                self.send(command)
        except ValueError:
            self.say(f"[red]bad arguments[/red]: {escape(command)} (help lists the commands)")

    def action_quit_safely(self):
        if not self.closed:
            self.closed = True
            self.left.requested = self.right.requested = 0
            self.send("stop")
            self.link.close()
        self.exit()

    # ---- display --------------------------------------------------------

    def say(self, text):
        self.query_one(RichLog).write(text)

    def refresh_panels(self):
        self.query_one("#left", Static).update(self.left.panel())
        self.query_one("#right", Static).update(self.right.panel())
        self.query_one("#link", Static).update(self.link_panel())

    def link_panel(self):
        max_gap = max((g for _, g in self.gaps), default=None)
        lines = [
            f"[b]Link[/b]  {escape(self.link.name)}",
            f"telemetry   {len(self.arrivals)} rec/s, max gap "
            + ("-" if max_gap is None else f"{max_gap:.0f} ms") + " (5 s)",
            f"robot t     {'-' if self.last_t_ms is None else f'{self.last_t_ms:.0f} ms'}",
            f"records     {self.records} ok, {self.bad_lines} other lines",
        ]
        if self.live:
            rtts = [r for r, _ in self.live]
            lines.append(f"ping        last {rtts[-1]:.1f}  median {statistics.median(rtts):.1f}  "
                         f"max {max(rtts):.1f} ms")
            legs = [l for _, l in self.live if l is not None]
            if legs:
                lines.append(f"  robot leg median {statistics.median(legs):.1f} ms "
                             f"(rest is the logger)")
        else:
            lines.append("ping        waiting for an answer...")
        lines.append(f"lost pings  {self.lost_live}")
        if self.ping_run is not None:
            lines.append(f"[b]ping test[/b]   {self.ping_run.sent}/{self.ping_run.count} sent")
        if self.test is not None:
            row = dict(zip(self.test["columns"], self.test["row"] or []))
            if row:
                where = f"trial {row.get('trial', '?')}/{self.test['trials']} {row.get('phase', '')} " \
                        f"{row.get('wheel', '')} pwm {row.get('pwm', '?')}"
            else:
                where = "starting"
            lines.append(f"[b]test {escape(self.test['name'])}[/b] {escape(where)}, {self.test['rows']} rows")
        return "\n".join(lines)


def open_link(args):
    if args.serial:
        return SerialLink(args.serial, args.baud)
    if args.direct:
        return TcpLink(args.host or ROBOT_HOST, args.port or ROBOT_PORT, via_logger=False)
    return TcpLink(args.host or LOGGER_HOST, args.port or LOGGER_PORT, via_logger=True)


def main():
    args = parse_args()
    try:
        link = open_link(args)
    except ImportError:
        print("--serial needs pyserial: pip install pyserial")
        return
    except (OSError, ConnectionError) as e:
        where = "the robot's USB port" if args.serial else (
            "the robot (is the logger holding its one connection?)" if args.direct
            else "the logger (is logger.py running?)")
        print(f"can't reach {where}: {e}")
        return
    MotorTui(link).run()


if __name__ == "__main__":
    main()
