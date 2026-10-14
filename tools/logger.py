import argparse
import csv
from datetime import datetime
from pathlib import Path
import select
import socket
import logging
import time
import errno
import json
from rich.table import Table
from rich.live import Live
from rich.logging import RichHandler
import math
import rerun as rr
import rerun.blueprint as rrb

# Robot: "192.168.4.1", 80 (defaults for --host and --port).
# Local test with tools/fake_robot.py: --host 127.0.0.1 --port 9000.
HOST = "192.168.4.1"
PORT = 80
TIMEOUT_S = 3
USE_RERUN = True          # stream every record to a Rerun viewer
USE_CSV = True            # save every record to a CSV file in LOG_DIR (--csv/--no-csv)
LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"   # <robot id>.json per robot
USE_TELEOP = True         # accept commands from tools/teleop.py (--teleop/--no-teleop)
TELEOP_HOST = "127.0.0.1" # local only, so nobody else on the network can drive the robot
TELEOP_PORT = 9100
MAX_PENDING_PINGS = 1000  # forwarded pings still waiting for the robot's "#P" answer
HEADING_ARROW_MM = 30     # length of the heading arrow drawn at the robot
SURFACE_MAX = 2942        # highest surface reading (black, or robot lifted)
LINE_THRESHOLD = 1400     # firmware default line_threshold, drawn on the profile
PROFILE_HEIGHT = 4        # drawn height of SURFACE_MAX; sensors are 1 unit apart

# Field order sent by Controller_c::publishTelemetry (see Controller.h).
COLUMNS = [
    "t_ms", "x_mm", "y_mm", "theta_rad",
    "pwm_left", "pwm_right", "enc_left", "enc_right",
    "dn1", "dn2", "dn3", "dn4", "dn5", "signal",
]
NUM_FIELDS = len(COLUMNS)

log = logging.getLogger(__name__)


def parse_line(line):
    fields = line.split(",")

    if len(fields) != NUM_FIELDS:
        log.warning("expected %d fields, got %d: %r", NUM_FIELDS, len(fields), line)
        return None

    for field in fields:
        try:
            float(field)
        except ValueError:
            log.warning("not a number %r in: %r", field, line)
            return None

    return fields


def make_table(fields):
    """Builds the dashboard from the latest record, or a placeholder before the first one."""
    table = Table(title="SLAMDunk telemetry")
    table.add_column("Field")
    table.add_column("Value", justify="right")

    if fields is None:
        table.add_row("status", "waiting for data...")
        return table

    for name, value in zip(COLUMNS, fields):
        table.add_row(name, value)
    return table


# Panes in the Rerun layout, by the name used with --use.
PANES = ["path", "pose", "motors", "encoders", "surface", "profile", "bars"]


def make_blueprint(used=PANES):
    """Rerun viewer layout: robot path on the left, time series on the right.

    Only panes named in used start switched on; turn the others on with the
    eye icon in the viewer's blueprint panel.
    """
    def shown(pane):
        return pane in used

    # Show every point up to the time cursor, so the path leaves a trail.
    trail = rrb.VisibleTimeRange(
        "robot_time",
        start=rrb.TimeRangeBoundary.infinite(),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world", name="Path", time_ranges=[trail],
                              visible=shown("path")),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="pose", name="Pose", visible=shown("pose")),
                rrb.TimeSeriesView(origin="motors", name="Motor PWM", visible=shown("motors")),
                rrb.TimeSeriesView(origin="encoders", name="Encoders", visible=shown("encoders")),
                rrb.TimeSeriesView(origin="surface/series", name="Surface sensors",
                                   visible=shown("surface")),
                rrb.Horizontal(
                    rrb.Spatial2DView(origin="surface/profile", name="Surface profile",
                                      visible=shown("profile")),
                    rrb.BarChartView(origin="surface/now", name="Surface now",
                                     visible=shown("bars")),
                ),
            ),
        ),
    )


def log_to_rerun(fields):
    """Sends one validated record to Rerun, stamped with the robot's own clock."""
    values = dict(zip(COLUMNS, (float(f) for f in fields)))

    rr.set_time("robot_time", duration=values["t_ms"] / 1000)

    # Rerun's 2D view has +y pointing down; the robot's +y is anticlockwise
    # (up), so y is negated for drawing only. The pose/ series keep true values.
    x, y, theta = values["x_mm"], values["y_mm"], values["theta_rad"]
    rr.log("world/trail", rr.Points2D([[x, -y]], radii=0.5))
    rr.log("world/robot", rr.Arrows2D(
        origins=[[x, -y]],
        vectors=[[HEADING_ARROW_MM * math.cos(theta), -HEADING_ARROW_MM * math.sin(theta)]],
    ))

    for name in ("x_mm", "y_mm", "theta_rad"):
        rr.log(f"pose/{name}", rr.Scalars(values[name]))
    for name in ("pwm_left", "pwm_right"):
        rr.log(f"motors/{name}", rr.Scalars(values[name]))
    for name in ("enc_left", "enc_right"):
        rr.log(f"encoders/{name}", rr.Scalars(values[name]))

    surface = [values[f"dn{i}"] for i in range(1, 6)]
    for i, reading in enumerate(surface, start=1):
        rr.log(f"surface/series/dn{i}", rr.Scalars(reading))
    rr.log("surface/now", rr.BarChart(surface))

    # Profile across the sensor array: x is the sensor number (dn1 left to dn5
    # right), height is the reading. y is negated so higher readings draw upward.
    profile = [[i, -surface_height(reading)] for i, reading in enumerate(surface, start=1)]
    rr.log("surface/profile/line", rr.LineStrips2D([profile]))
    rr.log("surface/profile/sensors", rr.Points2D(
        profile,
        radii=0.08,
        labels=[f"dn{i} {reading:.0f}" for i, reading in enumerate(surface, start=1)],
    ))


def surface_height(reading):
    """Drawn height of a surface reading on the profile, 0 to PROFILE_HEIGHT."""
    return reading / SURFACE_MAX * PROFILE_HEIGHT


def log_surface_frame(line_threshold=LINE_THRESHOLD):
    """Static outline and threshold line that fix the profile view's scale."""
    top = -PROFILE_HEIGHT
    rr.log("surface/profile/frame", rr.LineStrips2D(
        [[[0.5, 0], [5.5, 0], [5.5, top], [0.5, top], [0.5, 0]]],
        colors=[[128, 128, 128]],
    ), static=True)
    threshold = -surface_height(line_threshold)
    rr.log("surface/profile/threshold", rr.LineStrips2D(
        [[[0.5, threshold], [5.5, threshold]]],
        colors=[[255, 80, 80]],
        labels=[f"line threshold {line_threshold:g}"],
    ), static=True)


class RobotConfig:
    """Sends the robot its config file when the logger connects.

    The logger sends "id". When "#ID <name>" comes back it loads
    config/<name>.json (or --config), and sends "config reset", one
    "set <key> <value>" per entry, "save" and "get". The robot answers
    in order, so each "#C"/"#E" reply is matched to the next key sent. A
    second "id" after the "get" marks the end of the robot's list of values.
    values holds every "#C <key> <value>" seen, so it is what the robot
    is actually using.
    """

    def __init__(self, path=None, enabled=True):
        self.path = path
        self.enabled = enabled
        self.robot_id = None
        self.values = {}
        self.pending = []      # keys sent with "set", waiting for their reply
        self.expected = {}
        self.syncing = False   # from "#ID" until the second "#ID"
        self.problems = 0

    def start(self, sock):
        if self.enabled:
            sock.sendall(b"id\n")

    def load(self):
        path = self.path or CONFIG_DIR / f"{self.robot_id}.json"
        if not Path(path).exists():
            log.warning("no %s: %s keeps its saved settings (or the firmware defaults)", path, self.robot_id)
            return None
        try:
            data = json.loads(Path(path).read_text())
        except (OSError, json.JSONDecodeError) as e:
            log.error("can't read %s: %s; config not sent", path, e)
            return None
        settings = {}
        for key, value in data.items():
            if key.startswith("_"):
                continue   # "_comment" and similar
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                log.warning("%s: %s is not a number, skipped", path, key)
                continue
            settings[key] = value
        log.info("sending %d settings from %s", len(settings), path)
        return settings

    def handle(self, line, sock):
        """Acts on a robot reply; returns True if it shouldn't be printed."""
        text = line.decode("utf-8", errors="replace")
        parts = text.split()
        if parts[:1] == ["#ID"] and len(parts) == 2:
            if self.syncing:
                self.finish()
                return True
            self.robot_id = parts[1]
            log.info("connected to %s", self.robot_id)
            settings = self.load() if self.enabled else None
            commands = []
            if settings is not None:
                self.pending = list(settings)
                self.expected = settings
                commands = ["config reset"] + [f"set {k} {v}" for k, v in settings.items()] + ["save"]
            commands += ["get", "id"]
            self.syncing = True
            sock.sendall("".join(c + "\n" for c in commands).encode())
            return True
        if parts[:1] == ["#C"] and len(parts) == 3:
            try:
                self.values[parts[1]] = float(parts[2])
            except ValueError:
                pass
            if self.pending and parts[1] == self.pending[0]:
                key = self.pending.pop(0)
                if not math.isclose(self.values[key], self.expected[key], rel_tol=1e-5, abs_tol=1e-6):
                    log.warning("config: %s is %s on the robot, file says %s", key, parts[2], self.expected[key])
                    self.problems += 1
            return self.syncing
        if self.syncing and text.startswith("#E") and self.pending:
            key = self.pending.pop(0)
            log.error("config: %s not set: %s", key, text)
            self.problems += 1
            return True
        if self.syncing and text.startswith("#I config"):
            log.info("robot: %s", text)
            return True
        return False

    def finish(self):
        self.syncing = False
        if self.pending:
            log.error("config: no reply for %s", ", ".join(self.pending))
            self.problems += len(self.pending)
            self.pending = []
        log.info("%s is using %d settings%s", self.robot_id, len(self.values),
                 f", {self.problems} problem(s) above" if self.problems else "")
        if USE_RERUN and "line_threshold" in self.values:
            log_surface_frame(self.values["line_threshold"])


class BenchRecorder:
    """Writes each on-robot test (#BH, #BC, #B rows, #BE) to logs/bench_<test>_<time>.csv + .json"""

    def __init__(self, log_dir, robot_config=None):
        self.log_dir = log_dir
        self.robot_config = robot_config
        self.file = None
        self.meta = None
        self.rows = 0

    def handle(self, line):
        if line.startswith(b"#B,"):
            if self.file is not None:
                self.file.write(line[3:].decode("utf-8", errors="replace") + "\n")
                self.rows += 1
            return True
        text = line.decode("utf-8", errors="replace")
        if line.startswith(b"#BH "):
            self.close("no #BE: a new test started")
            parts = text.split()
            test = parts[1] if len(parts) > 1 else "test"
            stem = f"bench_{test}_{datetime.now():%Y%m%d_%H%M%S}"
            self.log_dir.mkdir(exist_ok=True)
            self.path = self.log_dir / f"{stem}.csv"
            self.meta = {"test": test, "header": text, "started": datetime.now().isoformat(timespec="seconds"),
                         "params": dict(p.split("=", 1) for p in parts[2:] if "=" in p)}
            if self.robot_config is not None:
                self.meta["robot"] = self.robot_config.robot_id
                self.meta["config"] = dict(self.robot_config.values)
            self.file = open(self.path, "w", buffering=1)
            self.rows = 0
            log.info("test %s started: saving rows to %s", test, self.path)
            return True
        if line.startswith(b"#BC "):
            if self.file is not None:
                self.file.write(text[4:] + "\n")
            return True
        if line.startswith(b"#BE"):
            self.close(text)
            return True
        return False

    def close(self, end):
        if self.file is None:
            return
        self.file.close()
        self.meta.update(ended=datetime.now().isoformat(timespec="seconds"), end=end, rows_saved=self.rows)
        self.path.with_suffix(".json").write_text(json.dumps(self.meta, indent=2) + "\n")
        log.info("test saved: %s (%d rows; %s)", self.path, self.rows, end)
        self.file = None


class TeleopServer:
    """Local TCP port where tools/teleop.py sends commands to forward to the robot.

    Accepts one teleop client at a time. Commands are forwarded only as complete
    lines, so a teleop that disconnects mid-command can't leave half a command
    on the robot's side to merge with the next one.

    The robot's "#" reply lines are passed back to the client. Lines from the
    client that start with "#" are for the logger, not the robot:
    "#telemetry on" also passes back every telemetry record (tools/motor_tui.py
    uses this), "#telemetry off" stops that.

    "ping <id>" commands are timed: when the robot's "#P <id> <t_ms>" comes
    back, the logger adds the microseconds between forwarding the ping and
    receiving the answer, so the client can split its round trip into the
    logger's share and the robot's (WiFi + controller cycle).
    """

    def __init__(self, host, port, robot):
        self.robot = robot   # "host:port" of the robot, for the greeting
        self.listener = socket.create_server((host, port))
        self.listener.setblocking(False)
        self.client = None
        self.buffer = b""
        self.last_command = None   # last command forwarded, to log only changes
        self.telemetry = False     # pass telemetry records back to the client
        self.pings = {}            # ping id -> time.perf_counter() when forwarded

    def sockets(self):
        """Sockets to watch for incoming connections or commands."""
        return [self.listener] + ([self.client] if self.client else [])

    def accept(self):
        try:
            conn, addr = self.listener.accept()
        except BlockingIOError:
            return
        if self.client is not None:
            conn.sendall(b"busy: another teleop is already connected\n")
            conn.close()
            log.warning("refused a second teleop from %s:%s", *addr)
            return
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # replies go out at once
        self.client = conn
        self.buffer = b""
        conn.sendall(f"connected to logger, robot at {self.robot}\n".encode())
        log.info("teleop connected from %s:%s", *addr)

    def read_commands(self):
        """Complete command lines received from the client, or None if it left."""
        try:
            data = self.client.recv(4096)
        except OSError:
            data = b""
        if not data:
            self.drop_client()
            return None

        self.buffer += data
        *lines, self.buffer = self.buffer.split(b"\n")
        return [line.strip() for line in lines if line.strip()]

    def send(self, line):
        """Passes one line back to the client; drops a client that can't take it."""
        if self.client is None:
            return
        try:
            self.client.sendall(line + b"\n")
        except OSError:
            log.warning("teleop stopped reading: disconnecting it")
            self.drop_client()

    def directive(self, line):
        """Acts on a "#..." line from the client, meant for the logger."""
        if line == b"#telemetry on":
            self.telemetry = True
        elif line == b"#telemetry off":
            self.telemetry = False
        else:
            self.send(b"#E logger: unknown directive " + line)

    def forwarded(self, command, sent_at):
        """Notes the time a "ping <id>" left for the robot."""
        parts = command.split()
        if len(parts) == 2 and parts[0] == b"ping":
            if len(self.pings) >= MAX_PENDING_PINGS:
                self.pings.clear()   # lost answers; don't grow without bound
            self.pings[parts[1]] = sent_at

    def robot_reply(self, line, received_at):
        """Passes a robot "#" line to the client, timing "#P" answers."""
        parts = line.split()
        if len(parts) == 3 and parts[0] == b"#P":
            sent_at = self.pings.pop(parts[1], None)
            if sent_at is not None:
                line += b" %d" % round((received_at - sent_at) * 1e6)
        self.send(line)

    def drop_client(self):
        self.client.close()
        self.client = None
        self.buffer = b""
        self.last_command = None
        self.telemetry = False
        self.pings.clear()

    def close(self):
        if self.client is not None:
            self.drop_client()
        self.listener.close()


def parse_args():
    parser = argparse.ArgumentParser(description="Stream SLAMDunk telemetry to a table and Rerun.")
    parser.add_argument("--host", default=HOST, help=f"robot address (default: {HOST})")
    parser.add_argument("--port", type=int, default=PORT, help=f"robot port (default: {PORT})")
    parser.add_argument(
        "--use", nargs="+", default=PANES, choices=PANES, metavar="PANE",
        help=f"Rerun panes to show (default: all): {', '.join(PANES)}",
    )
    parser.add_argument(
        "--csv", action=argparse.BooleanOptionalAction, default=USE_CSV,
        help=f"save records to a CSV file in {LOG_DIR.name}/",
    )
    parser.add_argument(
        "--name", metavar="NAME",
        help="CSV file name instead of run_<timestamp>, e.g. --name straight_line",
    )
    parser.add_argument(
        "--teleop", action=argparse.BooleanOptionalAction, default=USE_TELEOP,
        help=f"accept commands from tools/teleop.py on {TELEOP_HOST}:{TELEOP_PORT}",
    )
    parser.add_argument(
        "--config", metavar="FILE", type=Path,
        help=f"settings to send on connect (default: {CONFIG_DIR.name}/<robot id>.json)",
    )
    parser.add_argument(
        "--send-config", action=argparse.BooleanOptionalAction, default=True,
        help="send the config file when connecting (--no-send-config leaves the robot as it is)",
    )
    return parser.parse_args()


def csv_path_for(name):
    """CSV path for this run: LOG_DIR/<name>.csv, or a timestamped name by default."""
    if name is None:
        name = f"run_{datetime.now():%Y%m%d_%H%M%S}"
    return LOG_DIR / f"{name.removesuffix('.csv')}.csv"


def main():
    args = parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s",
        handlers=[RichHandler()],
    )

    if args.csv:
        csv_path = csv_path_for(args.name)
        # Checked before connecting, so a clash doesn't cost a connection attempt.
        if csv_path.exists():
            log.error("%s already exists; pick another --name.", csv_path)
            return
    elif args.name is not None:
        log.warning("--name ignored: CSV logging is off.")

    if USE_RERUN:
        rr.init("slamdunk_logger", spawn=True, default_blueprint=make_blueprint(args.use))
        log_surface_frame()

    try:
        sock = socket.create_connection((args.host, args.port), timeout=TIMEOUT_S)
    except ConnectionRefusedError:
        log.error("Server not listening. Robot must be booting.")
        return
    except socket.timeout:
        log.error("robot did not answer within %s s: robot off or out of range.", TIMEOUT_S)
        return
    except OSError as e:
        if e.errno in (errno.EHOSTUNREACH, errno.ENETUNREACH):
            log.error("not on the robot's WiFi (%s)", e)
        else:
            log.exception("unexpected connect error")
        return

    # Send each command at once instead of holding small writes back until the
    # previous one is acknowledged (Nagle), which adds up to a WiFi round trip.
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    buffer = b""
    latest = None
    csv_writer = None
    teleop = None
    robot_config = RobotConfig(args.config, args.send_config)
    bench = BenchRecorder(LOG_DIR, robot_config)
    try:
        robot_config.start(sock)
    except OSError as e:
        log.error("couldn't ask the robot for its id: %s", e)

    if args.teleop:
        try:
            teleop = TeleopServer(TELEOP_HOST, TELEOP_PORT, f"{args.host}:{args.port}")
            log.info("teleop: run tools/teleop.py to send commands (port %s)", TELEOP_PORT)
        except OSError as e:
            log.error("teleop disabled, can't listen on port %s: %s", TELEOP_PORT, e)

    if args.csv:
        LOG_DIR.mkdir(exist_ok=True)
        # Line buffered, so every record is on disk even if the logger is killed.
        csv_file = open(csv_path, "w", newline="", buffering=1)
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(COLUMNS)
        log.info("saving records to %s", csv_path)

    last_data = time.monotonic()

    with Live(make_table(latest), refresh_per_second=10) as live:
        while True:
            # Wait for whichever comes first: robot telemetry, a new teleop
            # connection, or a teleop command. The short timeout lets the
            # no-data check run even when nothing arrives.
            watched = [sock] + (teleop.sockets() if teleop else [])
            readable, _, _ = select.select(watched, [], [], 0.1)

            if teleop is not None:
                if teleop.listener in readable:
                    teleop.accept()
                if teleop.client is not None and teleop.client in readable:
                    commands = teleop.read_commands()
                    try:
                        if commands is None:
                            # Don't leave the robot driving on the last command.
                            log.info("teleop disconnected: sending stop")
                            sock.sendall(b"stop\n")
                        else:
                            for command in commands:
                                if command.startswith(b"#"):
                                    teleop.directive(command)
                                    continue
                                sock.sendall(command + b"\n")
                                teleop.forwarded(command, time.perf_counter())
                                # The teleop resends pwm commands several times a
                                # second, and pings all differ; log only when a
                                # (non-ping) command changes.
                                if command.startswith(b"ping "):
                                    continue
                                if command != teleop.last_command:
                                    log.info("teleop -> robot: %s",
                                             command.decode("utf-8", errors="replace"))
                                teleop.last_command = command
                    except OSError as e:
                        log.error("couldn't send to robot: %s", e)
                        break

            if sock not in readable:
                if time.monotonic() - last_data > TIMEOUT_S:
                    log.error("no data for %s s: robot reset or out of range.", TIMEOUT_S)
                    break
                continue

            try:
                data = sock.recv(4096)
                received_at = time.perf_counter()
                if not data:
                    raise ConnectionError("Closed by the server.")
            except ConnectionError as e:
                log.info("connection ended: %s", e)
                break
            except OSError:
                log.exception("unexpected error while reading")
                break

            last_data = time.monotonic()
            buffer += data
            parts = buffer.split(b"\n")
            for line in parts[:-1]:
                line = line.strip()
                if line.startswith(b"#"):
                    # A reply to a command, not a telemetry record.
                    if teleop is not None:
                        teleop.robot_reply(line, received_at)
                    try:
                        quiet = robot_config.handle(line, sock)
                    except OSError as e:
                        log.error("couldn't send config to robot: %s", e)
                        quiet = False
                    if not quiet and not bench.handle(line) and not line.startswith(b"#P"):
                        log.info("robot: %s", line.decode("utf-8", errors="replace"))
                    continue
                text = line.decode("utf-8", errors="replace")
                fields = parse_line(text)
                if fields is not None:
                    # Passed on first, so Rerun's work doesn't delay the client.
                    if teleop is not None and teleop.telemetry:
                        teleop.send(line)
                    latest = fields
                    if csv_writer is not None:
                        csv_writer.writerow(fields)
                    if USE_RERUN:
                        log_to_rerun(fields)
            buffer = parts[-1]

            live.update(make_table(latest))

    if buffer:
        log.warning("discarding incomplete last line: %r", buffer)

    sock.close()
    bench.close("logger stopped before #BE")
    if teleop is not None:
        teleop.close()
    if csv_writer is not None:
        csv_file.close()
        log.info("records saved to %s", csv_path)
    log.info("logger stopped.")


if __name__ == "__main__":
    main()
