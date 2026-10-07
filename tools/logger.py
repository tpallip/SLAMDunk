import argparse
import csv
from datetime import datetime
from pathlib import Path
import socket
import logging
import errno
from rich.table import Table
from rich.live import Live
from rich.logging import RichHandler
import math
import rerun as rr
import rerun.blueprint as rrb

# Robot: "192.168.4.1", 80. Local test with nc: "127.0.0.1", 9000.
HOST = "192.168.4.1"
PORT = 80
TIMEOUT_S = 3
USE_RERUN = True          # stream every record to a Rerun viewer
USE_CSV = True            # save every record to a CSV file in LOG_DIR (--csv/--no-csv)
LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
HEADING_ARROW_MM = 30     # length of the heading arrow drawn at the robot
SURFACE_MAX = 2942        # highest surface reading (black, or robot lifted)
LINE_THRESHOLD = 1400     # Controller_c::LINE_THRESHOLD, drawn on the profile
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


def log_surface_frame():
    """Static outline and threshold line that fix the profile view's scale."""
    top = -PROFILE_HEIGHT
    rr.log("surface/profile/frame", rr.LineStrips2D(
        [[[0.5, 0], [5.5, 0], [5.5, top], [0.5, top], [0.5, 0]]],
        colors=[[128, 128, 128]],
    ), static=True)
    threshold = -surface_height(LINE_THRESHOLD)
    rr.log("surface/profile/threshold", rr.LineStrips2D(
        [[[0.5, threshold], [5.5, threshold]]],
        colors=[[255, 80, 80]],
        labels=[f"line threshold {LINE_THRESHOLD}"],
    ), static=True)


def parse_args():
    parser = argparse.ArgumentParser(description="Stream SLAMDunk telemetry to a table and Rerun.")
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
        sock = socket.create_connection((HOST, PORT), timeout=TIMEOUT_S)
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

    buffer = b""
    latest = None
    csv_writer = None

    if args.csv:
        LOG_DIR.mkdir(exist_ok=True)
        # Line buffered, so every record is on disk even if the logger is killed.
        csv_file = open(csv_path, "w", newline="", buffering=1)
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(COLUMNS)
        log.info("saving records to %s", csv_path)

    with Live(make_table(latest), refresh_per_second=10) as live:
        while True:
            try:
                data = sock.recv(4096)
                if not data:
                    raise ConnectionError("Closed by the server.")

            except socket.timeout:
                log.error("no data for %s s: robot reset or out of range.", TIMEOUT_S)
                break
            except ConnectionError as e:
                log.info("connection ended: %s", e)
                break
            except OSError:
                log.exception("unexpected error while reading")
                break

            buffer += data
            parts = buffer.split(b"\n")
            for line in parts[:-1]:
                text = line.decode("utf-8", errors="replace").strip()
                fields = parse_line(text)
                if fields is not None:
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
    if csv_writer is not None:
        csv_file.close()
        log.info("records saved to %s", csv_path)
    log.info("logger stopped.")


if __name__ == "__main__":
    main()
