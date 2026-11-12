#!/usr/bin/env python3
"""Interactive TCP sensor logger for the StampC3 robot controller."""

import argparse
import csv
import math
import socket
import sys
import threading
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


def parse_response(line):
    """Return a parsed trial sample or telemetry record, if present."""
    fields = next(csv.reader([line]))
    if not fields:
        return None

    if fields[0] == "TRIAL_SAMPLE":
        values = fields[1:]
        if len(values) != len(TRIAL_COLUMNS):
            raise ValueError("invalid TRIAL_SAMPLE field count")
        return "trial_sample", values

    if fields[0] == "DATA":
        values = fields[1:]
        if len(values) != len(DATA_COLUMNS):
            raise ValueError("invalid DATA field count")
        return "telemetry", values

    return None


def connect_to_robot(host, port):
    sock = socket.create_connection((host, port), timeout=10)
    sock.settimeout(None)
    return sock


def receive_records(
    sock,
    telemetry_writer,
    telemetry_file,
    output_lock,
    stopped,
    stop_acknowledged,
):
    buffer = bytearray()
    trial_rows_received = 0
    while not stopped.is_set():
        try:
            data = sock.recv(4096)
        except OSError as error:
            if not stopped.is_set():
                print("TCP receive error: {}".format(error), file=sys.stderr)
            break
        if not data:
            if not stopped.is_set():
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
            if not line:
                continue
            try:
                record = parse_response(line)
            except (csv.Error, ValueError) as error:
                print("Invalid robot response {!r}: {}".format(line, error), file=sys.stderr)
                continue

            if line == "ACK,STOP":
                stop_acknowledged.set()
            if line.startswith("TRIAL_STARTED,"):
                trial_rows_received = 0
                with output_lock:
                    telemetry_writer.writerow(TRIAL_COLUMNS)
                    telemetry_file.flush()
                print("< {}".format(line))
                continue
            if record is None:
                print("< {}".format(line))
                continue

            record_type, values = record
            if record_type == "trial_sample":
                trial_rows_received += 1
                with output_lock:
                    telemetry_writer.writerow(values)
                    telemetry_file.flush()
                if (
                    trial_rows_received == 1
                    or trial_rows_received % 50 == 0
                    or trial_rows_received == TRIAL_SAMPLE_COUNT
                ):
                    print(
                        "Trial samples received: {}/{}".format(
                            trial_rows_received,
                            TRIAL_SAMPLE_COUNT,
                        )
                    )
                continue
            print("< {}".format(line))


def build_command(text):
    parts = text.strip().split()
    if not parts:
        return None

    lower_parts = [part.lower() for part in parts]
    command = parts[0].lower()
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
        if not (-100 <= left_pwm <= 100 and -100 <= right_pwm <= 100):
            raise ValueError("motor PWM values must be between -100 and 100")
        return "MOTOR,{},{}\n".format(left_pwm, right_pwm)

    if lower_parts == ["start", "telemetry"]:
        return "START TELEMETRY\n"

    if lower_parts == ["stop", "telemetry"]:
        return "STOP TELEMETRY\n"

    if command in ("status", "stop") and len(parts) == 1:
        return "{}\n".format(command.upper())

    if command in ("help", "?") and len(parts) == 1:
        print(
            "Commands: start telemetry, stop telemetry, trial <trial_no>, "
            "<percentage>, status, motor <left -100..100> "
            "<right -100..100>, stop, quit"
        )
        return None

    if command in ("quit", "exit") and len(parts) == 1:
        return "QUIT\n"

    raise ValueError("unknown command or incorrect arguments; type help")


def main():
    parser = argparse.ArgumentParser(
        description="Collect sensor trials and control telemetry over TCP."
    )
    parser.add_argument("--host", default="192.168.4.1", help="robot access-point IP")
    parser.add_argument("--port", type=int, default=80, help="robot TCP port")
    parser.add_argument("--telemetry-csv", type=Path, default=Path("telemetry.csv"))
    args = parser.parse_args()

    try:
        sock = connect_to_robot(args.host, args.port)
    except OSError as error:
        parser.error("could not connect to {}:{}: {}".format(args.host, args.port, error))

    stopped = threading.Event()
    stop_acknowledged = threading.Event()
    output_lock = threading.Lock()
    with args.telemetry_csv.open("a", newline="", encoding="utf-8") as telemetry_file:
        telemetry_writer = csv.writer(telemetry_file, lineterminator="\n")
        receiver = threading.Thread(
            target=receive_records,
            args=(
                sock,
                telemetry_writer,
                telemetry_file,
                output_lock,
                stopped,
                stop_acknowledged,
            ),
            daemon=True,
        )
        receiver.start()
        print("Connected to {}:{}".format(args.host, args.port))
        print("Trial samples append to: {}".format(args.telemetry_csv))
        print("Type help for commands.")

        try:
            while True:
                try:
                    text = input("> ")
                except EOFError:
                    break
                try:
                    command = build_command(text)
                except (ValueError, OverflowError) as error:
                    print("Invalid command: {}".format(error), file=sys.stderr)
                    continue
                if command is None:
                    continue
                if command == "QUIT\n":
                    break

                try:
                    sock.sendall(command.encode("ascii"))
                except OSError as error:
                    print("TCP send error: {}".format(error), file=sys.stderr)
                    break
        except KeyboardInterrupt:
            print()
        finally:
            stop_acknowledged.clear()
            try:
                sock.sendall(b"STOP\n")
                stop_acknowledged.wait(timeout=0.5)
            except OSError:
                pass
            stopped.set()
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
            receiver.join(timeout=1)


if __name__ == "__main__":
    main()
