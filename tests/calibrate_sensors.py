#!/usr/bin/env python3
"""Compute per-sensor white/black calibration limits from trial CSVs.

Each limit is the mean raw reading of that sensor over every sample taken at
0 % blackness (white) or 100 % blackness (black). Means are used rather than
extremes so that a typical white surface reads 0 and a typical black surface
reads 1 after normalisation; isolated noise is clamped instead.

Usage (from the repository root):
    python tests\\calibrate_sensors.py [telemetry.csv ...]

Prints a JSON snippet with white_dn1..5 and black_dn1..5 to paste into
config/robot_params.json (or send with `set` in tcp_logger.py).
"""

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

SENSOR_COUNT = 5
WHITE_PERCENT = 0.0
BLACK_PERCENT = 100.0


def compute_limits(rows):
    """Return {"white_dnN": mean, "black_dnN": mean, ...} from trial rows.

    rows are lists of strings in the trial CSV layout:
    timestamp, trial_no, percentage, reading0..reading4. Header rows (repeated
    before every trial) and malformed rows are skipped.
    """
    samples = {WHITE_PERCENT: [[] for _ in range(SENSOR_COUNT)],
               BLACK_PERCENT: [[] for _ in range(SENSOR_COUNT)]}
    for row in rows:
        if len(row) != 3 + SENSOR_COUNT:
            continue
        try:
            int(row[0])
            percentage = float(row[2])
            readings = [int(value) for value in row[3:]]
        except ValueError:
            continue
        if not math.isfinite(percentage) or percentage not in samples:
            continue
        for sensor, value in enumerate(readings):
            samples[percentage][sensor].append(value)

    limits = {}
    for prefix, percentage in (("white", WHITE_PERCENT), ("black", BLACK_PERCENT)):
        for sensor in range(SENSOR_COUNT):
            values = samples[percentage][sensor]
            if not values:
                raise ValueError("no {} ({:g}%) samples for DN{}".format(
                    prefix, percentage, sensor + 1))
            limits["{}_dn{}".format(prefix, sensor + 1)] = round(statistics.mean(values), 1)

    for sensor in range(1, SENSOR_COUNT + 1):
        if limits["black_dn{}".format(sensor)] <= limits["white_dn{}".format(sensor)]:
            raise ValueError("DN{}: black mean is not above white mean".format(sensor))
    return limits


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", nargs="*", type=Path, default=[Path("telemetry.csv")],
                        help="trial CSV file(s) (default: telemetry.csv)")
    args = parser.parse_args()

    rows = []
    for path in args.csv:
        with path.open(newline="", encoding="utf-8") as handle:
            rows.extend(csv.reader(handle))

    try:
        limits = compute_limits(rows)
    except ValueError as error:
        parser.error(str(error))

    print("Sensor   white(0)   black(1)    range", file=sys.stderr)
    for sensor in range(1, SENSOR_COUNT + 1):
        white = limits["white_dn{}".format(sensor)]
        black = limits["black_dn{}".format(sensor)]
        print("DN{}     {:8.1f}   {:8.1f}  {:7.1f}".format(sensor, white, black, black - white),
              file=sys.stderr)
    print(json.dumps(limits, indent=2))


if __name__ == "__main__":
    main()
