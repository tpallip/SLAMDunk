#!/usr/bin/env python3
"""Guided multi-trial reflectance-sensor experiments (course exercises 4-7).

Walks the operator through every placement / position, collects a stationary
batch of fresh surface readings for each, and appends them to a long-format
CSV that follows the course column layout (plus a few extra columns):

    test_name,surface_value,trial_num,sample_num,sample_time,sensor_num,
    sensor_reading,blackness_pct,role,position_mm,rectangle_width_mm,direction

Protocols:
    sweep      gain + non-linearity (Ex 4/5): fit and held-back blackness %
    noise      measurement noise (Ex 6): repeated long batches at fixed points
    footprint  spatial footprint (Ex 7): strip traverses in mm steps

Run from the repository root, e.g.:
    python tests\\experiments.py sweep --config config\\robot_params.json
"""

import argparse
import csv
import json
import random
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tcp_logger import RobotSession, connect_to_robot  # noqa: E402

SENSOR_COUNT = 5
CSV_COLUMNS = [
    "test_name",
    "surface_value",
    "trial_num",
    "sample_num",
    "sample_time",
    "sensor_num",
    "sensor_reading",
    "blackness_pct",
    "role",
    "position_mm",
    "rectangle_width_mm",
    "direction",
]
DN_MAPPING_NOTE = ("reading index i is recorded as sensor_num i+1 (DN1..DN5); "
                   "verify physically which index is the left-most sensor")


@dataclass
class Step:
    """One stationary batch to collect."""
    test_name: str
    prompt: str
    blackness_pct: float
    role: str
    trial_num: int
    samples: int
    position_mm: float = None
    rectangle_width_mm: float = None
    direction: str = ""


def fill_value(blackness_pct):
    """Course surface_value: RGB fill 0..255, where 0 is black and 255 white."""
    return int(round(255 * (1 - blackness_pct / 100.0)))


def _fmt(value):
    if value is None:
        return ""
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


def batch_rows(step, batch):
    """Expand one batch [(timestamp_ms, [r0..r4]), ...] into long CSV rows."""
    rows = []
    for sample_num, (timestamp_ms, readings) in enumerate(batch, start=1):
        for index, reading in enumerate(readings):
            rows.append([
                step.test_name,
                fill_value(step.blackness_pct),
                step.trial_num,
                sample_num,
                timestamp_ms,
                index + 1,
                reading,
                _fmt(step.blackness_pct),
                step.role,
                _fmt(step.position_mm),
                _fmt(step.rectangle_width_mm),
                step.direction,
            ])
    return rows


# -------------------------------------------------------------- protocols ---

def sweep_steps(fit, holdout, placements, samples, seed):
    """Every condition once per placement pass, in a shuffled order per pass."""
    rng = random.Random(seed)
    steps = []
    for placement in range(1, placements + 1):
        conditions = [(pct, "fit") for pct in fit] + [(pct, "holdout") for pct in holdout]
        rng.shuffle(conditions)
        for pct, role in conditions:
            steps.append(Step(
                test_name="surfaceSweep",
                prompt="Placement pass {}/{}: lift and place the robot with all sensors on the "
                       "{:g}% patch ({})".format(placement, placements, pct, role),
                blackness_pct=pct, role=role, trial_num=placement, samples=samples))
    return steps


def noise_steps(points, placements, samples):
    """Long batches at fixed operating points; the last placement is held back."""
    steps = []
    for pct in points:
        for placement in range(1, placements + 1):
            role = "holdout" if placement == placements else "fit"
            steps.append(Step(
                test_name="noiseTest",
                prompt="{:g}% patch, placement {}/{} ({}): lift, re-place, keep still for "
                       "{:.0f} s".format(pct, placement, placements, role, samples * 0.01),
                blackness_pct=pct, role=role, trial_num=placement, samples=samples))
    return steps


def footprint_positions(start, stop, step):
    count = int(round((stop - start) / step))
    return [round(start + i * step, 6) for i in range(count + 1)]


def footprint_steps(sensor, widths, positions, samples, reference_samples,
                    directions, repeats):
    steps = [
        Step("footprintWhiteReference",
             "Reference: put sensor DN{} over plain WHITE paper".format(sensor),
             0.0, "reference", 0, reference_samples),
        Step("footprintBlackReference",
             "Reference: put sensor DN{} over a SOLID BLACK area".format(sensor),
             100.0, "reference", 0, reference_samples),
    ]
    trial = 0
    for width in widths:
        for repeat in range(1, repeats + 1):
            for direction in directions:
                trial += 1
                ordered = positions if direction == "increasing" else list(reversed(positions))
                for index, position in enumerate(ordered):
                    lead = ""
                    if index == 0:
                        lead = "NEW TRAVERSE (realign robot square to the strip). "
                    steps.append(Step(
                        test_name="footprintTest",
                        prompt="{}{:g} mm strip, traverse {} ({}, repeat {}): move DN{} to "
                               "{:+g} mm from the strip centre".format(
                                   lead, width, trial, direction, repeat, sensor, position),
                        blackness_pct=0.0, role="fit", trial_num=trial, samples=samples,
                        position_mm=position, rectangle_width_mm=width, direction=direction))
    return steps


# ----------------------------------------------------------------- runner ---

def describe_batch(batch, highlight=None):
    parts = []
    for index in range(SENSOR_COUNT):
        values = [readings[index] for _, readings in batch]
        median = statistics.median(values)
        sd = statistics.stdev(values) if len(values) > 1 else 0.0
        mark = "*" if highlight == index + 1 else " "
        parts.append("{}DN{} {:6.0f} sd {:<4.0f}".format(mark, index + 1, median, sd))
    return " ".join(parts)


def run_steps(session, steps, writer, handle, settle_s=0.5, input_fn=input,
              highlight=None):
    """Prompt for and collect each step. Returns the number of batches written.

    The most recent batch is held back until the operator moves on, so 'r'
    can discard and redo it without leaving bad rows in the CSV.
    """
    pending = None
    written = 0
    index = 0

    def flush():
        nonlocal pending, written
        if pending is not None:
            writer.writerows(batch_rows(*pending))
            handle.flush()
            written += 1
            pending = None

    while index < len(steps):
        step = steps[index]
        answer = input_fn("[{}/{}] {}\n    Enter=collect, r=redo previous, q=quit > ".format(
            index + 1, len(steps), step.prompt)).strip().lower()
        if answer == "q":
            break
        if answer == "r":
            if pending is None:
                print("    Nothing to redo.")
                continue
            pending = None
            index -= 1
            print("    Previous batch discarded.")
            continue

        flush()
        time.sleep(settle_s)
        try:
            batch = session.collect_batch(step.samples, step.blackness_pct)
        except (RuntimeError, TimeoutError, OSError) as error:
            print("    Collection failed: {} (try again)".format(error))
            continue
        pending = (step, batch)
        print("    {} samples: {}".format(len(batch), describe_batch(batch, highlight)))
        index += 1

    flush()
    return written


def parse_list(text):
    return [float(item) for item in text.split(",") if item.strip()]


def main():
    parser = argparse.ArgumentParser(description="Guided reflectance-sensor experiments.")
    parser.add_argument("protocol", choices=["sweep", "noise", "footprint"])
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=80)
    parser.add_argument("--config", type=Path, help="parameter JSON to load first")
    parser.add_argument("--out-dir", type=Path, default=Path("data/experiments"))
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    parser.add_argument("--settle", type=float, default=0.5,
                        help="seconds to wait after Enter before sampling")
    parser.add_argument("--note", default="", help="free-text note stored in the metadata")
    parser.add_argument("--samples", type=int, help="samples per batch (protocol default)")
    parser.add_argument("--placements", type=int, default=3)
    # sweep
    parser.add_argument("--fit", default="0,20,40,60,80,100", help="fit blackness %% list")
    parser.add_argument("--holdout", default="10,50,90", help="held-back blackness %% list")
    parser.add_argument("--seed", type=int, help="shuffle seed (default: time based)")
    # noise
    parser.add_argument("--points", default="20,60", help="noise operating points (%%)")
    # footprint
    parser.add_argument("--sensor", type=int, default=3, choices=range(1, 6))
    parser.add_argument("--widths", default="14,10,6,3,1", help="strip widths in mm")
    parser.add_argument("--from", dest="start", type=float, default=-20.0)
    parser.add_argument("--to", dest="stop", type=float, default=20.0)
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--reference-samples", type=int, default=200)
    parser.add_argument("--direction", choices=["increasing", "decreasing", "both"],
                        default="increasing")
    parser.add_argument("--repeats", type=int, default=1)
    args = parser.parse_args()

    seed = args.seed if args.seed is not None else int(time.time())
    highlight = None
    if args.protocol == "sweep":
        steps = sweep_steps(parse_list(args.fit), parse_list(args.holdout),
                            args.placements, args.samples or 100, seed)
    elif args.protocol == "noise":
        steps = noise_steps(parse_list(args.points), args.placements, args.samples or 1000)
    else:
        directions = (["increasing", "decreasing"] if args.direction == "both"
                      else [args.direction])
        steps = footprint_steps(
            args.sensor, parse_list(args.widths),
            footprint_positions(args.start, args.stop, args.step),
            args.samples or 50, args.reference_samples, directions, args.repeats)
        highlight = args.sensor

    total_s = sum(step.samples for step in steps) * 0.01
    print("{} batches, about {:.0f} s of sampling plus handling time.".format(
        len(steps), total_s))

    try:
        sock = connect_to_robot(args.host, args.port)
    except OSError as error:
        parser.error("could not connect to {}:{}: {}".format(args.host, args.port, error))

    session = RobotSession(sock, Path("telemetry.csv"), args.log_dir)
    session.quiet = True
    session.start()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.out_dir / "{}_{}.csv".format(args.protocol, stamp)
    meta_path = csv_path.with_suffix(".meta.json")

    try:
        if args.config is not None:
            session.load_config(args.config)
        try:
            params = session.fetch_params()
        except (TimeoutError, OSError):
            params = {}
            print("Warning: could not read robot parameters for the metadata.", file=sys.stderr)

        meta = {
            "protocol": args.protocol,
            "created": datetime.now().isoformat(timespec="seconds"),
            "arguments": {k: (str(v) if isinstance(v, Path) else v)
                          for k, v in vars(args).items()},
            "seed": seed,
            "robot_params": params,
            "sensor_mapping": DN_MAPPING_NOTE,
            "sample_period_ms": params.get("update_interval_ms", 10),
        }
        meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")

        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(CSV_COLUMNS)
            handle.flush()
            print("Writing {}".format(csv_path))
            written = run_steps(session, steps, writer, handle, args.settle,
                                highlight=highlight)
        print("Saved {}/{} batches to {}".format(written, len(steps), csv_path))
        print("Analyse with: python tests\\reflectance_analysis.py {} {}".format(
            {"sweep": "gain", "noise": "noise", "footprint": "footprint"}[args.protocol],
            csv_path))
    except KeyboardInterrupt:
        print("\nInterrupted; completed batches are already saved in {}".format(csv_path))
    finally:
        session.close()


if __name__ == "__main__":
    main()
