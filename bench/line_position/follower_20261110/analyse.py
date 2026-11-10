"""Corner behaviour of the offset follower on the 2026-11-10 run.

Line offsets come from tools/line_position.py's mirror of the robot's line
sensor, re-run on the logged readings. Wheel speeds are from the encoders
over 5 records (50 ms). Prints the numbers in ../report.md and writes
summary.txt next to this script.

    python3 bench/line_position/follower_20261110/analyse.py
"""
import csv
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
RAW = HERE / "raw" / "run.csv"
# Settings the robot ran with (config/lokum.json at 2026-11-10 20:25).
SETTINGS = dict(left_wheel_radius_mm=16.26, right_wheel_radius_mm=16.26, wheel_separation_mm=84.43,
                left_run_min_pwm=26, right_run_min_pwm=19, line_speed_mm_s=90)
COUNTS_PER_REV = 358.3
LAPS = 4.75   # net turning over the run / 360 deg


def line_offsets():
    with tempfile.TemporaryDirectory() as out:
        subprocess.run([sys.executable, str(ROOT / "tools" / "line_position.py"), str(RAW), "--out", out],
                       check=True, capture_output=True)
        return {r["t_ms"]: r for r in csv.DictReader(open(Path(out) / "line.csv"))}


def main():
    rows = [r for r in csv.DictReader(open(RAW)) if r["signal"] == "1"]
    line = line_offsets()
    t = np.array([float(r["t_ms"]) for r in rows])
    enc = {s: np.array([float(r[f"enc_{s}"]) for r in rows]) for s in ("left", "right")}
    pwm = {s: np.array([float(r[f"pwm_{s}"]) for r in rows]) for s in ("left", "right")}
    seen = np.array([line[r["t_ms"]]["detected"] == "1" for r in rows])
    dur = (t[-1] - t[0]) / 1000

    k = 5
    speed = {}
    for s in ("left", "right"):
        mm = 2 * math.pi * SETTINGS[f"{s}_wheel_radius_mm"] / COUNTS_PER_REV
        speed[s] = (enc[s][k:] - enc[s][:-k]) * mm / ((t[k:] - t[:-k]) / 1000)
    fwd = (speed["left"] + speed["right"]) / 2

    losses = []
    i = 0
    while i < len(seen):
        if not seen[i]:
            j = i
            while j < len(seen) and not seen[j]:
                j += 1
            losses.append((t[j - 1] - t[i]) / 1000 + 0.01)
            i = j
        else:
            i += 1
    losses = np.array(losses)

    out = [f"run: {RAW.relative_to(ROOT)}, {dur:.0f} s following, {LAPS} laps anticlockwise "
           f"({dur / LAPS:.1f} s a lap), line_speed_mm_s {SETTINGS['line_speed_mm_s']}",
           f"forward speed (50 ms) p5 / median / p95: " + " / ".join(f"{v:.0f}" for v in np.percentile(fwd, [5, 50, 95]))
           + " mm/s",
           f"line lost {len(losses)} times ({len(losses) / LAPS:.1f} a lap), median {np.median(losses):.2f} s, "
           f"longest {losses.max():.2f} s, {losses.sum():.1f} s in all"]
    for s in ("left", "right"):
        p = pwm[s]
        flips = int(np.sum(np.sign(p[1:]) * np.sign(p[:-1]) < 0))
        jump = np.abs(np.diff(p))
        floor = SETTINGS[f"{s}_run_min_pwm"]
        out.append(f"{s} wheel: PWM forward/reverse flips {flips} ({flips / dur:.1f}/s), "
                   f"largest change in one cycle {jump.max():.0f} PWM, at the run-min floor "
                   f"{100 * np.mean(np.abs(np.round(p)) == floor):.0f} % of the time, "
                   f"speed {speed[s].min():.0f} to {speed[s].max():.0f} mm/s")
    text = "\n".join(out)
    print(text)
    (HERE / "summary.txt").write_text(text + "\n")


if __name__ == "__main__":
    main()
