"""Surface sensor static noise and room lighting, robot standing still (TODO 3.1, 3.3).

    python3 bench/3.1_static_noise/analyse.py

Reads the logger CSVs in raw/ and writes summary.csv next to this script:
per run and sensor, mean, sd, sd from successive differences (equal to sd
when the noise is white, smaller when the reading wanders slowly),
min, max, drift (mean of the last third minus the first) and whether the
reading sat at the 2942 ceiling. Tape levels for comparison come from the
line spin run (bench/3.4_spatial_response/spread_cmd70_01/summary.csv).
"""
import csv
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TAPE = HERE.parent / "3.4_spatial_response" / "spread_cmd70_01" / "summary.csv"
SENSORS = ["dn1", "dn2", "dn3", "dn4", "dn5"]
CEILING = 2942                      # highest raw reading seen: the RC timing's cap
RUNS = [   # file, surface, lighting, sensors on black
    ("all_white.csv", "white", "normal", ""),
    ("all_white_low_light.csv", "white", "low", ""),
    ("all_white_low_light_blinds.csv", "white", "low, blinds down", ""),
    ("dn1_black.csv", "black", "normal", "dn1"),
    ("dn234_black.csv", "black", "normal", "dn2 dn3 dn4"),
    ("dn5_black.csv", "black", "normal", "dn5"),
]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}


def main():
    out = []
    for name, surface, light, on_black in RUNS:
        d = load(HERE / "raw" / name)
        t = d["t_ms"]
        moved = np.ptp(d["enc_left"]) + np.ptp(d["enc_right"])
        print(f"{name}: {len(t)} records over {(t[-1] - t[0]) / 1000:.1f} s, "
              f"every {np.median(np.diff(t)):.0f} ms, encoders moved {moved:.0f} counts")
        for s in SENSORS:
            v = d[s]
            third = len(v) // 3
            out.append({"run": name, "surface": surface if s in on_black.split() or surface == "white" else "partly",
                        "lighting": light, "sensor": s, "n": len(v), "mean": v.mean(), "sd": v.std(),
                        "sd_diff": np.std(np.diff(v)) / np.sqrt(2), "min": v.min(), "max": v.max(),
                        "drift": v[-third:].mean() - v[:third].mean(), "at_ceiling": bool(v.max() >= CEILING)})

    with open(HERE / "summary.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(out[0]))
        writer.writeheader()
        for row in out:
            writer.writerow({k: (f"{x:.1f}" if isinstance(x, float) else x) for k, x in row.items()})

    white = {r["sensor"]: r for r in out if r["run"] == "all_white.csv"}
    print(f"\n{'sensor':6} {'white':>6} {'sd':>5} {'low':>5} {'blinds':>6} {'black':>6} {'sd':>5} "
          f"{'tape w':>6} {'tape b':>6} {'50 %':>5} {'noise mm':>8}")
    tape = {r["sensor"]: r for r in csv.DictReader(open(TAPE))} if TAPE.exists() else {}
    for s in SENSORS:
        low = next(r for r in out if r["run"] == "all_white_low_light.csv" and r["sensor"] == s)
        blinds = next(r for r in out if r["run"] == "all_white_low_light_blinds.csv" and r["sensor"] == s)
        black = next(r for r in out if r["surface"] == "black" and r["sensor"] == s)
        line = f"{s:6} {white[s]['mean']:6.0f} {white[s]['sd']:5.1f} {low['mean'] - white[s]['mean']:+5.0f} " \
               f"{blinds['mean'] - white[s]['mean']:+6.0f} {black['mean']:6.0f} {black['sd']:5.1f}"
        if s in tape:
            tw, tb, sigma = float(tape[s]["white"]), float(tape[s]["black"]), float(tape[s]["sigma_mm"])
            # Reading slope at the 50 % point of a Gaussian-blurred edge: span / (sigma sqrt(2 pi)) per mm.
            noise_mm = white[s]["sd"] / ((tb - tw) / (sigma * np.sqrt(2 * np.pi)))
            line += f" {tw:6.0f} {tb:6.0f} {(tw + tb) / 2:5.0f} {noise_mm:8.2f}"
        print(line)
    print("\nlow, blinds: change from normal light on white. black: the run with that sensor on black "
          "(2942 = ceiling). tape: line spin run levels; 50 %: their midpoint. noise mm: white sd as "
          "position noise at the tape edge's 50 % point.")
    print(f"saved {HERE / 'summary.csv'}")


if __name__ == "__main__":
    main()
