"""Checks tools/line_position.py's offsets on the line spin run (spread_cmd70_01).

    python3 bench/line_position/check_spin.py [logs/spread_cmd70_01.csv]

Truth from the spin itself: for each turn, the spin centre's distance from the
line's middle (c) and the line's direction (psi) are fitted to all five
normalised readings, each sensor at u = c + r sin(heading + angle - psi) from
the line's middle (the spin centre drifts, so c is fitted per turn). Where the
robot faces along the line (heading within 3 deg of psi or psi + 180) the
line's true offset at the sensors, about 42 mm ahead, is -(c + 42 sin(d)) or
c + 42 sin(d), d the misalignment; the estimator is compared with that.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import line_position as lp    # noqa: E402
import sensor_spread as ss    # noqa: E402

HEADING_SCALE = 1.0003        # true / odometry, sensor_spread on this run
AHEAD_MM = 42.0
ALIGNED_DEG = 3.0


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "logs" / "spread_cmd70_01.csv")
    data = lp.load(path)
    cfg = lp.settings(lp.CONFIG)
    sensor = lp.LineSensor(cfg)
    theta = data["theta_rad"] * HEADING_SCALE
    raw = np.column_stack([data[s] for s in lp.SENSORS])
    n = sensor.normalise(raw)
    offset = np.full(len(theta), np.nan)
    for i in range(len(theta)):
        _, o, _, seen = sensor.update(raw[i])
        if seen:
            offset[i] = o
    angles = np.radians(lp.ANGLES_DEG)

    def predicted(c, psi, idx):
        u = c + lp.RADIUS_MM * np.sin(theta[idx, None] + angles[None, :] - psi)
        w, sigma = cfg["width"], cfg["sigma"]
        return lp.normal_cdf((u + w / 2) / sigma) - lp.normal_cdf((u - w / 2) / sigma)

    errors = []
    print("run turn      c   psi  fit rms  aligned  err median")
    for k, (a, b) in enumerate(ss.spin_runs(data)):
        run = np.arange(a, b)
        direction = np.sign(theta[b - 1] - theta[a])
        turn = np.floor((theta[run] - theta[a]) * direction / (2 * math.pi)).astype(int)
        for q in np.unique(turn):
            idx = run[turn == q]
            if len(idx) < 200:
                continue
            best = min(((((predicted(c, psi, idx) - n[idx]) ** 2).mean(), c, psi)
                        for c in np.arange(-25, 25.1, 0.5) for psi in np.radians(np.arange(0, 180, 0.5))),
                       key=lambda x: x[0])
            mse, c, psi = best
            mis = (theta[idx] - psi + math.pi) % (2 * math.pi) - math.pi
            turn_errors = []
            for back in (False, True):
                d = mis - np.sign(mis) * math.pi if back else mis
                sel = (np.abs(d) < math.radians(ALIGNED_DEG)) & ~np.isnan(offset[idx])
                truth = c + AHEAD_MM * np.sin(d[sel])
                turn_errors += list(offset[idx][sel] - (truth if back else -truth))
            errors += turn_errors
            print(f"{k:3} {q:4} {c:+6.1f} {math.degrees(psi):5.1f} {math.sqrt(mse):8.3f} {len(turn_errors):8} "
                  f"{np.median(turn_errors):+11.2f}")
    errors = np.array(errors)
    print(f"all aligned records: n={len(errors)}, median {np.median(errors):+.2f} mm, "
          f"rms {np.sqrt(np.mean(errors ** 2)):.2f} mm, p95 |err| {np.percentile(np.abs(errors), 95):.1f} mm")


if __name__ == "__main__":
    main()
