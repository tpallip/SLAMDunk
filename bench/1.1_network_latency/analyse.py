"""Summarises the ping tests in raw/ (saved by tools/motor_tui.py "ping N MS").

    python3 bench/1.1_network_latency/analyse.py [logs/latency_logger_session.csv]

Writes summary.csv next to this script and prints it. The optional argument is
the logger's telemetry CSV from the same session, for the record-gap numbers.
"""
import csv
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RUNS = [   # file, link, motors
    ("latency_logger_20261013_180406.csv", "WiFi via logger", "off"),
    ("latency_logger_20261013_180548.csv", "WiFi via logger", "40/40"),
    ("latency_serial_20261013_182915.csv", "USB serial", "off"),
]
PERCENTILES = [50, 95, 99]


def stats(values):
    p = np.percentile(values, PERCENTILES)
    return {"median": p[0], "p95": p[1], "p99": p[2], "max": values.max(), "min": values.min()}


def main():
    out = []
    for name, link, motors in RUNS:
        rows = list(csv.DictReader(open(HERE / "raw" / name)))
        ok = [r for r in rows if r["lost"] == "0"]
        rtt = np.array([float(r["rtt_ms"]) for r in ok])
        record = {"run": name, "link": link, "motors": motors, "pings": len(rows),
                  "lost": len(rows) - len(ok), "part": "round trip", **stats(rtt)}
        out.append(record)
        if ok and ok[0]["robot_leg_ms"]:
            leg = np.array([float(r["robot_leg_ms"]) for r in ok])
            for part, values in (("robot leg", leg), ("logger share", rtt - leg)):
                out.append({**record, "part": part, **stats(values)})
        if link == "USB serial":
            slow = rtt > 13
            print(f"{name}: {slow.mean():.0%} of answers > 13 ms; "
                  f"median per 1 s block: "
                  + " ".join(f"{np.median(rtt[i * 50:(i + 1) * 50]):.1f}" for i in range(len(rtt) // 50)))

    with open(HERE / "summary.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(out[0]))
        writer.writeheader()
        for r in out:
            writer.writerow({k: f"{v:.2f}" if isinstance(v, float) else v for k, v in r.items()})
    for r in out:
        print(f"{r['link']:16} motors {r['motors']:5} {r['part']:12} median {r['median']:5.1f}  "
              f"p95 {r['p95']:5.1f}  p99 {r['p99']:5.1f}  max {r['max']:5.1f}  min {r['min']:4.1f} ms"
              f"  lost {r['lost']}/{r['pings']}")

    if len(sys.argv) > 1:
        t = np.array([float(r["t_ms"]) for r in csv.DictReader(open(sys.argv[1]))])
        gaps = np.diff(t)
        print(f"telemetry: {len(t)} records over {(t[-1] - t[0]) / 1000:.0f} s "
              f"({len(t) / ((t[-1] - t[0]) / 1000):.1f}/s); gaps == 10 ms: {np.mean(gaps == 10):.1%}, "
              f"p99 {np.percentile(gaps, 99):.0f} ms, max {gaps.max():.0f} ms")


if __name__ == "__main__":
    main()
