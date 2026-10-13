# Command round-trip latency: WiFi (via logger) vs USB

TODO.md 1.1, 1.6, 2.10 (partial). Run 2026-10-13.

**Question.** Is USB worth using instead of the WiFi AP for motor
characterisation?
**Answer.** No. The links perform about the same where it matters, and the
motor tests keep their own timing on the robot anyway.

## Method

`tools/motor_tui.py`, command `ping 500 20`: 500 `ping <id>` commands, one
every 20 ms (10 s per run). The robot answers `#P <id> <t_ms>` in its next
10 ms controller cycle. The round trip is laptop → robot → laptop, and
includes 0–10 ms of waiting for that cycle (about 5 ms on average), which
every real command also pays. Through the logger, the logger also times its
own leg ("robot leg": WiFi + robot); the rest is the "logger share".

Conditions: robot on its stand. Laptop `aramaki` (Ubuntu, Linux 6.8). WiFi:
robot AP `lokum`, TCP port 80, logger with Rerun on. USB: CH9102 at 115200
baud (`/dev/ttyACM0`), no WiFi client connected. Firmware:
`pranay/motor_characteristics` at 8a43471 plus the uncommitted ping/USB
changes. Battery charge and WiFi distance were not recorded.

## Results (ms; 500 pings per run, 0 lost in every run)

| Link | Motors | Median | p95 | p99 | Max | Min |
|---|---|---|---|---|---|---|
| WiFi via logger | off | 10.5 | 16.9 | 21.8 | 36.7 | 3.9 |
| WiFi via logger | 40/40 | 9.7 | 15.9 | 25.8 | 52.2 | 3.7 |
| USB | off | 7.2 | 16.6 | 18.0 | 28.9 | 2.5 |

The WiFi round trip splits into a robot leg (median 9.3–9.6 ms, p95
14.8–15.7 ms) and a logger share (median 0.2 ms, p95 2–3 ms, max 23–32 ms).

Telemetry over WiFi, from the logger CSV of the same session: 22 743
records in 230 s (98.9/s). 97.4 % of gaps are exactly 10 ms; p99 is 14 ms,
max 42 ms.

## Findings

1. **USB is about 3 ms faster at the median and no better in the tail.**
   With the ~5 ms cycle wait taken out, USB adds about 2 ms and WiFi about
   4–5 ms.
2. **The logger adds almost nothing**: 0.2 ms median, with occasional 5–30 ms
   stalls in 1–3 % of pings (likely table redraw or Rerun; not tested).
3. **Motors at 40/40 made no difference over WiFi.** Its 52 ms max came
   from a short cluster of slow robot legs around t = 4 s (a WiFi retry
   burst).
4. **The robot's 10 ms cycle sometimes runs late, by up to 32 ms.** Motor
   speed must be computed from recorded timestamps, never an assumed 10 ms.
5. **Two quirks in the USB run** (not yet investigated):
   - *Every 5th answer is about 10 ms late* (16 % of answers fall at
     15–20 ms), so something on the robot costs about one cycle every
     100 ms. Suspect: with no WiFi client connected, the robot checks for a
     new one every 100 ms (`_acceptIntervalMs` in `RobotWifiAP.h`).
   - *The median climbs from 4.1 to 9.1 ms over the run*: the fixed 20 ms
     ping schedule drifts against the robot's cycle, so a 10 s run doesn't
     sample the whole 0–10 ms wait. Random jitter on the send times would
     fix this.

## Decision

Use **WiFi through the logger** for motor characterisation. It passes the
plan's threshold (p95 < 50 ms, no losses) and leaves room for the `#B`
benchmark stream. USB at 115200 baud is already ~60 % used by the 100 Hz
telemetry. A cable would also load the robot in floor tests, and it must be
plugged in *before* switching the 3Pi+ on. Keep USB as a fallback.

**Not done:** direct WiFi (`--direct`), distances, longer runs, and USB with
a WiFi client connected (would test the 100 ms suspect).

Files: `raw/` (per-ping CSVs), `analyse.py` (writes `summary.csv`; pass the
logger CSV for the telemetry gaps).
