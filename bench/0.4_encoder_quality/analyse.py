"""Encoder check (plan 0.4) from a logger CSV: hand-turn counts, then freshness.

    python3 bench/0.4_encoder_quality/analyse.py logs/encoder_check_20261013.csv

1. Splits the run into stretches of constant (pwm_left, pwm_right).
2. In stretches with both PWMs at 0, finds hand turns: runs of count changes
   separated by more than STILL_S of stillness.
3. In stretches with both PWMs equal and non-zero, skips SETTLE_S, then per
   wheel: counts/s, expected counts per 10 ms cycle, the histogram of count
   changes per record, how often a record repeats the previous count, and the
   speed spread for 10/20/50/100 ms windows.
"""
import csv
import sys

import numpy as np

MM_PER_COUNT = 2 * np.pi * 16.0 / 358.3
STILL_S = 1.5
SETTLE_S = 1.0
MIN_TURN_COUNTS = 30


def load(path):
    rows = list(csv.DictReader(open(path)))
    return {k: np.array([float(r[k]) for r in rows])
            for k in ("t_ms", "pwm_left", "pwm_right", "enc_left", "enc_right")}


def stretches(d):
    """(start, end) index ranges of constant PWM pair."""
    pwm = np.stack([d["pwm_left"], d["pwm_right"]], axis=1)
    change = np.r_[True, np.any(pwm[1:] != pwm[:-1], axis=1), True]
    edges = np.flatnonzero(change)
    return list(zip(edges[:-1], edges[1:]))


def hand_turns(t_s, enc):
    """(start, end) indices of each burst of motion in a still stretch."""
    moved = np.flatnonzero(np.diff(enc) != 0) + 1
    if len(moved) == 0:
        return []
    bursts, start, prev = [], moved[0] - 1, moved[0]
    for i in moved[1:]:
        if t_s[i] - t_s[prev] > STILL_S:
            bursts.append((start, prev))
            start = i - 1
        prev = i
    bursts.append((start, prev))
    return [(a, b) for a, b in bursts if abs(enc[b] - enc[a]) >= MIN_TURN_COUNTS]


def freshness(t, enc):
    d = np.diff(enc)
    dt = np.diff(t)
    rate = (enc[-1] - enc[0]) / (t[-1] - t[0]) * 1000          # counts/s
    vals, counts = np.unique(d, return_counts=True)
    resid = d - rate / 1000 * dt                                 # after allowing for the actual dt
    on_time = dt == 10
    windows = []
    for w in (10, 20, 50, 100):
        k = w // 10
        i = np.arange(0, len(t) - k, k)
        speed = (enc[i + k] - enc[i]) / ((t[i + k] - t[i]) / 1000) * MM_PER_COUNT
        windows.append((w, speed.std(), 100 * speed.std() / speed.mean()))
    return {
        "rate": rate, "mm_s": rate * MM_PER_COUNT, "per_cycle": rate / 100,
        "repeats": np.mean(d == 0), "hist": dict(zip(vals.astype(int), counts)),
        "resid_max": np.abs(resid[on_time]).max(), "resid_big": np.mean(np.abs(resid[on_time]) > 1.5),
        "windows": windows,
    }


def main():
    d = load(sys.argv[1])
    t, t_s = d["t_ms"], d["t_ms"] / 1000
    gaps = np.diff(t)
    print(f"{len(t)} records, {(t[-1] - t[0]) / 1000:.0f} s. Cycle gaps: 10 ms {np.mean(gaps == 10):.1%}, "
          f"11-19 ms {np.mean((gaps > 10) & (gaps < 20)):.1%}, >= 20 ms {np.mean(gaps >= 20):.2%}, "
          f"max {gaps.max():.0f} ms")

    print("\nStretches (PWM L/R: counts moved L/R)")
    for a, b in stretches(d):
        b1 = b - 1
        print(f"  {t_s[a]:7.1f}-{t_s[b1]:7.1f} s  PWM {d['pwm_left'][a]:+4.0f}/{d['pwm_right'][a]:+4.0f}: "
              f"{d['enc_left'][b1] - d['enc_left'][a]:+6.0f} / {d['enc_right'][b1] - d['enc_right'][a]:+6.0f}")

    print("\nHand turns (motors off)")
    for a, b in stretches(d):
        if d["pwm_left"][a] or d["pwm_right"][a]:
            continue
        for wheel, key in (("left", "enc_left"), ("right", "enc_right")):
            for s, e in hand_turns(t_s[a:b], d[key][a:b]):
                n = d[key][a + e] - d[key][a + s]
                print(f"  {wheel:5} {t_s[a + s]:7.1f}-{t_s[a + e]:7.1f} s: {n:+6.0f} counts = {n / 358.3:+.3f} rev")

    print("\nFreshness at constant PWM (first %.0f s of each stretch skipped)" % SETTLE_S)
    for a, b in stretches(d):
        pl, pr = d["pwm_left"][a], d["pwm_right"][a]
        if pl == 0 and pr == 0:
            continue
        m = slice(a, b)
        keep = t[m] >= t[a] + SETTLE_S * 1000
        if keep.sum() < 100:
            continue
        for wheel, key in (("L", "enc_left"), ("R", "enc_right")):
            enc = d[key][m][keep]
            if enc[-1] - enc[0] < 50:
                print(f"  PWM {pl:+.0f}/{pr:+.0f} {wheel}: not moving")
                continue
            f = freshness(t[m][keep], enc)
            hist = " ".join(f"{v}:{n}" for v, n in f["hist"].items() if n >= 2)
            win = ", ".join(f"{w} ms {sd:.1f} mm/s ({pc:.1f} %)" for w, sd, pc in f["windows"])
            print(f"  PWM {pl:+.0f}/{pr:+.0f} {wheel}: {f['mm_s']:5.1f} mm/s, {f['per_cycle']:.2f} counts/cycle, "
                  f"repeated reads {f['repeats']:.1%}, on-time residual max {f['resid_max']:.1f} "
                  f"(>1.5 counts {f['resid_big']:.1%})\n      changes per record {hist}\n      speed sd: {win}")


if __name__ == "__main__":
    main()
