import csv
import math
import random
import tempfile
import unittest
from pathlib import Path

import numpy as np

from experiments import CSV_COLUMNS, Step, batch_rows
from reflectance_analysis import (
    analyse_footprint,
    analyse_gain,
    analyse_noise,
    analyse_nonlinearity,
    fit_power_gamma,
    golden_section_min,
    lag1_autocorrelation,
    linear_fit,
    load_rows,
    mae,
    noise_stats,
    normalise_z,
    peak_and_fwhm,
)

WHITE = [860, 750, 550, 630, 900]
BLACK = [2300, 2830, 2100, 2560, 2650]


def write_csv(path, steps_and_batches):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(CSV_COLUMNS)
        for step, batch in steps_and_batches:
            writer.writerows(batch_rows(step, batch))


def sweep_batch(pct, gamma=1.0, noise=0.0, n=30, rng=None):
    u = pct / 100.0
    batch = []
    for i in range(n):
        readings = []
        for w, b in zip(WHITE, BLACK):
            value = w + (b - w) * u ** gamma
            if noise:
                value += rng.gauss(0, noise)
            readings.append(int(round(value)))
        batch.append((i * 10, readings))
    return batch


class PureMathTests(unittest.TestCase):
    def test_linear_fit_recovers_known_line(self):
        m, c, r2 = linear_fit([0, 10, 20, 30], [5, 25, 45, 65])
        self.assertAlmostEqual(m, 2.0)
        self.assertAlmostEqual(c, 5.0)
        self.assertAlmostEqual(r2, 1.0)

    def test_mae(self):
        self.assertAlmostEqual(mae([1, 2, 3], [1, 3, 5]), 1.0)
        self.assertTrue(math.isnan(mae([], [])))

    def test_golden_section_and_power_gamma(self):
        self.assertAlmostEqual(golden_section_min(lambda x: (x - 1.7) ** 2, 0, 5), 1.7, places=4)
        u = np.linspace(0, 1, 11)
        self.assertAlmostEqual(fit_power_gamma(u, u ** 1.6), 1.6, places=3)

    def test_noise_stats(self):
        stats = noise_stats([10, 12, 11, 13, 9])
        self.assertEqual(stats["count"], 5)
        self.assertAlmostEqual(stats["mean"], 11.0)
        self.assertAlmostEqual(stats["median"], 11.0)
        self.assertAlmostEqual(stats["sd"], math.sqrt(2.5))
        self.assertAlmostEqual(stats["mad"], 1.0)
        self.assertEqual((stats["min"], stats["max"], stats["levels"]), (9, 13, 5))

    def test_lag1_autocorrelation(self):
        rng = np.random.default_rng(1)
        self.assertLess(abs(lag1_autocorrelation(rng.normal(size=5000))), 0.05)
        self.assertGreater(lag1_autocorrelation(np.cumsum(rng.normal(size=5000))), 0.9)

    def test_normalise_z(self):
        self.assertEqual(normalise_z(500, 500, 2500), 0.0)
        self.assertEqual(normalise_z(2500, 500, 2500), 1.0)
        self.assertEqual(normalise_z(1500, 500, 2500), 0.5)

    def test_fwhm_of_blurred_strip(self):
        # 6 mm strip convolved with a Gaussian footprint (sigma 1.5 mm).
        x = np.arange(-20, 20.01, 0.5)
        sigma, half = 1.5, 3.0
        z = [0.5 * (math.erf((p + half) / (sigma * math.sqrt(2)))
                    - math.erf((p - half) / (sigma * math.sqrt(2)))) for p in x]
        peak, position, fwhm = peak_and_fwhm(x, z)
        self.assertAlmostEqual(position, 0.0)
        self.assertGreater(peak, 0.9)
        self.assertAlmostEqual(fwhm, 6.0, delta=0.1)

    def test_fwhm_none_when_peak_below_half(self):
        self.assertEqual(peak_and_fwhm([-1, 0, 1], [0.1, 0.3, 0.1])[2], None)


class AnalysisPipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _sweep_csv(self, gamma):
        rng = random.Random(3)
        items = []
        for placement in (1, 2, 3):
            for pct, role in [(0, "fit"), (20, "fit"), (40, "fit"), (60, "fit"), (80, "fit"),
                              (100, "fit"), (10, "holdout"), (50, "holdout"), (90, "holdout")]:
                step = Step("surfaceSweep", "", float(pct), role, placement, 30)
                items.append((step, sweep_batch(pct, gamma, 2.0, 30, rng)))
        path = self.dir / "sweep.csv"
        write_csv(path, items)
        return load_rows([path])

    def test_gain_on_linear_sensor(self):
        rows = self._sweep_csv(gamma=1.0)
        summary = analyse_gain(rows, self.dir / "gain").read_text(encoding="utf-8")
        self.assertIn("DN5", summary)
        self.assertTrue((self.dir / "gain" / "gain_fit_and_residuals.png").exists())
        # DN1 gain should be (2300-860)/100 = 14.4 per %.
        dn1 = next(l for l in summary.splitlines() if l.startswith("| DN1 |"))
        self.assertAlmostEqual(float(dn1.split("|")[3]), 14.4, delta=0.1)

    def test_nonlinearity_recovers_gamma(self):
        rows = self._sweep_csv(gamma=1.5)
        summary = analyse_nonlinearity(rows, self.dir / "nl").read_text(encoding="utf-8")
        shared = next(l for l in summary.splitlines() if l.startswith("Shared γ"))
        self.assertAlmostEqual(float(shared.split(":")[1]), 1.5, delta=0.05)

    def test_noise_pipeline(self):
        rng = random.Random(5)
        items = []
        for pct in (20, 60):
            for placement in (1, 2, 3):
                role = "holdout" if placement == 3 else "fit"
                step = Step("noiseTest", "", float(pct), role, placement, 300)
                # sigma 25 so integer quantisation barely affects coverage fractions.
                items.append((step, sweep_batch(pct, 1.0, 25.0, 300, rng)))
        path = self.dir / "noise.csv"
        write_csv(path, items)
        summary = analyse_noise(load_rows([path]), self.dir / "noise").read_text(encoding="utf-8")
        self.assertIn("held-back placement", summary)
        row = next(l for l in summary.splitlines() if l.startswith("| 20 | DN1 |")
                   and "%" not in l and l.count("|") == 10)
        within_1sigma = float(row.split("|")[5])
        self.assertAlmostEqual(within_1sigma, 68, delta=8)

    def test_footprint_pipeline(self):
        sigma = 1.5
        items = [
            (Step("footprintWhiteReference", "", 0.0, "reference", 0, 50),
             [(i, list(WHITE)) for i in range(50)]),
            (Step("footprintBlackReference", "", 100.0, "reference", 0, 50),
             [(i, list(BLACK)) for i in range(50)]),
        ]
        for trial, width in enumerate((10.0, 2.0), start=1):
            for position in np.arange(-12, 12.01, 1.0):
                half = width / 2
                z = 0.5 * (math.erf((position + half) / (sigma * math.sqrt(2)))
                           - math.erf((position - half) / (sigma * math.sqrt(2))))
                readings = [int(round(w + (b - w) * z)) for w, b in zip(WHITE, BLACK)]
                step = Step("footprintTest", "", 0.0, "fit", trial, 10, float(position),
                            width, "increasing")
                items.append((step, [(i, readings) for i in range(10)]))
        path = self.dir / "fp.csv"
        write_csv(path, items)
        summary = analyse_footprint(load_rows([path]), self.dir / "fp", 3).read_text(
            encoding="utf-8")
        ten = next(l for l in summary.splitlines() if l.startswith("| 10 |"))
        self.assertAlmostEqual(float(ten.split("|")[6]), 10.0, delta=0.3)
        self.assertIn("Narrowest strip", summary)


if __name__ == "__main__":
    unittest.main()
