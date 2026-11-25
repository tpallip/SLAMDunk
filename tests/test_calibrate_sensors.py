import csv
import json
import unittest
from pathlib import Path

from calibrate_sensors import compute_limits

REPO_ROOT = Path(__file__).resolve().parent.parent
HEADER = ["timestamp", "trial_no", "percentage",
          "reading0", "reading1", "reading2", "reading3", "reading4"]


class ComputeLimitsTests(unittest.TestCase):
    def test_means_of_white_and_black_samples(self):
        rows = [
            HEADER,
            ["1", "1", "0", "100", "200", "300", "400", "500"],
            ["2", "1", "0", "110", "210", "310", "410", "510"],
            ["3", "1", "50", "9999", "9999", "9999", "9999", "9999"],  # ignored
            HEADER,                                                     # repeated header
            ["4", "2", "100", "1000", "2000", "3000", "4000", "5000"],
            ["5", "2", "100.0", "1100", "2100", "3100", "4100", "5100"],
            ["bad", "2", "100", "1", "1", "1", "1", "1"],               # malformed
            ["6", "2", "100", "1", "1"],                                # short
        ]
        limits = compute_limits(rows)
        self.assertEqual(limits["white_dn1"], 105.0)
        self.assertEqual(limits["white_dn5"], 505.0)
        self.assertEqual(limits["black_dn1"], 1050.0)
        self.assertEqual(limits["black_dn5"], 5050.0)

    def test_missing_surface_is_an_error(self):
        with self.assertRaises(ValueError):
            compute_limits([["1", "1", "0", "1", "2", "3", "4", "5"]])

    def test_reproduces_shipped_calibration_from_recorded_trials(self):
        with (REPO_ROOT / "telemetry.csv").open(newline="", encoding="utf-8") as handle:
            limits = compute_limits(list(csv.reader(handle)))
        config = json.loads((REPO_ROOT / "config" / "robot_params.json").read_text(encoding="utf-8"))
        for key, value in limits.items():
            with self.subTest(key=key):
                self.assertAlmostEqual(config[key], value, delta=0.1)


if __name__ == "__main__":
    unittest.main()
