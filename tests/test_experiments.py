import socket
import tempfile
import threading
import unittest
from pathlib import Path

from experiments import (
    CSV_COLUMNS,
    Step,
    batch_rows,
    fill_value,
    footprint_positions,
    footprint_steps,
    noise_steps,
    run_steps,
    sweep_steps,
)
from tcp_logger import RobotSession


class ProtocolTests(unittest.TestCase):
    def test_fill_value_matches_course_convention(self):
        self.assertEqual(fill_value(0), 255)
        self.assertEqual(fill_value(100), 0)
        self.assertEqual(fill_value(50), 128)

    def test_batch_rows_expand_to_long_format(self):
        step = Step("surfaceSweep", "", 40.0, "fit", 2, 2)
        rows = batch_rows(step, [(100, [1, 2, 3, 4, 5]), (110, [6, 7, 8, 9, 10])])
        self.assertEqual(len(rows), 10)
        self.assertEqual(len(rows[0]), len(CSV_COLUMNS))
        self.assertEqual(rows[0][:9], ["surfaceSweep", 153, 2, 1, 100, 1, 1, "40", "fit"])
        self.assertEqual(rows[-1][3:7], [2, 110, 5, 10])

    def test_sweep_order_is_shuffled_but_reproducible(self):
        a = sweep_steps([0, 50, 100], [25], 3, 30, seed=7)
        b = sweep_steps([0, 50, 100], [25], 3, 30, seed=7)
        self.assertEqual([s.blackness_pct for s in a], [s.blackness_pct for s in b])
        self.assertEqual(len(a), 12)
        for placement in (1, 2, 3):
            conditions = sorted(s.blackness_pct for s in a if s.trial_num == placement)
            self.assertEqual(conditions, [0, 25, 50, 100])
        self.assertEqual({s.role for s in a if s.blackness_pct == 25}, {"holdout"})

    def test_noise_last_placement_is_held_back(self):
        steps = noise_steps([20, 60], 3, 1000)
        self.assertEqual([s.role for s in steps], ["fit", "fit", "holdout"] * 2)

    def test_footprint_steps(self):
        positions = footprint_positions(-2, 2, 1)
        self.assertEqual(positions, [-2, -1, 0, 1, 2])
        steps = footprint_steps(3, [10, 3], positions, 50, 200, ["increasing", "decreasing"], 1)
        self.assertEqual([s.role for s in steps[:2]], ["reference", "reference"])
        traverse = [s for s in steps if s.trial_num == 2]
        self.assertEqual([s.position_mm for s in traverse], [2, 1, 0, -1, -2])
        self.assertEqual(len(steps), 2 + 2 * 2 * 5)


class FakeRobot:
    """Answers SET and TRIAL like the firmware, over a real socket pair."""

    def __init__(self):
        self.server, self.client = socket.socketpair()
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        buffer = b""
        while True:
            try:
                data = self.server.recv(1024)
            except OSError:
                return
            if not data:
                return
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self._handle(line.decode())

    def _send(self, text):
        self.server.sendall((text + "\n").encode())

    def _handle(self, line):
        if line.startswith("SET,"):
            _, name, value = line.split(",")
            self.count = int(float(value))
            self._send("ACK,SET,{},{}".format(name, value))
        elif line.startswith("TRIAL,"):
            _, trial, pct = line.split(",")
            self._send("TRIAL_STARTED,{},{},{}".format(trial, pct, self.count))
            for i in range(self.count):
                self._send("TRIAL_SAMPLE,{},{},{},{},{},{},{},{}".format(
                    1000 + 10 * i, trial, pct, 100 + i, 200, 300, 400, 500))
            self._send("TRIAL_DONE,{},{}".format(trial, self.count))
        elif line == "STOP":
            self._send("ACK,STOP")


class CollectBatchTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        tmp = Path(self.tmp.name)
        self.robot = FakeRobot()
        self.session = RobotSession(self.robot.client, tmp / "trial.csv", tmp / "logs")
        self.session.quiet = True
        self.session.start()

    def tearDown(self):
        self.session.close()
        self.robot.server.close()
        self.tmp.cleanup()

    def test_collect_batch_returns_samples_without_touching_trial_csv(self):
        batch = self.session.collect_batch(5, 40)
        self.assertEqual(len(batch), 5)
        self.assertEqual(batch[0], (1000, [100, 200, 300, 400, 500]))
        self.assertEqual(batch[-1][1][0], 104)
        self.assertEqual(self.session.params["trial_sample_count"], 5)
        self.session.trial_file.flush()
        self.assertEqual(self.session.telemetry_csv_path.read_text(), "")

    def test_run_steps_with_redo_writes_only_kept_batches(self):
        import csv
        import io
        steps = [Step("t", "a", 0.0, "fit", 1, 3), Step("t", "b", 100.0, "fit", 1, 3)]
        answers = iter(["", "r", "", "", ""])   # collect a, redo a, collect a, collect b
        handle = io.StringIO()
        writer = csv.writer(handle, lineterminator="\n")
        written = run_steps(self.session, steps, writer, handle, settle_s=0,
                            input_fn=lambda prompt: next(answers))
        self.assertEqual(written, 2)
        self.assertEqual(len(handle.getvalue().splitlines()), 2 * 3 * 5)


if __name__ == "__main__":
    unittest.main()
