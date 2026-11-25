import json
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from tcp_logger import (
    DATA_COLUMNS,
    KNOWN_PARAMS,
    LOG_COLUMNS,
    TRIAL_COLUMNS,
    WATCH_LABELS,
    build_command,
    calibration_from_params,
    connect_to_robot,
    format_watch_table,
    json_to_set_commands,
    normalise_readings,
    params_from_lines,
    parse_response,
)


class TcpLoggerTests(unittest.TestCase):
    def test_trial_sample_contains_expected_csv_columns(self):
        response = "TRIAL_SAMPLE,1234,3,25.00,120,121,122,123,124"
        record_type, values = parse_response(response)

        self.assertEqual(record_type, "trial_sample")
        self.assertEqual(len(values), len(TRIAL_COLUMNS))
        self.assertEqual(
            values,
            ["1234", "3", "25.00", "120", "121", "122", "123", "124"],
        )

    def test_data_response_has_expected_telemetry_shape(self):
        response = "DATA,50,1.00,2.00,0.10,10,11,12,13,100,101,102,103,104,1"
        record_type, values = parse_response(response)

        self.assertEqual(record_type, "telemetry")
        self.assertEqual(len(values), len(DATA_COLUMNS))

    def test_builds_trial_telemetry_and_control_commands(self):
        self.assertEqual(build_command("trial 3, 25"), "TRIAL,3,25.0\n")
        self.assertEqual(build_command("start telemetry"), "START TELEMETRY\n")
        self.assertEqual(build_command("stop telemetry"), "STOP TELEMETRY\n")
        self.assertEqual(build_command("motor -35 40"), "MOTOR,-35,40\n")
        self.assertEqual(build_command("stop"), "STOP\n")

    def test_rejects_out_of_range_commands(self):
        with self.assertRaises(ValueError):
            build_command("trial 1, 101")
        with self.assertRaises(ValueError):
            build_command("motor 101 0")

    def test_connected_socket_has_no_idle_receive_timeout(self):
        sock = Mock()
        with patch("tcp_logger.socket.create_connection", return_value=sock) as connect:
            self.assertIs(connect_to_robot("127.0.0.1", 80), sock)

        connect.assert_called_once_with(("127.0.0.1", 80), timeout=10)
        sock.settimeout.assert_called_once_with(None)


class ParameterAndControlTests(unittest.TestCase):
    def test_builds_parameter_commands(self):
        self.assertEqual(build_command("set base_pwm 30"), "SET,base_pwm,30\n")
        self.assertEqual(build_command("get line_threshold"), "GET,line_threshold\n")
        self.assertEqual(build_command("params"), "PARAMS\n")
        self.assertEqual(build_command("save"), "SAVE\n")
        self.assertEqual(build_command("defaults"), "DEFAULTS\n")

    def test_builds_log_and_motion_commands(self):
        self.assertEqual(build_command("log start"), "LOG START\n")
        self.assertEqual(build_command("log start run1.csv"), "LOG START\n")
        self.assertEqual(build_command("log stop"), "LOG STOP\n")
        self.assertEqual(build_command("move -150"), "MOVE,-150.0\n")
        self.assertEqual(build_command("rotate 90"), "ROTATE,1.570796\n")
        self.assertEqual(build_command("selftest"), "SELFTEST\n")
        self.assertEqual(build_command("odom-reset"), "ODOM_RESET\n")
        self.assertEqual(build_command("odom-reset 1 2 0.5"), "ODOM_RESET,1.0,2.0,0.5\n")

    def test_rejects_bad_parameter_and_motion_commands(self):
        for text in ("set base_pwm nan", "set base_pwm", "move 2500",
                     "rotate 400", "odom-reset 1 2"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                build_command(text)

    def test_motor_limit_follows_robot_parameter(self):
        self.assertEqual(build_command("motor 150 0", motor_limit=150), "MOTOR,150,0\n")
        with self.assertRaises(ValueError):
            build_command("motor 150 0")

    def test_parses_log_and_param_records(self):
        line = "LOG," + ",".join(str(i) for i in range(len(LOG_COLUMNS)))
        self.assertEqual(parse_response(line)[0], "log")
        self.assertEqual(parse_response("PARAM,base_pwm,45"), ("param", ["base_pwm", "45"]))
        with self.assertRaises(ValueError):
            parse_response("LOG,1,2,3")

    def test_json_to_set_commands(self):
        commands = json_to_set_commands(
            {"_note": "ignored", "base_pwm": 30, "wheel_separation_mm": 89.2})
        self.assertEqual(commands, [
            ("base_pwm", "SET,base_pwm,30\n"),
            ("wheel_separation_mm", "SET,wheel_separation_mm,89.2\n"),
        ])

    def test_json_to_set_commands_rejects_bad_input(self):
        for config in ({"not_a_param": 1}, {"base_pwm": "30"}, {"base_pwm": True},
                       {"base_pwm": {"nested": 1}}, [1, 2]):
            with self.subTest(config=config), self.assertRaises(ValueError):
                json_to_set_commands(config)

    def test_shipped_config_uses_only_known_params(self):
        path = Path(__file__).resolve().parent.parent / "config" / "robot_params.json"
        config = json.loads(path.read_text(encoding="utf-8"))
        names = [name for name, _ in json_to_set_commands(config)]
        self.assertEqual(sorted(names), sorted(KNOWN_PARAMS))

    def test_params_from_lines(self):
        lines = ["PARAM,base_pwm,45", "ACK,OTHER", "PARAM,line_follow_gain,0.01"]
        self.assertEqual(params_from_lines(lines), {"base_pwm": 45, "line_follow_gain": 0.01})


CALIBRATION = ([862.2, 746.2, 548.7, 629.8, 898.3],
               [2309.4, 2834.1, 2095.4, 2561.8, 2645.4])
WATCH_VALUES = ["1000", "1.0", "2.0", "0.5", "30", "-30", "10", "12",
                "862", "1800", "2500", "500", "0", "1"]


class NormalisationTests(unittest.TestCase):
    def test_limits_map_to_zero_and_one(self):
        white, black = CALIBRATION
        self.assertEqual(normalise_readings(white, white, black), [0.0] * 5)
        self.assertEqual(normalise_readings(black, white, black), [1.0] * 5)
        middle = [(w + b) / 2 for w, b in zip(white, black)]
        for value in normalise_readings(middle, white, black):
            self.assertAlmostEqual(value, 0.5)

    def test_out_of_range_readings_clamp(self):
        self.assertEqual(normalise_readings([0, 9999], [500, 500], [2000, 2000]), [0.0, 1.0])

    def test_invalid_calibration_reads_zero(self):
        self.assertEqual(normalise_readings([1500], [2000], [2000]), [0.0])
        self.assertEqual(normalise_readings([1500], [2000], [1000]), [0.0])

    def test_calibration_from_params(self):
        params = dict(zip(["white_dn%d" % i for i in range(1, 6)], CALIBRATION[0]))
        self.assertIsNone(calibration_from_params(params))
        params.update(zip(["black_dn%d" % i for i in range(1, 6)], CALIBRATION[1]))
        self.assertEqual(calibration_from_params(params), CALIBRATION)


class WatchTableTests(unittest.TestCase):
    def test_table_has_all_fields_and_fixed_height(self):
        tables = [
            format_watch_table(WATCH_VALUES, 20.0, 5.0, 0.5, "logging", CALIBRATION, False),
            format_watch_table(WATCH_VALUES, 20.0, 5.0, 0.5, "logging", CALIBRATION, True),
            format_watch_table(WATCH_VALUES, 20.0, 5.0, 0.5, "", None, True),
            format_watch_table(None, 0.0, None),
        ]
        for label, _ in WATCH_LABELS:
            self.assertIn(label, tables[0])
        self.assertEqual(len(WATCH_LABELS), len(DATA_COLUMNS))
        self.assertEqual(len({t.count("\n") for t in tables}), 1)
        self.assertIn("28.65 deg", tables[0])

    def test_normalised_mode_shows_calibrated_values(self):
        table = format_watch_table(WATCH_VALUES, 20.0, 5.0, 0.5, "", CALIBRATION, True)
        self.assertIn("NORMALISED", table)
        dn_rows = {line.split()[0]: line for line in table.splitlines()
                   if line.strip().startswith("DN")}
        self.assertIn(" 0.000 ", dn_rows["DN1"])   # white limit
        self.assertIn(" 0.505 ", dn_rows["DN2"])   # (1800-746.2)/(2834.1-746.2)
        self.assertIn(" 1.000 ", dn_rows["DN3"])   # above black limit, clamped
        self.assertIn("LINE", dn_rows["DN2"])
        self.assertIn("LINE", dn_rows["DN3"])
        self.assertNotIn("LINE", dn_rows["DN1"])

    def test_raw_mode_shows_raw_counts_but_uses_calibrated_threshold(self):
        table = format_watch_table(WATCH_VALUES, 20.0, 5.0, 0.5, "", CALIBRATION, False)
        self.assertIn("[RAW]", table)
        dn_rows = {line.split()[0]: line for line in table.splitlines()
                   if line.strip().startswith("DN")}
        self.assertIn(" 2500 ", dn_rows["DN3"])
        self.assertIn("LINE", dn_rows["DN3"])
        self.assertNotIn("LINE", dn_rows["DN4"])

    def test_normalised_mode_without_calibration_says_so(self):
        table = format_watch_table(WATCH_VALUES, 20.0, 5.0, 0.5, "", None, True)
        self.assertIn("no calibration", table)


if __name__ == "__main__":
    unittest.main()
