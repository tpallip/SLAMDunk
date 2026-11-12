import unittest
from unittest.mock import Mock, patch

from tcp_logger import (
    DATA_COLUMNS,
    TRIAL_COLUMNS,
    build_command,
    connect_to_robot,
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


if __name__ == "__main__":
    unittest.main()
