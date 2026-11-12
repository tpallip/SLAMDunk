# TCP sensor logger

The logger connects to the StampC3's existing Wi-Fi access point and TCP server.
Connect the computer to that access point first, then run:

```powershell
python tests\tcp_logger.py
```

By default the script connects to `192.168.4.1:80`. Override these values with
`--host` and `--port` if the firmware network configuration differs. Completed
trial samples are appended to `telemetry.csv`; use `--telemetry-csv` to change
the path. Existing contents are not overwritten.

Enter commands in the logger:

- `start telemetry` begins the periodic telemetry stream.
- `stop telemetry` ends the periodic telemetry stream.
- `trial <trial_no>, <percentage>` collects 500 fresh sensor readings, then
  appends them to `telemetry.csv` with columns
  `timestamp,trial_no,percentage,reading0,reading1,reading2,reading3,reading4`.
- `timestamp` is milliseconds since the robot controller started.
- `status` requests an immediate telemetry record.
- `motor <left> <right>` sets motor PWM values from -100 to 100.
- `stop` stops both motors.
- `quit` sends a stop request and exits.

Periodic telemetry is off by default and streams approximately every 50 ms only
between `start telemetry` and `stop telemetry`. A trial collects one reading
per successful sensor update (normally about every 10 ms). Motor commands are
limited to -100..100, and the firmware requests a stop if the TCP client
disconnects while a non-zero motor command is active.

Each trial appends its own header and 500 rows. Existing telemetry data is
preserved, so earlier records with a different schema remain unchanged.

The TCP protocol uses one newline-terminated command per line:

- `START TELEMETRY`
- `STOP TELEMETRY`
- `TRIAL,<trial>,<percentage>`
- `STATUS`
- `MOTOR,<left_pwm>,<right_pwm>`
- `STOP`

Responses include `TRIAL_SAMPLE,...` rows, `TRIAL_STARTED,...`,
`TRIAL_DONE,...`, periodic or requested `DATA,...` records, `ACK,...`
acknowledgements, and `ERROR,...` diagnostics.
