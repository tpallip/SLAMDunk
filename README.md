# SLAMDunk

Fork of the University of Bristol MSc Robotics "Robotics Science and Systems"
unit repository, containing the digital twin simulation platform and StampC3
robot template code used for coursework/labs on the module, plus our own
tooling for logging and benchmarking the robot.

Course guide: <https://semtm0043-42.github.io/getting-started.html>

## Repository layout

| Path | What it is |
|---|---|
| `StampC3_Template/` | Arduino sketch for the M5Stack StampC3 that controls the Pololu 3Pi+ over I2C and streams telemetry over WiFi |
| `DigitalTwin/` | Processing sketch: simulated robot, live comparison with the real robot |
| `tools/logger.py` | Python telemetry logger: live terminal dashboard and Rerun visualisation |
| `logs/` | Local captures (ignored by git) |
| `TODO.md` | Benchmarking plan for the final project |

## System overview

```
Pololu 3Pi+ (ATmega32U4, course middleware)
        │ I2C, address 0x08
M5Stack StampC3 (ESP32-C3) ── runs StampC3_Template
        │ WiFi access point, TCP 192.168.4.1:80, one client at a time
Laptop ── tools/logger.py  or  DigitalTwin
```

The robot sends one CSV line every 50 ms, in this order (see `StampC3_Template/Controller.h`):

```
t_ms, x_mm, y_mm, theta_rad, pwm_left, pwm_right, enc_left, enc_right, dn1, dn2, dn3, dn4, dn5, signal
```

`t_ms` is the StampC3's uptime. x, y and theta come from the StampC3's own
odometry. dn1–dn5 are the surface sensors. The line thresholds for each sensor should ideally be tuned individually for each individual robot. `signal` is 0 while waiting and 1 once the button has
started the controller.

---

## 1. Robot setup (Arduino)

**All of these instructions can also be found in the course guide (Link above or in your course blackboard). Treat this as distilled information from the course guide.**

### Arduino IDE

Install Arduino IDE 2.x from <https://www.arduino.cc/en/software>.

On Linux, use the **AppImage**, not the snap. The snap is sandboxed and cannot
open files outside your home folder (for example, a repo under `/data`).

```bash
chmod +x arduino-ide_*_Linux_64bit.AppImage
./arduino-ide_*_Linux_64bit.AppImage
```

If it fails to start with a FUSE error, run `sudo apt install libfuse2`.

For macOS and Windows check the course guide for instructions.

### Board package and libraries

1. **File → Preferences → Additional boards manager URLs**, add:
   ```
   https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
   ```
2. **Tools → Board → Boards Manager**: search `M5Stack` and install it.
3. **Tools → Manage Libraries**: install `LSM6` and `LIS3MDL` (both by Pololu).

### Flashing

1. Open `StampC3_Template/StampC3_Template.ino`.
2. Tools settings:
   - **Board:** M5Stack Arduino → **M5StampC3**
   - **USB CDC On Boot:** **Disabled** (otherwise the Serial Monitor shows nothing)
   - **Upload Speed:** 921600, or 115200 if uploads fail
3. **Set a unique WiFi name and password** on this line of the sketch:
   ```cpp
   RobotWifiAP_c server("myAP", "myPassword", 80);
   ```
   Every robot running the template broadcasts `myAP` by default, so in a lab
   you could end up connected to someone else's robot.
4. Upload, then turn the 3Pi+ on with its power button.

The Serial Monitor runs at **115200 baud** and shows the same CSV lines as WiFi.

### Serial port access (Linux)

Your user must be in the `dialout` group to upload over USB:

```bash
sudo usermod -aG dialout $USER     # then log out and back in
```

### Batteries

Use 4 × AA NiMH cells, fully charged; wait until every charger channel shows
green. **Motor speed at a fixed PWM drops as the batteries run down, which
affects tuning and odometry. Note the charge state in any benchmark.**

---

## 2. Digital Twin (Processing)

1. Install Processing 4 from <https://processing.org/download>.
2. Open `DigitalTwin/DigitalTwin.pde`.
3. Run it. It works standalone as a simulator. For live comparison it connects
   to the robot at `192.168.4.1:80` (set in `DigitalTwin/Simulator.pde`).

The robot serves only **one TCP client**, so the Digital Twin and the logger
cannot both be connected over WiFi at the same time.

---

## 3. Telemetry logger (`tools/logger.py`)

### Install

The logger needs **Python 3.10 or newer** and two packages:

- `rich` draws the live terminal dashboard.
- `rerun-sdk` includes the Rerun viewer (the `rerun` command). Tested with
  rerun-sdk 0.27.2. Rerun's API changes between releases, so if a newer
  version breaks something, install that version instead: `rerun-sdk==0.27.2`.

Install them in a **virtual environment** (`.venv`, a private folder of
packages for this project). Newer Linux distributions and Homebrew Python
refuse `pip install` outside one, and it keeps the versions separate from
the rest of your system. Run these from the repo's root folder.

#### Linux (Ubuntu/Debian)

```bash
sudo apt install python3 python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
pip install rich rerun-sdk
```

#### macOS

Install Python 3 from <https://www.python.org/downloads/> or with Homebrew
(`brew install python`). The `python3` that comes with Xcode's command line
tools may be older than 3.10; check with `python3 --version`.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install rich rerun-sdk
```

#### Windows

Install Python 3 from <https://www.python.org/downloads/> and tick
**"Add python.exe to PATH"** in the installer, or run
`winget install Python.Python.3.12`. Use Windows Terminal (PowerShell) so the
dashboard draws correctly.

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install rich rerun-sdk
```

If PowerShell refuses to run `Activate.ps1`, allow local scripts once with
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, then try again. In
Command Prompt (cmd) the activate command is `.venv\Scripts\activate.bat`.

#### Check the install

With the virtual environment active (your prompt starts with `(.venv)`):

```bash
python -c "import rich, rerun; print('ok', rerun.__version__)"
```

Activate the environment again (`source .venv/bin/activate`, or
`.venv\Scripts\Activate.ps1` on Windows) in each new terminal before running
the logger. `.venv/` is ignored by git.

### Connect to the robot

The robot is its own WiFi network. Join it with the name and password you set
in the sketch. While connected, your computer has no internet over WiFi.

You can connect to the the robot like you connect to a normal wifi network, or follow the commands below to connect from terminal.

**Linux:**

```bash
nmcli dev wifi connect <robot-ssid> password <robot-password>
```

To stop NetworkManager switching back to a network with internet in the
middle of a run, give the robot's network priority:

```bash
nmcli con modify <robot-ssid> connection.autoconnect-priority 100
```

**macOS:** choose the network from the WiFi menu in the menu bar, or:

```bash
networksetup -setairportnetwork en0 <robot-ssid> <robot-password>
```

(`en0` is the WiFi interface on most Macs; `networksetup -listallhardwareports`
shows yours.) macOS may warn that the network has no internet connection;
stay connected anyway.

**Windows:** choose the network from the WiFi icon in the taskbar. When
Windows reports "No internet", stay connected anyway. If Windows asks whether
to allow Python or Rerun through the firewall, allow it on private networks.

**Check that data is arriving** (all platforms, `Ctrl+C` to stop):

```bash
python -c "import socket; s=socket.create_connection(('192.168.4.1',80)); [print(d.decode(), end='') for d in iter(lambda: s.recv(4096), b'')]"
```

On Linux and macOS, `nc 192.168.4.1 80` does the same.


### Run

Settings are constants at the top of `tools/logger.py`:

| Constant | Robot | Local test | Meaning |
|---|---|---|---|
| `HOST` | `"192.168.4.1"` | `"127.0.0.1"` | Address to connect to (default for `--host`) |
| `PORT` | `80` | `9000` | TCP port (default for `--port`) |
| `TIMEOUT_S` | `3` | `3` | Seconds of silence before giving up |
| `USE_RERUN` | `True` | `True` | Open the Rerun viewer |
| `USE_CSV` | `True` | `True` | Save every record to a CSV file (default for `--csv`/`--no-csv`) |
| `LOG_DIR` | `logs/` | `logs/` | Folder for the CSV files |

Then, with the virtual environment active:

```bash
python tools/logger.py            # Linux, macOS
```

```powershell
python tools\logger.py            # Windows
```

Stop it with **Ctrl+C**.

The first time the Rerun viewer opens, macOS or Windows may ask whether to
allow incoming network connections. Allow it: the logger sends data to the
viewer over a local network port.


### What you see

**Terminal:** a table of the latest record with every field labelled, updated
about 10 times a second. Warnings and errors are printed above the table.

**Rerun viewer** (opens automatically when `USE_RERUN = True`):

| Panel | Shows |
|---|---|
| Path | The robot's trail and a heading arrow (odometry x, y, theta) |
| Pose | x, y, theta over time |
| Motor PWM | Commanded left and right PWM |
| Encoders | Left and right encoder counts |
| Surface sensors | dn1–dn5 over time |
| Surface now | Bar chart of the current dn1–dn5 |

Notes on the viewer:

- The time axis is the robot's own clock (`robot_time`), so WiFi delays don't
  distort the plots. Drag the time cursor to scrub back through a run.
- The Path view draws y flipped, because Rerun's 2D +y points down and the
  robot's +y is anticlockwise (up). The Pose plots show the true y.
- If a viewer is already open, the logger adds a new recording to it rather
  than opening a second window. Pick recordings from the list on the left.
- After a robot reset, `t_ms` restarts near 0, so the new data is drawn over
  the old run on the same timeline.
- Without a display (for example, over SSH) the viewer cannot open. Set
  `USE_RERUN = False`.

**CSV file** (on by default): every valid record is saved to
`logs/run_YYYYMMDD_HHMMSS.csv`, one file per run, named by the time the logger
started. Options:

```bash
python tools/logger.py --name straight_line   # save to logs/straight_line.csv
python tools/logger.py --no-csv               # don't save a CSV this run
```

`--name` won't overwrite an existing file: the logger stops before
connecting and asks for another name. The first row is the column names (`t_ms`, `x_mm`, ... `signal`) and
each record is written as soon as it arrives, so the file is complete even
after `Ctrl+C`. Skipped lines (wrong field count, not a number) are not saved.
`logs/` is ignored by git.

### Log messages

| Message | Meaning |
|---|---|
| `not on the robot's WiFi` | The laptop has no route to 192.168.4.1. Join the robot's network. |
| `Server not listening. Robot must be booting.` | Something answered, but nothing is listening on the port. Wait for the robot to finish booting, or check `nc` is running for a local test. |
| `robot did not answer within 3 s` | No answer to the connection at all: robot off or out of range. |
| `no data for 3 s` | Connected, but the data stopped: robot reset, out of range, laptop left the network, or another client holds the connection. |
| `connection ended: Closed by the server.` | The other side closed the connection normally. |
| `expected 14 fields, got N` / `not a number` | A line that isn't a telemetry record was received and skipped. |
| `discarding incomplete last line` | The connection ended partway through a line. |

### Test without the robot

`tools/fake_robot.py` stands in for the robot on your computer: it replays any
CSV saved by the logger as live telemetry (every 50 ms, on a loop) and prints
every command it receives. Commands change the replayed data the way the
firmware would (`manual` + `pwm 40 40` shows up in the pwm columns, `start`
sets `signal`), so you can test the logger and the teleop end to end. No
robot or robot WiFi is needed.

1. Terminal 1, the fake robot (any CSV from `logs/` works):
   ```bash
   python tools/fake_robot.py logs/all_white.csv
   ```
2. Terminal 2, the logger pointed at it:
   ```bash
   python tools/logger.py --host 127.0.0.1 --port 9000
   ```
3. Terminal 3, optionally, the teleop:
   ```bash
   python tools/teleop.py
   ```

The fake robot prints each command and its effect, for example
`robot got 'pwm 40 40' -> mode=manual signal=0 pwm=40,40`. It keeps running
after the logger disconnects and waits for the next one; stop it with
`Ctrl+C`. `--port` changes its port.

To test against a particular recording, save one from the robot with the
logger (`--name my_capture`) and replay that file.


### Teleop: send commands to the robot (`tools/teleop.py`)

The robot accepts one TCP client, so commands go through the logger's
connection. The logger listens on `127.0.0.1:9100` (this computer only) for a
teleop client and forwards each line it receives to the robot. Start the
logger first, then in a second terminal:

```bash
python tools/teleop.py
```

```
> manual           # manual control, motors stopped
> vel 100 100      # left/right wheel speed in mm/s, held by the robot (max 250)
> vel -60 60       # turn on the spot
> pwm 40 40        # raw left/right motor PWM, clamped on the robot to ±100
> move 10          # drive 10 mm straight, then stop (move -10: backwards)
> stop             # stop the motors
> auto             # back to the line follower, waiting
> start            # start the line follower
> help             # list commands; quit (or Ctrl+D) exits
```

- The logger prints each forwarded command (`teleop -> robot: ...`) above the
  table, and the CSV keeps recording while you drive.
- Use `vel` to drive straight. The two motors differ (the left one needs more
  PWM to start and is slower), so equal `pwm` values make the robot curve.
  `vel` closes the loop on each wheel's encoder; see "Wheel speed control".
- `move <mm>` drives a set distance (up to 500 mm) on the encoders, through
  the speed loop, and stops by itself; Serial prints how far each wheel ended
  from its target. It slows down near the end and stops each wheel early by
  the distance it coasts (`MOVE_STOP_LEAD_S`): if steps overshoot on your
  robot, raise it; if they stop short, lower it. Any other motion command or
  `stop` cancels a move.
- In manual mode the robot stops the motors if no `pwm` or `vel` command
  arrives for 500 ms (`MANUAL_TIMEOUT_MS` in `Controller.h`), so losing WiFi
  can't leave it driving. The teleop resends a moving `pwm`/`vel` every 0.2 s
  until your next command, so a typed `vel 100 100` keeps going. A program
  that sends commands some other way must repeat them itself.
- When the teleop exits or disconnects, the logger sends `stop`. This also
  stops a line-follower run started from the teleop.
- One teleop at a time; a second one is told the logger is busy.
- `--no-teleop` runs the logger without the teleop port. The port is set by
  `TELEOP_PORT` in `tools/logger.py` (and `PORT` in `tools/teleop.py`).

### Wheel speed control (`vel`)

`WheelSpeedController_c` (`StampC3_Template/WheelSpeedController.h`) holds one
wheel at a target speed. Each control cycle (10 ms) it sets

```
pwm = feedforward + SPEED_KP * error + SPEED_KI * integral(error)
feedforward = offset + target / slope        (per wheel)
```

- **Feedforward** gets each motor close at once despite its deadband and
  strength. The constants in `Controller.h` (`LEFT_FF_*`, `RIGHT_FF_*`) are a
  straight-line fit of encoder speed against PWM from
  `logs/run_20261011_172405.csv`: left 17 PWM + 3.5 mm/s per PWM, right
  12 PWM + 3.75 mm/s per PWM. Refit them for a different robot.
- **PI** removes the rest: battery level, load. The integral is cleared when
  the target is zero or changes direction, and stops growing while the output
  sits at ±100 PWM. `SPEED_KP = 0.3`, `SPEED_KI = 3` were chosen in simulation
  (about half the slow-speed wobble of the earlier 0.1 / 1.0).
- **Stiction compensation.** A motor needs much more PWM to start than to
  keep turning (on the stand: starts at 30-40+, keeps turning at ~13 left /
  ~9 right), and the start threshold varies. Two guards:
  - *Kick:* when a wheel should be turning but no encoder count has arrived
    for 2.5 count periods at the target speed (at least 30 ms), its PWM starts
    at the breakaway value (`LEFT_BREAKAWAY_PWM = 30`, `RIGHT_BREAKAWAY_PWM =
    24`) and rises 200 PWM/s until it moves, with the integral held.
  - *Floor:* while a wheel is meant to turn, its PWM stays at least
    `LEFT_RUN_MIN_PWM = 13` / `RIGHT_RUN_MIN_PWM = 9`, so the PI can't pull a
    wheel that has just lurched free back down until it sticks again.
- **Speed** is measured over a window long enough for about 6 counts at the
  previous speed (30-80 ms), capped at one count over the time since the last
  count, so a stall shows within a few cycles.

#### Speed test (`tools/speed_test.py`)

With the logger running (teleop on) and the robot on a stand:

```bash
python tools/speed_test.py run                    # vel 20, 30, 50, 80: 4 s each, stops between
python tools/speed_test.py report logs/<run>.csv  # after stopping the logger
```

`--spin` spins on the spot instead (for the floor), `--speeds` sets the
steps. The report gives, per step and wheel: mean speed and error, wobble (sd
over 100 ms windows, as mm/s and % of target), p5-p95, time stuck, time to
90 %, start overshoot, PWM mean and sd; plus `summary.csv` and
`speed_steps.png` in `bench/2.6_speed_control/<run>/`. Run it before and after
changing a gain to see the effect.

Tuning: lots of wobble but little stuck time, raise `SPEED_KI` (or
`SPEED_KP`); large start overshoot, lower `SPEED_KI`; still stuck often, raise
the run floor (`*_RUN_MIN_PWM`); slow targets running fast, lower it.

`pwm` still sets the motors directly, bypassing the speed loop.

### Surface sensor spread (`tools/sensor_spread.py`)

Measures how wide a patch of floor each surface sensor sees (TODO 3.4).
Spin the robot on the spot over a straight black/white edge; each sensor
sweeps a 45 mm circle and crosses the edge twice per turn, and the readings
against distance from the edge give its footprint.

1. Pattern: a straight edge with at least 70 mm of solid black on one side and
   white on the other (half of an A4 sheet printed black works). Matte, flat,
   taped down.
2. Put the midpoint between the wheels over the edge, roughly; the script
   measures how far off it is.
3. Logger running, then in the teleop: `manual`, `vel -25 25` for about six
   turns (~70 s), `stop`, `vel 25 -25` for six more, `stop`.
4. `python tools/sensor_spread.py logs/<run>.csv --plot`

It finds the spin from the opposite-sign PWM columns and reports per sensor:
sigma (Gaussian footprint), FWHM, 10-90 % rise distance, white/black levels,
and the odometry heading scale it calibrated from the crossings. Results go
to `bench/3.4_spatial_response/<run>/` (`summary.csv`, `profiles.csv`, and
`spread.png` with `--plot`, which needs `pip install matplotlib`). Measure
each sensor's distance from the wheel midpoint and pass it with
`--radius 45 45 45 45 45`; the spread scales with it. If the script warns the
samples are coarse, spin slower or set `TELEMETRY_INTERVAL_MS` to 20.

### Floor map from a run (`tools/surface_map.py`)

Rebuilds the floor pattern the surface sensors passed over. It recomputes the
pose from the encoder columns and places each reading at its sensor's floor
position, so any motion works, including turns that don't stay centred.

```bash
python tools/surface_map.py logs/<run>.csv --fit-separation --fill 3 --plot
```

- Output in `bench/surface_map/<run>/`: `map.png` (1 px per `--cell` mm, +y
  up, grey where no sensor passed), `map.npz` (the grid), and `map_plot.png`
  with mm axes and the robot's path (`--plot`, needs matplotlib).
- The wheel separation matters most: a few % off twists later passes against
  earlier ones. `--fit-separation` picks the value at which overlapping passes
  agree best; it needs passes that cross the same floor.
- All five sensors sit on one 45 mm circle, so a spin on the spot images only
  a thin ring. Move between spins or drive in lanes to cover an area; `--fill`
  closes small gaps between tracks.
- Sensor positions default to the Digital Twin's (`--radius`, `--angles`).
  Measure your robot's: on a spin all five sensors sweep the same ring and
  their samples interleave, but each degree of angle error shifts a sensor's
  samples about 0.8 mm.
- Readings are interpolated along each sensor's track (`--no-interp` to turn
  off). `--sharp` adds `map_sharp.png`, black or white at the 50 % level.
- For a sharp map, record slowly: spin with `vel -20 20` (not raw `pwm`, which
  spins fast and slips) and, for mapping runs, set `TELEMETRY_INTERVAL_MS` to
  10. That puts samples about 0.2 mm apart instead of 5 mm.

### Surface sensor black/white levels (`tools/sensor_levels.py`)

From a run over tape (such as a slow spin), labels each reading black when
the sensor was well inside a tape pass and white when well away from any edge
(`--margin`, default 5 mm), skipping readings near edges. Per sensor it
reports white and black mean, noise and extremes, the separation, the gap
between the classes, a suggested threshold, and errors at `LINE_THRESHOLD`.

```bash
python tools/sensor_levels.py logs/<run>.csv --separation 83.2
```

Results go to `bench/3.2_sensor_levels/<run>/levels.csv`.

### Statistics from a saved run (`tools/stats.py`)

Give the CSV file and one or more column names:

```bash
python tools/stats.py logs/run_20261007_171309.csv dn1 dn2 dn3
```

For each column it prints the mean, minimum, maximum, standard deviation and
variance over the whole run, plus the number of records and the run length
(from `t_ms`). The standard deviation and variance are population values,
since the file holds every record of the run. An unknown column name prints
the list of valid ones.

### Current limitations

- **No automatic reconnect.** The logger exits when the connection drops.
  Restart it after the robot resets.
- **One client at a time.** While the logger is connected, the Digital Twin
  (or a second logger) will connect but receive nothing.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Arduino IDE can't see the repo folder | You're using the snap version. Install the AppImage (see above). |
| Serial Monitor is blank | Set **USB CDC On Boot → Disabled** and re-upload; check the baud rate is 115200. |
| Upload fails or port not found | Add yourself to `dialout`; try Upload Speed 115200. |
| Robot doesn't turn on | Press the 3Pi+ power button; check the cells' orientation and charge. |
| Robot's WiFi doesn't appear | Robot is off, still booting, or the sketch isn't flashed. |
| Logger connects, then exits with `no data for 3 s` | Another client (Digital Twin, `nc`, another logger) may hold the connection. Close it, wait a few seconds, and try again. |
| `ModuleNotFoundError: rich` or `rerun` | The virtual environment isn't active in this terminal. Activate it (see Install), or install the packages into it. |
| `error: externally-managed-environment` from pip | You ran pip outside the virtual environment. Create and activate `.venv` first (see Install). |
| Windows: `python` opens the Microsoft Store or isn't found | Use `py` instead of `python`, or reinstall Python with "Add python.exe to PATH" ticked. |
| Windows: `Activate.ps1 cannot be loaded` | Run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once in PowerShell. |
| Logger says `robot did not answer` instead of `not on the robot's WiFi` | Normal on some systems when you aren't on the robot's network: the connection times out instead of failing straight away. Check which WiFi you're on. |
| Rerun viewer doesn't open (macOS/Windows) | Allow the firewall or "incoming connections" prompt, then run the logger again. |
