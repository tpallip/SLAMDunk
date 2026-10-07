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
> pwm 40 40        # left/right motor PWM, clamped on the robot to ±100
> pwm -30 30       # turn on the spot
> stop             # stop the motors
> auto             # back to the line follower, waiting
> start            # start the line follower
> help             # list commands; quit (or Ctrl+D) exits
```

- The logger prints each forwarded command (`teleop -> robot: ...`) above the
  table, and the CSV keeps recording while you drive.
- When the teleop exits or disconnects, the logger sends `stop`. This also
  stops a line-follower run started from the teleop.
- One teleop at a time; a second one is told the logger is busy.
- `--no-teleop` runs the logger without the teleop port. The port is set by
  `TELEOP_PORT` in `tools/logger.py` (and `PORT` in `tools/teleop.py`).

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
