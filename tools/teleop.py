import argparse
import _thread
import socket
import threading
import time

# Must match TELEOP_HOST and TELEOP_PORT in logger.py.
HOST = "127.0.0.1"
PORT = 9100
MAX_COMMAND_LEN = 63      # sizeof(Controller_c::cmd_buf) - 1; longer lines are cut short
# The robot stops if no "pwm" or "vel" arrives for Controller_c::MANUAL_TIMEOUT_MS
# (500 ms), so a moving pwm/vel command is resent this often to keep it driving.
RESEND_INTERVAL_S = 0.2

HELP = """\
Commands (sent to the robot through logger.py):
  manual             switch to manual control, motors stopped
  vel <left> <right> set wheel speeds in mm/s in manual mode, e.g. vel 100 100;
                     the robot holds each wheel at its speed (max 250)
  pwm <left> <right> set raw motor PWM in manual mode, e.g. pwm 40 40, pwm -30 30
                     (vel and pwm are resent every 0.2 s until the next command)
  forward <mm> [speed]  drive that far straight, then stop, e.g. forward 200
  back <mm> [speed]     the same backwards, e.g. back 50
  twist <deg> [speed]   turn on the spot, + anticlockwise, e.g. twist 90, twist -45
  left <deg> [speed]    turn anticlockwise, e.g. left 90 (same as twist 90)
  right <deg> [speed]   turn clockwise, e.g. right 90 (same as twist -90)
                     (manual mode; sent once as move/turn and finish by themselves;
                     speed in mm/s per wheel, default move_max_speed_mm_s;
                     the robot replies "#I move done" / "#I turn done")
  move <mm> [speed]  the robot's own command behind forward/back
  turn <deg> [speed] the robot's own command behind twist/left/right
  stop               stop the motors (auto mode: back to waiting)
  auto               back to the line follower, waiting for start
  start              start the line follower (auto mode)
Local:
  help               show this list
  quit               exit (Ctrl+D or Ctrl+C also work); the logger then sends stop"""


def parse_args():
    parser = argparse.ArgumentParser(
        description="Type commands for the robot; logger.py forwards them over its connection.")
    parser.add_argument("--port", type=int, default=PORT, help=f"logger's teleop port (default: {PORT})")
    return parser.parse_args()


class Sender:
    """Sends commands to the logger and keeps the last moving pwm/vel command alive.

    The robot stops the motors when these commands stop arriving, so a lost link
    can't leave it driving. This resends the current one until another command
    replaces it.
    """

    def __init__(self, sock):
        self.sock = sock
        self.lock = threading.Lock()   # the resend thread and the prompt share the socket
        self.repeat = None             # encoded command to resend, or None

    def send(self, command):
        line = command.encode() + b"\n"
        with self.lock:
            self.sock.sendall(line)
            self.repeat = line if is_moving(command) else None

    def resend_forever(self):
        while True:
            time.sleep(RESEND_INTERVAL_S)
            with self.lock:
                if self.repeat is None:
                    continue
                try:
                    self.sock.sendall(self.repeat)
                except OSError:
                    return   # the prompt reports the lost logger on its next send


def is_moving(command):
    """Whether command is "pwm/vel <left> <right>" with either side non-zero."""
    parts = command.split()
    if len(parts) != 3 or parts[0] not in ("pwm", "vel"):
        return False
    try:
        return float(parts[1]) != 0 or float(parts[2]) != 0
    except ValueError:
        return False


# TurtleBot-style motion verbs, sent to the robot as its own "move"/"turn"
# with the distance or angle multiplied by the sign.
MOTION_VERBS = {
    "forward": ("move", 1), "fwd": ("move", 1), "f": ("move", 1),
    "back": ("move", -1), "backward": ("move", -1), "b": ("move", -1),
    "twist": ("turn", 1),
    "left": ("turn", 1), "l": ("turn", 1),
    "right": ("turn", -1), "r": ("turn", -1),
}


def translate(command):
    """Turns a motion verb into the robot's command, e.g. "right 90" -> "turn -90".

    Returns the command unchanged if it isn't a motion verb, or raises
    ValueError with a message for the user if its numbers don't parse.
    """
    parts = command.split()
    if parts[0] not in MOTION_VERBS:
        return command
    robot_cmd, sign = MOTION_VERBS[parts[0]]
    unit = "mm" if robot_cmd == "move" else "deg"
    if len(parts) not in (2, 3):
        raise ValueError(f"usage: {parts[0]} <{unit}> [speed]")
    try:
        amount = float(parts[1])
        speed = float(parts[2]) if len(parts) == 3 else None
    except ValueError:
        raise ValueError(f"usage: {parts[0]} <{unit}> [speed]") from None
    out = f"{robot_cmd} {sign * amount:g}"
    if speed is not None:
        out += f" {speed:g}"
    return out


def print_from_logger(sock):
    """Prints messages from the logger; ends the prompt when the logger goes away."""
    buffer = b""
    while True:
        try:
            data = sock.recv(4096)
        except OSError:
            data = b""
        if not data:
            print("\nlogger closed the connection.")
            _thread.interrupt_main()   # wakes input() in the main thread
            return
        buffer += data
        *lines, buffer = buffer.split(b"\n")
        for line in lines:
            print(f"[logger] {line.decode('utf-8', errors='replace')}")


def main():
    args = parse_args()

    try:
        sock = socket.create_connection((HOST, args.port), timeout=3)
    except OSError as e:
        print(f"can't reach the logger on {HOST}:{args.port} ({e}). Is logger.py running?")
        return
    # The logger answers first with "connected ..." or "busy ..."; read that
    # before starting the prompt, so a refusal ends here cleanly.
    greeting = b""
    try:
        while not greeting.endswith(b"\n"):
            chunk = sock.recv(256)
            if not chunk:
                break
            greeting += chunk
    except OSError:
        pass
    greeting = greeting.decode("utf-8", errors="replace").strip()
    if not greeting.startswith("connected"):
        print(greeting or "logger closed the connection.")
        sock.close()
        return
    print(f"{greeting}. Type 'help' for commands.")
    sock.settimeout(None)

    try:
        sender = Sender(sock)
        threading.Thread(target=print_from_logger, args=(sock,), daemon=True).start()
        threading.Thread(target=sender.resend_forever, daemon=True).start()
        while True:
            command = input("> ").strip()
            if not command:
                continue
            if command == "help":
                print(HELP)
                continue
            if command == "quit":
                break
            try:
                command = translate(command)
            except ValueError as e:
                print(f"{e}. Not sent.")
                continue
            if len(command) > MAX_COMMAND_LEN:
                print(f"too long ({len(command)} chars); the robot keeps {MAX_COMMAND_LEN}. Not sent.")
                continue
            sender.send(command)
    except (EOFError, KeyboardInterrupt):
        print()
    except OSError as e:
        print(f"lost the logger: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
