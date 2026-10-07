import argparse
import _thread
import socket
import threading

# Must match TELEOP_HOST and TELEOP_PORT in logger.py.
HOST = "127.0.0.1"
PORT = 9100
MAX_COMMAND_LEN = 63      # sizeof(Controller_c::cmd_buf) - 1; longer lines are cut short

HELP = """\
Commands (sent to the robot through logger.py):
  manual             switch to manual control, motors stopped
  pwm <left> <right> set motor PWM in manual mode, e.g. pwm 40 40, pwm -30 30
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
        threading.Thread(target=print_from_logger, args=(sock,), daemon=True).start()
        while True:
            command = input("> ").strip()
            if not command:
                continue
            if command == "help":
                print(HELP)
                continue
            if command == "quit":
                break
            if len(command) > MAX_COMMAND_LEN:
                print(f"too long ({len(command)} chars); the robot keeps {MAX_COMMAND_LEN}. Not sent.")
                continue
            sock.sendall(command.encode() + b"\n")
    except (EOFError, KeyboardInterrupt):
        print()
    except OSError as e:
        print(f"lost the logger: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
