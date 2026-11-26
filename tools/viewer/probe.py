#!/usr/bin/env python3
"""Fast state log for the ROSbot 2R, to time a power cut and what precedes it.

Runs on the robot's host, standard library only. Each sample is one JSON line,
flushed to the card at once: the last line on disk is the last moment the Pi
was alive. Per sample: the Pi's throttle flags, whether the camera and lidar
are present on USB, and how many bytes the STM32 sent on its serial line.

The serial line is only read, never written. Bytes on it with no driver
running come from the firmware looking for its micro-ROS agent, so a change in
their rhythm before a cut would show a firmware restart.
"""
import argparse
import json
import os
import subprocess
import termios
import time

USB_DEVICES = {"camera": "/sys/bus/usb/devices/3-1", "lidar": "/sys/bus/usb/devices/1-1.3"}


def open_serial(path):
    """Open the STM32 line read-only, raw, 576000 baud, non-blocking."""
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                        # iflag
    attrs[1] = 0                                        # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                                        # lflag
    attrs[4] = termios.B576000
    attrs[5] = termios.B576000
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def drain(fd):
    """Bytes waiting on the serial line, without blocking."""
    chunks = []
    while True:
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            break
        if not chunk:
            break
        chunks.append(chunk)
    return b"".join(chunks)


def throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2, check=False).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    return out.strip().split("=")[-1] or None


def sample(serial_fd):
    record = {
        "wall": round(time.time(), 3),
        "up": round(time.clock_gettime(time.CLOCK_BOOTTIME), 3),
        "thr": throttled(),
        "load": os.getloadavg()[0],
    }
    for name, path in USB_DEVICES.items():
        record[name] = os.path.exists(path)
    if serial_fd is not None:
        data = drain(serial_fd)
        record["ser"] = len(data)
        record["hex"] = data[:16].hex()
    return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--serial", default=None, help="STM32 serial device to listen on")
    parser.add_argument("--period", type=float, default=0.25)
    parser.add_argument("--duration", type=float, default=3600.0)
    args = parser.parse_args()

    serial_fd = open_serial(args.serial) if args.serial else None
    deadline = time.monotonic() + args.duration
    with open(args.out, "a") as log:
        while time.monotonic() < deadline:
            started = time.monotonic()
            log.write(json.dumps(sample(serial_fd)) + "\n")
            log.flush()
            os.fsync(log.fileno())
            time.sleep(max(0.0, args.period - (time.monotonic() - started)))


if __name__ == "__main__":
    main()
