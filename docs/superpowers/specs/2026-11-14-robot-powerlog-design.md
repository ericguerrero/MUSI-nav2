# ROSbot 2R power log — design

Date: 2026-11-14. Status: approved by Eric (all four parts).

## Intent

After a power loss or a "dead" robot, be able to say what happened: whether the supply
sagged or was cut, whether the Raspberry Pi or the STM32 went down first, and whether the
STM32 firmware is damaged. A 2023 thesis blamed "firmware corruption" on low supply voltage
with no evidence either way, and the robot keeps no record that could settle it.

## What the robot gives us (measured 2026-11-14)

- Raspberry Pi 4, Ubuntu 22.04, no RTC. Every boot starts at the same wall-clock time and the
  clock is only right after chrony reaches an NTP server. The robot has no internet today.
  Wall-clock time cannot order events across boots.
- The Pi cannot measure its supply voltage. The kernel `rpi_volt` hwmon device exposes one
  bit, "under-voltage seen in the last 2 s", and logs each change to the kernel ring buffer.
- The 12 V input is measured only by the STM32, published on `/battery`
  (`sensor_msgs/BatteryState`, 2.5 Hz) through the driver container. If the STM32 or the
  driver stops, the measurement stops; that silence is a signal.
- `sudo` on the robot needs a password; `docker` does not.
- The driver image `husarion/rosbot:humble-0.14.0-20250319` is on the robot and holds
  `rclpy`, `stm32flash`, the Husarion flasher and `firmware.bin` (244712 bytes). The flasher
  verifies after writing, so the STM32 flash can be read back.

## Goals

1. A continuous record that survives a power cut, with at most about 2 s lost.
2. For each boot, a statement of how the previous boot ended.
3. A yes or no answer to "is the firmware on the chip the pinned image?".
4. A readable timeline of the last boots.

Non-goals: measuring current, catching sags shorter than the 2 s poll (those show only as
kernel events), fixing the robot's internet or clock, alerting.

Anti-goal: the logger must never disturb the driver or write to the STM32.

## Approaches

- **Kernel journal only.** Already there, but it holds events and no voltage, and its
  timestamps follow the broken clock. Rejected.
- **`ros2 bag` of `/battery`.** Large, stops with the driver, and has nothing from the Pi
  side. Rejected.
- **Hardware logger (INA219 or similar on the 12 V input).** The best data, but a purchase
  and wiring on a shared robot. Kept as the next step if the software log is inconclusive.
- **Software logger in a container on the Pi (chosen).** Uses only what the robot has,
  needs no `sudo`, and starts at boot through Docker's restart policy.

## Design

All code lives in `robot/powerlog/`. It is deployed to `~/powerlog/` on the robot as its own
Compose project, so the main stack in `~/compose.yaml` is not touched.

### 1. Logger (`powerlog` service)

One container from the driver image, `network_mode: host`, `ipc: host`,
`restart: unless-stopped`, running as root with `/dev/kmsg` and `CAP_SYSLOG`.
Every 2 s it appends one JSON line to `~/powerlog/log/powerlog.jsonl` and calls `fsync`:

| Field | Meaning |
|---|---|
| `t` | record type: `start`, `s` (sample), `uv` (kernel under-voltage event), `stop` |
| `boot` | kernel boot ID |
| `up` | seconds since boot (monotonic) |
| `wall` | UTC wall clock, for reading only |
| `uv` | `rpi_volt` alarm bit: under-voltage in the last 2 s |
| `batt` | last `/battery` voltage, or `null` if none arrived in the last 5 s |
| `drv` | whether the driver's `controller_manager` node is visible |

Kernel under-voltage events are read from `/dev/kmsg` from the start of the ring buffer, so
events before Docker starts are kept. Each is written once per boot, with its kernel time.

The file rotates at 20 MB; ten files are kept (about 200 MB, a month of continuous running).

### 2. Boot record

The `start` record carries how the previous run ended, read from the last line on disk:
`stop` (the logger received a stop signal, so a shutdown or a deliberate stop), `cut` (no stop
record, so power was lost or the Pi reset), or `restart` (same boot ID: only the container
restarted). It repeats the last sample before the end.

### 3. `fwcheck` (on demand)

Stops the driver if it runs, puts the STM32 in its bootloader with the Husarion flasher's own
pin sequence, reads back the image length with `stm32flash -r`, compares it with
`/root/firmware.bin`, leaves the bootloader, and starts the driver again if it was running.
It never passes `-w`, `-k` or `-u`. Result: `MATCH`, `MISMATCH` (count and first offset) or
`READ FAILED`, printed and appended to `~/powerlog/log/fwcheck.jsonl`. The driver is down
for about one minute.

### 4. Report

`report.py`, standard library only, runs on the robot or on a copy of the log. For each of
the last N boots, in file order: duration, how it ended, voltage min, max and last, each
under-voltage event with its uptime, periods with no `/battery`, and gaps in the samples.

## Calls made here (mine, not discussed)

- A container instead of a systemd unit, because `sudo` needs a password on the robot.
- The `rpi_volt` alarm bit and kernel events instead of `vcgencmd get_throttled`. They carry
  the same under-voltage information without the host binary; the throttling and
  frequency-cap bits are dropped.
- `fsync` on every line. It costs roughly 0.5 to 1 GB of card writes per day of running.
- A `stop` record cannot tell a clean shutdown from someone stopping the logger by hand.
- Parsing and decision code is pure and unit-tested with `pytest` on the development
  machine; the rclpy node and `fwcheck` are checked on the robot.

## Limits

The log shows that power died and what the trend was before it. It does not show the
electrical cause. The last 2 s before a cut are lost.
