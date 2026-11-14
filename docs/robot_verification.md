# Robot Verification

Evidence that the lab's Husarion ROSbot 2R works, sensor by sensor and actuator
by actuator. The simulation has its own record in [verification.md](verification.md).
Re-run whenever the robot's image tags, firmware or wiring change.

The check runs on the robot, in a throwaway container of the driver image, so a
failure is a hardware or driver failure and never a network one. It is started
from the development machine:

```bash
./robot/run_hw_check.sh --stage sensors                      # robot standing still
./robot/run_hw_check.sh --stage motors --wheels-off-ground   # all four wheels in the air
./robot/run_hw_check.sh --stage interactive                  # a person at the robot
./robot/run_hw_check.sh --stage distance                     # robot in a clear area
./robot/run_hw_check.sh --stage camera                       # robot in a clear area, camera started
```

Reports land in `robot/reports/`, one JSON file per stage.

## Setup checked

| Item | Value |
|---|---|
| Robot computer | Raspberry Pi 4 Model B, Husarion OS image build 2025-07-29, Ubuntu 22.04.5 |
| Driver | `husarion/rosbot:humble-0.14.0-20250319` |
| Firmware | 0.11.0. Read back from the STM32 and compared byte for byte with the image's `firmware.bin`: MATCH |
| Lidar | `husarion/rplidar:humble-1.0.1-20260522`, 256000 baud, firmware 1.32, 16 kHz sampling at 10 Hz. Model read as A2M12 class from the baud rate and sampling rate, not from a label |
| Camera | Orbbec Astra (USB `2bc5:0401`), `husarion/astra:humble-1.1.0-20240319` |
| Power | 12 V 5 A bench supply, no batteries fitted |

## First pass — 2026-11-14

Robot on the bench, where it stood. Three stages complete.

### Passive sensors: 19 of 19 PASS

| Check | Result | Measured |
|---|---|---|
| Pi under-voltage flag | PASS | `0x0` |
| `/joint_states`, `/imu_broadcaster/imu`, `/odometry/filtered`, `/rosbot_base_controller/odom` | PASS | 19.8 Hz each, expected 20 |
| `/range/fl`, `/range/fr`, `/range/rl`, `/range/rr` | PASS | 5.0 Hz each, expected 5 |
| `/scan`, `/scan_filtered` | PASS | 9.6 Hz, expected 10 |
| Supply voltage | PASS | 12.09 to 12.17 V |
| IMU at rest | PASS | gravity 9.98 m/s², rotation 0.0011 rad/s |
| Wheel joints | PASS | four joints named as expected |
| Range readings | PASS | 0.277 m, 0.102 m, 0.742 m; rear left `nan` (nothing in range, see findings) |
| Lidar scan | PASS | 1800 points per turn, 60 % valid, full 6.28 rad |

With the lidar stopped on purpose, `/scan` and `/scan_filtered` read
FAIL "no messages", every other row still passed and the exit code was 1.

### Motors: 6 of 6 PASS

Wheels off the ground, so without load.

| Check | Result | Measured |
|---|---|---|
| Forward, 0.10 m/s | PASS | all four wheels, mean 2.26 rad/s (expected 2.35) |
| Backward, 0.10 m/s | PASS | mean 2.40 rad/s |
| Turn left, 0.5 rad/s | PASS | mean 1.64 rad/s (expected 1.59), sides opposed |
| Turn right, 0.5 rad/s | PASS | mean 1.67 rad/s |
| Stop when commands end | PASS | all wheels at rest within 1 s, no stop command sent |
| Supply voltage while driving | PASS | minimum 12.04 V |

No wheel turned the wrong way.

### Interactive: 8 of 8 PASS

| Check | Result | Measured |
|---|---|---|
| Hand in front of each range sensor | PASS | 0.068 m, 0.130 m, 0.124 m, 0.094 m (fl, fr, rl, rr) |
| Left and right button of the pair | PASS | press received on `/button/left` and `/button/right` |
| Left and right green LED | PASS | seen lit by the operator |

The rear left sensor, which read `nan` in the passive stage, answered the hand:
it works.

## Second pass — pending

Both need the robot moved to a clear area. Not done yet.

| Stage | What it needs | What was seen so far, in an unprepared scene |
|---|---|---|
| `distance` | A box 0.5 m in front of the robot, nothing else within 1 m | Nearest object at 42° from the front. Says nothing yet about the lidar's orientation |
| `camera` | A clear view beyond 0.6 m, the camera's minimum range. Start it first with `./robot/deploy.sh --profile camera` | Colour 14.7 Hz and depth 14.3 Hz (expected 30), 7 % of pixels with depth. Objects stood 0.10 and 0.27 m in front, and the Pi's load was 7.7 |

The `distance` row is recorded as FAIL in `interactive.json` of the first pass,
from before the stage was split.

## Findings

### The Raspberry Pi loses power when the STM32 firmware restarts

Seven unclean resets on 2026-11-14, each a power-on reset
(`vcgencmd get_rsts` = `1000`) with no under-voltage warning before it.

What was ruled out, with the supply voltage logged throughout:

- Load. Four cores at 100 %, lidar spin-up and heavy disk reads together for
  150 s: no reset, 12 V input between 12.00 and 12.20 V.
- The 12 V supply sagging. It read flat up to the last sample before each cut.

What precedes the cuts:

| Trigger | Trials | Reset |
|---|---|---|
| STM32 leaves its bootloader (end of a flash, end of a firmware read-back) | 3 | 2, within 1.5 s |
| CORE2 reset button, the single button on the rear panel | 1 | 1, within 2 s |
| Clock stepped by 20 h while the driver runs | 2 | 2, about 30 s later; `/battery` stopped about 4 s before |
| Clock stepped by 20 h with the driver stopped | 3 | 0 |
| Driver container recreated | 1 | 1 |
| Stack started by a restart policy at boot | 2 | 1 |
| Driver stopped, nothing else | 3 | 0 |

The firmware source says it resets the microcontroller by itself when the
micro-ROS agent is lost or its start-up fails, which would turn the clock step
and the container restart into firmware restarts too. That link is a reading of
the source, not a measurement. Why a firmware restart cuts the Pi's power, and
why not every time, is not known. No code in the firmware touches the Pi's
supply.

Rules that follow, until the cause is found:

- Start and stop the stack only with `~/stack.sh up` and `~/stack.sh down` on
  the robot. It sets the clock while the driver is down and leaves NTP off.
- Never set the clock or switch NTP on while the driver runs.
- Do not press the single button on the rear panel.
- Flash or read back the firmware only when needed: expect a hard power cut at
  the end, with the risk that carries for the microSD card.

### Other findings

- **Topic names differ from the migration spec's contract** for driver 0.14.0:
  `/battery` (spec: `/battery_state`), `/imu_broadcaster/imu` (spec:
  `/imu/data`), `/rosbot_base_controller/odom` (spec: `/odometry/wheels`).
- **Range sensors report `nan` for "nothing in range".** The firmware replaces
  every reading outside 0.01 to 0.90 m with `nan`. Only a hand in front of the
  sensor tells a working sensor from a dead one.
- **The camera node needs the Pi's H.264 encoder.** Without `/dev/video11`
  mapped into the container it dies with a segmentation fault as soon as the
  colour stream starts.
- **The robot's NTP service is chrony.** `systemd-timesyncd` is masked in the
  Husarion image. `timedatectl set-ntp` switches whichever is installed.
- **The lidar starts 30 s after the driver**, to keep its motor's start-up
  current away from the driver's start.
- **The stack does not start at boot.** A restart policy turned one power cut
  into a reboot loop, so it was removed.
- **The supply held with the wheels in the air.** Driving on the floor draws
  more and has not been measured.
