# ROSbot 2R Hardware Check Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One repeatable bench test that exercises every sensor and actuator of the lab's ROSbot 2R and records a PASS or FAIL per item.

**Architecture:** The robot stack is a versioned `robot/compose.yaml` (driver, lidar, optional camera). The check is a small Python tool in `robot/hw_check/`: pure decision functions in `checks.py`, unit-tested off the robot, and one rclpy runner `hw_check.py` that samples topics and calls them. The runner executes on the robot in a throwaway container of the driver image, so a failure is a hardware or driver failure and never a network one.

**Tech Stack:** ROS 2 Humble, rclpy, Docker Compose, pytest, bash. Images: `husarion/rosbot:humble-0.14.0-20250319`, `husarion/rplidar:humble-1.0.1-20260522`, `husarion/astra:humble-1.1.0-20240319`.

**Spec:** No design spec exists for hardware bring-up. Sources this plan argues from:
- `~/wiki/musi-11764/projects/rosbot2r-first-setup.md`, "Phase 4 — first tests".
- `docs/superpowers/specs/2026-09-24-rosbot2r-migration-design.md` §2.1 (topic contract), on branch `feature/rosbot2r-migration`.
- Values measured on the robot on 2026-11-14, listed under "Measured baseline" below.

## Global Constraints

- Python only. Files at most 500 lines, functions at most 50 lines.
- Robot access: `ssh husarion@192.168.77.2` from `atlas`, key login. `sudo` on the robot asks for a password; read it from the environment variable `ROSBOT_SUDO_PASS`, never write it to a file.
- Nothing is installed on the robot's host with `apt`. The robot gets Docker images and files under `/home/husarion` only.
- Do not run `setup_robot_configuration`, `factory_reset.sh` or `flash_firmware.sh` on the robot.
- Motor commands are capped in code: 0.15 m/s linear, 0.6 rad/s angular. The motor stage runs only with the wheels off the ground.
- Rates are measured with the receiver's monotonic clock, not message stamps. The robot has no real-time clock and its time is wrong without internet.
- Branch `feature/rosbot2r-hw-check` from `feature/control-platform`. Commits are conventional, `type(scope): subject`, with no attribution footer.
- Other sessions work in this repository. Create the worktree with EnterWorktree before the first edit.

## Measured baseline (robot, 2026-11-14, driver 0.14.0, firmware 0.11.0)

| Topic | Type | Rate |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz |
| `/imu_broadcaster/imu` | `sensor_msgs/Imu` | 20 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | 20 Hz |
| `/rosbot_base_controller/odom` | `nav_msgs/Odometry` | 20 Hz |
| `/range/fl`, `/range/fr`, `/range/rl`, `/range/rr` | `sensor_msgs/Range` | 5 Hz, `min_range` 0.01, `max_range` 0.90 |
| `/battery` | `sensor_msgs/BatteryState` | slow; 12.15 to 12.20 V on the 12 V supply |
| `/button/left`, `/button/right` | `std_msgs/Bool` | on event |
| `/led/left`, `/led/right` | `std_msgs/Bool` | subscribed by the firmware |
| `/cmd_vel` | `geometry_msgs/Twist` | subscribed by `rosbot_base_controller` |
| `/scan` | `sensor_msgs/LaserScan` | no publisher yet: the lidar container does not exist |

- Joint names: `fl_wheel_joint`, `fr_wheel_joint`, `rl_wheel_joint`, `rr_wheel_joint`. `effort` is `nan`.
- Controller: `wheel_radius` 0.0425, `wheel_separation` 0.186, `wheel_separation_multiplier` 1.45, `cmd_vel_timeout` 0.5.
- Three names differ from the spec's contract: `/battery` (spec: `/battery_state`), `/imu_broadcaster/imu` (spec: `/imu/data`), `/rosbot_base_controller/odom` (spec: `/odometry/wheels`). Task 6 records this; this plan uses the measured names.
- The robot has no internet after a reboot, and `vcgencmd get_throttled` reads `0x50000` (one 2-second under-voltage at boot, none since).

## Review Focus

1. **The SSH link drops during the motor stage.** Expected: the wheels stop. The runner's motor sequence is bounded and ends by itself; Task 4 tests that its total duration is at most 30 s and that the stop check runs with no zero command sent.
2. **A topic has no publisher** (lidar not started, camera off). Expected: that item reports FAIL "no messages", every other item still runs, exit code 1. Task 3 tests it.
3. **A message carries `nan`** (`effort` already does). Expected: FAIL for that item, no exception. Task 2 tests `nan` in every numeric check.
4. **The robot clock is wrong or jumps.** Expected: rates are still correct. Task 2 fixes `check_rate` to receive times; Task 3 passes `time.monotonic()`.
5. **The motor stage is started with the wheels on the floor.** Expected: it refuses. Task 4 tests that `--stage motors` without `--wheels-off-ground` exits before any publisher is created.

---

### Task 1: Versioned robot stack with the lidar

**Files:**
- Create: `robot/compose.yaml`, `robot/.env`, `robot/deploy.sh`
- Modify: `.gitignore` (add `robot/reports/`)

**Interfaces:**
- Produces: containers `rosbot` and `rplidar` running on the robot; `/scan` published; `robot/deploy.sh [global compose options]` copies `robot/compose.yaml` and `robot/.env` to `husarion@192.168.77.2:~/` and runs `docker compose <options> pull` and `docker compose <options> up -d` there. The host is overridable with `ROBOT_HOST`.

- [ ] **Step 1: Write `robot/compose.yaml`**

```yaml
# ROSbot 2R stack. Driver image and STM32 firmware are a pair: 0.14.0 with 0.11.0.
x-common: &common
  network_mode: host
  ipc: host
  restart: unless-stopped

services:
  rosbot:
    <<: *common
    image: husarion/rosbot:humble-0.14.0-20250319
    container_name: rosbot
    devices:
      - /dev/ttySERIAL
    command: ros2 launch rosbot_bringup bringup.launch.py mecanum:=False

  rplidar:
    <<: *common
    image: husarion/rplidar:humble-1.0.1-20260522
    container_name: rplidar
    devices:
      - /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_2473bdf883f7a4499f60d30ee717398b-if00-port0:/dev/ttyUSB0
    command: >
      ros2 launch /husarion_utils/rplidar.launch.yaml
        channel_type:=serial
        serial_baudrate:=${RPLIDAR_BAUDRATE}

  astra:
    <<: *common
    profiles: [camera]
    image: husarion/astra:humble-1.1.0-20240319
    container_name: astra
    devices:
      - /dev/bus/usb/
    command: ros2 launch /husarion_utils/astra.launch.py device_namespace:=camera
```

`robot/.env` holds one line: `RPLIDAR_BAUDRATE=256000`.

- [ ] **Step 2: Validate the file**

Run: `docker compose -f robot/compose.yaml --env-file robot/.env config --quiet && echo OK`
Expected: `OK`

- [ ] **Step 3: Give the robot internet (runtime only, lost on reboot)**

The robot's Ethernet socket must be on the office switch. Run:
`ssh -t husarion@192.168.77.2 "sudo nmcli device modify eth0 ipv4.method auto ipv4.addresses 192.168.77.2/24 && sleep 8 && ping -c 2 1.1.1.1"`
Expected: 2 of 2 packets answered, and `ip -4 -br addr show eth0` lists a second `192.168.0.x` address.

- [ ] **Step 4: Write `robot/deploy.sh` and deploy**

`deploy.sh` copies the two files with `scp`, then runs `ssh "$ROBOT_HOST" "docker compose $* pull && docker compose $* up -d"`. Its arguments are global Compose options such as `--profile camera`, which go before the subcommand. Use `set -euo pipefail`.
Run: `./robot/deploy.sh`
Expected: `docker ps` on the robot shows `rosbot` and `rplidar` with status `Up`.

- [ ] **Step 5: Confirm the lidar launch file and baud rate**

Run: `ssh husarion@192.168.77.2 "docker logs --tail 30 rplidar"`
- If the log reports a missing launch file: list `/husarion_utils` in the image and correct the `command`.
- If the log reports an operation timeout: the unit is an A2M8. Set `RPLIDAR_BAUDRATE=115200` in `robot/.env` and deploy again.

Then run: `ssh husarion@192.168.77.2 "docker exec rosbot bash -c 'source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /scan'"`
Expected: an average rate between 5 and 15 Hz. Record the lidar model and firmware lines from the log for Task 6.

- [ ] **Step 6: Commit**

```bash
git add robot/compose.yaml robot/.env robot/deploy.sh .gitignore
git commit -m "feat(robot): add versioned ROSbot 2R stack with lidar"
```

### Task 2: Check functions

**Files:**
- Create: `robot/hw_check/checks.py`
- Test: `tests/test_hw_checks.py`
- Modify: `tests/run_unit_tests.sh` (add the mount `-v "${REPO}/robot:/robot:ro"`)

**Interfaces:**
- Produces, all in `checks.py`, with no ROS import:

```python
WHEEL_JOINTS = ("fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint")
WHEEL_RADIUS = 0.0425
WHEEL_SEPARATION = 0.186 * 1.45
MAX_LINEAR = 0.15
MAX_ANGULAR = 0.6

@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: str

def check_rate(name: str, receive_times: list[float], expected_hz: float, tolerance: float) -> CheckResult
def check_voltage(volts: list[float], minimum: float = 11.0, maximum: float = 13.0) -> CheckResult
def check_imu_stationary(orientation: tuple, accel: tuple, gyro: tuple) -> CheckResult
def check_joint_names(names: list[str]) -> CheckResult
def check_range_reading(sensor: str, value: float, min_range: float, max_range: float) -> CheckResult
def check_scan(ranges: list[float], range_min: float, range_max: float, angle_min: float, angle_max: float) -> CheckResult
def check_throttled(value: int) -> CheckResult
def expected_wheel_velocities(linear: float, angular: float) -> dict[str, float]
def check_wheel_response(segment: str, measured: dict[str, float], expected: dict[str, float], tolerance: float = 0.30) -> CheckResult
def check_stop(velocities: dict[str, float], limit: float = 0.1) -> CheckResult
def nearest_bearing_in_base(ranges: list[float], angle_min: float, angle_increment: float, laser_yaw: float, max_distance: float = 1.0) -> float | None
def check_depth_image(valid_fraction: float, width: int, height: int) -> CheckResult
```

Rules the signatures do not fix:
- `check_rate`: fewer than 2 times is FAIL with detail `no messages`. Rate is `(n - 1) / (last - first)`.
- `check_imu_stationary`: PASS when the quaternion norm is within 0.01 of 1, the acceleration norm is within 0.5 of 9.81, and the gyro norm is below 0.05.
- `check_range_reading`: `nan` or a negative value is FAIL. `inf` or a value above `max_range` is PASS with detail `no target in range`.
- `check_scan`: PASS when at least 30 % of the ranges are finite and inside `[range_min, range_max]`, and `angle_max - angle_min` is at least 6.0.
- `check_throttled`: FAIL when any of bits 0 to 3 is set (a fault present now). Higher bits are history and go into `detail` only.
- `expected_wheel_velocities`: left wheels `(linear - angular * WHEEL_SEPARATION / 2) / WHEEL_RADIUS`, right wheels with `+`.
- `check_wheel_response`: each wheel must have the expected sign and a magnitude within `tolerance` of the expected one. The detail names every failing wheel.
- `nearest_bearing_in_base`: angle of the nearest finite return closer than `max_distance`, plus `laser_yaw`, wrapped to `(-pi, pi]`. `None` when there is no such return.
- `check_depth_image`: PASS when the size is 640 by 480 and `valid_fraction` is at least 0.20.
- Any `nan` input gives FAIL, never an exception.

- [ ] **Step 1: Write the failing tests in `tests/test_hw_checks.py`**

Import with `sys.path.insert(0, "/robot/hw_check")`, as `tests/test_trajectory_analyzer.py` does for its module. Cases, one test each:

```python
# check_rate
times_20hz = [i * 0.05 for i in range(200)]
assert check_rate("j", times_20hz, 20.0, 0.10).passed
assert not check_rate("j", [i / 15 for i in range(150)], 20.0, 0.10).passed
assert check_rate("j", [], 20.0, 0.10).detail == "no messages"
# check_voltage
assert check_voltage([12.15, 12.20]).passed
assert not check_voltage([12.1, 10.4]).passed
assert not check_voltage([]).passed
assert not check_voltage([12.1, float("nan")]).passed
# check_imu_stationary, with the sample measured on the robot
assert check_imu_stationary((-0.0078125, -0.06524658, 0.0, 0.99786377), (1.28, -0.12, 9.91), (0.0, 0.0011, 0.0022)).passed
assert not check_imu_stationary((0, 0, 0, 1), (0.0, 0.0, 0.0), (0, 0, 0)).passed
assert not check_imu_stationary((0, 0, 0, 1), (0.0, 0.0, float("nan")), (0, 0, 0)).passed
# check_joint_names: order differs on the robot (fr, rl, rr, fl)
assert check_joint_names(["fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint", "fl_wheel_joint"]).passed
assert not check_joint_names(["fr_wheel_joint", "rl_wheel_joint"]).passed
# check_range_reading
assert check_range_reading("fl", 0.277, 0.01, 0.90).passed
assert check_range_reading("fl", float("inf"), 0.01, 0.90).detail == "no target in range"
assert not check_range_reading("fl", float("nan"), 0.01, 0.90).passed
assert not check_range_reading("fl", -1.0, 0.01, 0.90).passed
# check_scan
assert check_scan([1.0] * 500 + [float("inf")] * 220, 0.15, 12.0, -3.14159, 3.14159).passed
assert not check_scan([float("inf")] * 720, 0.15, 12.0, -3.14159, 3.14159).passed
assert not check_scan([1.0] * 720, 0.15, 12.0, 0.0, 3.14159).passed
# check_throttled
assert check_throttled(0x50000).passed
assert not check_throttled(0x50005).passed
# expected_wheel_velocities
assert expected_wheel_velocities(0.10, 0.0)["fl_wheel_joint"] == pytest.approx(2.3529, abs=1e-3)
turn = expected_wheel_velocities(0.0, 0.5)
assert turn["fr_wheel_joint"] == pytest.approx(1.5865, abs=1e-3)
assert turn["rl_wheel_joint"] == pytest.approx(-1.5865, abs=1e-3)
# check_wheel_response
forward = expected_wheel_velocities(0.10, 0.0)
assert check_wheel_response("forward", {j: 2.2 for j in WHEEL_JOINTS}, forward).passed
stalled = {**{j: 2.3 for j in WHEEL_JOINTS}, "rr_wheel_joint": 0.0}
assert "rr_wheel_joint" in check_wheel_response("forward", stalled, forward).detail
reversed_one = {**{j: 2.3 for j in WHEEL_JOINTS}, "fl_wheel_joint": -2.3}
assert not check_wheel_response("forward", reversed_one, forward).passed
with_nan = {**{j: 2.3 for j in WHEEL_JOINTS}, "rl_wheel_joint": float("nan")}
assert not check_wheel_response("forward", with_nan, forward).passed
# check_stop
assert check_stop({j: 0.02 for j in WHEEL_JOINTS}).passed
assert not check_stop({**{j: 0.0 for j in WHEEL_JOINTS}, "fl_wheel_joint": 0.6}).passed
# nearest_bearing_in_base: nearest return at scan angle pi, laser mounted at yaw pi
ranges = [5.0] * 360; ranges[180] = 0.5
assert nearest_bearing_in_base(ranges, 0.0, math.radians(1), math.pi) == pytest.approx(0.0, abs=0.02)
assert nearest_bearing_in_base([5.0] * 360, 0.0, math.radians(1), 0.0) is None
# check_depth_image
assert check_depth_image(0.6, 640, 480).passed
assert not check_depth_image(0.05, 640, 480).passed
```

- [ ] **Step 2: Run to verify failure**

Run: `./tests/run_unit_tests.sh /tests/test_hw_checks.py`
Expected: FAIL, `ModuleNotFoundError: No module named 'checks'`

- [ ] **Step 3: Implement `robot/hw_check/checks.py`** to the interface and rules above.

- [ ] **Step 4: Run to verify pass**

Run: `./tests/run_unit_tests.sh /tests/test_hw_checks.py`
Expected: all tests PASS. Then `./tests/run_unit_tests.sh`: the existing suites still pass.

- [ ] **Step 5: Commit**

```bash
git add robot/hw_check/checks.py tests/test_hw_checks.py tests/run_unit_tests.sh
git commit -m "feat(robot): add hardware check decision functions"
```

### Task 3: Passive sensor stage

**Files:**
- Create: `robot/hw_check/hw_check.py`, `robot/run_hw_check.sh`
- Test: `tests/test_hw_check_cli.py`

**Interfaces:**
- Consumes: every function of Task 2.
- Produces:
  - `parse_args(argv: list[str]) -> argparse.Namespace` with `--stage {sensors,motors,interactive,camera}` (required), `--wheels-off-ground` (flag), `--throttled` (int, base 0, default 0), `--report` (path, default `/out/<stage>.json`), `--window` (seconds, default 10.0).
  - `summarise(results: list[CheckResult]) -> tuple[str, int]`: a fixed-width table of name, PASS or FAIL and detail, and the exit code (0 only when every result passed).
  - `class Sampler(rclpy.node.Node)`: `watch(topic: str, msg_type) -> None`, `collect(seconds: float) -> None`, `times(topic) -> list[float]` (receive times from `time.monotonic()`), `last(topic)` (latest message or `None`).
  - `run_sensors(sampler: Sampler, args) -> list[CheckResult]`.
  - `robot/run_hw_check.sh <hw_check args>`: syncs `robot/hw_check/` to `~/hw_check` on the robot, runs the tool there, copies `~/hw_check_out/*.json` back to `robot/reports/`.

`run_sensors` checks, in this order: `check_throttled`; `check_rate` at 20 Hz, tolerance 0.10, for `/joint_states`, `/imu_broadcaster/imu`, `/odometry/filtered`, `/rosbot_base_controller/odom`; `check_rate` at 5 Hz, tolerance 0.20, for the four `/range/*`; `check_rate` at 10 Hz, tolerance 0.50, for `/scan` and `/scan_filtered`; `check_voltage`; `check_imu_stationary`; `check_joint_names`; `check_range_reading` for the four sensors; `check_scan`. A topic with no message gives the FAIL of `check_rate` and skips its content check.

The command `run_hw_check.sh` runs on the robot:

```bash
docker run --rm -it --network host --ipc host \
  -v ~/hw_check:/hw_check:ro -v ~/hw_check_out:/out \
  husarion/rosbot:humble-0.14.0-20250319 \
  python3 /hw_check/hw_check.py --throttled "$(vcgencmd get_throttled | cut -d= -f2)" "$@"
```

- [ ] **Step 1: Write the failing tests in `tests/test_hw_check_cli.py`**

```python
def test_stage_is_required():
    with pytest.raises(SystemExit):
        parse_args([])

def test_throttled_accepts_hex():
    assert parse_args(["--stage", "sensors", "--throttled", "0x50000"]).throttled == 0x50000

def test_summary_exit_code_is_zero_only_when_all_pass():
    ok = CheckResult("a", True, "")
    bad = CheckResult("b", False, "no messages")
    assert summarise([ok, ok])[1] == 0
    table, code = summarise([ok, bad])
    assert code == 1 and "no messages" in table and "FAIL" in table
```

- [ ] **Step 2: Run to verify failure**

Run: `./tests/run_unit_tests.sh /tests/test_hw_check_cli.py`
Expected: FAIL, `ModuleNotFoundError: No module named 'hw_check'`

- [ ] **Step 3: Implement `hw_check.py`** (`parse_args`, `summarise`, `Sampler`, `run_sensors`, `main`) and `run_hw_check.sh`. `main` prints the table, writes the results as JSON to `--report`, and exits with the code from `summarise`.

- [ ] **Step 4: Run the unit tests**

Run: `./tests/run_unit_tests.sh /tests/test_hw_check_cli.py`
Expected: PASS

- [ ] **Step 5: Run on the robot, stationary**

Run: `./robot/run_hw_check.sh --stage sensors`
Expected: every row PASS, exit code 0, and `robot/reports/sensors.json` exists.

- [ ] **Step 6: Prove the absent-topic path on the robot**

Run: `ssh husarion@192.168.77.2 "docker stop rplidar"`, then `./robot/run_hw_check.sh --stage sensors`.
Expected: the `/scan` and `/scan_filtered` rate rows read FAIL `no messages`, every other row still PASS, exit code 1. Then `ssh husarion@192.168.77.2 "docker start rplidar"`.

- [ ] **Step 7: Commit**

```bash
git add robot/hw_check/hw_check.py robot/run_hw_check.sh tests/test_hw_check_cli.py
git commit -m "feat(robot): add passive sensor stage of the hardware check"
```

### Task 4: Motor stage

**Files:**
- Modify: `robot/hw_check/hw_check.py`
- Test: `tests/test_hw_check_cli.py`

**Interfaces:**
- Consumes: `expected_wheel_velocities`, `check_wheel_response`, `check_stop`, `check_voltage`, `MAX_LINEAR`, `MAX_ANGULAR`.
- Produces:
  - `MOTOR_SEQUENCE: tuple[tuple[str, float, float, float], ...]` of (segment, linear, angular, seconds):
    `("forward", 0.10, 0.0, 3.0)`, `("backward", -0.10, 0.0, 3.0)`, `("turn_left", 0.0, 0.5, 3.0)`, `("turn_right", 0.0, -0.5, 3.0)`.
  - `PAUSE_SECONDS = 1.0` between segments, `STOP_WAIT_SECONDS = 1.0` before the stop check.
  - `run_motors(sampler: Sampler, args) -> list[CheckResult]`.

Behaviour of `run_motors`:
- Refuses unless `args.wheels_off_ground` is set, and then asks the operator to type `yes`.
- Publishes `geometry_msgs/Twist` on `/cmd_vel` at 20 Hz for each segment. The wheel velocity compared is the mean of `/joint_states` over the last 2 s of the segment.
- After the last segment it stops publishing and sends no zero command. After `STOP_WAIT_SECONDS` it runs `check_stop`. This is the test of `cmd_vel_timeout`.
- It adds `check_voltage` over every `/battery` sample received during the stage: the supply must not sag below 11.0 V under motor load.
- On any exception or Ctrl-C it publishes five zero commands, then re-raises.

- [ ] **Step 1: Add the failing tests**

```python
def test_motor_stage_refuses_without_the_wheels_flag():
    args = parse_args(["--stage", "motors"])
    with pytest.raises(SystemExit):
        require_wheels_off_ground(args)

def test_motor_sequence_is_bounded_and_inside_the_caps():
    total = sum(seconds + PAUSE_SECONDS for *_, seconds in MOTOR_SEQUENCE) + STOP_WAIT_SECONDS
    assert total <= 30.0
    for _, linear, angular, _ in MOTOR_SEQUENCE:
        assert abs(linear) <= MAX_LINEAR and abs(angular) <= MAX_ANGULAR
```

`require_wheels_off_ground(args) -> None` is the guard `run_motors` calls first; it raises `SystemExit` with a message naming the flag.

- [ ] **Step 2: Run to verify failure**

Run: `./tests/run_unit_tests.sh /tests/test_hw_check_cli.py`
Expected: FAIL, `ImportError` on `MOTOR_SEQUENCE`

- [ ] **Step 3: Implement** the constants, `require_wheels_off_ground` and `run_motors`.

- [ ] **Step 4: Run the unit tests**

Run: `./tests/run_unit_tests.sh /tests/test_hw_check_cli.py`
Expected: PASS

- [ ] **Step 5: Run on the robot. A person is at the robot and the four wheels are off the ground.**

Run: `./robot/run_hw_check.sh --stage motors --wheels-off-ground`
Expected: rows `forward`, `backward`, `turn_left`, `turn_right`, `stop`, `voltage` all PASS. All four wheels turn in each segment and stop by themselves.
- A side that turns with the opposite sign is a finding about the joint axes. Record it in Task 6; do not change the expected signs to make it pass.
- A voltage FAIL is the answer to the open power question: record the minimum voltage.

- [ ] **Step 6: Commit**

```bash
git add robot/hw_check/hw_check.py tests/test_hw_check_cli.py
git commit -m "feat(robot): add motor and stop-on-timeout stage"
```

### Task 5: Interactive stage and camera stage

**Files:**
- Modify: `robot/hw_check/hw_check.py`
- Test: `tests/test_hw_check_cli.py`

**Interfaces:**
- Consumes: `check_range_reading`, `nearest_bearing_in_base`, `check_depth_image`, `check_rate`.
- Produces:
  - `run_interactive(sampler: Sampler, args, ask=input) -> list[CheckResult]`.
  - `run_camera(sampler: Sampler, args) -> list[CheckResult]`.
  - `depth_valid_fraction(data: bytes, encoding: str) -> float`: share of non-zero pixels for `16UC1`, share of finite non-zero pixels for `32FC1`.
  - `HAND_DISTANCE = 0.20`, `PROMPT_TIMEOUT = 15.0`, `FRONT_BEARING_LIMIT = math.radians(20)`.

`run_interactive` steps, each one a row:
1. For each of `fl`, `fr`, `rl`, `rr`: prompt to hold a hand about 10 cm in front of that sensor. PASS when its reading drops below `HAND_DISTANCE` within `PROMPT_TIMEOUT`.
2. For `left` and `right`: prompt to press that button. PASS when `/button/<side>` delivers `true` within `PROMPT_TIMEOUT`.
3. For `left` and `right`: publish `true` on `/led/<side>`, ask "is the LED on? [y/n]", publish `false`. PASS on `y`.
4. Lidar front: prompt to place a box 0.5 m in front of the robot with nothing else within 1 m. Read `laser_yaw` from the `base_link` to `laser` transform. PASS when `abs(nearest_bearing_in_base(...))` is below `FRONT_BEARING_LIMIT`. The detail states the bearing in degrees.

`run_camera` needs the `astra` container. Topics expected from the Husarion image with `device_namespace:=camera`: `/camera/color/image_raw` and `/camera/depth/image_raw`. Step 4 confirms the names on the robot before they are fixed as constants. Rows: `check_rate` at 30 Hz with tolerance 0.50 for both; `check_depth_image` on the latest depth frame.

**Camera as found on the robot (2026-11-14, read-only inspection over SSH, camera not started):**

- Device: Orbbec Astra, USB id `2bc5:0401`, product string "ORBBEC Depth Sensor", USB 2.0, bus powered, up to 500 mA. It enumerates on bus 3 (`dwc2`) as `/dev/bus/usb/003/002`.
- Interfaces: one vendor-specific interface with no kernel driver, plus audio interfaces bound to `snd-usb-audio` (the microphones). There is no UVC interface, so the camera has no `/dev/video*` node. Every `/dev/video*` on the robot belongs to the Raspberry Pi codec and ISP (`bcm2835-codec`, `bcm2835-isp`). `usb_cam` and `v4l2_camera` cannot drive it; the driver in the `astra` image talks to it through libusb, which is why the service maps `/dev/bus/usb/`.
- Not verified: that colour arrives over the same vendor-specific interface as depth. Step 4 settles it; if no colour topic appears, drop the colour rate row and say so in Task 6.
- Permissions: no udev rule on the host matches vendor `2bc5`, and the device node is `root:root` mode `0664`. The container must run as root to open it. Not verified for `husarion/astra:humble-1.1.0-20240319`; an "access denied" or "no device found" in `docker logs astra` points here first.
- Image: `husarion/astra:humble-1.1.0-20240319` is not on the robot (only `husarion/rosbot` is). The tag has an `arm64` variant. `deploy.sh` pulls on the robot, so it needs the internet step of Task 1. Without internet on the robot, load the image from `atlas`:
  `docker pull --platform linux/arm64 husarion/astra:humble-1.1.0-20240319 && docker save husarion/astra:humble-1.1.0-20240319 | ssh husarion@192.168.77.2 docker load`,
  then start it with `ssh husarion@192.168.77.2 "docker compose --profile camera up -d astra"`.
- Load before the camera starts: `ekf_node` in the `rosbot` container already logs `Failed to meet update rate!`, with cycles up to 0.29 s. Read `docker logs --tail 20 rosbot` again with the camera running and record in Task 6 whether the 20 Hz rows of the sensor stage still pass.
- Power: the camera adds up to 500 mA to a robot that already recorded an under-voltage at boot. Read `vcgencmd get_throttled` after the camera stage; any value other than `0x50000` is a finding for Task 6.
- Outside this check, for the student machines: uncompressed 640 by 480 at 30 Hz is about 27.6 MB/s for colour (`rgb8`) and 18.4 MB/s for depth (`16UC1`). The camera stage is not affected, because the runner executes on the robot.

- [ ] **Step 1: Add the failing tests**

```python
def test_depth_valid_fraction_counts_non_zero_16uc1_pixels():
    data = struct.pack("<4H", 0, 800, 0, 1200)
    assert depth_valid_fraction(data, "16UC1") == pytest.approx(0.5)

def test_depth_valid_fraction_ignores_nan_in_32fc1():
    data = struct.pack("<4f", float("nan"), 0.0, 1.5, 2.0)
    assert depth_valid_fraction(data, "32FC1") == pytest.approx(0.5)

def test_led_row_fails_when_the_operator_answers_no():
    row = led_result("left", answer="n")
    assert not row.passed and row.name == "led/left"
```

`led_result(side: str, answer: str) -> CheckResult` is the pure part of step 3.

- [ ] **Step 2: Run to verify failure**

Run: `./tests/run_unit_tests.sh /tests/test_hw_check_cli.py`
Expected: FAIL, `ImportError` on `depth_valid_fraction`

- [ ] **Step 3: Implement** `depth_valid_fraction`, `led_result`, `run_interactive`, `run_camera`.

- [ ] **Step 4: Start the camera and confirm its topic names**

Run: `./robot/deploy.sh --profile camera` (or the offline image load described above when the robot has no internet), then
`ssh husarion@192.168.77.2 "docker exec rosbot bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list -t | grep camera'"`
Expected: a colour image topic and a depth image topic of type `sensor_msgs/msg/Image`. If the names differ from the two above, set the constants to what is listed.

- [ ] **Step 5: Run both stages on the robot, with a person at the robot**

Run: `./tests/run_unit_tests.sh /tests/test_hw_check_cli.py`, then
`./robot/run_hw_check.sh --stage interactive` and `./robot/run_hw_check.sh --stage camera`.
Expected: unit tests PASS; every interactive row PASS; both camera rates and the depth row PASS.
Afterwards stop the camera: `ssh husarion@192.168.77.2 "docker stop astra"`.

- [ ] **Step 6: Commit**

```bash
git add robot/hw_check/hw_check.py tests/test_hw_check_cli.py
git commit -m "feat(robot): add interactive and camera stages"
```

### Task 6: Record the evidence

**Files:**
- Create: `docs/robot_verification.md`
- Modify: `README.md` (one link to the new page, in the documentation list)

**Interfaces:**
- Consumes: `robot/reports/sensors.json`, `motors.json`, `interactive.json`, `camera.json`.

- [ ] **Step 1: Write `docs/robot_verification.md`**, in the form of `docs/verification.md`: date, image tags, firmware version, one table per stage with a row per check, result and measured value. Add three short sections:
  - "How to re-run": the four `run_hw_check.sh` commands and the two physical preconditions.
  - "Findings": lidar model and baud rate, minimum voltage under motor load, any wheel with a reversed sign, the bearing the lidar reports for an object in front, the `get_throttled` value before and after.
  - "Topic names against the migration spec": the three differing names listed in "Measured baseline".

- [ ] **Step 2: Check the page against the reports**

Run: `jq -r '.[] | "\(.name) \(.passed)"' robot/reports/*.json | sort | uniq -c | sort -rn | head`
Expected: every check name in the reports appears in the page with the same result.

- [ ] **Step 3: Commit**

```bash
git add docs/robot_verification.md README.md
git commit -m "docs(robot): record ROSbot 2R hardware check evidence"
```

---

## Out of scope

- Driving on the floor and the "advance 2 m, turn 90°" controller: next plan, built on the result of Task 4.
- Persistent internet, Wi-Fi, time synchronisation, password change and the card image.
- A node that stops the robot from the range sensors.
- Servo outputs (`/cmd_ser`): no servo is fitted.
