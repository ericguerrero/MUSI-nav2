"""Pass/fail decisions for the ROSbot 2R hardware check.

Plain Python on plain numbers, with no ROS import, so every threshold can be
unit-tested away from the robot. The runner in hw_check.py collects the
samples and calls these.

A value that is nan never raises: it fails the check it belongs to. The robot
already publishes nan (the effort field of /joint_states), so a bad sample is
an expected input, not a programming error. The one exception is a range
sensor, where nan is how the firmware reports "nothing in range".
"""

import math
from dataclasses import dataclass

WHEEL_JOINTS = ("fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint")
LEFT_JOINTS = ("fl_wheel_joint", "rl_wheel_joint")

# From rosbot_controller's diff_drive_controller.yaml in driver 0.14.0. The
# separation carries the controller's 1.45 multiplier, which models skid-steer
# slip, because that is the geometry the controller converts cmd_vel with.
WHEEL_RADIUS = 0.0425
WHEEL_SEPARATION = 0.186 * 1.45

# Hard caps for the motor stage, far below the controller's 0.9 m/s and 3 rad/s.
MAX_LINEAR = 0.15
MAX_ANGULAR = 0.6

GRAVITY = 9.81
# Bits 0 to 3 of vcgencmd get_throttled describe the present moment; the bits
# from 16 up only remember that something happened since boot.
THROTTLED_NOW_MASK = 0xF
NO_MESSAGES = "no messages"


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: str


def _norm(values) -> float:
    return math.sqrt(sum(v * v for v in values))


def check_rate(name: str, receive_times: list, expected_hz: float,
               tolerance: float) -> CheckResult:
    """Rate from receive times, which must come from the receiver's own clock."""
    if len(receive_times) < 2:
        return CheckResult(name, False, NO_MESSAGES)
    span = receive_times[-1] - receive_times[0]
    if not span > 0:
        return CheckResult(name, False, "receive times do not advance")
    rate = (len(receive_times) - 1) / span
    passed = abs(rate - expected_hz) <= tolerance * expected_hz
    return CheckResult(name, passed, f"{rate:.2f} Hz, expected {expected_hz:g}")


def check_voltage(volts: list, minimum: float = 11.0,
                  maximum: float = 13.0) -> CheckResult:
    if not volts:
        return CheckResult("voltage", False, NO_MESSAGES)
    if any(math.isnan(v) for v in volts):
        return CheckResult("voltage", False, "nan in the samples")
    lowest, highest = min(volts), max(volts)
    passed = minimum <= lowest and highest <= maximum
    return CheckResult("voltage", passed,
                       f"min {lowest:.2f} V, max {highest:.2f} V over {len(volts)} samples")


def check_imu_stationary(orientation: tuple, accel: tuple, gyro: tuple) -> CheckResult:
    quaternion_norm, accel_norm, gyro_norm = _norm(orientation), _norm(accel), _norm(gyro)
    # Comparisons are written so that nan makes them false.
    passed = (abs(quaternion_norm - 1.0) < 0.01
              and abs(accel_norm - GRAVITY) < 0.5
              and gyro_norm < 0.05)
    return CheckResult(
        "imu stationary", passed,
        f"|q| {quaternion_norm:.3f}, |a| {accel_norm:.2f} m/s2, |w| {gyro_norm:.4f} rad/s")


def check_joint_names(names: list) -> CheckResult:
    missing = sorted(set(WHEEL_JOINTS) - set(names))
    detail = "four wheel joints" if not missing else "missing " + ", ".join(missing)
    return CheckResult("joint names", not missing, detail)


def check_range_reading(sensor: str, value: float, min_range: float,
                        max_range: float) -> CheckResult:
    name = f"range/{sensor}"
    if math.isnan(value):
        # Firmware 0.11.0 publishes nan for every reading outside
        # [min_range, max_range], and a sensor that never initialised reads the
        # same. Only the interactive stage, with a hand in front of the sensor,
        # can tell the two apart.
        return CheckResult(name, True, "nan: no target in range, or sensor inactive")
    if value < 0:
        return CheckResult(name, False, f"invalid reading {value}")
    if math.isinf(value) or value > max_range:
        return CheckResult(name, True, "no target in range")
    return CheckResult(name, True, f"{value:.3f} m")


def check_scan(ranges: list, range_min: float, range_max: float,
               angle_min: float, angle_max: float) -> CheckResult:
    if not ranges:
        return CheckResult("scan", False, "empty scan")
    valid = sum(1 for r in ranges if math.isfinite(r) and range_min <= r <= range_max)
    fraction = valid / len(ranges)
    span = angle_max - angle_min
    passed = fraction >= 0.30 and span >= 6.0
    return CheckResult(
        "scan", passed,
        f"{len(ranges)} points, {fraction:.0%} valid, span {span:.2f} rad")


def check_throttled(value: int) -> CheckResult:
    passed = value & THROTTLED_NOW_MASK == 0
    history = "under-voltage or throttling since boot" if value >> 16 else "no history"
    return CheckResult("pi throttled", passed, f"{value:#x}, {history}")


def expected_wheel_velocities(linear: float, angular: float) -> dict:
    """Wheel angular velocities, rad/s, the diff-drive controller commands."""
    offset = angular * WHEEL_SEPARATION / 2
    left = (linear - offset) / WHEEL_RADIUS
    right = (linear + offset) / WHEEL_RADIUS
    return {joint: left if joint in LEFT_JOINTS else right for joint in WHEEL_JOINTS}


def _wheel_fault(measured, expected: float, tolerance: float):
    """Why one wheel fails, or None when it follows the command."""
    if measured is None:
        return "not measured"
    if math.isnan(measured):
        return "nan"
    if measured * expected <= 0:
        return f"{measured:+.2f} rad/s, wrong direction (expected {expected:+.2f})"
    if abs(measured - expected) > tolerance * abs(expected):
        return f"{measured:+.2f} rad/s, expected {expected:+.2f}"
    return None


def check_wheel_response(segment: str, measured: dict, expected: dict,
                         tolerance: float = 0.30) -> CheckResult:
    faults = []
    for joint in WHEEL_JOINTS:
        fault = _wheel_fault(measured.get(joint), expected[joint], tolerance)
        if fault:
            faults.append(f"{joint}: {fault}")
    if faults:
        return CheckResult(segment, False, "; ".join(faults))
    mean = sum(abs(measured[j]) for j in WHEEL_JOINTS) / len(WHEEL_JOINTS)
    return CheckResult(segment, True, f"all four wheels follow, mean {mean:.2f} rad/s")


def check_stop(velocities: dict, limit: float = 0.1) -> CheckResult:
    if not velocities:
        return CheckResult("stop", False, NO_MESSAGES)
    moving = [f"{joint}: {v:+.2f} rad/s" for joint, v in sorted(velocities.items())
              if not abs(v) < limit]
    detail = "; ".join(moving) if moving else "all wheels at rest"
    return CheckResult("stop", not moving, detail)


def nearest_bearing_in_base(ranges: list, angle_min: float, angle_increment: float,
                            laser_yaw: float, max_distance: float = 1.0):
    """Bearing, in the robot's frame, of the nearest return within max_distance."""
    nearest_index, nearest_range = None, max_distance
    for index, value in enumerate(ranges):
        if math.isfinite(value) and 0 < value < nearest_range:
            nearest_index, nearest_range = index, value
    if nearest_index is None:
        return None
    angle = angle_min + nearest_index * angle_increment + laser_yaw
    return math.atan2(math.sin(angle), math.cos(angle))


def check_depth_image(valid_fraction: float, width: int, height: int) -> CheckResult:
    passed = (width, height) == (640, 480) and valid_fraction >= 0.20
    return CheckResult("depth image", passed,
                       f"{width}x{height}, {valid_fraction:.0%} of pixels with depth")
