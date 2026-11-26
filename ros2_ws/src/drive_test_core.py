"""Pure logic of the drive comparison test: the manoeuvres, the per-robot
presets, the geometry that turns two poses into table numbers, and wheel
odometry from encoder angles. No ROS imports, so it is unit-tested directly."""

import math
import re
from dataclasses import dataclass


@dataclass(frozen=True)
class Manoeuvre:
    name: str
    vx: float        # m/s, forward
    vy: float        # m/s, to the left
    wz: float        # rad/s, counter-clockwise
    duration: float  # s of simulation time


# Speeds stay inside the slowest robot's limits (TurtleBot3 Waffle: 0.26 m/s,
# 1.82 rad/s) so every robot receives a command it is allowed to follow and the
# rows are comparable. Heading changes stay below 180 degrees because the true
# heading is read only before and after a manoeuvre.
MANOEUVRES = (
    Manoeuvre("straight", 0.2, 0.0, 0.0, 5.0),
    Manoeuvre("spin", 0.0, 0.0, 1.0, 2.0),
    Manoeuvre("arc", 0.2, 0.0, 0.5, 4.0),
    Manoeuvre("sideways", 0.0, 0.2, 0.0, 5.0),
)

# One entry per robot. "truth" names where the simulator's own pose is read:
#   "model_states"  Gazebo Classic, topic /model_states (libgazebo_ros_state.so)
#   "ign_model"     Gazebo Fortress, the command `ign model -m <model> -p`
#   "odometry"      a nav_msgs/Odometry topic carrying the simulator's pose;
#                   "model" then holds the topic name
# "odom" is a nav_msgs/Odometry topic computed from the wheels only. When
# "wheels" is set instead, odometry is integrated here from /joint_states:
# (left joint, right joint, wheel radius m, wheel separation m).
PRESETS = {
    # The TurtleBot3 Gazebo plugin fills /odom from the simulator's pose, not
    # from the wheels, so it cannot show odometry error. Radius and separation
    # are the values in its model.sdf.
    "turtlebot3": {
        "cmd_vel": "/cmd_vel",
        "truth": "model_states",
        "model": "turtlebot3_waffle",
        "odom": None,
        "wheels": ("wheel_left_joint", "wheel_right_joint", 0.033, 0.287),
    },
    "rosbot": {
        "cmd_vel": "/cmd_vel",
        "truth": "ign_model",
        "model": "rosbot",
        "odom": "/rosbot_base_controller/odom",
        "wheels": None,
    },
    "rosbot_mecanum": {
        "cmd_vel": "/cmd_vel",
        "truth": "ign_model",
        "model": "rosbot",
        "odom": "/rosbot_base_controller/odom",
        "wheels": None,
    },
    # Car-like steering (ackermann_simulation.launch.py). It cannot spin on the
    # spot or move sideways, and it turns no tighter than a 0.50 m radius, so
    # the arc asks for more than it can do. Those rows are results, not errors.
    "ackermann": {
        "cmd_vel": "/cmd_vel",
        "truth": "odometry",
        "model": "/groundtruth/odom",
        "odom": "/ackermann_steering_controller/odometry",
        "wheels": None,
    },
}


class DriveTestError(Exception):
    """A problem the student can act on; the message is printed as is."""


def preset_for(robot: str) -> dict:
    if robot not in PRESETS:
        raise DriveTestError(
            f"unknown robot '{robot}'. Choose one of: {', '.join(sorted(PRESETS))}")
    return dict(PRESETS[robot])


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def ideal_motion(manoeuvre: Manoeuvre) -> tuple:
    """(forward m, left m, turn rad) of a body that follows the command exactly,
    in the frame it started in."""
    vx, vy, wz, t = manoeuvre.vx, manoeuvre.vy, manoeuvre.wz, manoeuvre.duration
    turn = wz * t
    if abs(wz) < 1e-9:
        return vx * t, vy * t, 0.0
    sin, cos = math.sin(turn), math.cos(turn)
    return ((vx * sin - vy * (1.0 - cos)) / wz,
            (vx * (1.0 - cos) + vy * sin) / wz,
            turn)


def relative_motion(start: tuple, end: tuple) -> tuple:
    """(forward m, left m, turn rad) from pose start to pose end, each (x, y,
    yaw), in the frame of start. The turn is the short way round, so a real
    turn beyond 180 degrees would be misread."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    cos, sin = math.cos(start[2]), math.sin(start[2])
    return (dx * cos + dy * sin,
            -dx * sin + dy * cos,
            normalize_angle(end[2] - start[2]))


_NUMBERS = r"\[\s*(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s*\]"
_IGN_POSE = re.compile(r"Pose.*?" + _NUMBERS + r"\s*" + _NUMBERS, re.DOTALL)


def parse_ign_model_pose(text: str):
    """(x, y, yaw) from the output of `ign model -m <model> -p`, or None when
    the model is not there. Only the first pose is the model's; link poses
    follow it when other flags are given."""
    match = _IGN_POSE.search(text)
    if match is None:
        return None
    x, y, _, _, _, yaw = (float(value) for value in match.groups())
    return x, y, yaw


class DiffDriveOdometry:
    """Dead reckoning from the two wheel angles of a differential drive, which
    is what the robot's encoders give. It assumes the wheels roll without
    slipping; where they slip, this pose and the true pose part ways."""

    def __init__(self, wheel_radius: float, wheel_separation: float):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.pose = None
        self._angles = None

    def update(self, left_angle: float, right_angle: float) -> tuple:
        if self._angles is None:
            self._angles = (left_angle, right_angle)
            self.pose = (0.0, 0.0, 0.0)
            return self.pose
        left = (left_angle - self._angles[0]) * self.wheel_radius
        right = (right_angle - self._angles[1]) * self.wheel_radius
        self._angles = (left_angle, right_angle)
        distance = (left + right) / 2.0
        turn = (right - left) / self.wheel_separation
        x, y, yaw = self.pose
        # Heading at the middle of the step: exact for a circular arc to
        # second order, where the heading at its start is only first order.
        x += distance * math.cos(yaw + turn / 2.0)
        y += distance * math.sin(yaw + turn / 2.0)
        self.pose = (x, y, yaw + turn)
        return self.pose


class SimClock:
    """Simulation time between /clock messages.

    Gazebo Classic publishes /clock at 10 Hz. Timing a 2 s manoeuvre on that
    alone would be off by up to 5 %, so the time since the last message is
    added, capped at one message period so a paused simulation stands still."""

    MAX_EXTRAPOLATION_S = 0.1

    def __init__(self):
        self._sim = None
        self.updated_at = None  # wall-clock time of the last /clock message

    def update(self, sim_seconds: float, wall_seconds: float) -> None:
        self._sim, self.updated_at = sim_seconds, wall_seconds

    def now(self, wall_seconds: float):
        if self._sim is None:
            return None
        ahead = min(max(wall_seconds - self.updated_at, 0.0), self.MAX_EXTRAPOLATION_S)
        return self._sim + ahead


def describe_command(manoeuvre: Manoeuvre) -> str:
    parts = []
    if manoeuvre.vx:
        parts.append(f"x {manoeuvre.vx:g} m/s")
    if manoeuvre.vy:
        parts.append(f"y {manoeuvre.vy:g} m/s")
    if manoeuvre.wz:
        parts.append(f"z {manoeuvre.wz:g} rad/s")
    return f"{', '.join(parts) or 'stop'} for {manoeuvre.duration:g} s"


def _triple(motion: tuple) -> str:
    forward, left, turn = motion
    # Adding 0.0 turns -0.0 into 0.0: "-0.000" reads as a measurement.
    return (f"{round(forward, 3) + 0.0:7.3f} {round(left, 3) + 0.0:7.3f} "
            f"{round(math.degrees(turn), 1) + 0.0:7.1f}")


def table_header() -> str:
    columns = f"{'fwd m':>7} {'left m':>7} {'turn °':>7}"
    return "\n".join((
        f"{'':<10}{'':<30} | {'commanded':^23} | {'true (Gazebo)':^23} | {'wheel odometry':^23}",
        f"{'manoeuvre':<10}{'command':<30} | {columns} | {columns} | {columns}",
    ))


def table_row(manoeuvre: Manoeuvre, true: tuple, odometry: tuple) -> str:
    return (f"{manoeuvre.name:<10}{describe_command(manoeuvre):<30} | "
            f"{_triple(ideal_motion(manoeuvre))} | {_triple(true)} | {_triple(odometry)}")
