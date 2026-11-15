"""Core of the motion primitives: "go forward 2 m", "turn 90 degrees".

Plain Python with no ROS import, so the control law is tested against a
simulated robot in milliseconds. motion_primitives.py is the node that feeds
it odometry and publishes its commands.

A primitive is closed on the pose the robot reports, not on time: the robot
stops when odometry says it has covered the distance, whatever the floor or
the battery did to its speed on the way.
"""

import math
from dataclasses import dataclass

# What the ROSbot 2R's own drive controller accepts (rosbot_controller,
# driver 0.14.0). Limits above these would be clipped by the robot anyway.
MAX_LINEAR_LIMIT = 0.9
MAX_ANGULAR_LIMIT = 3.0

# Bounds per primitive: far beyond any lab, and they turn a typing slip such
# as "forward 200" into an error instead of a robot that leaves the room.
MAX_DISTANCE = 10.0
MAX_TURN = math.radians(720.0)

DISTANCE_TOLERANCE = 0.01
ANGLE_TOLERANCE = math.radians(1.0)
# Inside the tolerance for this many updates in a row before a primitive is
# done, so that a robot still coasting through the target does not count.
SETTLED_TICKS = 5

# A real base does not move below some command (friction, motor deadband).
# Outside the tolerance the command never drops under these.
MIN_LINEAR = 0.02
MIN_ANGULAR = 0.10

# Turn rate per radian of heading error while driving straight.
HEADING_GAIN = 1.5

KINDS = ("forward", "turn")


class PrimitiveError(ValueError):
    """A primitive or a limit the robot cannot be asked to carry out."""


@dataclass(frozen=True)
class Primitive:
    kind: str       # "forward" or "turn"
    amount: float   # metres, or radians with positive meaning left


@dataclass(frozen=True)
class Limits:
    max_linear: float = 0.20     # m/s
    max_angular: float = 0.50    # rad/s
    linear_accel: float = 0.30   # m/s^2
    angular_accel: float = 1.00  # rad/s^2

    def __post_init__(self):
        bounds = (("max_linear", self.max_linear, MAX_LINEAR_LIMIT),
                  ("max_angular", self.max_angular, MAX_ANGULAR_LIMIT),
                  ("linear_accel", self.linear_accel, math.inf),
                  ("angular_accel", self.angular_accel, math.inf))
        for name, value, ceiling in bounds:
            if not 0 < value <= ceiling:
                raise PrimitiveError(
                    f"{name} must be above 0 and at most {ceiling}, got {value}")


def normalize_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _amount(kind: str, text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise PrimitiveError(f"'{text}' is not a number") from None
    if not math.isfinite(value) or value == 0:
        raise PrimitiveError(f"{kind} needs a finite amount other than zero, got '{text}'")
    if kind == "turn":
        value = math.radians(value)
    ceiling = MAX_TURN if kind == "turn" else MAX_DISTANCE
    if abs(value) > ceiling:
        unit = "720 degrees" if kind == "turn" else f"{MAX_DISTANCE:g} m"
        raise PrimitiveError(f"{kind} {text} is beyond the limit of {unit}")
    return value


def parse_primitives(words: list) -> list:
    """Read "forward <metres>" and "turn <degrees>" pairs."""
    if not words:
        raise PrimitiveError("nothing to do")
    if len(words) % 2:
        raise PrimitiveError(f"'{words[-1]}' has no amount")
    primitives = []
    for kind, text in zip(words[0::2], words[1::2]):
        if kind not in KINDS:
            raise PrimitiveError(f"unknown primitive '{kind}'")
        primitives.append(Primitive(kind, _amount(kind, text)))
    return primitives


def speed_toward(remaining: float, previous: float, max_speed: float,
                 accel: float, dt: float) -> float:
    """Speed for a trapezoidal profile: ramp up, cruise, brake to the target.

    The braking curve is the speed from which a constant deceleration stops
    exactly at the target, so it needs no separate "start braking" decision.
    """
    magnitude = min(max_speed, math.sqrt(2 * accel * abs(remaining)))
    magnitude = min(magnitude, abs(previous) + accel * dt)
    return math.copysign(magnitude, remaining)


def expected_duration(primitive: Primitive, limits: Limits) -> float:
    """Roughly how long a primitive takes: cruise time plus one ramp."""
    if primitive.kind == "turn":
        speed, accel = limits.max_angular, limits.angular_accel
    else:
        speed, accel = limits.max_linear, limits.linear_accel
    return abs(primitive.amount) / speed + speed / accel


class PrimitiveRunner:
    """Turns successive poses into velocity commands for one primitive."""

    def __init__(self, primitive: Primitive, limits: Limits) -> None:
        self._primitive = primitive
        self._limits = limits
        self._start = None
        self._previous_yaw = None
        self._turned = 0.0
        self._remaining = primitive.amount
        self._command = 0.0
        self._settled = 0

    @property
    def remaining(self) -> float:
        """Metres or radians still to go, signed like the primitive."""
        return self._remaining

    @property
    def done(self) -> bool:
        return self._settled >= SETTLED_TICKS

    def update(self, pose: tuple, dt: float) -> tuple:
        """Return (linear, angular) for the pose (x, y, yaw) seen now."""
        if self._start is None:
            self._start = pose
            self._previous_yaw = pose[2]
        if self._primitive.kind == "turn":
            return self._turn(pose, dt)
        return self._forward(pose, dt)

    def _settle(self, tolerance: float) -> bool:
        """Count updates inside the tolerance; True once the primitive is done."""
        inside = abs(self._remaining) < tolerance
        self._settled = self._settled + 1 if inside else 0
        return self.done

    def _profile(self, max_speed: float, accel: float, floor: float,
                 tolerance: float, dt: float) -> float:
        speed = speed_toward(self._remaining, self._command, max_speed, accel, dt)
        if abs(self._remaining) >= tolerance and abs(speed) < floor:
            speed = math.copysign(floor, self._remaining)
        self._command = speed
        return speed

    def _forward(self, pose: tuple, dt: float) -> tuple:
        x0, y0, yaw0 = self._start
        # Progress along the heading the robot started with: drifting sideways
        # does not count as distance covered.
        progress = (pose[0] - x0) * math.cos(yaw0) + (pose[1] - y0) * math.sin(yaw0)
        self._remaining = self._primitive.amount - progress
        if self._settle(DISTANCE_TOLERANCE):
            return 0.0, 0.0
        limits = self._limits
        linear = self._profile(limits.max_linear, limits.linear_accel, MIN_LINEAR,
                               DISTANCE_TOLERANCE, dt)
        angular = HEADING_GAIN * normalize_angle(yaw0 - pose[2])
        angular = max(-limits.max_angular, min(limits.max_angular, angular))
        return linear, angular

    def _turn(self, pose: tuple, dt: float) -> tuple:
        # Summing small wrapped steps gives the angle actually turned, so a
        # 270 degree turn is not mistaken for 90 degrees the other way.
        self._turned += normalize_angle(pose[2] - self._previous_yaw)
        self._previous_yaw = pose[2]
        self._remaining = self._primitive.amount - self._turned
        if self._settle(ANGLE_TOLERANCE):
            return 0.0, 0.0
        limits = self._limits
        angular = self._profile(limits.max_angular, limits.angular_accel, MIN_ANGULAR,
                                ANGLE_TOLERANCE, dt)
        return 0.0, angular
