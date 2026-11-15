# Motion primitives

Tell the robot to go forward a distance or to turn an angle, and it does that
and stops. The node reads the robot's pose from odometry and publishes velocity
commands until the pose says the move is complete.

## Use it

With the simulation running (see [rosbot_simulation.md](rosbot_simulation.md)),
from a second terminal in the container:

```bash
python3 /ros2_ws/src/motion_primitives.py forward 2.0
python3 /ros2_ws/src/motion_primitives.py turn 90
python3 /ros2_ws/src/motion_primitives.py forward 1 turn 90 forward 1 turn 90 forward 1 turn 90 forward 1 turn 90
```

- `forward <metres>`: straight along the heading the robot has when the move
  starts. A negative distance drives backward.
- `turn <degrees>`: on the spot. Positive is to the left. More than 180 turns
  the long way round, as asked.
- Several pairs on one line run one after the other. The last line above is a
  1 m square.

Each finished move prints what the odometry reports and how long it took:

```
forward +2.000 m: odometry reports +1.999 m, 11.4 s
turn +90.0 deg: odometry reports +90.0 deg, 4.9 s
```

## Parameters

Set after `--ros-args`, for example `--ros-args -p max_linear:=0.1`.

| Parameter | Default | Meaning |
|---|---|---|
| `max_linear` | 0.20 m/s | cruise speed; refused above the robot's 0.9 |
| `max_angular` | 0.50 rad/s | turn rate; refused above the robot's 3.0 |
| `feedback_topic` | `/odometry/filtered` | `nav_msgs/Odometry` giving the pose |
| `cmd_vel_topic` | `/cmd_vel` | `geometry_msgs/Twist` commands |

A move ends when the robot is within 1 cm or 1° of the target and at rest.

## How it moves

The speed follows a trapezoid: it ramps up, cruises at the maximum, and brakes
as the remaining distance shrinks. Close to the target the speed is
proportional to what is left, which is what stops a real base, slower to obey
than an ideal one, from overshooting. While driving straight the node also
steers back toward the starting heading.

## How it stops, and when it gives up

| Exit code | Meaning |
|---|---|
| 0 | every move done |
| 2 | bad input: unknown word, missing or zero amount, more than 10 m or 720° in one move. Nothing was sent to the robot |
| 3 | no odometry within 5 s of starting: is the robot or the simulation running? |
| 4 | a move took more than three times its expected duration plus 5 s: the robot is probably blocked |
| 5 | odometry stopped arriving for 0.5 s during a move |
| 130 | interrupted with Ctrl-C |

On every way out the node sends zero commands. The robot does not depend on
them: its own drive controller stops 0.5 s after the last command it receives.

All timing uses the clock of the computer the node runs on, never the stamps in
the messages. A robot whose clock is hours off, as the lab robot's is after a
power cut, makes no difference.

## Measured in simulation

`tests/run_motion_primitives_sim_test.sh`, ROSbot 2R in Gazebo Fortress, empty
world, default speeds, 2026-11-15. "Real" is Gazebo's own pose of the robot,
which the node never sees.

| Move | Odometry | Real |
|---|---|---|
| `forward 1.0` | 0.9991 m | 0.9993 m |
| `turn 90` | 89.77° | 89.76° |
| `forward -0.5` | -0.4994 m | -0.4993 m |

The 1 m square from "Use it", eight moves in a row, brought the robot back to
within 4 mm and 0.1° of where it started, measured on Gazebo's pose.

Odometry and real motion agree to a fraction of a millimetre here because the
simulated wheels barely slip. On a real floor they will not agree this well.

## On the real robot

Not run there yet. The defaults already match the robot's topics. Before the
first run:

- ROS 2 discovery between this computer and the robot checked on purpose: host
  networking, `ROS_DOMAIN_ID` 0, and no simulation running anywhere on that
  network.
- The robot's stack started with `~/stack.sh up`.
- Wheels off the ground for the first run, then a clear floor.

Expect the real distance to differ from what odometry reports, more so in
turns: a skid-steer base slips sideways to turn. Measuring that difference with
a tape is the next step, not part of this node.
