# Ackermann robot simulation

Gazebo Fortress with a small car-like robot: the rear wheels drive, the front
wheels steer. It takes the same `/cmd_vel` as the TurtleBot3 and the ROSbot,
but it cannot rotate on the spot and cannot turn tighter than a 0.50 m radius.
It has no sensors. It exists to be compared with the other drives.

It runs on the image as built: nothing to install or rebuild.

## Run it

Inside the container (`./run.sh`, then `docker exec -it nav2_workspace bash`):

```bash
ros2 launch /ros2_ws/src/ackermann_simulation.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `headless` | `false` | `true` runs Gazebo without its window |
| `x`, `y`, `yaw` | `0.0`, `0.0`, `0.0` | spawn pose, metres and radians |

The world is Gazebo's empty flat world. Drive from a second terminal:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
```

The robot stops 0.5 s after the last command.

The Gazebo terminal prints `Use of Twist message without stamped is deprecated`
for every command received. It is harmless: the Humble controller prefers
`TwistStamped`, and this robot is configured to take the plain `Twist` every
other robot of the course takes.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | in; `linear.x` and `angular.z` are used |
| `/ackermann_steering_controller/odometry` | `nav_msgs/msg/Odometry` | out, 50 Hz, wheel odometry, frame `odom` |
| `/groundtruth/odom` | `nav_msgs/msg/Odometry` | out, 50 Hz, Gazebo's own pose, frame `world` |
| `/joint_states` | `sensor_msgs/msg/JointState` | out; rear wheels and both steering angles |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | out; robot links only, no `odom` frame |

Both odometry topics describe `base_link`, the point on the ground under the
middle of the rear axle. Wheel odometry starts at zero wherever the robot is
spawned; `/groundtruth/odom` is the pose in the world. Wheel odometry is
computed from the measured rear wheel speed and steering angle.

## Geometry

| | |
|---|---|
| Wheelbase L | 0.26 m |
| Track | 0.20 m |
| Wheel radius | 0.04 m |
| Minimum turn radius of `base_link` | 0.50 m |
| Full lock, inner / outer front wheel | 0.576 rad (33.0°) / 0.409 rad (23.4°) |
| Wheel speed limit | 1.3 m/s at the rim |
| Mass | 2.7 kg |

A `Twist` asks for a speed v and a yaw rate ω, that is a turn of radius v / ω.
The controller turns the front wheels to the angle that gives it,
δ = atan(L · ω / v) for the middle of the front axle, and a little more on the
inner wheel than on the outer one so both point at the same centre. The
steering joints stop at the angles of a 0.50 m turn. A command for a tighter
turn is driven at full lock: the speed is kept, the yaw rate is not, and the
most the robot turns is ω = v / 0.50.

With v = 0 there is no angle that produces a yaw rate, so the robot stands
still whatever ω is. `linear.y` is ignored.

The numbers live in `ros2_ws/src/robots/ackermann/ackermann.urdf.xacro`
(`min_turn_radius` sets the steering limits) and are repeated for the
controller in `ackermann_controllers.yaml`.

## Measured

Headless, on 2026-11-26, each command held for the stated time from standstill.
The radius is path length over rotation, leaving out the first 2 s while the
wheels steer.

| Command (`linear.x`, `angular.z`) | Commanded radius | True | Wheel odometry |
|---|---|---|---|
| 0.3 m/s, 0, 5 s | straight | 1.427 m, 0.007 rad | 1.434 m, 0.000 rad |
| 1.0 m/s, 0, 4 s | straight | 3.763 m | 3.840 m |
| 0, 1.0 rad/s, 5 s | on the spot | 0.000 m, 0.000 rad | 0.000 m, 0.000 rad |
| `linear.y` 0.3 m/s, 3 s | sideways | 0.000 m | 0.000 m |
| 0.3 m/s, 0.3 rad/s, 8 s | 1.00 m | 1.025 m | 1.000 m |
| 0.3 m/s, 0.6 rad/s, 8 s | 0.50 m | 0.509 m | 0.500 m |
| 0.3 m/s, 2.0 rad/s, 8 s | 0.15 m | 0.509 m | 0.500 m |
| 0.3 m/s, −2.0 rad/s, 8 s | 0.15 m | 0.508 m | 0.500 m |
| 0.5 m/s, 5.0 rad/s, 6 s | 0.10 m | 0.501 m | 0.500 m |
| 1.0 m/s, 5.0 rad/s, 6 s | 0.20 m | 0.502 m | 0.500 m |
| −0.3 m/s, 2.0 rad/s, 6 s | 0.15 m, reversing | 0.509 m | 0.500 m |

Wheel odometry reads 0.5 to 3 % more distance and rotation than the robot
really made. The simulation ran at a real-time factor of 0.94 to 1.01 with one
robot.

## Do not share a network with the robot by accident

The lab robot listens on `/cmd_vel` on `ROS_DOMAIN_ID` 0, and the container
uses host networking. When the robot is on your network, start the simulation
with `ROS_LOCALHOST_ONLY=1` set in every terminal, or use another
`ROS_DOMAIN_ID`.

## Test

```bash
tests/run_ackermann_sim_test.sh
```

Starts the simulation headless in an isolated container and checks, against
Gazebo's pose: the robot spawns and its controller activates, a straight
command moves it, a turn command without speed neither rotates nor moves it, a
1.00 m arc is followed, and a 0.15 m arc in either direction is driven at
0.50 m (not below 0.485 m, not above 0.55 m). `RADIUS_TOLERANCE` overrides the
upper margin, 0.10 by default.

`tests/run_unit_tests.sh` checks that the xacro and the controller YAML agree
on the geometry and that the steering limits match the minimum radius.
