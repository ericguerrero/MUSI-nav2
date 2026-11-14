# ROSbot 2R simulation

Gazebo Fortress with the Husarion ROSbot 2R: the same drive controller, EKF,
LiDAR and IMU as the lab robot, and the same topic names. A controller written
against this simulation reads and writes what it will read and write on the
real robot.

## Run it

Inside the container (`./run.sh`, then `docker exec -it nav2_workspace bash`):

```bash
ros2 launch /ros2_ws/src/rosbot_simulation.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `world` | `husarion_world` | `husarion_world`, `husarion_office`, `empty_with_plugins`, `sonoma_raceway`, or an absolute `.sdf` path |
| `headless` | `false` | `true` runs Gazebo without its window |
| `x`, `y`, `yaw` | `0.0`, `-2.0`, `0.0` | spawn pose, metres and radians |
| `mecanum` | `false` | `true` switches to mecanum wheels and `mecanum_drive_controller` |

The camera is off because it halves the simulation speed on a computer without
a GPU. Start with `ROSBOT_CAMERA=true ros2 launch ...` to add it.

Drive it from a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | in |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | out, about 20 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | out, about 10 Hz |
| `/imu_broadcaster/imu` | `sensor_msgs/msg/Imu` | out, about 20 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | out |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | out |

The base frame is `base_link`; there is no `base_footprint`.

## Do not share a network with the robot by accident

The lab robot uses `ROS_DOMAIN_ID` 0 with no namespace, and the container uses
host networking. A simulation started on the same network and domain exchanges
`/cmd_vel` and `/odometry/filtered` with the real robot, and a controller meant
for Gazebo then drives the robot. When the robot is on your network, start the
simulation with `ROS_LOCALHOST_ONLY=1` set in every terminal, or use another
`ROS_DOMAIN_ID`.

## Test

```bash
tests/run_rosbot_sim_contract_test.sh
```

Starts the simulation headless in an isolated container and checks the topic
list against the one measured on the robot, the odometry rate, the real-time
factor, and that `/cmd_vel` moves the robot. Thresholds can be overridden with
`MIN_ODOM_HZ` and `MIN_RTF`.
