# Lab 1: compare wheel drives by measurement

You send the same four velocity commands to four robots in the simulator and
write down what each robot really does. The lecture made claims about
differential, skid-steer, mecanum and Ackermann (car-like) drives; here you
check them with numbers.

Time: 45 minutes. Hand in: this sheet with the four tables and the four
answers, one per team.

## What you need

- The course image, built (`./build.sh`), and the container running (`./run.sh`).
- Two terminals in the container. Open each with:

```bash
docker exec -it nav2_workspace bash
```

- **In every terminal, before anything else:**

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

These two lines keep your simulation on your own computer. The lab robot
listens on the same network. Without them, the commands meant for your
simulated robot can reach the real one and move it.

## The four robots

| Drive | Robot | Wheels |
|---|---|---|
| Differential | TurtleBot3 Waffle | two driven wheels, two ball casters |
| Skid-steer | ROSbot 2R | four driven wheels, no steering, no casters |
| Mecanum | ROSbot 2R with mecanum wheels | four driven wheels with 45° rollers |
| Ackermann | small car-like robot | rear wheels drive, front wheels steer; wheelbase 0.26 m |

## The four manoeuvres

`drive_test.py` sends each command for a fixed time, stops the robot, waits two
seconds and measures. The commands are the same for every robot.

| Manoeuvre | Command on `/cmd_vel` | Time | A robot that obeys exactly ends at |
|---|---|---|---|
| straight | `linear.x` = 0.2 m/s | 5 s | 1.000 m forward |
| spin | `angular.z` = 1.0 rad/s | 2 s | same place, turned 114.6° |
| arc | `linear.x` = 0.2 m/s and `angular.z` = 0.5 rad/s | 4 s | 0.364 m forward, 0.566 m left, turned 114.6° |
| sideways | `linear.y` = 0.2 m/s | 5 s | 1.000 m to the left |

For each manoeuvre the script prints three groups of three numbers:

- **commanded**: where a robot that obeys exactly would end.
- **true (Gazebo)**: where the robot really ended. The simulator knows this
  exactly. A real robot has no such signal.
- **wheel odometry**: where the robot believes it ended, counting only wheel
  rotation. This is what a real robot has.

Each group is `fwd m`, `left m`, `turn °`: metres forward, metres to the left
and degrees turned counter-clockwise, all measured in the direction the robot
faced when the manoeuvre started.

## Run it

Do the four robots one after the other. Only one simulation may run at a time.

### 1. Differential: TurtleBot3 Waffle

Terminal 1:

```bash
ros2 launch /ros2_ws/src/simulation.launch.py world:=empty_world rviz:=false
```

Wait until the robot stands in the Gazebo window. Terminal 2:

```bash
python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=turtlebot3
```

The run takes about 30 seconds. Copy the numbers into table 1. Then press
Ctrl-C in terminal 1 and wait for the prompt.

### 2. Skid-steer: ROSbot 2R

Terminal 1:

```bash
ros2 launch /ros2_ws/src/rosbot_simulation.launch.py world:=empty_with_plugins
```

This simulator needs about one minute to start. Wait for the line
`Configured and activated rosbot_base_controller`. Terminal 2:

```bash
python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=rosbot
```

Copy the numbers into table 2. Ctrl-C in terminal 1.

### 3. Mecanum: ROSbot 2R with mecanum wheels

Terminal 1:

```bash
ros2 launch /ros2_ws/src/rosbot_simulation.launch.py world:=empty_with_plugins mecanum:=true
```

Wait for the same line. Terminal 2:

```bash
python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=rosbot_mecanum
```

Copy the numbers into table 3. Ctrl-C in terminal 1.

### 4. Ackermann: car-like robot

Terminal 1:

```bash
ros2 launch /ros2_ws/src/ackermann_simulation.launch.py
```

Wait for the line `Configured and activated ackermann_steering_controller`.
The terminal then prints `Use of Twist message without stamped is deprecated`
for every command; ignore it. Terminal 2:

```bash
python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=ackermann
```

Copy the numbers into table 4. Ctrl-C in terminal 1.

### If something goes wrong

- `drive_test: no wheel odometry ... Is the simulation ... running?` The
  simulation is not ready, or `robot:=` does not match the simulation that is
  running, or the two `export` lines are missing in one of the terminals.
- Ctrl-C in terminal 2 stops the robot and the script at any time.
- The computer is slow, or the Gazebo window does not open or closes by itself:
  add `headless:=true` to the launch command. Gazebo then runs without its
  window and the numbers are the same.
- A simulation does not start after you stopped another one: run
  `pkill -9 -f "ign gazebo"; pkill -9 gzserver` and start it again.

## Tables

Write the numbers exactly as printed.

**Table 1. Differential (TurtleBot3 Waffle)**

| Manoeuvre | Commanded fwd / left / turn | True fwd / left / turn | Wheel odometry fwd / left / turn |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | | |
| spin | 0.000 / 0.000 / 114.6 | | |
| arc | 0.364 / 0.566 / 114.6 | | |
| sideways | 0.000 / 1.000 / 0.0 | | |

**Table 2. Skid-steer (ROSbot 2R)**

| Manoeuvre | Commanded fwd / left / turn | True fwd / left / turn | Wheel odometry fwd / left / turn |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | | |
| spin | 0.000 / 0.000 / 114.6 | | |
| arc | 0.364 / 0.566 / 114.6 | | |
| sideways | 0.000 / 1.000 / 0.0 | | |

**Table 3. Mecanum (ROSbot 2R, mecanum wheels)**

| Manoeuvre | Commanded fwd / left / turn | True fwd / left / turn | Wheel odometry fwd / left / turn |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | | |
| spin | 0.000 / 0.000 / 114.6 | | |
| arc | 0.364 / 0.566 / 114.6 | | |
| sideways | 0.000 / 1.000 / 0.0 | | |

**Table 4. Ackermann (car-like robot)**

| Manoeuvre | Commanded fwd / left / turn | True fwd / left / turn | Wheel odometry fwd / left / turn |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | | |
| spin | 0.000 / 0.000 / 114.6 | | |
| arc | 0.364 / 0.566 / 114.6 | | |
| sideways | 0.000 / 1.000 / 0.0 | | |

## Questions

Answer each with numbers from your tables.

1. **Spin on the spot.** Which robots turned without leaving their place? For
   each robot, give the true turn and how far the robot moved from where it
   started. One robot does not turn at all: which one, and why can it not turn
   while standing still? One of the others leaves its place by a few
   centimetres: which one, and what do its wheels have to do in order to turn?

2. **Sideways.** Which robots moved to the left when `linear.y` was commanded?
   What did the others do with the command? Name the part of the robot that
   makes the difference.

3. **Wheel odometry in a turn.** For the spin and for the arc, compute
   `wheel odometry turn − true turn` for each robot. Which drive has the
   largest error? The straight line shows almost no error on any robot. Why
   does turning expose an error that driving straight does not? Think about
   which robot dimension the odometry needs for a turn and does not need for a
   straight line.

   Then look at the arc alone. The command asks for a circle of radius
   `linear.x / angular.z` = 0.2 / 0.5 = 0.40 m. The radius a robot really drove
   is `sqrt(fwd² + left²) / (2 · sin(turn / 2))`, with the true numbers.
   Compute it for the Ackermann robot and for one other robot. The Ackermann
   robot's steering stops at a turn of radius 0.50 m. What did it do with a
   command for a tighter turn, and does its wheel odometry know?

4. **Which drive for which task.** Choose a drive for each task and justify the
   choice with one number from your tables:
   a) a warehouse robot that must slide into a narrow gap between two shelves;
   b) an outdoor robot on gravel and grass;
   c) an indoor robot that must know its position from wheel odometry alone for
   as long as possible;
   d) a robot that carries a load at speed along roads and wide corridors, and
   never needs to turn in a space smaller than one metre across.

## Optional

The ROSbot also publishes `/odometry/filtered`, which adds the gyroscope to the
wheels. Repeat the skid-steer or the mecanum run with it and compare the turn
with table 2 or 3:

```bash
python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=rosbot_mecanum -p odom_topic:=/odometry/filtered
```
