# Lab 1 drive comparison: lecturer notes

Reference numbers and expected answers for
[control_01_drive_comparison.md](control_01_drive_comparison.md). Not for
students before the hand-in.

## How the numbers were measured

- 2026-11-26, image `nav2_student:humble` (built 2026-11-18), `headless:=true`,
  worlds `empty_world` (TurtleBot3) and `empty_with_plugins` (ROSbot).
- Each simulation in its own container on Docker's default bridge network with
  `ROS_LOCALHOST_ONLY=1` and a non-zero `ROS_DOMAIN_ID`. Not with `./run.sh`:
  the student path (host networking, Gazebo window open) was not run.
- Three consecutive runs of `drive_test.py` per robot in one simulation
  session, without restarting the simulation between runs. The tables show the
  first run. The spread over the three runs is given below each table.
- Each cell is `fwd m / left m / turn °` in the frame the robot had at the
  start of the manoeuvre.

## Reference tables

**Differential (TurtleBot3 Waffle)**

| Manoeuvre | Commanded | True (Gazebo) | Wheel odometry |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | 1.020 / 0.000 / 0.0 | 1.020 / 0.000 / 0.0 |
| spin | 0.000 / 0.000 / 114.6 | -0.001 / -0.003 / 120.6 | 0.000 / 0.000 / 121.0 |
| arc | 0.364 / 0.566 / 114.6 | 0.400 / 0.551 / 115.5 | 0.399 / 0.554 / 115.9 |
| sideways | 0.000 / 1.000 / 0.0 | 0.000 / 0.000 / -0.2 | 0.000 / 0.000 / -0.2 |

Spread over three runs: straight 1.020 m every time, with up to 0.022 m and
0.9° of drift to the right; spin true 119.1° to 120.6°; arc true 115.5° to
115.9°. Wheel odometry turn minus true turn: +0.4° to +0.5° in every spin and
arc.

**Skid-steer (ROSbot 2R)**

| Manoeuvre | Commanded | True (Gazebo) | Wheel odometry |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | 1.000 / 0.000 / 0.0 | 1.000 / 0.000 / 0.0 |
| spin | 0.000 / 0.000 / 114.6 | 0.039 / -0.020 / 117.1 | 0.000 / 0.000 / 114.6 |
| arc | 0.364 / 0.566 / 114.6 | 0.394 / 0.544 / 114.3 | 0.354 / 0.573 / 114.6 |
| sideways | 0.000 / 1.000 / 0.0 | 0.000 / 0.000 / 0.0 | 0.000 / 0.000 / 0.0 |

Spread over three runs: spin true 114.2° to 117.8°, with the robot 0.034 to
0.039 m forward and 0.018 to 0.024 m to the right of where it started; spin
odometry 114.6° twice and 117.5° once; arc true 114.3° to 118.1°, arc odometry
identical in all three. Wheel odometry turn minus true turn: -3.5° to +0.4°.

**Mecanum (ROSbot 2R, `mecanum:=true`)**

| Manoeuvre | Commanded | True (Gazebo) | Wheel odometry |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | 1.000 / 0.000 / 0.0 | 1.000 / 0.000 / 0.0 |
| spin | 0.000 / 0.000 / 114.6 | 0.000 / 0.000 / 128.3 | 0.000 / 0.000 / 114.6 |
| arc | 0.364 / 0.566 / 114.6 | 0.269 / 0.583 / 128.4 | 0.354 / 0.573 / 114.6 |
| sideways | 0.000 / 1.000 / 0.0 | 0.000 / 1.000 / 0.0 | 0.000 / 1.000 / 0.0 |

Spread over three runs: none, the three runs printed the same numbers. Two
further runs in other sessions gave the same spin and arc; one of them gave
1.010 m sideways, true and odometry alike. Wheel odometry turn minus true turn:
-13.7° in the spin, -13.8° in the arc.

## What students should expect to differ from these tables

- TurtleBot3 numbers move by a few millimetres and up to 3° between runs (a spin in a freshly started simulation read 122.0°).
- ROSbot wheel odometry turns come in steps of about 2.9°: the controller runs
  at 20 Hz, and at 1 rad/s one period is 2.9°. A spin reads 114.6° or 117.5°.
- A slow computer does not change the numbers: the script times the manoeuvres
  on simulation time, not on the wall clock.

## Things about the simulation worth knowing before class

- **TurtleBot3 `/odom` is not wheel odometry.** The Gazebo plugin fills it from
  the simulator's pose. Comparing it with the truth always gives zero error.
  `drive_test.py` therefore integrates the wheel angles from `/joint_states`
  itself, with the radius (0.033 m) and separation (0.287 m) of the model.
- **The TurtleBot3 overshoots the command**: 1.020 m for 1.000 m and about
  120° for 114.6°. The wheel odometry agrees with the truth, so the wheels
  really turned that much. The cause was not investigated; the plugin applies
  commands at 30 Hz and has no command timeout.
- **Skid-steer odometry is good here because it was calibrated here.** The
  controller multiplies the track width by 1.45
  (`wheel_separation_multiplier`), a value that makes odometry match this
  simulated floor (see [rosbot_simulation.md](../rosbot_simulation.md)). On
  another floor the value is wrong and the error returns. The skid-steer signs
  that remain visible are the 4 cm the robot leaves its place while spinning
  and an arc that ends 3 to 5 cm from where odometry puts it.
- **The mecanum robot turns 12 % more than commanded and its odometry does not
  notice.** Odometry reports the commanded 114.6° exactly. This is consistent
  with a controller geometry (`wheel_separation_x` 0.106 m, `wheel_separation_y`
  0.192 m, multipliers 1.0) that does not match the simulated rollers, but that
  explanation was not tested. Straight and sideways motion are exact.
- `/odometry/filtered` adds the gyroscope. Measured once on the mecanum robot:
  spin 128.5° against a true 128.3°, arc 128.3° against 128.4°.

## Expected answers

1. **Spin on the spot.** All three turn on the spot: about 120°, 117° and
   128°. The differential and mecanum robots stay within 3 mm of their place.
   The skid-steer robot ends about 4 cm away: its four wheels point straight
   ahead, so to turn they must slide sideways over the floor, and the sliding
   is not symmetric.
2. **Sideways.** Only the mecanum robot moves: 1.000 m. The other two stay at
   0.000 m; their controllers have no use for `linear.y` and drop it. The
   difference is the wheel: the rollers at 45° let a wheel push diagonally, and
   four of them combine into a sideways force. An ordinary wheel cannot roll
   along its axle.
3. **Wheel odometry in a turn.** Mecanum is worst, about -14° in both turns;
   skid-steer is between -3.5° and +0.4°; differential about +0.4°. A straight
   line needs only the wheel radius. A turn also needs the distance between the
   left and right contact points. For a differential drive this is a fixed,
   known length. For skid-steer and mecanum wheels the effective length depends
   on how the wheels slip, so it is a calibration value, and here it is
   calibrated for the skid-steer robot and not for the mecanum robot.
4. **Which drive.** a) Mecanum: the only one with sideways motion (1.000 m
   against 0.000 m). b) Skid-steer: four driven wheels and no casters or
   rollers to sink or clog; the price is the scrubbing seen in the spin (4 cm).
   The tables do not measure rough ground, so accept any answer argued from the
   wheel layout. c) Differential: smallest turn error (0.4° against up to 3.5°
   and 14°).

## Ackermann robot

`drive_test.py` has a preset `robot:=ackermann` for the car-like robot on
branch `feature/ackermann-robot`
(`ros2 launch /ros2_ws/src/ackermann_simulation.launch.py`). One run against
that branch's files, same isolation as above:

| Manoeuvre | Commanded | True (Gazebo) | Wheel odometry |
|---|---|---|---|
| straight | 1.000 / 0.000 / 0.0 | 1.000 / 0.000 / 0.0 | 1.000 / 0.000 / 0.0 |
| spin | 0.000 / 0.000 / 114.6 | 0.000 / 0.000 / 0.0 | 0.000 / 0.000 / 0.0 |
| arc | 0.364 / 0.566 / 114.6 | 0.530 / 0.475 / 86.1 | 0.525 / 0.490 / 88.8 |
| sideways | 0.000 / 1.000 / 0.0 | 0.000 / 0.000 / 0.0 | 0.000 / 0.000 / 0.0 |

It cannot spin on the spot or move sideways, and the commanded arc (radius
0.40 m) is tighter than its minimum radius (0.50 m), so it turns 86° where
114.6° was asked. The student worksheet does not include this robot yet.

## Tests

```bash
tests/run_unit_tests.sh -k drive_test
tests/run_drive_test_sim_test.sh            # all three robots, about 6 minutes
tests/run_drive_test_sim_test.sh rosbot     # one robot
```
