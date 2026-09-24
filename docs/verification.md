# Demo Verification

Evidence that the demos in this repository actually run. Nothing here had been
executed between 2025-11-07 and 2026-09-05, so this record replaces assumption.
Re-run whenever the image changes: a new package set makes old results stale.

Regenerate with:

```bash
./tests/run_demo_verification.sh
```

Each demo runs in its own fresh container, which is both what a student does and
the only way to keep one demo's shutdown from corrupting the next one's checks.
Logs land in `shared/verification_logs/`. The sweep takes about 20 minutes.

## Results — 2026-09-24

Image `nav2_student:humble` with `ros2_control`, `ros2_controllers`,
`gazebo_ros2_control` and matplotlib, running as the non-root `student` user.
Re-run in full after each image change, since a changed image invalidates
earlier evidence.

| Demo | Result | What was checked |
|---|---|---|
| `sim_landmark` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `sim_turtlebot3_world` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `sim_turtlebot3_house` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `sim_empty_world` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `groundtruth` | PASS | `/groundtruth/pose` and `/groundtruth/odom` publish |
| `control_loop` | PASS | Controller drives from groundtruth; `/cmd_vel` and `/reference/pose` publish |
| `nav2` | PASS | `bt_navigator` running; `/map` served from the prebuilt map |
| `slam_nav2` | PASS | SLAM Toolbox builds `/map` online, no prebuilt map |
| `slam_cartographer` | PASS | Cartographer publishes `/map` |
| `replay_bag` | PASS | Recorded bag replays; `/scan` republished |

Groundtruth additionally passes an 8-check integration suite
(`./tests/run_integration_tests.sh`), including a measured publish rate of
**49.966 Hz** and confirmation that an absent model warns rather than silently
substituting another pose source.

The control loop passes a 7-check end-to-end test
(`./tests/run_control_loop_test.sh`): the robot moves, the run is scored, and a
plot is written. A representative 44-second circle-following run with the shipped
proportional controller:

| Metric | Value |
|---|---|
| samples | 2171 |
| position RMSE | 0.761 m |
| position max error | 2.828 m |
| cross-track RMSE | 0.291 m |
| cross-track max | 1.236 m |
| settling time | never settled |

These numbers are deliberately unimpressive. The shipped controller is a
proportional baseline that lags a moving target and never settles, which is what
students are asked to improve on. The test asserts that the loop closes and the
run is measurable, **not** that tracking is good — so it would also pass a
controller driving steadily away from the reference. Guarding control quality
needs a separate threshold test with a number chosen on purpose.

## Two defects this sweep existed to catch

**The simulation was broken before 2026-09-05.** `spawn_entity.py` waited 30
seconds for `/spawn_entity`, but Gazebo's ROS interface needs about 60 seconds in
this container. The robot never spawned. The error blamed a missing plugin:

```
Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?
```

`libgazebo_ros_factory.so` had in fact loaded correctly; the message is
misleading. Fixed by raising the spawn timeout to 180 seconds.

**`GazeboRosState` cannot be loaded with `gzserver -s`.** It registers as a world
plugin, not a system plugin, so `-s` fails silently — gzserver starts, reports no
error, and `/model_states` never appears. It must be declared inside each world
file's `<world>` element. All four world files now do, each with a comment saying
why, because `-s` is the natural first guess.

## Root-owned output — fixed 2026-09-24

The container used to run as root, so **every** file it wrote into `./shared` and
`./ros2_ws` — saved maps, bags, results, logs, even the world files — was
root-owned on the host. Students could not edit or delete their own work without
`sudo`.

It now runs as a `student` user whose UID/GID match the host user; `build.sh`
passes `$(id -u)` and `$(id -g)` at build time. Passwordless `sudo` remains inside
the container for installing packages.

`./tests/test_file_ownership.sh` checks this from the host. After the full sweep
above ran as the new user, a scan found zero root-owned files in the repository.

**Not covered by any test: the GUI.** Every check here is headless, and the
Xauthority mount moved from `/root` to `/home/student`. Confirm Gazebo and RViz
windows open with `./run.sh` on a real display.
