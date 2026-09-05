# Demo Verification

Evidence that the demos in this repository actually run. Nothing here had been
executed between 2025-11-07 and 2026-09-05, so this record replaces assumption.

Regenerate with:

```bash
./tests/run_demo_verification.sh
```

Each demo runs in its own fresh container, which is both what a student does and
the only way to keep one demo's shutdown from corrupting the next one's checks.
Logs land in `shared/verification_logs/`. The sweep takes about 20 minutes.

## Results — 2026-09-06

Image `nav2_student:humble`, ROS2 Humble, Gazebo Classic, headless.

| Demo | Result | What was checked |
|---|---|---|
| `sim_landmark` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `sim_turtlebot3_world` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `sim_turtlebot3_house` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `sim_empty_world` | PASS | Robot spawns; `/scan` and `/odom` publish |
| `groundtruth` | PASS | `/groundtruth/pose` and `/groundtruth/odom` publish |
| `nav2` | PASS | `bt_navigator` running; `/map` served from the prebuilt map |
| `slam_nav2` | PASS | SLAM Toolbox builds `/map` online, no prebuilt map |
| `slam_cartographer` | PASS | Cartographer publishes `/map` |
| `replay_bag` | PASS | Recorded bag replays; `/scan` republished |

Groundtruth additionally passes an 8-check integration suite
(`./tests/run_integration_tests.sh`), including a measured publish rate of
**49.966 Hz** and confirmation that an absent model warns rather than silently
substituting another pose source.

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

## Known issue, not yet fixed

Three world files (`empty_world`, `turtlebot3_house`, `turtlebot3_world`) are
root-owned on the host. They were created through the bind mount by the container,
which runs as root, so host-side edits fail with `PermissionError`. Students
editing a world will hit this. Fixing it needs `sudo chown` on the host.
