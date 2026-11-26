# ROSbot 2R Simulator Migration — Design

**Date**: 2026-09-24
**Status**: Approved; M1 (ROSbot spawns in Gazebo Fortress) implemented 2026-11-14
**Builds on**: `2026-09-05-control-platform-design.md` (Phases 1 and 2 complete)
**Branch**: `feature/rosbot2r-simulation` (renamed from `feature/rosbot2r-migration`
on 2026-11-14). Simulation only: robot-side work does not belong on this branch.

---

## 1. Context

The lab owns a Husarion ROSbot 2R. The course plan is for student teams to
develop their controllers in simulation and then, during a lab session, run
the same code on the physical robot. The first such session is **within four
weeks** of this document.

MUSI-nav2 simulates a TurtleBot3 Waffle in Gazebo Classic 11. Neither matches
the lab robot, so code written against the simulator today would not run on it
unchanged. This design replaces the simulated robot and the simulator so that
the simulation and the hardware expose one interface.

### 1.1 Decisions taken during brainstorming

| Question | Decision |
|---|---|
| Relationship to the control platform | ROSbot 2R **replaces** TurtleBot3. One robot, one simulator, one topic contract |
| Feedback signal for controllers | A `feedback_topic` parameter. Simulation defaults to `/groundtruth/odom`, hardware to `/odometry/filtered` |
| Scope of this spec | **Simulator migration only.** Hardware bringup and classroom access get their own specs |
| Integration approach | **A**: thin wrapper over Husarion's launch, `rosbot_ros` built from source in our Dockerfile |
| Ackermann reference robot | **Cut.** Becomes a documented recipe only (see §8) |

### 1.2 Why Humble and Fortress, not Jazzy and Harmonic

Control-platform spec §3.5 recommends Jazzy plus Gazebo Harmonic as the
long-term target. That recommendation does not survive contact with the robot:

- `husarion/rosbot_ros` on `jazzy` (v1.x) supports ROSbot 3, 3 PRO and XL only.
  **ROSbot 2R was dropped.** Its `robot_model` choices are `rosbot` meaning
  ROSbot 3, and `rosbot_xl`.
- The `humble` branch (v0.15.x) is the ROSbot 2R line. Its simulation runs on
  **Gazebo Fortress** through `ros_gz_sim`, `ros_gz_bridge` and
  `ign_ros2_control`.
- Fortress reaches end of life in **May 2027**, the same month as Humble. Both
  are valid for this cohort.

The migration therefore moves off end-of-life Gazebo Classic onto supported
Fortress while staying on Humble. It also means this repository tracks a
maintenance-only branch: the `husarion/rosbot:humble` driver image was last
built on 2025-03-19. That is acceptable for one semester and is recorded as a
risk in §9.

---

## 2. Architecture

The five-layer split from the control-platform spec survives unchanged. Only L0
is coupled to the simulator, which is what makes this migration affordable.

| Layer | Before | After |
|---|---|---|
| L0 Simulation | `gzserver`/`gzclient`, TurtleBot3 spawn | `rosbot_gazebo/simulation.launch.py` on Fortress, wrapped |
| L1 Groundtruth | `/model_states` into a node | gz `OdometryPublisher`, bridged, into a smaller node |
| L2 Control | `controller_template.py` | Unchanged apart from the `feedback_topic` parameter |
| L3 Evaluation | analyzer and visualizer | Unchanged apart from an `actual_topic` parameter |
| L4 Nav2 / SLAM | Classic topics and frames | Parameter changes only |

### 2.1 The robot's interface

**Revised 2026-11-14 after the robot was brought up.** The contract is the
topic list measured on the lab robot (driver `husarion/rosbot:humble-0.14.0-20250319`),
not the list in `ROS_API.md` below, which names topics the robot does not
publish. Measured and required in simulation: `/cmd_vel`, `/odometry/filtered`,
`/imu_broadcaster/imu`, `/joint_states`, `/scan`, `/tf`, `/tf_static`. The
robot also publishes `/battery`; there is no `imu/data`, `battery_state` or
`odometry/wheels` (the wheel odometry is `/rosbot_base_controller/odom`).
`tests/test_rosbot_sim_contract.sh` checks this list.

Original text, superseded: Husarion documents one topic set for simulation and hardware in
`rosbot_ros@humble/ROS_API.md`. Both targets publish:

`cmd_vel`, `scan`, `scan_filtered`, `odometry/filtered`, `odometry/wheels`,
`imu/data`, `joint_states`, `dynamic_joint_states`, `tf`, `tf_static`,
`robot_description`, `diagnostics`, `set_pose`.

Hardware alone adds `/battery_state`. This list is the contract the whole
migration exists to honour; §6 turns it into a test.

Frames follow from one fact: **ROSbot 2R has no `base_footprint`.** The base
frame is `base_link`, the odometry frame is `odom`, the IMU frame is
`imu_link`.

Kinematics: the robot is skid-steer, driven by `diff_drive_controller`. Setting
`mecanum:=True` switches the URDF and the controller to
`mecanum_drive_controller`. Two of the three kinematics the course names come
with the robot.

---

## 3. Components

### 3.1 Container (`Dockerfile`)

**Revised 2026-11-14 during M1.** Three findings changed this section:

- **Building from source is no longer possible at the robot's version.** The
  robot runs the image built from `rosbot_ros` commit `c860d02` (the `0.14.0`
  tag is older and differs). That commit pins `husarion/ros_components_description`,
  which now returns 404; the renamed `husarion_components_description` does not
  hold the pinned commit. The Dockerfile therefore copies `/ros2_ws/install`
  from `husarion/rosbot-gazebo:humble-0.14.0-20250319`, pinned by digest, into
  `/opt/rosbot_ws/install`. Same binaries as the robot's driver.
- **The camera halves simulation speed.** Without a GPU the Astra renders on
  the CPU: real-time factor 0.15–0.52 on atlas with it, 0.84–1.00 without.
  `patches/rosbot_ros/0001-optional-camera.patch` makes it conditional on
  `ROSBOT_CAMERA`, off by default.
- **Only two runtime packages are missing** from `humble-desktop-full`:
  `ros-humble-ign-ros2-control` and `ros-humble-robot-localization`. They are
  listed by hand; `rosdep install` over the install tree finds nothing because
  of colcon's `COLCON_IGNORE`.

Classic is not removed in M1. TurtleBot3 and `gazebo_ros` stay until the
students move over, so the existing demos keep working; removal moves to M4.

Original plan, superseded where it conflicts with the above.

Remove:

- `ros-humble-turtlebot3*`
- `ros-humble-gazebo-ros-pkgs`
- `ros-humble-gazebo-ros2-control`
- `ENV TURTLEBOT3_MODEL`, `ENV GAZEBO_MODEL_PATH`

Add:

- `ros-humble-ros-gz` (the Fortress pairing for Humble)
- `ros-humble-ign-ros2-control`
- `python3-vcstool`
- A build layer: clone `rosbot_ros` at `humble`, `vcs import` its
  `rosbot/rosbot_simulation.repos`, `rosdep install`, then
  `colcon build --packages-up-to rosbot` with
  `HUSARION_ROS_BUILD_TYPE=simulation`. Built into an underlay outside
  `/ros2_ws`, so the bind-mounted student workspace cannot shadow it.

`ros-humble-ros2-control` and `ros-humble-ros2-controllers` stay. Husarion's
controllers depend on them.

Pin `rosbot_ros` to a tag or commit rather than the moving `humble` branch
head, so that a rebuild produces the image that was verified.

### 3.2 Launch wrapper (`ros2_ws/src/simulation.launch.py`)

Keep the parts this repository owns: `worlds.yaml` resolution, the `headless`
argument, spawn `x`/`y`, and `simulation_params.yaml` defaults, including the
error messages that list supported worlds.

Replace the Gazebo half with an `IncludeLaunchDescription` of
`rosbot_gazebo/launch/simulation.launch.py`, passing `robot_model:=rosbot`,
`gz_world`, `gz_headless_mode`, `x`, `y` and `mecanum`. Add one
`ros_gz_bridge` node for the groundtruth odometry (§3.3) and start
`groundtruth_pose.py`.

Husarion's own `gz_bridge.yaml` bridges only `/clock`, and its
`rosbot_bridge.yaml` only `cmd_vel`. Sensors are bridged by
`husarion_components_description`. Groundtruth is bridged by nobody, so it is
ours to add.

### 3.3 Groundtruth (L1)

Fortress ships `gz-sim-odometry-publisher-system`. It computes odometry from
the model's **true world pose** and publishes `gz.msgs.Odometry`, carrying both
pose and twist. Bridged to `nav_msgs/Odometry`, it replaces `/model_states`
entirely.

**The plugin is model-level.** Control-platform spec §3.1 records that
`libgazebo_ros_state.so` failed silently because it is a world plugin and
`gzserver -s` only loads system plugins. Fortress has the same class of trap:
`OdometryPublisher` must sit inside the robot model. Declared anywhere else it
loads without error and publishes nothing. It is injected through a `<gazebo>`
block in a small xacro that includes Husarion's `rosbot.urdf.xacro`, so their
files stay untouched. A comment at that block records why.

Plugin configuration:

- `<odom_frame>world</odom_frame>`
- `<robot_base_frame>base_link_gt</robot_base_frame>`
- `<dimensions>2</dimensions>`
- `<odom_publish_frequency>50</odom_publish_frequency>`, matching the 50 Hz
  that control-platform criterion 2 already measured
- `<odom_topic>` set to a gz topic distinct from anything Husarion bridges

The `_gt` suffix keeps the groundtruth TF out of `ign_ros2_control`'s real
`odom -> base_link` tree, the same discipline as the current
`base_footprint_gt`.

`groundtruth_pose.py` becomes smaller. It subscribes to the bridged
`nav_msgs/Odometry` and republishes it as `/groundtruth/pose` (`PoseStamped`)
and `/groundtruth/odom` (`Odometry`), broadcasting TF `world -> base_link_gt`.
Model-name lookup, index search and rate throttling disappear, because the
plugin publishes exactly one model at a fixed rate. Twist now comes from the
simulator directly rather than from `ModelStates`.

### 3.4 Control and evaluation (L2, L3)

- `controller_template.py`: replace the hardcoded `/groundtruth/odom`
  subscription at line 156 with a `feedback_topic` parameter, default
  `/groundtruth/odom`. The docstring states that on the physical robot the
  value is `/odometry/filtered`, a drifting estimate rather than the truth.
- `trajectory_analyzer.py`: the same change, as an `actual_topic` parameter.
  On hardware the analyzer measures tracking against the robot's own estimate.
  Its report says which topic it recorded so the two are never confused.

The default `speed` of 0.15 m/s sits well inside the robot's 1.0 m/s limit and
does not change.

### 3.5 Navigation and SLAM configuration (L4)

`shared/configs/nav2_params.yaml` and `nav2_params_slam.yaml`:

| Key | From | To |
|---|---|---|
| `base_frame_id` | `base_footprint` | `base_link` |
| `odom_topic` | `/odom` | `/odometry/filtered` |
| `max_vel_x` | 0.22 | 1.0, from `rosbot_controller/config/rosbot/controllers.yaml` |
| `max_vel_theta` | 1.0 | 3.14, from the same file |
| `robot_radius` | 0.22 | Computed from the ROSbot 2R footprint in `rosbot_description` |
| costmap scan `topic` | `/scan` | `/scan_filtered` |

Four `slamtoolbox_*.yaml` files: `base_frame` becomes `base_link`, and
`max_laser_range` is taken from the `dist_max` of the lidar variant that
`basic.yaml` instantiates (`LDR02`) — read from the xacro, not assumed.

Three `cartographer_*.lua` files are unchanged. Their `tracking_frame` is
`imu_link`, which ROSbot 2R also uses.

### 3.6 Worlds

| File | Disposition |
|---|---|
| `turtlebot3_world.world`, `turtlebot3_house.world`, `empty_world.world` | Deleted. `husarion_gz_worlds` provides `husarion_world.sdf`, `husarion_office.sdf` and `empty_with_plugins.sdf` |
| `landmark_world.world` | Ported to Fortress SDF as `landmark_world.sdf`. It is the only world this repository authored. Its `libgazebo_ros_*` plugins are dropped; Fortress needs the world-level system plugins that `empty_with_plugins.sdf` declares |
| `shared/configs/worlds.yaml` | Rewritten to name `.sdf` paths |

Deleting the three TurtleBot3 worlds also closes the root-ownership defect in
control-platform spec §1.1, since those are the three affected files.
`landmark_world.sdf` is created by a host-side commit, not through the bind
mount, so it is owned by the user.

---

## 4. Data flow

```
gz sim (Fortress) + husarion_gz_worlds / landmark_world.sdf
  ├── ign_ros2_control ──► drive_controller ──► /odometry/wheels
  │                                   └── ekf_node ──► /odometry/filtered  (drifts)
  ├── gz sensors ──► bridge ──► /scan, /scan_filtered, /imu/data
  └── OdometryPublisher (inside robot model, true world pose)
        └── gz.msgs.Odometry ──► ros_gz_bridge ──► nav_msgs/Odometry
                                                     │
                                            groundtruth_pose.py
                                              ├── /groundtruth/pose
                                              ├── /groundtruth/odom
                                              └── TF: world -> base_link_gt
                                                        │
                        controller_template.py ◄────────┘
                          feedback_topic:
                            simulation → /groundtruth/odom
                            hardware   → /odometry/filtered
                          ├── /cmd_vel ──► drive_controller
                          └── /reference/pose ──► trajectory_analyzer.py
```

---

## 5. Error handling

The rule from control-platform spec §3.1 carries over unchanged: **warn loudly,
publish nothing, never substitute a different signal.**

| Condition | Behaviour |
|---|---|
| Bridged groundtruth odometry never arrives | Warn that `OdometryPublisher` is probably missing or declared outside the model; name the fix. Publish nothing |
| `/odometry/filtered` absent | Warn that the `ign_ros2_control` spawners failed. Do not fall back to `/odometry/wheels` |
| `feedback_topic` has no publisher | Warn with the topic name and both valid defaults. Never substitute |
| World `.sdf` not found | Existing `worlds.yaml` resolution raises with the supported list. Kept as is |

---

## 6. Testing strategy

Tests are written before the code they cover (RED, GREEN, REFACTOR).

1. **Contract test.** New, and the most valuable. Launch headless, collect
   `ros2 topic list`, assert it contains every topic in §2.1. `/battery_state`
   is the only permitted hardware-only topic. This test protects sim-to-real
   between now and the lab session, and it can be written before any robot is
   touched.
2. **Unit.** Surviving pure functions in `groundtruth_pose.py` keep their
   tests. Tests for model-name lookup are deleted with the code they covered.
3. **Integration.** Command a known `/cmd_vel`, assert `/groundtruth/odom`
   tracks it. Replaces the body of `tests/test_groundtruth_integration.sh`.
4. **Launch.** Each world in `worlds.yaml` spawns the robot and produces
   `/scan`, `/odometry/filtered` and `/cmd_vel`. Replaces the body of
   `tests/test_multi_world_launch.sh`.
5. **Demo verification.** Re-run `tests/run_demo_verification.sh` against the
   new stack and commit the evidence, as `997f2a0` and `2a4166f` did. This is
   the go/no-go gate.

### 6.1 Acceptance tests

- **Given** simulation in any world from `worlds.yaml`, **when** it has started,
  **then** every topic in §2.1 except `/battery_state` is listed.
- **Given** simulation is running, **when** `/groundtruth/odom` is sampled,
  **then** it publishes within 10% of 50 Hz.
- **Given** `OdometryPublisher` is removed from the model, **when** the
  groundtruth node runs, **then** it warns and publishes nothing.
- **Given** a student's unmodified controller, **when** `feedback_topic` is
  changed to `/odometry/filtered`, **then** it runs with no code change.
- **Given** `mecanum:=True`, **when** commanded pure `linear.y`, **then** the
  robot translates sideways.

---

## 7. Success criteria

1. The contract test passes in every world.
2. `ros2 run tf2_ros tf2_echo world base_link_gt` resolves.
3. `/groundtruth/odom` measures within 10% of 50 Hz.
4. `controller_template.py`, unmodified, drives ROSbot 2R around its default
   circle reference, and `trajectory_analyzer.py` scores the run.
5. Nav2 and SLAM Toolbox demos run on ROSbot 2R in `husarion_world.sdf`.
6. The Cartographer demo runs with its `.lua` configurations unchanged;
   `slam_cartographer.launch.py` loses its TurtleBot3 references.
7. The demo verification table is re-committed with every row passing, or
   with each failure explained.
8. No file under `shared/worlds/` is root-owned.
9. `rg -i 'turtlebot3|base_footprint|gazebo_ros'` over `ros2_ws/`, `shared/`,
   `tests/` and `Dockerfile` returns nothing.

---

## 8. Changes to the control-platform spec

- **§3.4 Robot design path**: the worked Ackermann robot, its side-by-side
  controller YAML and the `SmacPlannerHybrid` variant are cut.
  `docs/robot_design.md` remains, as a recipe that shows how a team would
  author an Ackermann chassis with `ackermann_steering_controller` on
  `ign_ros2_control`. ROSbot 2R already covers skid-steer and mecanum.
- **§3.5 Stack currency**: superseded by §1.2 of this document.
- **§7 criteria 8 and 9**: dropped with the Ackermann example.
- **§6.1**: the `ModelStates` and `model_name` acceptance tests are replaced by
  §6.1 of this document.

---

## 9. Implementation order

Each step leaves the repository verifiable.

0. Tag `feature/control-platform` HEAD as `archive/gazebo-classic`. The
   Classic stack stays recoverable in one command.
1. Dockerfile (§3.1). Verify: image builds, `ros2 pkg list` shows
   `rosbot_gazebo`.
2. Launch wrapper and `worlds.yaml` (§3.2, §3.6 first half). Verify: robot
   spawns in `husarion_world.sdf`.
3. Contract test (§6 item 1). Written now, before anything depends on it.
4. Configuration changes (§3.5).
5. Groundtruth port (§3.3), test first.
6. `feedback_topic` and `actual_topic` parameters (§3.4).
7. `landmark_world.sdf` port (§3.6).
8. Launch, integration and unit tests brought to green.
9. Demo verification and committed evidence.

The contract test comes before groundtruth deliberately. It is cheap, it is the
interface guarantee, and if it fails everything after it rests on a wrong
assumption.

---

## 10. Risks

| Risk | Severity | Mitigation |
|---|---|---|
| Four-week deadline, with two more specs behind this one | High | Scope limited to simulation. Ackermann cut. Order in §9 puts the contract test early |
| `rosbot_ros@humble` is maintenance-only; driver image last built 2025-03-19 | Medium | Pin to a verified commit. Acceptable for one cohort. A future ROSbot 3 or XL purchase forces a move to Jazzy |
| Lab robot's installed software and firmware unknown | High, for the next spec | First task of the hardware spec: record image tags, `snap list` and firmware version before anything else. Firmware must match the driver version |
| `OdometryPublisher` silently misplaced | Medium, same class as a bug already hit once | Explicit comment at the plugin, and an acceptance test covering its absence |
| Fortress SDF port of `landmark_world` harder than expected | Low | The Husarion worlds alone satisfy every criterion except the landmark world itself |
| Image size grows with a source build | Low | Underlay built in one layer with build artefacts removed |

---

## 11. Out of scope

- **Hardware bringup**: driver container, micro-ROS serial link, firmware
  version check, single-robot bench test. Next spec.
- **Classroom access**: discovery across the lab network, per-team namespaces,
  topic filtering, arbitration of one robot between teams, safety stop. Spec
  after that.
- Migration to Jazzy and Harmonic (see §1.2).
- A worked Ackermann robot (see §8).
