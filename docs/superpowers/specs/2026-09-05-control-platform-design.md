# Control-Focused Demo Platform — Design

**Date**: 2026-09-05
**Status**: Approved, not yet implemented
**Tracking**: Vikunja task #59 (project MUSI-nav2)

---

## 1. Context

MUSI-nav2 was built as a Nav2 and SLAM teaching repository. Its last commit,
672f282, landed on 2025-11-07; nothing in it has been run or verified in the ten
months since.

The upcoming semester changes what the repository is for. The navigation exercises
(`docs/exercises/03`, `04`, `05`) will not be delivered. Instead, students choose
their own application, justify a robot design and a control strategy suited to it,
and build on this repository as a base. Control is the focus. SLAM and Nav2 stay
present and runnable so that students can see and use the full stack, but the
position signal used for control practice comes from Gazebo groundtruth rather than
from AMCL or SLAM.

Students may swap kinematics — Ackermann, omni/mecanum, skid-steer — so the base
cannot assume differential drive.

The semester starts within weeks. Scope is deliberately thin: verify what exists,
add the missing groundtruth and control layers, and ship one worked non-diff-drive
example plus a recipe rather than a pre-built chassis library.

### 1.1 Current defects

Found during the review that produced this design:

| Location | Defect |
|---|---|
| `docs/exercises/03_navigation_basics.md:5` | Links `../docker_setup.md`, deleted in 672f282 |
| `docs/exercises/04_slam_basics.md` | Filename says SLAM; title says "Autonomous Waypoint Navigation" |
| `.gitignore` | Bag glob is `shared/bags/*.db3`; real layout is `shared/bags/<session>/<name>_0.db3`, one level deeper. 15 MB sits loose |
| `shared/tests/test_multi_world_launch.sh` | Byte-identical duplicate of `tests/test_multi_world_launch.sh` |
| `ros2_ws/src/nav2.launch.py` | Docstring and defaults reference `ex3_2_spawn_robot.launch.py` and `nav2_params_ex3.yaml`; neither exists |
| Working tree | `ros2_ws/src/replay_bag.launch.py` and `shared/scripts/` untracked and undecided |

### 1.2 Stale branches

Neither branch is ahead of master in any useful whole-branch sense. Both forked
before master gained multi-world support, the Cartographer and SLAM Toolbox config
sets, and the world files. **Merging either would regress master.**

- `dev` — 12 commits. Holds `trajectory_analyzer.py` (147 lines) and
  `visualize_trajectory.py` (196 lines), which are worth taking. Also holds a
  groundtruth implementation buried inside `nav2_waypoint_follower.py`, and a
  "spawn coordinate fix" to `(-2.0, 2.0)` that is probably obsolete — master
  already defaults to `x=-1.0, y=1.0` at `simulation.launch.py:128`. Verify before
  porting.
- `feature/session3-navigation-exercise` — 4 commits, 2 unpushed. Nothing needed;
  its Gazebo plugin additions are already matched on master.

Disposition: cherry-pick the two evaluation tools, tag both branches as archive
refs, delete the branches.

---

## 2. Architecture

Five layers. Each is usable without the ones above it.

| Layer | Provides | Status |
|---|---|---|
| L0 Simulation | Gazebo, robot, RViz; 4 worlds; headless and spawn-pose args | Exists — keep |
| L1 Groundtruth | `/groundtruth/pose`, `/groundtruth/odom`, TF | **Build** |
| L2 Control | `controller_template.py` with a fillable stub | **Build** |
| L3 Evaluation | Tracking error metrics and plots | Port from `dev`, retarget |
| L4 Full stack | Nav2, SLAM Toolbox, Cartographer, waypoints, bags | Exists — untouched |

Control students work in L0 through L3. L4 remains available for students whose
application needs autonomous navigation or mapping.

### 2.1 Environment is already capable

Verified against the existing `nav2_student:humble` image (4.8 GB). No Dockerfile
change and no rebuild are required:

```
libgazebo_ros_state.so             libgazebo_ros_ackermann_drive.so
libgazebo_ros_diff_drive.so        libgazebo_ros_planar_move.so
libnav2_smac_planner.so            gazebo_msgs/msg/ModelStates
xacro
```

This is the single most important finding in this design. It means the "swap
kinematics" requirement is a matter of authoring robot descriptions, not of
extending the container.

---

## 3. Components

### 3.1 Groundtruth layer (L1)

**Enabling the signal is a one-line change.** `simulation.launch.py:227-230`
starts gzserver with `-s libgazebo_ros_init.so -s libgazebo_ros_factory.so`.
Adding `-s libgazebo_ros_state.so` to that argument list publishes `/model_states`
for all four worlds at once.

Editing the world files is unnecessary. The `<plugin>` entries at
`landmark_world.world:6-7` duplicate what the launch file already passes, which is
why the other three worlds work despite declaring no plugins of their own.

**New node**: `ros2_ws/src/groundtruth_pose.py`

- Subscribes: `/model_states` (`gazebo_msgs/ModelStates`)
- Publishes: `/groundtruth/pose` (`geometry_msgs/PoseStamped`),
  `/groundtruth/odom` (`nav_msgs/Odometry`, carrying twist so controllers get
  velocity feedback without differentiating pose)
- Broadcasts TF: `world -> base_footprint_gt`
- Parameters: `model_name` (default `waffle`; overridable so student-authored
  robots work), `publish_rate` (default 50 Hz; `/model_states` fires far faster
  than any controller needs)

The node is standalone. In `dev` this logic lives inside
`nav2_waypoint_follower.py` and silently degrades to an odom-versus-AMCL fallback
when groundtruth is unavailable — which is the likely reason it appeared to work
while doing nothing. The replacement warns loudly and publishes nothing rather
than substituting a different signal.

### 3.2 Control layer (L2)

`ros2_ws/src/controller_template.py`

- Subscribes `/groundtruth/odom`, publishes `geometry_msgs/Twist` on `/cmd_vel`
- One stub for students to fill: `compute_command(pose, twist, reference)`
- Ships a trivial P-controller so the file runs unmodified out of the box

**Where the reference comes from.** The template owns its reference internally: a
`reference_at(t)` function returning a target pose, selected by a `reference` parameter
(`point`, `circle`, `figure8`, or `waypoints` read from a YAML list). The template also
publishes the current target on `/reference/pose` so that RViz can draw it and the
analyzer can log it alongside groundtruth. No separate reference-publisher node — one
file stays one file, and a student who wants a different reference edits one function.

`Twist` on `/cmd_vel` is the command interface for the diff-drive, Ackermann,
planar-move, and skid-steer Gazebo plugins alike. A student controller therefore
survives a chassis swap unchanged. This uniformity is what makes the "swap
kinematics" requirement affordable in the available time — without it, each
chassis would need its own controller interface and the scope would not fit.

### 3.3 Evaluation layer (L3)

Cherry-pick `trajectory_analyzer.py` and `visualize_trajectory.py` from `dev`,
retargeting both from odometry-drift analysis to control tracking analysis:

- Analyzer: cross-track error, heading error, RMSE, max error, settling time,
  computed against a reference trajectory rather than against AMCL output
- Visualizer: plot reference against actual, write PNG

### 3.4 Robot design path

One worked Ackermann reference under `ros2_ws/src/robots/`: xacro description plus
`libgazebo_ros_ackermann_drive.so`. Alongside it, a Nav2 parameter variant using
`SmacPlannerHybrid` with a nonzero minimum turning radius, since the existing
`nav2_params.yaml` is diff-drive-only.

`docs/robot_design.md` documents the recipe for authoring a chassis.

One complete example proves the path works. Building a chassis library would
remove the deliverable: authoring a robot for their chosen application *is* the
student assignment.

---

## 4. Data flow

```
Gazebo (gzserver -s libgazebo_ros_state.so)
  └── /model_states ──► groundtruth_pose.py
                          ├── /groundtruth/pose   (PoseStamped)
                          ├── /groundtruth/odom   (Odometry, pose + twist)
                          └── TF: world -> base_footprint_gt
                                    │
                                    └──► controller_template.py
                                           │  (owns reference_at(t))
                                           ├── /cmd_vel (Twist)
                                           │      └──► Gazebo drive plugin
                                           │           (diff / ackermann / planar)
                                           └── /reference/pose (PoseStamped)

  recorded run ──► trajectory_analyzer.py ──► error metrics
               └─► visualize_trajectory.py ─► reference-vs-actual PNG
```

---

## 5. Error handling

| Condition | Behaviour |
|---|---|
| `model_name` absent from `/model_states` | Log a warning naming the models actually present; publish nothing. Never substitute odometry. |
| `/model_states` never arrives | Warn that `libgazebo_ros_state.so` is probably not loaded; name the fix. |
| Analyzer given an empty or single-point trajectory | Return explicit "insufficient data", not a divide-by-zero or a silent zero. |
| Reference and actual trajectories differ in length | Interpolate onto a common time base; state the assumption in output. |

The unifying rule: a missing signal must be loud. The defect being corrected in
`dev` was silence, not incorrectness.

---

## 6. Testing strategy

ROS2 launch integration is awkward to unit test, so tests split in two. Both
categories are written before the implementation they cover (RED, GREEN,
REFACTOR).

**Unit tests** (pytest, no ROS graph): pose extraction from a synthetic
`ModelStates` message; rate-limiting arithmetic; missing-model handling;
cross-track and heading error maths against hand-computed fixtures; RMSE and
max-error aggregation; empty and single-point trajectory edge cases.

**Integration smoke tests** (bash, headless Gazebo, in the style of the existing
`tests/test_multi_world_launch.sh`): topic presence and rate; TF availability;
controller template producing `/cmd_vel`; Ackermann spawn and turning-radius
behaviour.

### 6.1 Acceptance tests

- **Given** simulation is launched in any of the four worlds, **when** the
  groundtruth node starts, **then** `/groundtruth/pose` publishes the robot's true
  Gazebo pose at 50 Hz.
- **Given** a robot whose model name is not `waffle`, **when** `model_name` is set
  to match, **then** groundtruth tracks that robot instead.
- **Given** the robot model is absent from `/model_states`, **when** the node runs,
  **then** it logs a clear warning and publishes nothing.
- **Given** a student edits only `compute_command()`, **when** they run the
  template, **then** their controller drives the robot with no other file changes.
- **Given** a completed run, **when** the analyzer runs, **then** it reports
  tracking error metrics and the visualizer writes a reference-versus-actual plot.
- **Given** the Ackermann robot, **when** commanded `linear.x = 0` with nonzero
  `angular.z`, **then** it does not rotate in place, confirming genuine car-like
  kinematics.

---

## 7. Success criteria

1. `ros2 topic echo /groundtruth/pose` returns non-empty `PoseStamped` messages in
   all four worlds (landmark, turtlebot3_world, turtlebot3_house, empty_world).
2. `/groundtruth/odom` publishes at the configured rate, measured with
   `ros2 topic hz`, within 10% of the 50 Hz target.
3. `ros2 run tf2_ros tf2_echo world base_footprint_gt` resolves.
4. The `model_name` parameter resolves a non-default model.
5. `controller_template.py` runs unmodified and drives the robot toward a
   reference pose using its shipped P-controller.
6. `trajectory_analyzer.py` emits cross-track error, heading error, RMSE, and max
   error for a recorded run.
7. `visualize_trajectory.py` writes a PNG plotting reference against actual.
8. The Ackermann robot spawns, accepts `Twist` on `/cmd_vel`, and exhibits a
   nonzero turning radius.
9. *(Stretch)* Nav2 plans a feasible path for the Ackermann robot using
   `SmacPlannerHybrid`. Criteria 1-8 and 10-12 are required; this one is
   droppable under schedule pressure, consistent with the risk table in §9.
10. A committed table records pass/fail for every demo: simulation, nav2,
    slam_nav2, slam_cartographer, waypoints, bag record, bag replay, groundtruth,
    control template.
11. `git status` is clean after a bag recording.
12. No documentation link under `docs/` resolves to a missing file.

---

## 8. Implementation phases

Phases are ordered by deliverability. An overrun costs Phase 3, not the task.

### Phase 1 — Verified base with groundtruth (MVP)

Hygiene first, because it is cheap and unblocks clean commits:

- `.gitignore`: `shared/bags/*.db3` becomes `shared/bags/**/*.db3`, plus nested
  metadata
- Delete `shared/tests/test_multi_world_launch.sh`; keep the copy under `tests/`
- `nav2.launch.py`: strip dead `ex3_2_spawn_robot.launch.py` and
  `nav2_params_ex3.yaml` references
- `03_navigation_basics.md:5`: repoint `../docker_setup.md` to `../installation.md`
- Track `replay_bag.launch.py` and `shared/scripts/`

Then the substance: add `-s libgazebo_ros_state.so` to gzserver, write
`groundtruth_pose.py` test-first, and run every existing demo once, recording
pass/fail in a committed table.

Phase 1 alone leaves the repository in a shippable state.

### Phase 2 — Control and evaluation

Write `controller_template.py` with its example P-controller. Cherry-pick and
retarget the two evaluation tools. Add an RViz config showing reference against
groundtruth. Verify the closed loop end to end: template drives the robot,
analyzer scores the run, visualizer plots it.

### Phase 3 — Robot design path and polish

Author the Ackermann reference robot and its `SmacPlannerHybrid` Nav2 variant.
Write `docs/robot_design.md`. Rewrite README around the platform framing rather
than the retired exercise curriculum. Archive and delete the two stale branches.

---

## 9. Risks

| Risk | Severity | Mitigation |
|---|---|---|
| Ten-month dormancy; demos may already be broken | High impact, unknown probability | Verification is scheduled in Phase 1, before new construction depends on it |
| Ackermann plus Nav2 tuning is open-ended | Medium | Ackermann is Phase 3, so a squeeze drops it without harming Phases 1-2. Nav2 support is a stretch goal; spawning and responding to `Twist` is the real requirement |
| Students blocked authoring URDFs | Medium | The shipped Ackermann example is complete and copyable, not a fragment |
| Groundtruth failing silently | Medium, previously realised in `dev` | Node warns and publishes nothing rather than substituting a signal; an acceptance test covers exactly this |
| Schedule — semester within weeks | High | Phases ordered by deliverability; Phase 1 alone is a defensible handover |
| ROS2 Humble currency | Low | Humble reaches EOL in May 2027, valid for this cohort |

---

## 10. Out of scope

- Omni/mecanum reference robot (the recipe covers it; no worked example ships)
- Migration from Humble to Jazzy
- Rewriting exercises 03, 04, 05 — they are not being delivered
- The previously planned 117-test suite

---

## 11. Open questions

- **Semester start date.** Not yet supplied, so Vikunja task #59 carries no
  `start_date` or `due_date` and draws no bar in the Gantt view. Needed to
  schedule the phases.
- **Fate of `docs/exercises/`.** The three existing exercises are not being
  delivered. They can be left in place as reference material, moved to an
  `archive/` subdirectory, or deleted. Left in place for now; the dead link in
  Exercise 3 is fixed either way.
