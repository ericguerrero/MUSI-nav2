# Class Leaderboard for the Control Project — Design

**Date:** 2026-11-11
**Subject:** 11764, Sensors & Control for Mobile Robots (MUSI, UIB), Eric's block, 2026-27
**Status:** Draft, awaiting Eric's approval
**Builds on:** `2026-09-05-control-platform-design.md` (layers L0 to L4)

Statements marked **(Eric)** were decided by Eric. Statements marked **(call)** are
design choices made while writing this document and are open to correction.

## 1. Intent

The graded control project becomes a staged class leaderboard. Each stage maps to one
course topic, produces a number from the evaluation harness, and feeds the next stage.
Teams present and demonstrate their result live in class.

Why:

- Earlier cohorts chose a robot and configured Nav2 controllers. They compared tools and
  invented little. A fixed task with a fixed metric moves the effort from setup to method.
- A shared metric lets students see where they stand against each other and against
  published baselines, which is what research and development work looks like.
- The strongest results should continue as master theses and reach a congress. The
  course acts as the qualifier; the thesis delivers the paper.

## 2. Goals, non-goals, anti-goals

**Goals**

- One measurable result per topic: kinematics, dynamics, control, and sensing in the loop.
- Every team writes its own method. Nav2 controllers are baselines to beat, not solutions.
- The work runs in simulation first; the same code then runs on the lab's ROSbot 2R.
- The localization and SLAM work from 11765 is reused as the real-robot state source (Eric).
- A visible path from the course project to a thesis and a congress submission.

**Non-goals**

- Entering a 2027 competition as a course objective. No competition with confirmed 2027
  rules accepts the ROSbot 2R (research of 2026-11-11, section 9).
- Requiring a paper or abstract from every team inside the block.
- Adding sensors to the robot. The existing RPLIDAR plus a map covers the missing
  groundtruth. An external reference sensor is a later option, not part of this design.
- An online submission server or automatic grading service.

**Anti-goals** (the design has failed if these happen)

- Rank decides the mark. With four or five teams, rank is noisy and discourages weaker teams.
- Teams overfit to the public tracks and fail on anything else.
- One student carries the team and both receive the same mark.
- Students spend the block on installation, as in 2024-25.

## 3. Constraints

- Block: Thursdays 17:00 to 20:00, 19 November 2026 to about 21 January 2027, about
  eight sessions. Exact dates are not confirmed.
- About 8 to 9 students, the same cohort that has just finished 11765 (Eric).
- 11765 used this repository's environment (Eric), so students already know the Docker
  image, ROS 2 and the localization and SLAM stack of layer L4. Stage 4 plugs in that
  stack; no student-written filter from elsewhere has to be supported.
- Teams of 2, individual grades (Eric).
- Results are shown by presentation and live demo in class (Eric).
- Python only. ROS 2 Humble. The existing Docker image `nav2_student:humble`.
- One ROSbot 2R in the lab. Its simulator migration
  (`2026-09-24-rosbot2r-migration-design.md` on `feature/rosbot2r-migration`) is approved
  and not implemented: on 2026-11-11 that branch holds two documentation commits and no code.
- **The leaderboard targets the ROSbot 2R in simulation and on hardware (Eric).** The
  migration is therefore a prerequisite and is implemented first.
- The platform today is TurtleBot3 Waffle in Gazebo Classic with `/groundtruth/odom`,
  `controller_template.py`, `trajectory_analyzer.py` and `visualize_trajectory.py`.

## 4. Stages

Each stage has a task, a metric the harness computes, and a baseline. Every stage is
scored on public tracks, which students have from the start, and on one hidden track,
revealed on demo day.

| # | Topic | Task | Metric | Baseline |
|---|---|---|---|---|
| 1 | Kinematics | Compute odometry from wheel encoders (`joint_states`) with the team's own forward kinematics | Position drift per metre travelled and final heading error against `/groundtruth/odom` | The simulator's own wheel odometry |
| 2 | Dynamics | Identify a velocity response model (lag, delay, acceleration limits) from logged step and chirp commands | Open-loop prediction error on a held-out command sequence | Ideal model: commanded velocity equals actual velocity |
| 3 | Control | Fill `compute_command()` to follow a path or track a trajectory | Cross-track RMSE and maximum, heading RMSE, completion time, control effort | Nav2 Regulated Pure Pursuit, DWB and MPPI on the same tracks |
| 4 | Localization in the loop | Run the stage 3 controller with feedback from the 11765 localization stack instead of groundtruth | Stage 3 score loss, and localization error against groundtruth | The team's own stage 3 result |

Notes per stage:

- **Stage 1.** Skid-steer first. Calibrating the effective track width against slip is
  the open-ended part. Mecanum kinematics is an extra tier and needs the ROSbot migration.
- **Stage 2.** The identified model is reused in stage 3 as feedforward or as the
  prediction model of a model predictive controller. Dynamics is not yet confirmed in the
  syllabus; if it is dropped, stage 2 becomes an optional tier of stage 3 (call).
- **Stage 3.** At least one method per team member, so each team compares two strategies,
  which keeps the comparison required by the earlier project statement.
- **Stage 4.** Students build the map and tune localization themselves, since they did
  this in 11765. The controller reads the fused `map` to `odom` to `base_link` transform,
  not the raw AMCL pose, which updates at scan rate and jumps. Simulation measures the
  localization error against groundtruth, so students know how far to trust the
  real-robot score.

### 4.1 Scoring and ranking (call)

Each stage has one ranking metric. The other metrics are reported on the board and
discussed in the report, and do not decide rank.

| Stage | Ranking metric (lower is better) | Validity condition |
|---|---|---|
| 1 | Position drift per metre | Run covers the full track |
| 2 | Prediction RMSE of linear and angular velocity on the held-out sequence | Model runs open-loop, with no access to measured velocity |
| 3 | Cross-track RMSE | Track completed within the time limit of that track |
| 4 | Cross-track RMSE measured against groundtruth | Same as stage 3, with feedback from localization |

- Stage 3 is **path following**: the reference is a geometric path and the controller
  chooses its speed, as the Nav2 baselines do. The time limit per track is set from the
  slowest Nav2 baseline plus a margin, so no entry wins by crawling. Trajectory tracking
  with a timed reference remains available in the template as an optional variant and is
  not ranked.
- A team **beats a baseline** when its run is valid and its ranking metric is lower than
  that baseline's on the hidden track.
- "Stage 3 score loss" in stage 4 is the stage 4 ranking metric minus the team's own
  stage 3 ranking metric on the same track.
- Nav2 baselines run with their default parameters for the robot in use, at the same
  maximum speed the student template allows.

### 4.2 Where official runs happen (call)

- Official runs execute on one instructor machine, so the Gazebo real-time factor is the
  same for every team. A runner script takes a team repository and a track, runs the
  stage, and writes the metrics file the board reads.
- Hidden tracks live only on the instructor machine until the demo day.
- Teams still demonstrate on their own laptops during the presentation; that run is not
  the scored one.
- The recorded-run fallback handed in before the session uses the public tracks.

### 4.3 Feedback interface for stage 4 (call)

`feedback_topic` stays a single `nav_msgs/Odometry` parameter. A small adapter node
publishes the `map` to `base_link` pose from TF as Odometry, taking the twist from the
wheel or filtered odometry. Stage 4 then changes one parameter and adds one launch file.

### 4.4 Real robot

- Official ranking is in simulation, where groundtruth exists (call).
- The real ROSbot run is the final-session showcase. It uses the stage 4 configuration:
  the lab is mapped once and the localization pose is both the controller feedback and
  the score reference.
- Localization error is a few centimetres, the same order as the tracking errors being
  ranked. The real-robot board therefore reports coarse results only (completed or not,
  time, approximate error) and does not decide fine ranking (call).

## 5. Demo days and grading

**Demo days (call).** Two in-class sessions rather than four, to protect teaching time:

- Mid-block: stages 1 and 2.
- Final session: stages 3 and 4, plus the real-robot showcase.

On a demo day each team gives a short presentation, then runs its code live on the
hidden track. The score script writes each run's metrics, and a board shown on the
classroom projector ranks the teams and the baselines.

**Individual grades from a team entry (call).**

- One repository and one board entry per team.
- Each stage has a lead, and the lead alternates: one student leads stages 1 and 3, the
  other leads 2 and 4. The lead writes that stage's short report.
- Each student gives a five-minute individual defence per demo day, with questions on
  the partner's stage as well.
- Git history is supporting evidence only.
- With an odd class size, one team of 3 takes a required extra tier (mecanum kinematics
  or a model predictive controller).

**Mark composition per stage (call).**

| Part | Share |
|---|---|
| Method and analysis in the report | 50 % |
| Individual defence | 30 % |
| Result against the baseline | 15 % |
| Rank among teams | 5 % |

Beating a baseline counts for more than rank among classmates. A team with a sound
analysis and a result below the baseline still passes. The weight of this project in the
final mark is assumed to be 50 % and is not confirmed.

## 6. Path to thesis and publication

- Session 1 pitches thesis ideas and names the leaderboard as the entry route (Eric
  wants thesis ideas pitched early; the Umar collaboration is one candidate, details
  still to be supplied by Eric).
- A team that beats all three Nav2 baselines on the hidden track of stage 3 is offered
  continuation as a thesis (call).
- Candidate thesis angle that follows from stage 4: tracking control that uses the
  localization uncertainty.
- Venues checked on 2026-11-11: CASE 2027 work-in-progress papers (1 April 2027) and
  ICARSC 2027 (Felgueiras, 21 to 22 April 2027; deadline not published, 4 February in
  2026) are the realistic first targets. RoMoCo 2027 (13 February 2027) fits the topic
  but needs writing to start in December. Jornadas de Automática 2027 and the ROBOT 2027
  poster track suit the thesis timeline; their dates are not announced.
- Competitions for later cohorts: Sener-CEA's Bot Talent (registration in October) and
  Festival Nacional de Robótica 2027, whose rules for custom robots are not yet verified.

## 7. What has to be built

Left to the builder unless stated:

1. **Tracks.** A small set of public reference tracks and one hidden track per demo day,
   in a format the controller template and the analyzer both read.
2. **Stage metrics.** Extend the analyzer for the metrics in section 4 that it lacks:
   heading error, control effort, completion time, odometry drift per metre, and model
   prediction error. Existing metrics stay as they are.
3. **Feedback switch.** A `feedback_topic` parameter on the controller template, as the
   ROSbot migration spec already defines, so stage 4 changes one parameter.
4. **Baseline runs.** Nav2 Regulated Pure Pursuit, DWB and MPPI run on the same tracks
   and scored by the same analyzer, with results stored in the repository.
   Real-robot bring-up for the showcase is outside this design; it belongs to the
   hardware bring-up spec named in the ROSbot migration design.
5. **Runner and board.** The runner of section 4.2, and a script that reads the metrics files of all teams and baselines and
   renders one ranking page for the projector. A static page is enough.
6. **Stage statements.** One short student-facing statement per stage: task, metric,
   baseline, deliverable.
7. **Tests.** Each added metric has a unit test with a hand-computed case, in line with
   `tests/test_trajectory_analyzer.py`.

Order: the ROSbot 2R simulator migration comes first (Eric), under its own spec and
plan. Work that does not depend on the robot (metrics, tracks format, runner, board,
stage statements) can proceed in parallel on the present simulation (call). Baseline
runs are recorded only on the ROSbot simulation. Within the leaderboard, stage 3 is
built first because it is the core of the project; stages 1, 2 and 4 follow (call).

## 8. Risks

| Risk | Effect | Response |
|---|---|---|
| ROSbot migration not ready by 19 November | Stage 1 cannot start on the target robot | Session 1 is an introduction and needs no robot; stage 3 tooling is robot-agnostic and can be shown on TurtleBot3. Stages 1 and 2 wait for the migration, since they are flat on TurtleBot3 |
| Physical ROSbot unusable (firmware corruption and battery wear reported in 2025-26, not rechecked) | No real-robot showcase | Official ranking is in simulation; check the robot early in the block |
| Four stages do not fit eight sessions | Rushed final stage | Fold stage 2 into stage 3 as an optional tier |
| Baseline papers have no public code | Published numbers cannot be reproduced | Baselines are our own Nav2 runs on our tracks, not the papers' numbers |
| Live demo fails in class | A team has no result | Teams also hand in a recorded run and its metrics file before the session |
| Students tune to the hidden track after seeing it | Later demo day is compromised | A different hidden track for each demo day |

## 9. Research record (2026-11-11)

- The BARN Challenge is not among the accepted ICRA 2027 competitions; its worlds and
  baselines remain usable as a benchmark.
- The RoboRacer Sim Racing League is free and remote but uses its own Ackermann car;
  no 2027 league is announced.
- Reference comparison for the Nav2 baselines: Urrea and Valencia-Aragón, *Systems*
  14(3), 2026, and Macenski et al., *Autonomous Robots*, 2023 (Regulated Pure Pursuit).
  Code availability for the former is not confirmed.

## 10. Open questions for Eric

- Exact session dates, and which two sessions are the demo days.
- Whether dynamics is taught this year (decides whether stage 2 stands alone).
- Project weight in the final mark.
