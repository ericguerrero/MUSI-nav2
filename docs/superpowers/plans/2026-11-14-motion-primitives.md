# Motion Primitives Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** From a computer, tell the ROSbot 2R "go forward 2 m" or "turn 90°" and have a node generate the `/cmd_vel` commands that do it, closed on odometry. Simulation first.

**Architecture:** A pure core (`motion_primitives_core.py`) turns a pose and a time step into a velocity command: a trapezoidal speed profile on the remaining distance or angle, with heading hold while driving straight. A thin rclpy node (`motion_primitives.py`) feeds it `nav_msgs/Odometry`, publishes `geometry_msgs/Twist`, and owns the safety exits. The core is unit-tested against a simulated unicycle; the node is tested against the ROSbot 2R in Gazebo Fortress.

**Tech Stack:** ROS 2 Humble, rclpy, pytest, bash, Gazebo Fortress through the image `nav2_student:humble`.

**Spec:** No design spec. Sources this plan argues from:
- Eric's request (2026-11-14): from his computer, a controller that can be told "advance straight 2 metres" or "turn so much", generating the `cmd_vel` commands accordingly.
- `docs/rosbot_simulation.md` on branch `feature/rosbot2r-simulation`: topics of the simulated robot.
- `docs/robot_verification.md` on branch `feature/rosbot2r-hw-check`: what the real robot publishes and its limits.

## Global Constraints

- Python only. Files at most 500 lines, functions at most 50 lines. All imports at the top of the file.
- Branch `feature/motion-primitives` from `feature/rosbot2r-simulation` (tip `16e8eb8`): the ROSbot simulation lives there. Create the worktree with EnterWorktree; it branches from `origin/master`, so reset the fresh branch to `feature/rosbot2r-simulation` and rename it.
- **Never run a simulation with host networking on `ROS_DOMAIN_ID` 0.** The real robot is on this machine's network on domain 0 with no namespace: such a simulation would exchange `/cmd_vel` with it. Every test container runs on the default bridge network with `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=77`, as `tests/run_rosbot_sim_contract_test.sh` does.
- Topics, the same in simulation and on the robot: feedback `/odometry/filtered` (`nav_msgs/Odometry`, about 20 Hz), command `/cmd_vel` (`geometry_msgs/Twist`, unstamped).
- The robot's controller stops by itself 0.5 s after the last command (`cmd_vel_timeout`), so the node publishes at 20 Hz and never relies on a final zero command arriving.
- Robot limits are 0.9 m/s and 3.0 rad/s. Defaults here are 0.20 m/s and 0.50 rad/s; parameters above the robot limits are refused.
- Commits are conventional, `type(scope): subject`, with no attribution footer. Each git command is run alone.
- This plan is not run on the physical robot. That is a later step with its own preconditions (see "Out of scope").

## Review Focus

1. **Feedback stops while the robot moves** (Wi-Fi drop, simulator killed). Expected: zero command, exit code 5, within 0.5 s. Task 2 tests the staleness rule; Task 3 tests a run with no feedback at all.
2. **The robot is blocked and never arrives.** Expected: zero command and exit code 4 after the timeout, not an endless push. Task 2 tests the timeout value; Task 1 tests that a blocked simulated robot never reports done.
3. **A turn larger than 180°.** Expected: it turns the commanded way round, all the way. Task 1 tests 270°.
4. **Yaw crossing ±π**, in a turn or while holding heading on a straight. Expected: no jump in the command. Task 1 tests a start yaw of 3.1 rad.
5. **Bad input**: unknown word, missing amount, `nan`, zero, an absurd distance. Expected: a usage message, exit code 2, nothing published. Task 1 tests each.

---

### Task 1: The core

**Files:**
- Create: `ros2_ws/src/motion_primitives_core.py`
- Test: `tests/test_motion_primitives_core.py`

**Interfaces:**
- Produces:

```python
MAX_LINEAR_LIMIT = 0.9
MAX_ANGULAR_LIMIT = 3.0
MAX_DISTANCE = 10.0                  # metres per primitive
MAX_TURN = math.radians(720.0)       # per primitive
DISTANCE_TOLERANCE = 0.01
ANGLE_TOLERANCE = math.radians(1.0)
SETTLED_TICKS = 5
MIN_LINEAR = 0.02                    # keeps the robot moving through its deadband
MIN_ANGULAR = 0.10
HEADING_GAIN = 1.5

class PrimitiveError(ValueError): ...

@dataclass(frozen=True)
class Primitive:
    kind: str       # "forward" or "turn"
    amount: float   # metres, or radians with positive meaning left

@dataclass(frozen=True)
class Limits:
    max_linear: float = 0.20
    max_angular: float = 0.50
    linear_accel: float = 0.30
    angular_accel: float = 1.00

def parse_primitives(words: list[str]) -> list[Primitive]
def normalize_angle(angle: float) -> float
def speed_toward(remaining: float, previous: float, max_speed: float, accel: float, dt: float) -> float
def expected_duration(primitive: Primitive, limits: Limits) -> float

class PrimitiveRunner:
    def __init__(self, primitive: Primitive, limits: Limits) -> None
    def update(self, pose: tuple[float, float, float], dt: float) -> tuple[float, float]
    remaining: float   # property; metres or radians still to go, signed
    done: bool         # property
```

Rules the signatures do not fix:
- `parse_primitives`: words come in pairs, `forward <metres>` and `turn <degrees>`. Raises `PrimitiveError` for an empty list, an odd count, an unknown word, a non-number, `nan`, `inf`, zero, a distance beyond `MAX_DISTANCE` or a turn beyond `MAX_TURN`.
- `Limits` raises `PrimitiveError` in `__post_init__` when a value is not positive or exceeds the robot limit.
- `speed_toward`: the speed has the sign of `remaining`. Its magnitude is `min(max_speed, sqrt(2 * accel * abs(remaining)))`, and it may exceed `abs(previous)` by at most `accel * dt`. Braking is not rate-limited further.
- `expected_duration`: `abs(amount) / max_speed + max_speed / accel`, with the linear or angular pair of limits.
- `PrimitiveRunner.update`: the first call stores the start pose. Forward: progress is the displacement projected on the start heading; linear from `speed_toward`, raised to `MIN_LINEAR` in magnitude while outside tolerance; angular is `HEADING_GAIN * normalize_angle(start_yaw - yaw)`, clamped to `max_angular`. Turn: the turned angle is the sum of `normalize_angle(yaw - previous_yaw)` over the calls; angular from `speed_toward`, raised to `MIN_ANGULAR`; linear 0. `done` becomes true after `abs(remaining)` has been inside the tolerance for `SETTLED_TICKS` consecutive calls; from then on `update` returns `(0.0, 0.0)`.

- [ ] **Step 1: Write the failing tests**

A helper in the test file, `simulate(primitive, limits, start=(0.0, 0.0, 0.0), lag=0.2, dt=0.05, max_time=60.0, blocked=False)`, integrates a unicycle whose velocities follow the command with a first-order lag and calls the runner each step until it is done or `max_time` passes. With `blocked=True` the pose never changes. It returns a `Run` with the fields `pose`, `runner`, `peak_linear`, `peak_angular`, `elapsed`, and `max_progress` (the largest absolute distance from the start seen at any step).

```python
# parse_primitives
assert parse_primitives(["forward", "2.0", "turn", "90"]) == [
    Primitive("forward", 2.0), Primitive("turn", pytest.approx(math.pi / 2))]
assert parse_primitives(["turn", "-45"])[0].amount == pytest.approx(-math.pi / 4)
for bad in ([], ["forward"], ["jump", "1"], ["forward", "abc"], ["forward", "nan"],
            ["forward", "inf"], ["forward", "0"], ["forward", "1000"], ["turn", "100000"]):
    with pytest.raises(PrimitiveError):
        parse_primitives(bad)
# Limits
with pytest.raises(PrimitiveError):
    Limits(max_linear=2.0)
with pytest.raises(PrimitiveError):
    Limits(max_angular=0.0)
# speed_toward
assert speed_toward(2.0, 0.0, 0.2, 0.3, 0.05) == pytest.approx(0.015)      # ramp up
assert speed_toward(2.0, 0.2, 0.2, 0.3, 0.05) == pytest.approx(0.2)        # cruise
assert speed_toward(0.03, 0.2, 0.2, 0.3, 0.05) == pytest.approx(0.1342, abs=1e-3)  # brake
assert speed_toward(-2.0, -0.2, 0.2, 0.3, 0.05) == pytest.approx(-0.2)
# expected_duration
assert expected_duration(Primitive("forward", 2.0), Limits()) == pytest.approx(10.667, abs=1e-3)
# closed loop, forward 2 m: arrives, straight, inside the speed cap, no overshoot beyond 2 cm
run = simulate(Primitive("forward", 2.0), Limits())
assert run.runner.done
assert run.pose[0] == pytest.approx(2.0, abs=0.01) and abs(run.pose[1]) < 0.01
assert run.peak_linear <= 0.2 + 1e-9 and run.elapsed < 20.0
assert run.max_progress <= 2.02
# backward
run = simulate(Primitive("forward", -0.5), Limits())
assert run.runner.done and run.pose[0] == pytest.approx(-0.5, abs=0.01)
# any start pose: 2 m along the start heading
run = simulate(Primitive("forward", 2.0), Limits(), start=(3.0, -2.0, 2.0))
assert math.hypot(run.pose[0] - 3.0, run.pose[1] + 2.0) == pytest.approx(2.0, abs=0.015)
# heading exactly at the wrap: holding it must not flip the command
run = simulate(Primitive("forward", 1.0), Limits(), start=(0.0, 0.0, math.pi))
assert run.runner.done and run.pose[0] == pytest.approx(-1.0, abs=0.01)
# turn 90 degrees left, then 270: the long way round, not the short one
run = simulate(Primitive("turn", math.pi / 2), Limits())
assert run.runner.done and run.pose[2] == pytest.approx(math.pi / 2, abs=math.radians(1.5))
assert run.peak_angular <= 0.5 + 1e-9
run = simulate(Primitive("turn", math.radians(270)), Limits())
assert run.runner.done
assert normalize_angle(run.pose[2] - math.radians(270)) == pytest.approx(0.0, abs=math.radians(1.5))
# yaw wrap: start near pi, turn 20 degrees across it
run = simulate(Primitive("turn", math.radians(20)), Limits(), start=(0.0, 0.0, 3.1))
assert run.runner.done
assert normalize_angle(run.pose[2] - 3.1 - math.radians(20)) == pytest.approx(0.0, abs=math.radians(1.5))
# a blocked robot never reports done
run = simulate(Primitive("forward", 1.0), Limits(), max_time=20.0, blocked=True)
assert not run.runner.done
# once done, the command is zero
run = simulate(Primitive("forward", 0.3), Limits())
assert run.runner.update(run.pose, 0.05) == (0.0, 0.0)
```

- [ ] **Step 2: Run to verify failure**

Run: `./tests/run_unit_tests.sh /tests/test_motion_primitives_core.py`
Expected: FAIL, `ModuleNotFoundError: No module named 'motion_primitives_core'`

- [ ] **Step 3: Implement `ros2_ws/src/motion_primitives_core.py`** to the interface and rules above. No ROS import.

- [ ] **Step 4: Run to verify pass**

Run: `./tests/run_unit_tests.sh`
Expected: every test passes, the existing suites included.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/motion_primitives_core.py tests/test_motion_primitives_core.py
git commit -m "feat(control): add motion primitive core with a trapezoidal profile"
```

### Task 2: The node

**Files:**
- Create: `ros2_ws/src/motion_primitives.py`
- Test: `tests/test_motion_primitives_node.py`

**Interfaces:**
- Consumes: `parse_primitives`, `Primitive`, `Limits`, `PrimitiveRunner`, `PrimitiveError`, `expected_duration`.
- Produces:
  - Command line: `python3 /ros2_ws/src/motion_primitives.py forward 2.0 turn 90 [--ros-args -p max_linear:=0.1]`. Several primitives run in order.
  - Parameters: `feedback_topic` (`/odometry/filtered`), `cmd_vel_topic` (`/cmd_vel`), `max_linear` (0.20), `max_angular` (0.50).
  - `EXIT_OK = 0`, `EXIT_USAGE = 2`, `EXIT_NO_FEEDBACK = 3`, `EXIT_TIMEOUT = 4`, `EXIT_STALE = 5`.
  - `CONTROL_RATE_HZ = 20.0`, `FEEDBACK_WAIT_S = 5.0`, `STALE_AFTER_S = 0.5`.
  - `timeout_for(primitive: Primitive, limits: Limits) -> float`: `3 * expected_duration + 5`.
  - `feedback_is_stale(now: float, last_feedback: float | None) -> bool`: true when there is no feedback yet or it is older than `STALE_AFTER_S`.
  - `split_arguments(argv: list[str]) -> list[str]`: the words left after `rclpy.utilities.remove_ros_args`, without the program name.
  - `main(argv: list[str]) -> int`.

Behaviour of the node:
- Parses the words before creating any publisher. A `PrimitiveError` prints the message and a usage line and returns `EXIT_USAGE`.
- Waits up to `FEEDBACK_WAIT_S` for the first odometry message; without one returns `EXIT_NO_FEEDBACK`.
- Each tick: stale feedback returns `EXIT_STALE`; a primitive older than `timeout_for` returns `EXIT_TIMEOUT`; otherwise it publishes the runner's command.
- After each primitive it prints one line: the primitive, the time taken, and the distance or angle the odometry reports.
- On every way out, a `KeyboardInterrupt` included, it publishes five zero commands.

- [ ] **Step 1: Write the failing tests**

```python
def test_timeout_scales_with_the_primitive():
    assert timeout_for(Primitive("forward", 2.0), Limits()) == pytest.approx(37.0, abs=0.01)

def test_no_feedback_yet_counts_as_stale():
    assert feedback_is_stale(now=10.0, last_feedback=None)

def test_recent_feedback_is_not_stale():
    assert not feedback_is_stale(now=10.0, last_feedback=9.7)

def test_feedback_older_than_half_a_second_is_stale():
    assert feedback_is_stale(now=10.0, last_feedback=9.4)

def test_ros_arguments_are_not_primitives():
    argv = ["motion_primitives.py", "forward", "2.0", "--ros-args", "-p", "max_linear:=0.1"]
    assert split_arguments(argv) == ["forward", "2.0"]

def test_bad_input_is_a_usage_error_before_ros_starts():
    # No rclpy.init has run in this test process: reaching ROS would raise.
    assert main(["motion_primitives.py", "jump", "1"]) == EXIT_USAGE
```

- [ ] **Step 2: Run to verify failure**

Run: `./tests/run_unit_tests.sh /tests/test_motion_primitives_node.py`
Expected: FAIL, `ModuleNotFoundError: No module named 'motion_primitives'`

- [ ] **Step 3: Implement `ros2_ws/src/motion_primitives.py`.**

- [ ] **Step 4: Run to verify pass**

Run: `./tests/run_unit_tests.sh`
Expected: every test passes.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/motion_primitives.py tests/test_motion_primitives_node.py
git commit -m "feat(control): add the motion primitives node and its safety exits"
```

### Task 3: Proof in the ROSbot simulation

**Files:**
- Create: `tests/test_motion_primitives_sim.sh`, `tests/run_motion_primitives_sim_test.sh`

**Interfaces:**
- Consumes: the node's command line and exit codes; `ros2_ws/src/rosbot_simulation.launch.py` from the base branch.
- Produces: `tests/run_motion_primitives_sim_test.sh`, exit code 0 when every check passes, printing one `PASS` or `FAIL` line per check and a final `Result: N failure(s)`.

`run_motion_primitives_sim_test.sh` is `tests/run_rosbot_sim_contract_test.sh` with the last line running the new test: same image, same mounts, default bridge network, `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=77`.

`test_motion_primitives_sim.sh` starts the simulation as the contract test does, with `world:=empty_with_plugins headless:=true`, and waits for "Configured and activated .*rosbot_base_controller". Then, reading the pose from `/odometry/filtered` before and after each run:

| Check | Command | Passes when |
|---|---|---|
| No feedback | `motion_primitives.py forward 1.0 --ros-args -p feedback_topic:=/nothing_here` | exit code 3 within 10 s |
| Usage | `motion_primitives.py jump 1` | exit code 2 |
| Forward | `motion_primitives.py forward 1.0` | exit code 0; odometry moved 1.00 m ± 0.03 |
| Turn | `motion_primitives.py turn 90` | exit code 0; odometry yaw changed 90° ± 3 |
| Backward | `motion_primitives.py forward -0.5` | exit code 0; odometry moved 0.50 m ± 0.03, against the heading |
| Stopped | after the last run | `/odometry/filtered` linear speed below 0.01 m/s two seconds later |
| True motion, forward | Gazebo's own pose of the robot model before and after the forward run | 1.00 m ± 10 % |
| True motion, turn | the same for the turn | 90° ± 20 % |

The true pose comes from Gazebo, not from ROS: find the model name with `ign model --list`, then read it with `ign model -m <name> -p`. The two true-motion thresholds are set before measuring. If one fails, that is a finding about wheel slip in the simulation: report the numbers, do not widen the threshold to make it pass.

- [ ] **Step 1: Write both scripts.**

- [ ] **Step 2: Run the test**

Run: `./tests/run_motion_primitives_sim_test.sh`
Expected: eight `PASS` lines and `Result: 0 failure(s)`. Keep the output: Task 4 records the measured distances and angles.

- [ ] **Step 3: Prove the test can fail**

The script reads `EXPECT_FORWARD_M` (default 1.00), the distance the forward check expects while it still commands 1.0 m. Run: `EXPECT_FORWARD_M=2.0 ./tests/run_motion_primitives_sim_test.sh` (add `-e EXPECT_FORWARD_M` to the runner).
Expected: the forward row reads `FAIL` and the result line is not zero. A check that cannot fail proves nothing.

- [ ] **Step 4: Commit**

```bash
git add tests/test_motion_primitives_sim.sh tests/run_motion_primitives_sim_test.sh
git commit -m "test(control): prove motion primitives on the simulated ROSbot 2R"
```

### Task 4: Document it

**Files:**
- Create: `docs/motion_primitives.md`
- Modify: `README.md` (one line in the documentation list)

- [ ] **Step 1: Write `docs/motion_primitives.md`**: what it does in two sentences; the command line with three examples (`forward 2.0`, `turn 90`, a square as `forward 1 turn 90` four times); the parameters and their defaults; the exit codes; how it stops (zero commands, plus the robot's own 0.5 s timeout); the results table from Task 3 with the measured numbers; and a section "On the real robot" that states it has not been run there and lists what must hold first (see "Out of scope").

- [ ] **Step 2: Check the examples run**

Run each example command from the page inside the simulation container used in Task 3.
Expected: exit code 0 for each.

- [ ] **Step 3: Commit**

```bash
git add docs/motion_primitives.md README.md
git commit -m "docs(control): document motion primitives and their simulation results"
```

---

## Out of scope

- **Running on the physical ROSbot 2R.** It needs, first: ROS 2 discovery between the development machine and the robot checked on purpose (host networking, domain 0, no simulation running anywhere on that network); the wheels off the ground for the first run; the stack started with `~/stack.sh up`. The node's defaults already match the robot's topics.
- Calibrating odometry against a tape measure (`wheel_separation_multiplier`, wheel radius).
- Arcs, paths and trajectories: those belong to `controller_template.py` and the leaderboard stages.
- Stopping for obstacles from `/scan` or the range sensors.
- A ROS 2 action or service interface. The command line is enough for a person at a terminal.
