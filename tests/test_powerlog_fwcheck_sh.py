"""Tests for robot/powerlog/fwcheck.sh with `docker` replaced by a stub.

They pin the order of driver stop, check and driver start on every path,
because a wrong order leaves the robot without a driver or the check racing
the driver for the STM32's UART.
"""
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest

POWERLOG = Path(__file__).parents[1] / "robot" / "powerlog"
PINNED = "husarion/rosbot:humble-0.14.0-20250319"

STUB_DOCKER = """#!/usr/bin/env bash
echo "$*" >> "$DOCKER_LOG"
case "$1" in
    inspect) [ -n "${STUB_DRIVER_IMAGE:-}" ] || exit 1; echo "$STUB_DRIVER_IMAGE" ;;
    ps) [ "${STUB_RUNNING:-0}" = 1 ] && echo abc123 ;;
    run) sleep "${STUB_RUN_SLEEP:-0}"; exit "${STUB_RUN_EXIT:-0}" ;;
    start) exit "${STUB_START_EXIT:-0}" ;;
esac
exit 0
"""


@pytest.fixture
def robot_dir(tmp_path):
    """A copy of what deploy.sh puts on the robot, plus the stub docker."""
    work = tmp_path / "powerlog"
    work.mkdir()
    shutil.copy(POWERLOG / "fwcheck.sh", work)
    shutil.copy(POWERLOG / "compose.yaml", work)
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    stub = stub_bin / "docker"
    stub.write_text(STUB_DOCKER)
    stub.chmod(0o755)
    return work


def environment(robot_dir, **stub):
    env = dict(os.environ)
    env["PATH"] = f"{robot_dir.parent / 'bin'}:{env['PATH']}"
    env["DOCKER_LOG"] = str(robot_dir.parent / "docker.log")
    env.update({key: str(value) for key, value in stub.items()})
    return env


def run_script(robot_dir, **stub):
    done = subprocess.run(
        [str(robot_dir / "fwcheck.sh")], env=environment(robot_dir, **stub),
        capture_output=True, text=True, timeout=20,
    )
    return done, calls(robot_dir)


def calls(robot_dir):
    log = robot_dir.parent / "docker.log"
    return log.read_text().splitlines() if log.exists() else []


def index_of(lines, start):
    return next(i for i, line in enumerate(lines) if line.startswith(start))


def test_running_driver_is_stopped_checked_and_started_again(robot_dir):
    done, lines = run_script(robot_dir, STUB_DRIVER_IMAGE=PINNED, STUB_RUNNING=1)
    assert done.returncode == 0
    order = [index_of(lines, s) for s in ("stop rosbot", "run ", "start rosbot")]
    assert order == sorted(order)


def test_check_runs_the_pinned_image_under_a_fixed_name(robot_dir):
    _done, lines = run_script(robot_dir, STUB_DRIVER_IMAGE=PINNED, STUB_RUNNING=1)
    run = lines[index_of(lines, "run ")]
    assert "--name powerlog-fwcheck" in run
    assert f"{PINNED} python3 /powerlog/fwcheck.py" in run


@pytest.mark.parametrize("code", [1, 2, 3])
def test_exit_code_of_the_check_is_kept_and_driver_still_started(robot_dir, code):
    done, lines = run_script(
        robot_dir, STUB_DRIVER_IMAGE=PINNED, STUB_RUNNING=1, STUB_RUN_EXIT=code
    )
    assert done.returncode == code
    assert "start rosbot" in lines


def test_stopped_driver_is_left_stopped(robot_dir):
    done, lines = run_script(robot_dir, STUB_DRIVER_IMAGE=PINNED, STUB_RUNNING=0)
    assert done.returncode == 0
    assert "stop rosbot" not in lines
    assert "start rosbot" not in lines
    assert any(line.startswith("run ") for line in lines)


def test_missing_driver_container_still_checks_with_the_pinned_image(robot_dir):
    done, lines = run_script(robot_dir)
    assert done.returncode == 0
    assert PINNED in lines[index_of(lines, "run ")]
    assert "start rosbot" not in lines


def test_unreviewed_driver_image_is_refused_before_anything_is_stopped(robot_dir):
    done, lines = run_script(
        robot_dir, STUB_DRIVER_IMAGE="husarion/rosbot:humble-9.9.9", STUB_RUNNING=1
    )
    assert done.returncode == 1
    assert "differs" in done.stderr
    assert not any(line.startswith(("stop", "run", "start")) for line in lines)


def test_failed_driver_start_is_loud_and_keeps_the_verdict(robot_dir):
    done, _lines = run_script(
        robot_dir, STUB_DRIVER_IMAGE=PINNED, STUB_RUNNING=1,
        STUB_RUN_EXIT=2, STUB_START_EXIT=1,
    )
    assert done.returncode == 2
    assert "DRIVER NOT RESTARTED" in done.stderr


def test_killed_script_stops_the_check_before_starting_the_driver(robot_dir):
    env = environment(robot_dir, STUB_DRIVER_IMAGE=PINNED, STUB_RUNNING=1, STUB_RUN_SLEEP=5)
    script = subprocess.Popen(
        [str(robot_dir / "fwcheck.sh")], env=env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    deadline = time.monotonic() + 5
    while not any(line.startswith("run ") for line in calls(robot_dir)):
        assert time.monotonic() < deadline, "check never started"
        time.sleep(0.05)
    script.send_signal(signal.SIGTERM)
    script.communicate(timeout=10)
    lines = calls(robot_dir)
    assert index_of(lines, "stop powerlog-fwcheck") < index_of(lines, "start rosbot")
    assert script.returncode != 0
