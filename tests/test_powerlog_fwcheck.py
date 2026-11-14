"""Unit tests for robot/powerlog/fwcheck.py.

The bootloader pins and stm32flash are replaced by fakes: the tests pin the
order of operations and the read-only command, not the hardware.
"""
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

import fwcheck


class FakeFlasher:
    port = "/dev/ttyFAKE"

    def __init__(self, enter_error=None, exit_errors=()):
        self.calls = []
        self._enter_error = enter_error
        self._exit_errors = list(exit_errors)

    def enter_bootloader_mode(self):
        self.calls.append("enter")
        if self._enter_error is not None:
            raise self._enter_error

    def exit_bootloader_mode(self):
        self.calls.append("exit")
        if self._exit_errors:
            raise self._exit_errors.pop(0)


@pytest.fixture
def restored_signals():
    """fwcheck installs process-wide handlers; put pytest's own back."""
    watched = (signal.SIGTERM, signal.SIGHUP)
    saved = {number: signal.getsignal(number) for number in watched}
    yield
    for number, handler in saved.items():
        signal.signal(number, handler)


@pytest.fixture
def robot_files(tmp_path, monkeypatch):
    """Point fwcheck at temporary files and a fake flasher."""
    reference = tmp_path / "firmware.bin"
    reference.write_bytes(b"\x10\x20\x30")
    flasher = FakeFlasher()
    monkeypatch.setattr(fwcheck, "REFERENCE", reference)
    monkeypatch.setattr(fwcheck, "READ_BACK", tmp_path / "read.bin")
    monkeypatch.setattr(fwcheck, "RESULTS", tmp_path / "log" / "fwcheck.jsonl")
    monkeypatch.setattr(fwcheck, "load_flasher", lambda _reference: flasher)
    return flasher


def fake_stm32flash(flasher, payload=b"", returncode=0, output=""):
    def run(command, **_kwargs):
        flasher.calls.append("read")
        Path(command[command.index("-r") + 1]).write_bytes(payload)
        return subprocess.CompletedProcess(command, returncode, stdout=output, stderr="")

    return run


def test_match():
    assert fwcheck.compare_images(b"\x01\x02\x03", b"\x01\x02\x03") == {"result": "MATCH"}


def test_mismatch_counts_and_locates():
    assert fwcheck.compare_images(b"\x01\xff\x03\xff", b"\x01\x02\x03\x04") == {
        "result": "MISMATCH", "diff_bytes": 2, "first_offset": 1,
        "read_len": 4, "ref_len": 4,
    }


def test_short_read_is_mismatch():
    r = fwcheck.compare_images(b"\x01\x02", b"\x01\x02\x03")
    assert (r["result"], r["diff_bytes"], r["first_offset"], r["read_len"]) == (
        "MISMATCH", 1, 2, 2,
    )


def test_empty_read_is_mismatch():
    assert fwcheck.compare_images(b"", b"\x01")["result"] == "MISMATCH"


def test_module_imports_without_robot_libraries():
    assert "gpiod" not in sys.modules


def test_read_command_is_exact_and_read_only():
    command = fwcheck.read_command("/dev/ttyAMA0", 244712, Path("/tmp/read.bin"))
    assert command == [
        "stm32flash", "-r", "/tmp/read.bin", "-S", "0x08000000:244712",
        "-b", "115200", "/dev/ttyAMA0",
    ]
    assert not {"-w", "-k", "-u", "-e", "-j", "-n", "-o"} & set(command)


def test_check_match_brackets_the_read_with_the_bootloader(tmp_path):
    flasher = FakeFlasher()
    ref = b"\x10\x20\x30"
    result = fwcheck.check(flasher, ref, tmp_path / "read.bin", fake_stm32flash(flasher, ref))
    assert result == {"result": "MATCH"}
    assert flasher.calls == ["enter", "read", "exit"]


def test_check_mismatch(tmp_path):
    flasher = FakeFlasher()
    run = fake_stm32flash(flasher, b"\x10\xff\x30")
    result = fwcheck.check(flasher, b"\x10\x20\x30", tmp_path / "read.bin", run)
    assert (result["result"], result["first_offset"]) == ("MISMATCH", 1)


def test_check_read_failure_reports_last_output_line_and_leaves_bootloader(tmp_path):
    flasher = FakeFlasher()
    run = fake_stm32flash(flasher, returncode=1, output="stm32flash 0.5\nFailed to init device.\n")
    result = fwcheck.check(flasher, b"\x10", tmp_path / "read.bin", run)
    assert result == {"result": "READ FAILED", "detail": "Failed to init device."}
    assert flasher.calls[-1] == "exit"


def test_check_timeout_is_read_failure_and_leaves_bootloader(tmp_path):
    flasher = FakeFlasher()

    def run(command, **_kwargs):
        raise subprocess.TimeoutExpired(command, 1)

    result = fwcheck.check(flasher, b"\x10", tmp_path / "read.bin", run)
    assert result["result"] == "READ FAILED"
    assert flasher.calls == ["enter", "exit"]


def test_check_leaves_bootloader_when_the_tool_is_missing(tmp_path):
    flasher = FakeFlasher()

    def run(command, **_kwargs):
        raise FileNotFoundError("stm32flash")

    with pytest.raises(FileNotFoundError):
        fwcheck.check(flasher, b"\x10", tmp_path / "read.bin", run)
    assert flasher.calls == ["enter", "exit"]


def test_check_leaves_bootloader_when_entering_it_fails(tmp_path):
    flasher = FakeFlasher(enter_error=OSError("gpio busy"))
    with pytest.raises(OSError):
        fwcheck.check(flasher, b"\x10", tmp_path / "read.bin", fake_stm32flash(flasher))
    assert flasher.calls == ["enter", "exit"]


def test_check_leaves_bootloader_when_a_signal_interrupts_the_read(tmp_path):
    flasher = FakeFlasher()

    def run(command, **_kwargs):
        raise fwcheck.Interrupted("SIGTERM")

    with pytest.raises(fwcheck.Interrupted):
        fwcheck.check(flasher, b"\x10", tmp_path / "read.bin", run)
    assert flasher.calls == ["enter", "exit"]


def test_check_repeats_the_exit_when_a_signal_lands_inside_it(tmp_path):
    flasher = FakeFlasher(exit_errors=[fwcheck.Interrupted("SIGHUP")])
    ref = b"\x10\x20"
    result = fwcheck.check(flasher, ref, tmp_path / "read.bin", fake_stm32flash(flasher, ref))
    assert result == {"result": "MATCH"}
    assert flasher.calls == ["enter", "read", "exit", "exit"]


@pytest.mark.parametrize("number", [signal.SIGTERM, signal.SIGHUP])
def test_stop_signals_become_an_exception(restored_signals, number):
    fwcheck.raise_on_signals()
    with pytest.raises(fwcheck.Interrupted):
        os.kill(os.getpid(), number)


def test_no_verdict_shares_the_exit_code_of_a_crash():
    assert fwcheck.EXIT_ERROR == 1
    assert fwcheck.EXIT_CODES == {"MATCH": 0, "MISMATCH": 2, "READ FAILED": 3}


def test_main_match(restored_signals, robot_files, capsys):
    run = fake_stm32flash(robot_files, b"\x10\x20\x30")
    assert fwcheck.main(run) == 0
    assert "fwcheck: MATCH" in capsys.readouterr().out
    saved = json.loads(fwcheck.RESULTS.read_text())
    assert saved["result"] == "MATCH"
    assert {"wall", "boot", "up"} <= set(saved)


def test_main_mismatch(restored_signals, robot_files):
    assert fwcheck.main(fake_stm32flash(robot_files, b"\x10\xff\x30")) == 2


def test_main_read_failed(restored_signals, robot_files):
    assert fwcheck.main(fake_stm32flash(robot_files, returncode=1, output="nack")) == 3


def test_main_that_cannot_run_gives_the_error_code_and_no_result(
    restored_signals, robot_files, monkeypatch, capsys
):
    def broken(_reference):
        raise OSError("gpiochip0 busy")

    monkeypatch.setattr(fwcheck, "load_flasher", broken)
    assert fwcheck.main() == 1
    assert "cannot run" in capsys.readouterr().err
    assert not fwcheck.RESULTS.exists()


def test_main_keeps_the_verdict_when_it_cannot_be_saved(
    restored_signals, robot_files, monkeypatch, tmp_path, capsys
):
    blocker = tmp_path / "blocker"
    blocker.write_text("")
    monkeypatch.setattr(fwcheck, "RESULTS", blocker / "fwcheck.jsonl")
    assert fwcheck.main(fake_stm32flash(robot_files, b"\x10\x20\x30")) == 0
    captured = capsys.readouterr()
    assert "fwcheck: MATCH" in captured.out
    assert "not saved" in captured.err


def test_main_interrupted_gives_the_error_code_and_leaves_bootloader(
    restored_signals, robot_files
):
    def run(command, **_kwargs):
        raise fwcheck.Interrupted("SIGTERM")

    assert fwcheck.main(run) == 1
    assert robot_files.calls == ["enter", "exit"]


def test_append_result_adds_one_line_per_check(tmp_path):
    path = tmp_path / "log" / "fwcheck.jsonl"
    fwcheck.append_result(path, {"result": "MATCH"})
    fwcheck.append_result(path, {"result": "MISMATCH"})
    lines = path.read_text().splitlines()
    assert [json.loads(line)["result"] for line in lines] == ["MATCH", "MISMATCH"]
