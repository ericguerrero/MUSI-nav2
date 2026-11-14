"""Unit tests for robot/powerlog/fwcheck.py.

The bootloader pins and stm32flash are replaced by fakes: the tests pin the
order of operations and the read-only command, not the hardware.
"""
import json
import subprocess
import sys
from pathlib import Path

import pytest

import fwcheck


class FakeFlasher:
    port = "/dev/ttyFAKE"

    def __init__(self):
        self.calls = []

    def enter_bootloader_mode(self):
        self.calls.append("enter")

    def exit_bootloader_mode(self):
        self.calls.append("exit")


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


@pytest.mark.parametrize(
    "result,code", [("MATCH", 0), ("MISMATCH", 1), ("READ FAILED", 2)]
)
def test_exit_codes(result, code):
    assert fwcheck.EXIT_CODES[result] == code


def test_append_result_adds_one_line_per_check(tmp_path):
    path = tmp_path / "log" / "fwcheck.jsonl"
    fwcheck.append_result(path, {"result": "MATCH"})
    fwcheck.append_result(path, {"result": "MISMATCH"})
    lines = path.read_text().splitlines()
    assert [json.loads(line)["result"] for line in lines] == ["MATCH", "MISMATCH"]
