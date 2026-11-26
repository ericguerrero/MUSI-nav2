#!/usr/bin/env python3
"""Is the firmware on the STM32 the pinned image? Read it back and compare.

Runs in a throwaway container of the driver image (see fwcheck.sh), with the
driver stopped. It only reads: stm32flash gets -r and nothing that erases,
writes or changes protection. The bootloader is entered with the Husarion
flasher's own pin sequence and always left again, so the firmware restarts.

Exit code: 0 MATCH, 2 MISMATCH, 3 READ FAILED, 1 no verdict (could not run).
"""
import importlib.util
import json
import os
import signal
import subprocess
import sys
from importlib.machinery import SourceFileLoader
from pathlib import Path
from typing import Callable, List

from hostinfo import boot_id, uptime, wall

FLASHER = "/flash-firmware.py"
REFERENCE = Path("/root/firmware.bin")
READ_BACK = Path("/tmp/read.bin")
RESULTS = Path("/log/fwcheck.jsonl")
FLASH_BASE = 0x08000000
BAUD = 115200
READ_TIMEOUT_S = 180
# 1 is what Python and the shell return for any crash, so it must never be
# a verdict: a broken check would read as "firmware differs".
EXIT_ERROR = 1
EXIT_CODES = {"MATCH": 0, "MISMATCH": 2, "READ FAILED": 3}


class Interrupted(Exception):
    """A stop signal arrived. Raised so the bootloader is left before exit."""


def raise_on_signals() -> None:
    """Without this, `docker stop` or a closed terminal would end the process
    with the STM32 still held in its bootloader, and the robot would look dead."""

    def interrupt(signum, _frame) -> None:
        raise Interrupted(signal.Signals(signum).name)

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)


def compare_images(read: bytes, ref: bytes) -> dict:
    """Compare the bytes read from the chip with the reference image.
    Bytes missing from a short read count as differing."""
    common = min(len(read), len(ref))
    differing = [i for i in range(common) if read[i] != ref[i]]
    diff_bytes = len(differing) + abs(len(ref) - len(read))
    if diff_bytes == 0:
        return {"result": "MATCH"}
    return {
        "result": "MISMATCH",
        "diff_bytes": diff_bytes,
        "first_offset": differing[0] if differing else common,
        "read_len": len(read),
        "ref_len": len(ref),
    }


def read_command(port: str, length: int, out: Path) -> List[str]:
    return [
        "stm32flash", "-r", str(out), "-S", f"0x{FLASH_BASE:08x}:{length}",
        "-b", str(BAUD), port,
    ]


def _leave_bootloader(flasher) -> None:
    try:
        flasher.exit_bootloader_mode()
    except Interrupted:
        # A second signal landed inside the pin sequence; finish it anyway.
        flasher.exit_bootloader_mode()


def _no_note(_step: str) -> None:
    pass


def check(
    flasher,
    ref: bytes,
    out: Path,
    run: Callable = subprocess.run,
    note: Callable[[str], None] = _no_note,
) -> dict:
    """Read `len(ref)` bytes back through the bootloader and compare.
    `flasher` gives the port and the bootloader pin sequence. The bootloader
    is left whatever happens, including a failure half way into entering it.
    `note` is told each step: if the Pi resets during the check, the last
    note on disk says how far it got."""
    try:
        note("entering bootloader")
        flasher.enter_bootloader_mode()
        done = run(
            read_command(flasher.port, len(ref), out),
            capture_output=True, text=True, timeout=READ_TIMEOUT_S,
        )
        note("read finished")
    except subprocess.TimeoutExpired:
        return {"result": "READ FAILED", "detail": f"no answer in {READ_TIMEOUT_S} s"}
    finally:
        # Nothing may come before this: a note that fails or is interrupted
        # must not keep the chip in its bootloader.
        _leave_bootloader(flasher)
        note("left bootloader")
    if done.returncode != 0:
        lines = (done.stdout + done.stderr).strip().splitlines()
        return {"result": "READ FAILED", "detail": lines[-1] if lines else ""}
    return compare_images(out.read_bytes(), ref)


def append_result(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "ab") as f:
        f.write((json.dumps(record, separators=(",", ":")) + "\n").encode())
        f.flush()
        os.fsync(f.fileno())


def load_flasher(reference: Path):
    """Husarion's flasher, for its pins and port. Loaded by path because the
    file name is not a module name and it exists only in the driver image."""
    loader = SourceFileLoader("husarion_flasher", FLASHER)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    if spec is None:
        raise ImportError(f"cannot load {FLASHER}")
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module.FirmwareFlasher(str(reference))


def _stamped(fields: dict) -> dict:
    return {"wall": wall(), "boot": boot_id(), "up": round(uptime(), 1), **fields}


def save_step(step: str) -> None:
    """Steps are evidence for a reset, not part of the check: a step that
    cannot be saved must not stop it."""
    try:
        append_result(RESULTS, _stamped({"step": step}))
    except OSError:
        pass


def main(run: Callable = subprocess.run) -> int:
    raise_on_signals()
    try:
        flasher = load_flasher(REFERENCE)
        result = check(flasher, REFERENCE.read_bytes(), READ_BACK, run, save_step)
    except (OSError, ImportError, AttributeError, TypeError, Interrupted) as error:
        print(f"fwcheck: cannot run, no verdict: {error!r}", file=sys.stderr)
        return EXIT_ERROR
    record = _stamped(result)
    # Printed before it is saved: a full card must not cost the verdict.
    print(json.dumps(record))
    print(f"fwcheck: {result['result']}")
    try:
        append_result(RESULTS, record)
    except OSError as error:
        print(f"fwcheck: result not saved: {error!r}", file=sys.stderr)
    return EXIT_CODES[result["result"]]


if __name__ == "__main__":
    sys.exit(main())
