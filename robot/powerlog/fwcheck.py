#!/usr/bin/env python3
"""Is the firmware on the STM32 the pinned image? Read it back and compare.

Runs in a throwaway container of the driver image (see fwcheck.sh), with the
driver stopped. It only reads: stm32flash gets -r and nothing that erases,
writes or changes protection. The bootloader is entered with the Husarion
flasher's own pin sequence and always left again, so the firmware restarts.

Exit code: 0 MATCH, 1 MISMATCH, 2 READ FAILED.
"""
import importlib.util
import json
import os
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
EXIT_CODES = {"MATCH": 0, "MISMATCH": 1, "READ FAILED": 2}


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


def check(flasher, ref: bytes, out: Path, run: Callable = subprocess.run) -> dict:
    """Read `len(ref)` bytes back through the bootloader and compare.
    `flasher` gives the port and the bootloader pin sequence."""
    flasher.enter_bootloader_mode()
    try:
        done = run(
            read_command(flasher.port, len(ref), out),
            capture_output=True, text=True, timeout=READ_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired:
        return {"result": "READ FAILED", "detail": f"no answer in {READ_TIMEOUT_S} s"}
    finally:
        flasher.exit_bootloader_mode()
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


def main() -> int:
    result = check(load_flasher(REFERENCE), REFERENCE.read_bytes(), READ_BACK)
    record = {"wall": wall(), "boot": boot_id(), "up": round(uptime(), 1), **result}
    append_result(RESULTS, record)
    print(json.dumps(record))
    print(f"fwcheck: {result['result']}")
    return EXIT_CODES[result["result"]]


if __name__ == "__main__":
    sys.exit(main())
