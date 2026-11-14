"""Power log file: append-only JSON lines that survive a power cut.

Every record is on disk before `append` returns, so a cut loses at most the
record being written. A cut can leave half a line at the end of the file;
readers skip it and the next writer starts on a fresh line.
Standard library only.
"""
import json
import os
import re
from collections import deque
from pathlib import Path
from typing import BinaryIO, Iterator, List

ACTIVE = "powerlog.jsonl"
_ROTATED = re.compile(r"^powerlog\.(\d+)\.jsonl$")


def _rotated(directory: Path, index: int) -> Path:
    return directory / f"powerlog.{index}.jsonl"


def _sync_dir(directory: Path) -> None:
    """Make a created or renamed file name itself survive a power cut."""
    fd = os.open(directory, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _ends_mid_line(path: Path) -> bool:
    if not path.exists() or path.stat().st_size == 0:
        return False
    with open(path, "rb") as f:
        f.seek(-1, os.SEEK_END)
        return f.read(1) != b"\n"


class LogWriter:
    """Appends records to the active file and rotates it by size.

    `keep` counts all files: the active one plus `keep - 1` rotated ones.
    """

    def __init__(self, directory: Path, max_bytes: int = 20_000_000, keep: int = 10):
        self.directory = Path(directory)
        self.max_bytes = max_bytes
        self.keep = keep
        self.directory.mkdir(parents=True, exist_ok=True)
        self._file = self._open()

    def __enter__(self) -> "LogWriter":
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()

    def _open(self) -> BinaryIO:
        path = self.directory / ACTIVE
        mid_line = _ends_mid_line(path)
        file = open(path, "ab")
        if mid_line:
            file.write(b"\n")
        _sync_dir(self.directory)
        return file

    def _write(self, data: bytes) -> None:
        self._file.write(data)
        self._file.flush()
        os.fsync(self._file.fileno())

    def _rotate(self) -> None:
        self._file.close()
        _rotated(self.directory, self.keep - 1).unlink(missing_ok=True)
        for index in range(self.keep - 2, 0, -1):
            source = _rotated(self.directory, index)
            if source.exists():
                source.rename(_rotated(self.directory, index + 1))
        if self.keep > 1:
            (self.directory / ACTIVE).rename(_rotated(self.directory, 1))
        else:
            (self.directory / ACTIVE).unlink()
        self._file = self._open()

    def append(self, record: dict) -> None:
        if self._file.tell() >= self.max_bytes:
            self._rotate()
        line = json.dumps(record, separators=(",", ":")) + "\n"
        self._write(line.encode())

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()


def log_files(directory: Path) -> List[Path]:
    """Existing log files, oldest first, the active file last."""
    directory = Path(directory)
    if not directory.is_dir():
        return []
    rotated = []
    for path in directory.iterdir():
        match = _ROTATED.match(path.name)
        if match:
            rotated.append((int(match.group(1)), path))
    files = [path for _, path in sorted(rotated, reverse=True)]
    active = directory / ACTIVE
    if active.exists():
        files.append(active)
    return files


def read_records(path: Path) -> Iterator[dict]:
    """Yield the records of one file, skipping lines a power cut damaged."""
    with open(path, "rb") as f:
        for raw in f:
            try:
                record = json.loads(raw.decode())
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(record, dict):
                yield record


def iter_records(directory: Path) -> Iterator[dict]:
    """Yield every record in the directory, oldest first."""
    for path in log_files(directory):
        yield from read_records(path)


def tail(directory: Path, n: int = 50) -> List[dict]:
    """The last `n` records of the newest file that holds any."""
    for path in reversed(log_files(directory)):
        last = deque(read_records(path), maxlen=n)
        if last:
            return list(last)
    return []
