"""Power log records: what one log line holds and how a run's end is judged.

Pure functions, standard library only, so they are tested off the robot.
Events are ordered by boot ID and `up` (seconds since boot). `wall` is kept
for reading only: the robot has no RTC and its clock is wrong after a boot.
"""
from typing import Iterable, Optional

BATT_MAX_AGE = 5.0

_UV_DETECTED = "Undervoltage detected"
_UV_NORMAL = "Voltage normalised"


def parse_kmsg(line: str) -> Optional[dict]:
    """Return the under-voltage event in one /dev/kmsg record, or None."""
    header, sep, message = line.partition(";")
    if not sep:
        return None
    if _UV_DETECTED in message:
        low = True
    elif _UV_NORMAL in message:
        low = False
    else:
        return None
    fields = header.split(",")
    try:
        return {"seq": int(fields[1]), "up": int(fields[2]) / 1e6, "low": low}
    except (IndexError, ValueError):
        return None


def sample(
    boot: str,
    up: float,
    wall: str,
    uv: Optional[bool],
    batt: Optional[float],
    batt_age: Optional[float],
    drv: bool,
) -> dict:
    """One periodic sample. A battery reading older than BATT_MAX_AGE is null:
    a silent STM32 must not look like a steady voltage."""
    if batt is None or batt_age is None or batt_age > BATT_MAX_AGE:
        fresh_batt = None
    else:
        fresh_batt = round(batt, 2)
    return {
        "t": "s",
        "boot": boot,
        "up": round(up, 1),
        "wall": wall,
        "uv": uv,
        "batt": fresh_batt,
        "drv": drv,
    }


def uv_event(boot: str, wall: str, kmsg: dict) -> dict:
    """A kernel under-voltage event, stamped with the kernel's own time."""
    return {
        "t": "uv",
        "boot": boot,
        "up": kmsg["up"],
        "wall": wall,
        "seq": kmsg["seq"],
        "low": kmsg["low"],
    }


def _previous_end(boot: str, tail: list) -> Optional[dict]:
    if not tail:
        return None
    last = tail[-1]
    if last["boot"] == boot:
        end = "restart"
    elif last["t"] == "stop":
        end = "stop"
    else:
        end = "cut"
    samples = [r for r in tail if r["t"] == "s" and r["boot"] == last["boot"]]
    last_sample = samples[-1] if samples else {}
    return {
        "end": end,
        "boot": last["boot"],
        "up": last["up"],
        "batt": last_sample.get("batt"),
        "uv": last_sample.get("uv"),
    }


def start(boot: str, up: float, wall: str, tail: list) -> dict:
    """The first record of a run. `tail` is the end of the log already on
    disk; `prev` says how the run before this one ended."""
    return {
        "t": "start",
        "boot": boot,
        "up": round(up, 1),
        "wall": wall,
        "prev": _previous_end(boot, tail),
    }


def stop(boot: str, up: float, wall: str, signal: str) -> dict:
    """Written when the logger is told to stop; its absence means a cut."""
    return {
        "t": "stop",
        "boot": boot,
        "up": round(up, 1),
        "wall": wall,
        "signal": signal,
    }


def max_uv_seq(records: Iterable[dict], boot: str) -> int:
    """Highest kernel sequence number already logged for `boot`, or -1."""
    seqs = [r["seq"] for r in records if r.get("t") == "uv" and r.get("boot") == boot]
    return max(seqs, default=-1)
