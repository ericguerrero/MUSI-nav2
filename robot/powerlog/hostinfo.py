"""Identity and clocks of the running boot, shared by the logger and fwcheck."""
import time
from datetime import datetime, timezone
from pathlib import Path


def boot_id() -> str:
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def uptime() -> float:
    """Seconds since boot. Unlike the wall clock, it never steps."""
    return time.clock_gettime(time.CLOCK_BOOTTIME)


def wall() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
