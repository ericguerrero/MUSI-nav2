"""Under-voltage events from the kernel ring buffer (/dev/kmsg).

Read non-blocking and from the start of the buffer, so events from before
the logger started are still seen. Standard library only; the device calls
are injectable so the reader is tested off the robot.
"""
import os
from typing import Callable, List, Optional

import records

DEVICE = "/dev/kmsg"
# The kernel returns one record per read and refuses a buffer that is too small.
RECORD_MAX = 8192


class Kmsg:
    def __init__(
        self,
        path: str = DEVICE,
        opener: Callable = os.open,
        reader: Callable = os.read,
    ):
        self._reader = reader
        try:
            self._fd: Optional[int] = opener(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            self._fd = None

    @property
    def available(self) -> bool:
        """False when the kernel log cannot be read: then an empty poll does
        not mean that no under-voltage happened."""
        return self._fd is not None

    def poll(self) -> List[dict]:
        """Under-voltage events that arrived since the last call."""
        events: List[dict] = []
        while self._fd is not None:
            try:
                data = self._reader(self._fd, RECORD_MAX)
            except BrokenPipeError:
                # Records were overwritten before we read them; the kernel
                # has moved us to the oldest one left.
                continue
            except OSError:
                # BlockingIOError: nothing more for now. Anything else: give
                # up this round rather than stop the logger.
                break
            if not data:
                break
            event = records.parse_kmsg(data.decode(errors="replace"))
            if event is not None:
                events.append(event)
        return events
