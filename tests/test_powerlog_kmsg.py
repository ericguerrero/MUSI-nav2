"""Unit tests for robot/powerlog/kmsg.py, with the device replaced by fakes."""
import kmsg

DETECTED = b"2,10,4500000,-;hwmon hwmon1: Undervoltage detected!\n SUBSYSTEM=hwmon\n"
NORMAL = b"6,11,6500000,-;hwmon hwmon1: Voltage normalised\n"
OTHER = b"6,12,7000000,-;usb 1-1: new high-speed USB device\n"


def reader_of(*steps):
    """A fake os.read: returns each bytes step, raises each exception step,
    then reports "nothing more for now" like a non-blocking device."""
    queue = list(steps)

    def read(_fd, _size):
        if not queue:
            raise BlockingIOError
        step = queue.pop(0)
        if isinstance(step, BaseException):
            raise step
        return step

    return read


def opened(*steps):
    return kmsg.Kmsg(opener=lambda _path, _flags: 7, reader=reader_of(*steps))


def test_poll_returns_only_under_voltage_events():
    assert opened(OTHER, DETECTED, NORMAL).poll() == [
        {"seq": 10, "up": 4.5, "low": True},
        {"seq": 11, "up": 6.5, "low": False},
    ]


def test_poll_returns_new_events_only_on_the_next_call():
    source = opened(DETECTED)
    assert len(source.poll()) == 1
    assert source.poll() == []


def test_overwritten_records_do_not_stop_the_read():
    assert len(opened(BrokenPipeError(), DETECTED).poll()) == 1


def test_empty_read_ends_the_poll():
    assert opened(b"", DETECTED).poll() == []


def test_other_device_error_ends_the_poll_without_raising():
    assert len(opened(DETECTED, OSError(22, "Invalid argument"), NORMAL).poll()) == 1


def test_unreadable_device_is_reported_and_polls_empty():
    def refuse(_path, _flags):
        raise PermissionError(1, "Operation not permitted")

    source = kmsg.Kmsg(opener=refuse, reader=reader_of(DETECTED))
    assert source.available is False
    assert source.poll() == []


def test_readable_device_is_reported():
    assert opened().available is True
