"""Unit tests for robot/powerlog/records.py (pure, runs off the robot)."""
import pytest

import records


def test_parse_kmsg_detected():
    line = "2,1234,4500000,-;hwmon hwmon1: Undervoltage detected!"
    assert records.parse_kmsg(line) == {"seq": 1234, "up": 4.5, "low": True}


def test_parse_kmsg_normalised():
    line = "6,1240,6500000,-;hwmon hwmon1: Voltage normalised"
    assert records.parse_kmsg(line)["low"] is False


@pytest.mark.parametrize(
    "line", ["6,1,10,-;usb 1-1: new device", " SUBSYSTEM=hwmon", "", "garbage"]
)
def test_parse_kmsg_ignores_other_lines(line):
    assert records.parse_kmsg(line) is None


def test_parse_kmsg_ignores_broken_header_with_matching_text():
    assert records.parse_kmsg("x,y;hwmon hwmon1: Undervoltage detected!") is None


def test_sample_fields():
    assert records.sample("b1", 12.34, "W", False, 12.153, 0.4, True) == {
        "t": "s", "boot": "b1", "up": 12.3, "wall": "W",
        "uv": False, "batt": 12.15, "age": 0.4, "drv": True,
    }


@pytest.mark.parametrize("batt,age", [(12.1, 5.1), (12.1, None), (None, None)])
def test_sample_stale_battery_is_null(batt, age):
    assert records.sample("b1", 1.0, "W", False, batt, age, False)["batt"] is None


def test_sample_keeps_the_age_of_a_stale_reading():
    stale = records.sample("b1", 1.0, "W", False, 12.1, 7.26, True)
    assert (stale["batt"], stale["age"]) == (None, 7.3)


@pytest.mark.parametrize("batt,age", [(None, None), (None, 0.1), (12.1, None)])
def test_sample_without_any_reading_has_no_age(batt, age):
    assert records.sample("b1", 1.0, "W", False, batt, age, False)["age"] is None


def test_sample_keeps_unknown_alarm_as_null():
    assert records.sample("b1", 1.0, "W", None, 12.1, 0.1, True)["uv"] is None


def test_uv_event():
    assert records.uv_event("b1", "W", {"seq": 7, "up": 4.5, "low": True}) == {
        "t": "uv", "boot": "b1", "up": 4.5, "wall": "W", "seq": 7, "low": True,
    }


def test_start_first_run_has_no_prev():
    assert records.start("b1", 20.04, "W", [], True) == {
        "t": "start", "boot": "b1", "up": 20.0, "wall": "W", "kmsg": True,
        "prev": None,
    }


def test_start_records_an_unreadable_kernel_log():
    assert records.start("b1", 20.0, "W", [], False)["kmsg"] is False


def test_start_after_cut_repeats_last_sample():
    tail = [
        records.sample("b0", 100.0, "W0", True, 10.9, 0.1, True),
        records.uv_event("b0", "W0", {"seq": 9, "up": 100.5, "low": True}),
    ]
    assert records.start("b1", 20.0, "W", tail, True)["prev"] == {
        "end": "cut", "boot": "b0", "up": 100.5, "batt": 10.9, "uv": True,
    }


def test_start_after_stop():
    tail = [
        records.sample("b0", 100.0, "W0", False, 12.1, 0.1, True),
        records.stop("b0", 101.0, "W0", "SIGTERM"),
    ]
    assert records.start("b1", 20.0, "W", tail, True)["prev"]["end"] == "stop"


def test_start_same_boot_is_restart_even_without_stop():
    tail = [records.sample("b1", 100.0, "W0", False, 12.1, 0.1, True)]
    assert records.start("b1", 130.0, "W", tail, True)["prev"]["end"] == "restart"


def test_start_prev_without_any_sample():
    tail = [records.stop("b0", 5.0, "W0", "SIGTERM")]
    assert records.start("b1", 20.0, "W", tail, True)["prev"] == {
        "end": "stop", "boot": "b0", "up": 5.0, "batt": None, "uv": None,
    }


def test_start_prev_ignores_samples_of_an_older_boot():
    tail = [
        records.sample("b0", 50.0, "W0", True, 9.0, 0.1, True),
        records.start("b1", 20.0, "W1", [], True),
    ]
    assert records.start("b2", 20.0, "W", tail, True)["prev"] == {
        "end": "cut", "boot": "b1", "up": 20.0, "batt": None, "uv": None,
    }


def test_stop():
    assert records.stop("b1", 9.96, "W", "SIGTERM") == {
        "t": "stop", "boot": "b1", "up": 10.0, "wall": "W", "signal": "SIGTERM",
    }


def test_max_uv_seq():
    recs = [
        records.uv_event("b1", "W", {"seq": 3, "up": 1.0, "low": True}),
        records.uv_event("b0", "W", {"seq": 99, "up": 1.0, "low": True}),
        records.sample("b1", 2.0, "W", False, None, None, False),
    ]
    assert records.max_uv_seq(recs, "b1") == 3
    assert records.max_uv_seq([], "b1") == -1


def test_unseen_keeps_only_events_after_the_last_logged_one():
    events = [{"seq": 3, "up": 1.0, "low": True}, {"seq": 5, "up": 2.0, "low": False}]
    assert records.unseen(events, 3) == [{"seq": 5, "up": 2.0, "low": False}]
    assert records.unseen(events, -1) == events
    assert records.unseen(events, 5) == []
