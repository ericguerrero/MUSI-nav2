"""Unit tests for robot/powerlog/report.py (pure, runs off the robot)."""
import records
import report
import store


def S(boot, up, batt=12.1, uv=False):
    return records.sample(boot, up, "W", uv, batt, 0.1, True)


def test_empty():
    assert report.summarise([]) == []
    assert report.render([]) == "no records"


def test_cut_then_open():
    b = report.summarise([S("b0", 2.0), S("b0", 4.0, batt=11.2), S("b1", 2.0)])
    assert [(x["boot"], x["end"]) for x in b] == [("b0", "cut"), ("b1", "open")]
    assert b[0]["up_max"] == 4.0
    assert (b[0]["batt_min"], b[0]["batt_max"], b[0]["batt_last"]) == (11.2, 12.1, 11.2)
    assert b[0]["samples"] == 2


def test_stop_is_clean_end():
    recs = [S("b0", 2.0), records.stop("b0", 3.0, "W", "SIGTERM"), S("b1", 2.0)]
    assert report.summarise(recs)[0]["end"] == "stop"


def test_last_boot_with_stop_record_is_stop_not_open():
    recs = [S("b0", 2.0), records.stop("b0", 3.0, "W", "SIGTERM")]
    assert report.summarise(recs)[0]["end"] == "stop"


def test_restart_inside_boot_is_one_boot_and_not_a_cut():
    recs = [
        S("b0", 2.0),
        records.stop("b0", 3.0, "W", "SIGTERM"),
        records.start("b0", 30.0, "W", []),
        S("b0", 32.0),
        S("b1", 2.0),
    ]
    b = report.summarise(recs)
    assert len(b) == 2
    assert b[0]["end"] == "cut"
    assert b[0]["gaps"] == [(2.0, 32.0)]


def test_no_battery_whole_boot():
    b = report.summarise([S("b0", 2.0, batt=None), S("b0", 4.0, batt=None)])
    assert b[0]["no_batt"] == [(2.0, 4.0)]
    assert b[0]["batt_min"] is None
    assert " - " in report.render(b)


def test_no_battery_period_inside_boot():
    recs = [S("b0", 2.0), S("b0", 4.0, batt=None), S("b0", 6.0, batt=None), S("b0", 8.0)]
    assert report.summarise(recs)[0]["no_batt"] == [(4.0, 6.0)]


def test_battery_last_is_the_last_reading_before_silence():
    recs = [S("b0", 2.0, batt=11.4), S("b0", 4.0, batt=None)]
    assert report.summarise(recs)[0]["batt_last"] == 11.4


def test_uv_events_deduplicated_by_seq():
    ev = records.uv_event("b0", "W", {"seq": 5, "up": 4.5, "low": True})
    b = report.summarise([S("b0", 2.0), ev, ev])
    assert b[0]["uv_events"] == [{"up": 4.5, "low": True}]


def test_gap_threshold():
    b = report.summarise([S("b0", 2.0), S("b0", 12.0), S("b0", 22.1)])
    assert b[0]["gaps"] == [(12.0, 22.1)]


def test_same_wall_clock_on_every_boot_does_not_merge_boots():
    recs = [
        records.sample(boot, up, "2026-11-13T17:34:50Z", False, 12.0, 0.1, True)
        for boot in ("b0", "b1", "b2")
        for up in (2.0, 4.0)
    ]
    assert [x["boot"] for x in report.summarise(recs)] == ["b0", "b1", "b2"]


def test_wall_clock_going_backwards_does_not_change_duration():
    recs = [
        records.sample("b0", 2.0, "2026-11-14T13:00:00Z", False, 12.0, 0.1, True),
        records.sample("b0", 3600.0, "2026-11-13T17:00:00Z", False, 12.0, 0.1, True),
    ]
    b = report.summarise(recs)
    assert b[0]["up_max"] == 3600.0
    assert "1:00:00" in report.render(b)


def test_records_without_boot_are_ignored():
    assert report.summarise([{"t": "s", "up": 1.0}, {"x": 1}]) == []


def test_render_limits_to_last_boots_and_names_the_end():
    boots = report.summarise([S("b0", 2.0), S("b1", 2.0), S("b2", 2.0)])
    out = report.render(boots, last=2)
    assert "b0" not in out
    assert "b1" in out
    assert "cut" in out
    assert "open" in out


def test_render_lists_events_with_uptime():
    recs = [
        S("b0", 2.0),
        records.uv_event("b0", "W", {"seq": 5, "up": 4.5, "low": True}),
        records.uv_event("b0", "W", {"seq": 6, "up": 6.5, "low": False}),
        S("b0", 20.0, batt=None),
    ]
    out = report.render(report.summarise(recs))
    assert "under-voltage at 4.5 s" in out
    assert "voltage normal at 6.5 s" in out
    assert "no /battery from 20.0 s to 20.0 s" in out
    assert "no samples from 2.0 s to 20.0 s" in out
    assert "12.10" in out


def test_cli_on_empty_directory(tmp_path, capsys):
    assert report.main([str(tmp_path)]) == 0
    assert "no records" in capsys.readouterr().out


def test_cli_reads_log_directory_and_limits_boots(tmp_path, capsys):
    with store.LogWriter(tmp_path) as w:
        for boot in ("b0", "b1", "b2"):
            w.append(S(boot, 2.0))
    assert report.main([str(tmp_path), "--boots", "1"]) == 0
    out = capsys.readouterr().out
    assert "b2" in out
    assert "b1" not in out
