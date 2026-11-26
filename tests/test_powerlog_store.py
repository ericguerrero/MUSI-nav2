"""Unit tests for robot/powerlog/store.py (pure file I/O, runs off the robot)."""
import store


def test_append_then_read(tmp_path):
    w = store.LogWriter(tmp_path)
    w.append({"a": 1})
    w.append({"a": 2})
    w.close()
    assert list(store.iter_records(tmp_path)) == [{"a": 1}, {"a": 2}]


def test_one_compact_line_per_record(tmp_path):
    with store.LogWriter(tmp_path) as w:
        w.append({"a": 1, "b": None})
    assert (tmp_path / store.ACTIVE).read_text() == '{"a":1,"b":null}\n'


def test_missing_directory_is_created_and_empty_reads(tmp_path):
    d = tmp_path / "log"
    assert list(store.iter_records(d)) == []
    assert store.tail(d) == []
    with store.LogWriter(d) as w:
        w.append({"a": 1})
    assert (d / store.ACTIVE).exists()


def test_half_written_line_is_skipped_and_next_write_is_clean(tmp_path):
    (tmp_path / store.ACTIVE).write_text('{"a": 1}\n{"a": 2, "b"')
    with store.LogWriter(tmp_path) as w:
        w.append({"a": 3})
    assert list(store.iter_records(tmp_path)) == [{"a": 1}, {"a": 3}]


def test_blank_and_non_object_lines_are_skipped(tmp_path):
    (tmp_path / store.ACTIVE).write_text('\n[1]\n"x"\n{"a": 1}\n')
    assert list(store.iter_records(tmp_path)) == [{"a": 1}]


def test_bytes_that_are_not_text_are_skipped(tmp_path):
    (tmp_path / store.ACTIVE).write_bytes(b'{"a": 1}\n\xff\xfe\x00\n{"a": 2}\n')
    assert list(store.iter_records(tmp_path)) == [{"a": 1}, {"a": 2}]


def test_rotation_keeps_order_and_drops_oldest(tmp_path):
    with store.LogWriter(tmp_path, max_bytes=30, keep=3) as w:
        for i in range(20):
            w.append({"i": i})
    files = store.log_files(tmp_path)
    assert len(files) == 3
    assert files[-1].name == store.ACTIVE
    got = [r["i"] for r in store.iter_records(tmp_path)]
    assert got == sorted(got)
    assert got[-1] == 19
    assert got[0] > 0


def test_rotation_continues_across_writers(tmp_path):
    for i in range(6):
        with store.LogWriter(tmp_path, max_bytes=1, keep=4) as w:
            w.append({"i": i})
    assert [r["i"] for r in store.iter_records(tmp_path)] == [2, 3, 4, 5]


def test_tail_returns_last_n(tmp_path):
    with store.LogWriter(tmp_path) as w:
        for i in range(60):
            w.append({"i": i})
    assert [r["i"] for r in store.tail(tmp_path, 5)] == [55, 56, 57, 58, 59]


def test_tail_falls_back_to_rotated_file_when_active_is_empty(tmp_path):
    (tmp_path / "powerlog.1.jsonl").write_text('{"i": 1}\n')
    (tmp_path / store.ACTIVE).write_text("")
    assert store.tail(tmp_path) == [{"i": 1}]


def test_append_reaches_disk_before_returning(tmp_path, monkeypatch):
    synced = []
    real_fsync = store.os.fsync
    monkeypatch.setattr(store.os, "fsync", lambda fd: (synced.append(fd), real_fsync(fd)))
    with store.LogWriter(tmp_path) as w:
        before = len(synced)
        w.append({"a": 1})
        assert len(synced) > before
