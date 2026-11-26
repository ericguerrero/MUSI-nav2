# ROSbot 2R Power Log Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A logger on the ROSbot 2R that survives power cuts, a report that reads it, and an on-demand check that the STM32 holds the pinned firmware.

**Architecture:** Pure, standard-library modules (`records.py`, `store.py`, `report.py`) hold every decision and are unit-tested on the development machine. One rclpy node (`powerlog.py`) does the I/O on the robot inside a container of the driver image. `fwcheck` runs in a second, throwaway container and reuses the Husarion flasher's bootloader entry.

**Tech Stack:** Python 3.10 standard library, rclpy (Humble, from the driver image), Docker Compose, `stm32flash` 0.5, pytest 6.

**Spec:** `docs/superpowers/specs/2026-11-14-robot-powerlog-design.md`

## Global Constraints

- All code in `robot/powerlog/`. Deployed to `~/powerlog/` on `husarion@192.168.77.2` as Compose project `powerlog`. Never edit `~/compose.yaml` on the robot.
- Image: `husarion/rosbot:humble-0.14.0-20250319`. The robot has no internet: nothing may need a pull or `pip install`.
- No `sudo` on the robot (it needs a password). Docker works without it.
- `records.py`, `store.py`, `report.py` import the standard library only.
- The logger never writes to the STM32 or its UART. `fwcheck` never passes `-w`, `-k` or `-u` to `stm32flash`.
- Sample period 2 s. `/battery` older than 5 s is recorded as `null`. Rotation at 20 MB, 10 files kept.
- Order events by file order and `up` (seconds since boot). Never sort or subtract by `wall`.
- Limits: file ≤ 500 lines, function ≤ 50 lines. Comments say why, not what.
- Commits: `type(scope): subject`, no AI attribution lines. Do not push.
- Another session uses the robot's driver. Deploying the logger is safe at any time. Run `fwcheck.sh` (Task 5, Step 6) only after Eric says the driver is free.

Facts measured on the robot on 2026-11-14 (do not re-derive):
- Inside a container with `--cap-add SYSLOG --device /dev/kmsg`, `/dev/kmsg` is readable non-blocking from the start of the ring buffer. A record reads `prio,seq,usec,flags;message`.
- `/sys/class/hwmon/hwmon*/name` equal to `rpi_volt` marks the directory that holds `in0_lcrit_alarm` (`0` or `1`).
- `/proc/sys/kernel/random/boot_id` and `time.clock_gettime(time.CLOCK_BOOTTIME)` work in the container.
- `/battery` is `sensor_msgs/msg/BatteryState`, RELIABLE, VOLATILE, 2.5 Hz, published by node `rosbot_ros2_firmware`. The driver container shows node `controller_manager`.
- Kernel messages: `Undervoltage detected!` and `Voltage normalised`.
- In the image: `/flash-firmware.py` defines `FirmwareFlasher(binary_file)` with `.port`, `.enter_bootloader_mode()`, `.exit_bootloader_mode()`; `/root/firmware.bin` is 244712 bytes; flash base address `0x08000000`, baud 115200. The image entrypoint sources ROS.

## Review Focus

1. **A line cut in half by a power loss.** Reading skips it; the next write starts on a fresh line, so the `start` record is never glued to garbage. (Task 2)
2. **First run, empty or missing log directory.** `start` has `prev: null`; the report prints `no records` and exits 0. (Tasks 1, 2, 3)
3. **`/battery` never arrives** (driver down since boot). Samples carry `batt: null`; the report shows the whole boot as without battery and prints `-` for min, max, last. (Tasks 1, 3)
4. **Container restart inside one boot.** `prev.end` is `restart`, not `cut`; under-voltage events are not written twice; the report still counts one boot. (Tasks 1, 3)
5. **Wall clock jumps backwards or repeats across boots.** Boots are split by boot ID in file order; duration comes from `up`. (Task 3)

---

### Task 1: Record building (`records.py`)

**Files:**
- Create: `robot/powerlog/records.py`
- Test: `tests/test_powerlog_records.py`

**Interfaces:**
- Produces:
  - `BATT_MAX_AGE = 5.0`
  - `parse_kmsg(line: str) -> dict | None` returns `{"seq": int, "up": float, "low": bool}` for an under-voltage message, else `None`.
  - `sample(boot: str, up: float, wall: str, uv: bool | None, batt: float | None, batt_age: float | None, drv: bool) -> dict`
  - `uv_event(boot: str, wall: str, kmsg: dict) -> dict`
  - `start(boot: str, up: float, wall: str, tail: list[dict]) -> dict`
  - `stop(boot: str, up: float, wall: str, signal: str) -> dict`
  - `max_uv_seq(records: Iterable[dict], boot: str) -> int` returns `-1` when none.

Tests import with `sys.path.insert(0, str(Path(__file__).parents[1] / "robot" / "powerlog"))`.

- [ ] **Step 1: Write the failing tests**

```python
def test_parse_kmsg_detected():
    assert records.parse_kmsg("2,1234,4500000,-;hwmon hwmon1: Undervoltage detected!") == {"seq": 1234, "up": 4.5, "low": True}

def test_parse_kmsg_normalised():
    assert records.parse_kmsg("6,1240,6500000,-;hwmon hwmon1: Voltage normalised")["low"] is False

@pytest.mark.parametrize("line", ["6,1,10,-;usb 1-1: new device", " SUBSYSTEM=hwmon", "", "garbage"])
def test_parse_kmsg_ignores_other_lines(line):
    assert records.parse_kmsg(line) is None

def test_sample_fields():
    assert records.sample("b1", 12.34, "W", False, 12.153, 0.4, True) == {
        "t": "s", "boot": "b1", "up": 12.3, "wall": "W", "uv": False, "batt": 12.15, "drv": True}

@pytest.mark.parametrize("batt,age", [(12.1, 5.1), (12.1, None), (None, None)])
def test_sample_stale_battery_is_null(batt, age):
    assert records.sample("b1", 1.0, "W", False, batt, age, False)["batt"] is None

def test_uv_event():
    assert records.uv_event("b1", "W", {"seq": 7, "up": 4.5, "low": True}) == {
        "t": "uv", "boot": "b1", "up": 4.5, "wall": "W", "seq": 7, "low": True}

def test_start_first_run_has_no_prev():
    assert records.start("b1", 20.0, "W", [])["prev"] is None

def test_start_after_cut_repeats_last_sample():
    tail = [records.sample("b0", 100.0, "W0", True, 10.9, 0.1, True),
            records.uv_event("b0", "W0", {"seq": 9, "up": 100.5, "low": True})]
    assert records.start("b1", 20.0, "W", tail)["prev"] == {
        "end": "cut", "boot": "b0", "up": 100.5, "batt": 10.9, "uv": True}

def test_start_after_stop():
    tail = [records.sample("b0", 100.0, "W0", False, 12.1, 0.1, True), records.stop("b0", 101.0, "W0", "SIGTERM")]
    assert records.start("b1", 20.0, "W", tail)["prev"]["end"] == "stop"

def test_start_same_boot_is_restart_even_without_stop():
    tail = [records.sample("b1", 100.0, "W0", False, 12.1, 0.1, True)]
    assert records.start("b1", 130.0, "W", tail)["prev"]["end"] == "restart"

def test_start_prev_without_any_sample():
    assert records.start("b1", 20.0, "W", [records.stop("b0", 5.0, "W0", "SIGTERM")])["prev"] == {
        "end": "stop", "boot": "b0", "up": 5.0, "batt": None, "uv": None}

def test_max_uv_seq():
    recs = [records.uv_event("b1", "W", {"seq": 3, "up": 1.0, "low": True}),
            records.uv_event("b0", "W", {"seq": 99, "up": 1.0, "low": True}),
            records.sample("b1", 2.0, "W", False, None, None, False)]
    assert records.max_uv_seq(recs, "b1") == 3
    assert records.max_uv_seq([], "b1") == -1
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_powerlog_records.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'records'`

- [ ] **Step 3: Implement `robot/powerlog/records.py`**

`start` returns `{"t": "start", "boot", "up", "wall", "prev"}`. `prev.up` is the `up` of the last record in `tail`; `prev.batt` and `prev.uv` come from the last `s` record in `tail` whose boot equals the last record's boot. `stop` returns `{"t": "stop", "boot", "up", "wall", "signal"}`. Round `up` to 1 decimal and `batt` to 2.

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest tests/test_powerlog_records.py -q`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add robot/powerlog/records.py tests/test_powerlog_records.py
git commit -m "feat(powerlog): add record building and kernel message parsing"
```

---

### Task 2: Durable log file (`store.py`)

**Files:**
- Create: `robot/powerlog/store.py`
- Test: `tests/test_powerlog_store.py`

**Interfaces:**
- Produces:
  - `ACTIVE = "powerlog.jsonl"`; rotated files are `powerlog.1.jsonl` (newest) to `powerlog.9.jsonl` (oldest).
  - `class LogWriter(directory: Path, max_bytes: int = 20_000_000, keep: int = 10)` with `append(record: dict) -> None` and `close() -> None`.
  - `log_files(directory: Path) -> list[Path]`, oldest first, active file last, existing files only.
  - `read_records(path: Path) -> Iterator[dict]`
  - `iter_records(directory: Path) -> Iterator[dict]`, across `log_files`.
  - `tail(directory: Path, n: int = 50) -> list[dict]`, the last `n` records of the newest non-empty file.

- [ ] **Step 1: Write the failing tests** (all use `tmp_path`)

```python
def test_append_then_read(tmp_path):
    w = store.LogWriter(tmp_path); w.append({"a": 1}); w.append({"a": 2}); w.close()
    assert list(store.iter_records(tmp_path)) == [{"a": 1}, {"a": 2}]

def test_missing_directory_is_created_and_empty_reads(tmp_path):
    d = tmp_path / "log"
    assert list(store.iter_records(d)) == [] and store.tail(d) == []
    store.LogWriter(d).append({"a": 1})
    assert (d / store.ACTIVE).exists()

def test_half_written_line_is_skipped_and_next_write_is_clean(tmp_path):
    (tmp_path / store.ACTIVE).write_text('{"a": 1}\n{"a": 2, "b"')
    w = store.LogWriter(tmp_path); w.append({"a": 3}); w.close()
    assert list(store.iter_records(tmp_path)) == [{"a": 1}, {"a": 3}]

def test_blank_and_non_object_lines_are_skipped(tmp_path):
    (tmp_path / store.ACTIVE).write_text('\n[1]\n"x"\n{"a": 1}\n')
    assert list(store.iter_records(tmp_path)) == [{"a": 1}]

def test_rotation_keeps_order_and_drops_oldest(tmp_path):
    w = store.LogWriter(tmp_path, max_bytes=30, keep=3)
    for i in range(12):
        w.append({"i": i})
    w.close()
    files = store.log_files(tmp_path)
    assert len(files) == 3 and files[-1].name == store.ACTIVE
    got = [r["i"] for r in store.iter_records(tmp_path)]
    assert got == sorted(got) and got[-1] == 11 and got[0] > 0

def test_tail_returns_last_n(tmp_path):
    w = store.LogWriter(tmp_path)
    for i in range(60):
        w.append({"i": i})
    assert [r["i"] for r in store.tail(tmp_path, 5)] == [55, 56, 57, 58, 59]

def test_tail_falls_back_to_rotated_file_when_active_is_empty(tmp_path):
    (tmp_path / "powerlog.1.jsonl").write_text('{"i": 1}\n'); (tmp_path / store.ACTIVE).write_text("")
    assert store.tail(tmp_path) == [{"i": 1}]

def test_append_reaches_disk_before_returning(tmp_path, monkeypatch):
    synced = []
    monkeypatch.setattr(store.os, "fsync", lambda fd: synced.append(fd))
    store.LogWriter(tmp_path).append({"a": 1})
    assert synced
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_powerlog_store.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'store'`

- [ ] **Step 3: Implement `robot/powerlog/store.py`**

`LogWriter` opens the active file in append mode; if the file is not empty and does not end in `\n`, it writes `\n` first. `append` rotates before writing when the active file is at or above `max_bytes`, then writes one compact JSON line, flushes and calls `os.fsync`. Rotation renames from the oldest down and deletes what exceeds `keep` files in total. `read_records` yields only lines that decode to a `dict`.

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest tests/test_powerlog_store.py -q`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add robot/powerlog/store.py tests/test_powerlog_store.py
git commit -m "feat(powerlog): add durable rotating log file"
```

---

### Task 3: Report (`report.py`)

**Files:**
- Create: `robot/powerlog/report.py`
- Test: `tests/test_powerlog_report.py`

**Interfaces:**
- Consumes: `store.iter_records(directory)`; record shapes from Task 1.
- Produces:
  - `GAP_S = 10.0`
  - `summarise(records: Iterable[dict]) -> list[dict]`, one dict per boot in order of first appearance, keys: `boot`, `first_wall`, `last_wall`, `up_max`, `end` (`"stop"`, `"cut"`, or `"open"` for the final boot without a stop record), `samples`, `batt_min`, `batt_max`, `batt_last`, `uv_events` (list of `{"up", "low"}`), `no_batt` (list of `(up_from, up_to)`), `gaps` (list of `(up_from, up_to)`).
  - `render(boots: list[dict], last: int = 5) -> str`
  - CLI: `python3 report.py [LOGDIR] [--boots N]`, default `LOGDIR` is `log` beside the script, default `N` is 5.

- [ ] **Step 1: Write the failing tests**

Helper in the test file: `S(boot, up, batt=12.1, uv=False)` builds a sample through `records.sample(boot, up, "W", uv, batt, 0.1, True)`.

```python
def test_empty():
    assert report.summarise([]) == []
    assert report.render([]) == "no records"

def test_cut_then_open():
    b = report.summarise([S("b0", 2.0), S("b0", 4.0, batt=11.2), S("b1", 2.0)])
    assert [(x["boot"], x["end"]) for x in b] == [("b0", "cut"), ("b1", "open")]
    assert (b[0]["up_max"], b[0]["batt_min"], b[0]["batt_max"], b[0]["batt_last"]) == (4.0, 11.2, 12.1, 11.2)

def test_stop_is_clean_end():
    b = report.summarise([S("b0", 2.0), records.stop("b0", 3.0, "W", "SIGTERM"), S("b1", 2.0)])
    assert b[0]["end"] == "stop"

def test_restart_inside_boot_is_one_boot_and_not_a_cut():
    recs = [S("b0", 2.0), records.stop("b0", 3.0, "W", "SIGTERM"),
            records.start("b0", 30.0, "W", []), S("b0", 32.0), S("b1", 2.0)]
    b = report.summarise(recs)
    assert len(b) == 2 and b[0]["end"] == "cut" and b[0]["gaps"] == [(2.0, 32.0)]

def test_no_battery_whole_boot():
    b = report.summarise([S("b0", 2.0, batt=None), S("b0", 4.0, batt=None)])
    assert b[0]["no_batt"] == [(2.0, 4.0)] and b[0]["batt_min"] is None
    assert " - " in report.render(b)

def test_no_battery_period_inside_boot():
    b = report.summarise([S("b0", 2.0), S("b0", 4.0, batt=None), S("b0", 6.0, batt=None), S("b0", 8.0)])
    assert b[0]["no_batt"] == [(4.0, 6.0)]

def test_uv_events_deduplicated_by_seq():
    ev = records.uv_event("b0", "W", {"seq": 5, "up": 4.5, "low": True})
    assert report.summarise([S("b0", 2.0), ev, ev])[0]["uv_events"] == [{"up": 4.5, "low": True}]

def test_gap_threshold():
    b = report.summarise([S("b0", 2.0), S("b0", 12.0), S("b0", 22.1)])
    assert b[0]["gaps"] == [(12.0, 22.1)]

def test_same_wall_clock_on_every_boot_does_not_merge_boots():
    recs = [records.sample(boot, up, "2026-11-13T17:34:50Z", False, 12.0, 0.1, True)
            for boot in ("b0", "b1", "b2") for up in (2.0, 4.0)]
    assert [x["boot"] for x in report.summarise(recs)] == ["b0", "b1", "b2"]

def test_render_limits_to_last_boots_and_names_the_end():
    boots = report.summarise([S("b0", 2.0), S("b1", 2.0), S("b2", 2.0)])
    out = report.render(boots, last=2)
    assert "b0" not in out and "b1" in out and "cut" in out and "open" in out

def test_cli_on_empty_directory(tmp_path, capsys):
    assert report.main([str(tmp_path)]) == 0
    assert "no records" in capsys.readouterr().out
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_powerlog_report.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'report'`

- [ ] **Step 3: Implement `robot/powerlog/report.py`**

`render` prints, per boot: the first 8 characters of the boot ID, `first_wall` to `last_wall`, duration from `up_max` as `h:mm:ss`, `end`, sample count, battery `min / max / last` in volts with 2 decimals (` - ` when `None`), then one indented line per under-voltage event, no-battery period and gap, each with its uptime in seconds. `main(argv: list[str] | None = None) -> int`.

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest tests/test_powerlog_report.py -q`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add robot/powerlog/report.py tests/test_powerlog_report.py
git commit -m "feat(powerlog): add per-boot report"
```

---

### Task 4: Logger node, Compose service, deploy

**Files:**
- Create: `robot/powerlog/powerlog.py`, `robot/powerlog/compose.yaml`, `robot/powerlog/deploy.sh` (executable)

**Interfaces:**
- Consumes: everything Tasks 1 and 2 produce.
- Produces: container `powerlog` on the robot writing `~/powerlog/log/powerlog.jsonl`; `./robot/powerlog/deploy.sh` (host overridable with `ROBOT_HOST`, default `husarion@192.168.77.2`).

- [ ] **Step 1: Write `robot/powerlog/compose.yaml`**

```yaml
name: powerlog
services:
  powerlog:
    image: husarion/rosbot:humble-0.14.0-20250319
    container_name: powerlog
    network_mode: host
    ipc: host
    restart: unless-stopped
    cap_add: [SYSLOG]
    devices: [/dev/kmsg]
    stop_grace_period: 5s
    volumes:
      - ./:/powerlog:ro
      - ./log:/log
    command: python3 /powerlog/powerlog.py
```

Run: `docker compose -f robot/powerlog/compose.yaml config --quiet && echo OK`
Expected: `OK`

- [ ] **Step 2: Write `robot/powerlog/powerlog.py`**

Constants: `PERIOD = 2.0`, `LOG_DIR = Path("/log")`, `BATT_TOPIC = "/battery"`, `DRIVER_NODE = "controller_manager"`.

- Small readers, one function each: `boot_id() -> str`; `uptime() -> float` from `CLOCK_BOOTTIME`; `wall() -> str` as UTC `YYYY-MM-DDTHH:MM:SSZ`; `uv_alarm() -> bool | None` (`None` when no `rpi_volt` device); `class Kmsg` that opens `/dev/kmsg` with `O_RDONLY | O_NONBLOCK` and whose `poll() -> list[dict]` reads until `BlockingIOError`, continues on `BrokenPipeError` (overwritten records), and returns the `records.parse_kmsg` hits. If `/dev/kmsg` cannot be opened, `poll` returns `[]`.
- `class PowerLog(Node)`: subscribes to `BATT_TOPIC` with `qos_profile_sensor_data`, storing the voltage and its arrival `uptime()`. On construction it reads `store.tail(LOG_DIR)`, computes `seen = records.max_uv_seq(store.read_records(LOG_DIR / store.ACTIVE), boot)` when that file exists, then appends `records.start(...)`. A 2 s timer appends each kmsg event with `seq > seen` (raising `seen`), then one `records.sample(...)` with `drv = DRIVER_NODE in self.get_node_names()`.
- `main()`: `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`; handlers for `SIGTERM` and `SIGINT` record the signal name; loop `rclpy.spin_once(node, timeout_sec=0.2)` until a signal arrives; then append `records.stop(...)` and close the writer.

- [ ] **Step 3: Write `robot/powerlog/deploy.sh`**

`set -euo pipefail`. Create `~/powerlog/log` on the robot, copy `robot/powerlog/*.py`, `*.sh` and `compose.yaml` to `~/powerlog/` with `scp`, then run `cd ~/powerlog && docker compose up -d --force-recreate` over `ssh`. Resolve its own directory so it works from any working directory.

- [ ] **Step 4: Deploy and verify on the robot**

Run: `./robot/powerlog/deploy.sh && sleep 12 && ssh husarion@192.168.77.2 'docker ps --format "{{.Names}} {{.Status}}" | grep powerlog; head -1 ~/powerlog/log/powerlog.jsonl; tail -3 ~/powerlog/log/powerlog.jsonl'`
Expected: `powerlog Up ...`; the first line has `"t":"start"` and `"prev":null`; the last lines have `"t":"s"`, `up` 2 s apart, `"uv":false`, `"drv":true`, and `batt` near 12 V when the driver runs.

If the log holds no kernel events although `ssh husarion@192.168.77.2 'journalctl -k -b 0 | grep -ci voltage'` is above 0, replace `cap_add` and `devices` with `privileged: true` and redeploy.

- [ ] **Step 5: Verify the stop and restart records, and that the driver was not disturbed**

Run: `ssh husarion@192.168.77.2 'cd ~/powerlog && docker compose restart && sleep 8 && grep -E "\"t\":\"(stop|start)\"" log/powerlog.jsonl | tail -2; docker ps --format "{{.Names}} {{.Status}}"; python3 report.py'`
Expected: a `stop` record with `"signal":"SIGTERM"`, then a `start` record with `"end":"restart"`; `rosbot` still `Up` with an age older than this task; the report shows one boot, `open`, with one gap.

- [ ] **Step 6: Commit**

```bash
git add robot/powerlog/powerlog.py robot/powerlog/compose.yaml robot/powerlog/deploy.sh
git commit -m "feat(powerlog): add logger node and robot deployment"
```

---

### Task 5: Firmware check (`fwcheck`)

**Files:**
- Create: `robot/powerlog/fwcheck.py`, `robot/powerlog/fwcheck.sh` (executable), `robot/powerlog/README.md`
- Test: `tests/test_powerlog_fwcheck.py`

**Interfaces:**
- Consumes: `store.LogWriter` is not used here; `fwcheck.py` appends one JSON line to `/log/fwcheck.jsonl` itself.
- Produces:
  - `compare_images(read: bytes, ref: bytes) -> dict`: `{"result": "MATCH"}` or `{"result": "MISMATCH", "diff_bytes": int, "first_offset": int, "read_len": int, "ref_len": int}`.
  - `fwcheck.py` exit codes: 0 `MATCH`, 1 `MISMATCH`, 2 `READ FAILED`.
  - `~/powerlog/fwcheck.sh` on the robot.

- [ ] **Step 1: Write the failing tests**

```python
def test_match():
    assert fwcheck.compare_images(b"\x01\x02\x03", b"\x01\x02\x03") == {"result": "MATCH"}

def test_mismatch_counts_and_locates():
    assert fwcheck.compare_images(b"\x01\xff\x03\xff", b"\x01\x02\x03\x04") == {
        "result": "MISMATCH", "diff_bytes": 2, "first_offset": 1, "read_len": 4, "ref_len": 4}

def test_short_read_is_mismatch():
    r = fwcheck.compare_images(b"\x01\x02", b"\x01\x02\x03")
    assert (r["result"], r["diff_bytes"], r["first_offset"], r["read_len"]) == ("MISMATCH", 1, 2, 2)

def test_empty_read_is_mismatch():
    assert fwcheck.compare_images(b"", b"\x01")["result"] == "MISMATCH"

def test_module_imports_without_robot_libraries():
    assert "gpiod" not in sys.modules
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_powerlog_fwcheck.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'fwcheck'`

- [ ] **Step 3: Implement `robot/powerlog/fwcheck.py`**

`compare_images` counts missing bytes of a short read as differing. `main() -> int`: load `/flash-firmware.py` with `importlib.machinery.SourceFileLoader` (inside `main`, so the module imports anywhere), build `FirmwareFlasher("/root/firmware.bin")`, call `enter_bootloader_mode()`, run `stm32flash -r /tmp/read.bin -S 0x08000000:<ref length> -b 115200 <flasher.port>` with `subprocess.run`, and call `exit_bootloader_mode()` in a `finally`. A non-zero `stm32flash` exit is `{"result": "READ FAILED", "detail": <last output line>}`. Print the result, append it with `wall`, `boot` and `up` to `/log/fwcheck.jsonl` (with `fsync`), return the exit code.

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest tests/ -q -k powerlog`
Expected: all pass

- [ ] **Step 5: Write `robot/powerlog/fwcheck.sh` and `README.md`**

`fwcheck.sh` (`set -euo pipefail`, run on the robot in `~/powerlog`): take the image from `docker inspect -f '{{.Config.Image}}' rosbot` and fail with a clear message if there is no `rosbot` container; remember whether `rosbot` runs; stop it; install an `EXIT` trap that starts it again only if it ran; then `docker run --rm --privileged -v "$PWD":/powerlog:ro -v "$PWD/log":/log "$IMAGE" python3 /powerlog/fwcheck.py`.

`README.md`, under 60 lines: what the logger records and where; `deploy.sh`; how to read `report.py` output and what `stop`, `cut`, `open` and `restart` mean; `fwcheck.sh`, its three results and that the driver is down for about a minute; the limits from the spec.

- [ ] **Step 6: Run on the robot (only after Eric confirms the driver is free)**

Run: `./robot/powerlog/deploy.sh && ssh husarion@192.168.77.2 'cd ~/powerlog && ./fwcheck.sh; echo "exit=$?"; sleep 20; docker ps --format "{{.Names}} {{.Status}}"; tail -1 log/fwcheck.jsonl; docker exec rosbot bash -c "source /opt/ros/humble/setup.bash; timeout 8 ros2 topic echo --once /battery --field voltage"'`
Expected: `MATCH`, `exit=0`, `rosbot Up` a few seconds, a `fwcheck.jsonl` line with `"result":"MATCH"`, and a voltage near 12 V (firmware runs again). During the check the power log shows `"batt":null` and `"drv":false`.

If the result is `READ FAILED`, do not retry with other flags; record the `stm32flash` output and stop.

- [ ] **Step 7: Commit**

```bash
git add robot/powerlog/fwcheck.py robot/powerlog/fwcheck.sh robot/powerlog/README.md tests/test_powerlog_fwcheck.py
git commit -m "feat(powerlog): add on-demand STM32 firmware check"
```
