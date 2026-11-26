#!/usr/bin/env python3
"""Power logger for the ROSbot 2R. Runs on the robot, in a container.

Every PERIOD seconds it writes one sample to /log: the Pi's under-voltage
alarm, the 12 V input as the STM32 reports it on /battery, and whether the
driver is up. Kernel under-voltage events are copied with their kernel time.
It only listens; it never writes to the STM32 or its UART.
"""
import signal
import sys
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState

import records
import store
from hostinfo import boot_id, uptime, wall
from kmsg import Kmsg

PERIOD = 2.0
LOG_DIR = Path("/log")
BATT_TOPIC = "/battery"
DRIVER_NODE = "controller_manager"


def find_uv_alarm() -> Optional[Path]:
    """The Pi's "under-voltage in the last poll" bit, if the kernel has it."""
    for name in Path("/sys/class/hwmon").glob("hwmon*/name"):
        if name.read_text().strip() == "rpi_volt":
            return name.parent / "in0_lcrit_alarm"
    return None


def read_alarm(path: Optional[Path]) -> Optional[bool]:
    if path is None:
        return None
    try:
        return path.read_text().strip() == "1"
    except OSError:
        return None


class PowerLog(Node):
    def __init__(
        self, writer: store.LogWriter, boot: str, kmsg: Kmsg, seen_uv_seq: int
    ):
        super().__init__("powerlog")
        self._writer = writer
        self._boot = boot
        self._kmsg = kmsg
        self._seen_uv_seq = seen_uv_seq
        self._alarm = find_uv_alarm()
        self._batt: Optional[float] = None
        self._batt_up: Optional[float] = None
        self.create_subscription(
            BatteryState, BATT_TOPIC, self._on_battery, qos_profile_sensor_data
        )

    def _on_battery(self, msg: BatteryState) -> None:
        self._batt = msg.voltage
        self._batt_up = uptime()

    def tick(self) -> None:
        for event in records.unseen(self._kmsg.poll(), self._seen_uv_seq):
            self._seen_uv_seq = event["seq"]
            self._writer.append(records.uv_event(self._boot, wall(), event))
        now = uptime()
        age = None if self._batt_up is None else now - self._batt_up
        self._writer.append(
            records.sample(
                self._boot,
                now,
                wall(),
                read_alarm(self._alarm),
                self._batt,
                age,
                DRIVER_NODE in self.get_node_names(),
            )
        )


def seen_uv_seq(boot: str) -> int:
    """Kernel events of this boot already in the log, so a container restart
    does not write them again."""
    active = LOG_DIR / store.ACTIVE
    if not active.exists():
        return -1
    return records.max_uv_seq(store.read_records(active), boot)


def main() -> int:
    stop_signals: List[str] = []

    def on_signal(signum, _frame) -> None:
        stop_signals.append(signal.Signals(signum).name)

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    boot = boot_id()
    tail = store.tail(LOG_DIR)
    seen = seen_uv_seq(boot)
    kmsg = Kmsg()
    with store.LogWriter(LOG_DIR) as writer:
        # Written before ROS starts: a boot is on record even if ROS fails.
        writer.append(records.start(boot, uptime(), wall(), tail, kmsg.available))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = PowerLog(writer, boot, kmsg, seen)
        # Paced by the monotonic clock, not a ROS timer: the robot's wall
        # clock steps by hours when NTP arrives, which would stall a timer.
        # The first sample waits one period so /battery has time to be
        # discovered; otherwise every start would log a false silence.
        next_tick = time.monotonic() + PERIOD
        while not stop_signals:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() >= next_tick:
                node.tick()
                next_tick += PERIOD
        writer.append(records.stop(boot, uptime(), wall(), stop_signals[0]))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
