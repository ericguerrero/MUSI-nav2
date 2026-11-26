#!/usr/bin/env python3
"""Save lidar scans and camera frames to disk as they arrive.

Runs on the robot, next to the sensor drivers, so it needs no network path to
another machine. Every file is flushed to the card as soon as it is complete:
the robot can lose power at any moment and what was captured must survive.
Uses no numpy; the raw image bytes are converted elsewhere.
"""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage

IMAGE_TOPICS = {
    "color": "/camera/color/image_raw",
    "depth": "/camera/depth/image_raw",
}
INFO_TOPICS = {
    "color": "/camera/color/camera_info",
    "depth": "/camera/depth/camera_info",
}


def write_durable(path, data):
    """Write through a temporary name, so a power cut never leaves half a file."""
    mode = "wb" if isinstance(data, bytes) else "w"
    tmp = path + ".tmp"
    with open(tmp, mode) as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.rename(tmp, path)


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Capture(Node):
    def __init__(self, out, scans, frames, frame_period):
        super().__init__("sensor_capture")
        self.out = out
        self.want_scans = scans
        self.want_frames = frames
        self.frame_period = frame_period
        self.scans = []
        self.frames = {name: [] for name in IMAGE_TOPICS}
        self.last_frame = {name: 0.0 for name in IMAGE_TOPICS}
        self.info = {}
        self.static_tf = {}
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        for name, topic in IMAGE_TOPICS.items():
            self.create_subscription(
                Image, topic, lambda msg, n=name: self.on_image(n, msg), qos_profile_sensor_data)
        for name, topic in INFO_TOPICS.items():
            self.create_subscription(
                CameraInfo, topic, lambda msg, n=name: self.on_info(n, msg), qos_profile_sensor_data)
        latched = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self.on_static_tf, latched)

    def done(self):
        return len(self.scans) >= self.want_scans and all(
            len(found) >= self.want_frames for found in self.frames.values())

    def on_scan(self, msg):
        if len(self.scans) >= self.want_scans:
            return
        self.scans.append({
            "stamp": stamp_of(msg.header), "frame_id": msg.header.frame_id,
            "angle_min": msg.angle_min, "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment, "scan_time": msg.scan_time,
            "time_increment": msg.time_increment,
            "range_min": msg.range_min, "range_max": msg.range_max,
            "ranges": [round(r, 4) for r in msg.ranges],
            "intensities": [round(i, 1) for i in msg.intensities],
        })
        count = len(self.scans)
        if count == 1 or count % 10 == 0 or count == self.want_scans:
            write_durable(os.path.join(self.out, "scans.json"), json.dumps(self.scans))
            print(f"scans {count}/{self.want_scans}", flush=True)

    def on_image(self, name, msg):
        now = time.monotonic()
        found = self.frames[name]
        if len(found) >= self.want_frames or now - self.last_frame[name] < self.frame_period:
            return
        self.last_frame[name] = now
        index = len(found)
        write_durable(os.path.join(self.out, f"{name}_{index:03d}.raw"), bytes(msg.data))
        found.append({
            "file": f"{name}_{index:03d}.raw", "stamp": stamp_of(msg.header),
            "frame_id": msg.header.frame_id, "width": msg.width, "height": msg.height,
            "encoding": msg.encoding, "step": msg.step, "is_bigendian": msg.is_bigendian,
        })
        self.write_index()
        print(f"{name} {index + 1}/{self.want_frames} {msg.width}x{msg.height} {msg.encoding}",
              flush=True)

    def on_info(self, name, msg):
        if name in self.info:
            return
        self.info[name] = {
            "width": msg.width, "height": msg.height, "frame_id": msg.header.frame_id,
            "distortion_model": msg.distortion_model,
            "k": list(msg.k), "d": list(msg.d), "p": list(msg.p),
        }
        self.write_index()

    def on_static_tf(self, msg):
        for tf in msg.transforms:
            t, r = tf.transform.translation, tf.transform.rotation
            self.static_tf[tf.child_frame_id] = {
                "parent": tf.header.frame_id,
                "xyz": [t.x, t.y, t.z], "quat_xyzw": [r.x, r.y, r.z, r.w],
            }
        self.write_index()

    def write_index(self):
        index = {"frames": self.frames, "camera_info": self.info, "static_tf": self.static_tf}
        write_durable(os.path.join(self.out, "index.json"), json.dumps(index, indent=1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/out")
    parser.add_argument("--scans", type=int, default=100)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--frame-period", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = Capture(args.out, args.scans, args.frames, args.frame_period)
    print("capture: waiting for topics", flush=True)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not node.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    counts = {name: len(found) for name, found in node.frames.items()}
    print(f"capture: finished, scans {len(node.scans)}, frames {counts}, "
          f"complete {node.done()}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
