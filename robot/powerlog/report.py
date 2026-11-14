#!/usr/bin/env python3
"""Print what the power log says about the last boots.

Boots are split by boot ID in file order and timed by `up`. The wall clock
is printed but never used: the robot has no RTC, so it repeats across boots.
Standard library only; runs on the robot or on a copy of the log.

Usage: python3 report.py [LOGDIR] [--boots N]
"""
import argparse
import sys
from pathlib import Path
from typing import Iterable, List, Optional

import store

GAP_S = 10.0


def _new_boot(record: dict) -> dict:
    return {
        "boot": record["boot"],
        "first_wall": record.get("wall"),
        "last_wall": record.get("wall"),
        "up_max": 0.0,
        "end": "open",
        "samples": 0,
        "batt_min": None,
        "batt_max": None,
        "batt_last": None,
        "uv_events": [],
        "no_batt": [],
        "gaps": [],
        "_uv_seqs": set(),
        "_last_sample_up": None,
        "_silent_from": None,
        "_last_type": None,
    }


def _add_sample(boot: dict, record: dict) -> None:
    up = record["up"]
    previous = boot["_last_sample_up"]
    if previous is not None and up - previous > GAP_S:
        boot["gaps"].append((previous, up))
    boot["_last_sample_up"] = up
    boot["samples"] += 1
    batt = record.get("batt")
    if batt is None:
        if boot["_silent_from"] is None:
            boot["_silent_from"] = up
            boot["no_batt"].append((up, up))
        else:
            boot["no_batt"][-1] = (boot["_silent_from"], up)
        return
    boot["_silent_from"] = None
    boot["batt_last"] = batt
    boot["batt_min"] = batt if boot["batt_min"] is None else min(boot["batt_min"], batt)
    boot["batt_max"] = batt if boot["batt_max"] is None else max(boot["batt_max"], batt)


def _add_uv(boot: dict, record: dict) -> None:
    seq = record.get("seq")
    if seq in boot["_uv_seqs"]:
        return
    boot["_uv_seqs"].add(seq)
    boot["uv_events"].append({"up": record["up"], "low": record.get("low")})


def _add(boot: dict, record: dict) -> None:
    boot["last_wall"] = record.get("wall")
    boot["up_max"] = max(boot["up_max"], record.get("up", 0.0))
    boot["_last_type"] = record.get("t")
    if record.get("t") == "s":
        _add_sample(boot, record)
    elif record.get("t") == "uv":
        _add_uv(boot, record)


def summarise(records: Iterable[dict]) -> List[dict]:
    """One summary per boot, in order of first appearance in the log."""
    boots: List[dict] = []
    for record in records:
        if "boot" not in record:
            continue
        if not boots or boots[-1]["boot"] != record["boot"]:
            boots.append(_new_boot(record))
        _add(boots[-1], record)
    for index, boot in enumerate(boots):
        if boot["_last_type"] == "stop":
            boot["end"] = "stop"
        elif index < len(boots) - 1:
            boot["end"] = "cut"
        for key in [k for k in boot if k.startswith("_")]:
            del boot[key]
    return boots


def _duration(seconds: float) -> str:
    whole = int(seconds)
    return f"{whole // 3600}:{whole % 3600 // 60:02d}:{whole % 60:02d}"


def _volts(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.2f}"


def _render_boot(boot: dict) -> List[str]:
    lines = [
        f"boot {boot['boot'][:8]}  {boot['first_wall']} -> {boot['last_wall']}"
        f"  up {_duration(boot['up_max'])}  end: {boot['end']}"
        f"  samples {boot['samples']}",
        f"  battery V  min {_volts(boot['batt_min'])} max {_volts(boot['batt_max'])}"
        f" last {_volts(boot['batt_last'])}",
    ]
    for event in boot["uv_events"]:
        name = "under-voltage" if event["low"] else "voltage normal"
        lines.append(f"  {name} at {event['up']:.1f} s")
    for start, end in boot["no_batt"]:
        lines.append(f"  no /battery from {start:.1f} s to {end:.1f} s")
    for start, end in boot["gaps"]:
        lines.append(f"  no samples from {start:.1f} s to {end:.1f} s")
    return lines


def render(boots: List[dict], last: int = 5) -> str:
    """Text for the last `last` boots, oldest first."""
    if not boots:
        return "no records"
    lines: List[str] = []
    for boot in boots[-last:]:
        lines.extend(_render_boot(boot))
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Summarise the power log per boot.")
    parser.add_argument("logdir", nargs="?", default=Path(__file__).parent / "log", type=Path)
    parser.add_argument("--boots", type=int, default=5, help="how many boots to show")
    args = parser.parse_args(argv)
    print(render(summarise(store.iter_records(args.logdir)), last=args.boots))
    return 0


if __name__ == "__main__":
    sys.exit(main())
