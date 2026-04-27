#!/usr/bin/env python3
"""
Analyze end-to-end software safety latency for the multi-threaded RTES node.

This script reads the dedicated latency event CSV emitted by realtime_system_node_mt
and computes the time from emergency detection to the first STOP serial write.

Usage:
  python3 analyze_e2e_safety_latency.py /tmp/rtes_safety_latency_mt.csv
  python3 analyze_e2e_safety_latency.py /tmp/rtes_safety_latency_mt.csv 10000
"""

from __future__ import annotations

import csv
import statistics
import sys
from pathlib import Path


def pct(part: int, whole: int) -> float:
    return (100.0 * part / whole) if whole else 0.0


def parse_rows(csv_path: str):
    rows = []
    with open(csv_path, "r", newline="") as handle:
        reader = csv.DictReader(row for row in handle if not row.lstrip().startswith("#"))
        for row in reader:
            try:
                rows.append(
                    {
                        "event_id": int(row["event_id"]),
                        "source_thread": row["source_thread"],
                        "event_type": row["event_type"],
                        "timestamp_us": int(row["timestamp_us"]),
                        "command": row["command"],
                        "distance_m": float(row["distance_m"]),
                        "ttc": float(row["ttc"]),
                        "bytes_written": int(row["bytes_written"]),
                    }
                )
            except (KeyError, ValueError):
                continue
    return rows


def percentile(sorted_values, fraction: float) -> float:
    if not sorted_values:
        return 0.0
    index = int(round(fraction * (len(sorted_values) - 1)))
    return float(sorted_values[max(0, min(index, len(sorted_values) - 1))])


def analyze(csv_path: str, target_us: int) -> int:
    path = Path(csv_path)
    if not path.exists():
        print(f"ERROR: File not found: {csv_path}")
        return 1

    rows = parse_rows(csv_path)
    if not rows:
        print("ERROR: No valid latency rows found.")
        return 1

    detects = {}
    first_stop_write = {}

    for row in rows:
        event_id = row["event_id"]
        if event_id <= 0:
            continue

        if row["event_type"] == "detect":
            detects.setdefault(event_id, row)
            continue

        if row["event_type"] == "serial_write" and row["command"] == "S" and row["bytes_written"] > 0:
            existing = first_stop_write.get(event_id)
            if existing is None or row["timestamp_us"] < existing["timestamp_us"]:
                first_stop_write[event_id] = row

    event_ids = sorted(detects.keys())
    if not event_ids:
        print("ERROR: No emergency detect events found.")
        return 1

    complete = []
    missing_stop = []
    source_counts = {}

    for event_id in event_ids:
        detect = detects[event_id]
        stop = first_stop_write.get(event_id)
        if stop is None:
            missing_stop.append(event_id)
            continue

        latency_us = stop["timestamp_us"] - detect["timestamp_us"]
        complete.append(
            {
                "event_id": event_id,
                "latency_us": latency_us,
                "detect_ts": detect["timestamp_us"],
                "stop_ts": stop["timestamp_us"],
                "distance_m": detect["distance_m"],
                "ttc": detect["ttc"],
                "source_thread": stop["source_thread"],
            }
        )
        source_counts[stop["source_thread"]] = source_counts.get(stop["source_thread"], 0) + 1

    print("=" * 72)
    print("RTES MT E2E SAFETY LATENCY")
    print("=" * 72)
    print(f"File: {csv_path}")
    print(f"Target latency: {target_us} us ({target_us / 1000.0:.3f} ms)")
    print(f"Emergency detect events: {len(event_ids)}")
    print(f"Completed detect->STOP pairs: {len(complete)}")
    print(f"Missing STOP writes: {len(missing_stop)} ({pct(len(missing_stop), len(event_ids)):.2f}%)")
    print()

    if not complete:
        print("ERROR: No completed detect->STOP pairs found.")
        return 1

    latencies = sorted(item["latency_us"] for item in complete)
    within_target = sum(1 for value in latencies if value <= target_us)

    print("Latency Summary")
    print("-" * 72)
    print(f"Mean:               {statistics.mean(latencies):10.2f} us")
    print(f"Min / Max:          {min(latencies):10d} us / {max(latencies):d} us")
    print(f"Median:             {statistics.median(latencies):10.2f} us")
    print(f"P95:                {percentile(latencies, 0.95):10.2f} us")
    print(f"Within target:      {within_target:10d} / {len(latencies)} ({pct(within_target, len(latencies)):.2f}%)")
    print()

    print("STOP Source Thread")
    print("-" * 72)
    for source_thread in sorted(source_counts):
        count = source_counts[source_thread]
        print(f"{source_thread:<20} {count:>8} ({pct(count, len(complete)):.2f}%)")
    print()

    print("Recent Events")
    print("-" * 72)
    for item in complete[-5:]:
        print(
            f"event={item['event_id']}, latency={item['latency_us']} us, "
            f"distance={item['distance_m']:.4f} m, ttc={item['ttc']:.4f}, "
            f"stop_source={item['source_thread']}"
        )
    print()

    print("Interpretation")
    print("-" * 72)
    print("This proves software-path latency from emergency detection to the first")
    print("successful STOP serial write recorded by the RTES node.")
    print("It does not include downstream UART wire time or Arduino motor actuation time.")

    return 0 if not missing_stop else 2


def main() -> int:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rtes_safety_latency_mt.csv"
    target_us = int(sys.argv[2]) if len(sys.argv) > 2 else 10000
    return analyze(csv_path, target_us)


if __name__ == "__main__":
    raise SystemExit(main())
