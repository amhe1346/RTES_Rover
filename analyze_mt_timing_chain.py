#!/usr/bin/env python3
"""
Analyze inter-thread timing alignment for the MT RTES node.

This script reads the latency CSV emitted by realtime_system_node_mt and measures
how a single ultrasonic sensor sample propagates through the software pipeline:

    sensor_update -> emergency detect -> planner update -> motor command write

It groups rows by sample_id and reports stage-to-stage delays, complete chains,
and ordering anomalies such as STOP writes that occur before a planner update.

Usage:
  python3 analyze_mt_timing_chain.py /tmp/rtes_safety_latency_mt.csv
"""

from __future__ import annotations

import csv
import statistics
import sys
from pathlib import Path


def pct(part: int, whole: int) -> float:
    return (100.0 * part / whole) if whole else 0.0


def percentile(sorted_values, fraction: float) -> float:
    if not sorted_values:
        return 0.0
    index = int(round(fraction * (len(sorted_values) - 1)))
    index = max(0, min(index, len(sorted_values) - 1))
    return float(sorted_values[index])


def parse_rows(csv_path: str):
    rows = []
    with open(csv_path, "r", newline="") as handle:
        reader = csv.DictReader(row for row in handle if not row.lstrip().startswith("#"))
        if reader.fieldnames is None or "sample_id" not in reader.fieldnames:
            raise ValueError("latency CSV does not contain a sample_id column")

        for row in reader:
            try:
                rows.append(
                    {
                        "event_id": int(row["event_id"]),
                        "sample_id": int(row["sample_id"]),
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


def first_row(rows, predicate):
    matches = [row for row in rows if predicate(row)]
    if not matches:
        return None
    return min(matches, key=lambda row: row["timestamp_us"])


def summarize_metric(name: str, values):
    if not values:
        print(f"{name:<24} no samples")
        return

    ordered = sorted(values)
    print(
        f"{name:<24} mean={statistics.mean(ordered):8.2f} us  "
        f"median={statistics.median(ordered):8.2f} us  "
        f"p95={percentile(ordered, 0.95):8.2f} us  "
        f"max={max(ordered):8.2f} us"
    )


def analyze(csv_path: str) -> int:
    path = Path(csv_path)
    if not path.exists():
        print(f"ERROR: File not found: {csv_path}")
        return 1

    try:
        rows = parse_rows(csv_path)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        print("Re-run the MT node after the sample_id logging change to collect a compatible latency CSV.")
        return 1

    if not rows:
        print("ERROR: No valid latency rows found.")
        return 1

    samples = {}
    for row in rows:
        sample_id = row["sample_id"]
        if sample_id <= 0:
            continue
        samples.setdefault(sample_id, []).append(row)

    if not samples:
        print("ERROR: No sample-linked timing events found.")
        return 1

    sensor_to_detect = []
    sensor_to_plan = []
    sensor_to_write = []
    detect_to_plan = []
    detect_to_write = []
    plan_to_write = []
    full_chain = []
    write_before_plan = []
    complete_stage_rows = []
    write_source_counts = {}

    for sample_id in sorted(samples):
        sample_rows = samples[sample_id]
        sensor = first_row(sample_rows, lambda row: row["event_type"] == "sensor_update")
        detect = first_row(sample_rows, lambda row: row["event_type"] == "detect")
        plan = first_row(sample_rows, lambda row: row["event_type"] == "plan_update")
        write = first_row(
            sample_rows,
            lambda row: row["event_type"] == "serial_write" and row["bytes_written"] > 0,
        )

        if sensor and detect:
            sensor_to_detect.append(detect["timestamp_us"] - sensor["timestamp_us"])
        if sensor and plan:
            sensor_to_plan.append(plan["timestamp_us"] - sensor["timestamp_us"])
        if sensor and write:
            sensor_to_write.append(write["timestamp_us"] - sensor["timestamp_us"])
        if detect and plan:
            detect_to_plan.append(plan["timestamp_us"] - detect["timestamp_us"])
        if detect and write:
            detect_to_write.append(write["timestamp_us"] - detect["timestamp_us"])
        if plan and write:
            plan_to_write.append(write["timestamp_us"] - plan["timestamp_us"])

        if write:
            source = write["source_thread"]
            write_source_counts[source] = write_source_counts.get(source, 0) + 1

        if sensor and detect and plan and write:
            ordered = (
                sensor["timestamp_us"] <= detect["timestamp_us"] <= plan["timestamp_us"] <= write["timestamp_us"]
            )
            full_latency = write["timestamp_us"] - sensor["timestamp_us"]
            full_chain.append(full_latency)
            complete_stage_rows.append(
                {
                    "sample_id": sample_id,
                    "ordered": ordered,
                    "sensor_ts": sensor["timestamp_us"],
                    "detect_ts": detect["timestamp_us"],
                    "plan_ts": plan["timestamp_us"],
                    "write_ts": write["timestamp_us"],
                    "write_source": write["source_thread"],
                    "write_command": write["command"],
                    "distance_m": sensor["distance_m"],
                    "full_latency_us": full_latency,
                }
            )

            if write["timestamp_us"] < plan["timestamp_us"]:
                write_before_plan.append(sample_id)

    ordered_full_chain = [item["full_latency_us"] for item in complete_stage_rows if item["ordered"]]

    print("=" * 76)
    print("RTES MT INTER-THREAD TIMING ALIGNMENT")
    print("=" * 76)
    print(f"File: {csv_path}")
    print(f"Samples with timing data: {len(samples)}")
    print(f"Samples with detect stage: {len(sensor_to_detect)}")
    print(f"Samples with plan stage:   {len(sensor_to_plan)}")
    print(f"Samples with write stage:  {len(sensor_to_write)}")
    print(f"Complete 4-stage chains:   {len(full_chain)}")
    print(f"Ordered 4-stage chains:    {len(ordered_full_chain)} ({pct(len(ordered_full_chain), len(full_chain)):.2f}%)")
    print(f"Write-before-plan cases:   {len(write_before_plan)}")
    print()

    print("Stage Delays")
    print("-" * 76)
    summarize_metric("sensor->detect", sensor_to_detect)
    summarize_metric("sensor->plan", sensor_to_plan)
    summarize_metric("sensor->write", sensor_to_write)
    summarize_metric("detect->plan", detect_to_plan)
    summarize_metric("detect->write", detect_to_write)
    summarize_metric("plan->write", plan_to_write)
    summarize_metric("sensor->write(full)", full_chain)
    summarize_metric("ordered full chain", ordered_full_chain)
    print()

    print("Write Source")
    print("-" * 76)
    if write_source_counts:
        for source_thread in sorted(write_source_counts):
            count = write_source_counts[source_thread]
            print(f"{source_thread:<20} {count:>8} ({pct(count, len(sensor_to_write)):.2f}%)")
    else:
        print("No motor writes matched to sensor samples.")
    print()

    print("Recent Complete Chains")
    print("-" * 76)
    if complete_stage_rows:
        for item in complete_stage_rows[-5:]:
            print(
                f"sample={item['sample_id']}, ordered={item['ordered']}, "
                f"sensor->write={item['full_latency_us']} us, "
                f"source={item['write_source']}, cmd={item['write_command']}, "
                f"distance={item['distance_m']:.4f} m"
            )
    else:
        print("No complete sensor->detect->plan->write chains found.")
    print()

    print("Interpretation")
    print("-" * 76)
    print("This aligns MT thread activity by shared sensor sample_id rather than cycle number.")
    print("If write-before-plan cases are non-zero, the emergency STOP path is outrunning the")
    print("planner for some samples, which is expected after adding direct STOP from the")
    print("emergency thread.")

    return 0


def main() -> int:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rtes_safety_latency_mt.csv"
    return analyze(csv_path)


if __name__ == "__main__":
    raise SystemExit(main())