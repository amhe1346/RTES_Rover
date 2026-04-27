#!/usr/bin/env python3
"""
Estimate RTES stop-command pipeline latency for MT RTES from measured software latency
plus downstream communication/firmware terms.

Formula:
    T_total = T_detect_to_serial_write + T_uart + T_arduino

Where:
- T_detect_to_serial_write is measured from /tmp/rtes_safety_latency_mt.csv
- T_uart is estimated from baud rate and frame bits
- T_arduino is either measured per-event or a fixed estimate

Usage:
  python3 analyze_full_physical_stop_latency.py /tmp/rtes_safety_latency_mt.csv
  python3 analyze_full_physical_stop_latency.py /tmp/rtes_safety_latency_mt.csv \
      --target-ms 10 --baud 115200 --arduino-us 50
    python3 analyze_full_physical_stop_latency.py /tmp/rtes_safety_latency_mt.csv \
            --measured-file /tmp/stop_measurements.csv
    python3 analyze_full_physical_stop_latency.py /tmp/rtes_safety_latency_mt.csv \
            --arduino-stoplat-file /tmp/arduino_stoplat.log
"""

from __future__ import annotations

import argparse
import csv
import statistics
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
                        "event_type": row["event_type"],
                        "timestamp_us": int(row["timestamp_us"]),
                        "command": row["command"],
                        "bytes_written": int(row["bytes_written"]),
                    }
                )
            except (KeyError, ValueError):
                continue
    return rows


def parse_stoplat_lines(file_path: str):
    """Parse raw Arduino telemetry lines: STOP_LAT,<recv_us>,<applied_us>,<processing_us>."""
    values = []
    with open(file_path, "r", newline="") as handle:
        for line in handle:
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            parts = [p.strip() for p in text.split(",")]
            if len(parts) != 4 or parts[0] != "STOP_LAT":
                continue
            try:
                values.append(float(parts[3]))
            except ValueError:
                continue
    return values


def parse_measured_file(file_path: str):
    """
    Parse measured timing CSV.

    Supported columns (any subset):
    - event_id
    - arduino_us
    - actuator_us (ignored by this script)

    If event_id is missing, rows are applied by order to complete events.
    """
    by_event = {}
    ordered = []

    with open(file_path, "r", newline="") as handle:
        reader = csv.DictReader(row for row in handle if not row.lstrip().startswith("#"))
        for row in reader:
            arduino = None
            actuator = None

            if row.get("arduino_us") not in (None, ""):
                try:
                    arduino = float(row["arduino_us"])
                except ValueError:
                    arduino = None

            if row.get("actuator_us") not in (None, ""):
                try:
                    actuator = float(row["actuator_us"])
                except ValueError:
                    actuator = None

            if arduino is None and actuator is None:
                continue

            event_id_value = row.get("event_id")
            if event_id_value in (None, ""):
                ordered.append({"arduino_us": arduino, "actuator_us": actuator})
                continue

            try:
                event_id = int(event_id_value)
            except ValueError:
                ordered.append({"arduino_us": arduino, "actuator_us": actuator})
                continue

            by_event[event_id] = {"arduino_us": arduino, "actuator_us": actuator}

    return by_event, ordered


def pair_latency_events(rows):
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
    complete = []
    missing = []

    for event_id in event_ids:
        detect = detects[event_id]
        stop = first_stop_write.get(event_id)
        if stop is None:
            missing.append(event_id)
            continue

        complete.append(
            {
                "event_id": event_id,
                "software_us": stop["timestamp_us"] - detect["timestamp_us"],
                "bytes_written": stop["bytes_written"],
            }
        )

    return event_ids, complete, missing


def uart_time_us(command_bytes: int, baud: int, frame_bits: int) -> float:
    if baud <= 0:
        return 0.0
    return (command_bytes * frame_bits * 1_000_000.0) / baud


def summarize(name: str, values):
    if not values:
        print(f"{name:<24} no samples")
        return
    print(f"{name:<24} mean={statistics.mean(values):8.2f} us  min={min(values):8.2f} us  max={max(values):8.2f} us")


def analyze(args) -> int:
    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        print(f"ERROR: File not found: {args.csv_path}")
        return 1

    rows = parse_rows(str(csv_path))
    if not rows:
        print("ERROR: No valid rows found in latency CSV.")
        return 1

    event_ids, complete, missing = pair_latency_events(rows)
    if not event_ids:
        print("ERROR: No emergency detect events found.")
        return 1
    if not complete:
        print("ERROR: No complete detect->STOP pairs found.")
        return 1

    software_us = [item["software_us"] for item in complete]

    # Use configured command byte count (default 1), but if bytes_written is larger,
    # conservatively use observed bytes for that sample.
    uart_us = [
        uart_time_us(max(args.command_bytes, item["bytes_written"]), args.baud, args.frame_bits)
        for item in complete
    ]

    arduino_us = [float(args.arduino_us)] * len(complete)
    applied_measured_arduino = 0

    if args.arduino_stoplat_file:
        stoplat_path = Path(args.arduino_stoplat_file)
        if not stoplat_path.exists():
            print(f"ERROR: STOP_LAT file not found: {args.arduino_stoplat_file}")
            return 1
        stoplat_values = parse_stoplat_lines(str(stoplat_path))
        for idx, value in enumerate(stoplat_values[: len(complete)]):
            arduino_us[idx] = value
            applied_measured_arduino += 1

    if args.measured_file:
        measured_path = Path(args.measured_file)
        if not measured_path.exists():
            print(f"ERROR: Measured file not found: {args.measured_file}")
            return 1

        by_event, ordered = parse_measured_file(str(measured_path))
        order_idx = 0
        for idx, item in enumerate(complete):
            event_id = item["event_id"]
            sample = by_event.get(event_id)
            if sample is None and order_idx < len(ordered):
                sample = ordered[order_idx]
                order_idx += 1

            if sample is None:
                continue

            if sample.get("arduino_us") is not None:
                arduino_us[idx] = sample["arduino_us"]
                applied_measured_arduino += 1

    total_us = [s + u + a for s, u, a in zip(software_us, uart_us, arduino_us)]

    target_us = args.target_ms * 1000.0
    within_target = sum(1 for value in total_us if value <= target_us)

    print("=" * 76)
    print("RTES MT STOP PIPELINE LATENCY ESTIMATE")
    print("=" * 76)
    print(f"File: {args.csv_path}")
    print(f"Emergency detect events: {len(event_ids)}")
    print(f"Complete detect->STOP pairs: {len(complete)}")
    print(f"Missing STOP writes: {len(missing)} ({pct(len(missing), len(event_ids)):.2f}%)")
    print()

    print("Assumptions")
    print("-" * 76)
    print(f"Target budget:          {args.target_ms:.3f} ms")
    print(f"UART baud:             {args.baud} bps")
    print(f"UART frame bits:       {args.frame_bits} bits/byte")
    print(f"STOP command bytes:    {args.command_bytes} byte(s)")
    print(f"Arduino processing:    {args.arduino_us:.2f} us")
    print(f"Measured arduino used: {applied_measured_arduino}/{len(complete)} event(s)")
    if args.arduino_stoplat_file:
        print(f"STOP_LAT file:         {args.arduino_stoplat_file}")
    if args.measured_file:
        print(f"Measured CSV:          {args.measured_file}")
    print()

    print("Component Breakdown")
    print("-" * 76)
    summarize("Software detect->write", software_us)
    summarize("UART transmit", uart_us)
    summarize("Arduino process", arduino_us)
    print()

    print("Total Latency")
    print("-" * 76)
    summarize("Total", total_us)
    print(f"Within target:          {within_target:4d}/{len(total_us)} ({pct(within_target, len(total_us)):.2f}%)")
    print()

    print("Formula")
    print("-" * 76)
    print("T_total = T_detect_to_serial_write + T_uart + T_arduino")

    return 0 if within_target == len(total_us) and not missing else 2


def main() -> int:
    parser = argparse.ArgumentParser(description="Estimate full physical stop latency from MT safety latency events")
    parser.add_argument("csv_path", nargs="?", default="/tmp/rtes_safety_latency_mt.csv")
    parser.add_argument("--target-ms", type=float, default=10.0, help="Target stop latency budget in milliseconds")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--frame-bits", type=int, default=10, help="UART frame bits per byte (e.g. 8N1 => 10)")
    parser.add_argument("--command-bytes", type=int, default=1, help="Expected STOP command bytes written")
    parser.add_argument("--arduino-us", type=float, default=50.0, help="Estimated Arduino processing latency in microseconds")
    parser.add_argument(
        "--arduino-stoplat-file",
        type=str,
        default=None,
        help="Raw Arduino STOP telemetry log with lines like STOP_LAT,<recv_us>,<applied_us>,<processing_us>",
    )
    parser.add_argument(
        "--measured-file",
        type=str,
        default=None,
        help="CSV with measured values using columns: event_id(optional), arduino_us(optional), actuator_us(optional)",
    )
    args = parser.parse_args()
    return analyze(args)


if __name__ == "__main__":
    raise SystemExit(main())
