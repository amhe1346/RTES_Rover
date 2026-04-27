#!/usr/bin/env python3
"""
Analyze periodic request and completion timing for MT RTES services.

This script reconstructs request-time and completion-time offsets from the
existing MT performance CSV fields:

- request offset from ideal release = jitter_us
- completion offset from ideal release = jitter_us + exec_us

Outputs:
1) A per-cycle CSV with derived timing values.
2) A human-readable summary report describing predictability and request
   frequency constancy for each service.

Usage:
  python3 analyze_mt_request_completion.py /tmp/rtes_performance_mt.csv
"""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path


THREAD_PERIOD_US = {
    "emergency": 10_000,
    "planning": 20_000,
    "motor": 20_000,
    "sensor": 70_000,
}

THREAD_PRIORITY = {
    "emergency": 99,
    "planning": 90,
    "sensor": 85,
    "motor": 80,
}


def pct(part: int, whole: int) -> float:
    return (100.0 * part / whole) if whole else 0.0


def percentile(values, fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round(fraction * (len(ordered) - 1)))
    idx = max(0, min(idx, len(ordered) - 1))
    return float(ordered[idx])


def parse_rows(csv_path: str):
    rows = []
    with open(csv_path, "r", newline="") as handle:
        reader = csv.DictReader(row for row in handle if not row.lstrip().startswith("#"))
        for row in reader:
            try:
                thread = row["thread"].strip()
                rows.append(
                    {
                        "thread": thread,
                        "cycle": int(row["cycle"]),
                        "exec_us": int(row["exec_us"]),
                        "jitter_us": int(row["jitter_us"]),
                    }
                )
            except (KeyError, ValueError):
                continue
    return rows


def analyze_thread(thread_name: str, rows):
    period_us = THREAD_PERIOD_US.get(thread_name)
    if period_us is None:
        return None

    ordered_rows = sorted(rows, key=lambda item: item["cycle"])
    request_offsets = [item["jitter_us"] for item in ordered_rows]
    response_times = [item["exec_us"] for item in ordered_rows]
    completion_offsets = [item["jitter_us"] + item["exec_us"] for item in ordered_rows]
    deadline_miss = [value > period_us for value in completion_offsets]

    gaps = 0
    max_gap = 0
    backwards = 0
    prev_cycle = None
    for item in ordered_rows:
        cycle = item["cycle"]
        if prev_cycle is not None:
            if cycle < prev_cycle:
                backwards += 1
            elif cycle > prev_cycle + 1:
                gap = cycle - prev_cycle - 1
                gaps += gap
                if gap > max_gap:
                    max_gap = gap
        prev_cycle = cycle

    return {
        "thread": thread_name,
        "priority": THREAD_PRIORITY.get(thread_name, -1),
        "period_us": period_us,
        "count": len(ordered_rows),
        "request_offsets": request_offsets,
        "response_times": response_times,
        "completion_offsets": completion_offsets,
        "deadline_miss_count": sum(1 for miss in deadline_miss if miss),
        "gaps": gaps,
        "max_gap": max_gap,
        "backwards": backwards,
        "rows": ordered_rows,
    }


def write_derived_csv(path: Path, summaries):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "thread",
                "priority",
                "cycle",
                "period_us",
                "request_offset_us",
                "response_time_us",
                "completion_offset_us",
                "deadline_met",
            ]
        )

        for summary in summaries:
            for item in summary["rows"]:
                request_offset = item["jitter_us"]
                response_time = item["exec_us"]
                completion_offset = request_offset + response_time
                deadline_met = completion_offset <= summary["period_us"]
                writer.writerow(
                    [
                        summary["thread"],
                        summary["priority"],
                        item["cycle"],
                        summary["period_us"],
                        request_offset,
                        response_time,
                        completion_offset,
                        1 if deadline_met else 0,
                    ]
                )


def write_report(path: Path, input_csv: str, summaries, emergency_core: int):
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("RTES MT PERIODIC REQUEST/COMPLETION ANALYSIS\n")
        handle.write("=" * 72 + "\n")
        handle.write(f"Input CSV: {input_csv}\n")
        handle.write("Scheduling model: fixed-priority preemptive\n")
        if emergency_core >= 0:
            handle.write(f"Core affinity: emergency thread pinned to core {emergency_core}\n")
        else:
            handle.write("Core affinity: no explicit emergency-thread pinning\n")
        handle.write("\n")

        handle.write("Per-Service Timing\n")
        handle.write("-" * 72 + "\n")
        handle.write(
            "Thread,Priority,Period_us,Cycles,ReqOffsetMean_us,ReqOffsetP95_us,"
            "RespMean_us,RespP95_us,CompOffsetP95_us,MaxCompOffset_us,"
            "DeadlineMisses,MissRate_pct,MissingCycles,MaxGap\n"
        )

        for summary in summaries:
            request_offsets = summary["request_offsets"]
            response_times = summary["response_times"]
            completion_offsets = summary["completion_offsets"]

            count = summary["count"]
            miss = summary["deadline_miss_count"]
            handle.write(
                f"{summary['thread']},{summary['priority']},{summary['period_us']},{count},"
                f"{statistics.mean(request_offsets):.2f},{percentile(request_offsets, 0.95):.2f},"
                f"{statistics.mean(response_times):.2f},{percentile(response_times, 0.95):.2f},"
                f"{percentile(completion_offsets, 0.95):.2f},{max(completion_offsets):.2f},"
                f"{miss},{pct(miss, count):.4f},{summary['gaps']},{summary['max_gap']}\n"
            )

        handle.write("\n")
        handle.write("Predictability Interpretation\n")
        handle.write("-" * 72 + "\n")
        handle.write(
            "Response predictability is indicated by low spread in request offsets\n"
            "(jitter) and completion offsets relative to each thread period.\n"
            "For hard periodic behavior, completion-offset P95 and max should remain\n"
            "well below each service period and deadline misses should remain near zero.\n"
        )
        handle.write("\n")
        handle.write("Request Frequency Constancy\n")
        handle.write("-" * 72 + "\n")
        handle.write(
            "Constancy is inferred by cycle continuity and release jitter stability.\n"
            "MissingCycles and MaxGap close to zero indicate no skipped periodic\n"
            "requests. A tight request-offset distribution indicates regular request\n"
            "frequency around the ideal period.\n"
        )


def print_console_summary(input_csv: str, derived_csv: Path, report_path: Path, summaries):
    print("=" * 76)
    print("RTES MT PERIODIC REQUEST/COMPLETION ANALYSIS")
    print("=" * 76)
    print(f"Input CSV: {input_csv}")
    print(f"Derived per-cycle CSV: {derived_csv}")
    print(f"Text report: {report_path}")
    print()
    print("Thread Summary")
    print("-" * 76)
    print("Thread      Pri  Period(us)  Cycles  ReqP95(us)  RespP95(us)  CompP95(us)  Misses  Gaps")
    for summary in summaries:
        req_p95 = percentile(summary["request_offsets"], 0.95)
        resp_p95 = percentile(summary["response_times"], 0.95)
        comp_p95 = percentile(summary["completion_offsets"], 0.95)
        print(
            f"{summary['thread']:<10}"
            f"{summary['priority']:>4}"
            f"{summary['period_us']:>12}"
            f"{summary['count']:>8}"
            f"{req_p95:>12.2f}"
            f"{resp_p95:>12.2f}"
            f"{comp_p95:>13.2f}"
            f"{summary['deadline_miss_count']:>8}"
            f"{summary['gaps']:>6}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze periodic request/completion timing for MT RTES services"
    )
    parser.add_argument("csv_path", nargs="?", default="/tmp/rtes_performance_mt.csv")
    parser.add_argument(
        "--output-csv",
        default="/tmp/rtes_request_completion_mt.csv",
        help="Output CSV containing per-cycle request/completion timing",
    )
    parser.add_argument(
        "--report-path",
        default="/tmp/rtes_request_completion_report.txt",
        help="Output text report path",
    )
    parser.add_argument(
        "--emergency-core",
        type=int,
        default=2,
        help="Emergency thread CPU core affinity (-1 if not pinned)",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        print(f"ERROR: File not found: {args.csv_path}")
        return 1

    rows = parse_rows(str(csv_path))
    if not rows:
        print("ERROR: No valid rows found in CSV.")
        return 1

    by_thread = {}
    for row in rows:
        by_thread.setdefault(row["thread"], []).append(row)

    summaries = []
    for thread in sorted(THREAD_PERIOD_US):
        thread_rows = by_thread.get(thread)
        if not thread_rows:
            continue
        summary = analyze_thread(thread, thread_rows)
        if summary is not None:
            summaries.append(summary)

    if not summaries:
        print("ERROR: No expected MT thread rows were found.")
        return 1

    derived_csv = Path(args.output_csv)
    report_path = Path(args.report_path)
    write_derived_csv(derived_csv, summaries)
    write_report(report_path, args.csv_path, summaries, args.emergency_core)
    print_console_summary(args.csv_path, derived_csv, report_path, summaries)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
