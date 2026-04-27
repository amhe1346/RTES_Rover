#!/usr/bin/env python3
"""
Analyze multi-threaded RTES CSV logs (thread,cycle,exec_us,jitter_us,distance_m,emergency,ttc).



Usage:
cd /home/trashbot/ROS2_Deploy/RTES/scripts
python3 analyze_mt_performance.py /tmp/rtes_performance_mt.csv
"""

from __future__ import annotations

import csv
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def safe_float(value: str) -> float:
    if value.lower() in {"inf", "+inf", "infinity"}:
        return math.inf
    return float(value)


def pct(part: int, whole: int) -> float:
    return (100.0 * part / whole) if whole else 0.0


def analyze(csv_path: str) -> int:
    path = Path(csv_path)
    if not path.exists():
        print(f"ERROR: File not found: {csv_path}")
        return 1

    rows = []
    with path.open("r", newline="") as f:
        # Skip summary lines that start with '#'
        reader = csv.DictReader(line for line in f if not line.lstrip().startswith("#"))
        for row in reader:
            try:
                rows.append(
                    {
                        "thread": row["thread"],
                        "cycle": int(row["cycle"]),
                        "exec_us": int(row["exec_us"]),
                        "jitter_us": int(row["jitter_us"]),
                        "distance_m": float(row["distance_m"]),
                        "emergency": int(row["emergency"]),
                        "ttc": safe_float(row["ttc"]),
                    }
                )
            except (KeyError, ValueError):
                # Skip malformed rows
                continue

    if not rows:
        print("ERROR: No valid MT rows found.")
        return 1

    print("=" * 72)
    print("RTES MT PERFORMANCE ANALYSIS")
    print("=" * 72)
    print(f"File: {csv_path}")
    print(f"Rows analyzed: {len(rows)}")
    print()

    # Per-thread stats
    by_thread = defaultdict(list)
    for r in rows:
        by_thread[r["thread"]].append(r)

    print("Per-Thread Timing")
    print("-" * 72)
    header = (
        f"{'Thread':<12} {'Cycles':>8} {'Mean(us)':>10} {'Min':>7} {'Max':>7} "
        f"{'P95':>7} {'MaxJit':>8} {'Emerg%':>8}"
    )
    print(header)
    print("-" * len(header))

    for thread_name in sorted(by_thread.keys()):
        tr = by_thread[thread_name]
        exec_vals = [x["exec_us"] for x in tr]
        jit_vals = [x["jitter_us"] for x in tr]
        emerg_count = sum(x["emergency"] for x in tr)

        exec_sorted = sorted(exec_vals)
        p95_idx = max(0, min(len(exec_sorted) - 1, int(round(0.95 * (len(exec_sorted) - 1)))))
        p95 = exec_sorted[p95_idx]

        print(
            f"{thread_name:<12} {len(tr):>8} {statistics.mean(exec_vals):>10.2f} "
            f"{min(exec_vals):>7} {max(exec_vals):>7} {p95:>7} {max(jit_vals):>8} "
            f"{pct(emerg_count, len(tr)):>7.2f}%"
        )

    print()

    # Emergency perspective (usually best from emergency thread only)
    emergency_rows = by_thread.get("emergency", [])
    if emergency_rows:
        distances = [x["distance_m"] for x in emergency_rows]
        emergencies = [x["emergency"] for x in emergency_rows]
        active = sum(emergencies)
        active_pct = pct(active, len(emergency_rows))

        print("Emergency Thread Focus")
        print("-" * 72)
        print(f"Cycles:                 {len(emergency_rows)}")
        print(f"Emergency active:       {active} ({active_pct:.2f}%)")
        print(f"Distance mean:          {statistics.mean(distances):.4f} m")
        print(f"Distance min / max:     {min(distances):.4f} m / {max(distances):.4f} m")
        print(f"Distance median:        {statistics.median(distances):.4f} m")

        # Show recent sample window
        tail = emergency_rows[-5:]
        print("Recent emergency samples:")
        for r in tail:
            print(
                f"  cycle={r['cycle']}, dist={r['distance_m']:.4f} m, "
                f"emergency={r['emergency']}, exec={r['exec_us']} us"
            )
        print()

    # Optional note to reduce confusion about units
    print("Notes")
    print("-" * 72)
    print("distance_m and thresholds are in meters.")
    print("emergency=1 means emergency stop active for that sample.")

    return 0


def main() -> int:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rtes_performance_mt.csv"
    return analyze(csv_path)


if __name__ == "__main__":
    raise SystemExit(main())
