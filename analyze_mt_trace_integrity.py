#!/usr/bin/env python3
"""
MT RTES trace integrity analyzer.
Checks whether all thread traces are present and quantifies cycle continuity.

Usage:
  python3 analyze_mt_trace_integrity.py /tmp/rtes_performance_mt.csv
"""

from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path


def pct(part: int, whole: int) -> float:
    return (100.0 * part / whole) if whole else 0.0


def parse_rows(csv_path: str):
    rows = []
    with open(csv_path, "r", newline="") as f:
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
                    }
                )
            except Exception:
                continue
    return rows


def continuity_metrics(cycles):
    if len(cycles) < 2:
        return 0, 0, 0

    sorted_cycles = sorted(cycles)
    gaps = 0
    max_gap = 0
    backward = 0

    prev = sorted_cycles[0]
    for cur in sorted_cycles[1:]:
        if cur <= prev:
            backward += 1
        delta = cur - prev
        if delta > 1:
            gaps += 1
            if delta > max_gap:
                max_gap = delta
        prev = cur

    return gaps, max_gap, backward


def main() -> int:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rtes_performance_mt.csv"
    if not Path(csv_path).exists():
        print(f"ERROR: File not found: {csv_path}")
        return 1

    rows = parse_rows(csv_path)
    if not rows:
        print("ERROR: No valid trace rows found")
        return 1

    by_thread = defaultdict(list)
    for r in rows:
        by_thread[r["thread"]].append(r)

    expected = ["emergency", "sensor", "planning", "motor"]

    print("=" * 72)
    print("RTES MT TRACE INTEGRITY")
    print("=" * 72)
    print(f"File: {csv_path}")
    print(f"Total rows: {len(rows)}")
    print()

    missing = [t for t in expected if t not in by_thread]
    if missing:
        print(f"Missing threads in trace rows: {', '.join(missing)}")
    else:
        print("All expected threads present in trace rows.")
    print()

    header = f"{'Thread':<12} {'Rows':>8} {'Emerg%':>8} {'Gaps':>6} {'MaxGap':>8} {'Backwards':>10}"
    print(header)
    print("-" * len(header))

    for t in sorted(by_thread.keys()):
        tr = by_thread[t]
        cycles = [x["cycle"] for x in tr]
        emerg = sum(x["emergency"] for x in tr)
        gaps, max_gap, backwards = continuity_metrics(cycles)
        print(f"{t:<12} {len(tr):>8} {pct(emerg, len(tr)):>7.2f}% {gaps:>6} {max_gap:>8} {backwards:>10}")

    print()
    dist_rows = [r for r in rows if r["thread"] in {"emergency", "sensor"}]
    if dist_rows:
        distances = [r["distance_m"] for r in dist_rows]
        print(f"Distance samples considered: {len(distances)}")
        print(f"Distance min/max: {min(distances):.4f} m / {max(distances):.4f} m")
        print(f"Distance mean:    {sum(distances)/len(distances):.4f} m")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
