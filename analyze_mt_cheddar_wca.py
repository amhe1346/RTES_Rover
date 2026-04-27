#!/usr/bin/env python3
"""
Cheddar-style simulation + worst-case response-time analysis for MT RTES services.

Given periodic fixed-priority services S_i with (C_i, T_i, D_i), this tool:
1) Extracts C_i candidates from measured CSV data.
2) Runs fixed-priority preemptive worst-case response-time analysis (RTA).
3) Runs an event-driven fixed-priority simulation over one hyperperiod.
4) Writes a concise report suitable for assignment evidence.

Usage:
  python3 analyze_mt_cheddar_wca.py /tmp/rtes_performance_mt.csv
  python3 analyze_mt_cheddar_wca.py /tmp/rtes_performance_mt.csv --ci-mode p95
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path


THREADS = ("emergency", "planning", "sensor", "motor")

THREAD_PERIOD_US = {
    "emergency": 10_000,
    "planning": 20_000,
    "motor": 20_000,
    "sensor": 70_000,
}

THREAD_DEADLINE_US = {
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


@dataclass
class Task:
    name: str
    priority: int
    c_us: int
    t_us: int
    d_us: int


def percentile(values, fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round(fraction * (len(ordered) - 1)))
    idx = max(0, min(idx, len(ordered) - 1))
    return float(ordered[idx])


def parse_exec_rows(csv_path: str):
    rows = []
    with open(csv_path, "r", newline="") as handle:
        reader = csv.DictReader(row for row in handle if not row.lstrip().startswith("#"))
        for row in reader:
            try:
                thread = row["thread"].strip()
                if thread not in THREADS:
                    continue
                rows.append((thread, int(row["exec_us"])))
            except (KeyError, ValueError):
                continue
    return rows


def derive_ci(exec_values, mode: str) -> int:
    if not exec_values:
        return 0
    if mode == "max":
        return int(max(exec_values))
    if mode == "p99":
        return int(math.ceil(percentile(exec_values, 0.99)))
    if mode == "p95":
        return int(math.ceil(percentile(exec_values, 0.95)))
    raise ValueError(f"Unsupported ci mode: {mode}")


def rta(tasks):
    """Classic fixed-priority preemptive response-time analysis (single core, B_i=0)."""
    ordered = sorted(tasks, key=lambda t: t.priority, reverse=True)
    result = {}
    for i, task in enumerate(ordered):
        hp = ordered[:i]
        if not hp:
            r = task.c_us
            result[task.name] = (r, r <= task.d_us, 1)
            continue

        r_prev = task.c_us
        iterations = 0
        converged = False
        while iterations < 1000:
            interference = 0
            for ht in hp:
                interference += math.ceil(r_prev / ht.t_us) * ht.c_us
            r = task.c_us + interference
            iterations += 1
            if r == r_prev:
                converged = True
                break
            if r > task.d_us and r > r_prev:
                # Monotonic growth past deadline; can stop early.
                r_prev = r
                break
            r_prev = r

        r_final = r_prev
        ok = converged and (r_final <= task.d_us)
        result[task.name] = (r_final, ok, iterations)

    return result


@dataclass
class Job:
    task_name: str
    release_us: int
    abs_deadline_us: int
    remaining_us: int
    priority: int


def lcm(a: int, b: int) -> int:
    return abs(a * b) // math.gcd(a, b)


def hyperperiod(periods):
    hp = periods[0]
    for p in periods[1:]:
        hp = lcm(hp, p)
    return hp


def simulate_fp(tasks, horizon_us):
    """Event-driven fixed-priority preemptive simulation on one core."""
    by_name = {t.name: t for t in tasks}
    next_release = {t.name: 0 for t in tasks}
    ready = []
    now = 0

    completed_response = {t.name: [] for t in tasks}
    misses = {t.name: 0 for t in tasks}

    while now < horizon_us:
        # Release all jobs due now.
        for t in tasks:
            while next_release[t.name] <= now:
                r = next_release[t.name]
                ready.append(
                    Job(
                        task_name=t.name,
                        release_us=r,
                        abs_deadline_us=r + t.d_us,
                        remaining_us=t.c_us,
                        priority=t.priority,
                    )
                )
                next_release[t.name] += t.t_us

        if not ready:
            now = min(next_release.values())
            continue

        # Highest priority job runs.
        ready.sort(key=lambda j: (-j.priority, j.release_us))
        job = ready[0]

        # Next potential preemption point is the next release of any higher-priority task.
        higher_next = []
        for t in tasks:
            if t.priority > job.priority:
                higher_next.append(next_release[t.name])
        next_hp_release = min(higher_next) if higher_next else None

        run_until = now + job.remaining_us
        if next_hp_release is not None and now < next_hp_release < run_until:
            delta = next_hp_release - now
            job.remaining_us -= delta
            now = next_hp_release
            continue

        # Job completes.
        now = run_until
        response = now - job.release_us
        completed_response[job.task_name].append(response)
        if now > job.abs_deadline_us:
            misses[job.task_name] += 1
        ready.pop(0)

        # Drop any jobs that can no longer be completed before horizon bookkeeping doesn't require drop,
        # keep normal backlog behavior.

    sim = {}
    for t in tasks:
        vals = completed_response[t.name]
        sim[t.name] = {
            "count": len(vals),
            "max_response": max(vals) if vals else 0,
            "p95_response": percentile(vals, 0.95) if vals else 0.0,
            "misses": misses[t.name],
        }
    return sim


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Cheddar-style fixed-priority simulation + worst-case response-time analysis"
    )
    parser.add_argument("csv_path", nargs="?", default="/tmp/rtes_performance_mt.csv")
    parser.add_argument(
        "--ci-mode",
        choices=("max", "p99", "p95"),
        default="p95",
        help="How to derive C_i from measured exec_us",
    )
    parser.add_argument(
        "--report-path",
        default="/home/trashbot/ROS2_Deploy/RTES/scripts/rtes_cheddar_wca_report.txt",
        help="Output report file",
    )
    parser.add_argument(
        "--emergency-core",
        type=int,
        default=2,
        help="Emergency core affinity used in deployment (-1 if none)",
    )
    args = parser.parse_args()

    path = Path(args.csv_path)
    if not path.exists():
        print(f"ERROR: File not found: {args.csv_path}")
        return 1

    rows = parse_exec_rows(str(path))
    if not rows:
        print("ERROR: No valid exec rows found.")
        return 1

    by_thread = {name: [] for name in THREADS}
    for thread, exec_us in rows:
        by_thread[thread].append(exec_us)

    tasks = []
    for name in THREADS:
        values = by_thread.get(name, [])
        if not values:
            continue
        c_us = derive_ci(values, args.ci_mode)
        tasks.append(
            Task(
                name=name,
                priority=THREAD_PRIORITY[name],
                c_us=c_us,
                t_us=THREAD_PERIOD_US[name],
                d_us=THREAD_DEADLINE_US[name],
            )
        )

    if len(tasks) < 2:
        print("ERROR: Need at least 2 services with valid timing data.")
        return 1

    util = sum(t.c_us / t.t_us for t in tasks)
    rta_result = rta(tasks)
    hp = hyperperiod([t.t_us for t in tasks])
    sim_result = simulate_fp(tasks, hp)

    report_path = Path(args.report_path)
    with open(report_path, "w", encoding="utf-8") as out:
        out.write("RTES FIXED-PRIORITY SCHEDULING ANALYSIS (CHEDDAR-STYLE)\n")
        out.write("=" * 78 + "\n")
        out.write(f"Input CSV: {args.csv_path}\n")
        out.write("Scheduler: single-core fixed-priority preemptive\n")
        if args.emergency_core >= 0:
            out.write(f"Core affinity: emergency pinned to core {args.emergency_core}\n")
        else:
            out.write("Core affinity: none\n")
        out.write(f"C_i derivation: {args.ci_mode}\n")
        out.write(f"Total utilization U = {util:.6f}\n")
        out.write(f"Simulation horizon (hyperperiod): {hp} us\n\n")

        out.write("Task Set (S_i with C_i, T_i, D_i)\n")
        out.write("-" * 78 + "\n")
        out.write("Task,Priority,C_i(us),T_i(us),D_i(us),U_i\n")
        for t in sorted(tasks, key=lambda x: x.priority, reverse=True):
            out.write(f"{t.name},{t.priority},{t.c_us},{t.t_us},{t.d_us},{t.c_us / t.t_us:.6f}\n")

        out.write("\nWorst-Case Response-Time Analysis (RTA)\n")
        out.write("-" * 78 + "\n")
        out.write("Task,R_i(us),D_i(us),R_i<=D_i,Iterations\n")
        for t in sorted(tasks, key=lambda x: x.priority, reverse=True):
            r_i, ok, iters = rta_result[t.name]
            out.write(f"{t.name},{r_i},{t.d_us},{1 if ok else 0},{iters}\n")

        out.write("\nCheddar-Style Simulation Results (event-driven FP simulation)\n")
        out.write("-" * 78 + "\n")
        out.write("Task,JobsCompleted,MaxResponse(us),P95Response(us),Misses\n")
        for t in sorted(tasks, key=lambda x: x.priority, reverse=True):
            sim = sim_result[t.name]
            out.write(
                f"{t.name},{sim['count']},{sim['max_response']},{sim['p95_response']:.2f},{sim['misses']}\n"
            )

        out.write("\nPredictability and Frequency Constancy\n")
        out.write("-" * 78 + "\n")
        out.write(
            "Predictable responses are indicated when R_i from analysis and max/p95 response\n"
            "from simulation stay well below D_i with very low miss counts. Constant periodic\n"
            "request frequency is reflected by periodic releases every T_i in the simulation\n"
            "model and by stable measured per-thread periodic logging in the input dataset.\n"
        )

    print("=" * 78)
    print("CHEDDAR-STYLE FP ANALYSIS + WCA")
    print("=" * 78)
    print(f"Report: {report_path}")
    print(f"Services analyzed: {len(tasks)}")
    print(f"Utilization U: {util:.6f}")
    print(f"Hyperperiod: {hp} us")
    print()
    print("RTA Summary")
    print("-" * 78)
    for t in sorted(tasks, key=lambda x: x.priority, reverse=True):
        r_i, ok, _ = rta_result[t.name]
        print(f"{t.name:<10} R_i={r_i:>8} us  D_i={t.d_us:>8} us  {'OK' if ok else 'FAIL'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
