#!/usr/bin/env python3
"""
RTES Performance Analysis Script

Analyzes CSV performance data from the RTES realtime_system_node.
Similar to the analysis in fifo_example, provides comprehensive statistics
on execution time, jitter, deadline misses, and motor commands.

Usage: python3 analyze_performance.py [csv_file]
Example: python3 analyze_performance.py /tmp/rtes_performance.csv
"""

import sys
import csv
import math
from collections import Counter

def analyze_csv(filename):
    """Analyze RTES performance CSV file"""
    
    print("═" * 70)
    print("RTES PERFORMANCE ANALYSIS")
    print("═" * 70)
    print(f"File: {filename}\n")
    
    # Data storage
    frames = []
    exec_times = []
    delta_times = []
    misses = []
    commands = []
    distances = []
    velocities = []
    emergencies = []
    
    # Read CSV
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Skip comment lines
                if row.get('frame', '').startswith('#'):
                    continue
                
                try:
                    frame = int(row['frame'])
                    exec_us = int(row['exec_us'])
                    delta_us = int(row['delta_us'])
                    miss = int(row['miss'])
                    command = row['command']
                    distance = float(row['distance_m'])
                    velocity = float(row['velocity_ms'])
                    emergency = int(row['emergency'])
                    
                    # Skip frame 0 for statistics (no delta)
                    if frame > 0:
                        frames.append(frame)
                        exec_times.append(exec_us)
                        delta_times.append(delta_us)
                        misses.append(miss)
                        commands.append(command)
                        distances.append(distance)
                        velocities.append(velocity)
                        emergencies.append(emergency)
                        
                except (ValueError, KeyError) as e:
                    continue  # Skip malformed rows
                    
    except FileNotFoundError:
        print(f"ERROR: File not found: {filename}")
        sys.exit(1)
    
    if not frames:
        print("ERROR: No valid data found in CSV file")
        sys.exit(1)
    
    # Calculate statistics
    n = len(frames)
    deadline_us = int(sys.argv[2]) if len(sys.argv) > 2 else 10000  # Default 10ms
    
    # Execution time stats
    exec_avg = sum(exec_times) / n
    exec_min = min(exec_times)
    exec_max = max(exec_times)
    exec_std = math.sqrt(sum((x - exec_avg)**2 for x in exec_times) / n)
    exec_jitter = exec_max - exec_min
    exec_cv = exec_std / exec_avg if exec_avg > 0 else 0
    
    # Delta (period) stats
    delta_avg = sum(delta_times) / n
    delta_min = min(delta_times)
    delta_max = max(delta_times)
    delta_std = math.sqrt(sum((x - delta_avg)**2 for x in delta_times) / n)
    delta_jitter = delta_max - delta_min
    delta_cv = delta_std / delta_avg if delta_avg > 0 else 0
    
    # Frame rate
    fps = 1000000.0 / delta_avg
    
    # Deadline analysis
    miss_count = sum(misses)
    miss_percent = (100.0 * miss_count) / n
    
    # Emergency stop analysis
    emergency_count = sum(emergencies)
    emergency_percent = (100.0 * emergency_count) / n
    
    # Motor command analysis
    command_counts = Counter(commands)
    
    # Distance/velocity stats
    dist_avg = sum(distances) / n
    vel_avg = sum(velocities) / n
    
    # ═══════════════════════════════════════════════════════════════════
    # Print Report
    # ═══════════════════════════════════════════════════════════════════
    
    print(f"Frames analyzed: {n}")
    print(f"Deadline: {deadline_us} μs ({deadline_us / 1000.0:.2f} ms)")
    print()
    
    print("─" * 70)
    print("EXECUTION TIME STATISTICS")
    print("─" * 70)
    print(f"  Mean:        {exec_avg:10.2f} μs  ({exec_avg / 1000.0:8.3f} ms)")
    print(f"  Std Dev:     {exec_std:10.2f} μs  ({exec_std / 1000.0:8.3f} ms)")
    print(f"  Min:         {exec_min:10} μs  ({exec_min / 1000.0:8.3f} ms)")
    print(f"  Max:         {exec_max:10} μs  ({exec_max / 1000.0:8.3f} ms)")
    print(f"  Jitter:      {exec_jitter:10} μs  ({exec_jitter / 1000.0:8.3f} ms)")
    print(f"  CV:          {exec_cv:10.4f}     ({exec_cv * 100.0:8.2f} %)")
    print()
    
    print("─" * 70)
    print("FRAME RATE STATISTICS")
    print("─" * 70)
    print(f"  Average FPS: {fps:10.2f}")
    print(f"  Delta Mean:  {delta_avg:10.2f} μs  ({delta_avg / 1000.0:8.3f} ms)")
    print(f"  Delta Std:   {delta_std:10.2f} μs  ({delta_std / 1000.0:8.3f} ms)")
    print(f"  Delta Min:   {delta_min:10} μs  ({delta_min / 1000.0:8.3f} ms)")
    print(f"  Delta Max:   {delta_max:10} μs  ({delta_max / 1000.0:8.3f} ms)")
    print(f"  Delta Jitter:{delta_jitter:10} μs  ({delta_jitter / 1000.0:8.3f} ms)")
    print(f"  Delta CV:    {delta_cv:10.4f}     ({delta_cv * 100.0:8.2f} %)")
    print()
    
    print("─" * 70)
    print("DEADLINE ANALYSIS")
    print("─" * 70)
    print(f"  Deadline:    {deadline_us:10} μs  ({deadline_us / 1000.0:8.3f} ms)")
    print(f"  Misses:      {miss_count:10} / {n}")
    print(f"  Miss %:      {miss_percent:10.2f} %")
    print()
    
    if miss_percent < 1.0:
        print("  ✓ Excellent real-time performance")
    elif miss_percent < 5.0:
        print("  ✓ Good real-time performance")
    elif miss_percent < 10.0:
        print("  ⚠ Marginal real-time performance")
    else:
        print("  ✗ Poor real-time performance")
    print()
    
    print("─" * 70)
    print("MOTOR COMMAND ANALYSIS")
    print("─" * 70)
    for cmd, count in sorted(command_counts.items(), key=lambda x: x[1], reverse=True):
        cmd_name = {
            'F': 'Forward',
            'B': 'Backward',
            'L': 'Left',
            'R': 'Right',
            'S': 'Stop'
        }.get(cmd, 'Unknown')
        percent = (100.0 * count) / n
        print(f"  {cmd} ({cmd_name:8}): {count:6} ({percent:5.1f}%)")
    print()
    
    print("─" * 70)
    print("SAFETY ANALYSIS")
    print("─" * 70)
    print(f"  Emergency stops:  {emergency_count:6} ({emergency_percent:5.1f}%)")
    print(f"  Normal operation: {n - emergency_count:6} ({100.0 - emergency_percent:5.1f}%)")
    print()
    print(f"  Avg distance: {dist_avg:6.2f} m")
    print(f"  Avg velocity: {vel_avg:6.2f} m/s")
    print()
    
    print("═" * 70)
    print("RECOMMENDATIONS")
    print("═" * 70)
    
    if miss_percent > 5.0:
        print("  ⚠ High deadline miss rate!")
        print("    - Enable RT features: enable_rt:=true")
        print("    - Use CPU isolation: sudo ./scripts/isolate_cpu.sh 2")
        print("    - Reduce control frequency if possible")
    
    if exec_cv > 0.5:
        print("  ⚠ High execution time variability!")
        print("    - Enable memory locking: enable_mlockall:=true")
        print("    - Check for page faults in dmesg")
    
    if delta_cv > 0.2:
        print("  ⚠ High frame rate jitter!")
        print("    - Use SCHED_FIFO: enable_rt:=true rt_priority:=80")
        print("    - Pin to isolated CPU core: cpu_affinity:=2")
        print("    - Close background applications")
    
    if miss_percent < 1.0 and exec_cv < 0.1 and delta_cv < 0.1:
        print("  ✓ Excellent performance! System is well-tuned.")
    
    print("═" * 70)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_performance.py <csv_file> [deadline_us]")
        print("Example: python3 analyze_performance.py /tmp/rtes_performance.csv 10000")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    analyze_csv(csv_file)
