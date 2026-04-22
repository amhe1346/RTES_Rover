#!/bin/bash
# Real-Time Configuration Verification Script
#
# Checks if the RTES node is running with proper real-time configuration:
# - SCHED_FIFO scheduling policy
# - Correct priority
# - CPU affinity (if configured)
# - Memory locking
# - Performance statistics
#
# Usage: ./verify_rt_config.sh [process_name]
# Example: ./verify_rt_config.sh realtime_system

PROCESS_NAME="${1:-realtime_system}"

echo "═══════════════════════════════════════════════════════════"
echo "Real-Time Configuration Verification"
echo "═══════════════════════════════════════════════════════════"
echo "Looking for process: $PROCESS_NAME"
echo ""

# Find the PID
PID=$(pgrep -f "$PROCESS_NAME" | head -1)

if [ -z "$PID" ]; then
    echo "✗ ERROR: Process '$PROCESS_NAME' not found"
    echo ""
    echo "Is the RTES node running?"
    echo "Start it with: ros2 launch trashbot_rtes full_system.launch.py enable_rt:=true"
    echo ""
    exit 1
fi

echo "✓ Process found (PID: $PID)"
echo ""

# ═══════════════════════════════════════════════════════════════════
# Check Scheduling Policy and Priority
# ═══════════════════════════════════════════════════════════════════
echo "─── Scheduling Configuration ───"

SCHED_INFO=$(chrt -p $PID 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "$SCHED_INFO"
    
    if echo "$SCHED_INFO" | grep -q "SCHED_FIFO"; then
        PRIORITY=$(echo "$SCHED_INFO" | grep -oP 'priority: \K\d+')
        echo "✓ SCHED_FIFO enabled with priority $PRIORITY"
        
        if [ "$PRIORITY" -ge 80 ]; then
            echo "  ✓ High priority ($PRIORITY >= 80)"
        elif [ "$PRIORITY" -ge 50 ]; then
            echo "  ⚠ Medium priority ($PRIORITY)"
        else
            echo "  ⚠ Low priority ($PRIORITY < 50)"
        fi
    else
        echo "✗ NOT using SCHED_FIFO (using default scheduler)"
        echo "  Enable with: enable_rt:=true rt_priority:=80"
    fi
else
    echo "✗ Could not check scheduling policy"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════
# Check CPU Affinity
# ═══════════════════════════════════════════════════════════════════
echo "─── CPU Affinity ───"

AFFINITY=$(taskset -cp $PID 2>/dev/null | grep -oP "list: \K.*")
if [ $? -eq 0 ]; then
    echo "Current affinity: $AFFINITY"
    
    # Check if pinned to single core
    if [[ "$AFFINITY" =~ ^[0-9]$ ]]; then
        echo "✓ Pinned to core $AFFINITY"
        
        # Check if that core is isolated
        ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)
        if [ "$ISOLATED" == "$AFFINITY" ]; then
            echo "  ✓ Core $AFFINITY is isolated (exclusive use)"
        else
            echo "  ⚠ Core $AFFINITY is NOT isolated (shared with other processes)"
            echo "    Run: sudo ./isolate_cpu.sh $AFFINITY"
        fi
    else
        echo "⚠ Not pinned to a single core"
        echo "  Set with: cpu_affinity:=2"
    fi
else
    echo "✗ Could not check CPU affinity"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════
# Check Memory Locking
# ═══════════════════════════════════════════════════════════════════
echo "─── Memory Locking ───"

if [ -f "/proc/$PID/status" ]; then
    VM_LOCK=$(grep VmLck /proc/$PID/status | awk '{print $2}')
    VM_RSS=$(grep VmRSS /proc/$PID/status | awk '{print $2}')
    
    echo "Locked memory: $VM_LOCK kB"
    echo "Resident memory: $VM_RSS kB"
    
    if [ "$VM_LOCK" -gt 0 ]; then
        LOCK_PERCENT=$((100 * VM_LOCK / VM_RSS))
        echo "✓ Memory locking enabled ($LOCK_PERCENT% of RSS locked)"
        
        if [ "$LOCK_PERCENT" -ge 90 ]; then
            echo "  ✓ Most memory is locked (good)"
        else
            echo "  ⚠ Only partial memory locking ($LOCK_PERCENT%)"
        fi
    else
        echo "✗ Memory NOT locked (risk of page faults)"
        echo "  Enable with: enable_mlockall:=true"
    fi
else
    echo "✗ Could not check memory locking"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════
# Check Process Activity
# ═══════════════════════════════════════════════════════════════════
echo "─── Process Activity ───"

# Get CPU usage
CPU_USAGE=$(ps -p $PID -o %cpu= | tr -d ' ')
echo "CPU usage: ${CPU_USAGE}%"

# Check which core(s) it's running on
RUNNING_ON=$(ps -eLo psr,pid | grep "^ *[0-9]* *$PID$" | awk '{print $1}' | sort -u | tr '\n' ',' | sed 's/,$//')
echo "Running on core(s): $RUNNING_ON"

# Thread count
THREAD_COUNT=$(ps -Lp $PID | wc -l)
echo "Thread count: $((THREAD_COUNT - 1))"
echo ""

# ═══════════════════════════════════════════════════════════════════
# Check Capabilities (Permissions)
# ═══════════════════════════════════════════════════════════════════
echo "─── Capabilities ───"

if [ -f "/proc/$PID/status" ]; then
    CAP_EFF=$(grep CapEff /proc/$PID/status | awk '{print $2}')
    
    # Check for CAP_SYS_NICE (for RT scheduling)
    CAP_SYS_NICE=$((0x$(echo $CAP_EFF) & 0x800000))
    if [ $CAP_SYS_NICE -ne 0 ]; then
        echo "✓ CAP_SYS_NICE present (RT scheduling allowed)"
    else
        echo "⚠ CAP_SYS_NICE missing (RT scheduling may fail)"
        echo "  Grant with: sudo setcap cap_sys_nice=eip realtime_system_node"
    fi
    
    # Check for CAP_IPC_LOCK (for memory locking)
    CAP_IPC_LOCK=$((0x$(echo $CAP_EFF) & 0x4000))
    if [ $CAP_IPC_LOCK -ne 0 ]; then
        echo "✓ CAP_IPC_LOCK present (memory locking allowed)"
    else
        echo "⚠ CAP_IPC_LOCK missing (memory locking may fail)"
        echo "  Grant with: sudo setcap cap_ipc_lock=eip realtime_system_node"
    fi
else
    echo "✗ Could not check capabilities"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════
# Performance Data Check
# ═══════════════════════════════════════════════════════════════════
echo "─── Performance Logging ───"

CSV_FILE="/tmp/rtes_performance.csv"
if [ -f "$CSV_FILE" ]; then
    LINE_COUNT=$(wc -l < "$CSV_FILE")
    FILE_SIZE=$(du -h "$CSV_FILE" | cut -f1)
    echo "✓ CSV log found: $CSV_FILE"
    echo "  Lines: $LINE_COUNT"
    echo "  Size: $FILE_SIZE"
    
    # Show last few summary lines if they exist
    if grep -q "^# Performance Summary" "$CSV_FILE"; then
        echo ""
        echo "Latest statistics:"
        grep "^# " "$CSV_FILE" | tail -8
    fi
else
    echo "⚠ No CSV log found at $CSV_FILE"
    echo "  Enable with: enable_csv_logging:=true"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════
echo "═══════════════════════════════════════════════════════════"
echo "Verification Complete"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "For full RT performance, ensure:"
echo "  1. SCHED_FIFO enabled (priority 80+)"
echo "  2. CPU pinned to isolated core"
echo "  3. Memory locked (mlockall)"
echo "  4. Proper capabilities set on binary"
echo ""
echo "To analyze performance:"
echo "  python3 ~/ROS2_Deploy/RTES/scripts/analyze_performance.py $CSV_FILE"
echo ""
