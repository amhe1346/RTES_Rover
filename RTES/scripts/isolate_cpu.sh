#!/bin/bash
# Dynamic CPU Isolation for RTES
# 
# This script dynamically moves all non-RT processes away from a specified CPU core
# using cpuset cgroups. The isolation is temporary and automatically reverses when
# the RTES node stops (or on system reboot).
#
# Usage: sudo ./isolate_cpu.sh [core_number]
# Example: sudo ./isolate_cpu.sh 2
#
# Default: Isolates core 2 for RTES, moves all other processes to cores 0,1,3

set -e

# Default core to isolate
ISOLATED_CORE=${1:-2}

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script must be run as root"
    echo "Usage: sudo $0 [core_number]"
    exit 1
fi

# Check if cpuset cgroup is available
if [ ! -d "/sys/fs/cgroup/cpuset" ]; then
    echo "ERROR: cpuset cgroup not available"
    echo "Try: sudo mount -t cgroup -o cpuset cpuset /sys/fs/cgroup/cpuset"
    exit 1
fi

echo "═══════════════════════════════════════════════════════════"
echo "Dynamic CPU Isolation for RTES"
echo "═══════════════════════════════════════════════════════════"
echo "Isolating core: $ISOLATED_CORE"
echo "Other cores will handle all non-RT processes"
echo ""

# Detect total number of cores
NUM_CORES=$(nproc)
echo "Total CPU cores: $NUM_CORES"

# Build cpuset for non-isolated cores (comma-separated list)
NON_ISOLATED_CORES=""
for ((i=0; i<NUM_CORES; i++)); do
    if [ $i -ne $ISOLATED_CORE ]; then
        if [ -z "$NON_ISOLATED_CORES" ]; then
            NON_ISOLATED_CORES="$i"
        else
            NON_ISOLATED_CORES="$NON_ISOLATED_CORES,$i"
        fi
    fi
done

echo "Non-isolated cores: $NON_ISOLATED_CORES"
echo ""

# Create cpuset for system processes (if it doesn't exist)
CPUSET_PATH="/sys/fs/cgroup/cpuset/system"

if [ ! -d "$CPUSET_PATH" ]; then
    echo "Creating cpuset for system processes..."
    mkdir -p "$CPUSET_PATH"
fi

# Configure the system cpuset
echo "$NON_ISOLATED_CORES" > "$CPUSET_PATH/cpuset.cpus"
echo 0 > "$CPUSET_PATH/cpuset.mems"  # Use all memory nodes

# Move all existing processes to the system cpuset
echo "Moving processes to non-isolated cores..."
MOVED_COUNT=0

# Get all PIDs in the root cpuset
for PID in $(cat /sys/fs/cgroup/cpuset/cgroup.procs); do
    # Skip kernel threads (no cmdline)
    if [ ! -f "/proc/$PID/cmdline" ] || [ ! -s "/proc/$PID/cmdline" ]; then
        continue
    fi
    
    # Try to move the process
    if echo $PID > "$CPUSET_PATH/cgroup.procs" 2>/dev/null; then
        MOVED_COUNT=$((MOVED_COUNT + 1))
    fi
done

echo "✓ Moved $MOVED_COUNT processes to cores $NON_ISOLATED_CORES"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "Core $ISOLATED_CORE is now isolated for RTES!"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "To verify isolation:"
echo "  1. Check isolated core is empty:"
echo "     ps -eLo psr,pid,comm | grep '^ *$ISOLATED_CORE '"
echo ""
echo "  2. Run RTES node with CPU affinity:"
echo "     ros2 launch trashbot_rtes full_system.launch.py enable_rt:=true cpu_affinity:=$ISOLATED_CORE"
echo ""
echo "  3. Verify RTES is on isolated core:"
echo "     ps -eLo psr,pid,comm | grep realtime_system"
echo ""
echo "To restore normal CPU allocation:"
echo "     sudo ./restore_cpus.sh"
echo ""
