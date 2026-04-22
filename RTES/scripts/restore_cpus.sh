#!/bin/bash
# Restore Normal CPU Allocation
# 
# This script removes the dynamic CPU isolation created by isolate_cpu.sh,
# allowing all processes to use all CPU cores again.
#
# Usage: sudo ./restore_cpus.sh

set -e

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script must be run as root"
    echo "Usage: sudo $0"
    exit 1
fi

echo "═══════════════════════════════════════════════════════════"
echo "Restoring Normal CPU Allocation"
echo "═══════════════════════════════════════════════════════════"

CPUSET_PATH="/sys/fs/cgroup/cpuset/system"

if [ ! -d "$CPUSET_PATH" ]; then
    echo "✓ No cpuset isolation found (already restored)"
    exit 0
fi

# Move all processes back to root cpuset
echo "Moving processes back to root cpuset..."
MOVED_COUNT=0

for PID in $(cat "$CPUSET_PATH/cgroup.procs" 2>/dev/null); do
    if echo $PID > /sys/fs/cgroup/cpuset/cgroup.procs 2>/dev/null; then
        MOVED_COUNT=$((MOVED_COUNT + 1))
    fi
done

echo "✓ Moved $MOVED_COUNT processes to all cores"

# Remove the system cpuset directory
echo "Removing cpuset directory..."
rmdir "$CPUSET_PATH" 2>/dev/null || echo "⚠ Could not remove cpuset directory (may have active processes)"

NUM_CORES=$(nproc)
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "✓ All $NUM_CORES cores available to all processes"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "System has returned to normal CPU scheduling."
echo ""
