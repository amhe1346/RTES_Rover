#!/bin/bash
# ULTRA-MINIMAL build for Raspberry Pi with severe memory constraints
# This script closes GUI applications to free memory before building

set -e

echo "═══════════════════════════════════════════════════════════"
echo "ULTRA-MINIMAL Build for Raspberry Pi"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "  WARNING: This script will:"
echo "   1. Close all GUI applications to free memory"
echo "   2. Build with minimal optimization (-O0)"
echo "   3. Single-threaded compilation"
echo ""
echo "Current memory status:"
free -h
echo ""

read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

echo ""
echo "Freeing memory..."

# Stop non-essential GNOME services
echo "Closing background apps..."
killall -q nautilus 2>/dev/null || true
killall -q gnome-software 2>/dev/null || true
killall -q evolution-alarm-notify 2>/dev/null || true
killall -q tracker-miner-fs-3 2>/dev/null || true
killall -q update-notifier 2>/dev/null || true
killall -q goa-daemon 2>/dev/null || true
killall -q evolution-calendar-factory 2>/dev/null || true

# Sync filesystems
sync

echo ""
echo "Memory after cleanup:"
free -h
available_mem=$(free -m | awk 'NR==2{print $7}')
echo ""

if [ "$available_mem" -lt 800 ]; then
    echo "⚠️  WARNING: Only ${available_mem}MB available!"
    echo "   Build may still fail. Consider:"
    echo "   - Closing VSCode completely"
    echo "   - Building via SSH from another machine"
    echo "   - Using text console (Ctrl+Alt+F3)"
    echo ""
    read -p "Try anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi
echo ""

cd ~/ROS2_Deploy

# Clean previous build
echo "Cleaning previous build..."
rm -rf build/trashbot_rtes install/trashbot_rtes log/trashbot_rtes

# Build with ABSOLUTE MINIMAL settings
echo "Building with minimal optimization (this will be SLOW)..."
MAKEFLAGS="-j1" colcon build \
    --packages-select trashbot_rtes \
    --parallel-workers 1 \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_FLAGS="-O0 -g0" \
        -DCMAKE_C_FLAGS="-O0 -g0"

# Source the workspace
source install/setup.bash

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "Build successful! ✓"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Available executables:"
echo "  • realtime_system_node"
echo "  • emergency_stop_node"
echo "  • path_planner_node"
echo "  • motor_control_node"
echo ""
