#!/bin/bash
# Safe build script for Raspberry Pi with memory constraints
# Prevents system freeze by limiting parallel compilation

set -e  # Exit on error

echo "═══════════════════════════════════════════════════════════"
echo "Building TrashBot RTES Package (Safe Mode for Pi)"
echo "═══════════════════════════════════════════════════════════"

# Check available memory
available_mem=$(free -m | awk 'NR==2{print $7}')
echo "Available memory: ${available_mem}MB"

if [ "$available_mem" -lt 500 ]; then
    echo "⚠️  WARNING: Less than 500MB available memory!"
    echo "   Consider closing other applications or rebooting."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

cd ~/ROS2_Deploy

# Clean previous build
echo "Cleaning previous build..."
rm -rf build/trashbot_rtes install/trashbot_rtes log/trashbot_rtes

# Build the package with SAFE settings for Raspberry Pi:
# - Single parallel worker to prevent memory exhaustion
# - Debug mode uses less memory than Release
echo "Building trashbot_rtes (single-threaded, Debug mode)..."
echo "This will take longer but won't crash your system."

colcon build \
    --packages-select trashbot_rtes \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source the workspace
source install/setup.bash

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "Build successful! ✓"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Available executables:"
echo "  • realtime_system_node    (Production - all 3 services)"
echo "  • emergency_stop_node     (Development - Service 1)"
echo "  • path_planner_node       (Development - Service 2)"
echo "  • motor_control_node      (Development - Service 3)"
echo ""
echo "Launch files:"
echo "  • ros2 launch trashbot_rtes full_system.launch.py"
echo "  • ros2 launch trashbot_rtes test_services.launch.py"
echo ""

# Optional: Test C library
read -p "Run C library tests? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Testing C library..."
    cd ~/ROS2_Deploy/RTES
    mkdir -p build_test && cd build_test
    cmake .. -DCMAKE_BUILD_TYPE=Debug
    make -j1 test_emergency  # Single-threaded compilation
    ./test_emergency
    echo ""
fi

echo "═══════════════════════════════════════════════════════════"
echo "All done! Ready to deploy."
echo "═══════════════════════════════════════════════════════════"
