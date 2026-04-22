#!/bin/bash
# Quick build and test script for RTES package

set -e  # Exit on error

echo "═══════════════════════════════════════════════════════════"
echo "Building TrashBot RTES Package"
echo "═══════════════════════════════════════════════════════════"

cd ~/ROS2_Deploy

# Clean previous build
echo "Cleaning previous build..."
rm -rf build/trashbot_rtes install/trashbot_rtes log/trashbot_rtes

# Build the package
# NOTE: Using single-threaded compilation to prevent memory exhaustion on Pi
echo "Building trashbot_rtes (safe mode for Raspberry Pi)..."
colcon build --packages-select trashbot_rtes --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Debug

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
echo "Testing C library..."
cd ~/ROS2_Deploy/RTES
mkdir -p build_test && cd build_test
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j1 test_emergency  # Single-threaded to prevent memory issues
./test_emergency

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "All done! Ready to deploy."
echo "═══════════════════════════════════════════════════════════"
