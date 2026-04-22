"""
full_system.launch.py - PRODUCTION LAUNCH FILE

Launches the integrated realtime_system_node with all 3 services and optional RT features.
Use for final deployment and performance testing.
Target: <10ms emergency response time.

Real-Time Features (optional):
- enable_rt: Enable POSIX real-time features (SCHED_FIFO, memory locking, CPU affinity)
- rt_priority: SCHED_FIFO priority (1-99, default 80)
- cpu_affinity: Pin to specific CPU core (-1 = no pinning, 0-3 = core number)
- enable_mlockall: Lock all memory to prevent page faults
- deadline_us: Deadline for cycle execution in microseconds (default 10000 = 10ms)
- enable_csv_logging: Enable CSV performance logging
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments - Serial Configuration
        DeclareLaunchArgument(
            'serial_port_front',
            default_value='/dev/ttyACM1',
            description='Front Arduino serial port'
        ),
        DeclareLaunchArgument(
            'serial_port_rear',
            default_value='/dev/ttyACM0',
            description='Rear Arduino serial port'
        ),
        
        # Declare launch arguments - Control Configuration
        DeclareLaunchArgument(
            'control_frequency_hz',
            default_value='1000.0',
            description='Control loop frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'emergency_distance_threshold',
            default_value='0.5',
            description='Emergency stop distance threshold (meters)'
        ),
        
        # Declare launch arguments - Real-Time Configuration
        DeclareLaunchArgument(
            'enable_rt',
            default_value='false',
            description='Enable POSIX real-time features (requires capabilities)'
        ),
        DeclareLaunchArgument(
            'rt_priority',
            default_value='80',
            description='SCHED_FIFO priority (1-99, higher = more priority)'
        ),
        DeclareLaunchArgument(
            'cpu_affinity',
            default_value='-1',
            description='CPU core to pin to (-1 = no pinning, 0-3 = core number)'
        ),
        DeclareLaunchArgument(
            'enable_mlockall',
            default_value='false',
            description='Lock all memory to prevent page faults'
        ),
        DeclareLaunchArgument(
            'deadline_us',
            default_value='10000',
            description='Deadline for control loop execution (microseconds)'
        ),
        
        # Declare launch arguments - Performance Logging
        DeclareLaunchArgument(
            'enable_csv_logging',
            default_value='true',
            description='Enable CSV performance logging'
        ),
        DeclareLaunchArgument(
            'csv_output_path',
            default_value='/tmp/rtes_performance.csv',
            description='Path to CSV output file'
        ),
        
        # ═══════════════════════════════════════════════════════════
        # PRODUCTION NODE: Integrated Realtime System
        # ═══════════════════════════════════════════════════════════
        Node(
            package='trashbot_rtes',
            executable='realtime_system_node',
            name='realtime_system',
            output='screen',
            parameters=[{
                'serial_port_front': LaunchConfiguration('serial_port_front'),
                'serial_port_rear': LaunchConfiguration('serial_port_rear'),
                'control_frequency_hz': LaunchConfiguration('control_frequency_hz'),
                'emergency_distance_threshold': LaunchConfiguration('emergency_distance_threshold'),
                'enable_rt': LaunchConfiguration('enable_rt'),
                'rt_priority': LaunchConfiguration('rt_priority'),
                'cpu_affinity': LaunchConfiguration('cpu_affinity'),
                'enable_mlockall': LaunchConfiguration('enable_mlockall'),
                'deadline_us': LaunchConfiguration('deadline_us'),
                'enable_csv_logging': LaunchConfiguration('enable_csv_logging'),
                'csv_output_path': LaunchConfiguration('csv_output_path'),
            }]
        ),
    ])
