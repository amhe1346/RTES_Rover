"""
full_system_mt.launch.py - MULTI-THREADED PRODUCTION LAUNCH FILE

Launches the multi-threaded realtime_system_node_mt with 4 independent priority-driven threads.
Use for maximum real-time performance with preemptive scheduling.
Target: <10ms emergency response time with 50-70% CPU reduction.

Architecture:
- Emergency Thread (P99, 100Hz/10ms) - Highest priority, preempts all others
- Sensor Thread (P85, 14Hz/70ms) - Respects HC-SR04 60ms minimum measurement cycle
- Planning Thread (P90, 50Hz/20ms) - Medium priority
- Motor Thread (P80, 50Hz/20ms) - Lowest priority

Real-Time Features:
- Per-thread SCHED_FIFO RT priorities (hardcoded in node)
- enable_rt: Enable memory locking and RT setup
- cpu_affinity: Pin emergency thread to isolated core
- enable_mlockall: Lock all memory to prevent page faults
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
            default_value='/dev/ttyACM0',
            description='Arduino serial port'
        ),
        
        # Declare launch arguments - Control Configuration
        DeclareLaunchArgument(
            'emergency_distance_threshold',
            default_value='0.25',
            description='Emergency stop distance threshold (meters)'
        ),
        
        # Declare launch arguments - Real-Time Configuration
        DeclareLaunchArgument(
            'enable_rt',
            default_value='true',
            description='Enable POSIX real-time features (mlockall, RT priorities)'
        ),
        DeclareLaunchArgument(
            'cpu_affinity',
            default_value='2',
            description='CPU core to pin emergency thread to (-1 = no pinning)'
        ),
        DeclareLaunchArgument(
            'enable_mlockall',
            default_value='true',
            description='Lock all memory to prevent page faults'
        ),
        DeclareLaunchArgument(
            'deadline_us',
            default_value='10000',
            description='Deadline for emergency thread (microseconds)'
        ),
        
        # Declare launch arguments - Performance Logging
        DeclareLaunchArgument(
            'enable_csv_logging',
            default_value='true',
            description='Enable CSV performance logging'
        ),
        DeclareLaunchArgument(
            'csv_output_path',
            default_value='/tmp/rtes_performance_mt.csv',
            description='Path to CSV output file'
        ),
        
        # Declare launch arguments - Ultrasonic Configuration
        DeclareLaunchArgument(
            'enable_ultrasonic_gpio',
            default_value='true',
            description='Enable ultrasonic sensor via GPIO'
        ),
        DeclareLaunchArgument(
            'use_rt_gpio',
            default_value='true',
            description='Use RT memory-mapped GPIO driver'
        ),
        DeclareLaunchArgument(
            'ultrasonic_trigger_pin',
            default_value='24',
            description='GPIO pin for ultrasonic trigger'
        ),
        DeclareLaunchArgument(
            'ultrasonic_echo_pin',
            default_value='18',
            description='GPIO pin for ultrasonic echo'
        ),
        
        # ═══════════════════════════════════════════════════════════
        # MULTI-THREADED PRODUCTION NODE
        # ═══════════════════════════════════════════════════════════
        Node(
            package='trashbot_rtes',
            executable='realtime_system_node_mt',
            name='realtime_system_mt',
            output='screen',
            parameters=[{
                'serial_port_front': LaunchConfiguration('serial_port_front'),
                'emergency_distance_threshold': LaunchConfiguration('emergency_distance_threshold'),
                'enable_rt': LaunchConfiguration('enable_rt'),
                'cpu_affinity': LaunchConfiguration('cpu_affinity'),
                'enable_mlockall': LaunchConfiguration('enable_mlockall'),
                'deadline_us': LaunchConfiguration('deadline_us'),
                'enable_csv_logging': LaunchConfiguration('enable_csv_logging'),
                'csv_output_path': LaunchConfiguration('csv_output_path'),
                'enable_ultrasonic_gpio': LaunchConfiguration('enable_ultrasonic_gpio'),
                'use_rt_gpio': LaunchConfiguration('use_rt_gpio'),
                'ultrasonic_trigger_pin': LaunchConfiguration('ultrasonic_trigger_pin'),
                'ultrasonic_echo_pin': LaunchConfiguration('ultrasonic_echo_pin'),
            }]
        ),
    ])
