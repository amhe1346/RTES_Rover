"""
test_services.launch.py - DEVELOPMENT LAUNCH FILE

Launches 3 separate nodes for testing individual services.
Use for development, debugging, and validation.
Allows independent testing of each service with inter-node communication.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
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
        
        # ═══════════════════════════════════════════════════════════
        # SERVICE 1: Emergency Stop (Highest Priority)
        # ═══════════════════════════════════════════════════════════
        Node(
            package='trashbot_rtes',
            executable='emergency_stop_node',
            name='emergency_stop',
            output='screen',
            prefix='xterm -e',  # Optional: open in separate terminal
        ),
        
        # ═══════════════════════════════════════════════════════════
        # SERVICE 2: Path Planning (Medium Priority)
        # ═══════════════════════════════════════════════════════════
        Node(
            package='trashbot_rtes',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
            prefix='xterm -e',  # Optional: open in separate terminal
        ),
        
        # ═══════════════════════════════════════════════════════════
        # SERVICE 3: Motor Control & Logging (Lowest Priority)
        # ═══════════════════════════════════════════════════════════
        Node(
            package='trashbot_rtes',
            executable='motor_control_node',
            name='motor_control',
            output='screen',
            parameters=[{
                'serial_port_front': LaunchConfiguration('serial_port_front'),
                'serial_port_rear': LaunchConfiguration('serial_port_rear'),
            }],
            prefix='xterm -e',  # Optional: open in separate terminal
        ),
    ])
