import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for MAVLink communication'
        ),
        
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        
        DeclareLaunchArgument(
            'system_id',
            default_value='255',
            description='MAVLink system ID'
        ),
        
        DeclareLaunchArgument(
            'component_id',
            default_value='1',
            description='MAVLink component ID'
        ),
        
        DeclareLaunchArgument(
            'target_system_id',
            default_value='1',
            description='Target MAVLink system ID'
        ),
        
        DeclareLaunchArgument(
            'target_component_id',
            default_value='1',
            description='Target MAVLink component ID'
        ),
        
        Node(
            package='mavlink_gui_tester',
            executable='mavlink_gui_tester_node',
            name='mavlink_gui_tester',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'system_id': LaunchConfiguration('system_id'),
                'component_id': LaunchConfiguration('component_id'),
                'target_system_id': LaunchConfiguration('target_system_id'),
                'target_component_id': LaunchConfiguration('target_component_id'),
            }]
        )
    ])