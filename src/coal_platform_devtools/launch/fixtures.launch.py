from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('phase_duration_s', default_value='4.0'),
        Node(package='coal_platform_devtools', executable='fixture_inputs.py', output='screen',
             parameters=[{'phase_duration_s': LaunchConfiguration('phase_duration_s')}])])
