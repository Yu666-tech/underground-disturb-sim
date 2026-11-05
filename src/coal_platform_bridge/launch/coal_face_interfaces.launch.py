from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scene = Path(get_package_share_directory('coal_face_sim'))
    bridge = Path(get_package_share_directory('coal_platform_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scene/'launch/coal_face_full.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge/'launch/bridge.launch.py'))),
    ])
