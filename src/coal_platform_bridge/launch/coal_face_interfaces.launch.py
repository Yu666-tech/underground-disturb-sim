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
        DeclareLaunchArgument('future_tolerance_ms', default_value='50.0'),
        DeclareLaunchArgument('publish_timing_diagnostics', default_value='false'),
        DeclareLaunchArgument('bc_inputs_config_file', default_value=str(bridge/'config/bc_inputs.json')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scene/'launch/coal_face_full.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge/'launch/bridge.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in
                ('future_tolerance_ms', 'publish_timing_diagnostics', 'bc_inputs_config_file')}.items()),
    ])
