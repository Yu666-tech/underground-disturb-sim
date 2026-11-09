"""UWB disturbance + optional five-table recorder, alongside existing raw sources."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('query_service', default_value='/mock/scene/query_ray'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('output_dir', description='New output directory; never overwrite an old run'),
        Node(package='coal_uwb_disturbance', executable='uwb_node', output='screen',
             parameters=[{'use_sim_time': True,
                          'query_service': LaunchConfiguration('query_service'),
                          'config_file': LaunchConfiguration('config_file')}]),
        Node(package='coal_platform_bridge', executable='uwb_csv_recorder.py', output='screen',
             parameters=[{'output_dir': LaunchConfiguration('output_dir')}]),
    ])
