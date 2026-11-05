from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('coal_face_sim'))/'config/scene_metadata.yaml'
    metadata = yaml.safe_load(config.read_text())
    if metadata.get('source_frame') != 'world':
        raise ValueError('This adapter supports identity map/world only')
    return LaunchDescription([
        DeclareLaunchArgument('scene_id', default_value=metadata['scene_id']),
        DeclareLaunchArgument('scene_version', default_value=metadata['scene_version']),
        DeclareLaunchArgument('query_world_file', default_value=str(Path(get_package_share_directory('coal_face_sim'))/'scenarios/coal_face_v1/worlds/coal_face.sdf')),
        Node(package='coal_platform_bridge', executable='platform_bridge.py', output='screen',
             parameters=[{'use_sim_time': True, 'scene_id': LaunchConfiguration('scene_id'),
                          'scene_version': LaunchConfiguration('scene_version'),
                          'robot_id': metadata['robot_id'], 'sensor_id': metadata['sensor_id']}]),
        Node(package='coal_platform_bridge', executable='bc_inputs.py', output='screen',
             parameters=[{'use_sim_time': True, 'config_file': str(Path(get_package_share_directory('coal_platform_bridge'))/'config/bc_inputs.json')}]),
        Node(package='coal_scene_query', executable='scene_query.py', output='screen',
             parameters=[{'use_sim_time': True, 'world_file': LaunchConfiguration('query_world_file'),
                          'models_root': str(Path(get_package_share_directory('coal_face_sim'))/'scenarios/coal_face_v1/models'),
                          'scene_id': LaunchConfiguration('scene_id'), 'scene_version': LaunchConfiguration('scene_version')}]),
    ])
