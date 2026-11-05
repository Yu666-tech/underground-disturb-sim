"""Attach only the real query service to an already running platform scene."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('coal_face_sim'))
    metadata = yaml.safe_load((share / 'config/scene_metadata.yaml').read_text())
    if metadata.get('source_frame') != 'world':
        raise ValueError('Only identity map/world is supported')
    scene = share / 'scenarios/coal_face_v1'
    return LaunchDescription([Node(
        package='coal_scene_query', executable='scene_query.py', output='screen',
        parameters=[{'use_sim_time': True, 'world_file': str(scene / 'worlds/coal_face.sdf'),
                     'models_root': str(scene / 'models'),
                     'scene_id': metadata['scene_id'], 'scene_version': metadata['scene_version']}])])
