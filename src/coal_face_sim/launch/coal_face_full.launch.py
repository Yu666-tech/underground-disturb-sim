"""Full-scale V0.2 scene with ideal raw sensors and independent Gazebo truth."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('coal_face_sim'))
    scene = share/'scenarios/coal_face_v1'
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', str(scene/'models')),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', str(Path(get_package_prefix('coal_face_sim'))/'lib') + os.pathsep + os.environ.get('GAZEBO_PLUGIN_PATH', '')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gazebo.launch.py')),
            launch_arguments={'world': str(scene/'worlds/coal_face.sdf'),
                'gui': LaunchConfiguration('gui'), 'params_file': str(share/'config/gazebo.yaml')}.items()),
        Node(package='coal_face_sim', executable='shearer_truth_tf.py', output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='coal_face_sim', executable='shearer_imu.py', output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/coal_face_lidar.rviz')],
            parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
