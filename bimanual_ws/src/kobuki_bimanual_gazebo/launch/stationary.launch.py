"""Launch the rotation-only, no-navigation three-object diagnostic."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('kobuki_bimanual_gazebo')
    pkg_behavior = get_package_share_directory('kobuki_bimanual_behavior')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': 'stationary',
            'mission_mode': 'stationary',
            'camera_pitch': '0.785398',
            'params_file': os.path.join(
                pkg_behavior, 'config', 'stationary_params.yaml'),
        }.items())

    return LaunchDescription([base_launch])
