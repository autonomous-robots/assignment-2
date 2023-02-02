import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_file_dir_custom = os.path.join(get_package_share_directory('custom_worlds'), 'launch')
    launch_file_dir_random = os.path.join(get_package_share_directory('turtlebot3_random_bouncer'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-1.0')
    headless = LaunchConfiguration('headless', default='False')
    custom_worlds_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir_custom, 'custom_worlds.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items()
    )
    random_bouncer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir_random, 'turtlebot3_random_bouncer.launch.py')
        ),
    )
    ld = LaunchDescription()
    ld.add_action(custom_worlds_cmd)
    ld.add_action(random_bouncer_cmd)
    return ld