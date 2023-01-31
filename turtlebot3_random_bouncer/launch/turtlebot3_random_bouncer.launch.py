from launch_ros.actions import Node
from launch import LaunchDescription

        
def generate_launch_description():
    start_random_bouncer_cmd = Node(
        package='turtlebot3_random_bouncer',
        executable='random_bouncer',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_random_bouncer_cmd)
    return ld