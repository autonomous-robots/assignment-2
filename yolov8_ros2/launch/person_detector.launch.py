import os

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the urdf file
    onnx_path = os.path.join(
        get_package_share_directory('yolov8_ros2'),
        'yolov8n.onnx'
    )

    start_person_detector_cmd = Node(
        package='yolov8_ros2',
        executable='person_detector',
        arguments=[
            onnx_path,
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_person_detector_cmd)
    return ld