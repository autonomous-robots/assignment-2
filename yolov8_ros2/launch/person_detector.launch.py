# Copyright 2023 Luiz Carlos Cosmi Filho and others.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
