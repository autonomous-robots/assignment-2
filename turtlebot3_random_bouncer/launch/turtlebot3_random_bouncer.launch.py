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
