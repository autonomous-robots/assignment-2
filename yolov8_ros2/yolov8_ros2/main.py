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

import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from yolov8_ros2.person_detector import PersonDetector


def main(args=None):
    rclpy.init(args=args)
    detector = PersonDetector(model_path=sys.argv[1])
    try:
        rclpy.spin(detector)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        detector.stop()
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
