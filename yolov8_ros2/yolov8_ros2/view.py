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

import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class DetectionsRender(Node):
    def __init__(self):
        super().__init__("detections_render")
        self.bridge = CvBridge()
        self.sub_jpeg = self.create_subscription(
            CompressedImage,
            "/rendered",
            callback=self.image_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

    def image_callback(self, message: CompressedImage) -> None:
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(buffer, flags=cv2.IMREAD_COLOR)
        cv2.namedWindow("Output", cv2.WINDOW_NORMAL)
        cv2.imshow("Output", image)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            exit(1)


def main(args=None):
    rclpy.init(args=args)
    render = DetectionsRender()
    try:
        rclpy.spin(render)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        render.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
