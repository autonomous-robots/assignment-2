import cv2
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from yolov8_ros2.yolov8 import YOLOv8

class PersonDetector(Node):
    def __init__(self, model_path: str):
        super().__init__("person_detector")
        self.bridge = CvBridge()
        self.sub_jpeg = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            callback=self.image_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self.pub_det = self.create_publisher(
            BoundingBox2DArray,
            "/detections",
            qos_profile=QoSProfile(
                depth=10,
            )
        )
        self.model = YOLOv8(model_path, conf_threshold=0.3, iou_threshold=0.5)

    def image_callback(self, message: CompressedImage) -> None:
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(buffer, flags=cv2.IMREAD_COLOR)
        bounding_boxes, scores, class_ids = self.model(image)
        detections = BoundingBox2DArray()
        biggest = 0.0
        for detection, score, class_id in zip(bounding_boxes, scores, class_ids):
            if class_id != 0:
                continue
            x1, y1, x2, y2 = detection.astype(int)
            detections.header = message.header
            area = (abs(x2-x1)//2)*(abs(y2-y1)//2)
            if  area > 100000 and area > biggest:
                biggest = area
                bbox = BoundingBox2D()
                bbox.center.position.x = float(x1 + ((x2 - x1)//2))
                bbox.center.position.y = float(y1 + ((y2 - y1)//2))
                bbox.size_x = float((x2 - x1)//2)
                bbox.size_y = float((x2 - x1)//2)
                if len(detections.boxes) == 0:
                    detections.boxes.append(bbox)
                else:
                    detections.boxes[0] = bbox
        self.pub_det.publish(detections)
        self.get_logger().info("Detected {} Persons".format(len(detections.boxes)))
            
        combined_img = self.model.draw_detections(image)
        cv2.namedWindow("Output", cv2.WINDOW_NORMAL)
        cv2.imshow("Output", combined_img)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            exit(1)