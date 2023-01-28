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
