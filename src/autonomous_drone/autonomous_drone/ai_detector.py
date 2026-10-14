import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class AIDetector(Node):
    def __init__(self):
        super().__init__('ai_detector')

        # Initialize image conversion
        self.bridge = CvBridge()

        # Load the lightweight YOLO11 nano model
        self.get_logger().info('Loading YOLO11n model...')
        self.model = YOLO('yolo11n.pt')
        self.get_logger().info('YOLO model loaded successfully.')

        # Subscribe to the existing camera node
        self.subscription = self.create_subscription(
            Image,
            '/camera/processed',
            self.image_callback,
            10
        )

        # Publish images containing detection annotations
        self.publisher = self.create_publisher(
            Image,
            '/camera/detections/image',
            10
        )

        self.get_logger().info('AI detector node started.')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV BGR format
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Run YOLO inference
            results = self.model.predict(
                source=frame,
                imgsz=320,
                conf=0.4,
                verbose=False
            )

            # Draw bounding boxes and labels
            annotated_frame = results[0].plot()

            # Publish the annotated image
            output_msg = self.bridge.cv2_to_imgmsg(
                annotated_frame,
                encoding='bgr8'
            )

            output_msg.header = msg.header
            self.publisher.publish(output_msg)

        except Exception as error:
            self.get_logger().error(
                f'Image processing failed: {error}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = AIDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
