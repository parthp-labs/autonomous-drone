import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')

        # Convert between OpenCV images and ROS 2 image messages
        self.bridge = CvBridge()

        # Open the USB camera directly using OpenCV
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error(
                'Could not open camera. Check the camera connection '
                'and make sure another process is not using it.'
            )
            raise RuntimeError('Failed to open camera')

        # Optional camera configuration
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Publish processed grayscale images
        self.publisher = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

        # Capture and process frames periodically
        self.timer = self.create_timer(0.033, self.process_frame)

        self.get_logger().info(
            'Camera opened successfully using OpenCV.'
        )

    def process_frame(self):
        # Capture a frame directly from the webcam
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning(
                'Failed to capture camera frame.'
            )
            return

        try:
            # Convert the color frame to grayscale
            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )

            # Convert the OpenCV image to a ROS 2 message
            msg = self.bridge.cv2_to_imgmsg(
                gray,
                encoding='mono8'
            )

            # Assign timestamp and camera frame ID
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'

            # Publish the processed image
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(
                f'Image processing failed: {e}'
            )

    def destroy_node(self):
        # Release the webcam when the node shuts down
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = CameraProcessor()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f'Camera processor stopped: {e}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
