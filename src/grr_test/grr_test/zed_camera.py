import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class ZED2iImageViewer(Node):
    def __init__(self):
        super().__init__('zed2i_image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/zed2i/zed_node/left/image_rect_color/compressed',
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to /zed2i/zed_node/stereo/image_rect_color/compressed")

    def image_callback(self, msg):
        try:
            # Convert the compressed image data to a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode the numpy array into an OpenCV image
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Display the image in a window
            cv2.imshow("ZED2i Stereo Image", image)
            # Wait for a key press for 1ms
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ZED2iImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down image viewer")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Destroy all OpenCV windows
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()