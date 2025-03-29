import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import onnxruntime as ort  # Import ONNX Runtime for inference
import time


class ZED2iImageViewer(Node):
    def __init__(self):
        super().__init__('zed2i_image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/zed2i/zed_node/left/image_rect_color/compressed',
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to /zed2i/zed_node/left/image_rect_color/compressed")

        # Load the ONNX model
        self.model_path = "/workspace/DevEnv/jazzy_ws/src/robot2025/best.onnx"  # Path to your ONNX model
        self.session = ort.InferenceSession(self.model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.get_logger().info(f"Loaded ONNX model: {self.model_path}")

    def preprocess_image(self, image, input_size):
        """Preprocess the image for the ONNX model."""
        image = cv2.resize(image, (input_size, input_size))  # Resize to model input size
        image = image.astype(np.float32) / 255.0  # Normalize to [0, 1]
        image = np.transpose(image, (2, 0, 1))  # HWC to CHW
        image = np.expand_dims(image, axis=0)  # Add batch dimension
        return image

    def draw_results(self, image, results):
        """Draw inference results on the image."""
        try:
            # Extract the first output tensor and reshape it
            detections = results[0].squeeze()  # Remove batch dimension, shape becomes (5, 8400)
            if detections.shape[0] != 5:
                self.get_logger().error(f"Unexpected result shape: {detections.shape}")
                return

            # Extract bounding boxes, confidence scores, and filter by confidence threshold
            boxes = detections[:4, :]  # First 4 rows are bounding box coordinates
            confidences = detections[4, :]  # 5th row is confidence scores

            # Apply confidence threshold
            threshold = 0.5
            indices = np.where(confidences > threshold)[0]  # Indices of detections above threshold
            boxes = boxes[:, indices]
            confidences = confidences[indices]

            # Scale boxes back to the original image size
            h, w, _ = image.shape
            boxes[0, :] *= w  # Scale x1
            boxes[1, :] *= h  # Scale y1
            boxes[2, :] *= w  # Scale x2
            boxes[3, :] *= h  # Scale y2

            # Draw each detection
            for i in range(boxes.shape[1]):
                x1, y1, x2, y2 = boxes[:, i].astype(int)
                confidence = confidences[i]

                # Draw bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Add label and confidence
                label = f"Confidence: {confidence:.2f}"
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        except Exception as e:
            self.get_logger().error(f"Error drawing results: {e}")
    def image_callback(self, msg):
        print("Received image")
        try:
            # Convert the compressed image data to a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode the numpy array into an OpenCV image
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Preprocess the image for the ONNX model
            input_size = 640  # Replace with your model's input size
            input_data = self.preprocess_image(image, input_size)

            # Run inference
            results = self.session.run([self.output_name], {self.input_name: input_data})

            # Draw results on the image
            self.draw_results(image, results)

            # Display the image
            cv2.imshow("ZED2i Stereo Image", image)
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
        cv2.destroyAllWindows()
        #wait for 3 seconds
        time.sleep(3)
        node.destroy_node()
        rclpy.shutdown()
        # Destroy all OpenCV windows
        


if __name__ == '__main__':
    main()