import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.client = InferenceHTTPClient(
            api_url="http://localhost:9001",  # use local inference server
            api_key="ojKXeQ73UvdQRUcAGxVU"
        )
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每秒调用一次

    def image_callback(self, msg):
        self.get_logger().info('Receiving image')
        try:
            # Convert ROS Image message to OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Camera Image", self.cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def timer_callback(self):
        if hasattr(self, 'cv_image'):
            # Save the current image to a file
            cv2.imwrite('current_image.jpg', self.cv_image)
            # Run inference on the saved image
            result = self.client.run_workflow(
                workspace_name="rmcv",
                workflow_id="detect-and-classify",
                images={
                    "image": "current_image.jpg"
                }
            )
            self.get_logger().info(f"Inference result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()