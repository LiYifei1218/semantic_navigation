import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import cv2
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient
import numpy as np
import warnings
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
# Suppress Matplotlib 3D import warning
warnings.filterwarnings("ignore", category=UserWarning, message="Unable to import Axes3D")
print(np.__version__)

class ImageLidarSubscriber(Node):
    def __init__(self):
        super().__init__('image_lidar_subscriber')
        
        # Image Subscriber
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw/compressed',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # LiDAR Subscriber
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.lidar_data = None  # Store LiDAR scan data
        
        # TF Buffer and Listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Object Position Publisher
        self.object_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        
        # Inference Client
        self.client = InferenceHTTPClient(
            api_url="https://detect.roboflow.com",  # Local inference server
            api_key="ojKXeQ73UvdQRUcAGxVU"
        )
        
        # Timer for inference
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

    
    def image_callback(self, msg):
        self.get_logger().info('Receiving image')
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Camera Image", self.cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    def lidar_callback(self, msg):
        self.lidar_data = msg.ranges  # Store LiDAR distance data
    
    def timer_callback(self):
        if hasattr(self, 'cv_image'):
            cv2.imwrite('current_image.jpg', self.cv_image)
            result_list = self.client.run_workflow(
                workspace_name="rmcv",
                workflow_id="custom-workflow-2",
                images={"image": "current_image.jpg"}
            )
            
            if isinstance(result_list, list) and len(result_list) > 0 and 'predictions' in result_list[0]:
                result = result_list[0]
                self.get_logger().info(f"Inference result: {result}")
                self.process_inference_result(result)
            else:
                self.get_logger().info("No valid predictions received from inference.")
    
    def process_inference_result(self, result):
        predictions = result.get('predictions', {}).get('predictions', [])
        image_width = result['predictions']['image']['width']

        if predictions:
            for detected_obj in predictions:
                center_x = detected_obj['x']
                width_px = detected_obj['width']
                object_class = detected_obj['class']
                confidence = detected_obj['confidence']

                lidar_angle = self.map_pixel_to_lidar_angle(center_x, image_width)

                if self.lidar_data:
                    CAMERA_FOV = 60.0  # Ensure this matches your actual camera FOV
                    angle_width = (width_px / image_width) * CAMERA_FOV
                    lidar_len = len(self.lidar_data)
                    lidar_center_idx = int((lidar_angle % 360) / 360.0 * lidar_len)
                    lidar_range = max(int((angle_width / 360.0) * lidar_len), 1)

                    indices = [i % lidar_len for i in range(lidar_center_idx - lidar_range, lidar_center_idx + lidar_range + 1)]
                    values = [self.lidar_data[i] for i in indices if 0.0 < self.lidar_data[i] < float('inf')]

                    if values:
                        distance = sum(values) / len(values)
                        world_position = self.transform_to_world(lidar_angle, distance)
                        if world_position:
                            self.publish_object_pose(object_class, world_position)
                            self.get_logger().info(f"Published: {object_class} at {world_position}")
                    else:
                        self.get_logger().info(f"No valid LiDAR data found at angle: {lidar_angle:.2f} degrees for {object_class}")
                else:
                    self.get_logger().info("LiDAR data not yet available.")
        else:
            self.get_logger().info("No predictions found in inference results.")


    def map_pixel_to_lidar_angle(self, pixel_x, image_width):
        CAMERA_FOV = 60.0
        camera_angle = (pixel_x / image_width - 0.5) * CAMERA_FOV
        lidar_angle = 180.0 - camera_angle
        return lidar_angle
    
    def get_robot_pose(self):
        try:
            # Get the latest transform available
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_scan',
                rclpy.time.Time(seconds=0),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            x_robot = t.transform.translation.x
            y_robot = t.transform.translation.y

            quat = t.transform.rotation
            r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            _, _, yaw_robot = r.as_euler('xyz')

            return x_robot, y_robot, yaw_robot

        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def transform_to_world(self, lidar_angle_deg, distance):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None

        x_robot, y_robot, yaw_robot = robot_pose

        # Convert lidar angle to radians
        lidar_angle_rad = np.radians(lidar_angle_deg)

        # Relative position of object in robot frame (forward: x, left: y)
        x_rel = distance * np.cos(lidar_angle_rad)
        y_rel = distance * np.sin(lidar_angle_rad)

        # Global position of object
        x_world = x_robot - (x_rel * np.cos(yaw_robot) - y_rel * np.sin(yaw_robot))
        y_world = y_robot - (x_rel * np.sin(yaw_robot) + y_rel * np.cos(yaw_robot))

        return (x_world, y_world)


    def publish_object_pose(self, object_class, world_position):
    # Marker for the object (e.g., Sphere)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = object_class
        marker.id = hash(object_class) % 1000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = world_position[0]
        marker.pose.position.y = world_position[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

        # Marker for the text label
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = object_class + "_label"
        text_marker.id = (hash(object_class) + 1) % 1000  # ensure unique ID
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = world_position[0]
        text_marker.pose.position.y = world_position[1]
        text_marker.pose.position.z = 0.3  # Slightly above the object marker
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2  # text height
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = object_class  # The label text
        text_marker.lifetime.sec = 0
        self.marker_pub.publish(text_marker)

        self.get_logger().info(
            f"Published Marker and Label: {object_class} at ({world_position[0]}, {world_position[1]})"
        )



def main(args=None):
    rclpy.init(args=args)
    node = ImageLidarSubscriber()
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
