import random
import rclpy
from rclpy.node import Node
import rclpy.action
from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Standard occupancy grid values
FREE_SPACE = 0         # Free cell
NO_INFORMATION = -1    # Unknown cell
OBSTACLE = 100         # Occupied cell

class Explorer(Node):
    def __init__(self):
        super().__init__('frontier_exploration')
        self.get_logger().info("Exploration started...")

        # Subscriptions for robot pose and map
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_topic_callback,
            10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.update_full_map,
            10
        )


        # Action client for navigation goals using the NavigateToPose action
        self._action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Member variables
        self.pose = None
        self.occupancy_grid = None

    def pose_topic_callback(self, msg: PoseWithCovarianceStamped):
        self.pose = msg
        self.get_logger().info(f"Received x: {self.pose.pose.pose.position.x}, y: {self.pose.pose.pose.position.y}")

    def update_full_map(self, occupancy_grid: OccupancyGrid):
        self.occupancy_grid = occupancy_grid
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        self.get_logger().info(f"Received occupancy grid: width={width}, height={height}")
        self.explore()

    # Helper functions for coordinate conversion
    def get_index(self, mx, my):
        return mx + my * self.occupancy_grid.info.width

    def index_to_cells(self, index):
        width = self.occupancy_grid.info.width
        my = index // width
        mx = index % width
        return mx, my

    def map_to_world(self, mx, my):
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        wx = origin_x + mx * resolution
        wy = origin_y + my * resolution
        return wx, wy

    def world_to_map(self, wx, wy):
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        mx = int((wx - origin_x) / resolution)
        my = int((wy - origin_y) / resolution)
        if mx < 0 or mx >= self.occupancy_grid.info.width or my < 0 or my >= self.occupancy_grid.info.height:
            return False, None, None
        return True, mx, my

    def find_random_unknown_point(self):
        """Find a random unknown point in the map"""
        if self.pose is None or self.occupancy_grid is None:
            return None

        # Get current robot position
        position = self.pose.pose.pose.position
        self.get_logger().info(f"Current position: {position.x}, {position.y}")
        
        # Parameters for search
        MAX_ATTEMPTS = 100
        MIN_DISTANCE = 1.0  # Min distance from current position in meters
        MAX_DISTANCE = 5.0  # Max distance from current position in meters
        
        cmap = self.occupancy_grid.data
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height

    def ask_goal_from_user(self):
        self.get_logger().info("Please enter the goal x and y coordinates:")
        goal_x = float(input("Enter x coordinate: "))
        goal_y = float(input("Enter y coordinate: "))
        return goal_x, goal_y

    def send_goal(self, goal_x, goal_y):
        # Create a new goal message for the action
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Waiting for action server to become available...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server available, sending goal.")

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        # Optionally, ask for a new goal once the current one finishes
        new_goal = self.ask_goal_from_user()
        self.send_goal(new_goal[0], new_goal[1])

    # def draw_markers(self, point):
    #     """Visualize exploration point in RViz"""
    #     markers = []
    #     if point:
    #         self.get_logger().info(f"Visualising exploration point at: {point.x}, {point.y}")
    #         marker = Marker()
    #         marker.header.frame_id = "map"
    #         marker.header.stamp = self.get_clock().now().to_msg()
    #         marker.ns = "exploration_points"
    #         self.marker_id += 1
    #         marker.id = self.marker_id
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
    #         marker.pose.position = point
    #         marker.pose.orientation.w = 1.0
    #         marker.scale.x = 0.3
    #         marker.scale.y = 0.3
    #         marker.scale.z = 0.3
    #         marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    #         markers.append(marker)
    #     self.markers_msg.markers = markers
    #     self.marker_array_pub.publish(self.markers_msg)

    # def clear_markers(self):
    #     """Clear all markers"""
    #     for marker in self.markers_msg.markers:
    #         marker.action = Marker.DELETE
    #     self.marker_array_pub.publish(self.markers_msg)
    #     self.markers_msg.markers.clear()

def main(args=None):
    rclpy.init(args=args)

    explorer = Explorer()

    # Request an initial goal from the user and send it
    goal_x, goal_y = explorer.ask_goal_from_user()
    explorer.send_goal(goal_x, goal_y)

    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
