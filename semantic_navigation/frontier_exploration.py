import random
import rclpy
from rclpy.node import Node

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
        self.get_logger().info("exploration started...")

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

        # Publisher for visualizing exploration points in RViz
        self.marker_array_pub = self.create_publisher(MarkerArray, '/frontiers', 10)
        self.markers_msg = MarkerArray()
        self.marker_id = 0

        # Publisher for navigation goals
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Parameter for map saving path (if needed)
        self.declare_parameter("map_path", "")
        self.map_path = self.get_parameter("map_path").get_parameter_value().string_value

        # Member variables
        self.pose = None
        self.occupancy_grid = None
        self.is_exploring = False
        
        # Create a timer to continue exploration periodically
        self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        """Timer callback to continue exploration if not currently exploring"""
        if not self.is_exploring:
            self.explore()
            
    def pose_topic_callback(self, msg: PoseWithCovarianceStamped):
        self.pose = msg
        self.get_logger().info("Received new pose.")

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

        self.get_logger().info(f"starting search for unknown point")
        
        for i in range(MAX_ATTEMPTS):
            self.get_logger().info(f"attempt {i} of {MAX_ATTEMPTS}")
            # Select a random index in the map
            idx = random.randint(0, len(cmap) - 1)
            
            # Check if it's an unknown cell
            if cmap[idx] == NO_INFORMATION:
                # Convert to map coordinates
                mx, my = self.index_to_cells(idx)
                
                # Check if it's near a free space (so it's accessible)
                has_free_neighbor = False
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    nx, ny = mx + dx, my + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        neighbor_idx = self.get_index(nx, ny)
                        if cmap[neighbor_idx] == FREE_SPACE:
                            has_free_neighbor = True
                            break
                
                if has_free_neighbor:
                    # Convert to world coordinates
                    wx, wy = self.map_to_world(mx, my)
                    
                    # Check distance from robot
                    dist = ((wx - position.x)**2 + (wy - position.y)**2)**0.5
                    if MIN_DISTANCE <= dist <= MAX_DISTANCE:
                        point = Point(x=wx, y=wy, z=0.0)
                        return point
        
        self.get_logger().warn("Could not find suitable unknown point after MAX_ATTEMPTS")
        return None

    def draw_markers(self, point):
        """Visualize exploration point in RViz"""
        markers = []
        if point:
            self.get_logger().info(f"Visualising exploration point at: {point.x}, {point.y}")
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "exploration_points"
            self.marker_id += 1
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            markers.append(marker)
        self.markers_msg.markers = markers
        self.marker_array_pub.publish(self.markers_msg)

    def clear_markers(self):
        """Clear all markers"""
        for marker in self.markers_msg.markers:
            marker.action = Marker.DELETE
        self.marker_array_pub.publish(self.markers_msg)
        self.markers_msg.markers.clear()

    def explore(self):
        """Find a random unknown point and navigate to it"""
        # if self.is_exploring or self.pose is None or self.occupancy_grid is None:
        #     return

        self.get_logger().info(f"exploring...")
        # Find a random unknown point
        point = self.find_random_unknown_point()
        if not point:
            self.get_logger().warn("No exploration points found!")
            return

        # Visualize the point
        self.draw_markers(point)
        
        # Create a PoseStamped message for the goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position = point
        goal_msg.pose.orientation.w = 1.0

        self.get_logger().info(f"Publishing goal at: {point.x}, {point.y}")
        self.goal_publisher.publish(goal_msg)
        self.is_exploring = True


    def stop(self):
        """Stop exploration and clean up"""
        self.get_logger().info("Stopped exploring.")
        self.pose_subscription.destroy()
        self.map_subscription.destroy()
        self.save_map()
        self.clear_markers()

def main(args=None):
    rclpy.init(args=args)
   