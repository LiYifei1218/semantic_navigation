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
FRONTIER = 50         # Frontier cell

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

        # publisher for frontiers
        self.frontiers_pub = self.create_publisher(
            MarkerArray,
            '/frontiers',
            10
        )
        
        # Member variables
        self.pose = None
        self.occupancy_grid = None

        # # 10 second timer to set new goal
        # self.timer = self.create_timer(10, self.timer_callback)

    def pose_topic_callback(self, msg: PoseWithCovarianceStamped):
        self.pose = msg
        self.get_logger().info(f"Received x: {self.pose.pose.pose.position.x}, y: {self.pose.pose.pose.position.y}")

    def update_full_map(self, occupancy_grid: OccupancyGrid):
        self.occupancy_grid = occupancy_grid
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        self.get_logger().info(f"Received occupancy grid: width={width}, height={height}")

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

        # find frontiers
        frontiers = self.find_frontiers()
        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        # self.get_logger().info(f"Frontiers: {frontiers}")

        # # cluster frontiers
        # frontier_clusters = self.find_connected_frontiers(frontiers)
        # self.get_logger().info(f"Found {len(frontier_clusters)} frontier clusters")
        # # self.get_logger().info(f"Frontier clusters: {frontier_clusters}")

        # draw frontiers
        self.draw_frontiers(frontiers)

        # randomly select one frontier
        selected_frontier = random.choice(frontiers)
        self.get_logger().info(f"Selected frontier: {selected_frontier.x}, {selected_frontier.y}")

        # ask for new goal from user
        # new_goal = self.ask_goal_from_user()

        frontier_x = selected_frontier.x
        frontier_y = selected_frontier.y

        new_goal = (frontier_x, frontier_y)
        
        self.send_goal(new_goal[0], new_goal[1])

    # timer callback
    def timer_callback(self):
        self.get_logger().info("timeout, asking for new goal")
        # get random frontier
        frontiers = self.find_frontiers()
        self.get_logger().info(f"Found {len(frontiers)} frontiers")

        # # cluster frontiers
        # frontier_clusters = self.find_connected_frontiers(frontiers)
        # self.get_logger().info(f"Found {len(frontier_clusters)} frontier clusters")
        # # self.get_logger().info(f"Frontier clusters: {frontier_clusters}")

        # draw frontiers
        self.draw_frontiers(frontiers)

        # randomly select one frontier
        selected_frontier = random.choice(frontiers)
        self.get_logger().info(f"Selected frontier: {selected_frontier.x}, {selected_frontier.y}")

        frontier_x = selected_frontier.x
        frontier_y = selected_frontier.y

        new_goal = (frontier_x, frontier_y)

        self.send_goal(new_goal[0], new_goal[1])

    def find_frontiers(self):
        """
        Naively find frontier cells.
        A frontier cell is defined as a free cell (FREE_SPACE) that has at least one neighbor with NO_INFORMATION.
        Returns a list of Point objects in world coordinates.
        """
        frontiers = []
        if not self.occupancy_grid:
            return frontiers
        
        grid = self.occupancy_grid
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = grid.data

        for row in range(height):
            for col in range(width):
                index = row * width + col
                if data[index] == FREE_SPACE:
                    # Check the 8-connected neighbors
                    is_frontier = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            n_row = row + dy
                            n_col = col + dx
                            if 0 <= n_row < height and 0 <= n_col < width:
                                n_index = n_row * width + n_col
                                if data[n_index] == NO_INFORMATION:
                                    is_frontier = True
                                    break
                        if is_frontier:
                            break
                    if is_frontier:
                        # Convert grid indices to world coordinates (center of the cell)
                        world_x = origin_x + (col + 0.5) * resolution
                        world_y = origin_y + (row + 0.5) * resolution
                        frontiers.append(Point(x=world_x, y=world_y, z=0.0))
                        # change cell to FRONTIER
                        data[index] = FRONTIER
        return frontiers

    def draw_frontiers(self, frontiers):
        markers = MarkerArray()
        for frontier in frontiers:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = int(frontier.x * 1000 + frontier.y)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = frontier
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
            markers.markers.append(marker)
        self.frontiers_pub.publish(markers)

    def find_connected_frontiers(self, frontier_cells):
        """
        Find connected frontier regions using breadth-first search.
        Returns a list of frontier clusters, where each cluster is a list of Point objects.
        """

        if not self.occupancy_grid:
            return []

        #frontier_cells is a list of Point objects, convert to (row, col) tuples
        hashable_frontier_cells = [(cell.x, cell.y) for cell in frontier_cells]
    
        grid = self.occupancy_grid
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = grid.data

        # Group connected frontier cells using BFS
        visited = set()
        frontier_clusters = []
        
        for i in range(len(hashable_frontier_cells)):
            cell = hashable_frontier_cells[i]
            self.get_logger().info(f"Checking cell {i}: {cell}")
            if cell in visited:
                continue
            
            # Start a new cluster
            cluster = []
            queue = [cell]
            visited.add(cell)
            
            while queue:
                current = queue.pop(0)
                row, col = current
                
                # Convert to world coordinates
                world_x = origin_x + (col + 0.5) * resolution
                world_y = origin_y + (row + 0.5) * resolution
                cluster.append(Point(x=world_x, y=world_y, z=0.0))
                
                # Add neighbors to queue (8-connected)
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        n_row = row + dy
                        n_col = col + dx
                        neighbor = (n_row, n_col)
                        if neighbor in frontier_cells and neighbor not in visited:
                            queue.append(neighbor)
                            visited.add(neighbor)
            
            if cluster:  # Only add non-empty clusters
                frontier_clusters.append(cluster)
        
        return frontier_clusters

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
