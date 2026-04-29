#!/usr/bin/env python3
"""Dijkstra path planner for ROS2 Jazzy."""

from queue import PriorityQueue

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from tf2_ros import Buffer, LookupException, TransformListener


class GraphNode:
    """Represents a node in the grid graph."""

    def __init__(self, x, y, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])


class DijkstraPlanner(Node):
    """Dijkstra path planner node."""

    def __init__(self):
        super().__init__("dijkstra_planner")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_ = None

        # Publishers and subscribers
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        self.get_logger().info("Dijkstra Planner Node Started")

    def map_callback(self, msg):
        """Store the occupancy grid map."""
        self.map_ = msg

    def goal_callback(self, msg):
        """Handle goal pose callback and compute path."""
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_link", rclpy.time.Time()
            )
        except LookupException:
            self.get_logger().error("Could not transform from map to base_link")
            return

        start_pose = Pose()
        start_pose.position.x = transform.transform.translation.x
        start_pose.position.y = transform.transform.translation.y

        self.get_logger().info(
            f"Goal received: ({msg.pose.position.x:.2f}, " f"{msg.pose.position.y:.2f})"
        )
        path = self.plan(start_pose, msg.pose)

        if path.poses:
            self.get_logger().info("Shortest path found!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to goal.")

    def world_to_grid(self, pose):
        """Convert world pose to grid coordinates."""
        x = int(
            (pose.position.x - self.map_.info.origin.position.x)
            / self.map_.info.resolution
        )
        y = int(
            (pose.position.y - self.map_.info.origin.position.y)
            / self.map_.info.resolution
        )
        return GraphNode(x, y)

    def grid_to_world(self, node):
        """Convert grid coordinates to world pose."""
        pose = Pose()
        pose.position.x = (
            node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        )
        pose.position.y = (
            node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        )
        return pose

    def is_valid(self, node):
        """Check if node is within map bounds and not an obstacle."""
        if (
            node.x < 0
            or node.x >= self.map_.info.width
            or node.y < 0
            or node.y >= self.map_.info.height
        ):
            return False
        idx = node.y * self.map_.info.width + node.x
        return self.map_.data[idx] == 0  # 0 = free space

    def plan(self, start, goal):
        """Execute Dijkstra path planning algorithm."""
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        if not self.is_valid(start_node) or not self.is_valid(goal_node):
            self.get_logger().error("Start or goal in obstacle")
            return Path()

        # Initialize data structures
        open_set = PriorityQueue()
        start_node.cost = 0
        open_set.put((start_node.cost, start_node))
        visited = set()
        cost_so_far = {start_node: 0}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal_node:
                break

            if current in visited:
                continue
            visited.add(current)

            for dx, dy in directions:
                neighbor = current + (dx, dy)
                if neighbor in visited or not self.is_valid(neighbor):
                    continue

                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    neighbor.cost = new_cost
                    neighbor.prev = current
                    open_set.put((new_cost, neighbor))

        # Reconstruct path
        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        if goal_node not in cost_so_far:
            return path

        node = goal_node
        while node and node.prev:
            pose = self.grid_to_world(node)
            waypoint = PoseStamped()
            waypoint.header.frame_id = self.map_.header.frame_id
            waypoint.pose = pose
            path.poses.append(waypoint)
            node = node.prev
        path.poses.reverse()
        return path


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
