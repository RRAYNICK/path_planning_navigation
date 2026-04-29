#!/usr/bin/env python3
"""A* path planner for ROS2 Jazzy with heuristic guidance."""

from queue import PriorityQueue

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from tf2_ros import Buffer, LookupException, TransformListener


class AStarNode:
    """Represents a node in the A* search with cost + heuristic."""

    def __init__(self, x, y, cost=0, heuristic=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost  # g(n): cost from start
        self.heuristic = heuristic  # h(n): estimated cost to goal
        self.prev = prev

    def total_cost(self):
        """f(n) = g(n) + h(n)"""
        return self.cost + self.heuristic

    def __lt__(self, other):
        return self.total_cost() < other.total_cost()

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return AStarNode(self.x + other[0], self.y + other[1])


class AStarPlanner(Node):
    """A* planner with Manhattan heuristic for grid-based navigation."""

    def __init__(self):
        super().__init__("a_star_planner")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_ = None

        self.path_pub = self.create_publisher(Path, "/a_star/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/a_star/visited_map", 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        self.get_logger().info("A* Planner Node Started")

    def map_callback(self, msg):
        self.map_ = msg

    def manhattan_distance(self, node, goal_node):
        """Admissible heuristic for grid-based movement."""
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y)

    def world_to_grid(self, pose):
        x = int(
            (pose.position.x - self.map_.info.origin.position.x)
            / self.map_.info.resolution
        )
        y = int(
            (pose.position.y - self.map_.info.origin.position.y)
            / self.map_.info.resolution
        )
        return AStarNode(x, y)

    def grid_to_world(self, node):
        pose = Pose()
        pose.position.x = (
            node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        )
        pose.position.y = (
            node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        )
        return pose

    def is_valid(self, node):
        if (
            node.x < 0
            or node.x >= self.map_.info.width
            or node.y < 0
            or node.y >= self.map_.info.height
        ):
            return False
        idx = node.y * self.map_.info.width + node.x
        return self.map_.data[idx] == 0

    def goal_callback(self, msg):
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

        path = self.plan(start_pose, msg.pose)

        if path.poses:
            self.get_logger().info("A* path found!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found")

    def plan(self, start, goal):
        """Execute A* path planning with Manhattan heuristic."""
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        if not self.is_valid(start_node) or not self.is_valid(goal_node):
            return Path()

        open_set = PriorityQueue()
        start_node.heuristic = self.manhattan_distance(start_node, goal_node)
        open_set.put((start_node.total_cost(), start_node))
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
                    neighbor.heuristic = self.manhattan_distance(neighbor, goal_node)
                    neighbor.prev = current
                    open_set.put((neighbor.total_cost(), neighbor))

        # Reconstruct path
        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        if goal_node not in cost_so_far:
            return path

        node = goal_node
        waypoints = []
        while node and node.prev:
            waypoints.append(self.grid_to_world(node))
            node = node.prev
        waypoints.reverse()

        for pose in waypoints:
            wp = PoseStamped()
            wp.header.frame_id = self.map_.header.frame_id
            wp.pose = pose
            path.poses.append(wp)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
