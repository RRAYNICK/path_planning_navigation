#!/usr/bin/env python3
"""Dynamic obstacle manager for Gazebo Harmonic."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DynamicObstacleManager(Node):
    """Manages dynamic obstacles in Gazebo simulation."""

    def __init__(self):
        super().__init__("dynamic_obstacle_manager")
        # Use a different name to avoid conflict with Node.publishers property
        self.obstacle_pubs = {}

        # Directly define obstacles (no ROS parameters to avoid type issues)
        self.obstacles = {
            "obstacle_1": {
                "name": "obstacle_1",
                "start_x": 2.0,
                "start_y": 2.0,
                "pattern": "linear",
                "speed": 0.3,
                "pattern_func": self._move_linear,
            },
            "obstacle_2": {
                "name": "obstacle_2",
                "start_x": -1.0,
                "start_y": 1.0,
                "pattern": "circular",
                "speed": 0.2,
                "pattern_func": self._move_circular,
            },
            "obstacle_3": {
                "name": "obstacle_3",
                "start_x": 1.0,
                "start_y": -2.0,
                "pattern": "zigzag",
                "speed": 0.4,
                "pattern_func": self._move_zigzag,
            },
        }

        # Create timers for updating obstacle positions
        self.timer = self.create_timer(0.05, self.update_obstacles)
        self.get_logger().info("Dynamic Obstacle Manager Started")

    def _move_linear(self, name, t, speed, start_x, start_y):
        """Linear back-and-forth movement."""
        x = start_x + speed * math.sin(t * 0.5) * 3.0
        y = start_y + speed * math.cos(t * 0.5) * 2.0
        return x, y

    def _move_circular(self, name, t, speed, start_x, start_y):
        """Circular movement pattern."""
        radius = 1.5
        x = start_x + radius * math.cos(t * speed)
        y = start_y + radius * math.sin(t * speed)
        return x, y

    def _move_zigzag(self, name, t, speed, start_x, start_y):
        """Zigzag movement pattern."""
        x = start_x + speed * t * 0.5
        y = start_y + 0.5 * math.sin(t * 2.0)
        return x, y

    def update_obstacles(self):
        """Update positions of all dynamic obstacles."""
        t = self.get_clock().now().nanoseconds / 1e9
        for obs in self.obstacles.values():
            x, y = obs["pattern_func"](
                obs["name"], t, obs["speed"], obs["start_x"], obs["start_y"]
            )
            self._update_obstacle_pose(obs["name"], x, y)

    def _update_obstacle_pose(self, name, x, y):
        """Publish new pose for an obstacle."""
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        if name in self.obstacle_pubs:
            self.obstacle_pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
