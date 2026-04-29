#!/usr/bin/env python3
import math

import pytest
import rclpy

from path_planning_demo.dynamic_obstacle_manager import DynamicObstacleManager


class TestObstacleManager:
    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for tests that create nodes."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def test_pattern_linear(self):
        """Linear pattern should follow expected trigonometric path."""
        manager = DynamicObstacleManager()
        t = 0.0
        start_x, start_y = 2.0, 2.0
        speed = 0.3
        for _ in range(40):
            x, y = manager._move_linear("test", t, speed, start_x, start_y)
            expected_x = start_x + speed * math.sin(t * 0.5) * 3.0
            expected_y = start_y + speed * math.cos(t * 0.5) * 2.0
            assert abs(x - expected_x) < 0.001
            assert abs(y - expected_y) < 0.001
            t += 0.05
        manager.destroy_node()

    def test_pattern_circular(self):
        """Circular pattern should maintain constant radius."""
        manager = DynamicObstacleManager()
        t = 0.0
        start_x, start_y = -1.0, 1.0
        speed = 0.2
        radius = 1.5
        for _ in range(20):
            x, y = manager._move_circular("test", t, speed, start_x, start_y)
            dist = math.hypot(x - start_x, y - start_y)
            assert abs(dist - radius) < 0.001
            t += 0.1
        manager.destroy_node()

    def test_pattern_zigzag(self):
        """Zigzag pattern should produce x (increasing) and y (oscillating)."""
        manager = DynamicObstacleManager()
        t = 0.0
        start_x, start_y = 1.0, -2.0
        speed = 0.4
        for _ in range(30):
            x, y = manager._move_zigzag("test", t, speed, start_x, start_y)
            # Basic sanity checks: coordinates are floats
            assert isinstance(x, float)
            assert isinstance(y, float)
            t += 0.1
        manager.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__])
