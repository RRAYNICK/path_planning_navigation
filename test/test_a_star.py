#!/usr/bin/env python3
import math

import pytest
import rclpy

from path_planning_demo.a_star_planner import AStarNode, AStarPlanner


class TestAStar:
    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for tests that create nodes."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def test_astar_node_total_cost(self):
        """Verify total_cost = g(n) + h(n)."""
        node = AStarNode(0, 0, cost=5, heuristic=3)
        assert node.total_cost() == 8  # 5 + 3

    def test_astar_node_comparison(self):
        """Priority queue should use total_cost for ordering."""
        node1 = AStarNode(0, 0, cost=10, heuristic=1)  # f=11
        node2 = AStarNode(1, 1, cost=5, heuristic=5)  # f=10
        assert node2 < node1  # lower f is "less"
        assert not (node1 < node2)

    def test_astar_heuristic_manhattan(self):
        """Manhattan distance should be computed correctly."""
        planner = AStarPlanner()
        node_a = AStarNode(0, 0)
        node_b = AStarNode(3, 4)
        # Manhattan = |3-0| + |4-0| = 7
        assert planner.manhattan_distance(node_a, node_b) == 7
        planner.destroy_node()

    def test_heuristic_admissible_simple(self):
        """Manhattan distance is admissible (equals true cost on grid)."""
        planner = AStarPlanner()
        node_start = AStarNode(0, 0)
        node_goal = AStarNode(5, 5)
        h_val = planner.manhattan_distance(node_start, node_goal)
        # True minimal cost on 4-connected grid is also 10
        assert h_val == 10
        planner.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__])
