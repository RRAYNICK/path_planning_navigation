#!/usr/bin/env python3
import pytest
import rclpy

from path_planning_demo.dijkstra_planner import DijkstraPlanner, GraphNode


class TestDijkstra:
    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for tests that create nodes."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def test_graph_node_comparison(self):
        """Test that GraphNode comparison uses cost only."""
        node1 = GraphNode(0, 0, cost=5)
        node2 = GraphNode(1, 1, cost=3)
        # Dijkstra: lower cost is "less"
        assert (node1 < node2) is False  # 5 is not < 3
        assert (node2 < node1) is True  # 3 < 5

    def test_graph_node_equality(self):
        """Test that equality is based on coordinates, not cost."""
        node1 = GraphNode(0, 0, cost=10)
        node2 = GraphNode(0, 0, cost=20)
        node3 = GraphNode(1, 0, cost=10)
        assert node1 == node2  # same coordinates
        assert node1 != node3  # different coordinates

    def test_graph_node_addition(self):
        """Test addition operator for neighbor generation."""
        node = GraphNode(2, 3)
        result = node + (1, -1)
        assert result.x == 3
        assert result.y == 2

    def test_graph_node_hashing(self):
        """Test that hashing uses coordinates (for set operations)."""
        node1 = GraphNode(0, 0)
        node2 = GraphNode(0, 0)
        node_set = {node1, node2}
        assert len(node_set) == 1  # same coordinates → same hash

    def test_is_valid_in_bounds_exists(self):
        """Verify that the is_valid method exists in DijkstraPlanner."""
        planner = DijkstraPlanner()
        assert hasattr(planner, "is_valid")
        planner.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__])
