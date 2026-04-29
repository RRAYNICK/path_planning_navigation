#!/usr/bin/env python3
"""Performance comparison analyzer for Dijkstra vs A*."""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


class ComparisonAnalyzer(Node):
    """Analyzes and compares path planning performance."""

    def __init__(self):
        super().__init__("comparison_analyzer")
        self.results = {
            "dijkstra": {
                "times": [],
                "path_lengths": [],
                "nodes_expanded": [],
                "success_rate": [0, 0],
            },
            "a_star": {
                "times": [],
                "path_lengths": [],
                "nodes_expanded": [],
                "success_rate": [0, 0],
            },
        }

        self.dijkstra_path_sub = self.create_subscription(
            Path, "/dijkstra/path", self.dijkstra_callback, 10
        )
        self.a_star_path_sub = self.create_subscription(
            Path, "/a_star/path", self.a_star_callback, 10
        )

    def dijkstra_callback(self, msg):
        self._analyze_path("dijkstra", msg)

    def a_star_callback(self, msg):
        self._analyze_path("a_star", msg)

    def _analyze_path(self, algorithm, path):
        """Analyze a computed path for performance metrics."""
        if not path.poses:
            self.results[algorithm]["success_rate"][1] += 1
            return

        # Record success
        self.results[algorithm]["success_rate"][0] += 1
        self.results[algorithm]["success_rate"][1] += 1

        # Compute path length
        length = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            length += math.hypot(p2.x - p1.x, p2.y - p1.y)
        self.results[algorithm]["path_lengths"].append(length)

        self.get_logger().info(f"{algorithm.upper()} Path Length: {length:.2f} meters")

    def generate_report(self):
        """Generate comprehensive comparison report."""
        dijkstra_rate = (
            self.results["dijkstra"]["success_rate"][0]
            / max(self.results["dijkstra"]["success_rate"][1], 1)
            * 100
        )
        a_star_rate = (
            self.results["a_star"]["success_rate"][0]
            / max(self.results["a_star"]["success_rate"][1], 1)
            * 100
        )

        dijkstra_length = (
            np.mean(self.results["dijkstra"]["path_lengths"])
            if self.results["dijkstra"]["path_lengths"]
            else 0.0
        )
        a_star_length = (
            np.mean(self.results["a_star"]["path_lengths"])
            if self.results["a_star"]["path_lengths"]
            else 0.0
        )

        report = f"""
========================================
 PATH PLANNING ALGORITHM COMPARISON
========================================

DIJKSTRA RESULTS:
-----------------
Success Rate: {dijkstra_rate:.1f}%
Avg Path Length: {dijkstra_length:.2f} m
Total Test Cases: {self.results['dijkstra']['success_rate'][1]}

A* RESULTS:
-----------
Success Rate: {a_star_rate:.1f}%
Avg Path Length: {a_star_length:.2f} m
Total Test Cases: {self.results['a_star']['success_rate'][1]}

CONCLUSION:
-----------
"""
        return report


def main(args=None):
    rclpy.init(args=args)
    node = ComparisonAnalyzer()
    # Run for 60 seconds to collect data
    rclpy.spin_once(node, timeout_sec=60)
    print(node.generate_report())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
