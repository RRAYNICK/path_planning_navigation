from setuptools import find_packages, setup

package_name = "path_planning_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/path_planning_demo.launch.py", "launch/analysis.launch.py"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/robot_params.yaml", "config/obstacles_params.yaml"],
        ),
        ("share/" + package_name + "/worlds", ["worlds/dynamic_obstacle_world.sdf"]),
        ("share/" + package_name + "/rviz", ["rviz/path_planning.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@umgc.edu",
    description="Path planning algorithm comparison (Dijkstra vs A*) in ROS2 Jazzy with Gazebo Harmonic",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dijkstra_planner = path_planning_demo.dijkstra_planner:main",
            "a_star_planner = path_planning_demo.a_star_planner:main",
            "comparison_analyzer = path_planning_demo.comparison_analyzer:main",
            "dynamic_obstacle_manager = path_planning_demo.dynamic_obstacle_manager:main",
        ],
    },
)
