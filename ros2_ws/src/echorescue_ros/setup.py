from glob import glob
from setuptools import find_packages, setup

package_name = "echorescue_ros"

setup(
    name=package_name,
    version="0.14.5",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pymavlink"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="EchoRescue contributors",
    maintainer_email="maintainers@example.invalid",
    description="ROS 2 telemetry adapters and simulation-only closed-loop MAVLink milestone.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_node = echorescue_ros.mission_node:main",
            "simulator_node = echorescue_ros.simulator_node:main",
            "mavlink_telemetry_bridge = echorescue_ros.mavlink_telemetry_bridge:main",
            "mavlink_flight_mission = echorescue_ros.mavlink_flight_mission:main",
            "flight_mission_observer = echorescue_ros.flight_mission_observer:main",
            "mavlink_waypoint_mission = echorescue_ros.mavlink_waypoint_mission:main",
            "waypoint_mission_observer = echorescue_ros.waypoint_mission_observer:main",
            "gazebo_range_sensor_bridge = echorescue_ros.gazebo_range_sensor_bridge:main",
            "mavlink_sensor_replanning_mission = echorescue_ros.mavlink_sensor_replanning_mission:main",
            "sensor_replanning_observer = echorescue_ros.sensor_replanning_observer:main",
            "telemetry_observer = echorescue_ros.telemetry_observer:main",
        ],
    },
)
