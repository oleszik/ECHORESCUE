from glob import glob
from setuptools import find_packages, setup

package_name = "echorescue_ros"

setup(
    name=package_name,
    version="0.14.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pymavlink"],
    zip_safe=True,
    maintainer="EchoRescue contributors",
    maintainer_email="maintainers@example.invalid",
    description="ROS 2 adapters for receive-only MAVLink telemetry and continuous 3D state.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_node = echorescue_ros.mission_node:main",
            "simulator_node = echorescue_ros.simulator_node:main",
            "mavlink_telemetry_bridge = echorescue_ros.mavlink_telemetry_bridge:main",
            "telemetry_observer = echorescue_ros.telemetry_observer:main",
        ],
    },
)
