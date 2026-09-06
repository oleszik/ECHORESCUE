from glob import glob
from setuptools import find_packages, setup

package_name = "echorescue_ros"

setup(
    name=package_name,
    version="0.13.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EchoRescue contributors",
    maintainer_email="maintainers@example.invalid",
    description="ROS 2 adapters for the EchoRescue closed loop.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_node = echorescue_ros.mission_node:main",
            "simulator_node = echorescue_ros.simulator_node:main",
        ],
    },
)
