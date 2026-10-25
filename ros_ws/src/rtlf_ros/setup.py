from glob import glob
from setuptools import setup

package_name = "rtlf_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rtlf",
    maintainer_email="user@example.com",
    description="GPS-denied return-to-last-fix experiment",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_node = rtlf_ros.mission_node:main",
            "slam_bridge_node = rtlf_ros.slam_bridge_node:main",
        ],
    },
)
