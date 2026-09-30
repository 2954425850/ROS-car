import os
from glob import glob

from setuptools import find_packages, setup

package_name = "l150pro_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="l150pro",
    maintainer_email="noreply@example.com",
    description="WHEELTEC L150Pro C30D(2.0) 四驱差速底盘 ROS 2 驱动",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "l150pro_driver = l150pro_driver.driver_node:main",
            "l150pro_monitor = l150pro_driver.monitor:main",
        ],
    },
)
