import os
from glob import glob

from setuptools import find_packages, setup

package_name = "paxini_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workspace maintainer",
    maintainer_email="maintainer@example.invalid",
    description="ROS 2 driver for PaXini PX6D 6-axis force/torque sensors.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "driver = paxini_driver.node:main",
            "paxini_driver = paxini_driver.node:main",
            "calibrate = paxini_driver.calibrator:main",
        ],
    },
)
