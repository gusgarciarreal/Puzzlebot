from setuptools import find_packages, setup
import os
from glob import glob

package_name = "puzzlebot_control_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Add this line to include all launch files from the 'launch' directory
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ggm",
    maintainer_email="gusgarciarrealm@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speed_moderator = puzzlebot_control_utils.speed_moderator:main",
        ],
    },
)
