from setuptools import find_packages, setup

package_name = "traffic_light_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "opencv-python", "numpy"],
    zip_safe=True,
    maintainer="ggm",
    maintainer_email="gusgarciarrealm@gmail.com",
    description="Detects traffic light colors and publishes status.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = traffic_light_detector.traffic_light_detector_node:main",
        ],
    },
)
