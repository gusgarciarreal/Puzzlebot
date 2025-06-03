from setuptools import find_packages, setup
from glob import glob
import os

package_name = "seguidor_poligonos"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Incluir archivo package.xml
        (os.path.join("share", package_name), ["package.xml"]),
        # Explicitly include all launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gusgm",
    maintainer_email="gusgarciarrealm@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_generator = seguidor_poligonos.path_generator:main",
            "controller = seguidor_poligonos.controller:main",
        ],
    },
    # Incluir archivos extra (como __init__.py en msg/)
    include_package_data=True,
)
