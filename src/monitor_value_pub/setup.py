from pathlib import Path

from setuptools import find_packages, setup

package_name = "monitor_value_pub"
_here = Path(__file__).resolve().parent
# Paths must be relative to this package root (colcon ament_python requirement).
_launch_files = [str(p.relative_to(_here)) for p in sorted(_here.glob("launch/*.launch.py"))]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/monitor_value.yaml"]),
        ("share/" + package_name + "/launch", _launch_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Publish unified monitor values and log to CSV.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "monitor_value_pub = monitor_value_pub.node:main",
        ],
    },
)
