from setuptools import find_packages, setup

package_name = "rpi_camera_ctrl"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/rpi_camera_ctrl.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="ROS node for Raspberry Pi Camera Module 3 control via MediaMTX/momo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rpi_camera_ctrl = rpi_camera_ctrl.node:main",
            "rpi_camera_cli = rpi_camera_ctrl.camera_cli:main",
        ],
    },
)
