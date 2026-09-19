from setuptools import find_packages, setup

package_name = "imu_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/imu_navigation.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "monitor_value_lib"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "imu_nav_node = imu_navigation.ros_node:main",
        ],
    },
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="IMU / depth fusion and navigation helpers.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
