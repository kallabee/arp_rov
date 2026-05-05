from setuptools import setup

package_name = "device_registry"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="I2C device YAML registry for ROS2 workspace packages.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
