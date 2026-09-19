from setuptools import find_packages, setup

package_name = "monitor_value_lib"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["share_config/monitor_value.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML", "smbus2"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="ROS-independent sensor collection + CSV logger library.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
