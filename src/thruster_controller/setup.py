from setuptools import setup

package_name = "thruster_controller"
submodules = "thruster_rot_conv"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    # py_modules=['thruster_controller.thruster_rot_conv'],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "listener = thruster_controller.subscriber_member_function:main",
        ],
    },
)
