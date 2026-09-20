import os
from pathlib import Path

from setuptools import find_packages, setup

package_name = "monitor_value_web"
_here = Path(__file__).resolve().parent


def _share_files():
    data = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/monitor_value_web.yaml"]),
        ("share/" + package_name + "/deploy", ["deploy/rov-jazzy.service"]),
    ]
    frontend_dist = _here / "frontend" / "dist"
    if frontend_dist.is_dir():
        for root, _dirs, files in os.walk(frontend_dist):
            rel_root = os.path.relpath(root, frontend_dist)
            dest = os.path.join("share", package_name, "frontend")
            if rel_root != ".":
                dest = os.path.join(dest, rel_root)
            srcs = [os.path.relpath(os.path.join(root, f), _here) for f in files]
            if srcs:
                data.append((dest, srcs))
    return data


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "frontend"]),
    data_files=_share_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Browser dashboard for unified monitor values.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "monitor_value_web = monitor_value_web.node:main",
            "monitor_value_web_demo = monitor_value_web.demo:main",
        ],
    },
)
