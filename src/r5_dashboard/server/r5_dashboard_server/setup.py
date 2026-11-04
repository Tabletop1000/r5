import os
from glob import glob

from setuptools import find_packages, setup

package_name = "r5_dashboard_server"

# The built web app (web/dist, from 'npm run build') is installed with the package if it exists.
# colcon needs these paths relative to the package folder.
here = os.path.dirname(os.path.abspath(__file__))
web_dist = os.path.join("..", "..", "web", "dist")


def web_files():
    files = []
    for path in glob(os.path.join(web_dist, "**", "*"), recursive=True, root_dir=here):
        if os.path.isfile(os.path.join(here, path)):
            rel_dir = os.path.relpath(os.path.dirname(path), web_dist)
            dest = os.path.normpath(os.path.join("share", package_name, "web", rel_dir))
            files.append((dest, [path]))
    return files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ]
    + web_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jdutra",
    maintainer_email="jadondutra@gmail.com",
    description="R5 dashboard web server",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "server = r5_dashboard_server.main:main",
            "set_password = r5_dashboard_server.set_password:main",
            "robot_status = r5_dashboard_server.robot_status_node:main",
            "motor_test = r5_dashboard_server.motor_test_node:main",
            "field_manager = r5_dashboard_server.field_manager_node:main",
            "job_manager = r5_dashboard_server.job_manager_node:main",
            "sprayer_stub = r5_dashboard_server.sprayer_stub_node:main",
        ],
    },
)
