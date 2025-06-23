import os
from glob import glob

from setuptools import setup

package_name = "yahboomcar"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.*")),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob(os.path.join("meshes", "**", "*.*"), recursive=True),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz*")),
        ),
        (
            os.path.join("share", package_name, "param"),
            glob(os.path.join("param", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unknown",
    maintainer_email="unknown@foo",
    description="Yahboomcar ROS2 package for robot control and sensors.",
    license="Apache License 2.0",
    python_requires=">=3.7",
    tests_require=["pytest"],
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    entry_points={
        "console_scripts": [
            "Ackermann_Driver_R2 = yahboomcar.Ackermann_Driver_R2:main",
            "base_node_R2 = yahboomcar.base_node_r2:main",
            "joystick = yahboomcar.joystick_node:main",
        ],
    },
)
