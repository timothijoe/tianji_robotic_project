from pathlib import Path

from setuptools import find_packages, setup


package_name = "cook_bringup"


def files_under(root: str) -> list[str]:
    return [
        str(path)
        for path in sorted(Path(root).rglob("*"))
        if path.is_file()
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", files_under("launch")),
        (f"share/{package_name}/rviz", files_under("rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="alen",
    maintainer_email="alen@example.com",
    description="ROS2 adapters, nodes, launch files, and RViz configuration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cook_mujoco_controller_node = cook_bringup.ros.mujoco_controller_node:main",
            "cook_demo_trajectory_publisher = cook_bringup.ros.demo_trajectory_publisher:main",
            "cook_teach_pendant = cook_bringup.ros.teach_pendant_node:main",
        ],
    },
)
