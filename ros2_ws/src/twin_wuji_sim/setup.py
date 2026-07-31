from glob import glob
from setuptools import find_packages, setup


package_name = "twin_wuji_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tianji Robotics",
    maintainer_email="dev@example.com",
    description="ROS 2 bridge for the MuJoCo Wuji left hand.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "bridge = twin_wuji_sim.bridge_node:main",
            "demo = twin_wuji_sim.demo_node:main",
        ],
    },
)
