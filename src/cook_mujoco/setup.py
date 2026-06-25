from setuptools import find_packages, setup


package_name = "cook_mujoco"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "mujoco>=3.0"],
    zip_safe=True,
    maintainer="alen",
    maintainer_email="alen@example.com",
    description="MuJoCo runtime, position controller, and local demo CLI.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cook_mujoco_demo = cook_mujoco.cli:main",
            "cook_force_control_demo = cook_mujoco.chopping_cli:main",
        ],
    },
)
