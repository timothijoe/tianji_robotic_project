from pathlib import Path

from setuptools import find_packages, setup


package_name = "cook_description"


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
        (f"share/{package_name}/assets/robot/urdf", files_under("assets/robot/urdf")),
        (f"share/{package_name}/assets/robot/meshes", files_under("assets/robot/meshes")),
        (f"share/{package_name}/assets/robot/mujoco", files_under("assets/robot/mujoco")),
    ],
    install_requires=["setuptools", "numpy", "mujoco>=3.0"],
    zip_safe=True,
    maintainer="alen",
    maintainer_email="alen@example.com",
    description="Robot assets, URDF metadata, and MuJoCo model conversion utilities.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cook_model_convert = cook_description.tools.convert_model:main",
        ],
    },
)
