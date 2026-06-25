from setuptools import find_packages, setup


package_name = "cook_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="alen",
    maintainer_email="alen@example.com",
    description="Core data interfaces, planning interfaces, and trajectory execution.",
    license="Apache-2.0",
)
