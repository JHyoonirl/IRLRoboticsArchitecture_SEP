from setuptools import find_packages, setup


package_name = "irl_object_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/object_perception.launch.py"]),
        (f"share/{package_name}/config", ["config/perception.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "object_perception = irl_perception.perception_node:main",
        ],
    },
)
