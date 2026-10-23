from setuptools import find_packages, setup


package_name = "irl_world_state"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/world_state.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "world_state = irl_world_state.world_state:main",
        ],
    },
)
