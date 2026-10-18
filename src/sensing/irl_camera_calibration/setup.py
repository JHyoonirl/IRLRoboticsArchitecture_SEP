from glob import glob
from setuptools import find_packages, setup


package_name = "irl_camera_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "charuco_extrinsic_calibration = "
            "irl_camera_calibration.calibration_node:main",
            "charuco_axes_viewer = "
            "irl_camera_calibration.charuco_axes_viewer:main",
        ],
    },
)
