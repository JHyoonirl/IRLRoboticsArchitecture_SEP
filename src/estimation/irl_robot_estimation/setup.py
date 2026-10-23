from setuptools import find_packages, setup


package_name = "irl_robot_estimation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "camera_tf = irl_robot_estimation.camera_tf:main",
            "mock_state_estimation = irl_robot_estimation.mock_state_estimation:main",
        ],
    },
)
