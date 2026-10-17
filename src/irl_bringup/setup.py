from setuptools import setup


package_name = "irl_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/architecture.launch.py",
            "launch/perception.launch.py",
            "launch/realsense_perception.launch.py",
            "launch/sensing.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
