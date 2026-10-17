from setuptools import find_packages, setup


package_name = "irl_sensor_prepro"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "sensor_preprocessor = irl_sensor_prepro.sensor_preprocessor:main",
        ],
    },
)
