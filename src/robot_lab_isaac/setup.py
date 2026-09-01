from setuptools import setup

package_name = "robot_lab_isaac"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={"": "python"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/isaac_simulator.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robot Lab",
    maintainer_email="robot@example.com",
    description="Isaac Sim simulator adapter for Robot Lab simulations.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "isaac_spawner = robot_lab_isaac.isaac_spawner:main",
            "sensor_bridge = robot_lab_isaac.sensor_bridge:main",
        ],
    },
)
