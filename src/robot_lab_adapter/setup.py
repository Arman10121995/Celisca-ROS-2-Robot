from setuptools import setup

package_name = 'robot_lab_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/select_robot.launch.py', 'launch/select_components.launch.py']),
        ('share/' + package_name + '/config', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Lab Team',
    maintainer_email='robot-lab@example.com',
    description='Unified bringup package for Robot Lab platform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot-lab-select = robot_lab_adapter.selectors:main',
            'joint-effort-commander = robot_lab_adapter.joint_effort_commander:main',
            'humanoid-standing-controller = robot_lab_adapter.humanoid_standing_controller:main',
            'mavros-offboard-controller = robot_lab_adapter.mavros_offboard_controller:main',
        ],
    },
)
