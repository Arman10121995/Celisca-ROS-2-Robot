from setuptools import setup, find_packages
setup(
    name='robot_lab_gui',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_lab_gui']),
        ('share/robot_lab_gui', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Lab',
    description='Simulation launcher GUI',
    license='MIT',
    entry_points={'console_scripts': ['robot_lab_gui = robot_lab_gui.launcher:main']},
)