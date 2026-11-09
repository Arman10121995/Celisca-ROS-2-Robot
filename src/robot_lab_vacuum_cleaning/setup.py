from glob import glob

from setuptools import find_packages, setup

setup(
    name='robot_lab_vacuum_cleaning',
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_lab_vacuum_cleaning']),
        ('share/robot_lab_vacuum_cleaning', ['package.xml']),
        ('share/robot_lab_vacuum_cleaning/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Lab',
    maintainer_email='robot@example.com',
    description='Robot-vacuum cleaning system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['vacuum_cleaner = robot_lab_vacuum_cleaning.vacuum_cleaner:main']},
)
