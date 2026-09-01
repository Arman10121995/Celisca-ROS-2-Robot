from setuptools import setup, find_packages
setup(
    name='robot_lab_vacuum_cleaning',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_lab_vacuum_cleaning']),
        ('share/robot_lab_vacuum_cleaning', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Lab',
    maintainer_email='robot@example.com',
    description='Vacuum cleaning controller',
    license='MIT',
    entry_points={'console_scripts': ['vacuum_cleaner = robot_lab_vacuum_cleaning.vacuum_cleaner:main']},
)
