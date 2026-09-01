from setuptools import setup, find_packages
setup(
    name='sim_launcher_gui',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/sim_launcher_gui']),
        ('share/sim_launcher_gui', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Lab',
    description='Simulation launcher GUI',
    license='MIT',
    entry_points={'console_scripts': ['sim_launcher_gui = sim_launcher_gui.launcher:main']},
)
