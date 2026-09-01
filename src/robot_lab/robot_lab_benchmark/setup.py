from setuptools import setup, find_packages

package_name = 'robot_lab_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mistral Vibe',
    maintainer_email='vibe@mistral.ai',
    description='Benchmark runner and standard result schema for Robot Lab experiments.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot-lab-benchmark = robot_lab_benchmark.cli:main',
        ],
    },
)
