from setuptools import setup, find_packages

package_name = 'robot_lab_registry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robots.yaml', 'config/environments.yaml', 'config/scenarios.yaml', 'config/experiments.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mistral Vibe',
    maintainer_email='vibe@mistral.ai',
    description='Robot Lab Registry: canonical catalogs, schemas, and validation CLI for the unified multi-robot simulation and benchmarking platform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot-lab = robot_lab_registry.cli:main',
        ],
    },
)
