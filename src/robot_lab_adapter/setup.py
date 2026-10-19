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
        ('share/' + package_name + '/policies/go2_velocity_flat', [
            'policies/go2_velocity_flat/policy.onnx',
            'policies/go2_velocity_flat/policy.onnx.data',
            'policies/go2_velocity_flat/deploy.yaml',
            'policies/go2_velocity_flat/SOURCE.md',
        ]),
        ('share/' + package_name + '/policies/rl_sar', [
            'policies/rl_sar/LICENSE',
            'policies/rl_sar/SOURCE.md',
        ]),
    ] + [
        ('share/' + package_name + '/policies/rl_sar/' + name,
         ['policies/rl_sar/%s/%s' % (name, item)
          for item in ('base.yaml', 'config.yaml', 'policy.onnx', 'source.json')])
        for name in ('a1_legged_gym', 'b2_robot_lab', 'go2w_robot_lab', 'go2_himloco',
                     'g1_robomimic_locomotion')
    ] + [
        ('share/' + package_name + '/policies/go2_recovery_nju', [
            'policies/go2_recovery_nju/policy.onnx',
            'policies/go2_recovery_nju/SOURCE.md',
            'policies/go2_recovery_nju/LICENSE',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Lab Team',
    maintainer_email='robot-lab@example.com',
    description='Unified bringup package for Robot Lab platform',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot-lab-select = robot_lab_adapter.selectors:main',
            'joint-effort-commander = robot_lab_adapter.joint_effort_commander:main',
            'humanoid-standing-controller = robot_lab_adapter.humanoid_standing_controller:main',
            'humanoid-policy-controller = robot_lab_adapter.humanoid_policy_controller:main',
            'go2-stance-gait-controller = robot_lab_adapter.go2_stance_gait_controller:main',
            'mavros-offboard-controller = robot_lab_adapter.mavros_offboard_controller:main',
            'px4-ros-controller = robot_lab_adapter.px4_ros_controller:main',
            'legged-policy-controller = robot_lab_adapter.legged_policy_controller:main',
            'legged-train = robot_lab_adapter.legged_train:main',
            'fleet-controller = robot_lab_utils.fleet_controller:main',
        ],
    },
)
