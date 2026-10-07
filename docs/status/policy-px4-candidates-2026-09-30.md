# Policy and PX4 candidates (2026-09-30)

> Historical source-stage record. Use [current status](CURRENT_STATUS.md),
> [the ledger](platform-status.yaml) and [the checklist](CHECKLIST.md) for the
> published October 7 work and remaining qualification. Measurements and
> original claims below retain their dates; later audited corrections apply.
> The old PX4 installation/candidate statement is superseded by measured
> [native X500 flight](../tutorials/px4_x500.md); policy mission gaps remain.

This is a source audit, not a qualification claim. A locomotion checkpoint is
only usable with the joint order, observation vector, action scaling, control
rate, actuator law and robot mass/inertia it was trained against. Each candidate
must pass a bounded standing/stop test and a velocity task on this project's
actual simulator model before its GUI mode can be enabled.

| Project profile | Primary candidate | Integration decision |
| --- | --- | --- |
| `berkeley_humanoid_lite_sim` | [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite), which ships an [ONNX biped policy configuration](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite/blob/main/configs/policy_biped_25hz_b.yaml) | Existing 22-joint controller is qualified only for limited MuJoCo walking. Its held turn and reverse remain unqualified. |
| `berkeley_humanoid_lite_biped` | The same upstream biped policy configuration | The local biped URDF has 12 actuated leg joints; the current controller hardcodes 22 joints, so it cannot be reused without a separate action/observation mapping and live test. |
| `berkeley_humanoid_lite` | Berkeley's humanoid policy | The full model has the same family of 22 joints, but the launch profile lacks the effort controller and IMU path used by `_sim`; sharing a checkpoint alone would not make it commandable. |
| `unitree_go2` | [Unitree RL Mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) Go2 velocity policy | The current project has a Go2 controller, but recovery and terrain tasks remain incomplete in the status ledger. |
| `unitree_h1_2` and other supported Unitree humanoids | [Unitree RL Mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) lists H1_2, G1 and others | Export the model's own trained checkpoint and map its model-specific observations and joints. The Go2 or Berkeley checkpoint is not interchangeable. |
| `unitree_go2w` | [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco) supplies a Go2W model; a [community Go2W policy project](https://github.com/koki67/unitree_rl_mjlab_go2w) documents simulation deployment | Go2W adds driven wheels, so the 12-leg Go2 policy cannot simply command the complete plant. Audit the community checkpoint/license and test wheel plus leg control before advertising locomotion. |
| `quadrotor_sitl` replacement | [PX4 Gazebo X500](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles), with [uXRCE-DDS ROS 2 bridge](https://docs.px4.io/main/en/middleware/uxrce_dds) | Use `gz_x500` SITL and its PX4 flight dynamics rather than treating the static quadrotor URDF as flight capable. Pin a PX4 release compatible with the host Gazebo/ROS distribution; start the agent; verify arming, takeoff, hover, offboard waypoint, landing and stop before registering modes. PX4 and Micro XRCE-DDS Agent are not installed in this workspace, so this remains a candidate. |

The GUI and CLI mode gates should continue to reflect measured tasks and sensor
contracts. In the current code, PyBullet/MuJoCo provide a raycast scan and
stable passive stance for legged localization; Gazebo descriptions without
LiDAR and Isaac's unstable passive legged stance do not meet that contract.
