# Third-Party Notices

This repository includes external assets from the following sources. Each
retains its original license. See the referenced license files for full terms.

## Robot/Model Assets

| Asset | Source | License | Location |
|-------|--------|---------|----------|
| Unitree robot descriptions (Go2, B2, H1, G1, etc.) | [Awesome-URDFs](https://github.com/Awesome-URDFs) / Unitree Robotics | BSD 3-Clause | `src/robot_lab_robots/_upstream/Awesome-URDFs/` |
| Berkeley Humanoid Lite | [Berkeley-Humanoid-Lite-Assets](https://github.com/) | CC BY-SA 4.0 | `src/robot_lab_robots/_upstream/Berkeley-Humanoid-Lite-Assets/` |
| Berkeley Humanoid Lite (dev) | [Berkeley-Humanoid-Lite](https://github.com/) | See asset license | `src/robot_lab_robots/_upstream/Berkeley-Humanoid-Lite/` |
| Legacy humanoid import | Internal conversion | See URDF metadata | `src/robot_lab_robots/_upstream/legacy_humanoid_import/` |

## Software Dependencies

| Package | License | Usage |
|---------|---------|-------|
| ROS 2 Humble | Apache 2.0 | Core framework |
| rclpy | Apache 2.0 | Python client library |
| nav_msgs, sensor_msgs | Apache 2.0 | Message definitions |
| matplotlib | PSF-based | Benchmark plotting |
| numpy | BSD 3-Clause | Numerical operations |
| xacro | BSD 3-Clause | URDF processing |
| PyYAML | YAML parsing | Registry configuration |

## Original Licenses

- `src/robot_lab_robots/_upstream/Awesome-URDFs/LICENSE` — BSD 3-Clause (Unitree Robotics)
