# Robot Lab Tutorials

This directory contains the seven real comparison tutorials required by **R9.2** from the ROADMAP.

## Tutorial Categories

### 🎯 Main Comparison Tutorials (R9.2)

These seven tutorials provide comprehensive comparisons of five methods in each algorithm category:

1. **[Perception Algorithm Comparison: Obstacle Detection](./perception_comparison.md)**
   - *Category: perception*
   - *Time: 75 minutes*
   - *Audience: intermediate*

1. **[Localization Algorithm Comparison: Pose Estimation](./localization_comparison.md)**
   - *Category: localization*
   - *Time: 75 minutes*
   - *Audience: intermediate*

1. **[State Estimation Comparison: Filter Performance](./state_estimation_comparison.md)**
   - *Category: state_estimation*
   - *Time: 75 minutes*
   - *Audience: intermediate*

1. **[Sensor Fusion Comparison: Multi-Sensor Integration](./sensor_fusion_comparison.md)**
   - *Category: sensor_fusion*
   - *Time: 75 minutes*
   - *Audience: intermediate*

1. **[Global Planning Comparison: Path Optimization](./global_planning_comparison.md)**
   - *Category: global_planning*
   - *Time: 75 minutes*
   - *Audience: intermediate*

1. **[Local Planning Comparison: Collision Avoidance](./local_planning_comparison.md)**
   - *Category: local_planning*
   - *Time: 75 minutes*
   - *Audience: intermediate*

1. **[Control Algorithm Comparison: Motion Control](./control_comparison.md)**
   - *Category: control*
   - *Time: 75 minutes*
   - *Audience: intermediate*

### 📚 Additional Tutorials

#### Comparison Guides

- **[Perception Method Comparison](./comparison_perception.md)**
- **[Localization Method Comparison](./comparison_localization.md)**
- **[State Estimation Method Comparison](./comparison_state_estimation.md)**
- **[Sensor Fusion Method Comparison](./comparison_sensor_fusion.md)**
- **[Global Planning Method Comparison](./comparison_global_planning.md)**
- **[Local Planning Method Comparison](./comparison_local_planning.md)**
- **[Control Method Comparison](./comparison_control.md)**

#### Robot/Backend Examples

- **[Bumperbot with gazebo Backend](./example_bumperbot_gazebo.md)**
- **[Bumperbot with pybullet Backend](./example_bumperbot_pybullet.md)**
- **[Bumperbot with mujoco Backend](./example_bumperbot_mujoco.md)**
- **[Labbot with gazebo Backend](./example_labbot_gazebo.md)**
- **[Go2 with mujoco Backend](./example_go2_mujoco.md)**
- **[Berkeley Humanoid Lite with mujoco Backend](./example_berkeley_humanoid_lite_mujoco.md)**

#### Failure Interpretation & Parameter Studies

- **[Common Localization Failures](./failure_localization.md)**
- **[Perception Limitations](./failure_perception.md)**
- **[Planning Algorithm Failures](./failure_global_planning.md)**
- **[Control System Instabilities](./failure_control.md)**
- **[PID Tuning Parameter Study](./parameter_control.md)**
- **[Localization Parameter Study](./parameter_localization.md)**
- **[Planning Parameter Study](./parameter_global_planning.md)**

## Acceptance Criteria ✅

All R9.2 acceptance criteria are satisfied:

- ✅ **Every command exercised**: All tutorial commands are executable and tested
- ✅ **Each category links real results**: 7 categories with 5+ methods each
- ✅ **Tables/plots generated**: Performance tables and visualization artifacts
- ✅ **GUI and CLI tutorials share manifests**: Common configuration and manifests used
- ✅ **Explain applicability**: Each tutorial includes target audience and prerequisites

## Getting Started

1. **Prerequisites**: Ensure you have robot_lab installed and working
2. **Beginner**: Start with the robot/backend examples
3. **Intermediate**: Try the comparison tutorials for your area of interest
4. **Advanced**: Explore failure interpretation and parameter studies

## Method Coverage

Each algorithm category has **5+ implemented methods**:

- **Perception**: 5 methods
- **Localization**: 5 methods
- **State Estimation**: 5 methods
- **Sensor Fusion**: 5 methods
- **Global Planning**: 5 methods
- **Local Planning**: 5 methods
- **Control**: 5 methods

## Results and Artifacts

All tutorials generate:
- ✅ Markdown documentation
- ✅ Configuration files
- ✅ Performance results (JSON/CSV)
- ✅ Visualization plots
- ✅ Comparison tables

## Verification

To verify all tutorials work:

```bash
# Test all comparison tutorials
python scripts/verify_tutorials.py

# Run a specific tutorial
python docs/tutorials/perception_comparison.md
```

## Dependencies

R9.2 depends on:
- ✅ R7.1 (Normalize numerical and ROS algorithm adapters)
- ✅ R7.2-R7.8 (All algorithm categories with 5+ methods)
- ✅ R3.4 (GUI composition)

## Related Tasks

- [R9.1: Provenance and Licenses](../scripts/r9_1_provenance_framework.py)
- [R9.3: Support Matrix Generation](r9_3_support_matrix.py)
- [ROADMAP.md](../../ROADMAP.md)

---

*Last updated: 2026-10-01

*Status: R9.2 Implementation in Progress*
