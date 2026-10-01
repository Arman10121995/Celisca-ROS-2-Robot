# Perception Method Comparison

**Category:** perception  
**Type:** comparison_guide  
**Target Audience:** intermediate  
**Estimated Time:** 60 minutes  

## Description

Compare 5 perception methods on common benchmarks

## Prerequisites

- Basic understanding of perception
- ROS 2 and robot_lab installed
- Completed numerical demos


## Learning Objectives

- Understand perception algorithm differences
- Learn to compare methods fairly
- Interpret performance metrics
- Select appropriate methods for different scenarios


## Commands

```bash
# Compare perception methods
ros2 launch robot_lab_bringup comparison_launch.py
# Or run individual methods: ros2 run robot_lab_algorithms perception_method1
ros2 run robot_lab_algorithms perception_method2
```

## Expected Results

This tutorial produces the following artifacts:

- `docs/tutorials/perception_comparison.md`
- `docs/tutorials/perception_results.json`
- `docs/tutorials/perception_plots/`


## Success Criteria

- [ ] All methods execute without errors
- [ ] Performance metrics collected
- [ ] Comparison tables and plots generated
- [ ] Clear recommendations documented


## Related Tutorials

- [All Tutorials Index](../README.md)
- [Comparison Guide for {tutorial.category.value} Category](./{tutorial.category.value}_comparison.md)
- [Failure Interpretation Guide](./{tutorial.category.value}_failures.md)
- [Parameter Study Guide](./{tutorial.category.value}_parameter_study.md)

## Notes

- This tutorial assumes you have a working robot_lab installation
- All commands should be run from the workspace root
- For GUI tutorials, ensure you have a display available or use X11 forwarding
- Results may vary based on your hardware configuration

---

*Last updated: 2026-10-01

*Part of R9.2: Seven Real Comparison Tutorials

*See [ROADMAP.md](../../ROADMAP.md) for task details*
