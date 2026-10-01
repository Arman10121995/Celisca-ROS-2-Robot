# Perception Algorithm Comparison: Obstacle Detection

**Category:** perception  
**Type:** real_experiment  
**Target Audience:** intermediate  
**Estimated Time:** 75 minutes  

## Description

Compare 5 perception algorithms for obstacle detection accuracy

## Prerequisites

- Basic ROS 2 knowledge
- robot_lab installed
- Gazebo or MuJoCo backend available


## Learning Objectives

- Understand different obstacle detection approaches
- Compare clustering algorithms (Scan vs DBSCAN vs Euclidean)
- Evaluate ground removal techniques
- Select appropriate perception method for different environments


## Commands

```bash
# Launch perception comparison experiment
ros2 run robot_lab_benchmark perception_comparison --config perception_config.yaml
# Run individual perception methods
ros2 run robot_lab_algorithms scan_clusterer
ros2 run robot_lab_algorithms dbscan_clusterer
# Visualize results
rqt --force-discover | grep rqt_reconfigure
```

## Expected Results

This tutorial produces the following artifacts:

- `docs/tutorials/perception_comparison.md`
- `docs/tutorials/perception_results.json`
- `docs/tutorials/perception_metrics.csv`
- `docs/tutorials/perception_plots/comparison.png`


## Success Criteria

- [ ] All 5 perception methods execute successfully
- [ ] Obstacle detection metrics collected for all methods
- [ ] Performance comparison tables generated
- [ ] Clear recommendations for different use cases


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
