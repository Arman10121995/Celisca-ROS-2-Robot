# Local Planning Comparison: Collision Avoidance

**Category:** local_planning  
**Type:** real_experiment  
**Target Audience:** intermediate  
**Estimated Time:** 75 minutes  

## Description

Compare 5 local planning algorithms for obstacle avoidance

## Prerequisites

- Global planner available
- Obstacle environment prepared
- Robot controller configured


## Learning Objectives

- Understand different local planning approaches
- Compare DWB vs Pure Pursuit vs TEB vs MPPI
- Evaluate obstacle avoidance behavior
- Analyze computational complexity


## Commands

```bash
# Launch local planning comparison
ros2 run robot_lab_benchmark local_planning_comparison --config local_planning_config.yaml
# Test different local planners
ros2 run robot_lab_algorithms dwb_local_planner
ros2 run robot_lab_algorithms teb_local_planner
# Analyze avoidance performance
python scripts/analyze_avoidance.py --input local_planning_results.json
```

## Expected Results

This tutorial produces the following artifacts:

- `docs/tutorials/local_planning_comparison.md`
- `docs/tutorials/local_planning_results.json`
- `docs/tutorials/avoidance_visualization.png`


## Success Criteria

- [ ] All 5 local planners execute
- [ ] Collision avoidance metrics collected
- [ ] Computation time vs safety analyzed
- [ ] Planner recommendations for different robot types


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
