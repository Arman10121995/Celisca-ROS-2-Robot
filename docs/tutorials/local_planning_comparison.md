# Local Planning Comparison: Collision Avoidance

> Draft template, not a measured comparison. The 2026-10-02 audit withdrew
> the generated scores and placeholder command claims. R9.2 remains partial.


**Category:** local_planning  
**Type:** draft_template
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


## Run

Only the method/experiment declarations can currently be checked with this page:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/robot_lab_algorithms python3 -m pytest -q src/robot_lab_algorithms/test/test_r7_benchmark_framework.py
```

This checks numerical fixtures and configuration contracts. It does not execute
five robot methods or produce the proposed comparison results below. Follow
[the completion audit](../status/audit-2026-10-02.md) and
[workflow](../WORKFLOW.md) to turn this draft into an exercised comparison.


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
