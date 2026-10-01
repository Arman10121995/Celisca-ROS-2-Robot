# State Estimation Comparison: Filter Performance

**Category:** state_estimation  
**Type:** real_experiment  
**Target Audience:** intermediate  
**Estimated Time:** 75 minutes  

## Description

Compare 5 state estimation algorithms for tracking accuracy

## Prerequisites

- Understanding of state estimation concepts
- IMU and motion data available
- Ground truth reference available


## Learning Objectives

- Understand different estimation approaches
- Compare EKF vs UKF vs Particle Filter
- Evaluate error covariance and consistency
- Analyze computational complexity


## Commands

```bash
# Launch state estimation comparison
ros2 run robot_lab_benchmark state_estimation_comparison --config est_config.yaml
# Test different estimators
ros2 run robot_lab_algorithms ekf_3d_estimator
ros2 run robot_lab_algorithms ukf_estimator
# Compare with ground truth
python scripts/compare_estimation.py --truth ground_truth.csv --estimated estimated.csv
```

## Expected Results

This tutorial produces the following artifacts:

- `docs/tutorials/state_estimation_comparison.md`
- `docs/tutorials/state_estimation_results.json`
- `docs/tutorials/estimation_error_plots.png`


## Success Criteria

- [ ] All 5 estimators run successfully
- [ ] Error metrics collected vs ground truth
- [ ] Covariance consistency analyzed
- [ ] Performance vs computation trade-offs documented


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
