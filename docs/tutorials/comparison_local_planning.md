# Local Planning Method Comparison

Documentation reviewed October 7, 2026 against runtime checkpoint `091d388`.
Read [current status](../status/CURRENT_STATUS.md) for available workflows and
remaining qualification; evidence below retains its named source stages.

> Draft template, not a measured comparison. The 2026-10-02 audit withdrew
> the generated scores and placeholder command claims. R9.2 remains partial.


**Category:** local_planning  
**Type:** comparison_guide  
**Target Audience:** intermediate  
**Estimated Time:** 60 minutes  

## Description

Compare 5 local_planning methods on common benchmarks

## Prerequisites

- Basic understanding of local_planning
- ROS 2 and robot_lab installed
- Completed numerical demos


## Learning Objectives

- Understand local_planning algorithm differences
- Learn to compare methods fairly
- Interpret performance metrics
- Select appropriate methods for different scenarios


## Run

Only the method/experiment declarations can currently be checked with this page:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/robot_lab_algorithms python3 -m pytest -q src/robot_lab_algorithms/test/test_r7_benchmark_framework.py
```

This checks numerical fixtures and configuration contracts. It does not execute
five robot methods or produce the proposed comparison results below. Follow
[the completion audit](../status/audit-2026-10-02.md) and
[workflow](../WORKFLOW.md) to turn this draft into an exercised comparison.


## Proposed artifacts

An executed comparison must produce and validate these artifacts:

- `docs/tutorials/local_planning_comparison.md`
- `docs/tutorials/local_planning_results.json`
- `docs/tutorials/local_planning_plots/`


## Success Criteria

- [ ] All methods execute without errors
- [ ] Performance metrics collected
- [ ] Comparison tables and plots generated
- [ ] Clear recommendations documented


## Related tutorials

- [Tutorial index](README.md)
- [Numerical learning notes](local_planning.md)
- [Related comparison draft](local_planning_comparison.md)
- [Measured-run and failure workflow](../WORKFLOW.md)
- [Current done/remaining checklist](../status/CHECKLIST.md)

## Notes

- This tutorial assumes you have a working robot_lab installation
- All commands should be run from the workspace root
- For GUI tutorials, ensure you have a display available or use a tested X11 display
- Results may vary based on your hardware configuration

---

*Draft reviewed: 2026-10-07; original generated template: 2026-10-01*

*Part of R9.2: Seven Real Comparison Tutorials

*See [ROADMAP.md](../../ROADMAP.md) for task details*
