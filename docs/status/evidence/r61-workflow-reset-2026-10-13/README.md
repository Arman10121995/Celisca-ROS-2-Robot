# R6.1: workflow and reset screens for Labbot and Bumperbot in every backend (2026-10-13)

## What each cell runs

Each cell runs `scripts/sim_workflow_check.sh <sim> <robot> nav_obstacle slam`.
It launches slam mode and drives with the GUI's `/key_vel` path.

**Drive.** Forward, reverse, turn and reverse turn. Each phase is measured
against truth.

**Publisher loss.** The command publisher stops and the robot must stop.

**SLAM map.** The check confirms that a live map is being built.

**Reset.** The simulator's `/robot_lab/reset` is called. The cell then checks
that:

- the robot is back at spawn and stopped;
- localization re-converges;
- the robot drives again and the estimator tracks the motion;
- mapping resumes;
- the simulated clock never runs backwards.

All 17 checks are listed in each JSON (`checks`). The cells ran at
`b8e8ba5` or earlier on the same day, one simulator at a time.

## Results

| Cell | Passed | Localization error before / after reset drive (m) | Map updates after reset | Reset response |
|---|---|---|---|---|
| MuJoCo Labbot | yes | 0.0061 / 0.0016 | 4 | Simulation reset successfully |
| MuJoCo Bumperbot | yes | 0.0101 / 0.0003 | 5 | Simulation reset successfully |
| PyBullet Labbot | yes | 0.0097 / 0.0011 | 7 | Simulation reset successfully |
| PyBullet Bumperbot | yes | 0.0003 / 0.0002 | 13 | Simulation reset successfully |
| Gazebo Labbot | yes | 0.0033 / 0.0099 | 4 | Gazebo robot pose/control reset; world clock preserved |
| Gazebo Bumperbot | yes | 0.0077 / 0.0029 | 4 | Gazebo robot pose/control reset; world clock preserved |
| Isaac Labbot | yes | 0.0016 / 0.0128 | 7 | Isaac physical reset acknowledged |
| Isaac Bumperbot | yes | 0.0007 / 0.0019 | 7 | Isaac physical reset acknowledged |

**All 8 cells pass all 17 checks.** No cell recorded a backward clock jump.

## Limits

These are screens of the workflow and reset path for the two lab
differential drives on one map. They are not long-duration missions.
Dynamic actors, sensor resets and the other robot classes are separate
R6 work.
