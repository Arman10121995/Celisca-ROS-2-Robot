# TIAGo tool-pose planning in the normal GUI (2026-11-20)

**Implemented, experimental.** One official PAL TIAGo (`asset_pal_tiago`)
run in MuJoCo Display on `nav_empty`, ROS 2 Humble / Jetson, base revision
`5d08e2a9` plus the changes listed in [the manifest](sha256.txt).
The normal GUI launched the profile with its autofilled command; only the
simulator viewer and RViz were disabled. A parallel Allegro screen started
around the same time, so this is a functional screen and is not timing evidence.

| Step | Measured result |
|---|---|
| Copy measured pose, jog Z by 20 mm, keep final angle | Target XYZ (0.111050, −0.927575, 0.726500) m in `base_footprint` |
| Plan pose | Four feasible IK endpoints; one waypoint selected. Head, fingers and base held. Plan start-to-ready joint change under 0.000001 m and 0.000001 rad |
| Edit X after Plan | Execute refused until the input matched the planned target again |
| Execute plan | Source servos settled; measured tool endpoint error 0.150 mm in the base frame, 0.151 mm in the frozen world frame, 0.0000154 rad final angle error |

The TCP is the original URDF's `gripper_grasping_frame`, recovered through
its rigid mount to the surviving MuJoCo body. Only torso and the seven arm
joints are planned. The existing source envelope, scene drift/collision,
settling, heartbeat and Stop guards remain in use. This is a pose goal
through joint space. It does not prove straight tool motion, object custody,
hardware accuracy, arbitrary reachability, or support on another backend.

62 focused checks pass (private planner, articulation bridge and real Tk
arm page); four selected packages build. The fixture uses actual simulated
servos and a nonzero source tool offset. The full GUI probe uses the live
simulation's measured pose without writing body positions.

To reproduce after building the updated packages:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
export ROS_DOMAIN_ID=114 CASE_OUTPUT="$ROBOT_LAB_RUNTIME_ROOT/tiago-cartesian-new"
mkdir -p "$CASE_OUTPUT"
timeout --signal=INT --kill-after=25s 135s xvfb-run -a python3 \
  docs/status/evidence/tiago-cartesian-2026-11-20/gui_trial.py
```

Raw commands, feedback and results are in the SSD reports under
`/workspace/molar/robot_lab_runtime/tiago-cartesian-20261120/run-1`.
