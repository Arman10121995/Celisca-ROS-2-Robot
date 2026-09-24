# Simulator workflows through the bringup launch (2026-09-23)

Jetson AGX Orin host (see `../r82-isaac-jetson-2026-09-16/host.json`), one run
per row, each on its own ROS domain and Gazebo partition, with no stray
processes from earlier runs (see "Orphaned simulators" below).

## Navigation: one Nav2 goal per robot, map and backend

`scripts/sim_nav_check.sh <sim> <robot> <map>` launches `mode:=nav`, picks a
free goal about 2 m from the robot out of the published `/map`, and sends it.

| | nav_obstacle bumperbot | nav_obstacle labbot | celisca_floor_1 bumperbot | celisca_floor_1 labbot |
|---|---|---|---|---|
| PyBullet | succeeded | succeeded | succeeded | succeeded |
| MuJoCo | succeeded | succeeded | succeeded | succeeded |
| Gazebo | succeeded | succeeded | succeeded | succeeded |
| Isaac Sim | succeeded | succeeded | succeeded | succeeded |

Final localization-estimate error 0.23-0.28 m (the goal tolerance) in every
row; details in `nav_goals.txt`. `nav_goals_gui_rviz_far.txt` adds: the
exact command the GUI builds (A*, DWB, EKF, fusion selections) in PyBullet;
the goal published on `/goal_pose` as RViz's 2D Goal Pose does, in PyBullet
and Gazebo; and a ~7 m goal across celisca_floor_1 in MuJoCo for both robots.
All succeeded.

## Localization for robots without wheels

`scripts/sim_loc_check.py` against `mode:=loc` on nav_obstacle compares the
`map` -> root-link transform with ground truth over 10 s
(`localization_first_pass.txt`, taken before the BHL sim spawn and biped
MuJoCo fixes; both re-run afterwards at 0.000-0.001 m).

- PyBullet and MuJoCo: every Unitree and Berkeley Humanoid Lite profile
  localizes (mean error <= 0.08 m for all but A1/Go1 in PyBullet, which slide
  ~0.8 m on the joint hold and track at 0.37-0.39 m).
- Isaac: not usable. Dogs drift up to 1.9 m, Aliengo is thrown 67 m, H1-2
  produces NaN poses: Isaac's joint hold does not keep them in place.
- None of these robots walks on /cmd_vel, so slam and nav stay unavailable.

These results are encoded in `robot_lab_utils/mode_capability.py` and
`robot_lab_robots/config/robots.yaml`; the GUI greys out the rest with the
reason.

## Orphaned simulators

Six PyBullet/MuJoCo spawners from earlier GUI navigation runs were found still
running about three hours after their launches were gone, each simulating
nav_obstacle and publishing /clock, /odom/ground_truth and /scan into the same
ROS domain. Every run bringup now starts carries a `ROBOT_LAB_RUN_ID` and a
detached reaper stops what is left when the launch exits; the spawners also
exit with their parent (`robot_lab_utils/partition_reaper.py`,
`process_lifetime.py`).

## MuJoCo with mesh maps

Map-only display, `gui:=false`, to "model loaded": celisca_floor_1_furniture
9 s, celisca_floor_2_furniture 10 s (previously never: pycollada missing in
the ROS interpreter; flex compile 84-134 s for the Celisca floors).
